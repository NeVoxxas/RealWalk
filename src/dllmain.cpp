/*-----------------------------------------*/
// Project	: ETS2MobileCam
// File		: dllmain.cpp
/*-----------------------------------------*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <vector>
#include <array>

#include "bmem.h"
#include "globals.h"
#include "PCInput.h"
#include "scs_logging.h"
#include "Prism3D/Types/camera_manager.h"
using namespace scs_logging;

#include "Prism3D/Actors/camera_manager.h"
#include "Net/CameraInput.h"
#include "Net/PhoneServer.h"
#include "Utils/orientation_math.h"
#include "Utils/PromptOverlay.h"

#include "Physx/PxQuery.h"
#include "Physx/Coupling.h"

#include "scs_sdk/common/scssdk_telemetry_truck_common_channels.h"

namespace
{
	uintptr_t freecam_tick_function_address{};

	struct truck_placement_t
	{
		bool valid{};
		double x{}, y{}, z{};
		float heading{}; // 0..1, 0 = šiaurė, didėja prieš laikrodžio rodyklę
	};
	truck_placement_t truck_placement{};

	float truck_speed_ms = 0.0f;
	bool truck_speed_valid = false;
	bool parking_brake_on = false;
	bool trailer_hooked_up = false;   // spėjimas: ar mes jau paleidome prikabinimo seką
	constexpr bool REQUIRE_HANDBRAKE = true;

	SCSAPI_VOID parking_brake_callback(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t)
	{
		parking_brake_on = value && value->type == SCS_VALUE_TYPE_bool && value->value_bool.value != 0;
	}

	SCSAPI_VOID truck_speed_callback(const scs_string_t name, const scs_u32_t index, const scs_value_t* const value, const scs_context_t context)
	{
		if (!value || value->type != SCS_VALUE_TYPE_float)
		{
			truck_speed_valid = false;
			return;
		}
		truck_speed_ms = value->value_float.value;
		truck_speed_valid = true;
	}

	truck_placement_t trailer_placement{};
	bool trailer_connected = false;

	SCSAPI_VOID trailer_connected_callback(const scs_string_t name, const scs_u32_t index, const scs_value_t* const value, const scs_context_t context)
	{
		trailer_connected = value && value->type == SCS_VALUE_TYPE_bool && value->value_bool.value != 0;
	}

	constexpr float MAX_EXIT_SPEED_MS = 0.5f;   // max greitis, kai leidžiama išlipti
	constexpr float MAX_WALK_RADIUS_M = 150.0f;

	SCSAPI_VOID truck_placement_callback(const scs_string_t name, const scs_u32_t index, const scs_value_t* const value, const scs_context_t context)
	{
		auto* out = static_cast<truck_placement_t*>(context);
		if (!value || value->type != SCS_VALUE_TYPE_dplacement)
		{
			out->valid = false;
			return;
		}
		out->x = value->value_dplacement.position.x;
		out->y = value->value_dplacement.position.y;
		out->z = value->value_dplacement.position.z;
		out->heading = value->value_dplacement.orientation.heading;
		out->valid = true;
	}

	// Stovėjimo aukštis išlipus ir sąveikos nustatymai
	constexpr float EXIT_UP_M = 1.7f;           // akių aukštis virš vilkiko pozicijos taško, kai stovi lauke
	constexpr int INTERACT_KEY = 'E';
	constexpr float ENTER_RADIUS_M = 2.5f;

	// Kelias pro duris
	constexpr float EXIT_OUT_M = 1.6f;          // kiek metrų į šoną nuo sėdynės atsistoji lauke
	constexpr float DOOR_SIDE_OVERRIDE = 0.0f;  // -1 = kairė, +1 = dešinė, 0 = automatiškai pagal sėdynės pusę
	constexpr float DOOR_SIDE_FALLBACK = -1.0f; // jei sėdynės pusės nustatyti nepavyksta

	// Animacijos
	constexpr float EXIT_ANIM_SECONDS = 1.9f;
	constexpr float ENTER_ANIM_SECONDS = 1.8f;

	// Šuolis / pritūpimas
	constexpr int JUMP_KEY = VK_SPACE;
	constexpr int CROUCH_KEY = VK_LCONTROL;
	constexpr int SPRINT_KEY = VK_LSHIFT;
	constexpr WORD HITCH_GAME_KEY = 'T';          // žaidimo Attach/Detach trailer klavišas
	constexpr float HITCH_RADIUS_M = 3.0f;        // kiek arti sujungimo vietos turi stovėti
	constexpr float HITCH_KEY_HOLD_S = 0.12f;     // kiek laikome klavišą nuspaustą
	constexpr bool GROUND_MODE = true;          // false = senas laisvas skraidymas (Space/Ctrl)
	constexpr float GRAVITY_MS2 = 9.81f;
	constexpr float JUMP_HEIGHT_M = 0.55f;
	constexpr float CROUCH_DEPTH_M = 0.65f;     // kiek nusileidžia akys pritūpus
	constexpr float CROUCH_SPEED_FACTOR = 0.45f;
	constexpr float SPRINT_SPEED_FACTOR = 1.7f;   // kiek kartų greičiau
	constexpr float SPRINT_TAU = 0.15f;           // kaip greitai įsibėgėja / sustoja
	constexpr float CROUCH_DOWN_TAU = 0.07f;
	constexpr float CROUCH_UP_TAU = 0.10f;
	constexpr float AIR_TAU_MULTIPLIER = 4.0f;  // ore sunkiau keisti judėjimo kryptį
	constexpr float LANDING_KICK = 0.5f;        // nusileidimo "kumštelėjimo" stiprumas
	constexpr float LAND_SPRING_K = 180.0f;
	constexpr float LAND_SPRING_C = 19.0f;
	constexpr float GROUND_FOLLOW_TAU = 0.1f;

	// Žemės aukštis iš PhysX raycast (ref_y = dabartinis akių aukštis); jei nepavyksta – vilkiko aukštis
	static double ground_y_at(float x, float z, double ref_y)
	{
		static double last_ground = 0.0;
		static bool   have_last = false;

		if (px::offset_valid())
		{
			px::Hit h;
			const px::Vec3 o = px::to_phys(x, ref_y + 0.3, z);
			if (px::raycast(o, { 0, -1, 0 }, 3.0f, h))
			{
				last_ground = (double)h.pos.y + px::g_off[1];
				have_last = true;
				return last_ground;
			}

			// --- diagnostika: trumpas spindulys nepataikė, bandome ilgesnį iš aukščiau ---
			static int diag = 0;
			if ((++diag % 30) == 0)
			{
				px::Hit h2;
				const px::Vec3 o2 = px::to_phys(x, ref_y + 3.0, z);
				if (px::raycast(o2, { 0, -1, 0 }, 15.0f, h2))
					scs_log(0, "long ray HIT y=%.2f (ref %.2f, last %.2f)", h2.pos.y + px::g_off[1], ref_y, last_ground);
				else
					scs_log(0, "long ray: no hit (ref %.2f)", ref_y);
			}
		}
		if (have_last) return last_ground;
		return truck_placement.y;
	}

	void send_game_key(WORD vk, bool down)
	{
		INPUT in{};
		in.type = INPUT_KEYBOARD;
		in.ki.wScan = static_cast<WORD>(MapVirtualKey(vk, MAPVK_VK_TO_VSC));
		in.ki.dwFlags = KEYEVENTF_SCANCODE | (down ? 0 : KEYEVENTF_KEYUP);
		SendInput(1, &in, sizeof(INPUT));
	}


	constexpr float PATH_TURN_END = 0.4f;       // iki šio animacijos momento (0..1) kamera pasisuka į duris
	constexpr float PATH_START_DELAY = 0.15f;   // kamera pradeda slinkti į šoną tik po šios dalies
	constexpr float PATH_DUCK_START_M = 0.35f;  // nuo kurio atstumo nuo sėdynės pradeda lenktis po staktomis
	constexpr float PATH_DOOR_M = 0.9f;         // atstumas nuo sėdynės iki durų slenksčio
	constexpr float PATH_DUCK_M = 0.12f;        // pasilenkimo gylis
	constexpr int PATH_STEP_COUNT = 3;          // laipteliai
	constexpr float PATH_LOOK_DOWN_RAD = 0.15f; // žvilgsnis žemyn lipant laiptais
	constexpr float ANIM_RESIDUAL_FRACTION = 0.4f; // per kiek laiko įlipant išnyksta pradinis nuokrypis

	// Atsarginė vairuotojo akių vieta (jei nepavyksta nuskaityti iš kameros), vilkiko lokalioje erdvėje
	constexpr float SEAT_RIGHT_M = -0.5f;
	constexpr float SEAT_FORWARD_M = 0.0f;
	constexpr float SEAT_UP_M = 2.0f;

	enum class walk_transition_t { none, exiting, entering };

	struct truck_axes_t { float fwd_x, fwd_z, right_x, right_z; };

	truck_axes_t truck_axes()
	{
		const float h = truck_placement.heading * 2.0f * PI;
		return { -std::sin(h), -std::cos(h), std::cos(h), -std::sin(h) };
	}

	// Vairuotojo galvos vieta iš žaidimo konfigūracijos, vilkiko erdvėje (x dešinėn, y aukštyn, z atgal)
	struct cab_config_t
	{
		bool have_cabin = false;
		bool have_head = false;
		float cabin[3] = { 0.0f, 0.0f, 0.0f };
		float head[3] = { 0.0f, 0.0f, 0.0f };
	};
	cab_config_t cab_config{};

	struct vehicle_dims_t
	{
		bool have_wheels = false;
		float wheel_f_min = 0.0f, wheel_f_max = 0.0f; // ratų išilginės padėtys (pirmyn teigiama)
		bool have_hook = false;
		float hook_f = 0.0f;
		std::vector<std::array<double, 2>> wheel_xz;   // (x, z) vilkiko lokalioje erdvėje
	};
	vehicle_dims_t truck_dims{}, trailer_dims{};

	bool near_hitch(float px_, float pz_)
	{
		if (!truck_placement.valid || !truck_dims.have_wheels) return false;
		const truck_axes_t a = truck_axes();
		const float f = truck_dims.wheel_f_min;       // pirmyn teigiama, tad rašome jos reikšmę tiesiogiai
		const float hx = static_cast<float>(truck_placement.x) + a.fwd_x * f;
		const float hz = static_cast<float>(truck_placement.z) + a.fwd_z * f;
		const float dx = px_ - hx, dz = pz_ - hz;
		return dx * dx + dz * dz <= HITCH_RADIUS_M * HITCH_RADIUS_M;
	}

	SCSAPI_VOID telemetry_configuration(const scs_event_t event, const void* const event_info, scs_context_t context)
	{
		const auto* info = static_cast<const scs_telemetry_configuration_t*>(event_info);
		if (!info || !info->id) return;

		const bool is_truck = std::strcmp(info->id, "truck") == 0;
		const bool is_trailer = std::strcmp(info->id, "trailer") == 0 || std::strcmp(info->id, "trailer.0") == 0;
		if (!is_truck && !is_trailer) return;

		cab_config_t cfg{};
		vehicle_dims_t dims{};
		for (const scs_named_value_t* a = info->attributes; a && a->name; ++a)
		{
			if (a->value.type != SCS_VALUE_TYPE_fvector) continue;
			const scs_value_fvector_t& v = a->value.value_fvector;

			if (is_truck && std::strcmp(a->name, "cabin.position") == 0)
			{
				cfg.cabin[0] = v.x; cfg.cabin[1] = v.y; cfg.cabin[2] = v.z;
				cfg.have_cabin = true;
			}
			else if (is_truck && std::strcmp(a->name, "head.position") == 0)
			{
				cfg.head[0] = v.x; cfg.head[1] = v.y; cfg.head[2] = v.z;
				cfg.have_head = true;
			}
			else if (std::strcmp(a->name, "wheel.position") == 0)
			{
				if (a->index == 0) dims.wheel_xz.clear();
				if (a->index != SCS_U32_NIL) {
					if (a->index >= dims.wheel_xz.size()) dims.wheel_xz.resize(a->index + 1);
					dims.wheel_xz[a->index] = { (double)v.x, (double)v.z };
				}
				const float f = -v.z; // vilkiko erdvėje pirmyn yra -Z
				if (!dims.have_wheels)
				{
					dims.wheel_f_min = dims.wheel_f_max = f;
					dims.have_wheels = true;
				}
				else
				{
					dims.wheel_f_min = std::min(dims.wheel_f_min, f);
					dims.wheel_f_max = std::max(dims.wheel_f_max, f);
				}
			}
			else if (is_trailer && std::strcmp(a->name, "hook.position") == 0)
			{
				dims.hook_f = -v.z;
				dims.have_hook = true;
			}
		}

		if (is_truck)
		{
			cab_config = cfg;
			truck_dims = dims;
			scs_log(0, "Cab config: cabin(%d) %.2f %.2f %.2f | head(%d) %.2f %.2f %.2f",
				static_cast<int>(cfg.have_cabin), cfg.cabin[0], cfg.cabin[1], cfg.cabin[2],
				static_cast<int>(cfg.have_head), cfg.head[0], cfg.head[1], cfg.head[2]);
			scs_log(0, "Truck dims: wheels(%d) f %.2f..%.2f", static_cast<int>(dims.have_wheels), dims.wheel_f_min, dims.wheel_f_max);
		}
		else
		{
			trailer_dims = dims;
			scs_log(0, "Trailer dims: wheels(%d) f %.2f..%.2f | hook(%d) f %.2f",
				static_cast<int>(dims.have_wheels), dims.wheel_f_min, dims.wheel_f_max, static_cast<int>(dims.have_hook), dims.hook_f);
		}
	}

	// Akių vieta vilkiko lokalioje erdvėje: f = pirmyn, r = dešinėn, u = aukštyn
	bool config_eye(float& f, float& r, float& u)
	{
		if (!cab_config.have_head) return false;

		float x = cab_config.head[0], y = cab_config.head[1], z = cab_config.head[2];
		if (cab_config.have_cabin)
		{
			x += cab_config.cabin[0];
			y += cab_config.cabin[1];
			z += cab_config.cabin[2];
		}
		r = x;
		u = y;
		f = -z; // vilkiko erdvėje pirmyn yra -Z
		return true;
	}

	// Vairuotojo akių vieta vilkiko lokalioje erdvėje, įsimenama išlipant iš kabinos
	struct cab_eye_t
	{
		float f = SEAT_FORWARD_M;
		float r = SEAT_RIGHT_M;
		float u = SEAT_UP_M;
		bool measured = false;
	};
	cab_eye_t cab_eye{};

	struct path_sample_t
	{
		prism::float3_t pos;
		float yaw;
		float pitch;
	};

	float smoothstep01(float t)
	{
		t = std::clamp(t, 0.0f, 1.0f);
		return t * t * (3.0f - 2.0f * t);
	}

	// 0..1 profilis su n atskirais "laipteliais"
	float stair01(float t, int n)
	{
		t = std::clamp(t, 0.0f, 1.0f);
		const float q = t * static_cast<float>(n);
		const float i = std::min(std::floor(q), static_cast<float>(n - 1));
		return (i + smoothstep01(q - i)) / static_cast<float>(n);
	}

	float door_side()
	{
		if (DOOR_SIDE_OVERRIDE != 0.0f) return DOOR_SIDE_OVERRIDE;
		if (cab_eye.measured && std::fabs(cab_eye.r) > 0.2f) return cab_eye.r < 0.0f ? -1.0f : 1.0f;
		return DOOR_SIDE_FALLBACK;
	}

	prism::float3_t cab_eye_world()
	{
		const truck_axes_t a = truck_axes();
		return {
			static_cast<float>(truck_placement.x) + a.fwd_x * cab_eye.f + a.right_x * cab_eye.r,
			static_cast<float>(truck_placement.y) + cab_eye.u,
			static_cast<float>(truck_placement.z) + a.fwd_z * cab_eye.f + a.right_z * cab_eye.r
		};
	}

	// s: 0 = vairuotojo sėdynė, 1 = stovi lauke. inward = true, kai einame į vilkiką (žiūrime į kabiną)
	path_sample_t cab_path(float s, bool inward)
	{
		s = std::clamp(s, 0.0f, 1.0f);

		const truck_axes_t a = truck_axes();
		const float side = door_side();
		const float side_x = a.right_x * side;
		const float side_z = a.right_z * side;

		const prism::float3_t eye = cab_eye_world();
		const float ground_y = static_cast<float>(truck_placement.y) + EXIT_UP_M;

		// Poslinkis į šoną nuo sėdynės; aukštis ir žvilgsnis priklauso nuo to, kur kamera jau yra
		const float lateral = EXIT_OUT_M * smoothstep01((s - PATH_START_DELAY) / (1.0f - PATH_START_DELAY));
		float y = eye.y;
		float pitch = 0.0f;

		if (lateral > PATH_DUCK_START_M && lateral < PATH_DOOR_M)
		{
			// Durų anga: pasilenkia po staktomis
			const float k = (lateral - PATH_DUCK_START_M) / (PATH_DOOR_M - PATH_DUCK_START_M);
			y = eye.y - PATH_DUCK_M * std::sin(PI * k);
		}
		else if (lateral >= PATH_DOOR_M)
		{
			// Laipteliai žemyn
			const float k = (lateral - PATH_DOOR_M) / (EXIT_OUT_M - PATH_DOOR_M);
			y = eye.y + (ground_y - eye.y) * stair01(k, PATH_STEP_COUNT);
			pitch = -PATH_LOOK_DOWN_RAD * std::sin(PI * k);
		}

		// Pasisukimas į duris (yaw = atan2(-dx, -dz), ta pati konvencija kaip pc_yaw)
		const float yaw_forward = truck_placement.heading * 2.0f * PI;
		const float yaw_out = std::atan2(-side_x, -side_z);
		const float yaw_door = inward ? yaw_out + PI : yaw_out;
		const float turn = smoothstep01(s / PATH_TURN_END);
		const float yaw = yaw_forward + std::remainder(yaw_door - yaw_forward, 2.0f * PI) * turn;

		return path_sample_t{ { eye.x + side_x * lateral, y, eye.z + side_z * lateral }, yaw, pitch };
	}

	// Taškas lauke, kur atsistoji išlipęs (horizontalioje plokštumoje, y = vilkiko y)
	prism::float3_t door_position()
	{
		const path_sample_t end = cab_path(1.0f, false);
		return { end.pos.x, static_cast<float>(truck_placement.y), end.pos.z };
	}

	// Ar kelias iš durų į stovėjimo tašką užblokuotas (tvora, mašina ir pan.)
	bool exit_blocked()
	{
		if (!px::offset_valid() || !truck_placement.valid) return false;

		const prism::float3_t p0 = cab_path(0.85f, false).pos;
		const prism::float3_t p1 = cab_path(1.0f, false).pos;
		const float dx = p1.x - p0.x, dz = p1.z - p0.z;
		const float seg = std::sqrt(dx * dx + dz * dz);
		if (seg < 0.01f) return false;

		const px::Vec3 d{ dx / seg, 0.f, dz / seg };
		const px::Vec3 o = px::to_phys(p0.x, truck_placement.y + 0.9, p0.z);
		px::Hit h;
		return px::sweep_sphere(o, d, seg + 0.25f, 0.30f, h);   // +0.25 m – užtikrina vietą ir už taško
	}

	bool game_has_focus()
	{
		const HWND fg = GetForegroundWindow();
		if (!fg) return false;
		DWORD pid = 0;
		GetWindowThreadProcessId(fg, &pid);
		return pid == GetCurrentProcessId();
	}

	// Susidūrimai su savo vilkiku ir priekaba (apytikslės dėžės, derinamos bandymais)
	constexpr bool COLLISION_ENABLED = true;
	constexpr float PLAYER_RADIUS_M = 0.35f;
	constexpr float TRUCK_HALF_WIDTH_M = 1.3f;
	constexpr float TRUCK_FRONT_OVERHANG_M = 1.35f;   // nuo priekinės ašies iki bamperio
	constexpr float TRUCK_REAR_OVERHANG_M = 0.9f;     // už paskutinės ašies
	constexpr float TRUCK_BOX_TOP_M = 3.9f;
	constexpr float TRAILER_HALF_WIDTH_M = 1.3f;
	constexpr float TRAILER_FRONT_OVERHANG_M = 1.7f;  // nuo kablio (kingpin) į priekį
	constexpr float TRAILER_REAR_OVERHANG_M = 1.5f;   // už paskutinės ašies
	constexpr float TRAILER_BOX_TOP_M = 4.2f;

	truck_axes_t vehicle_axes(float heading01)
	{
		const float h = heading01 * 2.0f * PI;
		return { -std::sin(h), -std::cos(h), std::cos(h), -std::sin(h) };
	}

	struct box_t
	{
		bool active = false;
		double px = 0.0, py = 0.0, pz = 0.0;
		float heading = 0.0f;
		float f_min = 0.0f, f_max = 0.0f, r_min = 0.0f, r_max = 0.0f;
		float top = 0.0f;
	};

	box_t make_truck_box()
	{
		box_t b;
		if (!truck_placement.valid || !truck_dims.have_wheels) return b;
		b.active = true;
		b.px = truck_placement.x; b.py = truck_placement.y; b.pz = truck_placement.z;
		b.heading = truck_placement.heading;
		b.f_max = truck_dims.wheel_f_max + TRUCK_FRONT_OVERHANG_M;
		b.f_min = truck_dims.wheel_f_min - TRUCK_REAR_OVERHANG_M;
		b.r_min = -TRUCK_HALF_WIDTH_M;
		b.r_max = TRUCK_HALF_WIDTH_M;
		b.top = TRUCK_BOX_TOP_M;
		return b;
	}

	box_t make_trailer_box()
	{
		box_t b;
		if (!trailer_connected || !trailer_placement.valid || !trailer_dims.have_wheels || !trailer_dims.have_hook) return b;
		b.active = true;
		b.px = trailer_placement.x; b.py = trailer_placement.y; b.pz = trailer_placement.z;
		b.heading = trailer_placement.heading;
		b.f_max = trailer_dims.hook_f + TRAILER_FRONT_OVERHANG_M;
		b.f_min = trailer_dims.wheel_f_min - TRAILER_REAR_OVERHANG_M;
		b.r_min = -TRAILER_HALF_WIDTH_M;
		b.r_max = TRAILER_HALF_WIDTH_M;
		b.top = TRAILER_BOX_TOP_M;
		return b;
	}

	// Išstumia tašką (x,z) iš dėžės; grąžina true ir sienos normalę (pasaulio erdvėje), jei buvo išstumta
	static bool push_out_of_box(const box_t& b, float y, float& x, float& z, float& out_nx, float& out_nz)
	{
		if (!b.active) return false;
		if (y < static_cast<float>(b.py) - 0.5f || y > static_cast<float>(b.py) + b.top + 0.3f) return false;
		const truck_axes_t a = vehicle_axes(b.heading);
		const float dx = x - static_cast<float>(b.px);
		const float dz = z - static_cast<float>(b.pz);
		const float f = dx * a.fwd_x + dz * a.fwd_z;
		const float r = dx * a.right_x + dz * a.right_z;
		const float R = PLAYER_RADIUS_M;
		const float f_hi = b.f_max + R, f_lo = b.f_min - R, r_hi = b.r_max + R, r_lo = b.r_min - R;
		if (f <= f_lo || f >= f_hi || r <= r_lo || r >= r_hi) return false;

		const float pen_f_hi = f_hi - f, pen_f_lo = f - f_lo, pen_r_hi = r_hi - r, pen_r_lo = r - r_lo;
		float best = pen_f_hi, nf = 1.0f, nr = 0.0f;
		if (pen_f_lo < best) { best = pen_f_lo; nf = -1.0f; nr = 0.0f; }
		if (pen_r_hi < best) { best = pen_r_hi; nf = 0.0f; nr = 1.0f; }
		if (pen_r_lo < best) { best = pen_r_lo; nf = 0.0f; nr = -1.0f; }

		out_nx = a.fwd_x * nf + a.right_x * nr;
		out_nz = a.fwd_z * nf + a.right_z * nr;
		x += out_nx * best;
		z += out_nz * best;
		return true;
	}

	// x,z – pasaulio koordinatės; dx,dz – norimas poslinkis šį kadrą. Grąžina true, jei atsitrenkta.
	static bool move_collide(double& x, double& z, double y_feet, double dx, double dz, float& out_nx, float& out_nz)
	{
		const float R = 0.30f;
		bool hit_any = false;
		double len = std::hypot(dx, dz);
		for (int i = 0; i < 3 && len > 1e-5; ++i)
		{
			const px::Vec3 d{ (float)(dx / len), 0.f, (float)(dz / len) };
			const px::Vec3 o = px::to_phys(x, y_feet + 0.9, z); // krūtinės aukštis
			px::Hit h;
			if (!px::sweep_sphere(o, d, (float)len + 0.02f, R, h))
			{
				x += dx; z += dz;
				return hit_any;
			}
			hit_any = true;
			// Atstumas iki smūgio: sferos centras smūgio momentu = kontaktas + normalė * R
			const double cx = (double)h.pos.x + (double)h.normal.x * R;
			const double cz = (double)h.pos.z + (double)h.normal.z * R;
			const double travelled = (cx - (double)o.x) * d.x + (cz - (double)o.z) * d.z;
			const double allowed = std::clamp(travelled - 0.02, 0.0, len);
			x += d.x * allowed; z += d.z * allowed;

			double nx = h.normal.x, nz = h.normal.z;
			const double nl = std::hypot(nx, nz);
			if (nl < 1e-4) break;
			nx /= nl; nz /= nl;
			out_nx = (float)nx; out_nz = (float)nz;

			const double rem = len - allowed;
			const double rx = d.x * rem, rz = d.z * rem;
			const double dot = rx * nx + rz * nz;
			dx = rx - dot * nx; dz = rz - dot * nz;
			len = std::hypot(dx, dz);
		}
		return hit_any;
	}

	SCSAPI_VOID telemetry_tick(const scs_event_t event, const void* const event_info, scs_context_t context)
	{
		static auto last_tick = std::chrono::steady_clock::now();
		const auto now = std::chrono::steady_clock::now();
		const float dt = std::min(std::chrono::duration<float>(now - last_tick).count(), 0.1f);
		last_tick = now;

		// Base FOV captured on enable so the slider scales relative to it
		static bool control_was_enabled = false;
		static float base_h_fov = 0.0f;
		static float base_v_fov = 0.0f;

		static prism::quat_t smoothed_rotation{ 1.0f, 0.0f, 0.0f, 0.0f };
		static float smoothed_fov = 1.0f;
		static float motion_estimate_deg = 0.0f;

		// Movement
		static float vel_x = 0.0f;
		static float vel_z = 0.0f;
		static float vel_y = 0.0f;

		static float step_phase = 0.0f;
		static float idle_phase = 0.0f;
		static prism::float3_t walk_pos{ 0.0f, 0.0f, 0.0f };

		// Camera to restore on disable
		static uint32_t previous_camera_index = 0;
		static bool was_applying_control = false;
		static bool control_was_pc = false;
		static bool freecam_tick_patched = false;
		static bool spawn_pending = false;

		// Walk state

		static float jump_offset = 0.0f;   // kiek metrų virš žemės
		static float jump_vel = 0.0f;
		static float crouch_amount = 0.0f; // 0..1
		static float land_off = 0.0f;      // nusileidimo "spyruoklė"
		static float land_vel = 0.0f;
		static float walk_y_mod = 0.0f;    // paskutinis bendras vertikalus poslinkis

		// Transition state for entering/exiting the truck

		static walk_transition_t transition = walk_transition_t::none;
		static float transition_t = 0.0f;
		static prism::float3_t anim_offset{ 0.0f, 0.0f, 0.0f };
		static float anim_yaw_offset = 0.0f;
		static float anim_pitch_offset = 0.0f;
		static bool exit_anim_pending = false;

		// PC walk mode (F9, see PCInput.h) - yaw-then-pitch mouse-look
		static float pc_yaw = 0.0f;
		static float pc_pitch = 0.0f;
		static float pc_tilt = 0.0f;
		static float pc_fov = 1.0f;

		if (prism::camera_manager_u* camera_manager = prism::camera_manager_u::get())
		{
			camera_input_state_t phone = CameraInput::get()->get_state();

			bool pc_toggled = PCInput::get()->poll_toggle();
			bool pc_active = PCInput::get()->active();

			// F9 ignoruojamas, jei vilkikas juda
			if (pc_toggled && pc_active && truck_speed_valid && std::fabs(truck_speed_ms) > MAX_EXIT_SPEED_MS)
			{
				PCInput::get()->deactivate();
				pc_active = false;
				scs_log(1, "Cannot exit: truck is moving (%.1f m/s)", truck_speed_ms);
			}

			// ---- [E]: išlipti / įlipti + užrašas ----
			if (!pc_active) transition = walk_transition_t::none;

			// Įlipimo animacijai pasibaigus baigiame walk režimą, kamera grįžta į kabiną
			if (pc_active && transition == walk_transition_t::entering && transition_t >= 1.0f)
			{
				PCInput::get()->deactivate();
				pc_active = false;
				transition = walk_transition_t::none;
				scs_log(0, "Entered truck");
			}

			if (!trailer_connected) trailer_hooked_up = false;

			// Kol žaidėjas vaikšto lauke, kabelių ir kojų veiksmai atidedami; išeinant iš walk režimo jie įvykdomi
			const bool defer_now = pc_active && transition == walk_transition_t::none;
			coupling::g_defer = defer_now;
			if (!defer_now) coupling::flush();

			PromptOverlay::Prompt prompt = PromptOverlay::Prompt::none;
			if (truck_placement.valid && transition == walk_transition_t::none)
			{
				if (pc_active)
				{
					const prism::float3_t door = door_position();
					const float dx = walk_pos.x - door.x;
					const float dz = walk_pos.z - door.z;
					if (std::sqrt(dx * dx + dz * dz) <= ENTER_RADIUS_M)
						prompt = PromptOverlay::Prompt::enter_truck;
				}
				else if (camera_manager->m_current_camera != 0
					&& truck_speed_valid && std::fabs(truck_speed_ms) <= MAX_EXIT_SPEED_MS
					&& (!REQUIRE_HANDBRAKE || parking_brake_on)
					&& !exit_blocked())
				{
					prompt = PromptOverlay::Prompt::exit_truck;
				}
				if (prompt == PromptOverlay::Prompt::none && near_hitch(walk_pos.x, walk_pos.z))
				{
					if (coupling::has_pending_vis())
						prompt = PromptOverlay::Prompt::cables_disconnect;                 // 1. atjungti kabelius
					else if (coupling::has_pending_legs())
						prompt = coupling::pending_legs_raise() ? PromptOverlay::Prompt::legs_raise
						: PromptOverlay::Prompt::legs_lower;   // 2. kojos
					else if (coupling::has_pending_cables())
						prompt = coupling::pending_cables_connect() ? PromptOverlay::Prompt::cables_connect
						: PromptOverlay::Prompt::trailer_detach; // 3. atkabinti priekaba
					else if (trailer_connected)
						prompt = trailer_hooked_up ? PromptOverlay::Prompt::disconnect_trailer : PromptOverlay::Prompt::connect_trailer;
				}
			}
			PromptOverlay::get()->set_prompt(prompt);

			static bool interact_was_down = false;
			const bool interact_down = game_has_focus() && (GetAsyncKeyState(INTERACT_KEY) & 0x8000) != 0;
			const bool interact_pressed = interact_down && !interact_was_down;
			interact_was_down = interact_down;

			if (interact_pressed && prompt == PromptOverlay::Prompt::exit_truck)
			{
				// Tas pats kelias kaip F9, tik iškviestas iš kodo
				PCInput::get()->activate();
				pc_toggled = true;
				pc_active = true;
			}
			else if (interact_pressed && prompt == PromptOverlay::Prompt::enter_truck)
			{
				// Kelias prasideda nuo stovėjimo taško lauke; pradinis nuokrypis (įskaitant šuolį/pritūpimą) sklandžiai išnyksta
				const path_sample_t start = cab_path(1.0f, true);
				anim_offset = { walk_pos.x - start.pos.x, (walk_pos.y + walk_y_mod) - start.pos.y, walk_pos.z - start.pos.z };
				anim_yaw_offset = std::remainder(pc_yaw - start.yaw, 2.0f * PI);
				anim_pitch_offset = pc_pitch - start.pitch;
				jump_offset = jump_vel = crouch_amount = land_off = land_vel = 0.0f;
				transition = walk_transition_t::entering;
				transition_t = 0.0f;
			}

			static float hitch_release_t = -1.0f;
			const bool at_hitch = pc_active && transition == walk_transition_t::none
				&& (prompt == PromptOverlay::Prompt::connect_trailer || prompt == PromptOverlay::Prompt::disconnect_trailer
					|| prompt == PromptOverlay::Prompt::cables_connect || prompt == PromptOverlay::Prompt::cables_disconnect
					|| prompt == PromptOverlay::Prompt::legs_raise || prompt == PromptOverlay::Prompt::legs_lower
					|| prompt == PromptOverlay::Prompt::trailer_detach);

			if (at_hitch && interact_pressed)
			{
				if (prompt == PromptOverlay::Prompt::cables_disconnect)
					coupling::run_vis();
				else if (prompt == PromptOverlay::Prompt::legs_raise || prompt == PromptOverlay::Prompt::legs_lower)
					coupling::run_legs();
				else if (prompt == PromptOverlay::Prompt::cables_connect || prompt == PromptOverlay::Prompt::trailer_detach)
					coupling::run_cables();
				else if (hitch_release_t < 0.0f)
				{
					send_game_key(HITCH_GAME_KEY, true);
					hitch_release_t = HITCH_KEY_HOLD_S;
					trailer_hooked_up = !trailer_hooked_up;
				}
			}
			if (hitch_release_t >= 0.0f)
			{
				hitch_release_t -= dt;
				if (hitch_release_t < 0.0f) send_game_key(HITCH_GAME_KEY, false);
			}

			// Enabling one control source disables the other
			static bool phone_control_was_enabled = false;
			const bool phone_control_enabling_now = phone.control_enabled && !phone_control_was_enabled;
			phone_control_was_enabled = phone.control_enabled;

			if (phone_control_enabling_now && pc_active)
			{
				PCInput::get()->deactivate();
				pc_active = false;
			}
			else if (pc_toggled && pc_active)
			{
				CameraInput::get()->set_control_enabled(false);
				phone.control_enabled = false;
			}

			if (pc_toggled && pc_active && camera_manager->m_cameras.size >= 1)
			{
				exit_anim_pending = false;
				transition = walk_transition_t::none;

				if (truck_placement.valid)
				{
					pc_yaw = truck_placement.heading * 2.0f * PI;
					pc_pitch = 0.0f;
					spawn_pending = true;

					// Vairuotojo akių vieta: pirmiausia iš konfigūracijos
					cab_eye = cab_eye_t{};
					float cf = 0.0f, cr = 0.0f, cu = 0.0f;
					const bool have_cfg = config_eye(cf, cr, cu);
					if (have_cfg)
					{
						cab_eye.f = cf;
						cab_eye.r = cr;
						cab_eye.u = cu;
						cab_eye.measured = true;
					}

					const uint32_t cur = camera_manager->m_current_camera;
					if (cur != 0 && cur < camera_manager->m_cameras.size)
					{
						const truck_axes_t ax = truck_axes();
						const prism::float3_t eye = camera_manager->m_cameras[cur]->m_placement.position;
						const float ex = eye.x - static_cast<float>(truck_placement.x);
						const float ez = eye.z - static_cast<float>(truck_placement.z);
						const float ey = eye.y - static_cast<float>(truck_placement.y);
						const float kf = ex * ax.fwd_x + ez * ax.fwd_z;
						const float kr = ex * ax.right_x + ez * ax.right_z;
						const bool have_cam = std::sqrt(ex * ex + ez * ez) <= 3.5f && std::fabs(ey) <= 4.0f;

						// Kameros rodmenis naudojame tik jei jis sutampa su konfigūracija (arba konfigūracijos nėra)
						if (have_cam)
						{
							const float df = kf - cf, dr = kr - cr, du = ey - cu;
							const bool agrees = !have_cfg || std::sqrt(df * df + dr * dr + du * du) <= 0.8f;
							if (agrees)
							{
								cab_eye.f = kf;
								cab_eye.r = kr;
								cab_eye.u = ey;
								cab_eye.measured = true;
							}
						}

						scs_log(0, "Cab eye: f=%.2f r=%.2f u=%.2f (cfg=%d cam=%d) | cfg f=%.2f r=%.2f u=%.2f | cam offset %.2f %.2f %.2f",
							cab_eye.f, cab_eye.r, cab_eye.u, static_cast<int>(have_cfg), static_cast<int>(have_cam),
							cf, cr, cu, ex, ey, ez);
						exit_anim_pending = true;
					}
					{
						const box_t tb = make_truck_box();
						const box_t trb = make_trailer_box();
						scs_log(0, "Collision boxes: truck %s f[%.2f..%.2f] r[%.2f..%.2f] | trailer %s f[%.2f..%.2f] r[%.2f..%.2f]",
							tb.active ? "on" : "off", tb.f_min, tb.f_max, tb.r_min, tb.r_max,
							trb.active ? "on" : "off", trb.f_min, trb.f_max, trb.r_min, trb.r_max);
					}
				}
				else
				{
					const prism::float3_t fwd = phone_math::rotate_vector(camera_manager->m_cameras[0]->m_placement.rotation, { 0.0f, 0.0f, -1.0f });
					pc_yaw = std::atan2(-fwd.x, -fwd.z);
					pc_pitch = std::asin(std::clamp(fwd.y, -1.0f, 1.0f));
				}
			}

			if (pc_active)
			{
				phone.control_enabled = true;

				float mouse_dx = 0.0f, mouse_dy = 0.0f;
				PCInput::get()->poll_mouse_delta(mouse_dx, mouse_dy);

				float target_tilt = 0.0f;
				if (!pc_toggled && transition == walk_transition_t::none)
				{
					// Skip the toggle-on frame's delta - the cursor just got warped to center
					constexpr float pitch_limit = PC_PITCH_LIMIT_DEG * PI / 180.0f;
					const float yaw_delta = -mouse_dx * PC_MOUSE_SENSITIVITY;
					pc_yaw += yaw_delta;
					pc_pitch = std::clamp(pc_pitch + -mouse_dy * PC_MOUSE_SENSITIVITY, -pitch_limit, pitch_limit);

					// Handheld lean into turns, proportional to turn rate, faded down while standing still
					constexpr float tilt_limit = PC_TILT_MAX_DEG * PI / 180.0f;
					const float yaw_rate = yaw_delta / std::max(dt, 0.001f);
					const float walk_speed_fraction = std::min(1.0f, std::sqrt(vel_x * vel_x + vel_z * vel_z) / std::max(phone.move_speed, 0.01f));
					const float tilt_gate = PC_TILT_STATIONARY_FRACTION + (1.0f - PC_TILT_STATIONARY_FRACTION) * walk_speed_fraction;
					target_tilt = std::clamp(-yaw_rate * PC_TILT_STRENGTH * tilt_gate, -tilt_limit, tilt_limit);
				}
				const float tilt_smoothing = 1.0f - std::exp(-dt / PC_TILT_TAU);
				pc_tilt += (target_tilt - pc_tilt) * tilt_smoothing;

				// Same yaw/pitch/roll axis convention as the trim system below; tilt composed as a roll
				phone.rotation = phone_math::quat_multiply(
					phone_math::quat_multiply(
						phone_math::quat_from_axis_angle(0.0, 1.0, 0.0, static_cast<double>(pc_yaw)),
						phone_math::quat_from_axis_angle(1.0, 0.0, 0.0, static_cast<double>(pc_pitch))),
					phone_math::quat_from_axis_angle(0.0, 0.0, 1.0, static_cast<double>(pc_tilt)));
				phone.joystick = PCInput::get()->poll_movement();

				pc_fov = PCInput::get()->poll_middle_click() ? 1.0f : std::clamp(pc_fov - PCInput::get()->poll_wheel_delta() * PC_ZOOM_STEP, PC_ZOOM_MIN, PC_ZOOM_MAX);
				phone.fov = pc_fov;
			}

			const bool on_free_camera = camera_manager->m_current_camera == 0 && camera_manager->m_cameras.size >= 1;


			// Something else moved us off the free camera without going through our disable path
			bool external_camera_change = false;
			if (was_applying_control && !on_free_camera && phone.control_enabled)
			{
				phone.control_enabled = false;
				CameraInput::get()->set_control_enabled(false);
				if (pc_active) PCInput::get()->deactivate();
				external_camera_change = true;
			}

			// m_target_camera is only a request the engine applies next tick, so these edges can be a frame ahead of the camera actually landing on the free camera
			const bool control_enabling_now = !control_was_enabled && phone.control_enabled;
			const bool control_disabling_now = control_was_enabled && !phone.control_enabled;

			if (control_enabling_now)
			{
				// Store whatever camera we were on so disabling can put it back
				previous_camera_index = camera_manager->m_current_camera;
				camera_manager->m_target_camera = 0;
				control_was_pc = pc_active;

				scs_log(0, control_was_pc ? "PC walk control enabled" : "Phone control enabled");
			}
			else if (control_disabling_now)
			{
				spawn_pending = false;
				exit_anim_pending = false;
				// Don't fight an external camera change by switching back to what we remembered
				if (!external_camera_change)
					camera_manager->m_target_camera = previous_camera_index;

				const char* source = control_was_pc ? "PC walk control" : "Phone control";
				if (external_camera_change)
					scs_log(0, "%s disabled (camera changed externally)", source);
				else
					scs_log(0, "%s disabled", source);
			}
			control_was_enabled = phone.control_enabled;

			CameraInput::get()->set_free_camera_active(on_free_camera);

			// Block the free cam tick function while we are controlling it
			const bool should_patch_freecam = on_free_camera && phone.control_enabled;
			if (should_patch_freecam && !freecam_tick_patched)
			{
				DWORD old;
				VirtualProtect(reinterpret_cast<void*>(freecam_tick_function_address), 2, PAGE_EXECUTE_READWRITE, &old);
				*reinterpret_cast<uint8_t*>(freecam_tick_function_address + 0) = 0xC3;
				VirtualProtect(reinterpret_cast<void*>(freecam_tick_function_address), 2, old, &old);
				freecam_tick_patched = true;
			}
			else if (!should_patch_freecam && freecam_tick_patched)
			{
				DWORD old;
				VirtualProtect(reinterpret_cast<void*>(freecam_tick_function_address), 2, PAGE_EXECUTE_READWRITE, &old);
				*reinterpret_cast<uint8_t*>(freecam_tick_function_address + 0) = 0x40;
				VirtualProtect(reinterpret_cast<void*>(freecam_tick_function_address), 2, old, &old);
				freecam_tick_patched = false;
			}

			if (on_free_camera)
			{
				prism::core_camera_u* freecam = camera_manager->m_cameras[0];

				// Camera can take a tick to actually land on the free camera after control turns on
				const bool applying_now = phone.control_enabled && !was_applying_control;
				if (applying_now)
				{
					base_h_fov = freecam->m_horizontal_fov;
					base_v_fov = freecam->m_vertical_fov;
					smoothed_rotation = phone.rotation;

					if (phone.fov > 0.0f)
						smoothed_fov = phone.fov;

					vel_x = 0.f;
					vel_y = 0.f;
					vel_z = 0.f;

					motion_estimate_deg = 0.f;
					step_phase = 0.f;
					idle_phase = 0.f;
					pc_tilt = 0.f;
					jump_offset = jump_vel = crouch_amount = land_off = land_vel = walk_y_mod = 0.0f;
					walk_pos = freecam->m_placement.position;
					scs_log(0, "Freecam pos: %.2f %.2f %.2f | Truck pos: %.2f %.2f %.2f",
						walk_pos.x, walk_pos.y, walk_pos.z, truck_placement.x, truck_placement.y, truck_placement.z);

					if (spawn_pending && truck_placement.valid)
					{
						const prism::float3_t door = door_position();
						const prism::float3_t spawn{ door.x, static_cast<float>(truck_placement.y) + EXIT_UP_M, door.z };

						if (exit_anim_pending)
						{
							const path_sample_t start = cab_path(0.0f, false);
							walk_pos = start.pos;
							anim_offset = { 0.0f, 0.0f, 0.0f };
							anim_yaw_offset = 0.0f;
							anim_pitch_offset = 0.0f;
							transition = walk_transition_t::exiting;
							transition_t = 0.0f;
							scs_log(0, "Exit animation (%s eye) f=%.2f r=%.2f u=%.2f side=%.0f",
								cab_eye.measured ? "measured" : "fallback", cab_eye.f, cab_eye.r, cab_eye.u, door_side());
						}
						else
						{
							walk_pos = spawn;
						}
					}
					spawn_pending = false;
					exit_anim_pending = false;
				}

				if (phone.control_enabled)
				{
					if (pc_active)
					{
						// Mouse input is already direct, no need for the phone gyro stabilizer
						smoothed_rotation = phone.rotation;
					}
					else
					{
						const float rotation_dot_raw = smoothed_rotation.w * phone.rotation.w
							+ smoothed_rotation.x * phone.rotation.x
							+ smoothed_rotation.y * phone.rotation.y
							+ smoothed_rotation.z * phone.rotation.z;
						const float rotation_dot = std::min(1.0f, std::abs(rotation_dot_raw));
						const float rotation_angle_deg = 2.0f * std::acos(rotation_dot) * (180.0f / PI);

						constexpr float motion_estimate_tau = 0.08f;
						const float motion_smoothing = 1.0f - std::exp(-dt / motion_estimate_tau);
						motion_estimate_deg += (rotation_angle_deg - motion_estimate_deg) * motion_smoothing;

						const float rotation_t = std::min(1.0f, motion_estimate_deg / STABILIZER_ANGLE_THRESHOLD_DEG);
						const float stabilizer_tau = phone.stabilizer_tau_max + (STABILIZER_TAU_MIN - phone.stabilizer_tau_max) * rotation_t;

						const float smoothing = 1.0f - std::exp(-dt / stabilizer_tau);
						smoothed_rotation = phone_math::quat_slerp(smoothed_rotation, phone.rotation, smoothing);
					}

					// Extra 90 degree roll
					if (phone.portrait_mode) {
						const prism::quat_t portrait_roll = phone_math::quat_from_axis_angle(0.0, 0.0, 1.0, PI / 2.0);
						freecam->m_placement.rotation = phone_math::quat_multiply(smoothed_rotation, portrait_roll);
					}
					else {
						freecam->m_placement.rotation = smoothed_rotation;
					}

					// Horizontal is relative to look direction, vertical is always world up/down
					prism::float3_t forward = phone_math::rotate_vector(phone.rotation, { 1.0f, 0.0f, 0.0f });
					prism::float3_t right = phone_math::rotate_vector(phone.rotation, { 0.0f, 0.0f, 1.0f });
					forward.y = 0.0f;
					right.y = 0.0f;
					forward = phone_math::normalize(forward);
					right = phone_math::normalize(right);

					float target_vel_x = 0.0f;
					float target_vel_z = 0.0f;
					if (phone.joystick.x != 0.0f || phone.joystick.y != 0.0f)
					{
						const float forward_amount = phone.joystick.x;
						const float right_amount = -phone.joystick.y;

						target_vel_x = (forward.x * forward_amount + right.x * right_amount) * phone.move_speed;
						target_vel_z = (forward.z * forward_amount + right.z * right_amount) * phone.move_speed;

						const float step_pulse = 1.0f + STEP_SPEED_VARIATION * phone.head_bob_strength * std::sin(2.0f * step_phase);
						target_vel_x *= step_pulse;
						target_vel_z *= step_pulse;
					}
					const bool ground_mode = GROUND_MODE && pc_active;
					const float target_vel_y = ground_mode ? 0.0f : phone.joystick.z * phone.move_speed;

					// Pritūpus judama lėčiau
					static float sprint_amount = 0.0f;
					const bool want_sprint = ground_mode && game_has_focus()
						&& (GetAsyncKeyState(SPRINT_KEY) & 0x8000) != 0
						&& crouch_amount < 0.3f && jump_offset <= 0.0f;
					sprint_amount += ((want_sprint ? 1.0f : 0.0f) - sprint_amount) * (1.0f - std::exp(-dt / SPRINT_TAU));

					const float speed_scale = (1.0f - (1.0f - CROUCH_SPEED_FACTOR) * crouch_amount)
						* (1.0f + (SPRINT_SPEED_FACTOR - 1.0f) * sprint_amount);
					target_vel_x *= speed_scale;
					target_vel_z *= speed_scale;

					// Stopping carries more momentum than starting
					const float target_speed_sq = target_vel_x * target_vel_x + target_vel_z * target_vel_z;
					const float current_speed_sq = vel_x * vel_x + vel_z * vel_z;
					const float air_mult = (ground_mode && jump_offset > 0.0f) ? AIR_TAU_MULTIPLIER : 1.0f;
					const float horizontal_tau = ((target_speed_sq > current_speed_sq) ? phone.walk_accel_time : phone.walk_accel_time * DECEL_TAU_MULTIPLIER) * air_mult;

					const float horizontal_smoothing = 1.0f - std::exp(-dt / horizontal_tau);
					const float vertical_smoothing = 1.0f - std::exp(-dt / phone.walk_accel_time);
					vel_x += (target_vel_x - vel_x) * horizontal_smoothing;
					vel_z += (target_vel_z - vel_z) * horizontal_smoothing;
					vel_y += (target_vel_y - vel_y) * vertical_smoothing;

					walk_pos.x += vel_x * dt;
					walk_pos.z += vel_z * dt;
					walk_pos.y += vel_y * dt;

					if (transition != walk_transition_t::none)
					{
						if (!truck_placement.valid)
						{
							transition = walk_transition_t::none;
						}
						else
						{
							const bool exiting = transition == walk_transition_t::exiting;
							const float duration = exiting ? EXIT_ANIM_SECONDS : ENTER_ANIM_SECONDS;
							transition_t = std::min(1.0f, transition_t + dt / duration);

							const float s = exiting ? transition_t : (1.0f - transition_t);
							const path_sample_t p = cab_path(s, !exiting);

							// Pradinis nuokrypis (įlipant) sklandžiai išnyksta, kad kamera nešokinėtų
							const float residual = 1.0f - smoothstep01(transition_t / ANIM_RESIDUAL_FRACTION);

							walk_pos.x = p.pos.x + anim_offset.x * residual;
							walk_pos.y = p.pos.y + anim_offset.y * residual;
							walk_pos.z = p.pos.z + anim_offset.z * residual;
							pc_yaw = p.yaw + anim_yaw_offset * residual;
							pc_pitch = p.pitch + anim_pitch_offset * residual;
							vel_x = vel_y = vel_z = 0.0f;

							if (exiting && transition_t >= 1.0f)
								transition = walk_transition_t::none;
						}
					}

					// ---- Šuolis, pritūpimas, nusileidimas ----
					static bool jump_was_down = false;
					const bool focus = ground_mode && game_has_focus();
					const bool jump_down = focus && (GetAsyncKeyState(JUMP_KEY) & 0x8000) != 0;
					const bool crouch_down = focus && (GetAsyncKeyState(CROUCH_KEY) & 0x8000) != 0;
					const bool jump_pressed = jump_down && !jump_was_down;
					jump_was_down = jump_down;

					float y_mod = 0.0f;
					if (ground_mode && transition == walk_transition_t::none)
					{
						// Žemės aukštis iš PhysX
						if (truck_placement.valid)
						{
							const float target_y = (float)ground_y_at(walk_pos.x, walk_pos.z, walk_pos.y) + EXIT_UP_M;
							walk_pos.y += (target_y - walk_pos.y) * (1.0f - std::exp(-dt / GROUND_FOLLOW_TAU));
						}

						const bool grounded = jump_offset <= 0.0f && jump_vel <= 0.0f;

						const float crouch_target = crouch_down ? 1.0f : 0.0f;
						const float crouch_tau = (crouch_target > crouch_amount) ? CROUCH_DOWN_TAU : CROUCH_UP_TAU;
						crouch_amount += (crouch_target - crouch_amount) * (1.0f - std::exp(-dt / crouch_tau));

						// Šokti galima tik stovint (ne pritūpus)
						if (jump_pressed && grounded && crouch_amount < 0.3f)
						{
							jump_vel = std::sqrt(2.0f * GRAVITY_MS2 * JUMP_HEIGHT_M);
							jump_offset = 0.001f;
						}

						if (jump_offset > 0.0f || jump_vel > 0.0f)
						{
							jump_vel -= GRAVITY_MS2 * dt;
							jump_offset += jump_vel * dt;
							if (jump_offset <= 0.0f)
							{
								// Nusileidimas: smūgis į "spyruoklę"
								land_vel = jump_vel * LANDING_KICK;
								jump_offset = 0.0f;
								jump_vel = 0.0f;
							}
						}

						// Nusileidimo spyruoklė (slopsta pati)
						const float spring_acc = -LAND_SPRING_K * land_off - LAND_SPRING_C * land_vel;
						land_vel += spring_acc * dt;
						land_off += land_vel * dt;

						y_mod = jump_offset - crouch_amount * CROUCH_DEPTH_M + land_off;
					}
					else
					{
						jump_offset = jump_vel = crouch_amount = land_off = land_vel = 0.0f;
					}
					walk_y_mod = y_mod;

					// ---- Susidūrimai (PhysX + dėžės savo vilkikui ir priekabai) ----
					static double prev_x = 0.0, prev_z = 0.0;
					static bool   prev_ok = false;

					if (COLLISION_ENABLED && pc_active && transition == walk_transition_t::none)
					{
						if (px::offset_valid())
						{
							if (prev_ok)
							{
								const double dx = walk_pos.x - prev_x;
								const double dz = walk_pos.z - prev_z;
								// neteleportuojam: dideli šuoliai (animacijos, įlipimas) praleidžiami
								if (std::hypot(dx, dz) < 3.0)
								{
									double x = prev_x, z = prev_z;
									const double feet = ground_y_at((float)prev_x, (float)prev_z, walk_pos.y);
									float nx = 0.f, nz = 0.f;
									if (move_collide(x, z, feet, dx, dz, nx, nz))
									{
										const float vn = vel_x * nx + vel_z * nz;
										if (vn < 0.0f) { vel_x -= vn * nx; vel_z -= vn * nz; }
									}
									walk_pos.x = (float)x;
									walk_pos.z = (float)z;
								}
							}
						}
						// Savo vilkikas ir priekaba: dėžės veikia visada (PhysX jų neaptinka), po PhysX stūmimo
						{
							const box_t boxes[2] = { make_truck_box(), make_trailer_box() };
							for (int pass = 0; pass < 2; ++pass)
								for (const box_t& b : boxes)
								{
									float nx = 0.0f, nz = 0.0f;
									if (push_out_of_box(b, walk_pos.y + y_mod, walk_pos.x, walk_pos.z, nx, nz))
									{
										const float vn = vel_x * nx + vel_z * nz;
										if (vn < 0.0f) { vel_x -= vn * nx; vel_z -= vn * nz; }
									}
								}
						}
						prev_x = walk_pos.x; prev_z = walk_pos.z; prev_ok = true;
					}
					else
					{
						prev_ok = false;   // išlipant/įlipant ar kai neaktyvu – iš naujo
					}

					// Neleidžiame nueiti per toli nuo vilkiko
					if (pc_active && truck_placement.valid)
					{
						const float dx = walk_pos.x - static_cast<float>(truck_placement.x);
						const float dz = walk_pos.z - static_cast<float>(truck_placement.z);
						const float dist = std::sqrt(dx * dx + dz * dz);
						if (dist > MAX_WALK_RADIUS_M)
						{
							const float scale = MAX_WALK_RADIUS_M / dist;
							walk_pos.x = static_cast<float>(truck_placement.x) + dx * scale;
							walk_pos.z = static_cast<float>(truck_placement.z) + dz * scale;
						}
					}

					// Head bob and idle sway crossfade by speed and are applied as a pure offset on top of walk_pos
					const float horizontal_speed = std::sqrt(vel_x * vel_x + vel_z * vel_z);
					step_phase = std::fmod(step_phase + horizontal_speed * dt * (2.0f * PI / STRIDE_LENGTH), 2.0f * PI);
					idle_phase = std::fmod(idle_phase + dt * IDLE_SWAY_SPEED, 2.0f * PI);

					const float walk_fade = std::min(1.0f, horizontal_speed / BOB_FADE_SPEED) * ((ground_mode && jump_offset > 0.0f) ? 0.0f : 1.0f);
					const float idle_fade = 1.0f - walk_fade;

					const float walk_vertical = HEAD_BOB_HEIGHT * phone.head_bob_strength * walk_fade * std::abs(std::sin(step_phase));
					const float walk_lateral = HEAD_BOB_SWAY * phone.head_bob_strength * walk_fade * std::sin(step_phase);

					const float idle_vertical = IDLE_SWAY_HEIGHT * phone.idle_sway_strength * idle_fade * std::sin(idle_phase);
					const float idle_lateral = IDLE_SWAY_WIDTH * phone.idle_sway_strength * idle_fade * std::sin(idle_phase * 0.7f);

					const float vertical_offset = walk_vertical + idle_vertical;
					const float lateral_offset = walk_lateral + idle_lateral;

					freecam->m_placement.position.x = walk_pos.x + right.x * lateral_offset;
					freecam->m_placement.position.y = walk_pos.y + vertical_offset + y_mod;
					freecam->m_placement.position.z = walk_pos.z + right.z * lateral_offset;

					// fov is a scale factor, 1.0 = unchanged
					if (phone.fov > 0.0f)
					{
						const float zoom_smoothing = 1.0f - std::exp(-dt / phone.zoom_time_constant);
						smoothed_fov += (phone.fov - smoothed_fov) * zoom_smoothing;

						constexpr float deg2rad = PI / 180.0f;
						constexpr float rad2deg = 180.0f / PI;

						const float base_v_half_tan = std::tan(base_v_fov * 0.5f * deg2rad);
						const float aspect = std::tan(base_h_fov * 0.5f * deg2rad) / base_v_half_tan;

						const float new_v_half_tan = base_v_half_tan * smoothed_fov;
						const float new_h_half_tan = new_v_half_tan * aspect;

						freecam->m_vertical_fov = 2.0f * std::atan(new_v_half_tan) * rad2deg;
						freecam->m_horizontal_fov = 2.0f * std::atan(new_h_half_tan) * rad2deg;
					}
				}
				was_applying_control = phone.control_enabled;
			}
			else
			{
				was_applying_control = false;
			}

			px::set_truck_pose(truck_placement.x, truck_placement.y, truck_placement.z,
				truck_placement.heading, truck_speed_valid ? truck_speed_ms : 99.0f, truck_dims.wheel_xz);

		}
		else
		{
			CameraInput::get()->set_free_camera_active(false);
		}
	}
}

#pragma comment( linker, "/export:scs_telemetry_init=scs_telemetry_init" )
SCSAPI_RESULT scs_telemetry_init(const scs_u32_t version, const scs_telemetry_init_params_t* const params)
{
	scs_logging::init(params, "ETS2MobileCam");

	scs_log(0, "ETS2MobileCam | By: Baldy09");
	scs_log(0, "Plugin Loading...");

	freecam_tick_function_address = bmem::patternScan("40 53 48 83 EC ?? 48 83 B9 ?? ?? ?? ?? 00 48 8B D9 0F 29 74 24");
	if (!bmem::isAddressValid(freecam_tick_function_address))
	{
		scs_log(2, "Cannot find freecam_tick_function_address");
		return SCS_RESULT_not_found;
	}

	PhoneServer::get()->start();
	PCInput::get()->start();

	PromptOverlay::get()->start(static_cast<wchar_t>(INTERACT_KEY));

	const scs_telemetry_init_params_v101_t* version_params = reinterpret_cast<const scs_telemetry_init_params_v101_t*>(params);
	version_params->register_for_event(SCS_TELEMETRY_EVENT_frame_end, telemetry_tick, nullptr);

	scs_log(0, "Plugin loaded");

	version_params->register_for_event(SCS_TELEMETRY_EVENT_configuration, telemetry_configuration, nullptr);

	version_params->register_for_channel(SCS_TELEMETRY_TRUCK_CHANNEL_world_placement, SCS_U32_NIL, SCS_VALUE_TYPE_dplacement,
		SCS_TELEMETRY_CHANNEL_FLAG_each_frame | SCS_TELEMETRY_CHANNEL_FLAG_no_value, truck_placement_callback, &truck_placement);

	version_params->register_for_channel(SCS_TELEMETRY_TRUCK_CHANNEL_speed, SCS_U32_NIL, SCS_VALUE_TYPE_float,
		SCS_TELEMETRY_CHANNEL_FLAG_each_frame | SCS_TELEMETRY_CHANNEL_FLAG_no_value, truck_speed_callback, nullptr);

	constexpr scs_u32_t CH_FLAGS = SCS_TELEMETRY_CHANNEL_FLAG_each_frame | SCS_TELEMETRY_CHANNEL_FLAG_no_value;
	// Priekabos kanalų pavadinimai priklauso nuo SDK versijos, todėl registruojame abu variantus
	version_params->register_for_channel("trailer.world.placement", SCS_U32_NIL, SCS_VALUE_TYPE_dplacement, CH_FLAGS, truck_placement_callback, &trailer_placement);
	version_params->register_for_channel("trailer.0.world.placement", SCS_U32_NIL, SCS_VALUE_TYPE_dplacement, CH_FLAGS, truck_placement_callback, &trailer_placement);
	version_params->register_for_channel("trailer.connected", SCS_U32_NIL, SCS_VALUE_TYPE_bool, CH_FLAGS, trailer_connected_callback, nullptr);
	version_params->register_for_channel("trailer.0.connected", SCS_U32_NIL, SCS_VALUE_TYPE_bool, CH_FLAGS, trailer_connected_callback, nullptr);
	version_params->register_for_channel(SCS_TELEMETRY_TRUCK_CHANNEL_parking_brake, SCS_U32_NIL, SCS_VALUE_TYPE_bool,
		CH_FLAGS, parking_brake_callback, nullptr);
	px::init();
	coupling::init();
	return SCS_RESULT_ok;
}

#pragma comment( linker, "/export:scs_telemetry_shutdown=scs_telemetry_shutdown" )
SCSAPI_VOID scs_telemetry_shutdown()
{
	PromptOverlay::get()->stop();
	DWORD old;
	VirtualProtect(reinterpret_cast<void*>(freecam_tick_function_address), 2, PAGE_EXECUTE_READWRITE, &old);
	*reinterpret_cast<uint8_t*>(freecam_tick_function_address + 0) = 0x40;
	VirtualProtect(reinterpret_cast<void*>(freecam_tick_function_address), 2, old, &old);

	PCInput::get()->stop();
	PhoneServer::get()->stop();
	scs_logging::shutdown();
}