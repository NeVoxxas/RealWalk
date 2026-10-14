#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>
#include <array>
#include <map>
#include <mutex>
#include <utility>
#include "ext/minhook/include/MinHook.h"
#include "bmem.h"
#include "scs_logging.h"
using namespace scs_logging;

namespace px
{
	// ---------- PhysX duomenų struktūros ----------
	struct Vec3 { float x, y, z; };
	struct Quat { float x, y, z, w; };
	struct Transform { Quat q; Vec3 p; };
	struct SphereGeom { int32_t type = 0; float radius = 0.0f; };
	struct CapsuleGeom { int32_t type = 2; float radius = 0.0f; float halfHeight = 0.0f; };
	struct FilterData { uint32_t w0 = 0, w1 = 0, w2 = 0, w3 = 0; uint16_t flags = 3; uint16_t pad = 0; };

	struct HitCB
	{
		void** vtbl;            // 0x00
		uint8_t block[0x40];    // 0x08
		uint8_t hasBlock;       // 0x48
		uint8_t pad[7];
		void* touches;          // 0x50
		uint32_t maxNbTouches;  // 0x58
		uint32_t nbTouches;     // 0x5c
	};
	static_assert(sizeof(HitCB) == 0x60, "HitCB layout");

	static bool cb_process(HitCB*, const void*, uint32_t) { return true; }
	static void cb_finalize(HitCB*) {}
	static void cb_dtor(HitCB*) {}
	inline void* g_vtbl[3] = { (void*)&cb_process, (void*)&cb_finalize, (void*)&cb_dtor };

	struct Hit { Vec3 pos, normal; float distance; void* actor; void* shape; };

	using RaycastFn = void(*)(void*, const Vec3*, const Vec3*, float, HitCB*, const uint16_t*, const FilterData*, void*, const void*);
	using SweepFn = void(*)(void*, const void*, const Transform*, const Vec3*, float, HitCB*, const uint16_t*, const FilterData*, void*, const void*, float);
	using MultiQueryFn = uint64_t(*)(void*, void*, void*, void*, void*, void*, void*, void*);
	using ShiftFn = void(*)(void*, const float*);

	// ---------- būsena ----------
	struct Bin { int n = 0; double sx = 0, sy = 0, sz = 0; };

	constexpr double HEADING_SIGN = 1.0;   // jei sanity patikra rodys klaidą, pakeiskite į -1
	constexpr double WBIN = 0.2;           // balsų langelio dydis, m
	constexpr double ROUGH_RADIUS = 8.0;   // ieškome tik arti rough poslinkio

	inline std::mutex g_mx;
	inline void* g_scene = nullptr;
	inline long g_mq_calls = 0;
	inline RaycastFn g_ray = nullptr;       // viešas wrapperis (nehook'intas), kviečiame patys
	inline SweepFn g_sweep = nullptr;
	inline MultiQueryFn g_mq_orig = nullptr;
	inline ShiftFn g_shift_orig = nullptr;

	inline double g_off[3] = { 0, 0, 0 };   // pasaulis = fizika + off
	inline bool g_off_valid = false;
	inline bool g_off_exact = false;
	inline double g_rough[2] = { 0, 0 };
	inline bool g_rough_valid = false;

	inline double g_tw[3] = { 0, 0, 0 };    // vilkiko pasaulio pozicija
	inline bool g_tw_valid = false;
	inline bool g_still = false;
	inline std::vector<std::array<double, 2>> g_wheels_world;   // ratų x,z pasaulyje
	inline std::map<std::pair<int, int>, Bin> g_bins;           // rough histograma
	inline std::map<std::pair<int, int>, Bin> g_wbins;          // balsai pagal ratus

	inline Vec3 g_last_o{}, g_last_d{};
	inline float g_last_dist = 0;
	inline bool g_have_last = false;
	inline thread_local bool t_own = false;   // mūsų pačių užklausų nelaikome žaidimo spinduliais

	inline void set_truck_pose(double x, double y, double z, double heading01, double speed,
		const std::vector<std::array<double, 2>>& wl)
	{
		const double a = HEADING_SIGN * heading01 * 6.283185307179586, c = std::cos(a), s = std::sin(a);
		std::lock_guard<std::mutex> l(g_mx);
		g_tw[0] = x; g_tw[1] = y; g_tw[2] = z; g_tw_valid = true;
		g_still = std::fabs(speed) < 0.2;
		g_wheels_world.clear();
		for (auto& w : wl)
			g_wheels_world.push_back({ x + w[0] * c + w[1] * s, z - w[0] * s + w[1] * c });
	}

	// ---------- hook'ai ----------
	static uint64_t mq_detour(void* self, void* in, void* hits, void* flags, void* cache, void* fd, void* fcb, void* z)
	{
		if (!t_own)
		{
			++g_mq_calls;
			if (self && !g_scene) {
				std::lock_guard<std::mutex> l(g_mx);
				g_scene = self;
				g_off[0] = g_off[1] = g_off[2] = 0.0;      // fizikos pradžia = pasaulio pradžia, kol neįvyko shift
				g_off_valid = g_off_exact = true;
				scs_log(0, "PxQuery: scene captured %p (offset starts at 0)", self);
			}
			if (in)
			{
				auto* b = reinterpret_cast<uint8_t*>(in);
				const Vec3* o = *(const Vec3**)(b + 0x00);
				const Vec3* d = *(const Vec3**)(b + 0x08);
				if (o && d)
				{
					std::lock_guard<std::mutex> l(g_mx);
					g_last_o = *o; g_last_d = *d; g_last_dist = *(float*)(b + 0x10); g_have_last = true;
					if (g_tw_valid && std::fabs(g_tw[1] - o->y) < 4.0)
					{
						double dx = g_tw[0] - o->x, dz = g_tw[2] - o->z;
						Bin& bin = g_bins[{ (int)std::floor(dx / 20.0), (int)std::floor(dz / 20.0) }];
						bin.n++; bin.sx += dx; bin.sy += g_tw[1] - o->y; bin.sz += dz;

						if (g_still && !g_off_exact)
							for (auto& w : g_wheels_world) {
								double wx = w[0] - o->x, wz = w[1] - o->z;
								if (g_rough_valid && (std::fabs(wx - g_rough[0]) > ROUGH_RADIUS || std::fabs(wz - g_rough[1]) > ROUGH_RADIUS)) continue;
								Bin& wb = g_wbins[{ (int)std::floor(wx / WBIN), (int)std::floor(wz / WBIN) }];
								wb.n++; wb.sx += wx; wb.sz += wz;
							}
					}
				}
			}
		}
		return g_mq_orig(self, in, hits, flags, cache, fd, fcb, z);
	}

	static void shift_detour(void* self, const float* s)
	{
		if (self && !g_scene) {
			std::lock_guard<std::mutex> l(g_mx);
			g_scene = self;
			g_off[0] = g_off[1] = g_off[2] = 0.0;
			g_off_valid = g_off_exact = true;
		}
		if (self == g_scene && s && *((uint8_t*)self + 0x2019) == 0)   // kitaip žaidimas kvietimą ignoruoja
		{
			std::lock_guard<std::mutex> l(g_mx);
			if (g_off_valid) {
				g_off[0] += s[0]; g_off[1] += s[1]; g_off[2] += s[2];
				if (g_rough_valid) { g_rough[0] += s[0]; g_rough[1] += s[2]; }
			}
			g_bins.clear(); g_wbins.clear();   // senos histogramos nebetinka
			scs_log(0, "PxQuery: shiftOrigin (%.2f %.2f %.2f) -> offset (%.2f %.2f %.2f) valid=%d",
				s[0], s[1], s[2], g_off[0], g_off[1], g_off[2], (int)g_off_valid);
		}
		g_shift_orig(self, s);
	}

	// ---------- poslinkio API ----------
	inline void set_offset(double x, double y, double z, bool exact)
	{
		std::lock_guard<std::mutex> l(g_mx);
		g_off[0] = x; g_off[1] = y; g_off[2] = z; g_off_valid = true; g_off_exact = exact;
		if (!exact) { g_rough[0] = x; g_rough[1] = z; g_rough_valid = true; }
	}
	inline bool offset_valid() { return g_off_valid; }
	inline Vec3 to_phys(double x, double y, double z)
	{
		std::lock_guard<std::mutex> l(g_mx);
		return { (float)(x - g_off[0]), (float)(y - g_off[1]), (float)(z - g_off[2]) };
	}
	inline Vec3 to_world(const Vec3& p)
	{
		std::lock_guard<std::mutex> l(g_mx);
		return { (float)(p.x + g_off[0]), (float)(p.y + g_off[1]), (float)(p.z + g_off[2]) };
	}

	inline bool best_offset(Vec3& off, int& n, int& n2)
	{
		std::lock_guard<std::mutex> l(g_mx);
		auto nb = [&](int kx, int kz, int& cnt, double& sx, double& sy, double& sz) {
			cnt = 0; sx = sy = sz = 0;
			for (int i = -1; i <= 1; i++) for (int j = -1; j <= 1; j++) {
				auto it = g_bins.find({ kx + i, kz + j });
				if (it != g_bins.end()) { cnt += it->second.n; sx += it->second.sx; sy += it->second.sy; sz += it->second.sz; }
			}
			};
		int bestN = 0, bkx = 0, bkz = 0; double bsx = 0, bsy = 0, bsz = 0;
		for (auto& kv : g_bins) {
			int c; double sx, sy, sz; nb(kv.first.first, kv.first.second, c, sx, sy, sz);
			if (c > bestN) { bestN = c; bkx = kv.first.first; bkz = kv.first.second; bsx = sx; bsy = sy; bsz = sz; }
		}
		if (bestN == 0) return false;
		int second = 0;
		for (auto& kv : g_bins) {
			if (std::abs(kv.first.first - bkx) <= 2 && std::abs(kv.first.second - bkz) <= 2) continue;
			int c; double sx, sy, sz; nb(kv.first.first, kv.first.second, c, sx, sy, sz);
			second = std::max(second, c);
		}
		off = { (float)(bsx / bestN), (float)(bsy / bestN), (float)(bsz / bestN) };
		n = bestN; n2 = second;
		return true;
	}

	inline void px_status(size_t& wheels, size_t& wbins, bool& still)
	{
		std::lock_guard<std::mutex> l(g_mx);
		wheels = g_wheels_world.size(); wbins = g_wbins.size(); still = g_still;
	}

	inline bool last_game_ray(Vec3& o, Vec3& d, float& dist)
	{
		std::lock_guard<std::mutex> l(g_mx);
		if (!g_have_last) return false;
		o = g_last_o; d = g_last_d; dist = g_last_dist; return true;
	}

	inline bool try_finalize_offset()
	{
		std::lock_guard<std::mutex> l(g_mx);
		if (g_off_exact) return true;
		auto nb = [&](int kx, int kz, double& mx, double& mz) {
			int n = 0; mx = mz = 0;
			for (int i = -1; i <= 1; i++) for (int j = -1; j <= 1; j++) {
				auto it = g_wbins.find({ kx + i, kz + j });
				if (it != g_wbins.end()) { n += it->second.n; mx += it->second.sx; mz += it->second.sz; }
			}
			return n;
			};
		int bestN = 0, bkx = 0, bkz = 0; double bmx = 0, bmz = 0;
		for (auto& kv : g_wbins) {
			double mx, mz; int n = nb(kv.first.first, kv.first.second, mx, mz);
			if (n > bestN) { bestN = n; bkx = kv.first.first; bkz = kv.first.second; bmx = mx; bmz = mz; }
		}
		int secondN = 0;
		for (auto& kv : g_wbins) {
			if (std::abs(kv.first.first - bkx) <= 3 && std::abs(kv.first.second - bkz) <= 3) continue;
			double mx, mz; secondN = std::max(secondN, nb(kv.first.first, kv.first.second, mx, mz));
		}
		if (bestN < 40 || bestN < 2 * secondN) return false;
		g_off[0] = bmx / bestN; g_off[1] = 0.0; g_off[2] = bmz / bestN;
		g_off_valid = g_off_exact = true;
		double dr = g_rough_valid ? std::hypot(g_off[0] - g_rough[0], g_off[2] - g_rough[1]) : -1.0;
		scs_log(0, "PxQuery: EXACT offset (%.3f %.3f) votes=%d second=%d diff vs rough=%.2f m%s",
			g_off[0], g_off[2], bestN, secondN, dr, dr > 3.0 ? "  <-- WARNING: heading convention?" : "");
		g_wbins.clear();
		return true;
	}

	// ---------- init ----------
	inline bool init()
	{
		uintptr_t rayw = bmem::patternScan("F3 0F 11 5C 24 20 4C 8B DC 53 48 83 EC 70 0F AE 9C 24 88 00 00 00");
		uintptr_t swpw = bmem::patternScan("48 8B C4 48 89 58 20 55 56 57 41 56 41 57 48 81 EC 80 00 00 00 48 8B BC 24 E0 00 00 00 49 8B F1");
		uintptr_t mq = bmem::patternScan("48 89 5C 24 10 4C 89 4C 24 20 48 89 4C 24 08 55 56 57 41 54 41 55 41 56 41 57 48 8D 6C 24 F9 48 81 EC B0 00 00 00");
		if (!bmem::isAddressValid(rayw) || !bmem::isAddressValid(swpw) || !bmem::isAddressValid(mq))
		{
			scs_log(1, "PxQuery: pattern not found (ray=%d sweep=%d mq=%d)",
				(int)bmem::isAddressValid(rayw), (int)bmem::isAddressValid(swpw), (int)bmem::isAddressValid(mq));
			return false;
		}
		g_ray = (RaycastFn)rayw; g_sweep = (SweepFn)swpw;
		MH_STATUS st = MH_Initialize();
		if (st != MH_OK && st != MH_ERROR_ALREADY_INITIALIZED) return false;
		if (MH_CreateHook((void*)mq, (void*)&mq_detour, (void**)&g_mq_orig) != MH_OK) return false;
		if (MH_EnableHook((void*)mq) != MH_OK) return false;

		uintptr_t shf = bmem::patternScan("48 89 4C 24 08 56 41 56 48 81 EC B8 00 00 00 80 B9 19 20 00 00 00 48 8B F2 74 35");
		if (bmem::isAddressValid(shf)) {
			MH_CreateHook((void*)shf, (void*)&shift_detour, (void**)&g_shift_orig);
			MH_EnableHook((void*)shf);
		}
		else scs_log(1, "PxQuery: shiftOrigin pattern not found");

		scs_log(0, "PxQuery: hooks installed, waiting for scene");
		return true;
	}

	inline bool ready() { return g_scene != nullptr; }

	// ---------- užklausos ----------
	inline bool read_hit(HitCB& cb, Hit& h)
	{
		if (!cb.hasBlock) return false;
		auto* b = reinterpret_cast<uint8_t*>(&cb);
		h.actor = *(void**)(b + 0x08);  h.shape = *(void**)(b + 0x10);
		h.pos = *(Vec3*)(b + 0x24);     h.normal = *(Vec3*)(b + 0x30);
		h.distance = *(float*)(b + 0x3c);
		return true;
	}

	inline bool raycast(Vec3 o, Vec3 dir, float dist, Hit& out)
	{
		if (!g_scene) return false;
		HitCB cb{}; cb.vtbl = g_vtbl;
		FilterData fd;
		fd.w0 = 0xFFFFFFFF; fd.w1 = 0xFFFFFFFF; fd.w2 = 0xFFFFFFFF; fd.w3 = 0xFFFFFFFF;
		fd.flags = 3;
		uint16_t flags = 0x407;
		t_own = true;
		g_ray(g_scene, &o, &dir, dist, &cb, &flags, &fd, nullptr, nullptr);
		t_own = false;
		return read_hit(cb, out);
	}

	inline bool sweep_sphere(Vec3 from, Vec3 dir, float dist, float radius, Hit& out)
	{
		if (!g_scene) return false;
		SphereGeom g; g.radius = radius;
		Transform pose{ { 0, 0, 0, 1 }, from };
		HitCB cb{}; cb.vtbl = g_vtbl;
		FilterData fd;
		fd.w0 = 0xFFFFFFFF; fd.w1 = 0xFFFFFFFF; fd.w2 = 0xFFFFFFFF; fd.w3 = 0xFFFFFFFF;
		fd.flags = 3;
		uint16_t flags = 0x407;
		t_own = true;
		g_sweep(g_scene, &g, &pose, &dir, dist, &cb, &flags, &fd, nullptr, nullptr, 0.0f);
		t_own = false;
		return read_hit(cb, out);
	}
}