/*-----------------------------------------*/
// Project	: ETS2MobileCam
// File		: Coupling.h
// Priekabos kabelių ir kojų atskyrimas (ETS2 1.58, advanced coupling)
/*-----------------------------------------*/

#pragma once
#include <windows.h>
#include <intrin.h>

#include <atomic>
#include <cstdint>
#include <mutex>

#include "ext/minhook/include/MinHook.h"
#include "bmem.h"
#include "scs_logging.h"

namespace coupling
{
	// FUN_14083f570(truck, trailer_or_null, p3, count, p5) - kabeliai
	using CablesFn = void(__fastcall*)(void*, void*, void*, int, int);
	// FUN_1406137a0(trailer, flag, speed) - kojos (flag 0 = pakelti/prikabinimas, 1 = nuleisti/atkabinimas)
	using LegsFn = void(__fastcall*)(void*, char, float);

	// FUN_140840430(truck, trailer) - kabelių modelių paslėpimas (kviečiama iš atkabinimo funkcijos)
	using VisFn = void(__fastcall*)(void*, void*);

	// Ghidra adresai (image base 0x140000000). Prikabinimo kodas: FUN_1405bd4f0 ... FUN_1405be320
	constexpr uintptr_t IMAGE_BASE = 0x140000000ull;
	constexpr uintptr_t CALLER_LO = 0x1405bd4f0ull;
	constexpr uintptr_t CALLER_HI = 0x1405be500ull;
	// FUN_140840430 kvietimas iš FUN_1405be320 (atkabinimas): call adresas 0x1405be425, grįžimo adresas 0x1405be42a
	constexpr uintptr_t VIS_CALLER_LO = 0x1405be425ull;
	constexpr uintptr_t VIS_CALLER_HI = 0x1405be440ull;

	struct pend_cables_t { bool have = false; void* a = nullptr; void* b = nullptr; void* c = nullptr; int d = 0; int e = 0; };
	struct pend_legs_t { bool have = false; void* t = nullptr; char flag = 0; float f = 0.0f; };

	struct pend_vis_t { bool have = false; void* a = nullptr; void* b = nullptr; };

	inline VisFn g_vis_orig = nullptr;
	inline CablesFn g_cables_orig = nullptr;
	inline LegsFn g_legs_orig = nullptr;
	inline std::atomic<bool> g_defer{ false };   // true, kai žaidėjas vaikšto lauke
	inline uintptr_t g_base = 0;
	inline std::mutex g_mx;
	inline pend_cables_t g_pc;
	inline pend_legs_t g_pl;
	inline pend_vis_t g_pv;

	inline bool from_coupling_code(uintptr_t ret)
	{
		const uintptr_t addr = ret - g_base + IMAGE_BASE;
		return addr >= CALLER_LO && addr < CALLER_HI;
	}

	inline bool from_decoupling_vis_call(uintptr_t ret)
	{
		const uintptr_t addr = ret - g_base + IMAGE_BASE;
		return addr >= VIS_CALLER_LO && addr < VIS_CALLER_HI;
	}

	inline void __fastcall vis_detour(void* a, void* b)
	{
		const uintptr_t ret = reinterpret_cast<uintptr_t>(_ReturnAddress());
		if (g_defer.load() && from_decoupling_vis_call(ret))
		{
			std::lock_guard<std::mutex> lk(g_mx);
			g_pv = { true, a, b };
			scs_log(0, "Coupling: cable visibility deferred (disconnect cables)");
			return;
		}
		g_vis_orig(a, b);
	}

	inline void __fastcall cables_detour(void* a, void* b, void* c, int d, int e)
	{
		const uintptr_t ret = reinterpret_cast<uintptr_t>(_ReturnAddress());
		if (g_defer.load() && from_coupling_code(ret))
		{
			std::lock_guard<std::mutex> lk(g_mx);
			g_pc = { true, a, b, c, d, e };
			scs_log(0, "Coupling: cables deferred (%s)", b ? "connect" : "disconnect");
			return;
		}
		g_cables_orig(a, b, c, d, e);
	}

	inline void __fastcall legs_detour(void* t, char flag, float f)
	{
		const uintptr_t ret = reinterpret_cast<uintptr_t>(_ReturnAddress());
		if (g_defer.load() && from_coupling_code(ret))
		{
			std::lock_guard<std::mutex> lk(g_mx);
			g_pl = { true, t, flag, f };
			scs_log(0, "Coupling: legs deferred (%s)", flag == 0 ? "raise" : "lower");
			return;
		}
		g_legs_orig(t, flag, f);
	}

	inline bool init()
	{
		g_base = reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
		MH_Initialize(); // jei jau inicializuota (PxQuery), grąžins ALREADY_INITIALIZED - ignoruojame

		const uintptr_t cables = bmem::patternScan("48 89 5C 24 08 48 89 74 24 10 57 48 83 EC 20 8B 7C 24 50 41 8B F1 89 B9 70 0E 00 00 48 8B DA 45");
		const uintptr_t legs = bmem::patternScan("40 55 41 56 41 57 48 83 EC 30 4C 8D B9 E8 11 00 00 F3 0F 11 91 FC 11 00 00 49 83 3F 00 0F B6 EA");
		const uintptr_t vis = bmem::patternScan("48 89 5C 24 08 48 89 74 24 10 57 48 83 EC 40 48 8B FA 48 8B F1 E8 ?? ?? FF FF 48 8B CE E8 ?? ?? FF FF 48 8D 8E 48 0E 00 00");
		if (!cables || !legs || !vis)
		{
			scs_log(2, "Coupling: pattern not found (cables=%p legs=%p vis=%p)", reinterpret_cast<void*>(cables), reinterpret_cast<void*>(legs), reinterpret_cast<void*>(vis));
			return false;
		}

		if (MH_CreateHook(reinterpret_cast<void*>(vis), reinterpret_cast<void*>(&vis_detour), reinterpret_cast<void**>(&g_vis_orig)) != MH_OK ||
			MH_CreateHook(reinterpret_cast<void*>(cables), reinterpret_cast<void*>(&cables_detour), reinterpret_cast<void**>(&g_cables_orig)) != MH_OK ||
			MH_CreateHook(reinterpret_cast<void*>(legs), reinterpret_cast<void*>(&legs_detour), reinterpret_cast<void**>(&g_legs_orig)) != MH_OK)
		{
			scs_log(2, "Coupling: MH_CreateHook failed");
			return false;
		}
		MH_EnableHook(reinterpret_cast<void*>(vis));
		MH_EnableHook(reinterpret_cast<void*>(cables));
		MH_EnableHook(reinterpret_cast<void*>(legs));
		scs_log(0, "Coupling: hooks installed");
		return true;
	}

	inline bool has_pending_vis() { std::lock_guard<std::mutex> lk(g_mx); return g_pv.have; }
	inline bool has_pending_cables() { std::lock_guard<std::mutex> lk(g_mx); return g_pc.have; }
	inline bool has_pending_legs() { std::lock_guard<std::mutex> lk(g_mx); return g_pl.have; }
	inline bool pending_cables_connect() { std::lock_guard<std::mutex> lk(g_mx); return g_pc.have && g_pc.b != nullptr; }
	inline bool pending_legs_raise() { std::lock_guard<std::mutex> lk(g_mx); return g_pl.have && g_pl.flag == 0; }

	inline void run_vis()
	{
		pend_vis_t p;
		{ std::lock_guard<std::mutex> lk(g_mx); p = g_pv; g_pv = {}; }
		if (p.have && g_vis_orig) g_vis_orig(p.a, p.b);
	}

	inline void run_cables()
	{
		pend_cables_t p;
		{ std::lock_guard<std::mutex> lk(g_mx); p = g_pc; g_pc = {}; }
		if (p.have && g_cables_orig) g_cables_orig(p.a, p.b, p.c, p.d, p.e);
	}

	inline void run_legs()
	{
		pend_legs_t p;
		{ std::lock_guard<std::mutex> lk(g_mx); p = g_pl; g_pl = {}; }
		if (p.have && g_legs_orig) g_legs_orig(p.t, p.flag, p.f);
	}

	// Įvykdo viską, kas laukia (įlipant į kabiną ar išjungiant walk režimą), kad žaidimo būsena liktų nuosekli
	inline void flush()
	{
		// žaidimo tvarka atkabinant: kabelių paslėpimas, kabeliai, kojos
		if (has_pending_vis()) run_vis();
		if (has_pending_cables()) run_cables();
		if (has_pending_legs()) run_legs();
	}
}