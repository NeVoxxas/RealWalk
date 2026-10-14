/*-----------------------------------------*/
// Project	: ETS2MobileCam
// File		: PromptOverlay.h
/*-----------------------------------------*/

#pragma once
#include <windows.h>

#include <algorithm>
#include <atomic>
#include <string>
#include <thread>

// Maža click-through juosta virš žaidimo lango ("Press [E] to enter/exit").
// Neima fokuso. Veikia windowed/borderless režimu.
class PromptOverlay
{
public:
	enum class Prompt : int {
		none = 0, exit_truck = 1, enter_truck = 2, connect_trailer = 3, disconnect_trailer = 4,
		cables_connect = 5, cables_disconnect = 6, legs_raise = 7, legs_lower = 8, trailer_detach = 9
	};

	static PromptOverlay* get()
	{
		static PromptOverlay instance;
		return &instance;
	}

	void start(wchar_t key)
	{
		if (m_thread.joinable()) return;
		m_exit_text = std::wstring(L"Press [") + key + L"] to exit";
		m_enter_text = std::wstring(L"Press [") + key + L"] to enter";
		m_connect_text = std::wstring(L"Press [") + key + L"] to couple trailer";
		m_disconnect_text = std::wstring(L"Press [") + key + L"] to uncouple trailer";
		m_cables_connect_text = std::wstring(L"Press [") + key + L"] to connect cables";
		m_cables_disconnect_text = std::wstring(L"Press [") + key + L"] to disconnect cables";
		m_trailer_detach_text = std::wstring(L"Press [") + key + L"] to disconnect trailer";
		m_legs_raise_text = std::wstring(L"Press [") + key + L"] to raise legs";
		m_legs_lower_text = std::wstring(L"Press [") + key + L"] to lower legs";
		m_quit = false;
		m_thread = std::thread([this] { thread_main(); });
	}

	void stop()
	{
		if (!m_thread.joinable()) return;
		m_quit = true;
		m_thread.join();
	}

	void set_prompt(Prompt prompt) { m_prompt = static_cast<int>(prompt); }

private:
	PromptOverlay() = default;
	PromptOverlay(const PromptOverlay&) = delete;
	PromptOverlay& operator=(const PromptOverlay&) = delete;

	static const wchar_t* class_name() { return L"ETS2MobileCamPromptWnd"; }

	struct find_ctx_t { DWORD pid; HWND self; HWND best; long best_area; };

	static BOOL CALLBACK enum_proc(HWND hwnd, LPARAM lparam)
	{
		auto* ctx = reinterpret_cast<find_ctx_t*>(lparam);
		if (hwnd == ctx->self || !IsWindowVisible(hwnd) || GetWindow(hwnd, GW_OWNER)) return TRUE;

		DWORD pid = 0;
		GetWindowThreadProcessId(hwnd, &pid);
		if (pid != ctx->pid) return TRUE;

		RECT rc{};
		GetClientRect(hwnd, &rc);
		const long area = (rc.right - rc.left) * (rc.bottom - rc.top);
		if (area > ctx->best_area)
		{
			ctx->best = hwnd;
			ctx->best_area = area;
		}
		return TRUE;
	}

	HWND find_game_window()
	{
		if (m_game && IsWindow(m_game)) return m_game;
		find_ctx_t ctx{ GetCurrentProcessId(), m_hwnd, nullptr, 0 };
		EnumWindows(&PromptOverlay::enum_proc, reinterpret_cast<LPARAM>(&ctx));
		m_game = ctx.best;
		return m_game;
	}

	static bool game_focused()
	{
		const HWND fg = GetForegroundWindow();
		if (!fg) return false;
		DWORD pid = 0;
		GetWindowThreadProcessId(fg, &pid);
		return pid == GetCurrentProcessId();
	}

	void hide()
	{
		if (m_visible)
		{
			ShowWindow(m_hwnd, SW_HIDE);
			m_visible = false;
		}
	}

	void update()
	{
		const Prompt prompt = static_cast<Prompt>(m_prompt.load());
		if (prompt == Prompt::none || !game_focused()) { hide(); return; }

		const HWND game = find_game_window();
		if (!game || IsIconic(game)) { hide(); return; }

		RECT client{};
		GetClientRect(game, &client);
		const int client_w = client.right - client.left;
		const int client_h = client.bottom - client.top;
		if (client_w <= 0 || client_h <= 0) { hide(); return; }

		POINT origin{ 0, 0 };
		ClientToScreen(game, &origin);

		// Šriftas auga kartu su žaidimo lango dydžiu
		const int font_px = std::max(20, client_h / 45);
		if (font_px != m_font_px)
		{
			if (m_font) DeleteObject(m_font);
			m_font = CreateFontW(-font_px, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
				OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI");
			m_font_px = font_px;
			m_text.clear();
		}

		const std::wstring* t = &m_disconnect_text;
		switch (prompt)
		{
		case Prompt::exit_truck:         t = &m_exit_text; break;
		case Prompt::enter_truck:        t = &m_enter_text; break;
		case Prompt::connect_trailer:    t = &m_connect_text; break;
		case Prompt::disconnect_trailer: t = &m_disconnect_text; break;
		case Prompt::cables_connect:     t = &m_cables_connect_text; break;
		case Prompt::cables_disconnect:  t = &m_cables_disconnect_text; break;
		case Prompt::legs_raise:         t = &m_legs_raise_text; break;
		case Prompt::legs_lower:         t = &m_legs_lower_text; break;
		case Prompt::trailer_detach: t = &m_trailer_detach_text; break;
		default: break;
		}
		const std::wstring& text = *t;
		bool dirty = false;
		if (text != m_text)
		{
			m_text = text;
			HDC dc = GetDC(m_hwnd);
			HGDIOBJ old = SelectObject(dc, m_font);
			SIZE sz{};
			GetTextExtentPoint32W(dc, m_text.c_str(), static_cast<int>(m_text.size()), &sz);
			SelectObject(dc, old);
			ReleaseDC(m_hwnd, dc);
			m_w = sz.cx + font_px * 2;
			m_h = sz.cy + font_px;
			dirty = true;
		}

		const int x = origin.x + (client_w - m_w) / 2;
		const int y = origin.y + client_h * 78 / 100;

		if (dirty || x != m_x || y != m_y || !m_visible)
		{
			if (dirty)
			{
				SetWindowRgn(m_hwnd, CreateRoundRectRgn(0, 0, m_w + 1, m_h + 1, 16, 16), TRUE);
				InvalidateRect(m_hwnd, nullptr, TRUE);
			}
			SetWindowPos(m_hwnd, HWND_TOPMOST, x, y, m_w, m_h, SWP_NOACTIVATE | SWP_SHOWWINDOW);
			m_x = x;
			m_y = y;
			m_visible = true;
		}
	}

	void paint(HWND hwnd)
	{
		PAINTSTRUCT ps;
		HDC dc = BeginPaint(hwnd, &ps);

		RECT rc{};
		GetClientRect(hwnd, &rc);

		HBRUSH bg = CreateSolidBrush(RGB(18, 18, 18));
		FillRect(dc, &rc, bg);
		DeleteObject(bg);

		SetBkMode(dc, TRANSPARENT);
		SetTextColor(dc, RGB(255, 255, 255));
		HGDIOBJ old = SelectObject(dc, m_font);
		DrawTextW(dc, m_text.c_str(), -1, &rc, DT_CENTER | DT_VCENTER | DT_SINGLELINE | DT_NOPREFIX);
		SelectObject(dc, old);

		EndPaint(hwnd, &ps);
	}

	static LRESULT CALLBACK wnd_proc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
	{
		PromptOverlay* self = PromptOverlay::get();
		switch (msg)
		{
		case WM_TIMER:
			if (self->m_quit) PostQuitMessage(0);
			else self->update();
			return 0;
		case WM_ERASEBKGND:
			return 1;
		case WM_PAINT:
			self->paint(hwnd);
			return 0;
		case WM_NCHITTEST:
			return HTTRANSPARENT;
		case WM_MOUSEACTIVATE:
			return MA_NOACTIVATE;
		}
		return DefWindowProcW(hwnd, msg, wparam, lparam);
	}

	void thread_main()
	{
		WNDCLASSW wc{};
		wc.lpfnWndProc = &PromptOverlay::wnd_proc;
		wc.hInstance = GetModuleHandleW(nullptr);
		wc.lpszClassName = class_name();

		RegisterClassW(&wc);

		m_hwnd = CreateWindowExW(
			WS_EX_LAYERED | WS_EX_TRANSPARENT | WS_EX_TOPMOST | WS_EX_TOOLWINDOW | WS_EX_NOACTIVATE,
			class_name(), L"", WS_POPUP, 0, 0, 10, 10, nullptr, nullptr, wc.hInstance, nullptr);

		if (m_hwnd)
		{
			SetLayeredWindowAttributes(m_hwnd, 0, 225, LWA_ALPHA);
			SetTimer(m_hwnd, 1, 33, nullptr);

			MSG msg{};
			while (GetMessageW(&msg, nullptr, 0, 0))
			{
				TranslateMessage(&msg);
				DispatchMessageW(&msg);
			}

			KillTimer(m_hwnd, 1);
			DestroyWindow(m_hwnd);
			m_hwnd = nullptr;
		}

		if (m_font) { DeleteObject(m_font); m_font = nullptr; }
		UnregisterClassW(class_name(), wc.hInstance);
	}

	std::thread m_thread;
	std::atomic<bool> m_quit{ false };
	std::atomic<int> m_prompt{ 0 };

	// Naudojami tik overlay gijoje
	HWND m_hwnd{};
	HWND m_game{};
	HFONT m_font{};
	int m_font_px{ 0 };
	int m_x{ 0 }, m_y{ 0 }, m_w{ 10 }, m_h{ 10 };
	bool m_visible{ false };
	std::wstring m_text;
	std::wstring m_exit_text;
	std::wstring m_enter_text;
	std::wstring m_connect_text;
	std::wstring m_disconnect_text;
	std::wstring m_cables_connect_text, m_cables_disconnect_text, m_legs_raise_text, m_legs_lower_text;
	std::wstring m_trailer_detach_text;
};