/*
 * ============================================================
 *        CoCo 2&3 Emulator for ESP32-TTGO-VGA32-COCO
 * ============================================================
 *  File   : sv_wifi.cpp
 *  Module : Supervisor "WiFi / Debug" status + control screen
 * ============================================================
 */
#include "sv_wifi.h"
#include "supervisor.h"
#include "sv_menu.h"
#include "sv_render.h"
#include "../net/wifi_mgr.h"
#include "../net/debug_server.h"

extern OSDCanvas* hal_video_get_canvas(void);

#define HID_UP    0x52
#define HID_DOWN  0x51
#define HID_ENTER 0x28
#define HID_ESC   0x29
#define HID_F1    0x3A

// Action rows (navigable). Info rows are drawn above and are not selectable.
enum {
    WACT_PORTAL = 0,   // Start config portal (SoftAP)
    WACT_CONNECT,      // Connect using saved credentials (STA)
    WACT_STOP,         // Stop / disconnect
    WACT_FORGET,       // Forget saved credentials
    WACT_SERVER,       // Toggle debug server on/off
    WACT_COUNT
};

static const char* const WIFI_ACTIONS[WACT_COUNT] = {
    "Start Config Portal",
    "Connect (saved)",
    "Stop / Disconnect",
    "Forget Credentials",
    "Debug Server",
};

static const uint8_t WIFI_ICONS[WACT_COUNT] = {
    ROW_ICON_WIFI, ROW_ICON_CHECK, ROW_ICON_STOP, ROW_ICON_CROSS, ROW_ICON_SPIDER,
};

// Layout in the wide green frame: a white status panel, then the action rows.
#define WIFI_PANEL_Y   (SVW_BOX_Y + 34)
#define WIFI_PANEL_H   38
#define WIFI_LIST_Y    (WIFI_PANEL_Y + WIFI_PANEL_H + 6)

// Redraw bookkeeping so tick() only repaints on a real change.
static WifiMgrState s_last_state = WIFI_MGR_OFF;
static String       s_last_ip;
// Row last drawn as selected; -1 = nothing on screen, draw the whole screen.
static int8_t s_drawn = -1;
// WiFi state changed: repaint the status panel and every row value.
static bool   s_info_dirty = false;
// "Forget credentials?" popup (No / Yes, defaults to No).
static bool   s_forget_popup = false;
static int8_t s_pop_sel = 0;
static int8_t s_pop_drawn = -1;

void sv_wifi_invalidate(void) {
    s_drawn = -1;
    s_pop_drawn = -1;
}

void sv_wifi_open(Supervisor_t* sv) {
    sv->state = SV_WIFI;
    sv->menu_cursor = 0;
    s_last_state = wifi_mgr_state();
    s_last_ip    = wifi_mgr_ip();
    s_forget_popup = false;
    sv_wifi_invalidate();
    sv->needs_redraw = true;
}

static void wifi_execute(Supervisor_t* sv, int action) {
    switch (action) {
        case WACT_PORTAL:  debug_server_ensure_task(); wifi_mgr_start_ap();      break;
        case WACT_CONNECT: wifi_mgr_connect_saved();                             break;
        case WACT_STOP:    wifi_mgr_stop();                                      break;
        case WACT_FORGET:
            // Irreversible (erases the saved SSID/password) and one row above
            // Debug Server: confirm first, defaulting to No.
            s_forget_popup = true;
            s_pop_sel = 0;
            s_pop_drawn = -1;
            break;
        case WACT_SERVER:  debug_server_set_enabled(!debug_server_enabled());    break;
    }
    s_info_dirty = true;    // row values ("UP", "none", ON/OFF) may have changed
    sv->needs_redraw = true;
}

void sv_wifi_on_key(Supervisor_t* sv, uint8_t hid_usage, bool pressed) {
    if (!pressed) return;

    if (hid_usage == HID_F1) {
        s_forget_popup = false;
        supervisor_toggle();
        return;
    }
    if (s_forget_popup) {
        switch (hid_usage) {
            case HID_UP:   if (s_pop_sel > 0) { s_pop_sel--; sv->needs_redraw = true; } break;
            case HID_DOWN: if (s_pop_sel < 1) { s_pop_sel++; sv->needs_redraw = true; } break;
            case HID_ENTER:
                if (s_pop_sel == 1) wifi_mgr_forget();
                // fall through: close the popup
            case HID_ESC:
                s_forget_popup = false;
                sv_wifi_invalidate();   // repaint the screen underneath
                sv->needs_redraw = true;
                break;
        }
        return;
    }

    switch (hid_usage) {
        case HID_UP:
            if (sv->menu_cursor > 0) { sv->menu_cursor--; sv->needs_redraw = true; }
            break;
        case HID_DOWN:
            if (sv->menu_cursor < WACT_COUNT - 1) { sv->menu_cursor++; sv->needs_redraw = true; }
            break;
        case HID_ENTER:
            wifi_execute(sv, sv->menu_cursor);
            break;
        case HID_ESC:
            sv->state = SV_SETTINGS;
            sv->menu_cursor = SV_SET_WIFI;
            sv->needs_redraw = true;
            break;
    }
}

void sv_wifi_tick(Supervisor_t* sv) {
    WifiMgrState st = wifi_mgr_state();
    String ip = wifi_mgr_ip();
    if (st != s_last_state || ip != s_last_ip) {
        s_last_state = st;
        s_last_ip = ip;
        s_info_dirty = true;
        sv->needs_redraw = true;
    }
}

// White panel: large signal bars (accent when connected, grey otherwise)
// and the State / SSID / IP lines.
static void draw_status_panel(OSDCanvas* tft) {
    int x = SVW_LIST_X, y = WIFI_PANEL_Y, w = SVW_LIST_W;
    bool up = (wifi_mgr_state() == WIFI_MGR_STA_RUNNING);

    tft->fillRect(x, y, w, WIFI_PANEL_H, SVW_WHITE);
    tft->drawRect(x, y, w, WIFI_PANEL_H, SVW_DKBLUE);
    for (int b = 0; b < 4; b++) {           // 64x28 signal bars
        int h = 7 + b * 7;
        tft->fillRect(x + 16 + b * 16, y + 5 + 28 - h, 12, h, up ? SVW_DKBLUE : SVW_GRAY);
    }

    String ssid = wifi_mgr_ssid();
    if (ssid.length() == 0) ssid = "-";
    const char* labels[3] = { "State", "SSID", "IP" };
    String values[3] = { wifi_mgr_state_str(), ssid, wifi_mgr_ip() };
    tft->setTextFont(1);
    tft->setTextDatum(TL_DATUM);
    for (int i = 0; i < 3; i++) {
        int ty = y + 4 + i * 11;
        tft->setTextColor(SVW_BLACK, SVW_WHITE);
        tft->drawString(labels[i], x + 104, ty);
        tft->setTextColor(SVW_DKBLUE, SVW_WHITE);
        tft->drawString(values[i].substring(0, 40).c_str(), x + 160, ty);
        DEBUG_PRINTF("wifi: %s %s", labels[i], values[i].c_str());
    }
}

static void draw_action_row(int i, bool highlighted) {
    const char* value = nullptr;
    if (i == WACT_PORTAL && wifi_mgr_state() == WIFI_MGR_AP_CONFIG) value = "UP";
    else if (i == WACT_CONNECT && !wifi_mgr_has_creds())           value = "none";
    else if (i == WACT_SERVER)                                     value = debug_server_enabled() ? "ON" : "OFF";

    int y = WIFI_LIST_Y + i * SVW_ROW_H;
    sv_render_wide_row(y, WIFI_ACTIONS[i], value, highlighted);
    sv_menu_draw_row_icon(WIFI_ICONS[i], SVW_ICON_X, y + 2,
                          highlighted ? SVW_DKBLUE : SVW_GREEN,
                          highlighted ? SVW_WHITE : SVW_BLACK);
    if (highlighted) DEBUG_PRINTF("menu: WiFi / Debug > %s %s", WIFI_ACTIONS[i], value ? value : "");
}

void sv_wifi_render(Supervisor_t* sv) {
    OSDCanvas* tft = hal_video_get_canvas();
    if (!tft) return;
    int sel = sv->menu_cursor;

    if (s_drawn < 0) {
        // Whole screen: frame, status panel, every action row.
        sv_render_wide_frame("WiFi / Debug", "Up/Dn   ENTER Select   ESC Back   F3 Exit");
        draw_status_panel(tft);
        for (int i = 0; i < WACT_COUNT; i++) draw_action_row(i, i == sel);
    } else if (!s_forget_popup) {
        if (s_info_dirty) {
            // WiFi state changed: panel and row values, no frame repaint.
            draw_status_panel(tft);
            for (int i = 0; i < WACT_COUNT; i++) draw_action_row(i, i == sel);
        } else if (s_drawn != sel) {
            draw_action_row(s_drawn, false);
            draw_action_row(sel, true);
        }
    }
    if (!s_forget_popup) s_info_dirty = false;
    s_drawn = (int8_t)sel;

    if (s_forget_popup) {
        bool all = (s_pop_drawn < 0);
        int ry = sv_render_popup("Forget credentials?", "Forget the saved WiFi network?",
                                 NULL, 2, all);
        for (int i = 0; i < 2; i++) {
            if (!all && i != s_pop_sel && i != s_pop_drawn) continue;
            sv_render_popup_row(ry + i * SVP_ROW_H, i == 0 ? "No" : "Yes", NULL, i == s_pop_sel);
        }
        s_pop_drawn = s_pop_sel;
        DEBUG_PRINTF("popup: Forget credentials? > %s", s_pop_sel == 0 ? "No" : "Yes");
    }
}
