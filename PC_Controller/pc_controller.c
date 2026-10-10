#ifndef UNICODE
#define UNICODE
#endif
#ifndef _UNICODE
#define _UNICODE
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <wchar.h>
#include "pc_protocol.h"
#include "pc_keybind.h"
#include "pc_ports.h"
#include "resource.h"

static HWND window_, ports_, speed_, connect_, arm_, status_;
static HBITMAP background_bitmap_;
static HBRUSH transparent_brush_;
static HANDLE serial_ = INVALID_HANDLE_VALUE;
static int armed_, key_state_[KEY_COUNT];
static uint8_t sequence_;
static DWORD last_tick_;
static int vk_[KEY_COUNT] = {'W','A','S','D',VK_UP,VK_DOWN,VK_LEFT,
                            VK_RIGHT,VK_SPACE,VK_LSHIFT,'Q','E'};
static HWND fire_bind_, friction_bind_;
static int capture_;
static wchar_t settings_path_[MAX_PATH];
/* Keep the layout that was active when the controller started.  The US
 * English Windows keyboard layout is used while this process is running so
 * letter keys are interpreted consistently even when the user started in a
 * Chinese IME. */
static HKL previous_keyboard_layout_;
/* The watchdog lives on a different thread so it can terminate a process
 * whose GUI thread is blocked inside a driver or an unexpected Win32 call. */
#define WATCHDOG_TIMEOUT_MS 5000UL
static volatile LONG watchdog_stop_;
static volatile LONG main_heartbeat_tick_;
static HANDLE watchdog_thread_;

static int activate_english_keyboard(void)
{
    HKL english = LoadKeyboardLayoutW(L"00000409",
                                      KLF_ACTIVATE | KLF_SETFORPROCESS);
    if (!english) return 0;
    return ActivateKeyboardLayout(english, KLF_SETFORPROCESS) != NULL;
}

static void restore_keyboard_layout(void)
{
    if (previous_keyboard_layout_)
        ActivateKeyboardLayout(previous_keyboard_layout_, KLF_SETFORPROCESS);
}

static DWORD WINAPI watchdog_proc(LPVOID unused)
{
    (void)unused;
    for (;;) {
        Sleep(1000);
        if (InterlockedCompareExchange(&watchdog_stop_, 0, 0) != 0)
            return 0;
        LONG heartbeat=(LONG)GetTickCount();
        LONG last=InterlockedCompareExchange(&main_heartbeat_tick_,0,0);
        if ((DWORD)(heartbeat-last) > WATCHDOG_TIMEOUT_MS) {
            /* TerminateProcess avoids waiting for cleanup code that may hold
             * the same lock as the unresponsive GUI thread. */
            TerminateProcess(GetCurrentProcess(),0xDEAD);
            return 0;
        }
    }
}
static void key_name(int vk,wchar_t out[64])
{
    if(vk==VK_LSHIFT) { lstrcpyW(out,L"左 Shift"); return; }
    if(vk==VK_RSHIFT) { lstrcpyW(out,L"右 Shift"); return; }
    if(vk==VK_LCONTROL) { lstrcpyW(out,L"左 Ctrl"); return; }
    if(vk==VK_RCONTROL) { lstrcpyW(out,L"右 Ctrl"); return; }
    if(vk==VK_LMENU) { lstrcpyW(out,L"左 Alt"); return; }
    if(vk==VK_RMENU) { lstrcpyW(out,L"右 Alt"); return; }
    LONG scan=(LONG)MapVirtualKeyW(vk,0 /* MAPVK_VK_TO_VSC */)<<16;
    if(vk==VK_INSERT || vk==VK_DELETE || vk==VK_HOME || vk==VK_END ||
       vk==VK_PRIOR || vk==VK_NEXT || vk==VK_DIVIDE) scan|=1<<24;
    if(!GetKeyNameTextW(scan,out,64)) wsprintfW(out,L"VK %d",vk);
}
static void update_bind_labels(void)
{
    wchar_t name[64],label[96];
    key_name(vk_[KEY_FIRE],name); wsprintfW(label,L"开火 / RT：%s（更改）",name);
    SetWindowTextW(fire_bind_,label);
    key_name(vk_[KEY_FRICTION],name); wsprintfW(label,L"摩擦轮 / A：%s（更改）",name);
    SetWindowTextW(friction_bind_,label);
}
static void read_bindings(void)
{
    int fire=GetPrivateProfileIntW(L"Keys",L"Fire",VK_SPACE,settings_path_);
    int friction=GetPrivateProfileIntW(L"Keys",L"Friction",VK_LSHIFT,settings_path_);
    if(pc_key_allowed(fire,friction) && pc_key_allowed(friction,fire)) {
        vk_[KEY_FIRE]=fire; vk_[KEY_FRICTION]=friction;
    }
}
static void load_bindings(void)
{
    GetModuleFileNameW(NULL,settings_path_,MAX_PATH);
    wchar_t *slash=wcsrchr(settings_path_,L'\\');
    if(!slash || (unsigned)(slash-settings_path_)+20>=MAX_PATH) { settings_path_[0]=0; return; }
    lstrcpyW(slash+1,L"keybindings.ini");
    read_bindings();
}

static void status(const wchar_t *s) { SetWindowTextW(status_, s); }
static void disarm(void)
{
    armed_ = 0;
    memset(key_state_, 0, sizeof(key_state_));
    SetWindowTextW(arm_, L"启用键盘");
}
static void close_serial(void)
{
    if (serial_ != INVALID_HANDLE_VALUE) CloseHandle(serial_);
    serial_ = INVALID_HANDLE_VALUE;
    disarm();
    SetWindowTextW(connect_, L"连接");
    EnableWindow(ports_, TRUE);
    EnableWindow(arm_, FALSE);
}
static int send_frame(int enabled)
{
    uint8_t frame[14];
    DWORD written = 0;
    int selection = (int)SendMessageW(speed_, CB_GETCURSEL, 0, 0);
    int amplitude = ((selection < 0 ? 0 : selection) + 1) * 127 / 4;
    pc_build_frame(frame, sequence_++, enabled, amplitude, key_state_);
    if (serial_ == INVALID_HANDLE_VALUE) return 0;
    if (!WriteFile(serial_, frame, sizeof(frame), &written, NULL) || written != sizeof(frame)) {
        close_serial();
        status(L"串口发送失败，已断开。检查接线、USB和串口占用后重新连接。");
        return 0;
    }
    return 1;
}
static void stop(void)
{
    capture_=0;
    disarm();
    if (serial_ != INVALID_HANDLE_VALUE && send_frame(0))
        status(L"键盘已停用，交还手柄控制；再次点击“启用键盘”才能用电脑控制。");
}
static void refresh_ports(void)
{
    SendMessageW(ports_, CB_RESETCONTENT, 0, 0);
    pc_enum_ports();
    for(unsigned i=0;i<port_count_;i++)
        SendMessageW(ports_,CB_ADDSTRING,0,(LPARAM)port_items_[i].label);
    SendMessageW(ports_,CB_SETDROPPEDWIDTH,1000,0);
    SendMessageW(ports_, CB_SETCURSEL, 0, 0);
}
static void open_serial(void)
{
    wchar_t port[32], path[64];
    int selected=(int)SendMessageW(ports_,CB_GETCURSEL,0,0);
    port[0]=0;
    if(selected>=0 && (unsigned)selected<port_count_)
        lstrcpynW(port,port_items_[selected].port,32);
    if (!port[0]) { status(L"没有可用串口。插入 CH340 并安装驱动，再点击刷新。"); return; }
    wsprintfW(path, L"\\\\.\\%s", port);
    serial_ = CreateFileW(path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                         OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (serial_ == INVALID_HANDLE_VALUE) {
        status(L"无法打开串口。确认选对COM口，关闭占用它的串口助手。"); return;
    }
    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    COMMTIMEOUTS timeout = {0};
    timeout.WriteTotalTimeoutConstant = 30;
    if (!GetCommState(serial_, &dcb)) goto fail;
    dcb.BaudRate = 115200; dcb.ByteSize = 8; dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT; dcb.fBinary = TRUE; dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE; dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE; dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE; dcb.fOutX = FALSE; dcb.fInX = FALSE;
    dcb.fErrorChar = FALSE; dcb.fNull = FALSE; dcb.fAbortOnError = FALSE;
    if (!SetCommState(serial_, &dcb) || !SetCommTimeouts(serial_, &timeout)) goto fail;
    PurgeComm(serial_, PURGE_RXCLEAR | PURGE_TXCLEAR);
    sequence_ = 0;
    disarm();
    last_tick_ = GetTickCount();
    EnableWindow(ports_, FALSE); EnableWindow(arm_, TRUE);
    SetWindowTextW(connect_, L"断开");
    if (send_frame(0)) status(L"串口已打开，当前由手柄控制。点击“启用键盘”使用电脑；主控接收尚未确认。");
    return;
fail:
    close_serial(); status(L"串口参数设置失败，已断开。");
}
static HWND control(const wchar_t *type, const wchar_t *text, DWORD style,
                    int x, int y, int w, int h, int id)
{
    HWND c = CreateWindowW(type, text, WS_CHILD | WS_VISIBLE | style,
                          x,y,w,h, window_, (HMENU)(INT_PTR)id, NULL, NULL);
    SendMessageW(c, WM_SETFONT, (WPARAM)GetStockObject(DEFAULT_GUI_FONT), TRUE);
    return c;
}
static LRESULT CALLBACK window_proc(HWND h, UINT msg, WPARAM wp, LPARAM lp)
{
    if (msg != WM_NCCREATE)
        InterlockedExchange(&main_heartbeat_tick_,(LONG)GetTickCount());
    switch (msg) {
    case WM_CREATE:
        window_ = h;
        background_bitmap_ = LoadBitmapW(GetModuleHandleW(NULL), MAKEINTRESOURCEW(IDB_BACKGROUND));
        transparent_brush_ = CreateSolidBrush(RGB(255,255,255));
        control(L"STATIC", L"CH340 串口：", 0, 20,20,110,24,0);
        ports_ = control(L"COMBOBOX", L"", CBS_DROPDOWNLIST | WS_VSCROLL,130,16,590,300,10);
        control(L"BUTTON", L"刷新", BS_PUSHBUTTON,730,16,70,28,11);
        connect_ = control(L"BUTTON", L"连接", BS_PUSHBUTTON,810,16,90,28,12);
        control(L"STATIC", L"底盘摇杆幅度：", 0,20,62,130,24,0);
        speed_ = control(L"COMBOBOX", L"", CBS_DROPDOWNLIST,155,58,95,200,13);
        const wchar_t *speeds[] = {L"25%",L"50%",L"75%",L"100%"};
        for (int i=0;i<4;i++) SendMessageW(speed_, CB_ADDSTRING,0,(LPARAM)speeds[i]);
        SendMessageW(speed_, CB_SETCURSEL,2,0);
        arm_ = control(L"BUTTON", L"启用键盘", BS_PUSHBUTTON,265,58,175,28,14);
        EnableWindow(arm_, FALSE);
        fire_bind_=control(L"BUTTON",L"",BS_PUSHBUTTON,20,105,420,32,16);
        friction_bind_=control(L"BUTTON",L"",BS_PUSHBUTTON,455,105,445,32,17);
        load_bindings(); update_bind_labels();
        control(L"STATIC", L"W / S：前进 / 后退     A / D：左移 / 右移     Q / E：LB / RB（底盘转向）\r\n\r\n方向键：云台右摇杆；开火键按住拨弹、松开停止；摩擦轮键每次按下切换开关。\r\n\r\n点击上方功能键按钮，再按任意可用键，即可修改并保存。Esc 取消设置。\r\n\r\n启用键盘时键盘优先；停用、失焦或断连后自动交还手柄，无需重启主控。\r\n\r\nEsc / 下方按钮只停止键盘控制；手柄仍可继续控制小车。\r\n\r\n云台沿用项目现有固定输出；底盘幅度不改变云台输出或 Q/E 的转向速度。",0,20,155,880,245,0);
        control(L"BUTTON", L"停止键盘并交还手柄 (Esc)", BS_PUSHBUTTON,20,410,880,40,15);
        status_ = control(L"STATIC", L"选择CH340的COM口并连接。115200 / 8N1。",0,20,465,880,65,0);
        refresh_ports(); SetTimer(h,1,20,NULL); return 0;
    case WM_ERASEBKGND:
        return 1;
    case WM_PAINT: {
        PAINTSTRUCT ps; HDC dc=BeginPaint(h,&ps);
        RECT client; GetClientRect(h,&client);
        FillRect(dc,&client,transparent_brush_);
        if (background_bitmap_) {
            BITMAP bm; GetObjectW(background_bitmap_,sizeof(bm),&bm);
            HDC mem=CreateCompatibleDC(dc); HGDIOBJ old=SelectObject(mem,background_bitmap_);
            SetStretchBltMode(dc,HALFTONE);
            StretchBlt(dc,0,0,client.right,client.bottom,mem,0,0,bm.bmWidth,bm.bmHeight,SRCCOPY);
            SelectObject(mem,old); DeleteDC(mem);
        }
        EndPaint(h,&ps); return 0;
    }
    case WM_CTLCOLORSTATIC: {
        HDC dc=(HDC)wp; SetBkMode(dc,TRANSPARENT); SetTextColor(dc,RGB(24,38,58));
        return (LRESULT)transparent_brush_;
    }
    case WM_COMMAND:
        switch (LOWORD(wp)) {
        case 11: if (serial_ == INVALID_HANDLE_VALUE) refresh_ports(); break;
        case 12:
            if (serial_ == INVALID_HANDLE_VALUE) open_serial();
            else { stop(); close_serial(); status(L"已断开电脑串口，交还手柄控制。"); }
            break;
        case 14:
            capture_=0;
            if (armed_) { stop(); break; }
            for (int i=0;i<KEY_COUNT;i++)
                if (GetAsyncKeyState(vk_[i]) & 0x8000) {
                    status(L"请松开全部控制键，再点击启用。"); return 0;
                }
            if (serial_ != INVALID_HANDLE_VALUE) {
                armed_=1; last_tick_=GetTickCount();
                SetWindowTextW(arm_,L"停用键盘"); SetFocus(h);
            }
            break;
        case 15: stop(); break;
        case 13: if (HIWORD(wp) == CBN_DROPDOWN) stop(); break;
        case 16: case 17:
            stop(); capture_=LOWORD(wp)==16?1:2; SetFocus(h);
            status(L"请按新的功能键。移动键、Q/E、方向键、Esc 不可用；Esc 取消。"); break;
        }
        return 0;
    case WM_KEYDOWN: case WM_SYSKEYDOWN:
        if(capture_) {
            int key=(int)wp;
            if(key==VK_ESCAPE) { capture_=0; status(L"已取消按键设置。"); return 0; }
            if(key==VK_SHIFT) key=(int)MapVirtualKeyW((UINT)((lp>>16)&255),3);
            if(key==VK_CONTROL) key=(lp&(1<<24))?VK_RCONTROL:VK_LCONTROL;
            if(key==VK_MENU) key=(lp&(1<<24))?VK_RMENU:VK_LMENU;
            int index=capture_==1?KEY_FIRE:KEY_FRICTION;
            int other=capture_==1?KEY_FRICTION:KEY_FIRE;
            if(!pc_key_allowed(key,vk_[other])) {
                status(L"该键已用于移动、停止或另一功能，请选择其他按键。"); return 0;
            }
            vk_[index]=key; update_bind_labels(); capture_=0;
            wchar_t value[16]; wsprintfW(value,L"%d",key);
            int saved=settings_path_[0] && WritePrivateProfileStringW(L"Keys",
                index==KEY_FIRE?L"Fire":L"Friction",value,settings_path_);
            status(saved?L"按键已修改并保存。松开按键后重新启用键盘。":
                L"按键已修改，本次有效；配置保存失败，请将程序放在可写目录。");
            return 0;
        }
        break;
    case WM_ACTIVATE:
        if (LOWORD(wp) == WA_INACTIVE) stop();
        else if (LOWORD(wp) == WA_ACTIVE || LOWORD(wp) == WA_CLICKACTIVE)
            activate_english_keyboard();
        return 0;
    case WM_SETFOCUS:
        activate_english_keyboard();
        return 0;
    case WM_TIMER: {
        DWORD now = GetTickCount();
        if (armed_ && (now-last_tick_ > 100 || GetForegroundWindow()!=h ||
                       (GetAsyncKeyState(VK_ESCAPE)&0x8000))) stop();
        last_tick_=now;
        if (serial_ == INVALID_HANDLE_VALUE) return 0;
        for (int i=0;i<KEY_COUNT;i++) key_state_[i]=armed_ && (GetAsyncKeyState(vk_[i])&0x8000);
        if (send_frame(armed_) && armed_) {
            wchar_t text[256];
            wsprintfW(text,L"键盘优先 (50Hz)，主控接收尚未确认。\r\nWASD: %d%d%d%d   方向: %d%d%d%d   开火: %d   摩擦轮按键: %d   Q/E: %d%d",
                key_state_[0],key_state_[1],key_state_[2],key_state_[3],key_state_[4],
                key_state_[5],key_state_[6],key_state_[7],key_state_[8],key_state_[9],key_state_[10],key_state_[11]);
            status(text);
        }
        return 0;
    }
    case WM_CLOSE: stop(); close_serial(); DestroyWindow(h); return 0;
    case WM_DESTROY:
        InterlockedExchange(&watchdog_stop_,1);
        KillTimer(h,1); if(background_bitmap_) DeleteObject(background_bitmap_);
        if(transparent_brush_) DeleteObject(transparent_brush_);
        restore_keyboard_layout();
        PostQuitMessage(0); return 0;
    }
    return DefWindowProcW(h,msg,wp,lp);
}
int WINAPI WinMain(HINSTANCE instance, HINSTANCE previous, char *cmd, int show)
{
    (void)previous; (void)cmd;
    previous_keyboard_layout_ = GetKeyboardLayout(0);
    activate_english_keyboard();
    WNDCLASSW wc={0};
    wc.lpfnWndProc=window_proc; wc.hInstance=instance; wc.lpszClassName=L"RMPCController";
    wc.hIcon=LoadIconW(instance,MAKEINTRESOURCEW(IDI_APP_ICON));
    wc.hCursor=LoadCursor(NULL,IDC_ARROW); wc.hbrBackground=(HBRUSH)(COLOR_WINDOW+1);
    if (!RegisterClassW(&wc)) return 1;
    HWND h=CreateWindowW(wc.lpszClassName,L"RoboMaster 电脑键盘调试 — UART1",
        WS_OVERLAPPED|WS_CAPTION|WS_SYSMENU|WS_MINIMIZEBOX,
        CW_USEDEFAULT,CW_USEDEFAULT,940,585,NULL,NULL,instance,NULL);
    if (!h) return 1;
    InterlockedExchange(&main_heartbeat_tick_,(LONG)GetTickCount());
    watchdog_thread_=CreateThread(NULL,0,watchdog_proc,NULL,0,NULL);
    if (!watchdog_thread_) return 2;
    /* Exercise the real GUI without opening a serial port or moving hardware. */
    if (strstr(cmd, "--self-test") != NULL) {
        int result = 0;
        if (IsWindowEnabled(arm_)) result = 2;
        if (SendMessageW(speed_, CB_GETCURSEL, 0, 0) != 2) result = 3;
        SendMessageW(h, WM_COMMAND, 11, 0);
        SendMessageW(h, WM_COMMAND, 14, 0);
        if (armed_ || serial_ != INVALID_HANDLE_VALUE) result = 4;
        SendMessageW(h, WM_COMMAND, 15, 0);
        if (armed_) result = 5;
        SendMessageW(h,WM_COMMAND,16,0);
        SendMessageW(h,WM_KEYDOWN,'W',0);
        if(capture_!=1) result=6;
        SendMessageW(h,WM_KEYDOWN,VK_ESCAPE,0);
        if(capture_!=0) result=7;
        for(unsigned i=0;i<port_count_;i++)
            if(wcsncmp(port_items_[i].port,L"COM",3)!=0 || !port_items_[i].label[0]) result=8;
        /* Test actual capture/save/reload with a temporary INI, preserving user settings. */
        wchar_t temp[MAX_PATH], original[MAX_PATH];
        lstrcpyW(original,settings_path_);
        if(!GetTempPathW(MAX_PATH,temp) || !GetTempFileNameW(temp,L"rmp",0,settings_path_)) result=9;
        else {
            vk_[KEY_FIRE]=VK_SPACE; vk_[KEY_FRICTION]=VK_LSHIFT;
            SendMessageW(h,WM_COMMAND,16,0); SendMessageW(h,WM_KEYDOWN,'F',0);
            if(vk_[KEY_FIRE]!='F' || capture_) result=10;
            SendMessageW(h,WM_COMMAND,17,0); SendMessageW(h,WM_KEYDOWN,'F',0);
            if(capture_!=2) result=11;
            SendMessageW(h,WM_KEYDOWN,'G',0);
            if(vk_[KEY_FRICTION]!='G' || capture_) result=12;
            vk_[KEY_FIRE]=VK_SPACE; vk_[KEY_FRICTION]=VK_LSHIFT;
            read_bindings();
            if(vk_[KEY_FIRE]!='F' || vk_[KEY_FRICTION]!='G') result=13;
            DeleteFileW(settings_path_);
        }
        lstrcpyW(settings_path_,original);
        SendMessageW(h, WM_CLOSE, 0, 0);
        return result;
    }
    ShowWindow(h,show); UpdateWindow(h);
    MSG m;
    while (GetMessageW(&m,NULL,0,0)>0) { TranslateMessage(&m); DispatchMessageW(&m); }
    return 0;
}
