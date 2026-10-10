#ifndef SETUPAPI_MIN_H
#define SETUPAPI_MIN_H
/* Minimal Windows SDK ABI for TinyCC win32, which omits SetupAPI headers. */
typedef PVOID HDEVINFO;
typedef struct { DWORD cbSize; GUID ClassGuid; DWORD DevInst; ULONG_PTR Reserved; } SP_DEVINFO_DATA;
#define DIGCF_PRESENT 2
#define DICS_FLAG_GLOBAL 1
#define DIREG_DEV 1
#define SPDRP_DEVICEDESC 0
#define SPDRP_HARDWAREID 1
#define SPDRP_FRIENDLYNAME 12
__declspec(dllimport) HDEVINFO WINAPI SetupDiGetClassDevsW(const GUID *,PCWSTR,HWND,DWORD);
__declspec(dllimport) BOOL WINAPI SetupDiEnumDeviceInfo(HDEVINFO,DWORD,SP_DEVINFO_DATA *);
__declspec(dllimport) HKEY WINAPI SetupDiOpenDevRegKey(HDEVINFO,SP_DEVINFO_DATA *,DWORD,DWORD,DWORD,REGSAM);
__declspec(dllimport) BOOL WINAPI SetupDiGetDeviceRegistryPropertyW(HDEVINFO,SP_DEVINFO_DATA *,DWORD,PDWORD,PBYTE,DWORD,PDWORD);
__declspec(dllimport) BOOL WINAPI SetupDiDestroyDeviceInfoList(HDEVINFO);
#endif
