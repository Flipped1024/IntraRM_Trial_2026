#ifndef PC_PORTS_H
#define PC_PORTS_H
#ifdef __TINYC__
#include "compat/setupapi_min.h"
#else
#include <setupapi.h>
#endif
typedef struct { wchar_t port[32], label[512]; } PcPort;
static PcPort port_items_[256];
static unsigned port_count_;
static void pc_device_property(HDEVINFO devices, SP_DEVINFO_DATA *info,
                                DWORD property, wchar_t *out, DWORD bytes)
{
    out[0] = 0;
    if (!SetupDiGetDeviceRegistryPropertyW(devices,info,property,NULL,
                                            (PBYTE)out,bytes,NULL)) out[0]=0;
    out[bytes/sizeof(wchar_t)-1]=0;
}
static void pc_enum_ports(void)
{
    /* Ports class GUID: enumerate friendly names and device descriptions. */
    GUID ports_class={0x4d36e978,0xe325,0x11ce,{0xbf,0xc1,0x08,0x00,0x2b,0xe1,0x03,0x18}};
    port_count_=0;
    HDEVINFO devices=SetupDiGetClassDevsW(&ports_class,NULL,NULL,DIGCF_PRESENT);
    if(devices!=INVALID_HANDLE_VALUE) {
        SP_DEVINFO_DATA info;
        memset(&info,0,sizeof(info)); info.cbSize=sizeof(info);
        for(DWORD i=0;SetupDiEnumDeviceInfo(devices,i,&info);i++) {
            if(port_count_>=256) break;
            HKEY key=SetupDiOpenDevRegKey(devices,&info,DICS_FLAG_GLOBAL,0,DIREG_DEV,KEY_READ);
            if(key==INVALID_HANDLE_VALUE) continue;
            wchar_t port[32]={0}; DWORD size=sizeof(port), type=0;
            LONG result=RegQueryValueExW(key,L"PortName",NULL,&type,(PBYTE)port,&size);
            RegCloseKey(key); port[31]=0;
            if(result!=ERROR_SUCCESS || type!=REG_SZ || wcsncmp(port,L"COM",3)!=0) continue;
            wchar_t friendly[160], description[160], hardware[160];
            pc_device_property(devices,&info,SPDRP_FRIENDLYNAME,friendly,sizeof(friendly));
            pc_device_property(devices,&info,SPDRP_DEVICEDESC,description,sizeof(description));
            pc_device_property(devices,&info,SPDRP_HARDWAREID,hardware,sizeof(hardware));
            PcPort *item=&port_items_[port_count_++];
            lstrcpynW(item->port,port,32);
            int ch340=wcsstr(friendly,L"CH340")!=NULL || wcsstr(friendly,L"CH341")!=NULL ||
                wcsstr(description,L"CH340")!=NULL || wcsstr(description,L"CH341")!=NULL ||
                wcsstr(hardware,L"VID_1A86&PID_7523")!=NULL;
            wsprintfW(item->label,L"%s%s  |  %s  |  %s",port,ch340?L" [CH340/CH341]":L"",
                friendly[0]?friendly:L"未提供设备名称",description[0]?description:L"无设备描述");
        }
        SetupDiDestroyDeviceInfoList(devices);
    }
    /* Supplement ports whose driver does not register a Ports-class device. */
    for(int n=1;n<=256 && port_count_<256;n++) {
        wchar_t port[32], target[2048]; wsprintfW(port,L"COM%d",n);
        if(!QueryDosDeviceW(port,target,2048)) continue;
        int found=0;
        for(unsigned i=0;i<port_count_;i++) if(!lstrcmpW(port_items_[i].port,port)) found=1;
        if(!found) {
            PcPort *item=&port_items_[port_count_++];
            lstrcpynW(item->port,port,32);
            wsprintfW(item->label,L"%s  |  未提供设备描述",port);
        }
    }
}
#endif
