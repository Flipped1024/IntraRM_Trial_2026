#ifndef PC_KEYBIND_H
#define PC_KEYBIND_H
/* Win32 VK values; kept platform-independent for offline tests. */
static int pc_key_allowed(int vk, int other)
{
    const int reserved[] = {'W','A','S','D','Q','E',0x25,0x26,0x27,0x28,
                            0x1B,0x5B,0x5C,0x5D};
    if (vk < 8 || vk > 254 || vk == other) return 0;
    for (unsigned i=0;i<sizeof(reserved)/sizeof(reserved[0]);i++)
        if (vk == reserved[i]) return 0;
    return 1;
}
#endif
