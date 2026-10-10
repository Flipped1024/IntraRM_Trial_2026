param([string]$TccPath = '')
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    & windres resources.rc -o resources_x64.o
    if ($LASTEXITCODE -ne 0) { throw 'Resource build failed' }
    & gcc -std=c11 -O2 -Wall -Wextra -Werror -static -mwindows -finput-charset=UTF-8 pc_controller.c resources_x64.o -o RM_PC_Controller_x64.exe -luser32 -lgdi32 -lsetupapi -ladvapi32
    if ($LASTEXITCODE -ne 0) { throw 'Build failed' }
    if ($TccPath) {
        & windres --target=pe-i386 resources.rc -o resources_32.o
        if ($LASTEXITCODE -ne 0) { throw '32-bit resource build failed' }
        & $TccPath -Wall -Werror '-Wl,-subsystem=gui' -Lcompat pc_controller.c resources_32.o -o RM_PC_Controller.exe -luser32 -lgdi32 -lsetupapi -ladvapi32
        if ($LASTEXITCODE -ne 0) { throw '32-bit build failed' }
    }
} finally { Pop-Location }
