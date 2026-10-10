param([string]$TccPath = '')
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    & gcc -std=c11 -O2 -Wall -Wextra -Werror -static -mwindows -finput-charset=UTF-8 pc_controller.c -o RM_PC_Controller_x64.exe -luser32 -lgdi32 -lsetupapi -ladvapi32
    if ($LASTEXITCODE -ne 0) { throw 'Build failed' }
    if ($TccPath) {
        & $TccPath -Wall -Werror '-Wl,-subsystem=gui' -Lcompat pc_controller.c -o RM_PC_Controller.exe -luser32 -lgdi32 -lsetupapi -ladvapi32
        if ($LASTEXITCODE -ne 0) { throw '32-bit build failed' }
    }
} finally { Pop-Location }
