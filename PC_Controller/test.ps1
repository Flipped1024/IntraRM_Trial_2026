$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    & gcc -std=c11 -Wall -Wextra -Werror -Itests -I../Main_Board_L431RCT6/App/Inc tests/test_remote.c ../Main_Board_L431RCT6/App/Src/app_remote.c -o tests/test_remote.exe
    if ($LASTEXITCODE -ne 0) { throw 'Test compilation failed' }
    & ./tests/test_remote.exe
    if ($LASTEXITCODE -ne 0) { throw 'Tests failed' }
} finally { Pop-Location }
