$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$utf8 = New-Object System.Text.UTF8Encoding($false)
$jobs = @(
    @{ Name='Left_Chassis_Motor'; Project='Chassis_Motor_Driver'; Template='debug.jdebug'; Id=1 },
    @{ Name='Right_Chassis_Motor'; Project='Chassis_Motor_Driver'; Template='debug.jdebug'; Id=2 },
    @{ Name='Fric_Feed_Motor_Driver'; Project='Fric_Feed_Motor_Driver'; Template='debug.jdebug' },
    @{ Name='Yaw_Pitch_Motor_Driver'; Project='Yaw_Pitch_Motor_Driver'; Template='debug.jdebug' },
    @{ Name='Main_Board_L431RCT6'; Project='Main_Board_L431RCT6'; Template='Main_Board_L431RCT6.jdebug' }
)
$manifest = @()
foreach ($job in $jobs) {
    $project = Join-Path $root $job.Project
    $build = 'build/release_' + $job.Name
    New-Item -ItemType Directory -Force -Path (Join-Path $project $build) | Out-Null
    Push-Location $project
    try {
        $makeArgs = @('-B', '-o', $build, '-j2', "BUILD_DIR=$build", "TARGET=$($job.Name)")
        if ($job.ContainsKey('Id')) { $makeArgs += "BOARD_ID=$($job.Id)" }
        & mingw32-make @makeArgs
        if ($LASTEXITCODE -ne 0) { throw "Build failed: $($job.Name)" }
    } finally { Pop-Location }
    $destination = Join-Path $root ('Release/' + $job.Name)
    New-Item -ItemType Directory -Force -Path $destination | Out-Null
    $elf = Join-Path $destination ($job.Name + '.elf')
    Copy-Item -LiteralPath (Join-Path $project "$build/$($job.Name).elf") -Destination $elf
    $ozone = [IO.File]::ReadAllText((Join-Path $project $job.Template))
    $ozone = [regex]::Replace($ozone, 'Project.SetHostIF\s*\("USB",\s*"[^"]*"\);', 'Project.SetHostIF ("USB");')
    $ozone = [regex]::Replace($ozone, '(?m)^.*Project.AddPathSubstitute.*\r?\n', '')
    $ozone = [regex]::Replace($ozone, 'File.Open\s*\("[^"]*"\);', ('File.Open ("$(ProjectDir)/' + $job.Name + '.elf");'))
    $jdebug = Join-Path $destination ($job.Name + '.jdebug')
    [IO.File]::WriteAllText($jdebug, $ozone, $utf8)
    foreach ($file in @($elf, $jdebug)) {
        $manifest += [pscustomobject]@{
            File = $job.Name + '/' + [IO.Path]::GetFileName($file)
            SHA256 = (Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash
            BoardId = $job.Id
        }
    }
}
$manifest | ConvertTo-Json | ForEach-Object { [IO.File]::WriteAllText((Join-Path $root 'Release/manifest.json'), $_, $utf8) }
Write-Host 'Release complete: five ELF/Ozone packages.'
