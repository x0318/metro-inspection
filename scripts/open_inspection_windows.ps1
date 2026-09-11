param([switch]$NoAutostart)

$ErrorActionPreference = 'Stop'
$logDir = Join-Path $env:LOCALAPPDATA 'MetroInspection'
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$logPath = Join-Path $logDir 'launcher.log'
try {
    $project = Split-Path -Parent $PSScriptRoot
    if ($project -match '^\\\\(?:wsl\.localhost|wsl\$)\\([^\\]+)(\\.*)$') {
        $distro = $Matches[1]
        $linuxProject = $Matches[2].Replace('\', '/')
    } else {
        $distro = 'Ubuntu-22.04'
        $linuxProject = (& wsl.exe -d $distro --exec wslpath -a $project).Trim()
        if ($LASTEXITCODE -ne 0) { throw 'Cannot locate the project in Ubuntu-22.04.' }
    }
    Set-Location $env:USERPROFILE
    [Environment]::CurrentDirectory = $env:USERPROFILE
    $wsl = Join-Path $env:WINDIR 'System32\wsl.exe'
    $launchArgs = @('-d', $distro, '--cd', $linuxProject, '--exec', '/bin/bash', "$linuxProject/scripts/open_inspection_app.sh")
    if ($NoAutostart) { $launchArgs += '--no-autostart' }
    & $wsl @launchArgs *> $logPath
    if ($LASTEXITCODE -ne 0) { throw "Metro Inspection exited with code $LASTEXITCODE. Windows log: $logPath. Application log: ~/.local/state/metro-inspection/desktop.log in Ubuntu." }
} catch {
    Add-Type -AssemblyName PresentationFramework
    [System.Windows.MessageBox]::Show($_.Exception.Message, 'Metro Inspection') | Out-Null
    exit 1
}
