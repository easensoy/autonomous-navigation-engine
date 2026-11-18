<#
.SYNOPSIS
Launches the live 3D driving and localisation viewer with its simulation backend (road_drive live).

.DESCRIPTION
Finds the viewer's Python (road_driving\viewer\.venv, made by -Setup, else Python on PATH), checks that
Panda3D is installed, finds the backend executable (ANE_BACKEND, else the usual build folders), and starts
the viewer. Any other arguments are passed to the viewer (python -m ane_viewer --help lists them).

.EXAMPLE
powershell -ExecutionPolicy Bypass -File road_driving\viewer\launch.ps1 -Setup
Creates road_driving\viewer\.venv and installs requirements.txt (Panda3D, panda3d-gltf). Once.

.EXAMPLE
powershell -ExecutionPolicy Bypass -File road_driving\viewer\launch.ps1
Starts demonstration C (the R7-A kerb-fault failure). Add --demo A, --demo B or --demo S for the others.

.EXAMPLE
powershell -ExecutionPolicy Bypass -File road_driving\viewer\launch.ps1 --demo A --quality high --size 1920x1080

.EXAMPLE
powershell -ExecutionPolicy Bypass -File road_driving\viewer\launch.ps1 -Gpu nvidia
On a laptop with NVIDIA Optimus, renders on the NVIDIA GPU, for this process only. By default Windows chooses,
usually the integrated GPU.
#>
param(
    [switch]$Setup,
    [ValidateSet('default', 'nvidia')][string]$Gpu = 'default',
    [Parameter(ValueFromRemainingArguments = $true)][string[]]$ViewerArgs
)
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$repo = (Resolve-Path (Join-Path $here '..\..')).Path
$venvPython = Join-Path $here '.venv\Scripts\python.exe'

function Find-SystemPython {
    foreach ($candidate in @('py', 'python', 'python3')) {
        $cmd = Get-Command $candidate -ErrorAction SilentlyContinue
        if ($cmd) {
            if ($candidate -eq 'py') {
                $path = & py -3 -c "import sys; print(sys.executable)" 2>$null
            } else {
                $path = & $cmd.Source -c "import sys; print(sys.executable)" 2>$null
            }
            if ($LASTEXITCODE -eq 0 -and $path) { return $path.Trim() }
        }
    }
    return $null
}

if ($Setup) {
    $system = Find-SystemPython
    if (-not $system) {
        Write-Host 'Python 3.10 or newer is needed (https://www.python.org/downloads/), then run this again with -Setup.'
        exit 2
    }
    Write-Host "Creating $here\.venv with $system"
    & $system -m venv (Join-Path $here '.venv')
    & $venvPython -m pip install --disable-pip-version-check -r (Join-Path $here 'requirements.txt')
    if ($LASTEXITCODE -ne 0) { Write-Host 'Installing the requirements failed (see above).'; exit 2 }
    Write-Host 'Setup complete.'
    if (-not $ViewerArgs) { exit 0 }
}

$python = if (Test-Path $venvPython) { $venvPython } else { Find-SystemPython }
if (-not $python) {
    Write-Host 'No Python found. Install Python 3.10 or newer, then run:'
    Write-Host "  powershell -ExecutionPolicy Bypass -File `"$here\launch.ps1`" -Setup"
    exit 2
}
& $python -c "import panda3d.core" 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "Panda3D is not installed for $python. Run once:"
    Write-Host "  powershell -ExecutionPolicy Bypass -File `"$here\launch.ps1`" -Setup"
    exit 2
}

$backend = $env:ANE_BACKEND
if (-not $backend) {
    foreach ($rel in @('build\road-msvc\road_driving\road_drive.exe', 'build\road-gcc\road_driving\road_drive.exe', 'build\road_driving\road_drive.exe')) {
        $p = Join-Path $repo $rel
        if (Test-Path $p) { $backend = $p; break }
    }
}
if (-not $backend -or -not (Test-Path $backend)) {
    Write-Host 'The simulation backend road_drive.exe was not found. Build it from the repository root, in a Visual Studio developer prompt:'
    Write-Host '  cmake -S . -B build\road-msvc -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_ROAD_DRIVING=ON'
    Write-Host '  cmake --build build\road-msvc --target road_drive'
    Write-Host 'or set ANE_BACKEND to its path.'
    exit 2
}

if ($Gpu -eq 'nvidia') {
    $env:SHIM_MCCOMPAT = '0x800000001'  # NVIDIA Optimus: this process's OpenGL on the discrete GPU
}
Push-Location $here
try {
    & $python -m ane_viewer --backend $backend @ViewerArgs
    exit $LASTEXITCODE
} finally {
    Pop-Location
}
