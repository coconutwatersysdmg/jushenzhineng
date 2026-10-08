# Build runtime_mecheye: Python 3.11 embed + MechEyeAPI (main runtime is 3.12).
param(
    [switch]$Rebuild,
    [switch]$NoPause
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root

$RuntimeDir = Join-Path $Root "runtime_mecheye"
$PyVersion = "3.11.9"
$PyTag = "311"
$CacheDir = Join-Path $Root "tools\_cache"
$EmbedName = "python-$PyVersion-embed-amd64.zip"
$EmbedZip = Join-Path $CacheDir $EmbedName
$EmbedUrl = "https://www.python.org/ftp/python/$PyVersion/$EmbedName"
$GetPip = Join-Path $CacheDir "get-pip.py"
$GetPipUrl = "https://bootstrap.pypa.io/get-pip.py"
$MinEmbedBytes = 8000000

Write-Host "========================================"
Write-Host " Build runtime_mecheye (Python $PyVersion)"
Write-Host "========================================"
Write-Host "Project: $Root"

function Test-MechEyePython {
    param([string]$Exe)
    if (-not (Test-Path $Exe)) { return $false }
    & $Exe -c "import sys; raise SystemExit(0 if sys.version_info[:2]==(3,11) else 1)" 2>$null
    return ($LASTEXITCODE -eq 0)
}

$needBuild = $Rebuild.IsPresent
$pyExe = Join-Path $RuntimeDir "python.exe"
if (-not (Test-MechEyePython $pyExe)) {
    $needBuild = $true
}

if ($needBuild) {
    if (Test-Path $RuntimeDir) {
        Write-Host "[INFO] Removing old runtime_mecheye ..."
        Remove-Item -LiteralPath $RuntimeDir -Recurse -Force
    }
    New-Item -ItemType Directory -Path $CacheDir -Force | Out-Null

    if (-not (Test-Path $EmbedZip) -or ((Get-Item $EmbedZip).Length -lt $MinEmbedBytes)) {
        Write-Host "[INFO] Downloading $EmbedUrl"
        Invoke-WebRequest -Uri $EmbedUrl -OutFile $EmbedZip -UseBasicParsing
    }
    if ((Get-Item $EmbedZip).Length -lt $MinEmbedBytes) {
        throw "Embed zip too small: $EmbedZip"
    }

    New-Item -ItemType Directory -Path $RuntimeDir -Force | Out-Null
    Write-Host "[INFO] Extracting to runtime_mecheye ..."
    Expand-Archive -LiteralPath $EmbedZip -DestinationPath $RuntimeDir -Force

    $pth = Join-Path $RuntimeDir "python$PyTag._pth"
    @"
python$PyTag.zip
.
..
Lib\site-packages
import site
"@ | Set-Content -LiteralPath $pth -Encoding ascii

    if (-not (Test-Path $GetPip)) {
        Write-Host "[INFO] Downloading get-pip.py"
        Invoke-WebRequest -Uri $GetPipUrl -OutFile $GetPip -UseBasicParsing
    }
    Write-Host "[INFO] Installing pip ..."
    & $pyExe $GetPip --no-warn-script-location
    if ($LASTEXITCODE -ne 0) { throw "get-pip failed" }
} else {
    Write-Host "[SKIP] Keep existing runtime_mecheye, refresh packages only."
}

Write-Host "[INFO] pip install MechEyeAPI==2.6.0 opencv-python numpy ..."
& $pyExe -m pip install --upgrade pip
& $pyExe -m pip install "MechEyeAPI==2.6.0" opencv-python numpy
if ($LASTEXITCODE -ne 0) { throw "pip install failed" }

Write-Host "[INFO] Self-check import mecheye ..."
& $pyExe -c "from mecheye.area_scan_3d_camera import Camera; print('OK', Camera)"
if ($LASTEXITCODE -ne 0) { throw "mecheye import failed" }

Write-Host ""
Write-Host "[OK] runtime_mecheye ready: $pyExe"
Write-Host "Main app will call this sidecar for Mech-Eye capture (no Viewer needed)."
if (-not $NoPause) { Read-Host "Press Enter to exit" }
