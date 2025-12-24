param(
  [string]$BuildDir = "build/EYE_AD8221",
  [string]$CMakeExe = "C:/ncs/toolchains/c1a76fddb2/opt/bin/cmake.exe",
  [string]$HexPath = "",
  [switch]$NoFlash
)

$ErrorActionPreference = "Stop"

Push-Location (Split-Path -Parent $MyInvocation.MyCommand.Path)
Pop-Location | Out-Null

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Set-Location $RepoRoot

$BuildDirAbs = (Resolve-Path $BuildDir).Path
Write-Host "Build dir: $BuildDirAbs"

if (-not (Test-Path $CMakeExe)) {
  throw "cmake.exe not found: $CMakeExe"
}

& $CMakeExe --build $BuildDirAbs

if ($NoFlash) {
  Write-Host "Build OK (flash skipped)."
  exit 0
}

if ([string]::IsNullOrWhiteSpace($HexPath)) {
  $HexPath = Join-Path $BuildDirAbs "zephyr/zephyr.hex"
}

if (-not (Test-Path $HexPath)) {
  throw "zephyr.hex not found: $HexPath"
}

Write-Host "Flashing: $HexPath"
& nrfjprog --program $HexPath --chiperase --verify --reset
Write-Host "Flash OK."

