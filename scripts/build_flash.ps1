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

$BuildDirAbs = (Resolve-Path $BuildDir -ErrorAction SilentlyContinue)
if (-not $BuildDirAbs) {
  $BuildDirAbs = (Join-Path $RepoRoot $BuildDir)
  New-Item -ItemType Directory -Force -Path $BuildDirAbs | Out-Null
}
$BuildDirAbs = (Resolve-Path $BuildDirAbs).Path
Write-Host "Build dir: $BuildDirAbs"

if (-not (Test-Path $CMakeExe)) {
  throw "cmake.exe not found: $CMakeExe"
}

if (-not $env:ZEPHYR_TOOLCHAIN_VARIANT) {
  $env:ZEPHYR_TOOLCHAIN_VARIANT = "zephyr"
}
if (-not $env:ZEPHYR_SDK_INSTALL_DIR) {
  $env:ZEPHYR_SDK_INSTALL_DIR = "C:/ncs/toolchains/c1a76fddb2/opt/zephyr-sdk"
}
if (-not $env:PYTHON_EXECUTABLE) {
  $env:PYTHON_EXECUTABLE = "C:/Users/zq/AppData/Local/Programs/Python/Python312/python.exe"
}

# Auto-configure sysbuild if build.ninja is missing (e.g. after a clean) or
# if a previous configure used a different generator (e.g. NMake).
$cache = Join-Path $BuildDirAbs "CMakeCache.txt"
$buildNinja = Join-Path $BuildDirAbs "build.ninja"
$needConfigure = -not (Test-Path $buildNinja)
if (Test-Path $cache) {
  $cacheText = Get-Content -Path $cache -Raw
  if ($cacheText -match "NMake Makefiles") {
    Write-Host "Cache was generated with NMake; cleaning and re-configuring for Ninja..."
    Remove-Item -Recurse -Force $BuildDirAbs
    New-Item -ItemType Directory -Force -Path $BuildDirAbs | Out-Null
    $needConfigure = $true
  }
}

if ($needConfigure) {
  Write-Host "Configuring (Ninja generator, sysbuild)..."
  $cmakeArgs = @(
    "-G", "Ninja",
    "-B", $BuildDirAbs,
    "-S", "C:/ncs/v3.1.1/zephyr/share/sysbuild",
    "-DBOARD=nrf52dk/nrf52810",
    "-DAPP_DIR:PATH=$RepoRoot",
    "-DPython3_EXECUTABLE:FILEPATH=$env:PYTHON_EXECUTABLE",
    "-DPYTHON_EXECUTABLE:FILEPATH=$env:PYTHON_EXECUTABLE"
  )
  & $CMakeExe @cmakeArgs
}

& $CMakeExe --build $BuildDirAbs

if ($NoFlash) {
  Write-Host "Build OK (flash skipped)."
  exit 0
}

if ([string]::IsNullOrWhiteSpace($HexPath)) {
  $candidateHex = @(
    (Join-Path $BuildDirAbs "zephyr/zephyr.hex"),                 # non-sysbuild layout
    (Join-Path $BuildDirAbs "EYE_AD8221/zephyr/zephyr.hex"),      # sysbuild app image
    (Join-Path $BuildDirAbs "merged.hex")                         # sysbuild merged (app+boot)
  )
  $HexPath = $candidateHex | Where-Object { Test-Path $_ } | Select-Object -First 1
}

if (-not $HexPath) {
  throw "zephyr.hex not found in: $BuildDirAbs"
}

if (-not (Test-Path $HexPath)) {
  throw "zephyr.hex not found: $HexPath"
}

Write-Host "Flashing: $HexPath"
& nrfjprog --program $HexPath --chiperase --verify --reset
Write-Host "Flash OK."
