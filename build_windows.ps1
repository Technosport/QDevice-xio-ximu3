$ErrorActionPreference = "Stop"

$RootDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$Ximu3Dir = Resolve-Path (Join-Path $RootDir "..\x-IMU3-Software")
$CargoToml = Join-Path $Ximu3Dir "x-IMU3-API\Rust\Cargo.toml"

Write-Host "[1/4] Building x-IMU3 Rust static library (ximu3.lib)..."
cargo build --manifest-path $CargoToml --release

Write-Host "[2/4] Configuring CMake for Windows (MSVC x64)..."
cmake -S $RootDir -B (Join-Path $RootDir "build") -A x64

Write-Host "[3/4] Building qdevice_ximu3.dll and unit tests..."
cmake --build (Join-Path $RootDir "build") --config Release

Write-Host "[4/4] Running unit tests and QDevice Validator..."
ctest --test-dir (Join-Path $RootDir "build") -C Release --output-on-failure

$DllPath = Join-Path $RootDir "build\Release\qdevice_ximu3.dll"
$ValidatorExe = Join-Path $RootDir "third_party\qdevice_sdk\validator\qdevice_validator.exe"

Write-Host ""
Write-Host "Build complete: $DllPath"
Write-Host "To validate with hardware connected, run:"
Write-Host "  `"$ValidatorExe`" --lib `"$DllPath`""
