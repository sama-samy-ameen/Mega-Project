[CmdletBinding()]
param(
    [string]$Port = "COM16",
    [ValidateSet("atmega328", "atmega328old", "atmega168")]
    [string]$Processor = "atmega328",
    [switch]$CompileOnly
)

$ErrorActionPreference = "Stop"
$projectDir = $PSScriptRoot
$sourcePath = Join-Path $projectDir "imu.c"
$calibrationPath = Join-Path $projectDir "imu_calibration.h"
$cliCommand = Get-Command "arduino-cli" -ErrorAction SilentlyContinue
if ($cliCommand) {
    $arduinoCli = $cliCommand.Source
} else {
    $arduinoCli = Join-Path $env:LOCALAPPDATA "Programs\Arduino CLI\arduino-cli.exe"
}

if (-not (Test-Path $arduinoCli)) {
    throw "arduino-cli was not found. Open a new terminal or install Arduino CLI."
}
if (-not (Test-Path $sourcePath)) {
    throw "Sketch source not found: $sourcePath"
}
if (-not (Test-Path $calibrationPath)) {
    throw "Calibration header not found: $calibrationPath"
}

$fqbn = "arduino:avr:nano:cpu=$Processor"
$tempRoot = Join-Path ([IO.Path]::GetTempPath()) ("imu-upload-" + [guid]::NewGuid().ToString("N"))
$sketchDir = Join-Path $tempRoot "imu"
$buildDir = Join-Path $tempRoot "build"

try {
    New-Item -ItemType Directory -Path $sketchDir -Force | Out-Null
    Copy-Item $sourcePath (Join-Path $sketchDir "imu.ino")
    Copy-Item $calibrationPath $sketchDir

    Write-Host "Compiling Arduino Nano sketch..."
    & $arduinoCli compile --fqbn $fqbn --build-path $buildDir $sketchDir
    if ($LASTEXITCODE -ne 0) {
        throw "Arduino compile failed with exit code $LASTEXITCODE."
    }

    if ($CompileOnly) {
        Write-Host "Compile succeeded. Upload skipped (-CompileOnly)."
        return
    }

    Write-Host "Uploading to $Port..."
    & $arduinoCli upload --fqbn $fqbn --port $Port --input-dir $buildDir --verify
    if ($LASTEXITCODE -ne 0) {
        throw "Arduino upload failed with exit code $LASTEXITCODE."
    }

    Write-Host "Upload and verification succeeded on $Port."
} finally {
    if (Test-Path $tempRoot) {
        Remove-Item $tempRoot -Recurse -Force
    }
}
