# Starts Vite + Express from repo/application after checking Docker + Node.
# Requires Docker Desktop (or Docker Engine) running and Node.js 20+ on PATH.
# First simulation connect from the UI still builds/downloads the large Docker image.

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
Set-Location $RepoRoot

try {
    docker version | Out-Null
} catch {
    Write-Host "[ERROR] Docker CLI failed. Install Docker Desktop and start the Docker daemon." -ForegroundColor Red
    exit 1
}

if (-not (Get-Command node -ErrorAction SilentlyContinue)) {
    Write-Host "[ERROR] Node.js not found. Install Node.js 20+ and ensure it is on PATH." -ForegroundColor Red
    exit 1
}

Set-Location (Join-Path $RepoRoot "application")

if (-not (Test-Path "node_modules")) {
    Write-Host "Installing npm dependencies (first run only; may take a minute)..."
    npm install
}

Write-Host ""
Write-Host "Starting backend + frontend dev servers in a new window."
Write-Host "Opening http://localhost:5173 shortly afterward."
Write-Host "Close that window to stop the servers."
Write-Host ""

Start-Process cmd.exe -ArgumentList '/k', 'npm run dev' -WorkingDirectory (Get-Location).Path
Start-Sleep -Seconds 5
Start-Process "http://localhost:5173"
