@echo off
REM Starts Vite + Express from repo/application after checking Docker + Node.
REM Requires Docker Desktop (or Docker Engine) running and Node.js 20+ on PATH.
REM First simulation connect from the UI still builds/downloads the large Docker image.
setlocal EnableDelayedExpansion

cd /d "%~dp0..\.."

docker version >nul 2>&1
if errorlevel 1 (
  echo [ERROR] Docker CLI failed. Install Docker Desktop and start the Docker daemon.
  exit /b 1
)

where node >nul 2>&1
if errorlevel 1 (
  echo [ERROR] Node.js not found. Install Node.js 20+ and ensure `node` is on PATH.
  exit /b 1
)

pushd application

if not exist "node_modules\" (
  echo Installing npm dependencies ^(first run only; may take a minute^)...
  call npm install
  if errorlevel 1 (
    echo [ERROR] npm install failed.
    popd
    exit /b 1
  )
)

echo.
echo Starting backend + frontend dev servers in a new window.
echo Opening http://localhost:5173 shortly afterward.
echo Close that window to stop the servers.
echo.

start "drone-control npm run dev" cmd /k "npm run dev"
timeout /t 5 /nobreak >nul
start "" "http://localhost:5173"

popd
exit /b 0
