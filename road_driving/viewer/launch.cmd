@echo off
rem Launches the live 3D viewer (see launch.ps1). First time: launch.cmd -Setup
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0launch.ps1" %*
