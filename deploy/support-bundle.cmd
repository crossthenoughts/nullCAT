@echo off
rem SPDX-FileCopyrightText: 2026 Tim Palmgren (O Werks) <tim@zerowerks.co.nz>
rem SPDX-License-Identifier: GPL-3.0-or-later
rem
rem Double-click to put the logs, your settings and a short system summary
rem into one zip next to nullCAT.exe (see support-bundle.ps1). Windows does
rem not run PowerShell scripts by default; this runs that one script only.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0support-bundle.ps1"
echo.
pause
