// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// Watchdog.cpp
//
// Lightweight launcher that monitors nullCAT.exe and
// relaunches it automatically on crash or restart request.
//
// Exit code conventions (must match WebServer /api/restart logic):
//   0 - clean user quit    → watchdog exits, does NOT relaunch
//   2 - restart requested  → watchdog relaunches after 500ms
//   other - crash           → watchdog relaunches after 5000ms
//
// The 5s crash delay gives Windows time to:
//   1. Release the single-instance mutex (nullCAT_SingleInstance)
//   2. Return the EtherCAT NIC/npcap handle to a clean state
//   3. Allow drives to finish their ESC state-machine reset
//
// Circuit breaker - if nullCAT crashes MAX_CRASHES times
// within CRASH_WINDOW_MS, the watchdog stops relaunching and shows a
// message box. This prevents infinite restart loops when the drive is
// in a persistently bad state (e.g. ESC unresponsive after power cycle).
// A restart request (exit code 2) does NOT count as a crash.
//
// The watchdog lives in the same directory as nullCAT.exe.
// The Windows Task Scheduler task (install-task.ps1) runs the
// watchdog with highest privileges, so neither the watchdog nor
// the main app will prompt for UAC.
// ============================================================

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <string>

static const int EXIT_CLEAN   = 0;
static const int EXIT_RESTART = 2;

// Circuit breaker - stop relaunching after this many crashes
// within CRASH_WINDOW_MS milliseconds. Restart requests (exit 2)
// are intentional and do not count toward the crash budget.
static const int   MAX_CRASHES      = 5;
static const DWORD CRASH_WINDOW_MS  = 60000;  // 1 minute

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{
    // Resolve path to nullCAT.exe (same directory as this exe)
    char selfPath[MAX_PATH] = {};
    GetModuleFileNameA(nullptr, selfPath, MAX_PATH);

    std::string dir(selfPath);
    auto sep = dir.find_last_of("\\/");
    if (sep != std::string::npos)
        dir = dir.substr(0, sep + 1);

    std::string appExe = dir + "nullCAT.exe";

    // Circular buffer of recent crash timestamps (GetTickCount values)
    DWORD crashTimes[MAX_CRASHES] = {};
    int   crashIdx   = 0;   // next write position (circular)
    int   crashTotal = 0;   // total crashes seen so far

    while (true)
    {
        STARTUPINFOA si = {};
        PROCESS_INFORMATION pi = {};
        si.cb = sizeof(si);

        if (!CreateProcessA(
                appExe.c_str(),
                nullptr,        // command line (use exe path)
                nullptr,        // process security
                nullptr,        // thread security
                FALSE,          // don't inherit handles
                0,              // creation flags
                nullptr,        // inherit environment
                dir.c_str(),    // working directory = exe directory
                &si, &pi))
        {
            MessageBoxA(nullptr,
                ("Failed to launch:\n" + appExe +
                 "\n\nError: " + std::to_string(GetLastError())).c_str(),
                "nullCAT Watchdog", MB_OK | MB_ICONERROR);
            return 1;
        }

        // Wait for the main process to exit
        WaitForSingleObject(pi.hProcess, INFINITE);

        DWORD exitCode = 1;
        GetExitCodeProcess(pi.hProcess, &exitCode);
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);

        if (exitCode == EXIT_CLEAN)
        {
            // User quit intentionally - exit watchdog too
            break;
        }

        // ---- Circuit breaker (crashes only, not intentional restarts) ----
        if (exitCode != EXIT_RESTART)
        {
            DWORD now = GetTickCount();
            crashTimes[crashIdx % MAX_CRASHES] = now;
            crashIdx++;
            crashTotal++;

            if (crashTotal >= MAX_CRASHES)
            {
                // Oldest of the last MAX_CRASHES crashes is at crashIdx % MAX_CRASHES
                // (the slot we're about to overwrite next time = the one we wrote first)
                DWORD oldest = crashTimes[crashIdx % MAX_CRASHES];
                if (now - oldest <= CRASH_WINDOW_MS)
                {
                    MessageBoxA(nullptr,
                        ("nullCAT.exe crashed " + std::to_string(MAX_CRASHES) +
                         " times within 60 seconds.\n\n"
                         "The watchdog has stopped relaunching to prevent an infinite restart loop.\n\n"
                         "Steps to recover:\n"
                         "  1. Check logs\\app.log for the crash reason\n"
                         "  2. Power cycle all drives\n"
                         "  3. Restart nullCATWatchdog.exe manually").c_str(),
                        "EtherCAT Watchdog \xe2\x80\x94 Crash Loop Detected",
                        MB_OK | MB_ICONERROR);
                    return 1;
                }
            }
        }

        // Crash or restart: wait then relaunch.
        // Short delay for explicit restart; long delay for crash so Windows
        // can release the single-instance mutex and NIC handles before we
        // try to acquire them again.
        int delayMs = (exitCode == EXIT_RESTART) ? 500 : 5000;

        // Extra 500ms grace period before relaunching regardless of reason -         // gives the OS time to fully clean up handles from the exited process.
        Sleep(500);
        Sleep(delayMs);
    }

    return 0;
}
