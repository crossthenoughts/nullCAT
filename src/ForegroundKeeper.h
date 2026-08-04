// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// ForegroundKeeper.h
//
// A 1x1 topmost frameless tool window that keeps the process
// classified as "foreground-active" by Windows even when the user
// has switched focus to another app. Windows 11 applies aggressive
// background-throttling (Power Throttling v2, EcoQoS, DWM-aware
// scheduling) to processes that don't own a visible foreground
// window, regardless of HIGH_PRIORITY_CLASS / Pro Audio MMCSS /
// PROCESS_POWER_THROTTLING settings. The invisible-window trick
// (also used by SimHub, DRSM and similar real-time sim tooling)
// works around this by maintaining a tiny visible top-level window
// that satisfies the foreground-classifier without disturbing the
// user.
//
// Implementation notes:
//   * QWidget top-level (NOT child of MainWindow -- must survive
//     MainWindow minimisation / hide)
//   * Qt::Tool keeps it out of the taskbar + alt-tab list
//   * Qt::WindowDoesNotAcceptFocus prevents focus theft on show
//   * Qt::WA_TransparentForMouseEvents lets clicks pass through
//   * setWindowOpacity() controls visibility:
//       alpha=255: fully visible (recommended -- DRSM proven recipe)
//       alpha=1:   barely visible -- "is it there?" experiment
//       alpha=0:   fully transparent -- risks defeating the defense,
//                  some Windows versions strip from foreground list
//
// Position is configurable. Default (0,0) = top-left corner, which
// avoids the typical top-right placement used by SimHub / DRSM so
// the visible pixel doesn't visually stack with theirs. Multiple
// processes with topmost 1x1 windows don't conflict at the OS
// scheduling layer -- the foreground classifier evaluates each
// process independently.
//
// To disable for diagnostic comparison: set
//   "foregroundKeeperEnabled": false  in config.json
// ============================================================

#include <QWidget>

class ForegroundKeeper : public QWidget
{
    Q_OBJECT
public:
    // alpha: 1-255 (clamped). Lower = less visible. 255 = fully opaque.
    // x, y:  top-left pixel position on primary screen.
    ForegroundKeeper(int x = 0, int y = 0, int alpha = 255);

protected:
    void paintEvent(QPaintEvent* event) override;
};
