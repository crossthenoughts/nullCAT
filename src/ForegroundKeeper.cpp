// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// ForegroundKeeper.cpp
// ============================================================

#include "ForegroundKeeper.h"
#include "Logging.h"

#include <QPainter>
#include <algorithm>

ForegroundKeeper::ForegroundKeeper(int x, int y, int alpha)
    : QWidget(nullptr,
        Qt::FramelessWindowHint |
        Qt::WindowStaysOnTopHint |
        Qt::Tool |
        Qt::WindowDoesNotAcceptFocus)
{
    // Don't activate (steal focus) when shown -- critical for not yanking
    // focus away from whatever the user is using.
    setAttribute(Qt::WA_ShowWithoutActivating);
    // Pass clicks through so the pixel can't intercept user input.
    setAttribute(Qt::WA_TransparentForMouseEvents);

    setFixedSize(1, 1);

    // Clamp alpha to [1, 255]; 0 is intentionally disallowed because fully
    // transparent windows can be stripped from the foreground list by some
    // Windows versions, defeating the throttling defense. To fully disable
    // the keeper, use the foregroundKeeperEnabled config flag instead.
    int clampedAlpha = std::clamp(alpha, 1, 255);
    setWindowOpacity(static_cast<double>(clampedAlpha) / 255.0);

    // Position. Defaults are (0,0) = top-left, away from SimHub/DRSM's
    // typical top-right placement. Configurable via config.json.
    move(x, y);

    // show() is deferred to the caller -- caller can decide ordering
    // (typically right before MainWindow::show()).
    show();

    LOG_INFO(strf(
        "ForegroundKeeper: 1x1 widget at (%d, %d) alpha=%d -- W11 background-throttling defense active.",
        x, y, clampedAlpha));
}

void ForegroundKeeper::paintEvent(QPaintEvent* /*event*/)
{
    // Fill the single pixel with a distinctive but unobtrusive colour.
    // Red so that if it ever does become visible, it's identifiable as
    // "the ForegroundKeeper pixel" rather than a glitch. At 1x1 it's
    // imperceptible against any background.
    QPainter painter(this);
    painter.fillRect(rect(), Qt::red);
}
