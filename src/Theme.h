// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// Theme.h - the desktop window's look: the web console's a6 slate
// housing (web/app.css tokens). One application stylesheet
// (resources/theme/a6.qss) on Fusion with an a6 palette and the
// console's fonts (resources/fonts, embedded via resources/nullcat.qrc).
// Visual only: nothing here changes what a widget does.
//
// Widgets carry object names and dynamic properties the stylesheet
// selects on: tone (a value's colour), lamp (a key's or an axis row's
// lamp), on (a latched key, a tripped E-STOP), kind (a small key), blink
// (an axis row's off-phase), invalid (a field that fails validation). The
// setters re-polish only when the value changes; the UI refresh runs at 5 Hz.
// ============================================================
#pragma once
#include <QString>
#include <QStyle>
#include <QVariant>
#include <QWidget>

class QApplication;

namespace theme {

// The a6 tokens code needs for rich text, which the stylesheet cannot reach
// (the rest live in a6.qss), verbatim from web/app.css.
namespace a6 {
constexpr const char* ink2 = "#c8d2dd";
constexpr const char* warn = "#ffb02e";
constexpr const char* bad  = "#ff4a3a";
}

// Fonts, style, palette and stylesheet. Call once, right after the
// QApplication is constructed and before any window or message box.
void apply(QApplication& app);

inline void setProp(QWidget* w, const char* name, const QString& value)
{
    if (!w || w->property(name).toString() == value) return;
    w->setProperty(name, value);
    w->style()->unpolish(w);
    w->style()->polish(w);
    w->update();
}
inline void setTone(QWidget* w, const char* tone) { setProp(w, "tone", QString::fromLatin1(tone)); }   // ok|warn|bad|acc|disp|dim
inline void setLamp(QWidget* w, const char* lamp) { setProp(w, "lamp", QString::fromLatin1(lamp)); }   // off|ok|warn (keys), +bad|acc (axis lamps)
inline void setOn(QWidget* w, bool on) { setProp(w, "on", on ? QStringLiteral("true") : QStringLiteral("false")); }

// An axis row's lamp and text tones for a shared-model web class: the same
// mapping as web/app.js LAMP_OF (s-op ok, s-home warn, s-fault and s-estop
// bad, s-park acc, s-off unlit), the text as the console's glass shows it
// (.vt: neutral = disp, unlit = dim).
struct StatusTones { const char* lamp; const char* text; };
StatusTones statusTones(const char* webClass);

} // namespace theme
