// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// Theme.cpp - see Theme.h.
// ============================================================
#include "Theme.h"

#include <QApplication>
#include <QColor>
#include <QDir>
#include <QFile>
#include <QFont>
#include <QFontDatabase>
#include <QPalette>
#include <QProxyStyle>
#include <QPushButton>
#include <QStyleFactory>
#include <QStyleHints>
#include <cstring>

namespace theme {

namespace {

QColor c(const char* hex) { return QColor(QString::fromLatin1(hex)); }

// Fusion takes a full palette and a stylesheet without drawing shapes of
// its own (the windows11 style does). One addition the stylesheet cannot
// express: key legends in capitals, the web's text-transform. A
// per-class application font loses the capitalization once the stylesheet
// resolves fonts; a font set on the widget itself keeps it.
class A6Style : public QProxyStyle
{
public:
    A6Style() : QProxyStyle(QStyleFactory::create("Fusion")) {}
    using QProxyStyle::polish;
    void polish(QWidget* w) override
    {
        QProxyStyle::polish(w);
        if (qobject_cast<QPushButton*>(w) && w->font().capitalization() != QFont::AllUppercase)
        {
            QFont f = w->font();
            f.setCapitalization(QFont::AllUppercase);
            w->setFont(f);
        }
    }
};

// What the stylesheet does not paint: message boxes, menus, selections,
// placeholders, focus. Window is the panel colour (dialogs sit on it; the
// main window paints desk through the stylesheet).
QPalette a6Palette()
{
    QPalette p;
    auto set = [&p](QPalette::ColorRole r, const char* hex) {
        p.setColor(QPalette::Active, r, c(hex));
        p.setColor(QPalette::Inactive, r, c(hex));
    };
    set(QPalette::Window,          "#36465a");   // pnl
    set(QPalette::WindowText,      "#f1f4f7");   // ink
    set(QPalette::Base,            "#1f2934");   // well
    set(QPalette::AlternateBase,   "#304056");   // pnl2
    set(QPalette::Text,            "#f1f4f7");
    set(QPalette::PlaceholderText, "#6f8297");   // faint
    set(QPalette::Button,          "#2a313a");   // keyTop
    set(QPalette::ButtonText,      "#e9edf1");   // keyInk
    set(QPalette::BrightText,      "#ffffff");
    set(QPalette::Highlight,       "#22b9a0");   // acc
    set(QPalette::HighlightedText, "#08201b");
    set(QPalette::Link,            "#22b9a0");
    set(QPalette::LinkVisited,     "#22b9a0");
    set(QPalette::ToolTipBase,     "#304056");
    set(QPalette::ToolTipText,     "#f1f4f7");
    set(QPalette::Light,           "#4a5c71");   // edge2
    set(QPalette::Midlight,        "#3e4f64");
    set(QPalette::Mid,             "#2b3847");   // hdr
    set(QPalette::Dark,            "#1c2631");   // edge
    set(QPalette::Shadow,          "#0d1218");
#if QT_VERSION >= QT_VERSION_CHECK(6, 6, 0)
    set(QPalette::Accent,          "#22b9a0");
#endif
    p.setColor(QPalette::Disabled, QPalette::WindowText, c("#6f8297"));
    p.setColor(QPalette::Disabled, QPalette::Text,       c("#6f8297"));
    p.setColor(QPalette::Disabled, QPalette::ButtonText, c("#6f8297"));
    p.setColor(QPalette::Disabled, QPalette::Base,       c("#28323e"));
    p.setColor(QPalette::Disabled, QPalette::Button,     c("#2f3a48"));
    p.setColor(QPalette::Disabled, QPalette::Window,     c("#36465a"));
    p.setColor(QPalette::Disabled, QPalette::Highlight,  c("#3d5f63"));
    return p;
}

} // namespace

void apply(QApplication& app)
{
    // TTF only: Qt's font loader cannot read the web's woff2 files. A face
    // that fails to load falls back to the system font; nothing breaks, but
    // it says so.
    const QDir fonts(":/fonts");
    for (const QString& f : fonts.entryList({ "*.ttf" }, QDir::Files))
        if (QFontDatabase::addApplicationFont(fonts.filePath(f)) < 0)
            qWarning("theme: font %s did not load; the system font stands in", qPrintable(f));

    app.setStyle(new A6Style);
#if QT_VERSION >= QT_VERSION_CHECK(6, 8, 0)
    app.styleHints()->setColorScheme(Qt::ColorScheme::Dark);   // dark Windows title bars
#endif
    app.setPalette(a6Palette());

    QFont body("Barlow");
    body.setPixelSize(14);
    body.setWeight(QFont::Medium);
    QApplication::setFont(body);

    QFile qss(":/theme/a6.qss");
    if (qss.open(QIODevice::ReadOnly | QIODevice::Text))
        app.setStyleSheet(QString::fromUtf8(qss.readAll()));
}

StatusTones statusTones(const char* webClass)
{
    const char* k = webClass ? webClass : "s-off";
    if (!std::strcmp(k, "s-op"))    return { "ok",   "ok"   };
    if (!std::strcmp(k, "s-home"))  return { "warn", "warn" };
    if (!std::strcmp(k, "s-fault") ||
        !std::strcmp(k, "s-estop")) return { "bad",  "bad"  };
    if (!std::strcmp(k, "s-park"))  return { "acc",  "disp" };
    return { "off", "dim" };                                   // s-off
}

} // namespace theme
