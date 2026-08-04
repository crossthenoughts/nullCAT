// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestB222M.cpp  (Build 222M)
//
// Unit tests for B222M ForegroundKeeper.
//
// The actual OS-level effect (Windows scheduling / power throttling
// behaviour) can only be verified on hardware. What we can verify in
// unit tests:
//   M-1   ForegroundKeeper constructs without crashing, with the right
//         window flags (frameless, topmost, tool, no-focus).
//   M-2   The widget is 1x1 pixel.
//   M-3   Window opacity reflects the alpha argument (clamped to 1-255).
//   M-4   Widget destruction is clean.
// ============================================================

#include <QtTest>
#include <QApplication>
#include <memory>

#include "../src/ForegroundKeeper.h"

class TestB222M : public QObject
{
    Q_OBJECT

private slots:

    void m1_construct_with_correct_flags()
    {
        ForegroundKeeper fg(0, 0, 255);
        Qt::WindowFlags f = fg.windowFlags();
        QVERIFY2(f & Qt::FramelessWindowHint, "Expected FramelessWindowHint");
        QVERIFY2(f & Qt::WindowStaysOnTopHint, "Expected WindowStaysOnTopHint");
        QVERIFY2(f & Qt::Tool,                 "Expected Tool flag (keeps out of taskbar/alt-tab)");
        QVERIFY2(f & Qt::WindowDoesNotAcceptFocus, "Expected WindowDoesNotAcceptFocus");
        QVERIFY(fg.testAttribute(Qt::WA_ShowWithoutActivating));
        QVERIFY(fg.testAttribute(Qt::WA_TransparentForMouseEvents));
    }

    void m2_size_is_one_pixel()
    {
        ForegroundKeeper fg(0, 0, 255);
        QCOMPARE(fg.width(),  1);
        QCOMPARE(fg.height(), 1);
        QCOMPARE(fg.minimumSize(), QSize(1, 1));
        QCOMPARE(fg.maximumSize(), QSize(1, 1));
    }

    void m3_alpha_maps_to_opacity()
    {
        // 255 -> 1.0
        {
            ForegroundKeeper fg(0, 0, 255);
            QVERIFY2(qAbs(fg.windowOpacity() - 1.0) < 0.005,
                qPrintable(QString("alpha=255 -> opacity=%1, expected ~1.0").arg(fg.windowOpacity())));
        }
        // 1 -> ~0.0039
        {
            ForegroundKeeper fg(0, 0, 1);
            QVERIFY2(qAbs(fg.windowOpacity() - (1.0/255.0)) < 0.005,
                qPrintable(QString("alpha=1 -> opacity=%1, expected ~%2")
                    .arg(fg.windowOpacity()).arg(1.0/255.0)));
        }
        // Clamping: alpha=0 should clamp to 1 (never fully transparent)
        {
            ForegroundKeeper fg(0, 0, 0);
            QVERIFY2(fg.windowOpacity() > 0.0,
                qPrintable(QString("alpha=0 should clamp up to >0, got opacity=%1").arg(fg.windowOpacity())));
        }
        // Clamping: alpha=500 should clamp to 255
        {
            ForegroundKeeper fg(0, 0, 500);
            QVERIFY2(qAbs(fg.windowOpacity() - 1.0) < 0.005,
                qPrintable(QString("alpha=500 should clamp to opacity=1.0, got %1").arg(fg.windowOpacity())));
        }
    }

    void m4_position_honoured()
    {
        // Note: QWidget::move() with hidden widgets buffers the position
        // until show() lays out the platform window. The widget's geometry
        // reports the requested position after construction completes.
        ForegroundKeeper fg(42, 137, 255);
        QCOMPARE(fg.x(), 42);
        QCOMPARE(fg.y(), 137);
    }

    void m5_destruction_is_clean()
    {
        // Construct/destruct in a tight loop. No leaks, no asserts.
        for (int i = 0; i < 5; ++i)
        {
            std::unique_ptr<ForegroundKeeper> fg(new ForegroundKeeper(0, 0, 255));
            fg.reset();  // explicit destruction before the loop iteration ends
        }
        QVERIFY(true);  // reaching here means none of the constructions/destructions crashed
    }
};

QTEST_MAIN(TestB222M)
#include "TestB222M.moc"
