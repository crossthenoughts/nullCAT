// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// TestAppSettingsDialog - the desktop app's Application Settings carries
// the host settings it owns on the PC: the control loop's CPU and the
// shaker cards. What a config holds loads into the dialog and comes back
// out of it through OK (as a user saves), and an edit made in the dialog
// lands in the config as host.json stores it. The dialog is built and
// driven but never shown.
//
//   ctest -R AppSettingsDialog
// ============================================================
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QPushButton>
#include <QSpinBox>
#include <QtTest>
#include "ApplicationSettingsDialog.h"
#include "Config.h"
#include "PlatformRT.h"

class TestAppSettingsDialog : public QObject
{
    Q_OBJECT

    // Press OK as a user does: the fields go into the config, then accept.
    static AppConfig pressOk(ApplicationSettingsDialog& dlg)
    {
        for (QPushButton* b : dlg.findChildren<QPushButton*>())
            if (b->text() == "OK") { b->click(); break; }
        return dlg.getConfig();
    }
    template <class W> static W* field(ApplicationSettingsDialog& dlg, const char* name)
    {
        W* w = dlg.findChild<W*>(name);
        if (!w) qFatal("no field %s", name);
        return w;
    }

private slots:
    // Two cards (one the list does not have: typed in, kept as typed), the
    // loop unpinned on a chosen CPU: everything comes back as it went in.
    void loopCpuAndTwoCards_roundTrip()
    {
        const int cpu = std::min(2, PlatformRT::logicalCpuCount() - 1);
        AppConfig c;
        c.rtPinCore = false; c.rtCore = cpu;
        c.shakersEnabled = true;
        c.audioDevice = "null"; c.audioChannels = 2;
        c.audioDevices = { { "null", 2 }, { "A card that is not plugged in", 4 } };
        ApplicationSettingsDialog dlg(c);
        QVERIFY(dlg.result() == QDialog::Rejected);   // not yet accepted
        const AppConfig out = pressOk(dlg);
        QCOMPARE(dlg.result(), int(QDialog::Accepted));
        QCOMPARE(out.rtPinCore, false);
        QCOMPARE(out.rtCore, cpu);
        QCOMPARE(out.shakersEnabled, true);
        QCOMPARE(static_cast<int>(out.audioDevices.size()), 2);
        QCOMPARE(out.audioDevices[0].device, std::string("null"));
        QCOMPARE(out.audioDevices[0].channels, 2);
        QCOMPARE(out.audioDevices[1].device, std::string("A card that is not plugged in"));
        QCOMPARE(out.audioDevices[1].channels, 4);
        QCOMPARE(out.audioDevice, std::string("null"));   // the first card mirrored
        QCOMPARE(out.audioChannels, 2);
    }

    // An older host.json: one card in audioDevice/audioChannels, no list,
    // shakers off, the loop on auto. Saved as the page saves it: the card in
    // the list too.
    void oneCardDefaults_roundTrip()
    {
        AppConfig c;   // shakers off, audioDevice "" (the system default), auto loop CPU
        c.audioChannels = 6;
        ApplicationSettingsDialog dlg(c);
        const AppConfig out = pressOk(dlg);
        QCOMPARE(out.rtPinCore, true);
        QCOMPARE(out.rtCore, -1);
        QCOMPARE(out.shakersEnabled, false);
        QCOMPARE(static_cast<int>(out.audioDevices.size()), 1);
        QCOMPARE(out.audioDevices[0].device, std::string(""));
        QCOMPARE(out.audioDevices[0].channels, 6);
    }

    // Edits made in the dialog: shakers on, the null card picked from the
    // list, a second card ticked, the loop pinned to a CPU.
    void editsLand()
    {
        AppConfig c;
        ApplicationSettingsDialog dlg(c);
        field<QCheckBox>(dlg, "shakersEnabled")->setChecked(true);
        QComboBox* d1 = field<QComboBox>(dlg, "shakerDevice1");
        const int nullItem = d1->findData(QStringLiteral("null"));
        QVERIFY(nullItem >= 0);
        d1->setCurrentIndex(nullItem);
        field<QSpinBox>(dlg, "shakerChannels1")->setValue(3);
        field<QCheckBox>(dlg, "shaker2")->setChecked(true);
        field<QComboBox>(dlg, "shakerDevice2")->setEditText("USB Sound Device");
        field<QSpinBox>(dlg, "shakerChannels2")->setValue(2);
        field<QCheckBox>(dlg, "pinLoopCore")->setChecked(true);
        field<QSpinBox>(dlg, "loopCore")->setValue(0);
        const AppConfig out = pressOk(dlg);
        QCOMPARE(out.shakersEnabled, true);
        QCOMPARE(static_cast<int>(out.audioDevices.size()), 2);
        QCOMPARE(out.audioDevices[0].device, std::string("null"));
        QCOMPARE(out.audioDevices[0].channels, 3);
        QCOMPARE(out.audioDevices[1].device, std::string("USB Sound Device"));
        QCOMPARE(out.rtCore, 0);
    }

    // More than 8 shaker channels across the cards: OK refuses (the
    // validator would refuse the file the same way).
    void tooManyChannelsRefused()
    {
        AppConfig c;
        c.shakersEnabled = true;
        c.audioDevices = { { "null", 6 }, { "null", 4 } };
        ApplicationSettingsDialog dlg(c);
        pressOk(dlg);
        QVERIFY(dlg.result() != QDialog::Accepted);
    }
};

int main(int argc, char** argv)
{
    // The platform nullCAT.exe itself uses (deployed beside it): the dialog
    // is built and driven but never shown, so no window appears.
    QApplication app(argc, argv);
    TestAppSettingsDialog t;
    return QTest::qExec(&t, argc, argv);
}

#include "TestAppSettingsDialog.moc"
