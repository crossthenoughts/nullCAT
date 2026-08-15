// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// ============================================================
// ApplicationSettingsDialog.h
//
// The PC's native HOST editor (host/rig split): it edits host.json
// only. On save it calls Config::saveHost() - the web UI owns
// rig.json (axes + global tuning) on both platforms, so this dialog
// never touches rig config.
//
// Host sections:
//   - Network:        nicName, webPort, webBindAddr, telemetryBindAddr
//   - Runtime:        controlLoopHz (locked to 500), pdoWatchdogMs
//   - Telemetry:      telemetryPort
//   - Logging:        logFile, logToConsole, logMinLevel, diagEnabled
//   - ForegroundKeep: enabled, alpha, X, Y
//   - Advanced:       (collapsible) capability scan, DC offset,
//                     command sync cycles, WKC validation knobs,
//                     simulation mode
//
// Rig fields are edited in the web UI, NOT here: axes (per-drive),
// conditioning mode, blend time/velocity, requireUserFaultReset,
// per-axis following-error window. webUIEnabled is toggled by the
// MainWindow "Enable Web UI" button.
//
// Buffered save: edits commit to a local m_cfg copy only on OK.
// Live validation paints invalid spinboxes red and disables OK.
// ============================================================

#include "Config.h"
#include <QDialog>
#include <QLineEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QGroupBox>
#include <QLabel>
#include <QToolButton>

class ApplicationSettingsDialog : public QDialog
{
    Q_OBJECT
public:
    explicit ApplicationSettingsDialog(const AppConfig& cfg, QWidget* parent = nullptr);
    AppConfig getConfig() const;

protected:
    // Block wheel events on spinboxes/combos unless they have focus -     // prevents scrolling the dialog body from accidentally editing values.
    bool eventFilter(QObject* obj, QEvent* ev) override;

private slots:
    void onResetToDefaults();
    void onToggleAdvanced();
    void validateFields();

private:
    void buildUI();
    void loadFromConfig();
    void saveToConfig();
    void setSpinboxError(QWidget* w, bool error);
    void installWheelGuards();

    AppConfig m_cfg;            // working copy (initialized from input)
    bool      m_loading = false;

    // Network
    QLineEdit*       m_editNic           = nullptr;
    QSpinBox*        m_spinWebPort       = nullptr;
    QLineEdit*       m_editWebBindAddr   = nullptr;

    // Runtime
    QSpinBox*        m_spinControlHz     = nullptr;  // locked to 500
    QSpinBox*        m_spinPdoWatchdog   = nullptr;

    // Telemetry
    QSpinBox*        m_spinTelemetryPort    = nullptr;

    // Logging
    QLineEdit*       m_editLogFile       = nullptr;
    QCheckBox*       m_checkLogConsole   = nullptr;
    QComboBox*       m_comboLogLevel     = nullptr;
    QCheckBox*       m_checkDiagEnabled  = nullptr;

    // Foreground keeper
    QCheckBox*       m_checkFgEnabled    = nullptr;
    QSpinBox*        m_spinFgAlpha       = nullptr;
    QSpinBox*        m_spinFgX           = nullptr;
    QSpinBox*        m_spinFgY           = nullptr;

    // Advanced (collapsible)
    QToolButton*     m_btnAdvancedToggle = nullptr;
    QGroupBox*       m_groupAdvanced     = nullptr;
    QCheckBox*       m_checkSimulation   = nullptr;
    QCheckBox*       m_checkCapScan      = nullptr;
    QSpinBox*        m_spinDcSyncOffset  = nullptr;
    QSpinBox*        m_spinCmdSyncCycles = nullptr;
    QSpinBox*        m_spinWkcCycles     = nullptr;
    QDoubleSpinBox*  m_spinWkcThreshold  = nullptr;

    // Action buttons
    QPushButton*     m_btnResetDefaults  = nullptr;
    QPushButton*     m_btnOk             = nullptr;
    QPushButton*     m_btnCancel         = nullptr;
    QLabel*          m_validationLabel   = nullptr;
};
