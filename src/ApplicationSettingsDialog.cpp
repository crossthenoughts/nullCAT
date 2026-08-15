// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
// ============================================================
// ApplicationSettingsDialog.cpp
//
// See ApplicationSettingsDialog.h for section layout. This file
// implements:
//   - buildUI(): groupbox sections with form layouts, scroll
//     area for fit on smaller monitors, collapsible Advanced
//     group via a QToolButton with arrow indicator.
//   - loadFromConfig() / saveToConfig(): no per-field signal/slot
//     plumbing - fields are read-back en masse on OK.
//   - validateFields(): live red borders + disabled OK button.
//   - Reset to Defaults: replaces m_cfg with AppConfig{} but
//     preserves drives[] (the web axis editor's domain) and the
//     nicName (changing NIC on reset would be surprising).
//
// Not in this dialog: webUIEnabled (toggled from the main
// window's web button) and the 'drives' array (owned by the web
// axis editor; rig.json is web-written).
// ============================================================

#include "ApplicationSettingsDialog.h"
#include "Logging.h"
#include <QFormLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QMessageBox>
#include <QStringList>
#include <QSignalBlocker>
#include <QEvent>
#include <QWheelEvent>
#include <QAbstractSpinBox>

ApplicationSettingsDialog::ApplicationSettingsDialog(const AppConfig& cfg, QWidget* parent)
    : QDialog(parent), m_cfg(cfg)
{
    setWindowTitle("Application Settings");
    setMinimumSize(620, 700);
    buildUI();
    installWheelGuards();
    loadFromConfig();
    validateFields();
}

bool ApplicationSettingsDialog::eventFilter(QObject* obj, QEvent* ev)
{
    // Same wheel-guard as AxisConfigDialog: only react to wheel when
    // the spinbox/combo has keyboard focus, otherwise let the scroll
    // area handle it.
    if (ev->type() == QEvent::Wheel)
    {
        QWidget* w = qobject_cast<QWidget*>(obj);
        if (w && !w->hasFocus())
        {
            ev->ignore();
            return true;
        }
    }
    return QDialog::eventFilter(obj, ev);
}

void ApplicationSettingsDialog::installWheelGuards()
{
    const auto spinboxes = findChildren<QAbstractSpinBox*>();
    for (QAbstractSpinBox* s : spinboxes)
    {
        s->setFocusPolicy(Qt::StrongFocus);
        s->installEventFilter(this);
    }
    const auto combos = findChildren<QComboBox*>();
    for (QComboBox* c : combos)
    {
        c->setFocusPolicy(Qt::StrongFocus);
        c->installEventFilter(this);
    }
}

AppConfig ApplicationSettingsDialog::getConfig() const
{
    return m_cfg;
}

void ApplicationSettingsDialog::buildUI()
{
    QVBoxLayout* outer = new QVBoxLayout(this);

    QScrollArea* scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    QWidget* host = new QWidget();
    QVBoxLayout* col = new QVBoxLayout(host);

    // ---- Network ----
    QGroupBox* grpNet = new QGroupBox("Network");
    QFormLayout* fNet = new QFormLayout(grpNet);
    m_editNic = new QLineEdit();
    m_editNic->setPlaceholderText("e.g. Ethernet 2");
    m_editNic->setToolTip(
        "Windows NIC name as shown in 'ipconfig /all' (display name).\n"
        "Must be the adapter where Npcap is bound and EtherCAT is wired.");
    fNet->addRow("NIC Name:", m_editNic);
    m_spinWebPort = new QSpinBox();
    m_spinWebPort->setRange(1, 65535);
    m_spinWebPort->setToolTip("HTTP/WebSocket port for the web dashboard (when enabled in config.json).");
    fNet->addRow("Web Port:", m_spinWebPort);
    m_editWebBindAddr = new QLineEdit();
    m_editWebBindAddr->setPlaceholderText("127.0.0.1 or 0.0.0.0");
    m_editWebBindAddr->setToolTip(
        "Bind address for the web dashboard.\n"
        "127.0.0.1 = local only; 0.0.0.0 = exposed on LAN.");
    fNet->addRow("Web Bind Address:", m_editWebBindAddr);
    col->addWidget(grpNet);

    // ---- Runtime ----
    QGroupBox* grpRT = new QGroupBox("Runtime");
    QFormLayout* fRT = new QFormLayout(grpRT);
    m_spinControlHz = new QSpinBox();
    m_spinControlHz->setRange(500, 500);
    m_spinControlHz->setValue(500);
    m_spinControlHz->setSuffix(" Hz");
    m_spinControlHz->setEnabled(false);
    m_spinControlHz->setToolTip(
        "Control loop rate. Locked to 500 Hz - the only rate validated on "
        "Windows (Npcap transport). 1000 Hz is not stable on this platform.");
    fRT->addRow("Control Loop Rate:", m_spinControlHz);
    m_spinPdoWatchdog = new QSpinBox();
    m_spinPdoWatchdog->setRange(10, 5000);
    m_spinPdoWatchdog->setSuffix(" ms");
    m_spinPdoWatchdog->setToolTip(
        "Drive-side PDO watchdog timeout. The drive faults if it doesn't see "
        "a fresh PDO frame within this window.");
    fRT->addRow("PDO Watchdog:", m_spinPdoWatchdog);
    col->addWidget(grpRT);

    // ---- Telemetry ----
    QGroupBox* grpSim = new QGroupBox("Telemetry");
    QFormLayout* fSim = new QFormLayout(grpSim);
    m_spinTelemetryPort = new QSpinBox();
    m_spinTelemetryPort->setRange(1, 65535);
    m_spinTelemetryPort->setToolTip(
        "UDP port for motion telemetry input (e.g. SimHub's Generic UDP "
        "output target port).");
    fSim->addRow("Telemetry UDP Port:", m_spinTelemetryPort);
    col->addWidget(grpSim);

    // Motion blending (blendTimeSec / blendMaxVelocityMmS) is rig.global config,
    // edited in the web UI, not here (host/rig split: Qt owns host only).

    // ---- Logging ----
    QGroupBox* grpLog = new QGroupBox("Logging");
    QFormLayout* fLog = new QFormLayout(grpLog);
    m_editLogFile = new QLineEdit();
    m_editLogFile->setPlaceholderText("logs/app.log");
    m_editLogFile->setToolTip(
        "Path relative to the executable. The directory is created on demand.");
    fLog->addRow("Log File:", m_editLogFile);
    m_checkLogConsole = new QCheckBox("Mirror log to console");
    fLog->addRow("", m_checkLogConsole);
    m_comboLogLevel = new QComboBox();
    m_comboLogLevel->addItems({"debug", "info", "warning", "error", "critical"});
    m_comboLogLevel->setToolTip(
        "Minimum level for LOG_*/RT_LOG_* macros. 'debug' keeps everything; "
        "'info' suppresses LOG_DEBUG and RT_LOG_DEBUG only.");
    fLog->addRow("Min Level:", m_comboLogLevel);
    m_checkDiagEnabled = new QCheckBox("Enable DIAG (SOEM diagnostic) log");
    m_checkDiagEnabled->setToolTip(
        "Gates logDiag() entirely. Disable to test whether high-rate DIAG "
        "output (RTT samples, pump_samples, recovery_scan) contributes to "
        "RT-loop jitter.");
    fLog->addRow("", m_checkDiagEnabled);
    col->addWidget(grpLog);

    // ---- Foreground Keeper ----
    QGroupBox* grpFg = new QGroupBox("Foreground Keeper (Windows throttling defense)");
    QFormLayout* fFg = new QFormLayout(grpFg);
    m_checkFgEnabled = new QCheckBox("Enabled (recommended on Windows 11)");
    m_checkFgEnabled->setToolTip(
        "1x1 always-on-top widget that prevents Windows from background-"
        "throttling the process when the main window loses focus. Validated "
        "fix for the 500Hz RT-thread starvation observed when alt-tabbing.");
    fFg->addRow("", m_checkFgEnabled);
    m_spinFgAlpha = new QSpinBox();
    m_spinFgAlpha->setRange(1, 255);
    m_spinFgAlpha->setToolTip(
        "1-255. Lower values are imperceptible to humans but still register "
        "as 'visible' to the Windows foreground classifier. Avoid alpha=0.");
    fFg->addRow("Alpha (1-255):", m_spinFgAlpha);
    m_spinFgX = new QSpinBox();
    m_spinFgX->setRange(0, 10000);
    m_spinFgX->setSuffix(" px");
    fFg->addRow("X Position:", m_spinFgX);
    m_spinFgY = new QSpinBox();
    m_spinFgY->setRange(0, 10000);
    m_spinFgY->setSuffix(" px");
    fFg->addRow("Y Position:", m_spinFgY);
    col->addWidget(grpFg);

    // ---- Advanced (collapsible) ----
    m_btnAdvancedToggle = new QToolButton();
    m_btnAdvancedToggle->setText("Advanced settings");
    m_btnAdvancedToggle->setCheckable(true);
    m_btnAdvancedToggle->setChecked(false);
    m_btnAdvancedToggle->setArrowType(Qt::RightArrow);
    m_btnAdvancedToggle->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    col->addWidget(m_btnAdvancedToggle);

    m_groupAdvanced = new QGroupBox();
    m_groupAdvanced->setVisible(false);
    QFormLayout* fAdv = new QFormLayout(m_groupAdvanced);
    m_checkSimulation = new QCheckBox("Simulation Mode (no EtherCAT hardware required)");
    m_checkSimulation->setToolTip("Bypasses Npcap/SOEM init; useful for UI work without rig present.");
    fAdv->addRow("", m_checkSimulation);

    m_checkCapScan = new QCheckBox("Enable capability scan during init");
    m_checkCapScan->setToolTip(
        "Runs 24+ SDO reads per drive at init. Useful for first-time setup; "
        "leave off in normal operation to reduce init time and crash risk.");
    fAdv->addRow("", m_checkCapScan);

    // requireUserFaultReset is rig.global config (safety policy travels with the
    // rig) - edited in the web UI, not here.

    // followingErrorWindowMm is per-axis (rig) config - edited in the web
    // axis editor, not here.

    m_spinDcSyncOffset = new QSpinBox();
    m_spinDcSyncOffset->setRange(-1000000, 1000000);
    m_spinDcSyncOffset->setSuffix(" ns");
    m_spinDcSyncOffset->setToolTip("Distributed Clock SYNC0 shift offset. 0 = no offset.");
    fAdv->addRow("DC Sync Offset:", m_spinDcSyncOffset);

    m_spinCmdSyncCycles = new QSpinBox();
    m_spinCmdSyncCycles->setRange(0, 1000);
    m_spinCmdSyncCycles->setToolTip(
        "Cycles to hold controlword at 0x07 while syncing the target "
        "PDO to actual position before transitioning to 0x0F.");
    fAdv->addRow("Command Sync Cycles:", m_spinCmdSyncCycles);

    m_spinWkcCycles = new QSpinBox();
    m_spinWkcCycles->setRange(0, 10000);
    m_spinWkcCycles->setToolTip(
        "Post-OP WKC validation window length, in control-loop cycles.");
    fAdv->addRow("WKC Validation Cycles:", m_spinWkcCycles);

    m_spinWkcThreshold = new QDoubleSpinBox();
    m_spinWkcThreshold->setRange(0.0, 1.0);
    m_spinWkcThreshold->setDecimals(2);
    m_spinWkcThreshold->setSingleStep(0.05);
    m_spinWkcThreshold->setToolTip("Fraction of validation cycles required to have WKC==expected.");
    fAdv->addRow("WKC Validation Threshold:", m_spinWkcThreshold);

    col->addWidget(m_groupAdvanced);
    connect(m_btnAdvancedToggle, &QToolButton::clicked,
            this, &ApplicationSettingsDialog::onToggleAdvanced);

    col->addStretch();
    scroll->setWidget(host);
    outer->addWidget(scroll, /*stretch*/ 1);

    // ---- Bottom row: Reset / validation / OK / Cancel ----
    QHBoxLayout* btnRow = new QHBoxLayout();
    m_btnResetDefaults = new QPushButton("Reset to Defaults");
    m_btnResetDefaults->setToolTip(
        "Restore all fields to factory defaults.\n"
        "NIC name and per-axis settings are preserved.");
    btnRow->addWidget(m_btnResetDefaults);

    m_validationLabel = new QLabel();
    m_validationLabel->setWordWrap(true);
    m_validationLabel->setStyleSheet("color: #c0392b; font-size: 9pt;");
    btnRow->addWidget(m_validationLabel, /*stretch*/ 1);

    m_btnOk     = new QPushButton("OK");
    m_btnOk->setDefault(true);
    m_btnCancel = new QPushButton("Cancel");
    btnRow->addWidget(m_btnOk);
    btnRow->addWidget(m_btnCancel);
    outer->addLayout(btnRow);

    connect(m_btnResetDefaults, &QPushButton::clicked,
            this, &ApplicationSettingsDialog::onResetToDefaults);
    connect(m_btnOk, &QPushButton::clicked, this, [this]()
    {
        saveToConfig();
        validateFields();
        if (!m_btnOk->isEnabled()) return;
        accept();
    });
    connect(m_btnCancel, &QPushButton::clicked, this, &QDialog::reject);

    // ---- Live validation hooks ----
    auto hookDouble = [this](QDoubleSpinBox* s){
        connect(s, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, [this](double){ validateFields(); });
    };
    auto hookInt = [this](QSpinBox* s){
        connect(s, QOverload<int>::of(&QSpinBox::valueChanged),
                this, [this](int){ validateFields(); });
    };
    auto hookText = [this](QLineEdit* e){
        connect(e, &QLineEdit::textChanged,
                this, [this](const QString&){ validateFields(); });
    };
    hookInt   (m_spinWebPort);
    hookInt   (m_spinPdoWatchdog);
    hookInt   (m_spinTelemetryPort);
    hookInt   (m_spinFgAlpha);
    hookInt   (m_spinCmdSyncCycles);
    hookInt   (m_spinWkcCycles);
    hookDouble(m_spinWkcThreshold);
    hookText  (m_editWebBindAddr);
}

void ApplicationSettingsDialog::onToggleAdvanced()
{
    bool show = m_btnAdvancedToggle->isChecked();
    m_groupAdvanced->setVisible(show);
    m_btnAdvancedToggle->setArrowType(show ? Qt::DownArrow : Qt::RightArrow);
}

void ApplicationSettingsDialog::loadFromConfig()
{
    m_loading = true;

    m_editNic->setText(QString::fromStdString(m_cfg.nicName));
    m_spinWebPort->setValue(m_cfg.webPort > 0 ? m_cfg.webPort : 8080);
    m_editWebBindAddr->setText(QString::fromStdString(
        m_cfg.webBindAddr.empty() ? "127.0.0.1" : m_cfg.webBindAddr));

    m_spinControlHz->setValue(500);          // locked
    m_spinPdoWatchdog->setValue(m_cfg.pdoWatchdogMs);

    m_spinTelemetryPort->setValue(m_cfg.telemetryPort);


    m_editLogFile->setText(QString::fromStdString(m_cfg.logFile));
    m_checkLogConsole->setChecked(m_cfg.logToConsole);
    int lvlIdx = m_comboLogLevel->findText(QString::fromStdString(m_cfg.logMinLevel));
    m_comboLogLevel->setCurrentIndex(lvlIdx >= 0 ? lvlIdx : 0);
    m_checkDiagEnabled->setChecked(m_cfg.diagEnabled);

    m_checkFgEnabled->setChecked(m_cfg.foregroundKeeperEnabled);
    m_spinFgAlpha->setValue(m_cfg.foregroundKeeperAlpha);
    m_spinFgX->setValue(m_cfg.foregroundKeeperX);
    m_spinFgY->setValue(m_cfg.foregroundKeeperY);

    m_checkSimulation->setChecked(m_cfg.simulationMode);
    m_checkCapScan->setChecked(m_cfg.enableCapabilityScan);
    m_spinDcSyncOffset->setValue(m_cfg.dcSyncOffsetNs);
    m_spinCmdSyncCycles->setValue(m_cfg.commandSyncCycles);
    m_spinWkcCycles->setValue(m_cfg.wkcValidationCycles);
    m_spinWkcThreshold->setValue(m_cfg.wkcValidationThreshold);

    m_loading = false;
}

void ApplicationSettingsDialog::saveToConfig()
{
    m_cfg.nicName       = m_editNic->text().trimmed().toStdString();
    m_cfg.webPort       = m_spinWebPort->value();
    m_cfg.webBindAddr   = m_editWebBindAddr->text().trimmed().toStdString();

    // controlLoopHz locked to 500.
    m_cfg.controlLoopHz = 500;
    m_cfg.pdoWatchdogMs = m_spinPdoWatchdog->value();

    m_cfg.telemetryPort    = m_spinTelemetryPort->value();


    m_cfg.logFile      = m_editLogFile->text().trimmed().toStdString();
    m_cfg.logToConsole = m_checkLogConsole->isChecked();
    m_cfg.logMinLevel  = m_comboLogLevel->currentText().toStdString();
    m_cfg.diagEnabled  = m_checkDiagEnabled->isChecked();

    m_cfg.foregroundKeeperEnabled = m_checkFgEnabled->isChecked();
    m_cfg.foregroundKeeperAlpha   = m_spinFgAlpha->value();
    m_cfg.foregroundKeeperX       = m_spinFgX->value();
    m_cfg.foregroundKeeperY       = m_spinFgY->value();

    m_cfg.simulationMode          = m_checkSimulation->isChecked();
    m_cfg.enableCapabilityScan    = m_checkCapScan->isChecked();
    m_cfg.dcSyncOffsetNs          = m_spinDcSyncOffset->value();
    m_cfg.commandSyncCycles       = m_spinCmdSyncCycles->value();
    m_cfg.wkcValidationCycles     = m_spinWkcCycles->value();
    m_cfg.wkcValidationThreshold  = m_spinWkcThreshold->value();
}

void ApplicationSettingsDialog::onResetToDefaults()
{
    auto reply = QMessageBox::question(this, "Reset to Defaults",
        "Reset all application settings to factory defaults?\n\n"
        "NIC name and per-axis settings are preserved.\n"
        "This only takes effect when you press OK.",
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (reply != QMessageBox::Yes) return;

    std::string keepNic = m_cfg.nicName;
    std::vector<DriveConfig> keepDrives = m_cfg.drives;

    m_cfg = AppConfig{};  // defaults
    m_cfg.nicName = keepNic;
    m_cfg.drives  = keepDrives;
    m_cfg.numDrives = (int)keepDrives.size();

    loadFromConfig();
    validateFields();
    LOG_INFO("ApplicationSettingsDialog: reset to defaults (NIC + drives preserved)");
}

// ---- Live validation ---------------------------------------------------

void ApplicationSettingsDialog::setSpinboxError(QWidget* w, bool error)
{
    if (!w) return;
    if (error)
        w->setStyleSheet("border: 1px solid #c0392b; background: #fdecea;");
    else
        w->setStyleSheet(QString());
}

void ApplicationSettingsDialog::validateFields()
{
    if (m_loading || !m_btnOk) return;

    QStringList errors;

    bool webPortBad = m_spinWebPort->value() < 1 || m_spinWebPort->value() > 65535;
    setSpinboxError(m_spinWebPort, webPortBad);
    if (webPortBad) errors << "Web port must be 1-65535.";

    QString bindAddr = m_editWebBindAddr->text().trimmed();
    bool bindBad = bindAddr.isEmpty();
    setSpinboxError(m_editWebBindAddr, bindBad);
    if (bindBad) errors << "Web bind address must not be empty.";

    bool wdBad = m_spinPdoWatchdog->value() < 10;
    setSpinboxError(m_spinPdoWatchdog, wdBad);
    if (wdBad) errors << "PDO watchdog must be >= 10 ms.";

    bool shPortBad = m_spinTelemetryPort->value() < 1 || m_spinTelemetryPort->value() > 65535;
    setSpinboxError(m_spinTelemetryPort, shPortBad);
    if (shPortBad) errors << "Telemetry port must be 1-65535.";

    bool alphaBad = m_spinFgAlpha->value() < 1 || m_spinFgAlpha->value() > 255;
    setSpinboxError(m_spinFgAlpha, alphaBad);
    if (alphaBad) errors << "Foreground keeper alpha must be 1-255.";

    bool csBad = m_spinCmdSyncCycles->value() < 0;
    setSpinboxError(m_spinCmdSyncCycles, csBad);
    if (csBad) errors << "Command sync cycles must be >= 0.";

    bool wkcCyBad = m_spinWkcCycles->value() < 0;
    setSpinboxError(m_spinWkcCycles, wkcCyBad);
    if (wkcCyBad) errors << "WKC validation cycles must be >= 0.";

    bool wkcThBad = m_spinWkcThreshold->value() < 0.0
                 || m_spinWkcThreshold->value() > 1.0;
    setSpinboxError(m_spinWkcThreshold, wkcThBad);
    if (wkcThBad) errors << "WKC validation threshold must be 0.0-1.0.";

    if (errors.isEmpty())
    {
        m_validationLabel->clear();
        m_btnOk->setEnabled(true);
        m_btnOk->setToolTip(QString());
    }
    else
    {
        m_validationLabel->setText(errors.join("\n"));
        m_btnOk->setEnabled(false);
        m_btnOk->setToolTip("Fix validation errors before clicking OK.");
    }
}
