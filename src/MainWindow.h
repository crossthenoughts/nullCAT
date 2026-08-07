// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "EtherCATMaster.h"
#include "TelemetryInput.h"
#include "MotionController.h"
#include "ControlLoop.h"
#include "Config.h"
#include "ApplicationSettingsDialog.h"
#include "ForegroundKeeper.h"
#include "WebServer.h"
#include "StatusModel.h"   // shared canonical status model (deriveAxis/deriveAggregate/styleOf)

#include <memory>
#include <vector>
#include <QMainWindow>
#include <QPointer>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QTimer>
#include <QGroupBox>
#include <QStatusBar>
#include <QFileSystemWatcher>

class QVBoxLayout;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

    void setComponents(
        EtherCATMaster* master,
        TelemetryInput* telemetry,
        MotionController* motion,
        ControlLoop* loop,
        Config* config,
        std::unique_ptr<ForegroundKeeper>* fgKeeper = nullptr,
        WebServer* webServer = nullptr
    );

public slots:
    void onNewLogLine(const QString& line, int level);
    void onLoopStarted();
    void onLoopStopped();
    void onStatsUpdated(LoopStats stats);
    void onDriveStatusUpdated(int index, DriveStatus status);
    void onMasterStateChanged(ECState state);
    void onSlaveError(int slaveIndex, const QString& message);
    void onFaultLockoutOccurred(int driveIndex, QString message);

private slots:
    void onToggleEtherCAT();    // context-aware: Initialize when off, Stop EtherCAT when in OP
    void onInitializeEtherCAT();
    void onStopEtherCAT();      // de-init (mirror web /api/deinit): seat + OP->INIT teardown
    void onToggleLoop();        // context-aware: Start when stopped, Stop when running
    void onTogglePark();        // context-aware: Park All when running, Unpark All when parked
    void onToggleBelts();       // torque-axis don/doff: Slack <-> Tension (belt rigs only)
    void onStartControlLoop();
    void onStopControlLoop();
    void onResetFaults();
    void onEmergencyStop();
    void onApplicationSettings();
    void onToggleWebServer();   // start/stop the web UI live + persist webUIEnabled
    void onOpenWebUI();         // launch the default browser at the web UI URL
    void onRefreshTimer();
    void onStartHoming();
    void onCountdownTick();
    void onRigFileChanged(const QString& path);  // web wrote rig.json -> reload (debounced)

protected:
    void closeEvent(QCloseEvent* event) override;
    bool nativeEvent(const QByteArray& eventType, void* message, qintptr* result) override;

private:
    EtherCATMaster* m_master = nullptr;
    TelemetryInput* m_telemetry = nullptr;
    MotionController* m_motion = nullptr;
    ControlLoop* m_loop = nullptr;
    Config* m_config = nullptr;
    // Pointer into main()'s scope-owned fgKeeper so Application Settings
    // can rebuild it live without an app restart.
    std::unique_ptr<ForegroundKeeper>* m_fgKeeper = nullptr;
    WebServer* m_webServer = nullptr;

    void buildUI();
    void updateMasterStateLabel(ECState state);
    void updateButtonStates();
    void updateWebButtons();    // reflect web-server running state on the two web buttons
    // Host/rig write safety: the web UI owns drives[]. Pull the on-disk
    // drives[] into cfg before any Qt-side host save so a concurrent web
    // axis edit can't be clobbered by a whole-file write.
    void adoptDiskAxes(AppConfig& cfg);
    // Gate config-editing entry points on whether EtherCAT is initialized
    // or the control loop is running. Returns true if the editor was
    // allowed to open.
    bool configEditAllowed();
    void buildAxisChips();   // (re)build the per-axis LED chips from config
    static QString ecStateToString(ECState state);

    // PC config-reload: on the native PC build the app reads config only at
    // startup, so a web UI save (rig.json) would otherwise need an app restart.
    // Watch rig.json (the web-owned file) and reload live — apply immediately
    // when EtherCAT is offline, or flag "re-initialize to apply" when it's up.
    void setupConfigWatcher();
    void reloadConfigFromWeb();

    // Shared status model rendering.
    void updateStatusIndicators();                 // per-axis buckets + the aggregate cat
    void updateCatLogo(status::Indicator agg);     // swap the summary cat logo on aggregate change

    // Status labels
    QLabel* m_labelMasterState = nullptr;
    QLabel* m_labelSlaveCount = nullptr;
    QLabel* m_labelLoopState = nullptr;
    QLabel* m_labelLoopHz = nullptr;     // loop rate (Hz)
    QLabel* m_labelJitter = nullptr;     // max cycle jitter (µs)
    QLabel* m_labelWkc = nullptr;        // working-counter error count
    QLabel* m_labelTelemetryState = nullptr;
    QLabel* m_labelWeb = nullptr;        // web server port / off (SYSTEM cluster)

    // Cached primary/neutral style of the two run buttons, so
    // updateButtonStates only re-applies the stylesheet on an actual change.
    int m_initBtnStyle  = -1;            // -1 unset, 0 neutral, 1 primary-green
    int m_startBtnStyle = -1;
    bool m_beltsSlack   = false;         // last-computed belt aggregate (all torque axes PARKED)
    int  m_beltsBtnStyle = -1;           // belt button style cache (-1/0/1)
    bool m_parked       = false;         // last-computed rig aggregate (every axis PARKED)
    int  m_parkBtnStyle = -1;            // park button style cache (-1/0/1)

    // Shared status model rendering
    QLabel* m_catLogo  = nullptr;   // the stateful cat (aggregate summary, web logos)
    QLabel* m_labelAgg = nullptr;   // text under the cat (ONLINE / FAULT / E-STOP / OFFLINE)
    bool    m_blinkOn  = true;      // pulse/blink phase, toggled each refresh tick
    int     m_lastAgg  = -1;        // last aggregate Indicator rendered (avoid pixmap reload)

    // Per-axis status chips (LED dot + name + state). The compact button-box
    // shows only these canonical indicators; full telemetry
    // (pos/target/statusword) lives in the web UI.
    struct AxisChip {
        QWidget* container = nullptr;
        QLabel*  led       = nullptr;   // colour dot in the canonical bucket colour
        QLabel*  name      = nullptr;
        QLabel*  state     = nullptr;   // verbatim state text from deriveAxis
    };
    std::vector<AxisChip> m_axisChips;
    QVBoxLayout*          m_axesLayout = nullptr;   // host layout, for rebuilds

    // Buttons. m_btnInitEC and m_btnStart are context-aware (double-duty):
    // Init<->Stop EtherCAT and Start<->Stop Loop, label/enable set in updateButtonStates.
    QPushButton* m_btnInitEC = nullptr;
    QPushButton* m_btnStart = nullptr;
    QPushButton* m_btnPark = nullptr;          // Park All <-> Unpark All (context-aware)
    QPushButton* m_btnBelts = nullptr;         // belt don/doff toggle (torque rigs only; hidden otherwise)
    QPushButton* m_btnStartHoming = nullptr;   // sub-button (Home)
    QPushButton* m_btnResetFaults = nullptr;
    QPushButton* m_btnAppSettings = nullptr;   // sub-button (Settings…)
    QPushButton* m_btnWebToggle = nullptr;     // sub-button (Web UI on/off)
    QPushButton* m_btnOpenWeb = nullptr;       // sub-button (Open browser)
    // Track the open settings dialog so it can be force-closed if
    // EtherCAT init starts from another path (web UI, scripted, etc).
    QPointer<ApplicationSettingsDialog> m_openAppSettingsDialog;
    QPushButton* m_btnEmergency = nullptr;
    QPushButton* m_btnToggleLog = nullptr;
    QGroupBox* m_logGroup = nullptr;
    bool         m_logExpanded = true;

    // Log
    QTextEdit* m_logView = nullptr;
    static constexpr int MAX_LOG_LINES = 500;
    int        m_logLineCount = 0;

    // Startup guard before the Initialize button enables (ms); 0 = enabled
    // immediately. If the first init attempt after app launch is flaky on a
    // machine (Npcap/NIC settle time), set to 15000 to restore the guard.
    static constexpr int kInitialInitDelayMs = 0;

    QTimer* m_refreshTimer   = nullptr;
    QTimer* m_countdownTimer = nullptr;

    // PC config-reload watcher (web save -> reload; see setupConfigWatcher).
    QFileSystemWatcher* m_configWatcher     = nullptr;
    QTimer*             m_configReloadTimer = nullptr;   // debounces a save's change-event burst
    QString             m_rigPath;                       // the rig.json path being watched

    int  m_initCountdownSec = kInitialInitDelayMs / 1000;

    ECState m_lastMasterState = ECState::None;
    bool    m_loopRunning     = false;
};