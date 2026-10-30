// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
#include "MainWindow.h"
#include "AxisKind.h"
#include "Logging.h"
#include "Theme.h"

#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#endif

#include <QApplication>
#include <QCoreApplication>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QFrame>
#include <QCloseEvent>
#include <QMessageBox>
#include <QSettings>
#include <QScrollBar>
#include <QFont>
#include <QColor>
#include <QBrush>
#include <QPixmap>
#include <QDesktopServices>
#include <QUrl>
#include <QFile>
#include <QFileSystemWatcher>
#include <vector>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("nullCAT - EtherCAT Motion");
    // Narrow instrument-panel window - readouts on top, run-deck pinned to
    // the bottom via a stretch.
    setMinimumSize(370, 620);
    // Default open size, used on FIRST run only: sized so the full collapsed
    // panel (which has grown since the original 682 was chosen - Park button
    // et al.) is readable without a manual resize. Every later run restores
    // whatever size the user last had (saved in closeEvent), so a resize
    // sticks instead of being needed on every launch.
    resize(430, 780);
    {
        QSettings settings;
        const QByteArray geo = settings.value("ui/mainWindowGeometry").toByteArray();
        if (!geo.isEmpty()) restoreGeometry(geo);
    }

    buildUI();

    m_refreshTimer = new QTimer(this);
    connect(m_refreshTimer, &QTimer::timeout, this, &MainWindow::onRefreshTimer);
    m_refreshTimer->start(200);

    // Optional startup guard delaying the Initialize button. The disable +
    // countdown text + timer only run when there's actually a delay to
    // enforce; with kInitialInitDelayMs=0 (the default), updateButtonStates()
    // below enables the Initialize button immediately from loop/master state.
    if (kInitialInitDelayMs > 0)
    {
        m_btnInitEC->setEnabled(false);
        m_btnInitEC->setText(QString("Initialize available in %1s...").arg(m_initCountdownSec));

        m_countdownTimer = new QTimer(this);
        connect(m_countdownTimer, &QTimer::timeout, this, &MainWindow::onCountdownTick);
        m_countdownTimer->start(1000);
    }

    updateButtonStates();
}

MainWindow::~MainWindow() {}

void MainWindow::setComponents(
    EtherCATMaster* master,
    TelemetryInput* telemetry,
    MotionController* motion,
    ControlLoop* loop,
    Config* config,
    std::unique_ptr<ForegroundKeeper>* fgKeeper,
    WebServer* webServer)
{
    m_master = master;
    m_telemetry = telemetry;
    m_motion = motion;
    m_loop = loop;
    m_config = config;
    m_fgKeeper = fgKeeper;
    m_webServer = webServer;

    // Backend objects expose std::function callbacks rather than Qt signals.
    // All callbacks are invoked from non-UI threads; QMetaObject::invokeMethod
    // marshals them to the Qt event loop (equivalent to Qt::QueuedConnection).

    if (m_master)
    {
        m_master->setOnMasterStateChanged([this](ECState state) {
            QMetaObject::invokeMethod(this, [this, state]() {
                onMasterStateChanged(state);
            }, Qt::QueuedConnection);
        });
        m_master->setOnSlaveError([this](int idx, const std::string& msg) {
            QString qmsg = QString::fromStdString(msg);
            QMetaObject::invokeMethod(this, [this, idx, qmsg]() {
                onSlaveError(idx, qmsg);
            }, Qt::QueuedConnection);
        });
    }

    if (m_loop)
    {
        m_loop->setOnLoopStarted([this]() {
            QMetaObject::invokeMethod(this, [this]() { onLoopStarted(); }, Qt::QueuedConnection);
        });
        m_loop->setOnLoopStopped([this]() {
            QMetaObject::invokeMethod(this, [this]() { onLoopStopped(); }, Qt::QueuedConnection);
        });
        m_loop->setOnStatsUpdated([this](LoopStats stats) {
            QMetaObject::invokeMethod(this, [this, stats]() { onStatsUpdated(stats); }, Qt::QueuedConnection);
        });
        // Deliberately NOT wired: this callback fires per-drive, per-cycle from the
        // RT thread inside the status write-lock. A queued Qt invoke here would
        // heap-allocate and post to the event queue thousands of times per second
        // (loop Hz x drives), lengthening the RT critical section for a no-op
        // handler. The UI pulls status at 5 Hz in onRefreshTimer instead. Do not re-add.
        // m_loop->setOnDriveStatusUpdated(...);   // (see onDriveStatusUpdated - no-op)
        m_loop->setOnFaultLockout([this](int idx, const std::string& msg) {
            QString qmsg = QString::fromStdString(msg);
            QMetaObject::invokeMethod(this, [this, idx, qmsg]() {
                onFaultLockoutOccurred(idx, qmsg);
            }, Qt::QueuedConnection);
        });
    }

    // Logger: drain thread calls callback; marshal to UI thread for display
    Logger::instance().setLogCallback([this](const std::string& line, int level) {
        QString qline = QString::fromStdString(line);
        QMetaObject::invokeMethod(this, [this, qline, level]() {
            onNewLogLine(qline, level);
        }, Qt::QueuedConnection);
    });

    if (m_config)
    {
        buildAxisChips();
        setupConfigWatcher();
        if (m_config->get().simulationMode)
            setWindowTitle("nullCAT - EtherCAT Motion  [SIMULATION MODE]");
    }

    updateWebButtons();
}

// ============================================================================
// Look: the web console's a6 slate housing, from the application stylesheet
// (resources/theme/a6.qss, applied in main.cpp via theme::apply). Widgets here
// carry object names and dynamic properties (tone / lamp / kind / on / blink)
// for the stylesheet to select on. The only colours set in code are in rich
// text, which the stylesheet cannot reach: the wordmark and the log lines.
// ============================================================================

void MainWindow::buildUI()
{
    QWidget* central = new QWidget(this);
    central->setObjectName("central");
    setCentralWidget(central);

    QVBoxLayout* mainLayout = new QVBoxLayout(central);
    mainLayout->setSpacing(7);
    mainLayout->setContentsMargins(13, 12, 13, 12);

    // Section rule (.sec): tracked label + 1px hairline.
    auto addSecline = [&](const QString& text) {
        QWidget* w = new QWidget();
        QHBoxLayout* h = new QHBoxLayout(w);
        h->setContentsMargins(2, 4, 2, 2);
        h->setSpacing(9);
        QLabel* l = new QLabel(text);
        l->setObjectName("secLabel");
        h->addWidget(l);
        QFrame* rule = new QFrame();
        rule->setObjectName("secRule");
        rule->setFrameShape(QFrame::NoFrame);
        rule->setFixedHeight(1);
        h->addWidget(rule, 1);
        mainLayout->addWidget(w);
    };

    // ---- Header (.top): brand plate left (cat + nullCAT + ØWERKS), status right.
    QFrame* hdr = new QFrame();
    hdr->setObjectName("hdr");
    QHBoxLayout* header = new QHBoxLayout(hdr);
    header->setContentsMargins(0, 0, 8, 0);
    header->setSpacing(11);

    QFrame* brandPlate = new QFrame();
    brandPlate->setObjectName("brand");
    QHBoxLayout* bp = new QHBoxLayout(brandPlate);
    bp->setContentsMargins(8, 4, 30, 4);
    bp->setSpacing(8);
    m_catLogo = new QLabel();
    m_catLogo->setObjectName("catLogo");
    m_catLogo->setAlignment(Qt::AlignCenter);
    m_catLogo->setFixedSize(64, 42);   // wide racer art - trim the box to its real height
    m_catLogo->setToolTip("Rig status (aggregate) - the same indicator the web UI shows.");
    bp->addWidget(m_catLogo, 0, Qt::AlignVCenter);

    QVBoxLayout* brand = new QVBoxLayout();
    brand->setSpacing(2);
    brand->setContentsMargins(0, 0, 0, 0);
    // .bn b: "null" in the mono at .65, "CAT" in Archivo 900 (rich text, so
    // the two faces sit in one label exactly as the web wordmark does). The
    // mono's Medium is named by face at normal weight, as in a6.qss.
    QLabel* wordmark = new QLabel(
        "<span style=\"font-family:'JetBrains Mono Medium'; font-weight:400; font-size:15px; color:#c1c9d3;\">null</span>"
        "<span style=\"font-family:'Archivo'; font-weight:900; font-size:17px; color:#ffffff;\">CAT</span>");
    wordmark->setObjectName("wordmark");
    wordmark->setTextFormat(Qt::RichText);
    wordmark->setAlignment(Qt::AlignLeft | Qt::AlignBottom);
    brand->addWidget(wordmark);
    QLabel* brandSub = new QLabel("ØWERKS · ETHERCAT MOTION");
    brandSub->setObjectName("brandSub");
    brandSub->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    brand->addWidget(brandSub);
    bp->addLayout(brand);
    header->addWidget(brandPlate);
    header->addStretch();

    // Aggregate status readout (glass) - tone set by state in updateCatLogo.
    m_labelAgg = new QLabel("OFFLINE");
    m_labelAgg->setObjectName("aggPill");
    m_labelAgg->setAlignment(Qt::AlignCenter);
    theme::setTone(m_labelAgg, "dim");
    header->addWidget(m_labelAgg, 0, Qt::AlignVCenter);
    mainLayout->addWidget(hdr);

    // ---- AXES: one panel (.pnl) wrapping borderless rows. --------------------
    addSecline("AXES");
    QFrame* axesPanel = new QFrame();
    axesPanel->setObjectName("axesPanel");
    m_axesLayout = new QVBoxLayout(axesPanel);
    m_axesLayout->setSpacing(2);
    m_axesLayout->setContentsMargins(6, 5, 6, 5);
    mainLayout->addWidget(axesPanel);
    // rows populated in buildAxisChips() once the config is set.

    // ---- SYSTEM: the status window (.stat glass, k|v|k|v). -------------------
    addSecline("SYSTEM");
    QWidget* cluster = new QWidget();
    cluster->setObjectName("statWin");
    QGridLayout* sg = new QGridLayout(cluster);
    sg->setContentsMargins(10, 8, 10, 8);
    sg->setHorizontalSpacing(10);
    sg->setVerticalSpacing(6);
    auto cap = [](const QString& t) {
        QLabel* l = new QLabel(t);
        l->setObjectName("statCap");
        return l;
    };
    // State words read uppercase like the web's .vt (QSS has no text-transform).
    // Readouts with units stay as written: uppercasing "µs" gives Greek
    // capital Mu, which reads "MS" (milliseconds).
    auto val = [](const QString& t, const char* tone, bool upper = true) {
        QLabel* l = new QLabel(t);
        l->setObjectName("statVal");
        if (upper) { QFont f = l->font(); f.setCapitalization(QFont::AllUppercase); l->setFont(f); }
        theme::setTone(l, tone);
        return l;
    };
    sg->addWidget(cap("MASTER"), 0, 0);
    m_labelMasterState = val("None", "dim");
    sg->addWidget(m_labelMasterState, 0, 1);
    sg->addWidget(cap("SLAVES"), 0, 2);
    m_labelSlaveCount = val("0", "disp");
    sg->addWidget(m_labelSlaveCount, 0, 3);
    sg->addWidget(cap("LOOP"), 1, 0);
    m_labelLoopState = val("Stopped", "warn");
    sg->addWidget(m_labelLoopState, 1, 1);
    sg->addWidget(cap("RATE"), 1, 2);
    m_labelLoopHz = val(" - ", "disp", false);
    sg->addWidget(m_labelLoopHz, 1, 3);
    sg->addWidget(cap("JITTER"), 2, 0);
    m_labelJitter = val(" - ", "disp", false);
    sg->addWidget(m_labelJitter, 2, 1);
    sg->addWidget(cap("WKC"), 2, 2);
    m_labelWkc = val("0", "ok");
    sg->addWidget(m_labelWkc, 2, 3);
    sg->addWidget(cap("TELEMETRY"), 3, 0);
    m_labelTelemetryState = val("No data", "dim");
    sg->addWidget(m_labelTelemetryState, 3, 1);
    sg->addWidget(cap("WEB"), 3, 2);
    m_labelWeb = val("off", "dim");
    sg->addWidget(m_labelWeb, 3, 3);
    sg->setColumnStretch(1, 1);
    sg->setColumnStretch(3, 1);
    mainLayout->addWidget(cluster);

    // ---- Log fold (collapsible). Folded = just this row (no gap); expanding
    // grows the window so the log pushes the run-deck down instead of overlapping
    // it (and shrinks it back on collapse). Fixed log height keeps the delta exact.
    static constexpr int kLogHeight = 150;
    m_btnToggleLog = new QPushButton("▸  Show Log");
    m_btnToggleLog->setObjectName("logFold");
    m_btnToggleLog->setFlat(true);
    m_btnToggleLog->setCursor(Qt::PointingHandCursor);
    connect(m_btnToggleLog, &QPushButton::clicked, this, [this]() {
        m_logExpanded = !m_logExpanded;
        m_logView->setVisible(m_logExpanded);
        m_btnToggleLog->setText(m_logExpanded ? "▾  Hide Log" : "▸  Show Log");
        const int delta = kLogHeight + 7;   // fixed log height + layout spacing
        resize(width(), height() + (m_logExpanded ? delta : -delta));
    });
    mainLayout->addWidget(m_btnToggleLog);

    m_logView = new QTextEdit();
    m_logView->setObjectName("log");
    m_logView->setReadOnly(true);
    m_logView->setFixedHeight(kLogHeight);
    m_logView->setVisible(false);
    m_logExpanded = false;
    mainLayout->addWidget(m_logView);

    // ---- RUN deck: E-STOP, the two toggle keys, secondary rows. --------------
    addSecline("RUN");

    m_btnEmergency = new QPushButton("⚡  EMERGENCY STOP");
    m_btnEmergency->setObjectName("estop");
    m_btnEmergency->setMinimumHeight(50);
    connect(m_btnEmergency, &QPushButton::clicked, this, &MainWindow::onEmergencyStop);
    mainLayout->addWidget(m_btnEmergency);
    mainLayout->addSpacing(2);

    // Init/Stop EtherCAT - context-aware (onToggleEtherCAT). Its lamp lights
    // green when it is the live GO (updateButtonStates).
    m_btnInitEC = new QPushButton("Initialize EtherCAT");
    m_btnInitEC->setMinimumHeight(40);
    theme::setLamp(m_btnInitEC, "off");
    m_btnInitEC->setToolTip("Initialize: open NIC, discover slaves, enter OP.\n"
                            "Once in OP (loop stopped) the same button stops EtherCAT.");
    connect(m_btnInitEC, &QPushButton::clicked, this, &MainWindow::onToggleEtherCAT);
    mainLayout->addWidget(m_btnInitEC);

    // Start/Stop Loop - context-aware (onToggleLoop).
    m_btnStart = new QPushButton("Start Loop");
    m_btnStart->setMinimumHeight(40);
    theme::setLamp(m_btnStart, "off");
    m_btnStart->setToolTip("Start the control loop (unparks all axes).\n"
                           "While running, the same button stops it (parks, then stops).");
    connect(m_btnStart, &QPushButton::clicked, this, &MainWindow::onToggleLoop);
    mainLayout->addWidget(m_btnStart);

    // Park/Unpark - rig-level toggle. Park eases every axis to its rest position;
    // Unpark returns them to standby without a rehome. Label/enable/lamp driven
    // in updateButtonStates from the SHARED motion aggregates, so this button and
    // the web's park-toggle can never disagree about what a click will do.
    m_btnPark = new QPushButton("Park All");
    m_btnPark->setMinimumHeight(36);
    theme::setLamp(m_btnPark, "off");
    m_btnPark->setToolTip("Park all axes (ease to rest position).\n"
                          "Once parked, the same button unparks - no rehome needed.");
    connect(m_btnPark, &QPushButton::clicked, this, &MainWindow::onTogglePark);
    mainLayout->addWidget(m_btnPark);

    // Belt don/doff - rig-level toggle for torque (belt-tension) axes only; hidden
    // on rigs without them. Slack eases tension to 0 (get in/out), Tension blends
    // back in. Visibility/label/enable/lamp all driven in updateButtonStates,
    // mirroring the web btn-belts. (onToggleBelts dispatches Slack vs Tension.)
    m_btnBelts = new QPushButton("Slack Belts");
    m_btnBelts->setMinimumHeight(36);
    theme::setLamp(m_btnBelts, "off");
    m_btnBelts->setVisible(false);
    m_btnBelts->setToolTip("Belt don/doff (torque axes): slack the belts to get in/out, then "
                           "tension to blend back in. Position axes keep running.");
    connect(m_btnBelts, &QPushButton::clicked, this, &MainWindow::onToggleBelts);
    mainLayout->addWidget(m_btnBelts);

    // Secondary row 1: Reset (amber legend) · Home · Settings (small keys).
    QWidget* secRow = new QWidget();
    QHBoxLayout* sec = new QHBoxLayout(secRow);
    sec->setContentsMargins(0, 0, 0, 0);
    sec->setSpacing(6);
    m_btnResetFaults = new QPushButton("Reset Faults");
    m_btnResetFaults->setObjectName("resetFaults");
    m_btnResetFaults->setProperty("kind", "sm");
    m_btnResetFaults->setMinimumHeight(30);
    connect(m_btnResetFaults, &QPushButton::clicked, this, &MainWindow::onResetFaults);
    sec->addWidget(m_btnResetFaults);
    // Home - rarely needed; its lamp lights amber when a rehome IS required (onRefreshTimer).
    m_btnStartHoming = new QPushButton("Home");
    m_btnStartHoming->setProperty("kind", "sm");
    theme::setLamp(m_btnStartHoming, "off");
    m_btnStartHoming->setMinimumHeight(30);
    m_btnStartHoming->setToolTip(
        "Run CiA402 homing for all axes (torque-home, back off, set zero, CSP).\n"
        "Requires EtherCAT in OP and the control loop running.");
    connect(m_btnStartHoming, &QPushButton::clicked, this, &MainWindow::onStartHoming);
    sec->addWidget(m_btnStartHoming);
    m_btnAppSettings = new QPushButton("Settings…");
    m_btnAppSettings->setProperty("kind", "sm");
    m_btnAppSettings->setMinimumHeight(30);
    m_btnAppSettings->setToolTip("PC host settings (NIC, foreground keeper, timing).\n"
                                 "Only available when EtherCAT is offline.");
    connect(m_btnAppSettings, &QPushButton::clicked, this, &MainWindow::onApplicationSettings);
    sec->addWidget(m_btnAppSettings);
    mainLayout->addWidget(secRow);

    // Secondary row 2: a small web-server on/off latch + the "Web UI" key
    // that opens it in a browser (only enabled while the server is on).
    QWidget* webRow = new QWidget();
    QHBoxLayout* wr = new QHBoxLayout(webRow);
    wr->setContentsMargins(0, 0, 0, 0);
    wr->setSpacing(6);
    m_btnWebToggle = new QPushButton("○ Off");
    m_btnWebToggle->setObjectName("webToggle");   // its glyph is the lamp; latched while serving
    m_btnWebToggle->setProperty("kind", "sm");
    m_btnWebToggle->setMinimumHeight(26);
    m_btnWebToggle->setFixedWidth(64);
    m_btnWebToggle->setToolTip("Web server on/off. Axis & tuning config and full telemetry are served here.");
    connect(m_btnWebToggle, &QPushButton::clicked, this, &MainWindow::onToggleWebServer);
    wr->addWidget(m_btnWebToggle);
    m_btnOpenWeb = new QPushButton("Web UI");
    m_btnOpenWeb->setProperty("kind", "sm");
    m_btnOpenWeb->setMinimumHeight(26);
    m_btnOpenWeb->setToolTip("Open the web UI in the default browser (enabled when the server is on).");
    connect(m_btnOpenWeb, &QPushButton::clicked, this, &MainWindow::onOpenWebUI);
    wr->addWidget(m_btnOpenWeb, 1);
    mainLayout->addWidget(webRow);

    // Top-pack the content; any surplus height sits at the very bottom (no mid gaps).
    mainLayout->addStretch(1);

    statusBar()->showMessage("Ready. Configure NIC in Settings, then Initialize.");
}

void MainWindow::buildAxisChips()
{
    if (!m_config || !m_axesLayout) return;

    // Tear down any existing chips (config may have changed, e.g. sim->real).
    for (auto& c : m_axisChips)
        if (c.container) { m_axesLayout->removeWidget(c.container); c.container->deleteLater(); }
    m_axisChips.clear();

    const AppConfig& cfg = m_config->get();
    for (int i = 0; i < cfg.numDrives && i < static_cast<int>(cfg.drives.size()); ++i)
    {
        // Axis row (.axr): lamp + bold UPPERCASE name (left) + state on glass (right).
        AxisChip chip;
        chip.container = new QWidget();
        chip.container->setObjectName("axisRow");
        QHBoxLayout* row = new QHBoxLayout(chip.container);
        row->setContentsMargins(8, 6, 8, 6);
        row->setSpacing(10);

        chip.led = new QLabel();
        chip.led->setObjectName("axisLamp");
        chip.led->setFixedSize(11, 4);   // the web's .lamp
        theme::setLamp(chip.led, "off");   // unlit until first update
        row->addWidget(chip.led, 0, Qt::AlignVCenter);

        chip.name = new QLabel(QString::fromStdString(cfg.drives[i].name).toUpper());
        chip.name->setObjectName("axisName");
        chip.name->setMinimumWidth(34);
        row->addWidget(chip.name);

        row->addStretch();

        chip.state = new QLabel(" - ");
        chip.state->setObjectName("axisState");
        chip.state->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        theme::setTone(chip.state, "dim");
        row->addWidget(chip.state);

        m_axesLayout->addWidget(chip.container);
        m_axisChips.push_back(chip);
    }
}

void MainWindow::onToggleEtherCAT()
{
    // Context-aware (mirror web app.js): in OP -> Stop EtherCAT (de-init);
    // otherwise -> Initialize. updateButtonStates() only enables the Stop path
    // when the loop is stopped, so this dispatch is safe.
    if (m_master && m_master->isInitialized())
        onStopEtherCAT();
    else
        onInitializeEtherCAT();
}

void MainWindow::onStopEtherCAT()
{
    if (!m_master) return;
    int ret = QMessageBox::question(this, "Stop EtherCAT",
        "Stop EtherCAT?\n\n"
        "The vertical axes seat onto the bottom stop and the drives return to "
        "INIT (leave OP). Re-run Initialize to bring them back.",
        QMessageBox::Yes | QMessageBox::Cancel);
    if (ret != QMessageBox::Yes) return;

    LOG_INFO("GUI: Stop EtherCAT (de-init) clicked.");
    m_btnInitEC->setEnabled(false);
    m_btnInitEC->setText("Stopping…");
    QApplication::processEvents();

    // Mirror the web /api/deinit teardown: stop the loop (defensive - the button
    // is only enabled when stopped), seat the verticals onto the bottom stop and
    // de-energize ON the stop, then walk OP->INIT. seatThenStop() self-guards.
    // NO manual disableAllDrives(): de-energizing mid-stroke under DC sync
    // trips the drives' Er74.1/ErC1.2 sync faults.
    // Quiet the mailbox FIRST, while the loop still runs: an in-flight SDO
    // (temp poll) completes under the live handler and the whole teardown
    // below is mailbox-silent. Stopping it later (inside shutdown) risked an
    // op dying half-conversed once the loop had exited.
    if (m_master) m_master->stopSdoWorker();
    if (m_loop && m_loop->isRunning()) { m_loop->stop(); m_loop->waitForStop(); }
    if (m_loop)   m_loop->seatThenStop();
    if (m_master) m_master->shutdown();

    statusBar()->showMessage("EtherCAT stopped - drives returned to INIT.");
    updateButtonStates();
}

void MainWindow::onToggleLoop()
{
    if (m_loopRunning) onStopControlLoop();
    else               onStartControlLoop();
}

void MainWindow::onTogglePark()
{
    if (!m_motion) return;
    // Mirror the server's /api/park-toggle: parked -> unpark, else park.
    // m_parked is refreshed in updateButtonStates from the shared aggregates,
    // and the button is disabled mid-transition, so this can never reverse a
    // park/unpark/home already in flight.
    LOG_INFO(m_parked ? "GUI: Unpark All clicked." : "GUI: Park All clicked.");
    m_motion->enqueueCommand({ m_parked ? MotionCommand::Type::StartUnpark
                                        : MotionCommand::Type::StartPark, -1 });
}

void MainWindow::onToggleBelts()
{
    if (!m_motion) return;
    // Mirror the web: if currently slack -> tension (blend back in); else -> slack.
    // Rig-level command (axis -1); the engine scopes it to torque axes and refuses
    // TensionBelts while e-stopped. m_beltsSlack is refreshed in updateButtonStates.
    LOG_INFO(m_beltsSlack ? "GUI: Tension Belts clicked." : "GUI: Slack Belts clicked.");
    m_motion->enqueueCommand({ m_beltsSlack ? MotionCommand::Type::TensionBelts
                                            : MotionCommand::Type::SlackBelts, -1 });
}

void MainWindow::onInitializeEtherCAT()
{
    if (!m_master || !m_config)
    {
        QMessageBox::warning(this, "Error", "Components not set.");
        return;
    }

    const AppConfig& cfg = m_config->get();
    // An e-stop from here on needs an initialize of its own.
    const uint32_t estopEvents = m_motion ? m_motion->estopEventCount() : 0;

    // Validate the config before touching the NIC.
    auto errors = cfg.validate();
    if (!errors.empty())
    {
        QString errList;
        for (const auto& e : errors)
            errList += "• " + QString::fromStdString(e) + "\n";
        QMessageBox::warning(this, "Configuration Error",
            "Config validation failed:\n\n" + errList +
            "\nPlease fix config.json and retry.");
        return;
    }

    if (cfg.nicName.empty() && !cfg.simulationMode)
    {
        QMessageBox::warning(this, "Configuration Error",
            "NIC name not set in config.json.\n"
            "Please set 'nicName' to your EtherCAT NIC name\n"
            "(e.g., 'Ethernet 2') and restart the application.");
        return;
    }

    LOG_INFO("GUI: Initialize EtherCAT clicked.");
    m_btnInitEC->setEnabled(false);
    m_btnInitEC->setText("Initializing...");
    QApplication::processEvents();

    // Guard against concurrent init from web UI
    if (m_master->isInitializing())
    {
        QMessageBox::information(this, "Busy",
            "Initialization already in progress (web UI request).\nPlease wait.");
        updateButtonStates();   // restores the context-aware label/enable
        return;
    }

    // applyConfig() called at startup in main.cpp; call again here so that
    // any config reload via onConfigureAxes() is picked up before re-init.
    m_master->applyConfig(cfg);

    // ...and the same for the motion controller. A web rig.json save made while
    // EtherCAT was up is reloaded into memory by reloadConfigFromWeb(), but that
    // deliberately returns early without touching a live MotionController. Without
    // this call the reloaded axis/belt tuning only ever reached the engine via
    // main.cpp's startup configure() -- i.e. it needed an application restart,
    // contradicting the "Stop & Re-initialize to apply" the UI promises.
    // Guarded on the loop being stopped: configure() reseats every axis to parkPos
    // and clears homed/arms rehome, which must never run under a live RT thread.
    if (m_motion && !m_loopRunning) m_motion->configure(cfg);

    InitResult initResult = m_master->initializeAndEnterOp(cfg.nicName);
    if (!initResult.ok)
    {
        QString prefix = (initResult.failedSlave >= 1)
            ? QString("Slave %1: ").arg(initResult.failedSlave)
            : QString();
        QMessageBox::critical(this, "EtherCAT Error",
            QString("Initialization failed:\n%1%2\n\n"
                "Check:\n"
                "• NIC name in config.json\n"
                "• Application running as Administrator\n"
                "• Npcap/WinPcap installed\n"
                "• EtherCAT cables connected")
                .arg(prefix)
                .arg(QString::fromStdString(initResult.detail)));
        updateButtonStates();   // restores the context-aware label/enable
        return;
    }

    // The drives are re-initialized: an e-stop trip no longer holds the rig
    // (refused while the e-stop is still held).
    const bool estopHeld = m_motion && !m_motion->clearEmergencyTrip(estopEvents);
    m_labelSlaveCount->setText(QString::number(m_master->getSlaveCount()));
    statusBar()->showMessage(estopHeld
        ? "EtherCAT initialized, but the E-STOP is held or was pressed meanwhile: release it, then Stop EtherCAT and Initialize again."
        : "EtherCAT initialized and operational.");
    updateButtonStates();   // label flips to "Stop EtherCAT" once OP + loop stopped
}

void MainWindow::onStartControlLoop()
{
    if (!m_loop) return;
    // The start DIAG breadcrumb is emitted inside ControlLoop::start() so the
    // web /api/start path produces the same SOEM-log trail as this button.
    LOG_INFO("GUI: Start Control Loop clicked.");

    if (m_motion && m_motion->isEmergencyStop())
    {
        QMessageBox::warning(this, "E-STOP", m_motion->isEstopInputHeld()
            ? "E-STOP is still held. Release it, then Stop EtherCAT and Initialize EtherCAT to resume."
            : "E-STOP tripped. Stop EtherCAT, then Initialize EtherCAT to resume.");
        return;
    }

    if (!m_loop->start())
    {
        QMessageBox::warning(this, "Error",
            "Failed to start control loop.\n"
            "Make sure EtherCAT is initialized and in OP state.");
        return;
    }
    updateButtonStates();
}

void MainWindow::onStopControlLoop()
{
    if (!m_loop) return;
    LOG_INFO("GUI: Stop Control Loop clicked.");
    // Do NOT call m_motion->startPark() here -- ControlLoop::run() calls
    // startPark() itself as part of its shutdown sequence, from the RT
    // thread. Calling it here from the UI thread at the same time corrupts
    // interpolation state mid-move (double-startPark race).
    m_loop->stop();
    updateButtonStates();
}

void MainWindow::onResetFaults()
{
    if (!m_master) return;
    LOG_INFO("GUI: Reset Faults clicked.");
    m_master->requestFaultResetAll();   // the RT loop runs the reset sequence
    statusBar()->showMessage("Fault reset sequence initiated.");
}

void MainWindow::onStartHoming()
{
    if (!m_motion) return;
    int ret = QMessageBox::question(this, "Start Homing",
        "This will move all axes to their hardstop limits.\n"
        "Ensure the machine is clear before proceeding.\n\n"
        "Start homing sequence?",
        QMessageBox::Yes | QMessageBox::Cancel);
    if (ret != QMessageBox::Yes) return;

    LOG_INFO("GUI: Homing sequence started for all axes.");
    // Enqueue so startHoming() executes on the RT thread, not the UI thread.
    m_motion->enqueueCommand({MotionCommand::Type::StartHoming, -1});
    statusBar()->showMessage("Homing in progress - watch drive state column...");
}

bool MainWindow::configEditAllowed()
{
    // Gate config editors on both loop-running and EtherCAT-initialized.
    // Either state means SOEM is actively driving slaves and changing axis/app
    // settings underneath could mismatch the runtime config.
    if (m_loopRunning)
    {
        QMessageBox::warning(this, "Cannot Configure",
            "Stop the control loop before changing configuration.");
        return false;
    }
    if (m_master && m_master->isInitialized())
    {
        QMessageBox::warning(this, "Cannot Configure",
            "Shut down EtherCAT before changing configuration.\n"
            "(Re-initialize from the main window after saving.)");
        return false;
    }
    return true;
}

// Axis configuration lives entirely in the web UI (rig.json is web-owned);
// nothing in the native shell edits rig config.

void MainWindow::onApplicationSettings()
{
    if (!m_config) return;
    if (!configEditAllowed()) return;
    if (m_openAppSettingsDialog)
    {
        m_openAppSettingsDialog->raise();
        m_openAppSettingsDialog->activateWindow();
        return;
    }

    // Snapshot the prior config so we can detect edges (sim->real, FG keeper
    // re-create) after the dialog returns.
    const AppConfig oldCfg = m_config->get();

    ApplicationSettingsDialog dlg(m_config->get(), this);
    m_openAppSettingsDialog = &dlg;
    int result = dlg.exec();
    m_openAppSettingsDialog.clear();
    if (result != QDialog::Accepted) return;

    AppConfig newCfg = dlg.getConfig();
    // Race-safety: the web UI owns drives[]. Pull the *on-disk* drives[] (which
    // include any concurrent web axis edits) rather than this app's in-memory
    // copy, so saving host settings can't clobber a web tuning change.
    adoptDiskAxes(newCfg);

    // A sim -> real transition resets axes to a single 'default' axis.
    // Sim-mode configs typically have throwaway axes that don't match the
    // user's actual hardware; carrying them into hardware mode is dangerous
    // and confusing. Confirm with the user before doing it.
    const bool simToReal = (oldCfg.simulationMode && !newCfg.simulationMode);
    if (simToReal)
    {
        auto reply = QMessageBox::question(this, "Switching out of Simulation",
            "Switching out of simulation mode will RESET the axes to a single "
            "default axis. Any sim-mode axis configuration will be lost.\n\n"
            "Continue?",
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (reply != QMessageBox::Yes)
        {
            // Back out the simulationMode change; keep everything else.
            newCfg.simulationMode = true;
        }
        else
        {
            DriveConfig defaultAxis;
            defaultAxis.slaveIndex = 1;
            defaultAxis.name       = "default";
            Config::recomputeDerivedFields(defaultAxis);
            newCfg.drives.clear();
            newCfg.drives.push_back(defaultAxis);
            newCfg.numDrives = 1;
            LOG_INFO("Application settings: simulation -> hardware mode; "
                     "axes reset to single 'default' axis.");
        }
    }

    m_config->get() = newCfg;

    QString cfgPath = QCoreApplication::applicationDirPath() + "/config.json";
    if (m_config->saveHost(cfgPath.toStdString()))   // Qt owns host.json only; web owns rig.json
        LOG_INFO("Application settings saved.");
    else
        LOG_WARNING("Could not save config.json - changes active until restart.");

    // ---- Live-apply runtime-mutable fields ----
    // Mode-lock guarantees EtherCAT is offline and the control loop is
    // stopped while the dialog is open, so it's safe to swap simulation
    // mode and rebuild the foreground keeper without any cleanup races.

    Logger::instance().setMinLevel(Logger::parseLevel(newCfg.logMinLevel));
    Logger::instance().setDiagEnabled(newCfg.diagEnabled);

    if (m_master && newCfg.simulationMode != oldCfg.simulationMode)
    {
        m_master->setSimulationMode(newCfg.simulationMode);
        LOG_INFO(strf("EtherCATMaster: simulationMode now %s (live).",
                      newCfg.simulationMode ? "ON" : "OFF"));
    }

    if (m_fgKeeper)
    {
        bool fgChanged =
            (newCfg.foregroundKeeperEnabled != oldCfg.foregroundKeeperEnabled) ||
            (newCfg.foregroundKeeperAlpha   != oldCfg.foregroundKeeperAlpha)   ||
            (newCfg.foregroundKeeperX       != oldCfg.foregroundKeeperX)       ||
            (newCfg.foregroundKeeperY       != oldCfg.foregroundKeeperY);
        if (fgChanged)
        {
            m_fgKeeper->reset();   // destroy any existing keeper widget
            if (newCfg.foregroundKeeperEnabled)
            {
                *m_fgKeeper = std::make_unique<ForegroundKeeper>(
                    newCfg.foregroundKeeperX,
                    newCfg.foregroundKeeperY,
                    newCfg.foregroundKeeperAlpha);
                LOG_INFO("ForegroundKeeper: rebuilt live from settings.");
            }
            else
            {
                LOG_INFO("ForegroundKeeper: disabled live from settings. "
                         "RT thread may suffer background throttling when the "
                         "main window loses focus.");
            }
        }
    }

    // The loop's CPU: taken by the next Start Loop (the loop is stopped while
    // this dialog is open). The recovery thread keeps off it from the next
    // Initialize (EtherCATMaster::applyConfig).
    if (m_loop && (newCfg.rtCore != oldCfg.rtCore || newCfg.rtPinCore != oldCfg.rtPinCore))
    {
        m_loop->setRtCore(newCfg.rtCore, newCfg.rtPinCore);
        LOG_INFO(newCfg.rtPinCore
            ? strf("Application settings: control loop CPU now %s (from the next loop start).",
                   newCfg.rtCore < 0 ? "auto" : std::to_string(newCfg.rtCore).c_str())
            : std::string("Application settings: control loop not pinned (from the next loop start)."));
    }

    // The shaker cards open once, at start (main.cpp): a change waits for it.
    bool shakersChanged = (newCfg.shakersEnabled != oldCfg.shakersEnabled);
    {
        const auto a = oldCfg.shakerDevices(), b = newCfg.shakerDevices();
        shakersChanged = shakersChanged || a.size() != b.size();
        for (size_t i = 0; i < a.size() && i < b.size(); ++i)
            shakersChanged = shakersChanged || a[i].device != b[i].device || a[i].channels != b[i].channels;
    }
    if (shakersChanged)
        LOG_INFO("Application settings: shaker settings saved -- they apply at the next nullCAT start.");

    if (simToReal)
    {
        // drives[] changed -- rebuild the per-axis chips and re-configure
        // the motion controller so the UI and engine match the new axis set.
        buildAxisChips();
        if (m_motion) m_motion->configure(m_config->get());
        statusBar()->showMessage(
            "Switched to hardware mode. Axes reset to single 'default' axis.");
    }
    else
    {
        statusBar()->showMessage(shakersChanged
            ? "Application settings saved. Shaker settings apply at the next nullCAT start."
            : "Application settings saved (live-applied where supported).");
    }
}

// Host/rig write safety: the web UI owns drives[]. Before any Qt-side host
// save, replace cfg's drives[] with the current on-disk array so a concurrent
// web axis edit isn't overwritten by the whole-file merge save.
void MainWindow::adoptDiskAxes(AppConfig& cfg)
{
    const QString cfgPath = QCoreApplication::applicationDirPath() + "/config.json";
    Config disk;
    if (disk.load(cfgPath.toStdString()))
    {
        cfg.drives    = disk.get().drives;
        cfg.numDrives = disk.get().numDrives;
    }
    // If the file can't be read, leave cfg.drives as-is (caller's in-memory copy).
}

// ---- PC config-reload on web save -----------------------------------------
// The native PC app reads config only at startup, so a web UI rig save would
// otherwise be invisible until an app restart. Watch rig.json (the web-owned
// file) and reload in-process instead: the Pi keeps its restart-required flow;
// the PC picks the change up live (offline) or on the next re-init (online).
void MainWindow::setupConfigWatcher()
{
    m_rigPath = QCoreApplication::applicationDirPath() + "/rig.json";

    m_configReloadTimer = new QTimer(this);
    m_configReloadTimer->setSingleShot(true);
    connect(m_configReloadTimer, &QTimer::timeout, this, &MainWindow::reloadConfigFromWeb);

    m_configWatcher = new QFileSystemWatcher(this);
    if (QFile::exists(m_rigPath)) m_configWatcher->addPath(m_rigPath);
    connect(m_configWatcher, &QFileSystemWatcher::fileChanged,
            this, &MainWindow::onRigFileChanged);
}

void MainWindow::onRigFileChanged(const QString& /*path*/)
{
    // The web save is atomic (write .tmp -> rename over rig.json), which can fire
    // several change events and briefly drops the file from the watcher. Debounce,
    // then reload once; the reload re-arms the watch.
    if (m_configReloadTimer) m_configReloadTimer->start(250);
}

void MainWindow::reloadConfigFromWeb()
{
    if (!m_config) return;

    // Re-arm the watch (an atomic replace removes the path from the watcher).
    if (m_configWatcher && QFile::exists(m_rigPath) &&
        !m_configWatcher->files().contains(m_rigPath))
        m_configWatcher->addPath(m_rigPath);

    const QString cfgPath = QCoreApplication::applicationDirPath() + "/config.json";
    if (!m_config->load(cfgPath.toStdString()))
    {
        LOG_WARNING("Config file changed but reload failed - restart to apply.");
        return;
    }

    // While EtherCAT is up or the loop is running we must NOT hot-swap drive/PDO
    // config; the reload keeps memory current so a Stop -> Initialize applies it.
    const bool busy = m_loopRunning || (m_master && m_master->isInitialized());
    if (busy)
    {
        LOG_INFO("Config updated from web UI - Re-initialize EtherCAT to apply.");
        statusBar()->showMessage("Config saved from web - Stop & Re-initialize EtherCAT to apply.");
        return;
    }

    // Offline: apply immediately so it's live for the next Initialize.
    buildAxisChips();
    if (m_motion) m_motion->configure(m_config->get());
    updateButtonStates();     // belt/torque axis presence may have changed
    LOG_INFO("Config updated from web UI - applied (active on next Initialize).");
    statusBar()->showMessage("Config updated from web UI - active on next Initialize.");
}

void MainWindow::onToggleWebServer()
{
    if (!m_webServer || !m_config) return;

    const bool wantOn = !m_webServer->isRunning();
    if (wantOn)
    {
        // Refresh the operator's allowed Host-header names (host.json
        // webAllowedHosts) from the current config before a live start, so a
        // config reload since app launch is honoured without a restart.
        m_webServer->setAllowedHosts(m_config->get().webAllowedHosts);
        if (!m_webServer->start())
        {
            QMessageBox::warning(this, "Web UI",
                "Failed to start the web server (port in use?). See the log.");
            updateWebButtons();
            return;
        }
        LOG_INFO(strf("WebServer started on port %d (via Qt button).", m_webServer->port()));
    }
    else
    {
        m_webServer->stop();
        LOG_INFO("WebServer stopped (via Qt button).");
    }

    // Persist webUIEnabled (a host field) so the choice survives restart.
    // saveHost writes host.json only, so it can't touch web-owned rig.json --
    // single-writer means no adoptDiskAxes() race-guard is needed here.
    m_config->get().webUIEnabled = wantOn;
    const QString cfgPath = QCoreApplication::applicationDirPath() + "/config.json";
    if (!m_config->saveHost(cfgPath.toStdString()))
        LOG_WARNING("Could not persist webUIEnabled to host.json.");

    updateWebButtons();
}

void MainWindow::onOpenWebUI()
{
    if (!m_webServer || !m_config) return;
    if (!m_webServer->isRunning())
    {
        QMessageBox::information(this, "Web UI",
            "The web UI is not running. Click \"Enable Web UI\" first.");
        return;
    }
    // Bind addr 0.0.0.0 means "all interfaces"; browse to loopback locally.
    std::string host = m_config->get().webBindAddr;
    if (host.empty() || host == "0.0.0.0") host = "127.0.0.1";
    const QString url = QString("http://%1:%2/")
        .arg(QString::fromStdString(host))
        .arg(m_webServer->port());
    QDesktopServices::openUrl(QUrl(url));
}

void MainWindow::updateWebButtons()
{
    const bool running = (m_webServer && m_webServer->isRunning());
    if (m_btnWebToggle)
    {
        m_btnWebToggle->setText(running ? "● On" : "○ Off");
        theme::setOn(m_btnWebToggle, running);            // latched while serving
        m_btnWebToggle->setEnabled(m_webServer != nullptr);
    }
    if (m_btnOpenWeb)
        m_btnOpenWeb->setEnabled(running);

    // SYSTEM cluster WEB cell: port when running, "off" otherwise.
    if (m_labelWeb)
    {
        if (running)
        {
            m_labelWeb->setText(QString(":%1").arg(m_webServer->port()));
            theme::setTone(m_labelWeb, "acc");   // active
        }
        else
        {
            m_labelWeb->setText("off");
            theme::setTone(m_labelWeb, "dim");
        }
    }
}

void MainWindow::onEmergencyStop()
{
    LOG_WARNING("GUI: EMERGENCY STOP button pressed!");

    // The motion e-stop first (the cycle that carries out the stop then
    // disables in process() too); the bus owner disables every drive before
    // its next frame.
    if (m_motion) m_motion->setEmergencyStop(true, MotionController::EstopSource::App);
    if (m_master) m_master->requestStopAll();

    m_btnEmergency->setText("⚡ E-STOP ACTIVE - click to clear");
    theme::setOn(m_btnEmergency, true);

    disconnect(m_btnEmergency, &QPushButton::clicked, this, &MainWindow::onEmergencyStop);
    connect(m_btnEmergency, &QPushButton::clicked, this, [this]()
        {
            LOG_INFO("GUI: Emergency stop released.");
            if (m_motion) m_motion->setEmergencyStop(false, MotionController::EstopSource::App);
            if (m_motion && m_motion->isEstopTripped())
                statusBar()->showMessage("E-STOP released. The drives stay stopped: re-initialize them to resume.");

            m_btnEmergency->setText("⚡  EMERGENCY STOP");
            theme::setOn(m_btnEmergency, false);
            disconnect(m_btnEmergency, nullptr, nullptr, nullptr);
            connect(m_btnEmergency, &QPushButton::clicked, this, &MainWindow::onEmergencyStop);
            updateButtonStates();
        });

    updateButtonStates();
    statusBar()->showMessage("EMERGENCY STOP - ramping to halt.");
}

void MainWindow::onRefreshTimer()
{
    m_blinkOn = !m_blinkOn;          // drives the pulse/blink phase of the status indicators
    updateStatusIndicators();        // per-axis buckets + the aggregate cat (shared model)
    updateButtonStates();            // keep the context-aware buttons live (settling grey-out, busy text)

    if (m_telemetry && m_telemetry->hasRecentData())
    {
        m_labelTelemetryState->setText("Receiving");
        theme::setTone(m_labelTelemetryState, "ok");
    }
    else if (m_telemetry && m_telemetry->isInitialized())
    {
        m_labelTelemetryState->setText("Listening…");
        theme::setTone(m_labelTelemetryState, "warn");
    }
    else
    {
        m_labelTelemetryState->setText("Not connected");
        theme::setTone(m_labelTelemetryState, "dim");
    }

    if (m_motion)
    {
        // Read all motion state from the RT-published snapshot - no direct
        // access to m_axisState[] or m_runtime[] from the UI thread. Per-axis
        // state text is rendered by updateStatusIndicators() into the chips; here
        // we only drive the rehome/homing status-bar + Home-button highlighting.
        MotionStatus ms = m_motion->getMotionStatus();

        if (m_loopRunning && ms.needsRehome)
        {
            // Make it obvious that the user needs to press Home after a fault-park:
            // Home's lamp lights amber (the web's .key.amb.lit).
            statusBar()->showMessage("REHOME REQUIRED - press 'Home' to continue.");
            theme::setLamp(m_btnStartHoming, "warn");
        }
        else
        {
            theme::setLamp(m_btnStartHoming, "off");         // back to unlit
            bool allHomed = (ms.numDrives > 0);
            for (int i = 0; i < ms.numDrives; ++i)
                if (!ms.homed[i]) { allHomed = false; break; }
            if (allHomed && m_loopRunning)
            {
                QString current = statusBar()->currentMessage();
                if (current.contains("Homing"))
                    statusBar()->showMessage("Homing complete - axes unparking.");
            }
        }
    }
}

void MainWindow::onNewLogLine(const QString& line, int level)
{
    // The web's .log colours: errors and above red, warnings amber, the rest
    // ink2. Critical (and DIAG above it) is red in bold, so it still reads
    // apart from an error. A synthesized bold: the bundled SemiBold is too
    // close to Regular at this size to tell the two apart.
    QString color = theme::a6::ink2;
    QString weight;
    if (level >= static_cast<int>(LogLevel::LVL_CRITICAL))     { color = theme::a6::bad; weight = "font-weight:bold;"; }
    else if (level >= static_cast<int>(LogLevel::LVL_ERROR))   color = theme::a6::bad;
    else if (level >= static_cast<int>(LogLevel::LVL_WARNING)) color = theme::a6::warn;

    m_logView->append(QString("<span style='color:%1;%2'>%3</span>")
        .arg(color, weight, line.toHtmlEscaped()));

    if (++m_logLineCount > MAX_LOG_LINES)
    {
        QTextCursor cursor = m_logView->textCursor();
        cursor.movePosition(QTextCursor::Start);
        cursor.movePosition(QTextCursor::Down, QTextCursor::KeepAnchor, 50);
        cursor.removeSelectedText();
        m_logLineCount -= 50;
    }

    QScrollBar* sb = m_logView->verticalScrollBar();
    sb->setValue(sb->maximum());
}

void MainWindow::onLoopStarted()
{
    m_loopRunning = true;
    m_labelLoopState->setText("Running");
    theme::setTone(m_labelLoopState, "ok");
    statusBar()->showMessage("Control loop running - unparking axes...");
    updateButtonStates();

#ifdef _WIN32
    // Inhibit OS idle-sleep while the loop runs, so the machine can't auto-suspend
    // mid-operation and free-fall the verticals (the suspend path can't park/seat in
    // Windows' ~2s power-handler window). ES_SYSTEM_REQUIRED only -- the display may
    // still sleep. This is a power-manager flag ONLY: it does not touch thread
    // priority, affinity, MMCSS, CPU C-states/frequency or any RT timing, and runs
    // once here on the GUI thread (never on the RT path). Cleared in onLoopStopped.
    // (Does NOT block a *deliberate* user sleep -- lid/Sleep button -- only idle.)
    SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED);
#endif
}

void MainWindow::onLoopStopped()
{
    m_loopRunning = false;
    m_labelLoopState->setText("Stopped");
    theme::setTone(m_labelLoopState, "warn");
    m_labelLoopHz->setText(" - ");
    m_labelJitter->setText(" - ");
    m_labelWkc->setText("0");
    theme::setTone(m_labelWkc, "ok");
    statusBar()->showMessage("Control loop stopped.");
    updateButtonStates();

#ifdef _WIN32
    // Loop stopped -- allow OS idle-sleep again (clears the ES_SYSTEM_REQUIRED set
    // in onLoopStarted). Same thread (GUI) as the set, per ES_CONTINUOUS semantics.
    SetThreadExecutionState(ES_CONTINUOUS);
#endif
}

void MainWindow::onStatsUpdated(LoopStats stats)
{
    m_labelLoopHz->setText(QString("%1 Hz").arg(stats.loopHz, 0, 'f', 0));
    m_labelJitter->setText(QString("%1 µs").arg(stats.maxJitterUs, 0, 'f', 1));

    // WKC errors: 0 is the healthy steady state; any non-zero is worth calling out.
    m_labelWkc->setText(QString::number(stats.wkcErrors));
    theme::setTone(m_labelWkc, stats.wkcErrors > 0 ? "bad" : "ok");
}

void MainWindow::onDriveStatusUpdated(int /*index*/, DriveStatus /*status*/)
{
    // Telemetry (pos/target/statusword) lives in the web UI. The button-box
    // shows only the canonical per-axis indicators, pulled each refresh tick in
    // updateStatusIndicators(); this push callback is intentionally a no-op.
}

void MainWindow::onMasterStateChanged(ECState state)
{
    ECState prev = m_lastMasterState;
    m_lastMasterState = state;

    updateMasterStateLabel(state);
    updateButtonStates();
}

void MainWindow::onSlaveError(int slaveIndex, const QString& message)
{
    LOG_ERROR(strf("GUI: Slave %d error: %s", slaveIndex, message.toStdString().c_str()));
    statusBar()->showMessage(QString("Slave %1 error: %2").arg(slaveIndex).arg(message));
}

void MainWindow::updateMasterStateLabel(ECState state)
{
    QString text; const char* tone = "dim";
    switch (state)
    {
    case ECState::None:   text = "None";    tone = "dim";  break;
    case ECState::Init:   text = "Init";    tone = "warn"; break;
    case ECState::PreOp:  text = "PreOp";   tone = "warn"; break;
    case ECState::SafeOp: text = "SafeOp";  tone = "acc";  break;   // was blue; a6 has none
    case ECState::Op:     text = "OP";      tone = "ok";   break;
    case ECState::Error:  text = "Error";   tone = "bad";  break;
    default:              text = "Unknown"; tone = "dim";  break;
    }
    m_labelMasterState->setText(text);
    theme::setTone(m_labelMasterState, tone);
}

void MainWindow::onCountdownTick()
{
    // ---- Init countdown ----
    if (m_initCountdownSec > 0)
    {
        --m_initCountdownSec;
        if (m_initCountdownSec > 0)
            m_btnInitEC->setText(
                QString("Initialize available in %1s...").arg(m_initCountdownSec));
        else
        {
            m_btnInitEC->setText("Initialize EtherCAT");
            updateButtonStates();
        }
    }

    if (m_initCountdownSec == 0)
        m_countdownTimer->stop();
}

void MainWindow::updateButtonStates()
{
    bool ecOp     = (m_lastMasterState == ECState::Op);
    bool loopRun  = m_loopRunning;
    bool ecInit   = (m_master && m_master->isInitialized());
    bool initBusy = (m_master && m_master->isInitializing());
    bool configEditOk = !loopRun && !ecInit;

    // ---- Init/Stop EtherCAT - one context-aware button (mirror web app.js) ----
    //   busy        -> disabled, "Initializing…/Stopping…"
    //   OP && !loop -> "Stop EtherCAT" (de-init)
    //   OP && loop  -> disabled, "EtherCAT: OP" (stop the loop first)
    //   else        -> "Initialize EtherCAT"
    if (initBusy)
    {
        m_btnInitEC->setEnabled(false);
        m_btnInitEC->setText(ecOp ? "Stopping…" : "Initializing…");
    }
    else if (ecOp && !loopRun)
    {
        m_btnInitEC->setEnabled(true);
        m_btnInitEC->setText("Stop EtherCAT");
    }
    else if (ecOp)
    {
        m_btnInitEC->setEnabled(false);
        m_btnInitEC->setText("EtherCAT: OP");
    }
    else
    {
        m_btnInitEC->setEnabled(m_initCountdownSec == 0 && !loopRun);
        m_btnInitEC->setText("Initialize EtherCAT");
    }

    // ---- Start/Stop Loop - one context-aware button (mirror web app.js) ----
    // Start enabled when OP && stopped; Stop enabled when running && !settling
    // (settling = an axis still homing / unparking / blending - greys Stop like
    // the web UI so the user can't interrupt the auto-home -> unpark blend).
    bool settling = false;
    if (loopRun && m_motion)
    {
        MotionStatus ms = m_motion->getMotionStatus();
        for (int i = 0; i < ms.numDrives; ++i)
        {
            const QString n = QString::fromStdString(ms.axisStateName[i]);
            if (n.contains("homing",    Qt::CaseInsensitive) ||
                n.contains("unparking", Qt::CaseInsensitive) ||
                n.contains("blending",  Qt::CaseInsensitive))
            { settling = true; break; }
        }
    }
    if (loopRun)
    {
        m_btnStart->setText("Stop Loop");
        m_btnStart->setEnabled(!settling);
    }
    else
    {
        m_btnStart->setText("Start Loop");
        m_btnStart->setEnabled(ecOp);
    }

    m_btnStartHoming->setEnabled(ecOp && loopRun);
    m_btnResetFaults->setEnabled(ecOp && !loopRun);

    // ---- Key lamps (visual only). The live GO action's lamp is lit green;
    // everything else unlit. Only re-applied on change (this runs at 5 Hz).
    int initStyle  = (!ecOp && !initBusy && m_btnInitEC->isEnabled()) ? 1 : 0;  // Initialize is the GO
    int startStyle = (ecOp && !loopRun) ? 1 : 0;                                // Start is the GO
    if (initStyle != m_initBtnStyle)
    { m_initBtnStyle = initStyle; theme::setLamp(m_btnInitEC, initStyle ? "ok" : "off"); }
    if (startStyle != m_startBtnStyle)
    { m_startBtnStyle = startStyle; theme::setLamp(m_btnStart, startStyle ? "ok" : "off"); }

    // ---- Rig-level toggles (Park/Unpark, Belt don/doff) ----
    // Both read the SHARED StatusModel aggregates -- the same derivations the
    // web UI and the /api/*-toggle endpoints use -- so no surface can disagree
    // about what a click will do. (These replaced a local copy of the belt
    // logic here, which was free to drift from the server's.)
    const bool estop = (m_motion && m_motion->isEmergencyStop());
    const MotionStatus ms = m_motion ? m_motion->getMotionStatus() : MotionStatus{};

    const status::MotionAggregates magg =
        status::deriveMotionAggregates(ms.axisState, ms.numDrives);

    // Belt mask: belt-typed torque axes only (Stage C re-key; matches
    // WebServer::beltAxisMask so both surfaces derive identical aggregates).
    bool isBelt[MAX_DRIVES] = {};
    int  numConfigured = 0;
    if (m_config)
    {
        const std::vector<DriveConfig>& drives = m_config->get().drives;
        numConfigured = std::min(static_cast<int>(drives.size()), MAX_DRIVES);
        for (int i = 0; i < numConfigured; ++i)
        {
            const AxisCaps c = axisCaps(drives[i].axisType, drives[i].mode);
            isBelt[i] = c.beltType && c.torqueMode;
        }
    }
    const status::BeltAggregates bagg = status::deriveBeltAggregates(
        ms.axisState, ms.numDrives, isBelt, numConfigured);

    const bool hasBelts   = bagg.hasBelts;
    const bool beltsSlack = bagg.beltsSlack;
    m_beltsSlack = beltsSlack;

    // Park/Unpark: label shows the ACTION the click takes. Disabled while any
    // axis is mid-transition, matching the server's park-toggle refusal, so a
    // park/unpark/home in flight can never be reversed by a stray click.
    m_parked = magg.allParked;
    if (m_btnPark)
    {
        m_btnPark->setText(m_parked ? "Unpark All" : "Park All");
        m_btnPark->setEnabled(loopRun && !estop && !magg.transitional);
        const int ps = (m_parked && loopRun && !estop && !magg.transitional) ? 1 : 0;
        if (ps != m_parkBtnStyle)
        { m_parkBtnStyle = ps; theme::setLamp(m_btnPark, ps ? "ok" : "off"); }
    }
    if (m_btnBelts)
    {
        m_btnBelts->setVisible(hasBelts);
        if (hasBelts)
        {
            m_btnBelts->setText(beltsSlack ? "Tension Belts" : "Slack Belts");
            m_btnBelts->setEnabled(loopRun && !estop);
            const int bs = (beltsSlack && loopRun && !estop) ? 1 : 0;   // green when it will tension
            if (bs != m_beltsBtnStyle)
            { m_beltsBtnStyle = bs; theme::setLamp(m_btnBelts, bs ? "ok" : "off"); }
        }
    }

    // Config editors are locked while EtherCAT is up or the loop is running.
    // Tooltip explains why so users don't think the UI is broken. (Axis config
    // lives in the web UI; only Application Settings is Qt-side.)
    if (m_btnAppSettings) m_btnAppSettings->setEnabled(configEditOk);

    QString lockTip;
    if (loopRun)     lockTip = "Stop the control loop to edit configuration.";
    else if (ecInit) lockTip = "Shut down EtherCAT to edit configuration.";
    if (m_btnAppSettings)
        m_btnAppSettings->setToolTip(lockTip.isEmpty()
            ? "PC host settings (NIC, foreground keeper, timing)."
            : lockTip);

    // If a config dialog is open and EtherCAT just came up (e.g. via the
    // web UI or external init), force-close it to keep the runtime/UI in sync.
    if (!configEditOk)
    {
        if (m_openAppSettingsDialog) m_openAppSettingsDialog->reject();
    }
}

QString MainWindow::ecStateToString(ECState state)
{
    switch (state)
    {
    case ECState::None:   return "NONE";
    case ECState::Init:   return "INIT";
    case ECState::PreOp:  return "PREOP";
    case ECState::SafeOp: return "SAFEOP";
    case ECState::Op:     return "OP";
    case ECState::Error:  return "ERROR";
    default:              return "UNKNOWN";
    }
}

// ---- Shared status model rendering -----------------------------------------
// One status truth: MainWindow consumes the same status::deriveAxis /
// deriveAggregate / styleOf as the web UI (and any external indicator box),
// so the status surfaces cannot diverge.

// Render a canonical indicator onto an axis row: lamp + text tone through its
// SHARED styleOf() web class (theme::statusTones, the web's LAMP_OF). Honors
// the pattern: pulse/blink dim on the off-phase; offline is the unlit lamp.
static void renderIndicator(QLabel* led, QLabel* text, status::Indicator s, bool blinkOn)
{
    const status::Style& st = status::styleOf(s);
    const theme::StatusTones t = theme::statusTones(st.webClass);
    const std::string pat = st.pattern ? st.pattern : "solid";
    const QString phase = (!blinkOn && (pat == "pulse" || pat == "blink")) ? "off" : "on";
    theme::setLamp(led, t.lamp);
    theme::setTone(text, t.text);
    theme::setProp(led, "blink", phase);
    theme::setProp(text, "blink", phase);
}

void MainWindow::updateStatusIndicators()
{
    if (!m_motion) return;

    MotionStatus ms       = m_motion->getMotionStatus();
    const bool   estop    = m_motion->isEmergencyStop();
    const bool   running  = m_loopRunning;
    const bool   masterOp = (m_lastMasterState == ECState::Op);

    std::vector<status::AxisIndicator> inds;
    inds.reserve(ms.numDrives);

    for (int i = 0; i < ms.numDrives && i < static_cast<int>(m_axisChips.size()); ++i)
    {
        DriveStatus ds = m_loop ? m_loop->getDriveStatus(i) : DriveStatus{};
        // Mirror the web's deriveAxis inputs exactly: statusword is only trusted in
        // OP (hasSw); motion state/name/electrical state are only meaningful while
        // the loop runs (else PARKED / empty / Unknown -> offline rendering).
        const bool            hasSw    = masterOp;
        const uint16_t        sw       = hasSw ? ds.statusword : 0;
        const AxisMotionState mst      = running ? ms.axisState[i] : AxisMotionState::PARKED;
        const DriveState      rawDrive = running ? ds.state : DriveState::Unknown;
        status::AxisIndicator ind = status::deriveAxis(
            i, mst, running ? ms.axisStateName[i] : std::string(),
            hasSw, sw, rawDrive, running, estop);
        inds.push_back(ind);

        AxisChip& chip = m_axisChips[i];
        renderIndicator(chip.led, chip.state, ind.state, m_blinkOn);
        chip.state->setText(QString::fromStdString(ind.text));
    }

    status::Indicator agg = status::deriveAggregate(
        inds.data(), static_cast<int>(inds.size()), running, estop);
    updateCatLogo(agg);
}

void MainWindow::updateCatLogo(status::Indicator agg)
{
    if (static_cast<int>(agg) == m_lastAgg) return;   // only reload the pixmap on change
    m_lastAgg = static_cast<int>(agg);

    // Aggregate is the 4-level web-logo set: ESTOP > FAULT > RUNNING > OFFLINE.
    QString logo, label;
    switch (agg)
    {
        case status::Indicator::ESTOP:   logo = "estop";   label = "E-STOP";  break;
        case status::Indicator::FAULT:   logo = "fault";   label = "FAULT";   break;
        case status::Indicator::RUNNING: logo = "online";  label = "ONLINE";  break;
        default:                         logo = "offline"; label = "OFFLINE"; break;  // OFFLINE / IDLE / BUSY
    }

    if (m_labelAgg)
    {
        // Aggregate readout - text tone through the shared styleOf() web class,
        // as the axis rows (ESTOP/FAULT bad, ONLINE ok, OFFLINE dim).
        m_labelAgg->setText(label);
        theme::setTone(m_labelAgg, theme::statusTones(status::styleOf(agg).webClass).text);
    }

    if (m_catLogo)
    {
        const QString path = QCoreApplication::applicationDirPath() + "/web/logo-" + logo + ".png";
        QPixmap pm(path);
        if (!pm.isNull())
            m_catLogo->setPixmap(pm.scaled(64, 42, Qt::KeepAspectRatio, Qt::SmoothTransformation));
        else
            m_catLogo->setText(label);                // graceful fallback if logos aren't deployed
    }
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    int ret = QMessageBox::question(this, "Quit",
        "Stop control loop and shut down EtherCAT before exiting?",
        QMessageBox::Yes | QMessageBox::Cancel);
    if (ret != QMessageBox::Yes) { event->ignore(); return; }

    // Persist the window size/position so the next launch opens where the
    // user left it (restored in the constructor). Saved only on a confirmed
    // quit -- a cancelled close changes nothing.
    {
        QSettings settings;
        settings.setValue("ui/mainWindowGeometry", saveGeometry());
    }

    // Clear all callbacks before stopping -- prevents stale UI updates
    // from the control loop or logger after the window is destroyed.
    Logger::instance().setLogCallback(nullptr);
    if (m_master) { m_master->setOnMasterStateChanged(nullptr); m_master->setOnSlaveError(nullptr); }
    if (m_loop)   { m_loop->setOnLoopStarted(nullptr); m_loop->setOnLoopStopped(nullptr);
                    m_loop->setOnStatsUpdated(nullptr); m_loop->setOnDriveStatusUpdated(nullptr);
                    m_loop->setOnError(nullptr); m_loop->setOnFaultLockout(nullptr); }

    // Stop timers
    if (m_refreshTimer)      { m_refreshTimer->stop();      m_refreshTimer->disconnect(); }
    if (m_countdownTimer)    { m_countdownTimer->stop();    m_countdownTimer->disconnect(); }
    if (m_configReloadTimer) { m_configReloadTimer->stop(); m_configReloadTimer->disconnect(); }
    if (m_configWatcher)     { m_configWatcher->disconnect(); }

    // Quiet the mailbox FIRST, while the loop still runs (mirrors the web
    // deinit): an in-flight SDO completes under the live handler instead of
    // dying half-conversed after the loop exits.
    if (m_master) m_master->stopSdoWorker();

    // Stop the loop first (axes park; drives stay energized in OP under the
    // background pump). The seat is a SEPARATE step below (seatThenStop), not a
    // stop() flag -- so the loop must be stopped before it.
    if (m_loop && m_loop->isRunning())
    {
        m_loop->stop();
        m_loop->waitForStop();
    }

    // Drain any remaining queued events now that thread is stopped
    QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

    if (m_telemetry) m_telemetry->shutdown();
    // Seat the vertical axes onto the bottom stop and de-energize ON the stop
    // (homing-based seat), then tear down -- so the OP->INIT teardown doesn't
    // free-fall them (a small drop/thunk). NO manual disableAllDrives(): the
    // seat de-energizes on the stop and shutdown() walks OP->INIT with the pump
    // still feeding SYNC0; a manual disable + gap before shutdown trips the
    // drives' Er74.1 / ErC1.2 DC-sync faults. Mirrors the web /api/deinit path.
    // seatThenStop() self-guards (no-ops if the loop is running or not in OP).
    if (m_loop)   m_loop->seatThenStop();
    if (m_master) m_master->shutdown();

    event->accept();
}

// ============================================================
// Windows sleep/resume handler.
//
// WM_POWERBROADCAST / PBT_APMSUSPEND fires before the system
// enters sleep or hibernate. We stop the control loop and shut
// down the EtherCAT master here so the drives don't hold their
// last commanded position while the NIC disappears. The SOEM
// context is destroyed; a fresh init is required on resume.
//
// PBT_APMRESUMEAUTOMATIC fires when the system comes back. We
// log a warning - the user must press Initialize + Start again.
// We do NOT attempt automatic reconnection because the NIC pcap
// handle is stale and the drive ESC state machines have reset.
// ============================================================
bool MainWindow::nativeEvent(const QByteArray& eventType, void* message, qintptr* result)
{
#ifdef _WIN32
    Q_UNUSED(eventType)
    Q_UNUSED(result)
    MSG* msg = static_cast<MSG*>(message);
    if (msg->message == WM_POWERBROADCAST)
    {
        if (msg->wParam == PBT_APMSUSPEND)
        {
            LOG_WARNING("MainWindow: System suspend detected -- stopping control loop and shutting down EtherCAT.");
            // NOTE: we deliberately do NOT seat-to-stop here (unlike app close).
            // The seat takes ~3-4s and Windows gives a power handler only a brief
            // window before forcing suspend -- blocking that long risks an
            // unclean teardown. Suspend is an abnormal teardown; the verticals may
            // drop. (Normal de-energize seats via closeEvent / web deinit.)
            // Quiet the mailbox first regardless -- same in-flight-SDO hygiene
            // as the normal deinit paths, and it is cheap.
            if (m_master) m_master->stopSdoWorker();
            if (m_loop && m_loop->isRunning())
                m_loop->stop();
            if (m_master && m_master->isInitialized())
                m_master->shutdown();
        }
        else if (msg->wParam == PBT_APMRESUMEAUTOMATIC)
        {
            LOG_WARNING("MainWindow: System resumed from sleep. "
                        "EtherCAT NOT restarted automatically -- press Initialize, then Start.");
        }
    }
#endif
    return false;  // let Qt continue processing the event
}

void MainWindow::onFaultLockoutOccurred(int driveIndex, QString message)
{
    // Called (via QueuedConnection) when a drive hits MAX_FAULT_RETRIES
    // and requireUserFaultReset=true in config. Give the user a clear choice:
    // retry from scratch, or leave the drive disabled until they fix the hardware.
    QString prompt = QString(
        "Drive %1 fault lockout:\n%2\n\n"
        "Click 'Clear Lockout' to reset retry counters and attempt recovery.\n"
        "Click 'Leave Locked' to keep the drive disabled (requires loop restart).")
        .arg(driveIndex).arg(message);

    QMessageBox box(this);
    box.setWindowTitle("Drive Fault Lockout");
    box.setText(prompt);
    box.setIcon(QMessageBox::Critical);
    QPushButton* clearBtn = box.addButton("Clear Lockout", QMessageBox::AcceptRole);
    box.addButton("Leave Locked", QMessageBox::RejectRole);
    box.exec();

    if (box.clickedButton() == clearBtn && m_loop)
    {
        LOG_INFO(strf("GUI: User cleared fault lockout for drive %d.", driveIndex));
        m_loop->clearFaultLockout(driveIndex);
    }
}
