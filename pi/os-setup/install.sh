#!/usr/bin/env bash
#
# nullCAT Pi installer - stock Raspberry Pi OS Lite (64-bit) to tuned
# EtherCAT motion controller in one run and one reboot.
#
# What it does (all idempotent; safe to re-run):
#   1. Installs build tools and the PREEMPT_RT kernel package
#   2. Applies the RT boot tuning (core isolation on 2,3; IRQs on 0,1)
#   3. Applies the headless config.txt block (audio/BT/camera/HDMI off)
#   4. Disables unneeded services (Bluetooth, ModemManager, triggerhappy)
#   5. Creates the network profiles: EtherCAT NIC with no IP; optional
#      static point-to-point PC link on a second (USB) NIC
#   6. Grants RT scheduling limits and gpio group to the installing user
#   7. Builds SOEM (pinned) and nullCAT from this repo checkout
#   8. Installs the systemd service, IRQ-affinity pinning, poweroff
#      sudoers rule, and the nullcat.local mDNS advert
#
# Usage (from a fresh Raspberry Pi OS Lite 64-bit, as your normal user):
#   git clone https://github.com/crossthenoughts/nullCAT.git
#   cd nullCAT
#   ./pi/os-setup/install.sh
#   sudo reboot
#
# Read SAFETY.md before connecting drives. See Docs/PI_SETUP.md for the
# full walkthrough including flashing and first-boot verification.

set -euo pipefail

# ----------------------------------------------------------------------------
# Configuration - edit if your setup differs
# ----------------------------------------------------------------------------
SOEM_REPO="https://github.com/OpenEtherCATsociety/SOEM.git"
SOEM_COMMIT="b410bf6ef599d5c85302ea45cae5f55f8e9aa394"   # pinned, verified build
SOEM_DIR="${HOME}/SOEM"

ECAT_IFACE="eth0"           # dedicated EtherCAT bus (onboard port, no IP)
PC_LINK_IFACE="eth1"        # optional USB NIC: point-to-point link to the PC
PC_LINK_ADDR="192.168.50.1/24"

RT_USER="${SUDO_USER:-$USER}"
CONFIG_TXT="/boot/firmware/config.txt"
CMDLINE_TXT="/boot/firmware/cmdline.txt"

# RT boot tuning: cores 2,3 isolated for the control loop (it pins core 3),
# hard IRQs steered to housekeeping cores 0,1, threaded IRQ handlers enabled
# so they can be pinned too (see the IRQ-affinity service below).
RT_CMDLINE="isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3 irqaffinity=0,1 threadirqs"

MARKER="# >>> nullCAT headless tuning >>>"
MARKER_END="# <<< nullCAT headless tuning <<<"

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

log()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
ok()   { printf '    \033[1;32m+ %s\033[0m\n' "$*"; }
skip() { printf '    \033[1;33m. %s\033[0m\n' "$*"; }
fail() { printf '    \033[1;31mx %s\033[0m\n' "$*" >&2; exit 1; }

# ----------------------------------------------------------------------------
# 0. Preflight
# ----------------------------------------------------------------------------
log "Preflight"
if [ "$(id -u)" -eq 0 ]; then
    echo "Run as your normal user, not root -- the script calls sudo where needed." >&2
    exit 1
fi
if [ "$(uname -m)" != "aarch64" ]; then
    echo "This needs 64-bit Raspberry Pi OS (aarch64). Reflash with the 64-bit Lite image." >&2
    exit 1
fi
FREE_GB=$(df --output=avail -BG / | tail -1 | tr -dc '0-9')
if [ "${FREE_GB}" -lt 5 ]; then
    echo "Less than 5 GB free on / -- use a 16 GB (or larger) SD card." >&2
    exit 1
fi
# Parallel build jobs: roughly one per GB of RAM, capped at core count.
# Round to the nearest GB: a "2 GB" board reports ~1.9 GB and must not
# truncate down to -j1.
MEM_GB=$(awk '/MemTotal/ {printf "%.0f", $2/1048576}' /proc/meminfo)
JOBS=$(( MEM_GB < 1 ? 1 : MEM_GB ))
CORES=$(nproc)
[ "${JOBS}" -gt "${CORES}" ] && JOBS="${CORES}"
ok "aarch64, ${FREE_GB}GB free, building with -j${JOBS} (${MEM_GB}GB RAM, ${CORES} cores)"

# ----------------------------------------------------------------------------
# 1. Packages: toolchain + PREEMPT_RT kernel
# ----------------------------------------------------------------------------
log "Installing packages (toolchain + RT kernel)"
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config ethtool libcap2-bin curl \
    qt6-base-dev libgpiod-dev linux-image-rpi-v8-rt
# linux-image-rpi-v8-rt: the stock Raspberry Pi PREEMPT_RT kernel. The
# bootloader prefers it once installed; verify after reboot with uname -r.
# qt6-base-dev: build-time only (Config JSON + the Qt Test suites).
# libgpiod-dev: optional GPIO control panel; build omits it if absent.
ok "Packages present (RT kernel installs alongside the stock kernel)"

# ----------------------------------------------------------------------------
# 2. RT boot tuning (cmdline.txt -- single line file)
# ----------------------------------------------------------------------------
log "RT boot tuning"
if ! grep -q "isolcpus=" "${CMDLINE_TXT}"; then
    sudo cp "${CMDLINE_TXT}" "${CMDLINE_TXT}.nullcat.bak"
    sudo sed -i "1 s/\$/ ${RT_CMDLINE}/" "${CMDLINE_TXT}"
    ok "Appended: ${RT_CMDLINE}"
else
    skip "cmdline.txt already carries isolcpus tuning"
fi

# ----------------------------------------------------------------------------
# 3. Headless config.txt block
# ----------------------------------------------------------------------------
log "Headless config.txt cleanup"
if ! sudo grep -q "${MARKER}" "${CONFIG_TXT}"; then
    sudo cp "${CONFIG_TXT}" "${CONFIG_TXT}.nullcat.bak"
    sudo sed -i 's/^dtoverlay=vc4-kms-v3d/#dtoverlay=vc4-kms-v3d/' "${CONFIG_TXT}"
    sudo tee -a "${CONFIG_TXT}" >/dev/null <<EOF

${MARKER}
# Headless RT controller: trim jitter sources and unused hardware.
# Wi-Fi is intentionally LEFT ON (wlan0 is the admin/SSH path).
# NOTE: config.txt has NO inline-comment support for dtparam/dtoverlay --
# keep every comment on its own line.
# disable onboard audio
dtparam=audio=off
# disable Bluetooth
dtoverlay=disable-bt
# no camera
camera_auto_detect=0
# no HDMI hotplug probing
display_auto_detect=0
# minimal GPU memory split
gpu_mem=16
${MARKER_END}
EOF
    ok "Applied headless block (backup: ${CONFIG_TXT}.nullcat.bak)"
else
    skip "config.txt headless block"
fi

# Select the RT kernel at boot. Installing linux-image-rpi-v8-rt drops
# kernel8_rt.img into /boot/firmware, but the firmware keeps booting the
# stock kernel8.img unless config.txt names the RT image explicitly.
log "RT kernel selection"
RT_KERNEL_IMG="kernel8_rt.img"
if [[ ! -f "/boot/firmware/${RT_KERNEL_IMG}" ]]; then
    fail "/boot/firmware/${RT_KERNEL_IMG} not found -- the RT kernel package did not install correctly"
fi
if sudo grep -q "^kernel=${RT_KERNEL_IMG}" "${CONFIG_TXT}"; then
    skip "config.txt already boots ${RT_KERNEL_IMG}"
elif sudo grep -q "^kernel=" "${CONFIG_TXT}"; then
    fail "config.txt already has a kernel= line pointing elsewhere -- resolve it manually before re-running"
else
    sudo tee -a "${CONFIG_TXT}" >/dev/null <<EOF

# nullCAT: boot the PREEMPT_RT kernel (linux-image-rpi-v8-rt)
kernel=${RT_KERNEL_IMG}
EOF
    ok "config.txt now boots ${RT_KERNEL_IMG}"
fi

# ----------------------------------------------------------------------------
# 4. Disable unused background services (Wi-Fi and ssh are NOT touched)
# ----------------------------------------------------------------------------
log "Disabling unused services"
for svc in bluetooth hciuart triggerhappy ModemManager; do
    if systemctl list-unit-files "${svc}.service" --no-legend 2>/dev/null | grep -q "${svc}"; then
        sudo systemctl disable --now "${svc}.service" >/dev/null 2>&1 || true
        ok "Disabled ${svc}"
    fi
done

# ----------------------------------------------------------------------------
# 5. Network profiles (NetworkManager)
#    eth0 = EtherCAT bus: link up, no IP, no DHCP chatter
#    eth1 = optional static point-to-point link to the PC (never default route)
#    wlan0 untouched (admin / SSH path)
# ----------------------------------------------------------------------------
log "Network profiles"
existing="$(nmcli -t -f NAME connection show 2>/dev/null || true)"

if ! grep -qx "ecat-bus" <<<"${existing}"; then
    sudo nmcli connection add type ethernet ifname "${ECAT_IFACE}" con-name ecat-bus \
        ipv4.method disabled ipv6.method disabled connection.autoconnect yes >/dev/null
    ok "Created ecat-bus (${ECAT_IFACE} = no IP)"
else
    skip "ecat-bus"
fi

if ! grep -qx "ecat-pc-link" <<<"${existing}"; then
    sudo nmcli connection add type ethernet ifname "${PC_LINK_IFACE}" con-name ecat-pc-link \
        ipv4.method manual ipv4.addresses "${PC_LINK_ADDR}" \
        ipv6.method disabled ipv4.never-default yes connection.autoconnect yes >/dev/null
    ok "Created ecat-pc-link (${PC_LINK_IFACE} = ${PC_LINK_ADDR}; harmless if no second NIC)"
else
    skip "ecat-pc-link"
fi

# Keep the PC-link NIC's static IP with no cable plugged in, so the UDP
# socket can bind at boot regardless of PC state.
IGNORE_CARRIER_CONF="/etc/NetworkManager/conf.d/10-ecat-ignore-carrier.conf"
if [ ! -f "${IGNORE_CARRIER_CONF}" ]; then
    sudo tee "${IGNORE_CARRIER_CONF}" >/dev/null <<EOF
# nullCAT: ${PC_LINK_IFACE} holds its static IP with no carrier present.
[device-ecat-pc-link]
match-device=interface-name:${PC_LINK_IFACE}
ignore-carrier=yes
EOF
    sudo nmcli general reload
    ok "PC-link ignores carrier"
else
    skip "ignore-carrier drop-in"
fi

# ----------------------------------------------------------------------------
# 6. RT limits (interactive shells) + gpio group
#    The systemd unit carries its own Limit* lines; this covers running the
#    binary by hand while commissioning.
# ----------------------------------------------------------------------------
log "RT limits + gpio group for '${RT_USER}'"
sudo tee /etc/security/limits.d/99-nullcat-realtime.conf >/dev/null <<EOF
# nullCAT real-time control thread privileges (interactive shells)
${RT_USER}   -   rtprio    99
${RT_USER}   -   memlock   unlimited
EOF
if getent group gpio >/dev/null 2>&1; then
    sudo usermod -aG gpio "${RT_USER}"
fi
ok "Limits + gpio group set (take effect on next login)"

# ----------------------------------------------------------------------------
# 7. SOEM -- clone, pin, build
# ----------------------------------------------------------------------------
log "SOEM (EtherCAT master library, pinned)"
if [ ! -d "${SOEM_DIR}/.git" ]; then
    git clone "${SOEM_REPO}" "${SOEM_DIR}"
    ok "Cloned SOEM"
else
    skip "SOEM repo exists"
fi
git -C "${SOEM_DIR}" fetch --quiet --tags origin || true
git -C "${SOEM_DIR}" checkout --quiet "${SOEM_COMMIT}"
cmake -S "${SOEM_DIR}" -B "${SOEM_DIR}/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "${SOEM_DIR}/build" -j"${JOBS}" >/dev/null
ok "SOEM built at ${SOEM_DIR}/build (pinned ${SOEM_COMMIT:0:10})"

SLAVEINFO="${SOEM_DIR}/build/samples/slaveinfo/slaveinfo"
if [ -x "${SLAVEINFO}" ]; then
    # Diagnostic tool; capability is wiped on rebuild, re-grant if you rebuild.
    sudo setcap cap_net_raw,cap_net_admin+ep "${SLAVEINFO}"
    ok "slaveinfo ready (raw-socket capability granted)"
fi

# ----------------------------------------------------------------------------
# 8. Build nullCAT (this repo checkout)
# ----------------------------------------------------------------------------
log "Building nullCAT (-j${JOBS}; this is the long step on a Pi)"
cmake -S "${REPO_DIR}/pi" -B "${REPO_DIR}/pi/build" >/dev/null
cmake --build "${REPO_DIR}/pi/build" -j"${JOBS}"
ok "Built ${REPO_DIR}/pi/build/nullcat-pi"

# Seed a Pi-appropriate host.json on first install only (never overwrite a
# user's config). The compiled defaults suit a desktop build: web UI on
# loopback and no NIC named. A headless Pi needs the dashboard reachable
# from the LAN and the EtherCAT NIC preset to the onboard port.
HOST_JSON="${REPO_DIR}/pi/build/host.json"
if [[ ! -f "${HOST_JSON}" ]]; then
    tee "${HOST_JSON}" >/dev/null <<EOF
{
    "nicName": "${ECAT_IFACE}",
    "webBindAddr": "0.0.0.0"
}
EOF
    ok "Seeded host.json (EtherCAT on ${ECAT_IFACE}, web UI on all interfaces)"
else
    skip "host.json already exists -- left untouched"
fi

# ----------------------------------------------------------------------------
# 9. IRQ affinity service -- keep NIC/USB IRQ threads off the RT cores
# ----------------------------------------------------------------------------
log "IRQ affinity service"
IRQ_HK_CPUS="0-1"
IRQ_HELPER="/usr/local/sbin/nullcat-irq-affinity.sh"
sudo tee "${IRQ_HELPER}" >/dev/null <<EOF
#!/usr/bin/env bash
# nullCAT: pin NIC/USB IRQs and their threaded handlers to the housekeeping
# cores, off the isolated RT cores. Installed by pi/os-setup/install.sh.
set -u
HK="${IRQ_HK_CPUS}"
for irq in \$(awk -F: '/eth0|eth1|xhci|dwc/ {gsub(/ /,"",\$1); print \$1}' /proc/interrupts); do
    echo "\${HK}" > "/proc/irq/\${irq}/smp_affinity_list" 2>/dev/null || true
done
for pid in \$(pgrep -f 'irq/[0-9]+-(eth0|eth1|xhci|dwc)'); do
    taskset -pc "\${HK}" "\${pid}" >/dev/null 2>&1 || true
done
EOF
sudo chmod +x "${IRQ_HELPER}"
sudo tee /etc/systemd/system/nullcat-irq-affinity.service >/dev/null <<EOF
[Unit]
Description=nullCAT: pin NIC/USB IRQs off the isolated RT cores
Wants=network.target
After=network.target

[Service]
Type=oneshot
ExecStart=${IRQ_HELPER}

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable nullcat-irq-affinity.service >/dev/null 2>&1
ok "nullcat-irq-affinity.service installed (housekeeping cores ${IRQ_HK_CPUS})"

# ----------------------------------------------------------------------------
# 10. Versioned install layout (/opt/nullcat) -- the click-updater's home
#     Every install (source-built or tarball) lands as a version directory;
#     the service runs /opt/nullcat/current, and the web UI's update button
#     swaps that symlink between versions. Config (host/rig/buttons.json)
#     lives INSIDE each version dir and is copied forward on every update.
# ----------------------------------------------------------------------------
log "Versioned install layout (/opt/nullcat)"
NULLCAT_VERSION="$("${REPO_DIR}/pi/build/nullcat-pi" --version)"
sudo mkdir -p /opt/nullcat/versions /opt/nullcat/staging
# One staging authority: the release packaging script builds the same
# layout a tarball ships, so source installs and click-updates are
# byte-identical in structure.
"${REPO_DIR}/deploy/package-release-pi.sh" "${NULLCAT_VERSION}" "${REPO_DIR}/pi/build"
DEST="/opt/nullcat/versions/v${NULLCAT_VERSION}"
# Stash the live config BEFORE anything is deleted: when re-installing the
# SAME version, /opt/nullcat/current points INTO ${DEST}, so the rm -rf
# below would destroy the only copy of the operator's edits (found the
# hard way: bench-tuned device curves vanished on a same-version
# re-install). carcache.json and devicepresets.json are learned/authored
# state and ferry with the config.
CFG_FILES="host.json rig.json buttons.json carcache.json devicepresets.json"
CFG_STASH="$(mktemp -d)"
for f in ${CFG_FILES}; do
    if sudo test -f "/opt/nullcat/current/${f}"; then
        sudo cp -p "/opt/nullcat/current/${f}" "${CFG_STASH}/${f}"
    fi
done
sudo rm -rf "${DEST}"
sudo tar -xzf "${REPO_DIR}/dist/nullCAT-v${NULLCAT_VERSION}-pi-aarch64.tar.gz" -C /opt/nullcat/versions
sudo mv "/opt/nullcat/versions/nullCAT-v${NULLCAT_VERSION}-pi-aarch64" "${DEST}"
# Config adoption: the stashed live config wins; a pre-0.9.5 source
# install's build-dir config is picked up once.
for f in ${CFG_FILES}; do
    if sudo test -f "${CFG_STASH}/${f}"; then
        sudo cp -p "${CFG_STASH}/${f}" "${DEST}/${f}"
    elif [ -f "${REPO_DIR}/pi/build/${f}" ]; then
        sudo cp -p "${REPO_DIR}/pi/build/${f}" "${DEST}/${f}"
    fi
done
sudo rm -rf "${CFG_STASH}"
sudo chown -R "${RT_USER}:${RT_USER}" "${DEST}"
sudo ln -sfn "${DEST}" /opt/nullcat/current.new
sudo mv -Tf /opt/nullcat/current.new /opt/nullcat/current
ok "v${NULLCAT_VERSION} installed at ${DEST} (current -> v${NULLCAT_VERSION})"

# ----------------------------------------------------------------------------
# 11. nullCAT service, updater unit, poweroff sudoers, mDNS advert
# ----------------------------------------------------------------------------
log "nullCAT service"
# Unit generated from the repo template with THIS user; the template's
# /opt/nullcat/current paths are already correct for the versioned layout.
sed -e "s|^User=.*|User=${RT_USER}|" \
    -e "s|^Group=.*|Group=${RT_USER}|" \
    -e "s|^WorkingDirectory=.*|WorkingDirectory=/opt/nullcat/current|" \
    -e "s|^ExecStart=.*|ExecStart=/opt/nullcat/current/nullcat-pi|" \
    "${REPO_DIR}/pi/nullcat-pi.service" | sudo tee /etc/systemd/system/nullcat-pi.service >/dev/null

# Click-updater: templated oneshot unit + the sudoers rule that lets the
# service user start it (the ONLY systemctl verb it is granted).
sudo cp "${REPO_DIR}/pi/nullcat-update@.service" /etc/systemd/system/
sed -e "s|^[a-zA-Z0-9_-]* |${RT_USER} |" "${REPO_DIR}/pi/nullcat-update.sudoers" \
    | sudo tee /etc/sudoers.d/nullcat-update >/dev/null
sudo chmod 440 /etc/sudoers.d/nullcat-update

# Web-UI Shutdown button: the service user may power the system off.
sed -e "s|^[a-zA-Z0-9_-]* |${RT_USER} |" "${REPO_DIR}/pi/nullcat-poweroff.sudoers" 2>/dev/null \
    | sudo tee /etc/sudoers.d/nullcat-poweroff >/dev/null || true
sudo chmod 440 /etc/sudoers.d/nullcat-poweroff 2>/dev/null || true

# nullcat.local:8080 advert (harmless if avahi is absent).
if [ -d /etc/avahi/services ]; then
    sudo cp "${REPO_DIR}/deploy/nullcat.service" /etc/avahi/services/nullcat.service
    ok "mDNS advert installed (http://$(hostname).local:8080)"
fi

sudo systemctl daemon-reload
sudo systemctl enable nullcat-pi.service >/dev/null 2>&1
ok "nullcat-pi.service installed and enabled (starts on boot)"

# ----------------------------------------------------------------------------
# Done
# ----------------------------------------------------------------------------
log "Install complete -- REBOOT REQUIRED (RT kernel + boot tuning)"
cat <<EOF

  sudo reboot

  After reboot, verify:
    uname -r                     # must end in -rt
    cat /proc/cmdline            # must contain isolcpus=2,3 ... threadirqs
    systemctl status nullcat-pi  # active (running)
    http://$(hostname).local:8080  from a browser on the same network

  Then read Docs/PI_SETUP.md "First contact with the drives" before
  connecting hardware -- and SAFETY.md before anything moves.
EOF
