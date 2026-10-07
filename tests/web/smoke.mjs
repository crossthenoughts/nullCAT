#!/usr/bin/env node
// nullCAT web UI smoke test (CI, Linux runner).
//
// Boots the REAL headless binary in simulation mode (no hardware, no root),
// points a REAL headless Chrome at it, and asserts the page actually RUNS:
// the connection badge reaches Connected (which proves app.js parsed AND the
// poll loop is applying /api/status), the link-lost banner is hidden, and
// the console carries zero errors. The apostrophe bug (issue #1) walked
// straight through every earlier gate because the page RENDERED while no
// script ran; this is the gate for that whole class.
//
// Usage: node tests/web/smoke.mjs <nullcat-binary> <chrome-binary>
// Needs puppeteer-core installed (npm install --no-save puppeteer-core).

import { spawn } from 'node:child_process';
import { writeFileSync, mkdirSync, existsSync } from 'node:fs';
import { dirname, resolve } from 'node:path';

const die = (msg) => { console.error('SMOKE FAIL: ' + msg); process.exit(1); };

const bin    = resolve(process.argv[2] || '');
const chrome = process.argv[3] || '/usr/bin/google-chrome';
if (!existsSync(bin))    die('binary not found: ' + bin);
if (!existsSync(chrome)) die('chrome not found: ' + chrome);

// host.json beside the binary is authoritative (over config.json) and read
// once at startup. Simulation mode: full web server, no NIC, no drives.
const dir  = dirname(bin);
const PORT = 18080;
writeFileSync(dir + '/host.json', JSON.stringify({
  configVersion: 2, simulationMode: true, webPort: PORT,
  gpioMode: 'off', gpioEnabled: false, logToConsole: true,
  webShowDevices: true,   // the Haptics strip and Devices section render
}));
// A rig with one position axis and one belt (torque) axis: the strip only
// renders when a torque-capable axis exists to route to.
const axis = (name, slaveIndex, mode, axisType) => ({
  slaveIndex, name, mode, axisType, invertDir: false, strokeMm: 100, ballscrewPitch: 10,
  encoderCountsPerRev: 131072, reductionRatio: '1:1', homeDirection: 'negative', parkMode: 'endstop',
  homingBackoffMm: 1.5, homingSpeed: 250, homingTorquePct: 25, maxVelocityMmS: 200,
  maxAccelerationMmS2: 10000, maxJerkMmS3: 60000, followingErrorWindowMm: 100, trackingWnHz: 30,
  unparkTimeSec: 3, parkTimeSec: 3, onlineHoldTimeoutSec: 15, spikeFilterEnabled: false, spikeMaxMm: 5,
  torqueMinPct: 5, torqueMaxPct: 50, beltSlewPctPerSec: 3000, beltOverspeedRpm: 600, beltOverspeedMs: 200,
  beltMaxTravelRevs: 3, beltMaxRpm: 800, beltRelaxerSec: 0, beltRelaxerPct: 80 });
writeFileSync(dir + '/rig.json', JSON.stringify({
  configVersion: 2, numDrives: 2,
  global: { conditioningMode: 'bypass', blendTimeSec: 2, blendMaxVelocityMmS: 20, requireUserFaultReset: false },
  axes: [axis('Seat', 1, 'csp', 'linear_vertical'), axis('Belt', 2, 'torque', 'belt')],
}));
mkdirSync(dir + '/logs', { recursive: true });

const child = spawn(bin, [], { cwd: dir, stdio: ['ignore', 'pipe', 'pipe'] });
let childOut = '';
child.stdout.on('data', d => { childOut += d; });
child.stderr.on('data', d => { childOut += d; });
let exited = false;
child.on('exit', () => { exited = true; });

const sleep = (ms) => new Promise(r => setTimeout(r, ms));

// Wait for the server (the binary writes/normalizes its config first).
let up = false;
for (let i = 0; i < 100 && !exited; i++) {
  try { const r = await fetch(`http://127.0.0.1:${PORT}/api/status`); if (r.ok) { up = true; break; } } catch {}
  await sleep(200);
}
if (!up) {
  console.error(childOut.slice(-3000));
  die('server never answered /api/status (binary ' + (exited ? 'exited' : 'silent') + ')');
}

const { default: puppeteer } = await import('puppeteer-core');
const browser = await puppeteer.launch({
  executablePath: chrome, headless: true,
  args: ['--no-sandbox', '--disable-dev-shm-usage'],
});
const pageErrors = [], consoleErrors = [];
try {
  const page = await browser.newPage();
  page.on('pageerror', e => pageErrors.push(String(e)));
  page.on('console', m => {
    // favicon 404 is browser noise, not an app error.
    if (m.type() === 'error' && !m.text().includes('favicon')) consoleErrors.push(m.text());
  });
  // networkidle never settles here (the app polls every 500 ms by design),
  // so wait for DOM, then give the poll and badge timers a few cycles.
  await page.goto(`http://127.0.0.1:${PORT}/`, { waitUntil: 'domcontentloaded', timeout: 30000 });
  await sleep(3500);

  const st = await page.evaluate(() => ({
    badge:        (document.getElementById('conn-status') || {}).textContent || '(no badge)',
    bannerHidden: (document.getElementById('linkBanner') || { hidden: true }).hidden,
    buttons:      document.querySelectorAll('button').length,
    // Haptics strip: built from /api/haptics/schema, one tile per effect
    // plus the Master tile, each effect tile with a Test button.
    hapTiles:     document.querySelectorAll('#hapStrip .hap-tile').length,
    hapTests:     document.querySelectorAll('#hapStrip [data-test]').length,
    hapHidden:    (document.getElementById('hapPanel') || { hidden: true }).hidden,
    // Sim channels bindings editor (Setup): one row per default binding,
    // its token list from the schema registry. This is the surface that
    // referenced an undefined token table after the 0.9.7 refactor.
    ncxRows:      document.querySelectorAll('#ncxRows .frow').length,
    ncxTokenOpts: (document.querySelector('#ncxRows select') || { options: [] }).options.length,
    // Sim dots: one per effect tile, all grey with no sim; the Master
    // tile's game selector exists.
    simDots:      document.querySelectorAll('#hapStrip .hap-tile [data-sdot]').length,
    simDotsGrey:  document.querySelectorAll('#hapStrip .hap-tile [data-sdot].d0').length,
    gameSel:      !!document.getElementById('hapGameSel'),
  }));
  // Open the Lateral slip tile's route drawer: a per-wheel effect lists a
  // part selector beside every axis gain.
  const drawer = await page.evaluate(() => {
    const chip = document.querySelector('#hapStrip .hap-tile[data-fx="slipLat"] .hap-routechip');
    if (!chip) return { chip: false };
    chip.click();
    const dr = document.getElementById('hapDrawer');
    return { chip: true, hidden: !dr || dr.hidden,
             gains: dr ? dr.querySelectorAll('input[data-axis]').length : 0,
             parts: dr ? dr.querySelectorAll('select[data-part]').length : 0,
             partOpts: dr && dr.querySelector('select[data-part]') ? dr.querySelector('select[data-part]').options.length : 0 };
  });
  const schema = await (await fetch(`http://127.0.0.1:${PORT}/api/haptics/schema`)).json();
  const nEffects = (schema.effects || []).length;
  const nTokens  = (schema.tokens  || []).length;

  const fails = [];
  if (pageErrors.length)        fails.push('page errors: ' + pageErrors.join(' | '));
  if (consoleErrors.length)     fails.push('console errors: ' + consoleErrors.join(' | '));
  if (st.badge !== 'Connected') fails.push(`badge is "${st.badge}" (poll loop never applied a status)`);
  if (!st.bannerHidden)         fails.push('link-lost banner is showing');
  if (st.buttons < 5)           fails.push(`page rendered only ${st.buttons} buttons`);
  if (nEffects < 10)            fails.push(`schema lists ${nEffects} effects`);
  if (st.hapHidden)             fails.push('haptics strip is hidden (experimental flag + belt axis set)');
  if (st.hapTiles !== nEffects + 1) fails.push(`strip has ${st.hapTiles} tiles, expected ${nEffects} effects + Master`);
  if (st.hapTests !== nEffects) fails.push(`strip has ${st.hapTests} Test buttons, expected ${nEffects}`);
  if (nTokens < 34)             fails.push(`schema lists ${nTokens} channel tokens`);
  if (st.ncxRows !== nTokens)   fails.push(`bindings editor has ${st.ncxRows} rows, expected one per token (${nTokens})`);
  if (st.ncxTokenOpts !== nTokens) fails.push(`bindings token list has ${st.ncxTokenOpts} options, expected ${nTokens}`);
  if (st.simDots !== nEffects)  fails.push(`${st.simDots} sim dots, expected one per effect (${nEffects})`);
  if (st.simDotsGrey !== nEffects) fails.push(`${st.simDotsGrey} grey sim dots with no sim, expected ${nEffects}`);
  if (!st.gameSel)              fails.push('no game selector on the Master tile');
  if (!drawer.chip)             fails.push('no route chip on the Lateral slip tile');
  else {
    if (drawer.hidden)          fails.push('route drawer did not open');
    if (drawer.gains !== 2)     fails.push(`route drawer lists ${drawer.gains} axes, expected 2 (Seat + Belt)`);
    if (drawer.parts !== 2)     fails.push(`route drawer has ${drawer.parts} part selectors, expected one per axis`);
    if (drawer.partOpts !== 7)  fails.push(`part selector has ${drawer.partOpts} options, expected 7 (all, front, rear, 4 corners)`);
  }

  if (fails.length) { console.error(childOut.slice(-2000)); die('\n  ' + fails.join('\n  ')); }
  console.log(`SMOKE OK: badge Connected, ${st.buttons} buttons, haptics strip ${st.hapTiles} tiles from a ${nEffects}-effect schema, ${st.ncxRows} binding rows, part selectors in the drawer, 0 page/console errors.`);
} finally {
  await browser.close().catch(() => {});
  child.kill('SIGTERM');
}
