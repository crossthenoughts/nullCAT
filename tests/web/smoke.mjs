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
  }));

  const fails = [];
  if (pageErrors.length)        fails.push('page errors: ' + pageErrors.join(' | '));
  if (consoleErrors.length)     fails.push('console errors: ' + consoleErrors.join(' | '));
  if (st.badge !== 'Connected') fails.push(`badge is "${st.badge}" (poll loop never applied a status)`);
  if (!st.bannerHidden)         fails.push('link-lost banner is showing');
  if (st.buttons < 5)           fails.push(`page rendered only ${st.buttons} buttons`);

  if (fails.length) { console.error(childOut.slice(-2000)); die('\n  ' + fails.join('\n  ')); }
  console.log(`SMOKE OK: badge Connected, ${st.buttons} buttons, 0 page/console errors.`);
} finally {
  await browser.close().catch(() => {});
  child.kill('SIGTERM');
}
