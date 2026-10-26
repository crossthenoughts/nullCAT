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
  // Shakers on the null backend (no card): the haptics idle clock then
  // steps the layer and publishes the mute state without the control
  // loop, so the mute clicks below read back through /api/status.
  shakersEnabled: true, audioDevice: 'null', audioChannels: 2,
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
const pageErrors = [], consoleErrors = [], posts = [];
try {
  const page = await browser.newPage();
  page.on('pageerror', e => pageErrors.push(String(e)));
  page.on('console', m => {
    // favicon 404 is browser noise, not an app error.
    if (m.type() === 'error' && !m.text().includes('favicon')) consoleErrors.push(m.text());
  });
  // Every POST the page makes, body and headers: the interaction checks
  // below assert what the clicks actually sent.
  page.on('request', r => { if (r.method() === 'POST') posts.push({ url: r.url(), body: r.postData() || '', headers: r.headers() }); });
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
    shakerFields: !!document.getElementById('cf-shakers') && !!document.getElementById('cf-audiodev2') && !!document.getElementById('hapAxisDelay'),
    // Readouts: the two slip tiles, Road and Kerb carry a per-wheel now/peak
    // line, every other continuous tile an input now/peak line, every
    // wave a source tag slot, and the magnitude-fed tiles a peak % trim.
    wheelLines:   document.querySelectorAll('#hapStrip .hap-tile [data-wheels]').length,
    surfLines:    document.querySelectorAll('#hapStrip .hap-tile [data-surf]').length,
    inLines:      document.querySelectorAll('#hapStrip .hap-tile [data-in]').length,
    waveTags:     document.querySelectorAll('#hapStrip .hap-tile [data-wtag]').length,
    peakTrims:    document.querySelectorAll('#hapStrip .hap-tile input[data-k="peakPct"]').length,
    wheelText:    (document.querySelector('#hapStrip .hap-tile[data-fx="slipLat"] [data-wheels]') || {}).textContent || '',
    // Car table: the Master tile's car line with its actions and the
    // follow switch, the preset search box over the whole table, and the
    // Engine tile's folded notes block (hidden with no car named).
    carRow:       !!document.getElementById('hapCarNow') && !!document.getElementById('hapCarPull')
                  && !!document.getElementById('hapCarSave') && !!document.getElementById('hapCarForget')
                  && !!document.getElementById('hapFollowCar') && !!document.getElementById('hapCarApply'),
    carNow:       (document.getElementById('hapCarNow') || {}).textContent || '',
    carFollow:    !!(document.getElementById('hapFollowCar') || {}).checked,
    // The preset search: nullCAT's own list over the whole table (app.js
    // hapCarSuggest); typing a make shows matching rows with their game.
    carOpts:      (typeof hapCarItems !== 'undefined') ? hapCarItems.length : -1,
    carSug:       (() => {
      const i = document.getElementById('hapCarPick'), b = document.getElementById('hapCarSug');
      if (!i || !b) return { rows: -1, shown: false, game: '' };
      i.value = 'bmw m3'; i.dispatchEvent(new Event('input'));
      const r = { rows: b.querySelectorAll('.car-sug-i').length, shown: !b.hidden,
                  game: (b.querySelector('.car-sug-g') || {}).textContent || '' };
      i.value = ''; i.dispatchEvent(new Event('input'));
      return r;
    })(),
    // Starting points: the type selector with the shipped types (plus its
    // prompt) and its apply button.
    presetRow:    !!document.getElementById('hapPresetSel') && !!document.getElementById('hapPresetApply'),
    presetOpts:   (document.getElementById('hapPresetSel') || { options: [] }).options.length,
    carNotes:     !!document.querySelector('#hapStrip .hap-tile[data-fx="rpmVibe"] [data-car]'),
    carNotesHidden: (document.querySelector('#hapStrip .hap-tile[data-fx="rpmVibe"] [data-car]') || { hidden: true }).hidden,
  }));
  const carsApi = await (await fetch(`http://127.0.0.1:${PORT}/api/cars/list`)).json();
  const nCars = (carsApi.cars || []).length;
  // Open the Lateral slip effect's editor from its mixer row: the tile moves
  // into the editor pane with the route drawer under it as the routing
  // section, and a per-wheel effect lists a part selector beside every axis
  // gain. Closing the editor puts the tile back in its row.
  const drawer = await page.evaluate(() => {
    const name = document.querySelector('#hapStrip .hap-tile[data-fx="slipLat"] .dh .nm');
    if (!name) return { chip: false };
    name.click();
    const dr = document.getElementById('hapDrawer'), edb = document.getElementById('hapEdBody');
    const r = { chip: true, hidden: !dr || dr.hidden,
             inEditor: !!edb && !!dr && dr.parentElement === edb && !!edb.querySelector('.hap-tile[data-fx="slipLat"]'),
             layout: (document.getElementById('ws') || { dataset: {} }).dataset.hp,
             gains: dr ? dr.querySelectorAll('input[data-axis]').length : 0,
             parts: dr ? dr.querySelectorAll('select[data-part]').length : 0,
             partOpts: dr && dr.querySelector('select[data-part]') ? dr.querySelector('select[data-part]').options.length : 0 };
    document.getElementById('hapEdClose').click();
    r.back = !!document.querySelector('#hapStrip .hap-tile[data-fx="slipLat"]') && !document.querySelector('#hapStrip [data-ph]');
    return r;
  });
  const schema = await (await fetch(`http://127.0.0.1:${PORT}/api/haptics/schema`)).json();
  const nEffects = (schema.effects || []).length;
  const nTokens  = (schema.tokens  || []).length;

  // ---- interactions: the clicks send what they should, and the page
  // shows what came back ----
  // A tile's M posts that tile's mute; after a poll the tile carries the
  // muted class (the idle clock publishes hapMute[]). The master MUTE
  // posts {"on":true}; after a poll every tile is muted and the Master
  // tile says MUTED. Then both back off.
  const clickMute = async (sel) => { await page.evaluate((s) => { document.querySelector(s).click(); }, sel); await sleep(1200); };
  posts.length = 0;
  await clickMute('#hapStrip .hap-tile[data-fx="driveline"] [data-mute]');
  const mute1 = await page.evaluate(() => ({
    tileMuted: document.querySelector('#hapStrip .hap-tile[data-fx="driveline"]').classList.contains('muted'),
    otherMuted: document.querySelector('#hapStrip .hap-tile[data-fx="abs"]').classList.contains('muted'),
    master: (document.getElementById('hapMuteState') || {}).textContent || '' }));
  await clickMute('#hapMuteBtn');
  const mute2 = await page.evaluate(() => ({
    allMuted: Array.from(document.querySelectorAll('#hapStrip .hap-tile[data-fx]')).every(t => t.classList.contains('muted')),
    master: (document.getElementById('hapMuteState') || {}).textContent || '',
    btn: (document.getElementById('hapMuteBtn') || {}).textContent || '' }));
  await clickMute('#hapMuteBtn');
  const mute3 = await page.evaluate(() => ({
    anyMuted: Array.from(document.querySelectorAll('#hapStrip .hap-tile[data-fx]')).some(t => t.classList.contains('muted')),
    master: (document.getElementById('hapMuteState') || {}).textContent || '' }));
  const mutePosts = posts.filter(p => p.url.endsWith('/api/haptics/mute')).map(p => p.body);
  // Untick follow car and Save: the strip's Save posts the whole rig with
  // followCar false and the apply generation the copy was loaded at; the
  // page reports the save and the file carries it.
  posts.length = 0;
  const genBefore = (await fetch(`http://127.0.0.1:${PORT}/api/rig`)).headers.get('x-nullcat-gen');
  await page.evaluate(() => { const fc = document.getElementById('hapFollowCar'); fc.click(); });
  await sleep(200);
  const saveBarShown = await page.evaluate(() => !(document.getElementById('hapSaveBar') || { hidden: true }).hidden);
  await page.evaluate(() => { document.getElementById('hapSave').click(); });
  await sleep(1500);
  const saveStatus = await page.evaluate(() => (document.getElementById('cfgStatus') || {}).textContent || '');
  const rigPost = posts.find(p => p.url.endsWith('/api/rig'));
  let rigPostFollow = null, rigPostGen = null;
  if (rigPost) {
    try { rigPostFollow = JSON.parse(rigPost.body).global.haptics.followCar; } catch {}
    rigPostGen = rigPost.headers['x-nullcat-gen'];
  }
  const rigAfterResp = await fetch(`http://127.0.0.1:${PORT}/api/rig`);
  const genAfter = rigAfterResp.headers.get('x-nullcat-gen');
  const rigAfter = await rigAfterResp.json();
  const followAfter = ((rigAfter.global || {}).haptics || {}).followCar;
  const pageGen = await page.evaluate(() => cfgGenLoaded);
  // The shaker status line names each card's own error when none is open
  // (the status carries per-device errors, never one for the sink).
  const shakerLine = await page.evaluate(() => {
    shakerStatus({ shakers: { enabled: true, open: false, channels: 0,
      devices: [{ name: 'USB Audio', open: false, error: 'card gone', first: 0, channels: 2 }] } });
    return (document.getElementById('shakerStatus') || {}).textContent || '';
  });
  // A card that is down shows reopen; one that came back as a stereo card
  // asked for four reads "2 of 4" with the reopen hidden and the line
  // redrawn (it used to stay blank after a card dropped out and came back).
  const shakerBack = await page.evaluate(() => {
    const vis = id => { const e = document.getElementById(id); return !!e && !e.hidden; };
    shakerStatus({ shakers: { enabled: true, open: true, channels: 4,
      devices: [{ name: 'USB Audio', open: true, error: '', first: 0, channels: 4, channelsWanted: 4, rate: 48000, latencyMs: 20, bufferMs: 15, underruns: 0, levels: [0, 0, 0, 0] }] } });
    shakerStatus({ shakers: { enabled: true, open: false, channels: 4,
      devices: [{ name: 'USB Audio', open: false, error: 'card gone', first: 0, channels: 4, channelsWanted: 4 }] } });
    const downReopen = vis('shakerReopen');
    shakerStatus({ shakers: { enabled: true, open: true, channels: 4,
      devices: [{ name: 'USB Audio', open: true, error: '', first: 0, channels: 2, channelsWanted: 4, rate: 48000, latencyMs: 20, bufferMs: 15, underruns: 0, levels: [0, 0, 0, 0] }] } });
    return { downReopen, upReopen: vis('shakerReopen'), tests: vis('shakerTests'),
             line: (document.getElementById('shakerLine') || {}).textContent || '' };
  });

  const fails = [];
  if (!mute1.tileMuted)         fails.push('tile mute click: the driveline tile did not read back muted after a poll');
  if (mute1.otherMuted)         fails.push('tile mute click: another tile read back muted');
  if (mute1.master !== '')      fails.push(`tile mute click: the master reads "${mute1.master}", expected nothing`);
  if (!mute2.allMuted)          fails.push('master mute click: not every tile read back muted');
  if (mute2.master !== 'MUTED') fails.push(`master mute click: the master reads "${mute2.master}", expected MUTED`);
  if (mute2.btn !== 'UNMUTE')   fails.push(`master mute click: the button reads "${mute2.btn}", expected UNMUTE`);
  if (mute3.anyMuted || mute3.master !== '') fails.push('master unmute click: a tile or the master still reads muted');
  if (JSON.stringify(mutePosts) !== JSON.stringify(['{"effect":"driveline","on":true}', '{"on":true}', '{"on":false}']))
    fails.push('mute clicks posted ' + JSON.stringify(mutePosts) + ', expected the tile mute, the master mute, the master unmute');
  if (!saveBarShown)            fails.push('unticking follow car did not show the strip\'s Save bar');
  if (!rigPost)                 fails.push('Save did not POST /api/rig');
  if (rigPostFollow !== false)  fails.push(`Save posted followCar ${JSON.stringify(rigPostFollow)}, expected false`);
  if (!genBefore || rigPostGen !== genBefore)
    fails.push(`Save sent X-Nullcat-Gen ${JSON.stringify(rigPostGen)}, expected ${JSON.stringify(genBefore)} (the generation the page loaded)`);
  if (!genBefore || genAfter !== String(Number(genBefore) + 1))
    fails.push(`after the save /api/rig hands out generation ${JSON.stringify(genAfter)}, expected one past ${JSON.stringify(genBefore)} (a save that changed the file moves it)`);
  if (pageGen !== Number(genAfter))
    fails.push(`the page holds generation ${JSON.stringify(pageGen)} after its save, expected ${JSON.stringify(genAfter)} (taken from the reply)`);
  if (!/^Saved/.test(saveStatus)) fails.push(`Save status reads "${saveStatus}", expected Saved`);
  if (followAfter !== false)    fails.push(`/api/rig carries followCar ${JSON.stringify(followAfter)} after the save, expected false`);
  if (!shakerLine.includes('card gone') || !shakerLine.includes('USB Audio'))
    fails.push(`shaker line reads "${shakerLine}" for a card that failed to open, expected the card and its error`);
  if (!shakerBack.downReopen)   fails.push('shaker card down: the reopen button is not shown');
  if (shakerBack.upReopen)      fails.push('shaker card back: the reopen button is still shown');
  if (!shakerBack.tests)        fails.push('shaker card back: the test buttons are hidden');
  if (!shakerBack.line.includes('2 of 4 channels') || !shakerBack.line.includes('shakers 3-4 silent'))
    fails.push(`shaker line reads "${shakerBack.line}" for a stereo card asked for four, expected "2 of 4 channels" and "shakers 3-4 silent"`);
  if (pageErrors.length)        fails.push('page errors: ' + pageErrors.join(' | '));
  if (consoleErrors.length)     fails.push('console errors: ' + consoleErrors.join(' | '));
  if (st.badge !== 'Connected') fails.push(`badge is "${st.badge}" (poll loop never applied a status)`);
  if (!st.bannerHidden)         fails.push('link-lost banner is showing');
  if (st.buttons < 5)           fails.push(`page rendered only ${st.buttons} buttons`);
  if (nEffects < 10)            fails.push(`schema lists ${nEffects} effects`);
  if (st.hapHidden)             fails.push('haptics strip is hidden (experimental flag + belt axis set)');
  if (st.hapTiles !== nEffects + 1) fails.push(`strip has ${st.hapTiles} tiles, expected ${nEffects} effects + Master`);
  if (st.hapTests !== nEffects) fails.push(`strip has ${st.hapTests} Test buttons, expected ${nEffects}`);
  if (nTokens < 36)             fails.push(`schema lists ${nTokens} channel tokens`);
  if (st.ncxRows !== nTokens)   fails.push(`bindings editor has ${st.ncxRows} rows, expected one per token (${nTokens})`);
  if (st.ncxTokenOpts !== nTokens) fails.push(`bindings token list has ${st.ncxTokenOpts} options, expected ${nTokens}`);
  if (st.simDots !== nEffects)  fails.push(`${st.simDots} sim dots, expected one per effect (${nEffects})`);
  if (st.simDotsGrey !== nEffects) fails.push(`${st.simDotsGrey} grey sim dots with no sim, expected ${nEffects}`);
  if (!st.gameSel)              fails.push('no game selector on the Master tile');
  if (!st.shakerFields)         fails.push('shaker host fields or the axis delay field are missing');
  if (st.wheelLines !== 6)      fails.push(`${st.wheelLines} per-wheel readout lines, expected 6 (Lateral, Longitudinal, Road, Kerb, Wheels, Impacts)`);
  if (st.surfLines !== 1)       fails.push(`${st.surfLines} Surface per-wheel lines, expected 1`);
  if (st.inLines < 5)           fails.push(`${st.inLines} input readout lines, expected one per other continuous tile`);
  if (st.waveTags !== nEffects) fails.push(`${st.waveTags} wave source tags, expected one per effect tile (${nEffects})`);
  if (st.peakTrims !== 4)       fails.push(`${st.peakTrims} peak % trims, expected 4 (two slip tiles, Kerb, Road)`);
  if (!/FL 0\.00\/0\.00/.test(st.wheelText)) fails.push(`Lateral slip per-wheel line reads "${st.wheelText}", expected now/peak per corner from the status`);
  if (!st.carRow)               fails.push('car table row on the Master tile is incomplete (car line, pull, save, forget, follow, apply)');
  if (st.carNow !== '(none)')   fails.push(`car line reads "${st.carNow}" with no sim, expected (none)`);
  if (!st.carFollow)            fails.push('follow car is not ticked by default');
  if (nCars < 450)              fails.push(`/api/cars/list carries ${nCars} entries, expected the shipped table (450+)`);
  if (st.carOpts !== nCars)     fails.push(`preset search holds ${st.carOpts} cars, expected ${nCars} from /api/cars/list`);
  if (!(st.carSug.rows > 0 && st.carSug.shown && st.carSug.game))
    fails.push(`preset search: typing "bmw m3" shows ${st.carSug.rows} rows (shown ${st.carSug.shown}, game "${st.carSug.game}"), expected matching cars with their game`);
  if (!st.presetRow)            fails.push('no starting point row (type selector and apply) on the Master tile');
  if (st.presetOpts !== 11)     fails.push(`starting point selector has ${st.presetOpts} options, expected the 10 shipped starting points and the prompt`);
  if (!st.carNotes)             fails.push('no car notes block on the Engine tile');
  if (!st.carNotesHidden)       fails.push('car notes block is showing with no car named');
  if (!drawer.chip)             fails.push('no name key on the Lateral slip mixer row');
  else {
    if (drawer.hidden)          fails.push('route drawer did not open with the editor');
    if (!drawer.inEditor)       fails.push('the Lateral slip tile and its route drawer did not move into the editor pane');
    if (drawer.layout !== 'ed') fails.push(`opening an effect left the Haptics panel in layout "${drawer.layout}", expected ed`);
    if (!drawer.back)           fails.push('closing the editor did not put the Lateral slip tile back in its mixer row');
    if (drawer.gains !== 2)     fails.push(`route drawer lists ${drawer.gains} axes, expected 2 (Seat + Belt)`);
    if (drawer.parts !== 2)     fails.push(`route drawer has ${drawer.parts} part selectors, expected one per axis`);
    if (drawer.partOpts !== 7)  fails.push(`part selector has ${drawer.partOpts} options, expected 7 (all, front, rear, 4 corners)`);
  }

  // A page left with an effect's editor open and Motion on the rail loads
  // again: the stored layout used to stop the script at load (the effect
  // state read before it was declared), leaving "Disconnected" for good.
  {
    await page.evaluate(() => { localStorage.setItem('nullcat.ui.layout', JSON.stringify({ mo: 'rail', hp: 'ed' })); localStorage.setItem('nullcat.ui.sel', 'slipLat'); });
    const before = pageErrors.length;
    await page.reload({ waitUntil: 'domcontentloaded', timeout: 30000 });
    await sleep(3500);
    const back = await page.evaluate(() => ({
      booted: !!window.__nullcatBooted,
      badge: (document.getElementById('conn-status') || {}).textContent || '',
      hp: (document.getElementById('ws') || { dataset: {} }).dataset.hp,
      mo: (document.getElementById('ws') || { dataset: {} }).dataset.mo,
      ed: !!document.querySelector('#hapEdBody .hap-tile[data-fx="slipLat"]') }));
    if (pageErrors.length > before)  fails.push('reload with a stored editor-open layout: page errors ' + pageErrors.slice(before).join(' | '));
    if (!back.booted)                fails.push('reload with a stored editor-open layout: the script did not finish its start');
    if (!/Connected/.test(back.badge)) fails.push(`reload with a stored editor-open layout: badge "${back.badge}", expected Connected`);
    if (!(back.hp === 'ed' && back.mo === 'rail' && back.ed)) fails.push(`reload with a stored editor-open layout: layout ${back.mo}/${back.hp}, editor ${back.ed}, expected rail/ed with Lateral slip open`);
    await page.evaluate(() => { localStorage.removeItem('nullcat.ui.layout'); localStorage.removeItem('nullcat.ui.sel'); });
  }

  if (fails.length) { console.error(childOut.slice(-2000)); die('\n  ' + fails.join('\n  ')); }
  console.log(`SMOKE OK: badge Connected, ${st.buttons} buttons, haptics strip ${st.hapTiles} tiles from a ${nEffects}-effect schema, ${st.ncxRows} binding rows, part selectors in the drawer, 0 page/console errors.`);
} finally {
  await browser.close().catch(() => {});
  child.kill('SIGTERM');
}
