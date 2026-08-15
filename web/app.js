// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
'use strict';
// ============================================================
// nullCAT web UI. Polls /api/status + /api/logs (WS if present).
// ============================================================
const $ = (id) => document.getElementById(id);
const API = '';

const elLogo=$('logo'), elConn=$('conn-status'), elClock=$('clock');
const vLoop=$('v-loop'), vEc=$('v-ec'), vRate=$('v-rate'), vCyc=$('v-cyc'),
      vWkc=$('v-wkc'), vSlv=$('v-slv'), vTelemetry=$('v-telemetry'), vUdp=$('v-udp'), vJit=$('v-jit'), vJpk=$('v-jpk'),
      sparkLine=$('sparkLine'), driveGrid=$('drive-grid'), driveCount=$('driveCount');
const btn={ init:$('btn-init'),start:$('btn-start'),stop:$('btn-stop'),home:$('btn-home'),
  park:$('btn-park'),belts:$('btn-belts'),fault:$('btn-fault'),restart:$('btn-restart'),shutdown:$('btn-shutdown'),estop:$('btn-estop'),release:$('btn-release') };

let lastRecvMs=0, jpeak=0, ecatMode='init', parkMode='park', beltsSlack=false;  // ecatMode/parkMode/beltsSlack: what those buttons do now
const SPARK_N=60; const jbuf=new Float32Array(SPARK_N); let jhead=0, jfill=0;
const peaks={};   // per-drive sticky {vel,trq}

const TYPE={ linear_vertical:'VERT', linear_horizontal:'HORIZ', belt:'BELT' };
const hex4=(v)=> '0x'+(((v|0)&0xFFFF)>>>0).toString(16).toUpperCase().padStart(4,'0');
// (DS402 statusword decode + the local status derivation moved to the shared
//  StatusModel; the web consumes /api/status `ind`/`aggregate`. hex4() is still
//  used to show the raw statusword on each card.)

/* ---- theme (persisted) ---- */
const THEME_KEY='nullcat-theme';
function applyTheme(t){ document.documentElement.setAttribute('data-theme', t==='dark'?'dark':'light'); }
let theme = localStorage.getItem(THEME_KEY)||'light'; applyTheme(theme);

/* ---- transport: poll-only. httplib has no WebSocket server, so the old
   /ws reconnect-every-3s just churned failing connections. Pure polling. ---- */
async function pollStatus(){ try{ const r=await fetch(API+'/api/status'); if(r.ok) applyState(await r.json()); }catch(_){} }

/* ---- apply status ---- */
function applyState(s){
  lastRecvMs=Date.now();
  setLinkDown(false);   // fresh data: clear the link-lost guard (button states recomputed below)
  const running=!!s.loopRunning, op=!!s.masterOp, estop=!!s.estop;

  setV(vLoop, running?'RUN':'STOP', running?'ok':'');
  setV(vEc, op?'OP':'OFF', op?'ok':'warn');
  vRate.innerHTML = s.loopHz? s.loopHz.toFixed(0)+'<span class="u">Hz</span>' : ' - ';
  vCyc.innerHTML  = s.loopHz? (1e6/s.loopHz).toFixed(0)+'<span class="u">µs</span>' : ' - ';
  setV(vWkc, String(s.wkcErrors ?? 0), (s.wkcErrors>0)?'bad':'ok');
  vSlv.textContent = (s.slavesFound ?? 0) + ' / ' + (s.numDrives ?? 0);
  setV(vTelemetry, s.telemetryReceiving?'RX':(s.telemetryInit?'idle':'off'), s.telemetryReceiving?'ok':(s.telemetryInit?'':'warn'));
  // UDP rate diagnostic: new/arrival Hz + hold% ( - when diag off / no window yet)
  if(vUdp){ const ua=+s.udpArrivalHz, un=+s.udpNewHz, uh=+s.udpHoldPct;
    vUdp.innerHTML = (ua>=0)? `${un.toFixed(0)}/${ua.toFixed(0)}<span class="u">Hz</span> ${uh.toFixed(0)}%` : ' - '; }
  // Config-page hint: suggest a mode from the measured new-frame rate (inform, never auto-switch).
  { const e=$('cf-udphint'); if(e){ const un=+s.udpNewHz;
    e.textContent = (un<0) ? 'UDP rate: enable diagnostics to measure'
      : (un>500) ? `UDP new ${un.toFixed(0)}Hz - high; Bypass/Interpolate suggested`
      : (un<150) ? `UDP new ${un.toFixed(0)}Hz - low; Interpolate or Filter suggested`
      : `UDP new ${un.toFixed(0)}Hz`; } }

  const j=+s.maxJitterUs||0; jbuf[jhead]=j; jhead=(jhead+1)%SPARK_N; if(jfill<SPARK_N) jfill++;
  if(j>jpeak) jpeak=j;
  vJit.textContent = j? j.toFixed(0):' - '; vJpk.textContent = jpeak? jpeak.toFixed(0):' - ';
  renderSpark();

  buildDriveCards(s.drives||[], running);
  driveCount.textContent = (s.numDrives ?? (s.drives||[]).length)+' axes';

  document.body.classList.toggle('estop-active', estop);
  // Logo keys on the shared-model aggregate: estop > fault > running > offline -> the
  // four stateful images. This makes logo-fault.png actually trigger on a drive fault
  // (the old driveFault check tested d.state, a motion name that never says "fault",
  // so the fault image was dead). Fallback to the old derivation only if an older
  // server doesn't emit `aggregate`.
  const oLogo = estop?'is-estop'
    : (s.drives||[]).some(d=>/fault|fatal/i.test(d.state||''))?'is-fault'
    : running?'is-online':'is-offline';
  const logo = (s.aggregate && s.aggregate.state)
    ? ({estop:'is-estop',fault:'is-fault',running:'is-online',offline:'is-offline'}[s.aggregate.state] || oLogo)
    : oLogo;
  setLogo(logo);

  // One context-aware EtherCAT button. Off → Initialize; operational + loop
  // stopped → Stop EtherCAT (de-init, drives OP→INIT); running → greyed;
  // busy → progress label. masterOp stays true through de-init, so a busy+op
  // state means "stopping".
  const initBusy=!!s.initBusy;
  if(initBusy){ ecatMode=null; btn.init.disabled=true; btn.init.textContent=op?'Stopping…':'Initializing…'; }
  else if(op&&!running){ ecatMode='deinit'; btn.init.disabled=false; btn.init.textContent='Stop EtherCAT'; }
  else if(op){ ecatMode=null; btn.init.disabled=true; btn.init.textContent='EtherCAT: OP'; }
  else { ecatMode='init'; btn.init.disabled=false; btn.init.textContent='Initialize EtherCAT'; }
  btn.init.classList.toggle('btn-warn', ecatMode==='deinit');
  // Grey Stop Loop only while an axis is mid-UNPARK or mid-BLEND: stopping
  // there strands axes at centre (park timed out) and the seat then
  // false-latches a drive on its mid-stroke hold. HOMING deliberately does
  // NOT gate Stop -- the loop-stop path force-parks homing axes, and gating
  // on it deadlocked the UI when the post-e-stop-release rehome stalled
  // (the only way forward was re-engaging the e-stop). E-STOP stays live
  // as the escape throughout.
  const stopGate=(s.drives||[]).some(d=>/unparking|blending/i.test(d.state||''));
  const settling=(s.drives||[]).some(d=>/homing|unparking|blending/i.test(d.state||''));
  btn.start.disabled=running||!op; btn.stop.disabled=!running||stopGate;
  btn.home.disabled=!running||estop||settling;
  // Park toggles to Unpark once all axes are parked (no rehome needed to resume).
  const parked=!!s.parked;
  parkMode=parked?'unpark':'park';
  btn.park.textContent=parked?'Unpark All':'Park All';
  btn.park.disabled=!running||estop;
  btn.park.classList.toggle('btn-start',parked);  // green-ish when it will resume motion
  // Belt don/doff: hidden entirely unless the config has torque axes. Label shows
  // the ACTION it will take (slack -> get in/out; tension -> blend back in).
  const hasBelts=!!s.hasBelts; beltsSlack=!!s.beltsSlack;
  btn.belts.hidden=!hasBelts;
  btn.belts.textContent=beltsSlack?'Tension Belts':'Slack Belts';
  btn.belts.disabled=!running||estop;
  btn.belts.classList.toggle('btn-start',beltsSlack);  // green-ish when it will apply tension
  btn.estop.disabled=estop; btn.release.disabled=!estop;
}
function setV(el,t,c){ el.textContent=t; el.className='v'+(c?' '+c:''); }
function setLogo(c){ if(!elLogo) return; const f={'is-offline':'logo-offline.png','is-online':'logo-online.png','is-fault':'logo-fault.png','is-estop':'logo-estop.png'}[c]||'logo-offline.png';
  if(!(elLogo.getAttribute('src')||'').endsWith(f)) elLogo.setAttribute('src',f); elLogo.className='logo-badge '+c; }
function renderSpark(){ let p=''; for(let k=0;k<jfill;k++){ const i=(jhead-jfill+k+SPARK_N)%SPARK_N; const v=jbuf[i];
  p+=(k/(SPARK_N-1)*60).toFixed(1)+','+(18-Math.min(v,150)/150*16).toFixed(1)+' '; } sparkLine.setAttribute('points',p.trim()); }

function buildDriveCards(drives, loopRunning){
  while(driveGrid.children.length<drives.length){
    const i=driveGrid.children.length; const c=document.createElement('div'); c.className='dcard';
    c.innerHTML=
      `<div class="dh"><span class="dot"></span><span class="nm">Drive ${i}</span><span class="tag typ"> - </span><span class="tag dir"></span></div>`+
      `<div class="dcfg"> - </div>`+
      `<div class="srow"><span class="stxt"> - </span><span class="sw"></span></div>`+
      `<div class="metrics">`+
        `<div class="mr pos-only"><span class="mk">Pos mm</span><span class="mv c pos"> - </span></div>`+
        `<div class="mr pos-only"><span class="mk">Tgt mm</span><span class="mv tgt"> - </span></div>`+
        `<div class="mr pos-only"><span class="mk">Vel mm/s</span><span class="mv vel"> - </span></div>`+
        `<div class="mr sub pos-only"><span class="mk2">peak</span><span class="mv2 velpk"> - </span></div>`+
        `<div class="mr trq-only"><span class="mk">Cmd %</span><span class="mv cmdtrq"> - </span></div>`+
        `<div class="mr"><span class="mk">Trq %</span><span class="mv trq"> - </span></div>`+
        `<div class="mr sub"><span class="mk2">peak</span><span class="mv2 trqpk"> - </span></div>`+
        `<div class="mr trq-only"><span class="mk">Duty rms 60s</span><span class="mv rms"> - </span></div>`+
        `<div class="mr trq-only"><span class="mk">Speed rpm</span><span class="mv rpmv"> - </span></div>`+
        `<div class="mr trq-only"><span class="mk">IGBT °C</span><span class="mv igbt"> - </span></div>`+
        `<div class="mr trq-only"><span class="mk">Guard</span><span class="mv guardv"> - </span></div>`+
        `<div class="mr pos-only"><span class="mk">Accel mm/s²</span><span class="mv acc"> - </span></div>`+
        `<div class="abar pos-only"><div class="fill"></div><div class="lim"></div></div>`+
        `<div class="mr sub pos-only"><span class="mk2">clamp</span><span class="mv2 clip"> - </span></div>`+
        `<div class="mr pos-only"><span class="mk">Follow err mm</span><span class="mv ferr"> - </span></div>`+
      `</div>`;
    driveGrid.appendChild(c);
  }
  while(driveGrid.children.length>drives.length) driveGrid.removeChild(driveGrid.lastChild);

  drives.forEach((d,i)=>{
    const card=driveGrid.children[i];
    // Status from the shared model (StatusModel via /api/status `ind`): canonical
    // colour class + verbatim text. Proven identical to the former local derivation
    // by TestStatusModel's 210-combo golden-reference sweep. The raw statusword is
    // still shown separately below.
    const ind = d.ind || {};
    const stext = ind.text || ' - ';
    const sclass = ind.cls || 's-off';
    card.className='dcard '+sclass+(sclass==='s-fault'?' fault':'');
    card.querySelector('.nm').textContent  = d.name || ('Drive '+i);
    card.querySelector('.typ').textContent = TYPE[d.axisType] || ' - ';
    const dir=card.querySelector('.dir');
    if(typeof d.invertDir==='boolean'){ dir.textContent=d.invertDir?'REV':'FWD'; dir.style.display=''; }
    else { dir.style.display='none'; }
    // status row: chosen status text + raw statusword hex
    card.querySelector('.stxt').textContent = stext;
    card.querySelector('.sw').textContent   = ('sw' in d) ? hex4(d.sw) : '';
    // config line: stroke · pitch (small, static)
    // Torque axes have no stroke/ballscrew and no meaningful position rows -- hide them
    // (CSS .torque .pos-only) and show the mode instead. TRQ % stays (the useful readout).
    const isTorque = d.mode==='torque';
    card.classList.toggle('torque', isTorque);
    if(isTorque){
      card.querySelector('.dcfg').textContent = 'torque · CST';
      // Torque telemetry rows (see WebServer: cmdTrq/rms/rpm/igbtC/guard).
      const set=(sel,txt)=>{ const e=card.querySelector(sel); if(e) e.textContent=txt; return e; };
      set('.cmdtrq', typeof d.cmdTrq==='number'? d.cmdTrq.toFixed(1) : ' - ');
      const rmsEl=set('.rms', typeof d.rms==='number'? d.rms.toFixed(0)+'%' : ' - ');
      if(rmsEl) rmsEl.style.color = typeof d.rms!=='number' ? ''
        : d.rms>105 ? 'var(--danger)' : d.rms>85 ? 'var(--warn)' : 'var(--ok)';   // vs RATED; >100 = i2t clock
      set('.rpmv', typeof d.rpm==='number'? Math.abs(d.rpm).toFixed(0) : ' - ');
      set('.igbt', typeof d.igbtC==='number'? d.igbtC.toFixed(0) : ' - ');
      const GUARD={overspeed:'OVERSPEED TRIP',travel:'TRAVEL TRIP',relaxed:'RELAXED'};
      const gEl=set('.guardv', GUARD[d.guard]||' - ');
      if(gEl) gEl.style.color = d.guard==='relaxed' ? 'var(--warn)'
                              : d.guard ? 'var(--danger)' : 'var(--ink-soft)';
    }
    else {
      const stroke = (typeof d.strokeMm==='number')? d.strokeMm.toFixed(0)+'mm':' - ';
      const pitch  = (typeof d.ballscrewPitch==='number')? d.ballscrewPitch.toFixed(1)+'mm/rev':'';
      card.querySelector('.dcfg').textContent = pitch? stroke+' · '+pitch : stroke;
    }
    // metrics - current big, peak small (peaks sticky per session)
    const pk=peaks[i]||(peaks[i]={vel:0,trq:0});
    if(typeof d.vel==='number' && Math.abs(d.vel)>pk.vel) pk.vel=Math.abs(d.vel);
    if(typeof d.trq==='number' && d.trq>pk.trq) pk.trq=d.trq;
    card.querySelector('.pos').textContent   = typeof d.pos==='number'? d.pos.toFixed(3):' - ';
    card.querySelector('.tgt').textContent   = typeof d.target==='number'? d.target.toFixed(3):' - ';
    card.querySelector('.vel').textContent   = typeof d.vel==='number'? d.vel.toFixed(2):' - ';
    card.querySelector('.velpk').textContent = typeof d.vel==='number'? pk.vel.toFixed(2):' - ';
    card.querySelector('.trq').textContent   = typeof d.trq==='number'? d.trq.toFixed(1):' - ';
    card.querySelector('.trqpk').textContent = typeof d.trq==='number'? pk.trq.toFixed(1):' - ';
    // Peak WINDOWED commanded accel vs the Amax limit -- the real headroom gauge. It's the
    // macroscopic accel (|dvel| over ~10 ms), NOT per-cycle: the per-cycle value pegs at
    // exactly Amax on every frame-boundary discontinuity, so it read 100% at every Amax.
    // The windowed value dilutes those single-cycle corners and reads BELOW Amax when you
    // have headroom -- so the ratio finally means something. Paired with the accel-clamp
    // clip rate, the braking-clamp bind rate, and peak following-error. All server-latched
    // since loop-start/soft-reset (drive, pause, then read -- motion pauses on focus-loss).
    const accEl=card.querySelector('.acc'), bar=card.querySelector('.abar'), fill=card.querySelector('.fill');
    const amax=(typeof d.maxAccel==='number' && d.maxAccel>0)? d.maxAccel : 0;
    const clip=(typeof d.accelClipPct==='number')? d.accelClipPct : 0;
    const brake=(typeof d.accelBindPct==='number')? d.accelBindPct : 0;
    if(typeof d.accelPeakMms2==='number' && amax){
      const pct=d.accelPeakMms2/amax*100;
      accEl.textContent = `${d.accelPeakMms2.toFixed(0)} / ${amax.toFixed(0)} (${pct.toFixed(0)}%)`;
      fill.style.width = Math.min(pct/125*100,100).toFixed(1)+'%';   // track = 0..125% Amax, limit marked at 80%
      const hot = pct>=95 || clip>=10;   // at the accel limit, or the clamp is binding a lot
      bar.classList.toggle('hot',hot);
      accEl.style.color = hot ? 'var(--danger)' : '';
    } else { accEl.textContent=' - '; accEl.style.color=''; fill.style.width='0%'; bar.classList.remove('hot'); }
    const clipEl=card.querySelector('.clip');
    clipEl.textContent = (typeof d.accelClipPct==='number')? `${clip.toFixed(1)}% clip · ${brake.toFixed(1)}% brake` : ' - ';
    const ferrEl=card.querySelector('.ferr');
    ferrEl.textContent = (typeof d.followErrPeak==='number')? d.followErrPeak.toFixed(3) : ' - ';
  });
}

/* ---- connection (freshness) ---- */
/* Link-lost guard: on Disconnected (>=5s = 10 missed polls) grey EVERY button and show the
   banner, so a dead network path LOOKS dead instead of silently eating clicks against a
   frozen page (eth1 carrier-drop incident). Cleared by the next successful applyState:
   fault/restart/shutdown are restored here (applyState never touches them); the rest are
   recomputed by applyState's own state logic every poll. */
let linkDown=false;
function setLinkDown(d){ if(d===linkDown) return; linkDown=d;
  const b=$('linkBanner'); if(b) b.hidden=!d;
  if(d) Object.values(btn).forEach(x=>{ if(x) x.disabled=true; });
  else ['fault','restart','shutdown'].forEach(k=>{ if(btn[k]) btn[k].disabled=false; });
}
function updateConn(){ const age=lastRecvMs?(Date.now()-lastRecvMs):Infinity;
  if(age<1500){ elConn.textContent='Connected'; elConn.className='badge badge-on'; }
  else if(age<5000){ elConn.textContent='Stale'; elConn.className='badge badge-stale'; }
  else { elConn.textContent='Disconnected'; elConn.className='badge badge-off'; setLogo('is-offline'); setLinkDown(true); } }
setInterval(updateConn,500);
setInterval(()=>{ elClock.textContent=new Date().toTimeString().slice(0,8); },1000);

/* ---- commands ---- */
/* Commands answer {"ok":false,"error":"..."} when the server refuses (already
   operational, busy, e-stop held, cooldown...). Surface that ONE reason right
   above the buttons instead of dropping it: a click that did nothing must say
   why, and the message must never outlive the next click. */
let cmdErrTimer=null;
function showCmdErr(msg){
  const el=$('cmdErr'); if(!el) return;
  el.textContent='✗ '+msg; el.hidden=false;
  if(cmdErrTimer) clearTimeout(cmdErrTimer);
  cmdErrTimer=setTimeout(()=>{ el.hidden=true; },8000);
}
function clearCmdErr(){
  const el=$('cmdErr'); if(el) el.hidden=true;
  if(cmdErrTimer){ clearTimeout(cmdErrTimer); cmdErrTimer=null; }
}
async function postCmd(ep){
  clearCmdErr();
  try{
    const r=await fetch(API+ep,{method:'POST'});
    let j=null; try{ j=await r.json(); }catch(_){}
    if(j && j.ok===false) showCmdErr(j.error||'command refused');
    else if(!r.ok)        showCmdErr('HTTP '+r.status);
  }catch(e){ console.error(ep,e); showCmdErr('no response from the controller'); }
}
btn.init.onclick=()=>{
  if(ecatMode==='init') postCmd('/api/init');
  else if(ecatMode==='deinit' && confirm('Stop EtherCAT?\n\nDrives return to INIT (leave OP). Re-run Initialize to bring them back.')) postCmd('/api/deinit');
};
btn.start.onclick=()=>postCmd('/api/start');
btn.stop.onclick=()=>postCmd('/api/stop'); btn.home.onclick=()=>postCmd('/api/home');
btn.park.onclick=()=>postCmd(parkMode==='unpark'?'/api/unpark':'/api/park'); btn.fault.onclick=()=>postCmd('/api/reset-fault');
btn.belts.onclick=()=>postCmd(beltsSlack?'/api/belts/tension':'/api/belts/slack');
btn.estop.onclick=()=>postCmd('/api/estop'); btn.release.onclick=()=>postCmd('/api/estop/release');
btn.restart.onclick=()=>{ if(confirm('Restart the application?')) postCmd('/api/restart'); };
btn.shutdown.onclick=()=>{ if(confirm('Shut down the Pi?\n\nIt will power OFF - you must switch it back on manually to restart.')) postCmd('/api/shutdown'); };

/* ---- collapsible panels (arrows) ---- */
function toggle(panel, arrow){ const c=panel.classList.toggle('collapsed'); arrow.textContent=c?'▸':'▾'; }
$('cfgHead').onclick=()=>{ const wasCollapsed=$('cfgPanel').classList.contains('collapsed'); toggle($('cfgPanel'),$('cfgArrow')); if(wasCollapsed) loadConfig(); };
$('provHead').onclick=()=>toggle($('provPanel'),$('provArrow'));
// Button bindings: collapsed by default. It is a commissioning-time task, and
// left expanded its full-width panel pushed the config Save bar off screen.
{ const h=$('bindHead'); if(h) h.onclick=()=>toggle($('bindBody'),$('bindArrow')); }
$('logHead').onclick=()=>toggle($('logPanelWrap'),$('logArrow'));
$('logArrow').onclick=()=>toggle($('logPanelWrap'),$('logArrow'));

/* ---- theme select ---- */
const cfTheme=$('cf-theme');
if(cfTheme){ cfTheme.value=theme; cfTheme.onchange=()=>{ theme=cfTheme.value; applyTheme(theme); localStorage.setItem(THEME_KEY,theme); }; }

/* ---- config: split host.json (per-machine) + rig.json (portable).
   The web owns rig on every platform; host only when there is no native config
   UI (hostOwner === "web"). On "native" the host fields are read-only here.
   Restart to apply. ---- */
let cfgObj=null;          // merged working model for the axis editor (host + rig.global + drives)
let meta={hostOwner:'web'};
function setField(id,v){ const el=$(id); if(!el||v==null) return; if(el.type==='checkbox') el.checked=!!v; else el.value=v; }

// host.json inputs - disabled when a native app owns host (hostOwner==="native").
const HOST_INPUT_IDS=['cf-sim','cf-nic','cf-hz','cf-wd','cf-dc','cf-bind','cf-wport','cf-sport','cf-sbind',
  'cf-loglvl','cf-logfile','cf-logcon','cf-diag','cf-temppoll','cf-cmdsync','cf-wkccyc','cf-wkcthr','cf-capscan','cf-gpiomode','cf-ledtest'];
function applyHostOwnership(){
  const native=(meta.hostOwner==='native');
  // Ownership as labeled groups, not mysteriously-greyed fields: every group
  // containing a host-owned input gets a visible "desktop-app managed" badge
  // on native builds.
  for(const id of HOST_INPUT_IDS){ const el=$(id); if(el){ el.disabled=native;
    const g=el.closest('.cgrp'); if(g) g.classList.toggle('host-owned',native); } }
  const note=$('cf-hostnote'); if(note) note.style.display=native?'block':'none';
  // PC only: the button reader runs with the web server, so bound presses need
  // webUIEnabled on. On the Pi the web server is always the control surface.
  const bn=$('btnBindNativeNote'); if(bn) bn.style.display=native?'block':'none';
  const rn=$('cf-rignote'); if(rn) rn.textContent = native
    ? 'Rig & axis settings: portable rig config - saved here, applies on the next Stop → Re-initialize.'
    : 'Rig & axis settings: saved here, applied on service restart.';
}

/* ---- UI state clarity ----
   Three field states, made visible: EDITED-UNSAVED (amber row markers, Save(N),
   leave warning -- client-side vs the loaded snapshot); SAVED-NOT-APPLIED
   (SERVER-owned pending flags from /api/meta -> persistent pill, survives
   reloads and agrees across clients); APPLIED (neither). */
const CFG_WATCH_IDS=[...HOST_INPUT_IDS.filter(id=>id!=='cf-ledtest'),
  'cf-blendt','cf-blendv','cf-condmode','cf-reqreset'];
let cfgBaseline=null;
function fieldVal(el){ return el.type==='checkbox'?el.checked:el.value; }
function snapshotCfg(){
  const f={}; for(const id of CFG_WATCH_IDS){ const el=$(id); if(el) f[id]=fieldVal(el); }
  return { fields:f, drives:((cfgObj&&cfgObj.drives)||[]).map(d=>JSON.stringify(d)) };
}
function dirtyState(){
  const out={count:0, ids:new Set(), axisKeys:new Set()};
  if(!cfgBaseline) return out;
  for(const id of CFG_WATCH_IDS){ const el=$(id); if(!el) continue;
    if(fieldVal(el)!==cfgBaseline.fields[id]){ out.count++; out.ids.add(id); } }
  const drv=(cfgObj&&cfgObj.drives)||[];
  for(let i=0;i<Math.max(drv.length,cfgBaseline.drives.length);i++){
    const base=cfgBaseline.drives[i]?JSON.parse(cfgBaseline.drives[i]):null;
    if(!drv[i]){ out.count++; continue; }           // axis removed
    if(!base){                                       // axis newly added -- every field is unsaved
      for(const k of Object.keys(drv[i])){ out.count++; out.axisKeys.add(i+':'+k); }
      continue;
    }
    for(const k of new Set([...Object.keys(drv[i]),...Object.keys(base)]))
      if(JSON.stringify(drv[i][k])!==JSON.stringify(base[k])){ out.count++; out.axisKeys.add(i+':'+k); }
  }
  return out;
}
function refreshDirtyUI(){
  const d=dirtyState();
  for(const id of CFG_WATCH_IDS){ const el=$(id); if(el){ const row=el.closest('.frow');
    (row||el).classList.toggle('dirty', d.ids.has(id)); } }
  const sel=$('axisSel'); const ai=sel?(+sel.value||0):0;
  document.querySelectorAll('#axisFields [data-k]').forEach(el=>{ const row=el.closest('.frow');
    (row||el).classList.toggle('dirty', d.axisKeys.has(ai+':'+el.dataset.k)); });
  const b=$('cfgSave'); if(b){ b.textContent=d.count?`Save (${d.count} change${d.count>1?'s':''})`:'Save to config.json'; b.disabled=!d.count; }
  window.__cfgDirty=d.count;
}
window.addEventListener('beforeunload',e=>{ if(window.__cfgDirty){ e.preventDefault(); e.returnValue=''; } });
{ const p=$('cfgPanel'); if(p){ p.addEventListener('input',refreshDirtyUI); p.addEventListener('change',refreshDirtyUI); } }
async function refreshPendingPill(fetchMeta){
  if(fetchMeta){ try{ const r=await fetch(API+'/api/meta'); if(r.ok) meta=await r.json(); }catch(_){} }
  const pill=$('cfgPending'); if(!pill) return;
  const pend=!!(meta.rigPendingRestart||meta.hostPendingRestart);
  pill.style.display=pend?'':'none';
  pill.textContent=(meta.hostOwner==='native')?'config saved - re-initialize to apply':'config saved - restart to apply';
}
setInterval(()=>refreshPendingPill(true),15000);

/* ---- Button-binding page ----
   One row per bindable command: current binding + Bind (press-to-capture) +
   Clear. Saves to buttons.json (web-owned on BOTH platforms) and hot-applies.
   The bindable list mirrors Docs/COMMAND_CONTRACT.md "Box v1" -- and the
   server enforces it independently (Config::isBindableCommand). */
/* One button per stateful pair (Tim's ruling): toggles resolve server-side
   from live state, with transition + cooldown guards. Discrete half-tokens
   stay valid in validation (legacy maps / scripts) but aren't listed here. */
const BINDABLE=[
  ['belts-toggle','Tension / Slack belts'],
  ['park-toggle','Park / Unpark'],
  ['run-toggle','Start / Stop loop'],
  ['init-toggle','Initialize / De-init'],
  ['home','Home'],
  ['estop','E-STOP (software)'],
  ['reset-fault','Reset fault']];
let btnMap={configVersion:1,bindings:[]};
let btnDirty=false, btnCapturing=null;
function bindingFor(cmd){ return (btnMap.bindings||[]).find(b=>b.cmd===cmd); }
function setBindStatus(msg,color){ const s=$('btnBindStatus'); if(s){ s.textContent=msg||''; s.style.color=color||'var(--ink-soft)'; } }
function renderBindings(){
  const host=$('btnBindRows'); if(!host) return;
  host.className='';
  host.innerHTML=BINDABLE.map(([cmd,label])=>{
    const b=bindingFor(cmd);
    const cap=btnCapturing===cmd;
    return `<div class="frow bindrow">
      <span>${label}</span>
      <span class="bindcur${b?'':' unbound'}">${cap?'press a button…':(b?(b.label||(b.vendor+':'+b.product+' #'+b.code)):' - ')}</span>
      <button class="btn btn-sm" data-bindcmd="${cmd}" ${btnCapturing?'disabled':''}>${b?'Rebind':'Bind'}</button>
      <button class="btn btn-sm" data-clearcmd="${cmd}" ${(!b||btnCapturing)?'disabled':''}>Clear</button>
    </div>`; }).join('');
  const sv=$('btnBindSave'); if(sv){ sv.disabled=!btnDirty; sv.textContent=btnDirty?'Save bindings (unsaved)':'Save bindings'; }
}
async function loadBindings(){
  try{ const r=await fetch(API+'/api/buttons'); if(r.ok) btnMap=await r.json(); }catch(_){}
  if(!btnMap||typeof btnMap!=='object') btnMap={};
  btnMap.configVersion=btnMap.configVersion||1; btnMap.bindings=btnMap.bindings||[];
  btnDirty=false; renderBindings(); setBindStatus('');
}
async function captureFor(cmd){
  if(btnCapturing) return;
  try{
    const r=await fetch(API+'/api/buttons/listen',{method:'POST'});
    const j=await r.json();
    if(!j.ok){ setBindStatus(j.error||'capture unavailable on this host','var(--danger)'); return; }
  }catch(e){ setBindStatus('capture unavailable: '+e,'var(--danger)'); return; }
  btnCapturing=cmd; renderBindings();
  const t0=Date.now();
  while(Date.now()-t0<15000){
    await new Promise(res=>setTimeout(res,300));
    try{
      const c=await (await fetch(API+'/api/buttons/capture')).json();
      if(c.captured){
        btnMap.bindings=btnMap.bindings.filter(b=>b.cmd!==cmd);
        btnMap.bindings.push({cmd,vendor:c.vendor,product:c.product,code:c.code,label:c.label});
        btnDirty=true; btnCapturing=null; renderBindings();
        setBindStatus('captured - press Save bindings to apply');
        return;
      }
      if(!c.listening&&!c.captured) break;   // backend disarmed (timeout)
    }catch(_){}
  }
  btnCapturing=null; renderBindings(); setBindStatus('no button pressed - capture cancelled','var(--warn)');
}
async function saveBindings(){
  try{
    const r=await fetch(API+'/api/buttons',{method:'POST',body:JSON.stringify(btnMap)});
    const j=await r.json();
    if(j.ok){ btnDirty=false; renderBindings(); setBindStatus('saved ✓ live - no restart needed','var(--ok)'); }
    else setBindStatus('✗ '+(j.error||'save failed'),'var(--danger)');
  }catch(e){ setBindStatus('✗ '+e,'var(--danger)'); }
}
{ const host=$('btnBindRows'); if(host) host.addEventListener('click',e=>{
    const b=e.target.closest('[data-bindcmd]'), c=e.target.closest('[data-clearcmd]');
    if(b) captureFor(b.dataset.bindcmd);
    else if(c){ btnMap.bindings=btnMap.bindings.filter(x=>x.cmd!==c.dataset.clearcmd); btnDirty=true; renderBindings(); }
  });
  const sv=$('btnBindSave'); if(sv) sv.onclick=saveBindings; }

async function loadConfig(){
  try{
    const [mR,hR,rR]=await Promise.all([fetch(API+'/api/meta'),fetch(API+'/api/host'),fetch(API+'/api/rig')]);
    meta=mR.ok?await mR.json():{hostOwner:'web'};
    const host=hR.ok?await hR.json():{};
    const g=(rR.ok?await rR.json():{}); const rig=g.global||{};
    cfgObj=Object.assign({},host,rig,{ drives:g.axes||[], numDrives:(g.axes||[]).length, _configVersion:g.configVersion||2 });
    // host fields (per-machine)
    setField('cf-nic',host.nicName); setField('cf-hz',host.controlLoopHz); setField('cf-wd',host.pdoWatchdogMs);
    setField('cf-dc',host.dcSyncOffsetNs); setField('cf-bind',host.webBindAddr); setField('cf-wport',host.webPort);
    setField('cf-sport',host.telemetryPort); setField('cf-sbind',host.telemetryBindAddr);
    setField('cf-loglvl',host.logMinLevel); setField('cf-logcon',host.logToConsole); setField('cf-diag',host.diagEnabled);
    setField('cf-temppoll',(host.tempPollSec??0)>0);
    setField('cf-sim',host.simulationMode); setField('cf-logfile',host.logFile);
    setField('cf-cmdsync',host.commandSyncCycles); setField('cf-wkccyc',host.wkcValidationCycles);
    setField('cf-wkcthr',host.wkcValidationThreshold); setField('cf-capscan',host.enableCapabilityScan);
    setField('cf-gpiomode',host.gpioMode||'off');
    // rig.global fields (portable feel/policy)
    setField('cf-blendt',rig.blendTimeSec); setField('cf-blendv',rig.blendMaxVelocityMmS);
    setField('cf-condmode',rig.conditioningMode||'bypass'); updateCondNote();
    setField('cf-reqreset',rig.requireUserFaultReset);
    applyHostOwnership();
    populateAxisEditor();
    cfgBaseline=snapshotCfg();   // loaded state = the clean baseline
    refreshDirtyUI();
    refreshPendingPill(false);   // meta already fresh from the fetch above
    loadBindings();
  }catch(_){}
}
// Clamp every numeric axis field to its (possibly dynamic) declared range before save.
function clampDriveFields(){
  if(!cfgObj||!cfgObj.drives) return 0; let n=0;
  for(const d of cfgObj.drives){ for(const f of AXIS_SPEC){
    if(f.type!=='num'||!axisApplicable(f,d)) continue;
    let v=+d[f.k]; if(!isFinite(v)) continue;
    const emin=f.dynMin?f.dynMin(d,cfgObj):f.min, emax=f.dynMax?f.dynMax(d,cfgObj):f.max;
    if(emin!=null&&v<emin) v=emin; if(emax!=null&&v>emax) v=emax;
    if(v!==d[f.k]){ d[f.k]=v; n++; }
  }} return n;
}
async function saveConfig(){
  const st=$('cfgStatus'); if(!cfgObj) cfgObj={};
  const clamped=clampDriveFields();
  if(clamped){ const s=$('axisSel'); if(s) renderAxisFields(+s.value||0); }
  // rig.json (web owns it on both platforms)
  const g={ blendTimeSec:+$('cf-blendt').value, blendMaxVelocityMmS:+$('cf-blendv').value,
            conditioningMode:$('cf-condmode').value, requireUserFaultReset:$('cf-reqreset').checked };
  cfgObj.conditioningMode=g.conditioningMode;   // keep merged view in sync for axis logic
  const rig={ configVersion:cfgObj._configVersion||2, numDrives:(cfgObj.drives||[]).length, global:g, axes:cfgObj.drives||[] };
  st.textContent='Saving…'; st.style.color='var(--ink-soft)';
  try{
    const rr=await fetch(API+'/api/rig',{method:'POST',body:JSON.stringify(rig)}); const rj=await rr.json();
    if(!rj.ok){ st.textContent='✗ rig: '+(rj.error||'save failed'); st.style.color='var(--danger)'; return; }
    // host.json - only when the web owns it (headless); on "native" the desktop app owns it.
    if(meta.hostOwner==='web'){
      const host={ configVersion:cfgObj._configVersion||2,
        nicName:$('cf-nic').value, controlLoopHz:+$('cf-hz').value, pdoWatchdogMs:+$('cf-wd').value,
        dcSyncOffsetNs:+$('cf-dc').value, webBindAddr:$('cf-bind').value, webPort:+$('cf-wport').value,
        telemetryPort:+$('cf-sport').value, telemetryBindAddr:$('cf-sbind').value,
        logMinLevel:$('cf-loglvl').value, logToConsole:$('cf-logcon').checked, diagEnabled:$('cf-diag').checked,
        /* tickbox semantics: off -> 0 (no polling); on -> keep the file's custom
           period if it had one, else the 15s default. Finer control via host.json. */
        tempPollSec:$('cf-temppoll').checked ? ((cfgObj&&cfgObj.tempPollSec>0)?cfgObj.tempPollSec:15) : 0,
        simulationMode:$('cf-sim').checked, logFile:$('cf-logfile').value,
        commandSyncCycles:+$('cf-cmdsync').value, wkcValidationCycles:+$('cf-wkccyc').value,
        wkcValidationThreshold:+$('cf-wkcthr').value, enableCapabilityScan:$('cf-capscan').checked,
        gpioMode:$('cf-gpiomode').value, gpioEnabled:($('cf-gpiomode').value!=='off') };
      const hr=await fetch(API+'/api/host',{method:'POST',body:JSON.stringify(host)}); const hj=await hr.json();
      if(!hj.ok){ st.textContent='✗ host: '+(hj.error||'save failed'); st.style.color='var(--danger)'; return; }
    }
    // Native (desktop) reloads config in-process on save (QFileSystemWatcher) -     // no app restart; it applies live when EtherCAT is offline, or on the next
    // Stop→Re-initialize. Headless (hostOwner 'web', systemd) still needs a restart.
    const applyMsg=(meta.hostOwner==='native')?'Saved ✓ Re-initialize EtherCAT to apply (no app restart).':'Saved ✓ Restart to apply.';
    st.textContent=applyMsg+(clamped?` (${clamped} field${clamped>1?'s':''} clamped to safe range)`:''); st.style.color='var(--ok)';
    cfgBaseline=snapshotCfg(); refreshDirtyUI();   // saved = new clean baseline
    refreshPendingPill(true);                      // server now reports pending-restart
  }catch(e){ st.textContent='✗ '+e; st.style.color='var(--danger)'; }
}
/* ---- per-axis editor: writes into cfgObj.drives[i]; saved by saveConfig ---- */
// pos:true  = position/homing param -> hidden in TORQUE (CST) mode (a belt in CSP mode
//             still needs rotation, ratio, stroke, homing, so gate on mode not on type).
// belt:true = shown only for axisType 'belt'. torque:true = only in torque mode.
const AXIS_SPEC=[
  {k:'name',label:'Name',type:'text'},
  {k:'axisType',label:'Type',type:'select',opts:['linear_vertical','linear_horizontal','belt']},
  {k:'invertDir',label:'Invert dir',type:'bool',pos:true},
  {k:'mode',label:'Drive mode',type:'select',opts:['csp','pp','torque']},
  {k:'strokeMm',label:'Stroke',type:'num',min:1,max:2000,step:0.1,unit:'mm',pos:true},
  {k:'ballscrewPitch',label:'Screw pitch',type:'num',min:0.5,max:50,step:0.01,unit:'mm/rev',pos:true},
  {k:'encoderCountsPerRev',label:'Enc counts/rev',type:'num',min:1,max:100000000,step:1,text:true,pos:true},
  // Reduction shown in torque mode too: the strap sees motor torque x ratio, so it is
  // part of the SAFETY picture there (strap-side note + validation cap use it).
  {k:'reductionRatio',label:'Reduction',type:'select',opts:['1:1','1.5:1','2:1','3:1','4:1']},
  {k:'homeDirection',label:'Home dir',type:'select',opts:['negative','positive'],pos:true},
  {k:'homeMode',label:'Home mode',type:'select',opts:['center','endstop'],pos:true},
  {k:'homingBackoffMm',label:'Home backoff',type:'num',min:0.1,max:20,step:0.01,unit:'mm',pos:true},
  {k:'homingSpeedMmS',label:'Home speed',type:'num',min:1,max:5000,step:1,unit:'cmd',pos:true},
  {k:'homingTorquePct',label:'Home torque',type:'num',min:5,max:100,step:1,unit:'%',pos:true},
  {k:'maxVelocityMmS',label:'Max vel',type:'num',min:1,max:10000,step:1,unit:'mm/s',pos:true},
  {k:'maxAccelerationMmS2',label:'Max accel',type:'num',min:1,max:100000,step:1,unit:'mm/s²',pos:true},
  {k:'followingErrorWindowMm',label:'Following-err window',type:'num',min:0,max:10000,step:0.1,unit:'mm',pos:true},
  {k:'trackingWnHz',label:'Filter knee',type:'num',min:5,dynMax:(d,c)=>Math.min(125,Math.floor((c.controlLoopHz||2000)/2)),step:0.5,unit:'Hz',csp:true,filterOnly:true},
  {k:'unparkTimeSec',label:'Unpark time',type:'num',min:0.5,max:30,step:0.1,unit:'s'},
  {k:'parkTimeSec',label:'Park time',type:'num',min:0.5,max:30,step:0.1,unit:'s'},
  {k:'spikeFilterEnabled',label:'Spike filter',type:'bool',pos:true},
  {k:'spikeMaxMm',label:'Spike max',type:'num',min:0.1,max:500,step:0.01,unit:'mm/cyc',pos:true},
  {k:'torqueMinPct',label:'Torque min',type:'num',min:0,max:300,step:1,unit:'%',torque:true},
  {k:'torqueMaxPct',label:'Torque max',type:'num',min:0,max:300,step:1,unit:'%',torque:true},
  {k:'beltSlewPctPerSec',label:'Slew cap',type:'num',min:100,max:20000,step:100,unit:'%/s',torque:true},
  {k:'beltOverspeedRpm',label:'Overspeed',type:'num',min:50,max:6000,step:10,unit:'rpm',torque:true},
  {k:'beltOverspeedMs',label:'Overspeed time',type:'num',min:20,max:5000,step:10,unit:'ms',torque:true},
  {k:'beltMaxTravelRevs',label:'Travel cap (0=off)',type:'num',min:0,max:100,step:0.5,unit:'revs',torque:true},
  {k:'beltMaxRpm',label:'Speed fold (0=off)',type:'num',min:0,max:3000,step:50,unit:'rpm',torque:true},
  {k:'beltRelaxerSec',label:'Relaxer (0=off)',type:'num',min:0,max:120,step:1,unit:'s',torque:true},
  {k:'beltRelaxerPct',label:'Relaxer band',type:'num',min:20,max:100,step:1,unit:'%',torque:true},
];
function condMode(){ return (cfgObj&&cfgObj.conditioningMode)||'bypass'; }
function axisApplicable(f,d){ if(f.pos&&d.mode==='torque')return false; if(f.belt&&d.axisType!=='belt')return false; if(f.torque&&d.mode!=='torque')return false; if(f.csp&&d.mode!=='csp')return false; if(f.filterOnly&&condMode()!=='filter')return false; return true; }
function populateAxisEditor(){
  const sel=$('axisSel'); if(!sel) return; const drives=(cfgObj&&cfgObj.drives)||[];
  sel.innerHTML = drives.length ? drives.map((d,i)=>`<option value="${i}">Axis ${i} - ${d.name||('Drive '+i)}</option>`).join('') : '<option value="-1">no drives in config</option>';
  renderAxisFields(+sel.value||0);
}
function renderAxisFields(i){
  const host=$('axisFields'); if(!host) return; const d=((cfgObj&&cfgObj.drives)||[])[i];
  if(!d){ host.innerHTML='<p class="cfg-note">No drive at this index.</p>'; return; }
  let h='';
  for(const f of AXIS_SPEC){ if(!axisApplicable(f,d)) continue; const v=d[f.k];
    if(f.type==='bool') h+=`<label class="frow"><span>${f.label}</span><input type="checkbox" data-k="${f.k}" ${v?'checked':''}></label>`;
    else if(f.type==='select') h+=`<label class="frow"><span>${f.label}</span><select data-k="${f.k}">${f.opts.map(o=>`<option ${o===v?'selected':''}>${o}</option>`).join('')}</select></label>`;
    else if(f.type==='text') h+=`<label class="frow"><span>${f.label}</span><input type="text" data-k="${f.k}" value="${v??''}"></label>`;
    else { const it=f.text?'text':'number';   // f.text → no spinner arrows (e.g. encoder)
      const emin=f.dynMin?f.dynMin(d,cfgObj):f.min, emax=f.dynMax?f.dynMax(d,cfgObj):f.max;
      h+=`<label class="frow"><span>${f.label}</span><input type="${it}" inputmode="decimal" data-k="${f.k}" data-num="1" value="${v??''}"${(!f.text&&emin!=null)?` min="${emin}"`:''}${(!f.text&&emax!=null)?` max="${emax}"`:''}${(!f.text&&f.step!=null)?` step="${f.step}"`:''}>${f.unit?`<span class="u">${f.unit}</span>`:''}</label>`; }
  }
  // Filter-knee consequences, live next to the knob (Filter mode only).
  if(d.mode==='csp' && condMode()==='filter' && typeof d.trackingWnHz==='number' && d.maxAccelerationMmS2){
    const wn=2*Math.PI*d.trackingWnHz;
    h+=`<div class="cfg-note" style="grid-column:1/-1">filter knee ${d.trackingWnHz} Hz → group delay ≈ ${(2/wn*1000).toFixed(1)} ms, no-overshoot regime |err| &lt; ${(d.maxAccelerationMmS2/(wn*wn)).toFixed(2)} mm (braking clamp covers larger)</div>`; }
  if(d.mode!=='torque'&&typeof d.encoderCountsPerRev==='number'&&d.ballscrewPitch){ const rf=parseFloat((d.reductionRatio||'1:1').split(':')[0])||1;
    h+=`<div class="cfg-note" style="grid-column:1/-1">counts/mm = ${(d.encoderCountsPerRev*rf/d.ballscrewPitch).toFixed(1)}${rf!==1?' (incl. '+d.reductionRatio+')':''} (recomputed on save)</div>`; }
  // Torque mode: strap-side ceiling = motor torque x reduction. Re-derive torqueMax when
  // the ratio changes; config validation rejects maxPct x ratio > 300% of rated.
  if(d.mode==='torque'){ const rf=parseFloat((d.reductionRatio||'1:1').split(':')[0])||1;
    const strap=(d.torqueMaxPct||0)*rf, over=strap>300;
    h+=`<div class="cfg-note" style="grid-column:1/-1${over?';color:var(--danger)':''}">strap-side max = ${d.torqueMaxPct||0}% × ${d.reductionRatio||'1:1'} = <b>${strap.toFixed(0)}%</b> of rated motor torque${over?' - EXCEEDS 300% cap, reduce Torque max':''}. Overspeed guard is motor-side rpm (scales ×${rf} for the same strap speed).</div>`; }
  host.innerHTML=h;
  host.querySelectorAll('[data-k]').forEach(el=>{ el.onchange=()=>{ const k=el.dataset.k, dd=cfgObj.drives[i];
    dd[k] = el.type==='checkbox'?el.checked : el.dataset.num?(+el.value) : el.value;
    // re-render so the filter-knee note tracks accel/vel/knee/mode changes
    if(['axisType','mode','trackingWnHz','maxAccelerationMmS2','maxVelocityMmS','torqueMaxPct','reductionRatio'].includes(k)) renderAxisFields(i); }; });
  refreshDirtyUI();   // re-apply amber markers on the freshly rendered fields
}
{ const s=$('axisSel'); if(s) s.onchange=()=>renderAxisFields(+s.value||0); }
// Command-conditioning mode selector: one-liner + show/hide the Filter knee live.
const COND_NOTE={
  bypass:"Raw, lowest latency. Best with a smooth high-rate host stream (e.g. DRSM/SimHub ~1000 Hz).",
  interpolate:"Smooth gap-fill between frames, full detail. Best for a low UDP send rate (latency ~1 frame).",
  filter:"Feel-shaping smoothing. Tames aggressive sources (FlyPT) and reduces motion sickness. Adds latency (2/wn)."
};
function updateCondNote(){ const e=$('cf-condnote'); if(e) e.textContent=COND_NOTE[condMode()]||''; }
{ const m=$('cf-condmode'); if(m) m.onchange=()=>{ if(cfgObj) cfgObj.conditioningMode=m.value; updateCondNote(); const s=$('axisSel'); if(s) renderAxisFields(+s.value||0); }; }
function addAxis(){
  if(!cfgObj) return; cfgObj.drives=cfgObj.drives||[];
  if(cfgObj.drives.length>=10) return;
  const base=cfgObj.drives[cfgObj.drives.length-1]||{};
  const n=JSON.parse(JSON.stringify(base));
  n.slaveIndex=cfgObj.drives.length+1; n.name='Drive '+(cfgObj.drives.length+1);
  cfgObj.drives.push(n); cfgObj.numDrives=cfgObj.drives.length;
  populateAxisEditor(); const s=$('axisSel'); s.value=String(cfgObj.drives.length-1); renderAxisFields(cfgObj.drives.length-1);
}
function removeAxis(){
  if(!cfgObj||!cfgObj.drives||cfgObj.drives.length<=1) return;
  cfgObj.drives.splice(+$('axisSel').value||0,1); cfgObj.numDrives=cfgObj.drives.length;
  populateAxisEditor();
}
function applyToAll(){
  if(!cfgObj||!cfgObj.drives) return; const i=+$('axisSel').value||0; const src=cfgObj.drives[i]; if(!src) return;
  cfgObj.drives.forEach((d,j)=>{ if(j===i) return; const nm=d.name, sl=d.slaveIndex;
    Object.assign(d, JSON.parse(JSON.stringify(src))); d.name=nm; d.slaveIndex=sl; });   // keep identity
  const st=$('cfgStatus'); if(st){ st.textContent='Applied axis '+i+' to all axes (name + slaveIndex kept) - Save to persist'; st.style.color='var(--ink-soft)'; }
}
{ const a=$('axisAdd'), d=$('axisDel'), ap=$('axisApplyAll'); if(a)a.onclick=addAxis; if(d)d.onclick=removeAxis; if(ap)ap.onclick=applyToAll; }

$('cfgSave').onclick=saveConfig;
{ const lt=$('cf-ledtest'); if(lt) lt.onclick=()=>postCmd('/api/gpio/ledtest'); }
// Reset peaks: clear the server-latched tuning metrics AND the client-side sticky
// vel/trq peaks together, so one action re-baselines every card. Drives stay in OP.
{ const rs=$('btn-reset-stats'); if(rs) rs.onclick=()=>{
    for(const k in peaks) delete peaks[k];
    postCmd('/api/resetstats');
  }; }
loadConfig();

/* ---- log (clear sticks; no refill) ---- */
const logPanel=$('log-panel'), chkTail=$('chk-tail');
let serverNewest='', lastRendered='';
$('btn-clear-log').onclick=(e)=>{ e.stopPropagation(); logPanel.innerHTML=''; lastRendered=serverNewest; };
async function pollLogs(){
  try{ const r=await fetch(API+'/api/logs?n=200'); if(!r.ok) return;
    const lines=await r.json(); if(!lines.length) return;
    serverNewest=lines[lines.length-1];
    let start=0; if(lastRendered){ const idx=lines.lastIndexOf(lastRendered); start=idx>=0?idx+1:0; }
    const fresh=lines.slice(start); if(!fresh.length) return;
    lastRendered=lines[lines.length-1];
    const frag=document.createDocumentFragment();
    for(const line of fresh){ const lo=line.toLowerCase(); let cls='log-line';
      if(lo.includes('[critical]')) cls+=' critical'; else if(lo.includes('[error')) cls+=' error'; else if(lo.includes('[warning')) cls+=' warn';
      const div=document.createElement('div'); div.className=cls; div.textContent=line; frag.appendChild(div); }
    logPanel.appendChild(frag);
    if(chkTail.checked) logPanel.scrollTop=logPanel.scrollHeight;
  }catch(_){}
}
setInterval(pollLogs,500);

/* ---- wheel-guard: scrolling must not nudge a focused number field (Qt parity) ---- */
window.addEventListener('wheel', ()=>{ const a=document.activeElement; if(a&&a.tagName==='INPUT'&&a.type==='number') a.blur(); }, {passive:true});

/* ---- boot ---- */
updateConn(); pollStatus(); setInterval(pollStatus, 500); pollLogs();
