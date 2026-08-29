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

let lastRecvMs=0, jpeak=0, ecatMode='init', parkBtnMode='park', beltsSlack=false;  // ecatMode/parkBtnMode/beltsSlack: what those buttons do now (parkBtnMode is button state, NOT the parkMode config field)
const SPARK_N=60; const jbuf=new Float32Array(SPARK_N); let jhead=0, jfill=0;
const peaks={};   // per-drive sticky {vel,trq}

const TYPE={ linear_vertical:'VERT', linear_horizontal:'HORIZ', belt:'BELT', rotary_lever:'ROT',
             shifter:'SHIFT', pedal:'PEDAL' };
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
  devPoll(s);
  driveCount.textContent = (s.numDrives ?? (s.drives||[]).length)+' axes';
  // commissioning panel needs the axis list + loop state
  tstDrives = s.drives||[]; tstLoopRunning = running; refreshTestButtons();

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
  parkBtnMode=parked?'unpark':'park';
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
      `<div class="flt" style="display:none"></div>`+
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
    // decoded fault line: precise Er name once the 0x203F read lands, else
    // the coarse 603F class. Hidden entirely when not faulted.
    const fEl=card.querySelector('.flt');
    if(fEl){
      if(ind.fault && (ind.faultCode||ind.faultText)){
        fEl.textContent=[ind.faultCode,ind.faultText].filter(Boolean).join(' · ');
        fEl.style.display='';
      } else fEl.style.display='none';
    }
    // config line: stroke · pitch (small, static)
    // Torque axes have no stroke/ballscrew and no meaningful position rows -- hide them
    // (CSS .torque .pos-only) and show the mode instead. TRQ % stays (the useful readout).
    const isTorque = d.mode==='torque';
    card.classList.toggle('torque', isTorque);
    // Rotary lever: metric row labels read in degrees. Swapped once per
    // kind change (dataset flag), not per poll.
    const rotCard = d.axisType==='rotary_lever' ? '1':'0';
    if(card.dataset.rot!==rotCard){
      card.dataset.rot=rotCard;
      const rot = rotCard==='1';
      const lbl=(sel,lin,deg)=>{ const el=card.querySelector(sel); if(el){ const mk=el.parentElement.querySelector('.mk'); if(mk) mk.textContent=rot?deg:lin; } };
      lbl('.mv.pos','Pos mm','Pos °'); lbl('.mv.tgt','Tgt mm','Tgt °');
      lbl('.mv.vel','Vel mm/s','Vel °/s'); lbl('.mv.acc','Accel mm/s²','Accel °/s²');
      lbl('.mv.ferr','Follow err mm','Follow err °');
    }
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
    else if(d.axisType==='rotary_lever'){
      const arc = (typeof d.strokeMm==='number')? d.strokeMm.toFixed(0)+'°':' - ';
      card.querySelector('.dcfg').textContent = d.reductionRatio? arc+' · '+d.reductionRatio : arc;
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
btn.park.onclick=()=>postCmd(parkBtnMode==='unpark'?'/api/unpark':'/api/park'); btn.fault.onclick=()=>postCmd('/api/reset-fault');
btn.belts.onclick=()=>postCmd(beltsSlack?'/api/belts/tension':'/api/belts/slack');
btn.estop.onclick=()=>postCmd('/api/estop'); btn.release.onclick=()=>postCmd('/api/estop/release');
btn.restart.onclick=()=>{ if(confirm('Restart the application?')) postCmd('/api/restart'); };
btn.shutdown.onclick=()=>{ if(confirm('Shut down the Pi?\n\nIt will power OFF - you must switch it back on manually to restart.')) postCmd('/api/shutdown'); };

/* ---- collapsible panels (arrows) ---- */
function toggle(panel, arrow){ const c=panel.classList.toggle('collapsed'); arrow.textContent=c?'▸':'▾'; }
$('cfgHead').onclick=()=>{ const wasCollapsed=$('cfgPanel').classList.contains('collapsed'); toggle($('cfgPanel'),$('cfgArrow')); if(wasCollapsed) loadConfig(); };
$('provHead').onclick=()=>toggle($('provPanel'),$('provArrow'));
$('testHead').onclick=()=>{ const was=$('testPanel').classList.contains('collapsed');
  toggle($('testPanel'),$('testArrow')); if(was){ buildTestAxes(); pollTestStatus(); } };
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
  'cf-loglvl','cf-logfile','cf-logcon','cf-diag','cf-temppoll','cf-cmdsync','cf-wkccyc','cf-wkcthr','cf-capscan','cf-gpiomode','cf-ledtest',
  'cf-showdev'];
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
    updInit();
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
    setField('cf-showdev',host.webShowDevices);
    // rig.global fields (portable feel/policy)
    setField('cf-blendt',rig.blendTimeSec); setField('cf-blendv',rig.blendMaxVelocityMmS);
    setField('cf-condmode',rig.conditioningMode||'bypass'); updateCondNote();
    setField('cf-reqreset',rig.requireUserFaultReset);
    applyHostOwnership();
    devInit();
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
    // axisApplicable also skips fields hidden for this axis kind (e.g. the
    // pitch field on a rotary axis, which is pinned to 360 internally --
    // clamping it to the linear 50mm max would wreck the degree scaling).
    if(f.type!=='num'||!axisApplicable(f,d)) continue;
    let v=+d[f.k]; if(!isFinite(v)) continue;
    const e=effSpec(f,d);
    const emin=f.dynMin?f.dynMin(d,cfgObj):e.min, emax=f.dynMax?f.dynMax(d,cfgObj):e.max;
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
        gpioMode:$('cf-gpiomode').value, gpioEnabled:($('cf-gpiomode').value!=='off'),
        webShowDevices:$('cf-showdev').checked };
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
// Rotary lever axes (crank-arm 6DOF etc): the engineering unit is degrees at
// the lever shaft. Internally that is expressed by ballscrewPitch=360 ("360
// units per output rev"), forced automatically when the type is selected and
// hidden from the editor (hideRot). Per-field `rot:{...}` overrides supply
// the degree labels/ranges; any 'mm' in a unit string is displayed as
// degrees for rotary axes automatically.
const AXIS_SPEC=[
  {k:'name',label:'Name',type:'text'},
  {k:'axisType',label:'Type',type:'select',opts:['linear_vertical','linear_horizontal','rotary_lever','belt']},
  {k:'invertDir',label:'Foldback',type:'bool',pos:true,rot:{label:'Invert direction'}},
  {k:'mode',label:'Drive mode',type:'select',opts:['csp','pp','torque']},
  {k:'strokeMm',label:'Stroke',type:'num',min:1,max:2000,step:0.1,unit:'mm',pos:true,
     rot:{label:'Arc travel',max:360}},
  {k:'ballscrewPitch',label:'Screw pitch',type:'num',min:0.5,max:50,step:0.01,unit:'mm/rev',pos:true,hideRot:true},
  {k:'encoderCountsPerRev',label:'Enc counts/rev',type:'num',min:1,max:100000000,step:1,text:true,pos:true},
  // Reduction shown in torque mode too: the strap sees motor torque x ratio, so it is
  // part of the SAFETY picture there (strap-side note + validation cap use it).
  // Rotary rigs use gearboxes: extended ratio list (a value already in the
  // file that is not listed is injected into the dropdown, never clobbered).
  {k:'reductionRatio',label:'Reduction',type:'select',opts:['1:1','1.5:1','2:1','3:1','4:1'],
     rot:{label:'Gear ratio',opts:['1:1','3:1','5:1','10:1','15:1','20:1','30:1','40:1','50:1','80:1','100:1']}},
  {k:'homeDirection',label:'Home stop (neg=retracted)',type:'select',opts:['negative','positive'],pos:true},
  {k:'parkMode',label:'Park position',type:'select',opts:['center','endstop'],pos:true},
  {k:'homingBackoffMm',label:'Home backoff',type:'num',min:0.1,max:20,step:0.01,unit:'mm',pos:true,
     rot:{max:45}},
  {k:'homingSpeed',label:'Home speed',type:'num',min:1,max:5000,step:1,unit:'cmd',pos:true},
  {k:'homingTorquePct',label:'Home torque',type:'num',min:5,max:100,step:1,unit:'%',pos:true},
  {k:'maxVelocityMmS',label:'Max vel',type:'num',min:1,max:10000,step:1,unit:'mm/s',pos:true},
  // (rotary: mm/s reads as deg/s via the automatic unit swap; ranges shared)
  {k:'maxAccelerationMmS2',label:'Max accel',type:'num',min:1,max:100000,step:1,unit:'mm/s²',pos:true},
  // Rotary cap = the axis's own arc: a window wider than the travel can
  // never trip, so the limit is geometry, not opinion. Linear keeps 10000.
  {k:'followingErrorWindowMm',label:'Following-err window',type:'num',min:0,max:10000,step:0.1,unit:'mm',pos:true,
     dynMax:(d)=>isRot(d)?(+d.strokeMm||360):10000},
  {k:'trackingWnHz',label:'Filter knee',type:'num',min:5,dynMax:(d,c)=>Math.min(125,Math.floor((c.controlLoopHz||2000)/2)),step:0.5,unit:'Hz',csp:true,filterOnly:true},
  {k:'unparkTimeSec',label:'Unpark time',type:'num',min:0.5,max:30,step:0.1,unit:'s'},
  {k:'parkTimeSec',label:'Park time',type:'num',min:0.5,max:30,step:0.1,unit:'s'},
  {k:'spikeFilterEnabled',label:'Spike filter',type:'bool',pos:true},
  {k:'spikeMaxMm',label:'Spike max',type:'num',min:0.1,max:500,step:0.01,unit:'mm/cyc',pos:true},
  {k:'torqueMinPct',label:'Torque min',type:'num',min:0,max:300,step:1,unit:'%',torque:true,tip:'Floor tension while tracking - the belt stays snug at zero demand. Raise for a firmer resting hold.'},
  {k:'torqueMaxPct',label:'Torque max',type:'num',min:0,max:300,step:1,unit:'%',torque:true,tip:'Tension at full-scale telemetry - the overall strength of the effect. The drive 0x6072 limit still caps above it.'},
  {k:'beltSlewPctPerSec',label:'Slew cap',type:'num',min:100,max:20000,step:100,unit:'%/s',torque:true,tip:'How fast tension may change: lower = softer, laggier haptics; higher = snappier. Also stretches a single garbage frame instead of letting it snap the belt. 3000 passes every real effect.'},
  {k:'beltOverspeedRpm',label:'Overspeed',type:'num',min:50,max:6000,step:10,unit:'rpm',torque:true,tip:'Snapped-belt detector: shaft speed sustained above this trips to slack. Set above any speed a worn strap can reach in use.'},
  {k:'beltOverspeedMs',label:'Overspeed time',type:'num',min:20,max:5000,step:10,unit:'ms',torque:true,tip:'How long the overspeed must persist before tripping - haptic flicks and hand pulls reset it.'},
  {k:'beltMaxTravelRevs',label:'Travel cap (0=off)',type:'num',min:0,max:100,step:0.5,unit:'revs',torque:true,tip:'Net winding since tension-up. Catches the SLOW runaway an rpm limit never sees (bare shaft idling along). A strapped belt cannot reach it at any speed.'},
  {k:'beltMaxRpm',label:'Speed fold (0=off)',type:'num',min:0,max:3000,step:50,unit:'rpm',torque:true,tip:'THE enforced speed limit on these drives (their own CST speed objects do not restrain): tension folds to zero above this, capping the slack take-up lunge.'},
  {k:'beltRelaxerSec',label:'Relaxer (0=off)',type:'num',min:0,max:120,step:1,unit:'s',torque:true,tip:'Sustained near-max tension for this long eases to min until demand drops - pre-empts the drive thermal fault that would park the whole rig.'},
  {k:'beltRelaxerPct',label:'Relaxer band',type:'num',min:20,max:100,step:1,unit:'%',torque:true,tip:'Fraction of Torque max that counts as near-max for the relaxer dwell.'},
];
function condMode(){ return (cfgObj&&cfgObj.conditioningMode)||'bypass'; }
const isRot=d=>d&&d.axisType==='rotary_lever';
// Effective field spec for this drive: rotary overrides win, and any 'mm'
// in a unit string displays as degrees on a rotary axis.
function effSpec(f,d){
  const r=(isRot(d)&&f.rot)||{};
  const unit=r.unit??(isRot(d)&&f.unit?f.unit.replace(/mm/g,'°'):f.unit);
  return { label:r.label??f.label, unit,
           min:r.min??f.min, max:r.max??f.max, step:r.step??f.step,
           opts:r.opts??f.opts };
}
function axisApplicable(f,d){ if(f.hideRot&&isRot(d))return false; if(f.pos&&d.mode==='torque')return false;
  // Device axes (shifter/pedal) are torque-mode but NOT belts: the belt
  // tension/guard fields are never consumed for them (device guards live
  // in the device object, edited in the Devices section) - mode alone
  // must not surface them here.
  if(isDeviceType(d.axisType)&&(f.torque||f.belt))return false;
  if(f.belt&&d.axisType!=='belt')return false; if(f.torque&&d.mode!=='torque')return false; if(f.csp&&d.mode!=='csp')return false; if(f.filterOnly&&condMode()!=='filter')return false; return true; }
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
    const e=effSpec(f,d);
    if(f.type==='bool') h+=`<label class="frow"><span>${e.label}</span><input type="checkbox" data-k="${f.k}" ${v?'checked':''}></label>`;
    else if(f.type==='select'){
      // A file value not in the option list is injected, never clobbered
      // (e.g. a hand-edited 63:1 gear ratio must display and survive).
      const opts=(v!=null&&v!==''&&!e.opts.includes(v))?[v,...e.opts]:e.opts;
      h+=`<label class="frow"><span>${e.label}</span><select data-k="${f.k}">${opts.map(o=>`<option ${o===v?'selected':''}>${o}</option>`).join('')}</select></label>`;
    }
    else if(f.type==='text') h+=`<label class="frow"><span>${e.label}</span><input type="text" data-k="${f.k}" value="${v??''}"></label>`;
    else { const it=f.text?'text':'number';   // f.text → no spinner arrows (e.g. encoder)
      const emin=f.dynMin?f.dynMin(d,cfgObj):e.min, emax=f.dynMax?f.dynMax(d,cfgObj):e.max;
      h+=`<label class="frow"><span>${e.label}</span><input type="${it}" inputmode="decimal" data-k="${f.k}" data-num="1" value="${v??''}"${(!f.text&&emin!=null)?` min="${emin}"`:''}${(!f.text&&emax!=null)?` max="${emax}"`:''}${(!f.text&&e.step!=null)?` step="${e.step}"`:''}>${e.unit?`<span class="u">${e.unit}</span>`:''}</label>`; }
    // Feel/behaviour one-liner under fields that carry one (the belt
    // guards especially: what each knob does to the belt, not just units).
    if(f.tip) h+=`<div class="fldtip">${f.tip}</div>`;
  }
  // Filter-knee consequences, live next to the knob (Filter mode only).
  if(d.mode==='csp' && condMode()==='filter' && typeof d.trackingWnHz==='number' && d.maxAccelerationMmS2){
    const wn=2*Math.PI*d.trackingWnHz;
    h+=`<div class="cfg-note" style="grid-column:1/-1">filter knee ${d.trackingWnHz} Hz → group delay ≈ ${(2/wn*1000).toFixed(1)} ms, no-overshoot regime |err| &lt; ${(d.maxAccelerationMmS2/(wn*wn)).toFixed(2)} mm (braking clamp covers larger)</div>`; }
  if(d.mode!=='torque'&&typeof d.encoderCountsPerRev==='number'&&d.ballscrewPitch){ const rf=parseFloat((d.reductionRatio||'1:1').split(':')[0])||1;
    h+=`<div class="cfg-note" style="grid-column:1/-1">counts/${isRot(d)?'°':'mm'} = ${(d.encoderCountsPerRev*rf/d.ballscrewPitch).toFixed(1)}${rf!==1?' (incl. '+d.reductionRatio+')':''} (recomputed on save)</div>`; }
  // Torque mode: strap-side ceiling = motor torque x reduction. Re-derive torqueMax when
  // the ratio changes; config validation rejects maxPct x ratio > 300% of rated.
  if(d.mode==='torque'&&!isDeviceType(d.axisType)){ const rf=parseFloat((d.reductionRatio||'1:1').split(':')[0])||1;
    const strap=(d.torqueMaxPct||0)*rf, over=strap>300;
    h+=`<div class="cfg-note" style="grid-column:1/-1${over?';color:var(--danger)':''}">strap-side max = ${d.torqueMaxPct||0}% × ${d.reductionRatio||'1:1'} = <b>${strap.toFixed(0)}%</b> of rated motor torque${over?' - EXCEEDS 300% cap, reduce Torque max':''}. Overspeed guard is motor-side rpm (scales ×${rf} for the same strap speed).</div>`; }
  host.innerHTML=h;
  host.querySelectorAll('[data-k]').forEach(el=>{ el.onchange=()=>{ const k=el.dataset.k, dd=cfgObj.drives[i];
    dd[k] = el.type==='checkbox'?el.checked : el.dataset.num?(+el.value) : el.value;
    // Rotary lever: degrees ARE the engineering unit, expressed internally
    // as 360 "units" per output rev. Forced here so the user never sees or
    // maintains the convention (the pitch field is hidden for rotary).
    // The linear ferr-window default (100) would read as 100 DEGREES on a
    // lever, wider than most arcs - protection that never trips. Seed a
    // sane degree default on type switch (a value > 45 is clearly a
    // linear-era leftover); the arc-based dynMax governs from there.
    if(k==='axisType'&&dd.axisType==='rotary_lever'){
      dd.ballscrewPitch=360;
      if(!(dd.followingErrorWindowMm<=45)) dd.followingErrorWindowMm=20;
    }
    // Device families are torque-mode BY DEFINITION (validation enforces
    // it); force the mode and seed a starter feel so a fresh device axis
    // is valid the moment it is created. Devices card list tracks type edits.
    if(k==='axisType'&&isDeviceType(dd.axisType)){
      dd.mode='torque';
      if(!dd.device) dd.device=JSON.parse(JSON.stringify(DEV_PRESETS['H-gate shifter (firm)']));
    }
    if(k==='axisType') devRender();
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

/* ============================================================
   Commissioning tests. Selections + cycle mixing roles (front/rear,
   left/right) live in the browser (localStorage) - they are commissioning
   inputs, not rig config. The server builds the actual plan (one C++
   implementation of percentages, weights, note parsing) and the RT thread
   applies its own entry rails; a Start that 200s can still be refused
   there, which shows up in the status line.
   ============================================================ */
/* ============================================================
   Software update (Pi/headless only; hidden elsewhere). The BROWSER does
   the release check - GitHub's API allows cross-origin reads - so the
   controller needs no HTTPS stack and an offline rig just reports that it
   cannot check. The server only launches the updater unit; progress is
   observed by polling /api/meta until the NEW version answers.
   ============================================================ */
let updTarget=null;
function updInit(){
  const blk=$('updBlock'); if(!blk) return;
  if(meta.platform!=='linux'){ blk.hidden=true; return; }
  blk.hidden=false;
  const c=$('updCur'); if(c) c.textContent='v'+(meta.version||'?');
}
const semLt=(a,b)=>{ const A=String(a).split('.').map(Number),B=String(b).split('.').map(Number);
  for(let i=0;i<3;i++){ if((A[i]||0)<(B[i]||0))return true; if((A[i]||0)>(B[i]||0))return false; }
  return false; };
{ const c=$('updCheck'); if(c) c.onclick=async()=>{
    const n=$('updNote'); n.textContent='checking...'; $('updGo').hidden=true; updTarget=null;
    try{
      const r=await fetch('https://api.github.com/repos/crossthenoughts/nullCAT/releases/latest');
      if(!r.ok) throw 0;
      const j=await r.json(); const latest=String(j.tag_name||'').replace(/^v/,'');
      if(!/^\d+\.\d+\.\d+$/.test(latest)) throw 0;
      if(semLt(meta.version,latest)){
        updTarget=latest;
        const g=$('updGo'); g.textContent='Update to v'+latest; g.hidden=false;
        n.textContent='v'+latest+' is available. Park the rig and stop EtherCAT, then update.';
      } else n.textContent='Up to date (latest release is v'+latest+').';
    }catch(_){ n.textContent='Could not reach GitHub to check (offline?).'; }
  }; }
{ const g=$('updGo'); if(g) g.onclick=async()=>{
    if(!updTarget) return;
    if(!confirm('Update to v'+updTarget+' now? The controller restarts into the new '+
                'version; the two previous versions are kept for rollback.')) return;
    if(!await postJson('/api/update/start',{version:updTarget})) return;
    g.hidden=true;
    const n=$('updNote'); const target=updTarget;
    n.textContent='Updating to v'+target+' -- the page reconnects when done...';
    const t=setInterval(async()=>{
      try{
        const r=await fetch(API+'/api/meta'); if(!r.ok) return;
        const m=await r.json();
        if(m.version===target){ clearInterval(t); location.reload(); }
      }catch(_){}
    },3000);
  }; }

/* ============================================================
   Devices (0.9.5): shifter / active-pedal force axes. Pi-targeted --
   the section (and the host tickbox that enables it) only shows on
   linux builds, and then only when webShowDevices is on. Engage /
   Release go through /api/device/*; the feel is the axis's device{}
   object, saved through the normal config Save. Presets are starting
   points, not gospel; a graphical curve editor is on the roadmap.
   Curve convention: y = force resisting displacement at x (revs).
   ============================================================ */
/* Preset geometry notes: an H-gate's FEELABLE detents are the fore/aft
   engagement positions - never a detent at neutral (it would hide under
   the centring spring, which is exactly what made v1 feel detent-less).
   The spring is a GATE spring: strong near centre, fading past the
   engagement points so the detent holds the lever in gear. */
const DEV_PRESETS={
 'H-gate shifter (firm)':{dir:1,neutralRev:0,
   springCurve:[[0,0],[0.012,40],[0.03,55],[0.045,18],[0.07,8]],
   detents:[-0.055,0.055],
   detentCurve:[[-0.02,-70],[-0.012,-85],[0,0],[0.012,85],[0.02,70]],
   lashRev:0.004,breakoutScale:1.6,frictionPct:4,
   stopMinRev:-0.07,stopMaxRev:0.07,stopSpring:20000,stopDamp:60,
   dampPctPerRevS:10,velLpfHz:40,maxForcePct:100,homeTorquePct:30,homeDir:-1,
   slewPctPerSec:20000,thermalDwellSec:2,thermalPct:80,foldRpm:900},
 'Worn shifter (loose)':{dir:1,neutralRev:0,
   springCurve:[[0,0],[0.015,25],[0.03,32],[0.05,10],[0.07,6]],
   detents:[-0.05,0.05],
   detentCurve:[[-0.025,-40],[-0.012,-50],[0,0],[0.012,50],[0.025,40]],
   lashRev:0.012,breakoutScale:1.2,frictionPct:7,
   stopMinRev:-0.07,stopMaxRev:0.07,stopSpring:14000,stopDamp:60,
   dampPctPerRevS:7,velLpfHz:40,maxForcePct:90,homeTorquePct:25,homeDir:-1,
   slewPctPerSec:20000,thermalDwellSec:2,thermalPct:80,foldRpm:900},
 'Active pedal (progressive)':{dir:1,neutralRev:0,
   springCurve:[[0,12],[0.15,45],[0.3,95],[0.4,170]],
   detents:[],detentCurve:[],lashRev:0,breakoutScale:1,frictionPct:3,
   stopMinRev:-0.01,stopMaxRev:0.4,stopSpring:20000,stopDamp:60,
   dampPctPerRevS:18,velLpfHz:40,maxForcePct:120,homeTorquePct:20,homeDir:-1,
   slewPctPerSec:20000,thermalDwellSec:3,thermalPct:80,foldRpm:600}};
let DEV_USER_PRESETS={};   // devicepresets.json, loaded at config load
const isDeviceType=t=>t==='shifter'||t==='pedal';
const devLive={};   // latest per-axis lever position (revs) from the status poll
const devSweep={};  // per-axis {min,max} travel swept since homing (Capture travel)
const devDot={};    // per-svg live-dot updaters (curve editors)
function devEnabled(){ return meta.platform==='linux' && !!($('cf-showdev')&&$('cf-showdev').checked); }
function devAxes(){ return ((cfgObj&&cfgObj.drives)||[]).map((d,i)=>({d,i})).filter(x=>isDeviceType(x.d.axisType)); }
function devInit(){
  const row=$('devShowRow'); if(row) row.hidden=(meta.platform!=='linux');
  const cb=$('cf-showdev'); if(cb&&!cb._wired){ cb._wired=true; cb.addEventListener('change',devRender); }
  devRender();
  devLoadPresets();
}
async function devLoadPresets(){
  try{ const r=await fetch(API+'/api/devpresets');
       if(r.ok) DEV_USER_PRESETS=(await r.json()).presets||{}; }catch(_){}
  devRender();
}
function devRender(){
  const blk=$('devBlock'); if(!blk) return;
  const on=devEnabled();
  blk.hidden=!on;
  // The axis-type dropdown gains the device types only while the section
  // is enabled -- adding a device axis is part of the same opt-in.
  const typeSpec=AXIS_SPEC.find(f=>f.k==='axisType');
  if(typeSpec){ const has=typeSpec.opts.includes('shifter');
    if(on&&!has) typeSpec.opts.push('shifter','pedal');
    else if(!on&&has) typeSpec.opts=typeSpec.opts.filter(t=>!isDeviceType(t)); }
  if(!on) return;
  const hostEl=$('devCards'); if(!hostEl) return;
  const list=devAxes();
  if(!list.length){ hostEl.innerHTML='<p class="cfg-note">No device axes configured. Add an axis below and set its type to shifter or pedal.</p>'; return; }
  let h='';
  for(const {d,i} of list){
    const n=i+1;
    h+=`<div class="frow"><span>Axis ${n} · ${d.name||TYPE[d.axisType]}</span>
      <span style="display:flex;gap:6px;align-items:center">
        <span id="devState-${n}" class="tag">-</span>
        <button type="button" class="btn btn-sm btn-start" id="devTog-${n}">Home</button>
      </span></div>
      <div class="frow"><span>Preset</span>
      <span style="display:flex;gap:6px;align-items:center;flex-wrap:wrap">
        <select id="devPre-${n}">
          <optgroup label="Built-in">${Object.keys(DEV_PRESETS).map(p=>`<option>${p}</option>`).join('')}</optgroup>
          ${Object.keys(DEV_USER_PRESETS).length?`<optgroup label="Yours">${Object.keys(DEV_USER_PRESETS).map(p=>`<option>${p}</option>`).join('')}</optgroup>`:''}
        </select>
        <button type="button" class="btn btn-sm btn-warn" id="devApply-${n}">Apply preset</button>
        <button type="button" class="btn btn-sm" id="devSaveP-${n}">Save as preset…</button>
        <button type="button" class="btn btn-sm btn-stop" id="devDelP-${n}">Delete preset</button>
      </span></div>`;
    // Geometry + primary feel numbers. HOME-FRAME motor revs throughout:
    // homing anchors the found stop at Travel min or max (per Home toward),
    // everything else hangs off it. Gates = detent centre positions.
    const dv=d.device||{};
    const gv=(k,def)=>((dv[k]!==undefined&&dv[k]!==null)?dv[k]:def);
    h+=`<div class="devgeo" style="grid-column:1/-1">
      <label title="End of usable travel, motor revs in the homed frame. Capture by sweep, or type. Err small: a wall just inside the physical stop is fine."><span>Travel min · rev</span><input type="number" step="0.001" id="devF-${n}-stopMinRev" value="${gv('stopMinRev',-0.07)}"></label>
      <label title="End of usable travel, motor revs in the homed frame."><span>Travel max · rev</span><input type="number" step="0.001" id="devF-${n}-stopMaxRev" value="${gv('stopMaxRev',0.07)}"></label>
      <label title="Rest position the centring spring pulls toward."><span>Neutral · rev</span><input type="number" step="0.001" id="devF-${n}-neutralRev" value="${gv('neutralRev',0)}"></label>
      <label title="Detent centre positions (engagement points). Derive from a layout, or type."><span>Gates · rev, comma-sep</span><input type="text" id="devF-${n}-detents" value="${(dv.detents||[]).join(', ')}" placeholder="-0.055, 0.055"></label>
      <label title="Which stop homing pushes gently against."><span>Home toward</span><select id="devF-${n}-homeDir">
        <option value="-1"${gv('homeDir',-1)<0?' selected':''}>min stop</option>
        <option value="1"${gv('homeDir',-1)>0?' selected':''}>max stop</option></select></label>
      <label title="Homing push, % of rated. Keep low - it presses the mechanism against its own stop."><span>Home torque · %</span><input type="number" step="1" id="devF-${n}-homeTorquePct" value="${gv('homeTorquePct',30)}"></label>
      <label title="Force clamp for the whole feel. Start around 30 for first contact, raise as trusted."><span>Max force · %</span><input type="number" step="1" id="devF-${n}-maxForcePct" value="${gv('maxForcePct',100)}"></label>
      <label title="Zero-force slop about neutral - the worn-linkage feel."><span>Free play · rev</span><input type="number" step="0.001" id="devF-${n}-lashRev" value="${gv('lashRev',0)}"></label>
      <label title="Viscous drag everywhere. Raise to calm buzz or oscillation at rest."><span>Damping · %/(rev/s)</span><input type="number" step="1" id="devF-${n}-dampPctPerRevS" value="${gv('dampPctPerRevS',15)}"></label>
      <label title="Dry friction opposing motion - the mechanical-linkage feel. 0 = off."><span>Friction · %</span><input type="number" step="0.5" id="devF-${n}-frictionPct" value="${gv('frictionPct',0)}"></label>
      <label title="Detent force multiplier pulling OUT of a slot: into gear easy, out of gear firm. 1 = symmetric."><span>Breakout · x</span><input type="number" step="0.1" id="devF-${n}-breakoutScale" value="${gv('breakoutScale',1)}"></label>
      <label title="One flag for a mirrored mechanical build - never rewrite the geometry."><span>Mirror (dir)</span><select id="devF-${n}-dir">
        <option value="1"${gv('dir',1)>0?' selected':''}>normal</option>
        <option value="-1"${gv('dir',1)<0?' selected':''}>mirrored</option></select></label>
    </div>
    <div class="devgeo" style="grid-column:1/-1">
      <label><span>Lever now · rev</span><span class="devlive" id="devLive-${n}">-</span></label>
      <label><span>Swept · rev</span><span class="devlive" id="devSweep-${n}">-</span></label>
      <button type="button" class="btn btn-sm btn-start" id="devT-${n}-cap">Capture travel</button>
      <button type="button" class="btn btn-sm" id="devT-${n}-neu">Set neutral here</button>
      <label><span>Layout</span><select id="devLay-${n}">
        <option value="h">H / sequential</option>
        <option value="sel">Selector (auto)</option>
        <option value="custom">Custom</option></select></label>
      <label><span id="devLayPL-${n}">Throw · rev</span><input type="number" step="0.001" id="devLayP-${n}" value="0.055"></label>
      <button type="button" class="btn btn-sm btn-warn" id="devT-${n}-lay">Derive layout</button>
    </div>
    <div class="cfg-note" style="grid-column:1/-1">The device homes only from its own button (never with the rig): first press homes and rests limp, next press engages. Feel and geometry edits apply LIVE while the device is limp - Save, feel, adjust; if it is engaged when you save, they land on release. Teach by hand: while limp, hold the lever at a position and press the matching capture button, then Save.</div>
      <div style="grid-column:1/-1;display:flex;gap:14px;flex-wrap:wrap;align-items:flex-start">
        <div><div class="cfg-note">Centring spring</div><svg id="devSpring-${n}" class="devCurve"></svg></div>
        <div><div class="cfg-note">Detent profile</div><svg id="devDetent-${n}" class="devCurve"></svg></div>
      </div>`;
  }
  h+='<div class="cfg-note" id="devMsg" style="grid-column:1/-1">A preset overwrites the axis device parameters. Drag the curve nodes to shape the feel (double-click adds or removes a node); Save to persist. Everything else: the device object in rig.json.</div>';
  h+='<div class="cfg-note" style="grid-column:1/-1">Sim channel stream (NULLCATX): <span id="devNcx">-</span></div>';
  hostEl.innerHTML=h;
  for(const {i} of list){
    const n=i+1;
    // The three-state device button: Home (unhomed - the press authorizes
    // the gentle stall search, ends limp) -> Engage -> Release. The engage
    // endpoint homes-or-engages per the device's state; release releases.
    const tog=$('devTog-'+n); if(tog) tog.onclick=()=>{
      postJson(tog.dataset.act==='release'?'/api/device/release':'/api/device/engage',{axis:n}); };
    const ap=$('devApply-'+n); if(ap) ap.onclick=()=>{
      const v=$('devPre-'+n).value;
      const p=DEV_PRESETS[v]||DEV_USER_PRESETS[v]; if(!p||!cfgObj||!cfgObj.drives[i]) return;
      cfgObj.drives[i].device=JSON.parse(JSON.stringify(p));
      cfgObj.drives[i].mode='torque';
      devRender();   // rebuild cards so the curve editors show the preset
      const m=$('devMsg'); if(m) m.textContent='Preset applied to axis '+n+' - Save to persist.';
      refreshDirtyUI();
    };
    // A hand-crafted feel survives anything once it is a named preset.
    const sp=$('devSaveP-'+n); if(sp) sp.onclick=async()=>{
      const dd=cfgObj.drives[i]; if(!dd||!dd.device) return;
      const name=(window.prompt('Preset name:')||'').trim(); if(!name) return;
      if(await postJson('/api/devpresets',{name,device:dd.device})) devLoadPresets();
    };
    const dp=$('devDelP-'+n); if(dp) dp.onclick=async()=>{
      const v=$('devPre-'+n).value;
      const m=$('devMsg');
      if(!DEV_USER_PRESETS[v]){ if(m) m.textContent='Built-in presets cannot be deleted.'; return; }
      if(!window.confirm('Delete preset "'+v+'"?')) return;
      if(await postJson('/api/devpresets',{name:v,remove:true})) devLoadPresets();
    };
    devCurveEditor('devSpring-'+n, i, 'springCurve');
    devCurveEditor('devDetent-'+n, i, 'detentCurve');
    // Geometry/feel field wiring: writes into the axis device object;
    // saved by the normal config Save (dirty tracking sees the whole
    // drive object, so Save(N) counts these edits).
    const wire=(k,fn)=>{ const el=$(`devF-${n}-${k}`); if(el) el.onchange=()=>{
      const dd=cfgObj.drives[i]; dd.device=dd.device||{}; fn(el,dd.device); refreshDirtyUI(); }; };
    for(const k of ['stopMinRev','stopMaxRev','neutralRev','homeTorquePct','maxForcePct',
                    'lashRev','dampPctPerRevS','frictionPct','breakoutScale'])
      wire(k,(el,dv)=>{ const v=+el.value; if(isFinite(v)) dv[k]=v; });
    wire('homeDir',(el,dv)=>{ dv.homeDir=+el.value; });
    wire('dir',   (el,dv)=>{ dv.dir=+el.value; });
    wire('detents',(el,dv)=>{ dv.detents=el.value.split(',').map(s=>+s.trim()).filter(v=>isFinite(v)); });
    // Teach: travel is a HARDWARE property, taught once by sweep - home
    // the device (it rests limp), waggle the lever end to end, press
    // Capture travel. Layout is a USE-CASE property, DERIVED from that
    // range by profile (never taught per position: an H-pattern's lateral
    // select is mechanical and invisible here, so every column shares one
    // fore/neutral/aft geometry; a selector splits the range into slots).
    const setF=(k,v)=>{ const f=$(`devF-${n}-${k}`); if(f) f.value=v; };
    const reflect=(dv)=>{ setF('stopMinRev',dv.stopMinRev); setF('stopMaxRev',dv.stopMaxRev);
      setF('neutralRev',dv.neutralRev);
      const df=$(`devF-${n}-detents`); if(df) df.value=(dv.detents||[]).join(', '); };
    const msg=(t)=>{ const m=$('devMsg'); if(m) m.textContent=t; };
    const cap=$(`devT-${n}-cap`); if(cap) cap.onclick=()=>{
      const sw=devSweep[n];
      if(!sw||!(sw.max>sw.min+0.002)){ msg('Capture travel: home the device, then move the lever end to end by hand first.'); return; }
      const dd=cfgObj.drives[i]; dd.device=dd.device||{};
      dd.device.stopMinRev=+sw.min.toFixed(4); dd.device.stopMaxRev=+sw.max.toFixed(4);
      reflect(dd.device); msg('Travel captured - derive a layout (or set neutral), then Save.');
      refreshDirtyUI(); };
    const neu=$(`devT-${n}-neu`); if(neu) neu.onclick=()=>{
      const lv=devLive[n];
      if(lv===undefined){ msg('Set neutral needs a homed device.'); return; }
      const dd=cfgObj.drives[i]; dd.device=dd.device||{};
      dd.device.neutralRev=+lv.toFixed(4);
      reflect(dd.device); msg('Neutral captured - Save to persist.'); refreshDirtyUI(); };
    const laySel=$(`devLay-${n}`), layP=$(`devLayP-${n}`), layPL=$(`devLayPL-${n}`);
    if(laySel) laySel.onchange=()=>{
      const v=laySel.value;
      if(layPL) layPL.textContent=(v==='sel')?'Slots · count':'Throw · rev';
      if(layP){ layP.disabled=(v==='custom'); if(v==='sel'&&+layP.value<2) layP.value=5; } };
    const lay=$(`devT-${n}-lay`); if(lay) lay.onclick=()=>{
      const dd=cfgObj.drives[i]; dd.device=dd.device||{};
      const dv=dd.device;
      const lo=+dv.stopMinRev, hi=+dv.stopMaxRev;
      if(!(hi>lo)){ msg('Derive layout: capture (or type) the travel first.'); return; }
      const mode=laySel?laySel.value:'h';
      if(mode==='custom'){ msg('Custom layout: type gates and neutral directly.'); return; }
      const centre=+((lo+hi)/2).toFixed(4);
      if(mode==='h'){
        const throwR=Math.abs(+layP.value)||0.055;
        dv.neutralRev=centre;
        dv.detents=[+(centre-throwR).toFixed(4),+(centre+throwR).toFixed(4)];
        msg('H layout derived: neutral at centre, engagement gates fore and aft - Save to persist.');
      } else {
        const nSlots=Math.max(2,Math.min(9,Math.round(+layP.value)||5));
        const edge=(hi-lo)*0.06;
        const a=lo+edge, b=hi-edge, step=(b-a)/(nSlots-1);
        dv.detents=Array.from({length:nSlots},(_,k)=>+(a+k*step).toFixed(4));
        dv.neutralRev=centre;
        msg(nSlots+'-slot selector derived (slot labels live in the game) - Save to persist.');
      }
      reflect(dv); refreshDirtyUI(); };
  }
}

/* ---- graphical curve editor ----
   Edits a device [[x,y],...] node array IN PLACE (cfgObj.drives[i].device),
   saved through the normal config Save. y = the force resisting
   displacement at x (percent of rated); a curve whose first x is negative
   is asymmetric, otherwise the engine mirrors it about zero.
   Drag a node; double-click a segment to add one; double-click a node to
   remove it (a curve keeps at least two). x order is enforced by clamping
   each node between its neighbours. */
function devCurveEditor(svgId, axisIdx, key){
  const svg=$(svgId); if(!svg) return;
  const dev=()=>((cfgObj&&cfgObj.drives[axisIdx])||{}).device;
  if(!dev()) return;
  const W=300,H=140,PL=38,PR=10,PT=10,PB=20;
  svg.setAttribute('viewBox',`0 0 ${W} ${H}`);
  svg.setAttribute('width',W); svg.setAttribute('height',H);
  let rng=null;   // frozen while dragging so the scale doesn't chase the node
  function nodes(){ const d=dev(); return d[key]||(d[key]=[]); }
  function ranges(){
    const ns=nodes(); let xa=0.001,ya=25;
    for(const nd of ns){ xa=Math.max(xa,Math.abs(+nd[0]||0)); ya=Math.max(ya,Math.abs(+nd[1]||0)); }
    const x0=Math.min(0,...ns.map(nd=>+nd[0]||0));
    return {x0:x0*1.15, x1:xa*1.15, y1:ya*1.3};
  }
  const px=x=>PL+((x-rng.x0)/(rng.x1-rng.x0))*(W-PL-PR);
  const py=y=>PT+(1-(y+rng.y1)/(2*rng.y1))*(H-PT-PB);
  const ux=p=>rng.x0+((p-PL)/(W-PL-PR))*(rng.x1-rng.x0);
  const uy=p=>(1-(p-PT)/(H-PT-PB))*2*rng.y1-rng.y1;
  function fmt(v,d){ return (+v).toFixed(d); }
  function render(){
    if(!rng) rng=ranges();
    const ns=nodes();
    let s=`<line x1="${PL}" y1="${py(0)}" x2="${W-PR}" y2="${py(0)}" class="ax"/>`+
          `<line x1="${px(Math.max(rng.x0,0))}" y1="${PT}" x2="${px(Math.max(rng.x0,0))}" y2="${H-PB}" class="ax"/>`+
          `<text x="${PL-4}" y="${PT+8}" class="lbl" text-anchor="end">${fmt(rng.y1,0)}%</text>`+
          `<text x="${PL-4}" y="${H-PB}" class="lbl" text-anchor="end">${fmt(-rng.y1,0)}%</text>`+
          `<text x="${W-PR}" y="${H-6}" class="lbl" text-anchor="end">${fmt(rng.x1,3)} rev</text>`;
    if(ns.length){
      s+=`<polyline class="cv" points="${ns.map(nd=>px(nd[0])+','+py(nd[1])).join(' ')}"/>`;
      ns.forEach((nd,k)=>{ s+=`<circle class="nd" data-k="${k}" cx="${px(nd[0])}" cy="${py(nd[1])}" r="5"/>`; });
    } else {
      s+=`<text x="${W/2}" y="${H/2}" class="lbl" text-anchor="middle">double-click to add nodes</text>`;
    }
    s+=`<circle class="dot" r="3.5" visibility="hidden"/>`;   // live lever marker
    svg.innerHTML=s;
  }
  // Live dot: devPoll feeds (curve-relative x, commanded force). Clamped
  // into the plot box so the marker stays visible at the edges.
  devDot[svgId]=(x,y)=>{
    const dot=svg.querySelector('.dot'); if(!dot) return;
    if(x===null||x===undefined){ dot.setAttribute('visibility','hidden'); return; }
    if(!rng) rng=ranges();
    dot.setAttribute('cx',Math.max(PL,Math.min(W-PR,px(x))));
    dot.setAttribute('cy',Math.max(PT,Math.min(H-PB,py(y||0))));
    dot.setAttribute('visibility','visible');
  };
  function pt(ev){ const r=svg.getBoundingClientRect();
    return { x:(ev.clientX-r.left)*W/r.width, y:(ev.clientY-r.top)*H/r.height }; }
  function nearest(p,r2){ let best=-1,bd=r2||144; const ns=nodes();
    ns.forEach((nd,k)=>{ const dx=px(nd[0])-p.x, dy=py(nd[1])-p.y, d=dx*dx+dy*dy;
      if(d<bd){ bd=d; best=k; } });
    return best; }
  let drag=-1;
  svg.addEventListener('pointerdown',ev=>{
    if(!rng) rng=ranges();
    drag=nearest(pt(ev));
    if(drag>=0){ svg.setPointerCapture(ev.pointerId); ev.preventDefault(); }
  });
  svg.addEventListener('pointermove',ev=>{
    if(drag<0) return;
    const ns=nodes(), p=pt(ev);
    const lo=(drag>0)?(+ns[drag-1][0])+1e-4:rng.x0;
    const hi=(drag<ns.length-1)?(+ns[drag+1][0])-1e-4:rng.x1;
    ns[drag]=[ +Math.min(hi,Math.max(lo,ux(p.x))).toFixed(4),
               +Math.min(rng.y1,Math.max(-rng.y1,uy(p.y))).toFixed(1) ];
    render();
  });
  svg.addEventListener('pointerup',ev=>{
    if(drag<0) return;
    drag=-1; rng=null; render(); refreshDirtyUI();
  });
  svg.addEventListener('dblclick',ev=>{
    if(!rng) rng=ranges();
    // Delete gets a generous hit radius (20px): a double-click NEAR a
    // node removes it; only a clearly-empty spot adds one.
    const ns=nodes(), p=pt(ev), k=nearest(p,400);
    if(k>=0 && ns.length>2){ ns.splice(k,1); }
    else if(k<0){
      const nd=[ +ux(p.x).toFixed(4), +uy(p.y).toFixed(1) ];
      let at=ns.findIndex(m=>+m[0]>nd[0]); if(at<0) at=ns.length;
      ns.splice(at,0,nd);
    } else return;
    rng=null; render(); refreshDirtyUI();
  });
  render();
}
function devPoll(s){
  const blk=$('devBlock'); if(!blk||blk.hidden) return;
  const nx=$('devNcx'); if(nx) nx.textContent=(s.ncxRx?'receiving':'not receiving (effects idle, plain feel)')
    +((s.gearsKnown|0)>0?` · ${s.gearsKnown} gear ratio${s.gearsKnown>1?'s':''} known`:'');
  const running=!!s.loopRunning, estop=!!s.estop;
  for(const {i} of devAxes()){
    const n=i+1; const d=(s.drives&&s.drives[i])||{};
    const st=d.state||'-';
    const el=$('devState-'+n); if(el) el.textContent=st;
    // Live lever position for the teach row (only once homed - the frame
    // is unanchored before that, so keep the capture disabled).
    if(d.homed && typeof d.devRev==='number'){ devLive[n]=d.devRev;
      devSweep[n]={min:d.devMin,max:d.devMax};
      const lv=$('devLive-'+n); if(lv) lv.textContent=d.devRev.toFixed(4);
      const sw=$('devSweep-'+n); if(sw) sw.textContent=d.devMin.toFixed(3)+' … '+d.devMax.toFixed(3);
      // Live dot on the curve editors: where the lever sits on each curve
      // right now, with the actually-commanded force (whole field, so the
      // dot can sit off a single curve - it is a position marker first).
      const dv=(cfgObj&&cfgObj.drives[i]&&cfgObj.drives[i].device)||{};
      const f=(typeof d.cmdTrq==='number')?d.cmdTrq:0;
      const sd=devDot['devSpring-'+n]; if(sd) sd(d.devRev-(+dv.neutralRev||0), f);
      const ds=devDot['devDetent-'+n];
      if(ds){ const dets=dv.detents||[];
        let best=null; for(const g of dets) if(best===null||Math.abs(d.devRev-g)<Math.abs(d.devRev-best)) best=g;
        ds(best===null?null:d.devRev-best, f); } }
    else { delete devLive[n]; delete devSweep[n];
      const lv=$('devLive-'+n); if(lv) lv.textContent=d.homed===false?'(not homed)':'-';
      const sw=$('devSweep-'+n); if(sw) sw.textContent='-'; }
    const online=/online|blending/i.test(st);
    const homing=/homing/i.test(st);
    const tog=$('devTog-'+n);
    if(tog){
      tog.disabled=!running||estop||homing;
      const act=online?'release':(d.homed?'engage':'home');
      tog.dataset.act=act;
      tog.textContent=homing?'Homing…':(act==='release'?'Release':act==='engage'?'Engage':'Home');
      tog.classList.toggle('btn-stop',act==='release');
      tog.classList.toggle('btn-start',act!=='release');
    }
  }
}

let tstDrives=[], tstLoopRunning=false, tstActive=false;
const TSTKEY='nullcat.test.axes';
const esc=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function tstPrefs(){ try{ return JSON.parse(localStorage.getItem(TSTKEY))||{}; }catch(_){ return {}; } }
function tstSavePrefs(p){ try{ localStorage.setItem(TSTKEY,JSON.stringify(p)); }catch(_){} }
function tstGuessRole(name){ // best-effort default from the axis name; user-overridable
  const n=(name||'').toLowerCase();
  let fr=0, lr=0;
  if(/\bfl\b|front.?left/.test(n)){fr=1;lr=1;} else if(/\bfr\b|front.?right/.test(n)){fr=1;lr=-1;}
  else if(/\brl\b|rear.?left|back.?left/.test(n)){fr=-1;lr=1;} else if(/\brr\b|rear.?right|back.?right/.test(n)){fr=-1;lr=-1;}
  else { if(/front/.test(n))fr=1; else if(/rear|back/.test(n))fr=-1;
         if(/left/.test(n))lr=1; else if(/right/.test(n))lr=-1; }
  return {fr,lr};
}
function buildTestAxes(){
  const host=$('testAxes'); if(!host) return;
  host.textContent='';
  const prefs=tstPrefs();
  tstDrives.forEach((d,i)=>{
    if(d.mode==='torque'||d.axisType==='belt') return;   // belts are never testable
    const vert = d.axisType!=='linear_horizontal';
    const p = prefs[i] || Object.assign({sel:true}, vert?tstGuessRole(d.name):{fr:0,lr:0});
    const row=document.createElement('div'); row.className='taxis'; row.dataset.i=i;
    const cb=document.createElement('input'); cb.type='checkbox'; cb.checked=p.sel!==false; cb.className='tsel';
    const nm=document.createElement('span'); nm.className='tname'; nm.textContent=d.name||('Drive '+(i+1));
    const tg=document.createElement('span'); tg.className='ttag';
    tg.textContent=d.axisType==='rotary_lever'?'ROT':(vert?'VERT':'HORIZ');
    row.appendChild(cb); row.appendChild(nm); row.appendChild(tg);
    const mkSel=(cls,opts,val)=>{ const s=document.createElement('select'); s.className=cls;
      opts.forEach(([v,t])=>{ const o=document.createElement('option'); o.value=v; o.textContent=t; s.appendChild(o); });
      s.value=String(val); return s; };
    // Rotary levers get NO role selectors: the corner roles describe
    // four-post rigs, and platform pitch/roll on a hexapod is kinematics
    // nullCAT deliberately does not do. Levers take part in heave and the
    // per-leg tests only.
    if(vert && d.axisType!=='rotary_lever'){
      row.appendChild(mkSel('tfr',[[0,'f/r —'],[1,'Front'],[-1,'Rear']],p.fr|0));
      row.appendChild(mkSel('tlr',[[0,'l/r —'],[1,'Left'],[-1,'Right']],p.lr|0));
    } else {
      row.appendChild(document.createElement('span'));
      row.appendChild(document.createElement('span'));
    }
    // Per-axis home. Hexapod use: direction checks with the pushrods
    // DISCONNECTED, and near-park single-leg re-homes. A full solo sweep
    // on a coupled platform binds (see HEXAPOD_SETUP.md).
    const hb=document.createElement('button'); hb.type='button';
    hb.className='thome'; hb.textContent='Home';
    hb.title='Home this axis alone (direction checks unloaded; near-park re-homes)';
    hb.addEventListener('click',e=>{ e.preventDefault(); postJson('/api/home',{axis:i+1}); });
    row.appendChild(hb);
    row.addEventListener('change',()=>{
      const pr=tstPrefs();
      pr[i]={ sel:cb.checked,
              fr:row.querySelector('.tfr')?parseInt(row.querySelector('.tfr').value,10):0,
              lr:row.querySelector('.tlr')?parseInt(row.querySelector('.tlr').value,10):0 };
      tstSavePrefs(pr);
    });
    host.appendChild(row);
  });
  if(!host.children.length){
    const n=document.createElement('div'); n.className='test-note';
    n.textContent='No testable axes (belt axes are excluded).';
    host.appendChild(n);
  }
}
function tstAxesPayload(){
  const out=[];
  document.querySelectorAll('#testAxes .taxis').forEach(row=>{
    const i=parseInt(row.dataset.i,10);
    const fr=row.querySelector('.tfr'), lr=row.querySelector('.tlr');
    out.push({ i, sel:row.querySelector('.tsel').checked,
               fr:fr?parseInt(fr.value,10):0, lr:lr?parseInt(lr.value,10):0 });
  });
  return out;
}
document.querySelectorAll('#testModes input[name=tmode]').forEach(r=>{
  r.addEventListener('change',()=>{
    ['cycle','tone','sweep','step','song'].forEach(m=>{ $('tp-'+m).hidden = m!==r.value; });
  });
});
const numv=(id,def)=>{ const v=parseFloat($(id).value); return isFinite(v)?v:def; };
async function postJson(ep,body){
  clearCmdErr();
  try{
    const r=await fetch(API+ep,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    let j=null; try{ j=await r.json(); }catch(_){}
    if(j && j.ok===false){ showCmdErr(j.error||'refused'); return false; }
    if(!r.ok){ showCmdErr('HTTP '+r.status); return false; }
    return true;
  }catch(e){ showCmdErr('no response from the controller'); return false; }
}
$('btnTestStart').onclick=async()=>{
  const mode=document.querySelector('#testModes input[name=tmode]:checked').value;
  const body={ mode, axes:tstAxesPayload() };
  if(mode==='cycle'){
    body.enPitch=$('tcPitch').checked;  body.pitchPct=numv('tcPitchPct',30);
    body.enRoll=$('tcRoll').checked;    body.rollPct=numv('tcRollPct',30);
    body.enHeave=$('tcHeave').checked;  body.heavePct=numv('tcHeavePct',40);
    body.enHoriz=$('tcHoriz').checked;  body.horizPct=numv('tcHorizPct',30);
    body.freqHz=numv('tcFreq',0.2);     body.cycles=numv('tcCycles',2);
  } else if(mode==='tone'){
    body.freqHz=numv('ttFreq',25); body.pct=numv('ttPct',2); body.durationSec=numv('ttDur',5);
  } else if(mode==='sweep'){
    body.f0=numv('tsF0',5); body.f1=numv('tsF1',50); body.stepHz=numv('tsStep',2.5);
    body.dwellSec=numv('tsDwell',2); body.pct=numv('tsPct',2);
  } else if(mode==='step'){
    body.pct=numv('tpPct',5); body.holdSec=numv('tpHold',3);
  } else if(mode==='song'){
    body.notes=$('tgNotes').value; body.beatSec=numv('tgBeat',0.55); body.pct=numv('tgPct',2);
  }
  if(await postJson('/api/test/start',body)){ tstActive=true; refreshTestButtons(); pollTestStatus(); }
};
$('btnTestStop').onclick=()=>postCmd('/api/test/stop');
function refreshTestButtons(){
  const st=$('btnTestStart'), sp=$('btnTestStop');
  if(st) st.disabled = !tstLoopRunning || tstActive;
  if(sp) sp.disabled = !tstActive;
}
function renderTestResults(j){
  const host=$('testResults'); if(!host) return;
  if(!j.results || !j.results.length){ host.hidden=true; host.textContent=''; return; }
  const nameOf=i=>esc((tstDrives[i]&&tstDrives[i].name)||('Drive '+(i+1)));
  const anyStep=j.results.some(r=>r.kind===1);
  const anySine=j.results.some(r=>r.kind!==1);
  // Units ride on the axis cell (mm vs ° differs per row on a mixed rig).
  const unitOf=i=>(tstDrives[i]&&tstDrives[i].axisType==='rotary_lever')?'°':'mm';
  let h='<table><tr><th>segment</th><th>Hz</th><th>axis</th><th>cmd</th><th>act</th>'
       +'<th>ratio</th><th>phase°</th><th>ferr rms</th><th>ferr pk</th><th>trq σ%</th>'
       +(anySine?'<th>trq@f%</th><th>trq/acc</th>':'')
       +(anyStep?'<th>os%</th><th>rise ms</th><th>settle ms</th>':'')
       +'</tr>';
  j.results.forEach(r=>{
    const step=r.kind===1;
    r.axes.forEach((a,k)=>{
      h+='<tr><td>'+(k===0?esc(r.label):'')+'</td><td>'+(k===0&&!step?r.freqHz.toFixed(2):'')+'</td>'
        +'<td>'+nameOf(a.i)+' <span class="tunit">'+unitOf(a.i)+'</span>'
        +(a.derated?' <span class="drtd">drtd</span>':'')+'</td>'
        +'<td>'+a.cmdAmp.toFixed(3)+'</td><td>'+a.actAmp.toFixed(3)+'</td>'
        +'<td>'+a.ratio.toFixed(3)+'</td><td>'+(step?'':a.phaseDeg.toFixed(1))+'</td>'
        +'<td>'+a.ferrRms.toFixed(3)+'</td><td>'+a.ferrPeak.toFixed(3)+'</td>'
        +'<td>'+a.trqRms.toFixed(1)+'</td>';
      if(anySine) h+='<td>'+(step||a.trqAmp==null?'':a.trqAmp.toFixed(2))+'</td>'
                    +'<td>'+(step||a.trqPerAcc==null?'':a.trqPerAcc.toFixed(3))+'</td>';
      if(anyStep) h+='<td>'+(step?a.osPct.toFixed(1):'')+'</td>'
                    +'<td>'+(step?a.riseMs.toFixed(0):'')+'</td>'
                    +'<td>'+(step?a.settleMs.toFixed(0):'')+'</td>';
      h+='</tr>';
    });
  });
  host.innerHTML=h+'</table>'+renderTestAssessment(j); host.hidden=false;
}
/* Plain-language read-out of the numbers, so the table is useful without a
   controls background. Heuristics only - each verdict names the evidence. */
function renderTestAssessment(j){
  const byAxis={};
  j.results.forEach(r=>r.axes.forEach(a=>{
    (byAxis[a.i]=byAxis[a.i]||{sine:[],step:[]})[r.kind===1?'step':'sine'].push(Object.assign({f:r.freqHz},a));
  }));
  const nameOf=i=>esc((tstDrives[i]&&tstDrives[i].name)||('Drive '+(+i+1)));
  const lines=[];
  Object.keys(byAxis).forEach(i=>{
    const ax=byAxis[i], out=[];
    const sw=ax.sine.filter(s=>s.cmdAmp>1e-4).sort((a,b)=>a.f-b.f);
    if(sw.length>=3){
      // bandwidth: first frequency where the response falls below 0.707x
      let bw=null;
      for(const s of sw){ if(s.ratio<0.707){ bw=s.f; break; } }
      out.push(bw?('tracks well to ~'+bw.toFixed(0)+' Hz, falls off above that')
                 :('tracks the whole sweep (usable to at least '+sw[sw.length-1].f.toFixed(0)+' Hz)'));
      // resonance: response bigger than commanded
      const res=sw.filter(s=>s.ratio>1.15).sort((a,b)=>b.ratio-a.ratio)[0];
      if(res) out.push('RESONANCE near '+res.f.toFixed(1)+' Hz (response '
        +res.ratio.toFixed(2)+'x command) - worth trying a drive notch filter there');
      // low-frequency deficit = lash/compliance, not bandwidth
      if(sw[0].f<=5 && sw[0].ratio<0.9)
        out.push('weak tracking even at '+sw[0].f.toFixed(1)+' Hz (ratio '
          +sw[0].ratio.toFixed(2)+') - check backlash / belt tension / mechanical compliance');
      // hunting: torque ripple without matching motion
      const hunt=sw.filter(s=>s.trqRms>8&&s.ratio<0.9)[0];
      if(hunt) out.push('torque works hard for little motion at '+hunt.f.toFixed(1)
        +' Hz (ripple '+hunt.trqRms.toFixed(0)+'%) - possible hunting/lash');
      if(sw.some(s=>s.derated))
        out.push('high-frequency amplitudes were auto-reduced to fit accel limits - expected physics, not a fault');
    }
    ax.step.forEach(st=>{
      if(st.osPct>15)      out.push('step overshoot '+st.osPct.toFixed(0)+'% - tune is HOT (reduce gain/stiffness)');
      else if(st.osPct>5)  out.push('step overshoot '+st.osPct.toFixed(0)+'% - firm but acceptable');
      else if(st.settleMs>400) out.push('no overshoot but slow settling ('+st.settleMs.toFixed(0)+'ms) - tune is soft (more gain available)');
      else out.push('step response well damped (overshoot '+st.osPct.toFixed(1)+'%, settled in '+st.settleMs.toFixed(0)+'ms)');
    });
    if(out.length) lines.push('<b>'+nameOf(i)+'</b>: '+out.join('; '));
  });
  // cross-axis: an axis notably softer than its peers is mechanics, not tune
  const bws=Object.keys(byAxis).map(i=>{
    const sw=byAxis[i].sine.filter(s=>s.cmdAmp>1e-4).sort((a,b)=>a.f-b.f);
    if(sw.length<3) return null;
    for(const s of sw){ if(s.ratio<0.707) return {i,bw:s.f}; }
    return {i,bw:sw[sw.length-1].f};
  }).filter(Boolean);
  if(bws.length>=2){
    const best=Math.max.apply(null,bws.map(b=>b.bw));
    bws.filter(b=>b.bw<0.6*best).forEach(b=>
      lines.push('<b>'+nameOf(b.i)+'</b> is notably softer than its peers ('
        +b.bw.toFixed(0)+' Hz vs '+best.toFixed(0)+' Hz) - compare its mechanics/tune to the others'));
  }
  if(!lines.length) return '';
  return '<div class="test-verdict"><div class="tv-h">What this means</div>'
       +lines.map(l=>'<div class="tv-l">'+l+'</div>').join('')+'</div>';
}
async function pollTestStatus(){
  if($('testPanel').classList.contains('collapsed') && !tstActive) return;
  try{
    const r=await fetch(API+'/api/test/status'); if(!r.ok) return;
    const j=await r.json();
    tstActive=!!j.active; refreshTestButtons();
    const el=$('testStatus');
    if(j.active){
      el.textContent=j.title+' · '+j.phase
        +(j.phase==='running'?(' · segment '+(j.segIdx+1)+'/'+j.numSegments):'')
        +' · '+j.progressPct.toFixed(0)+'%';
    } else if(j.aborted && j.reason){
      el.textContent='aborted: '+j.reason;
    } else if(j.reason){
      el.textContent=j.reason;              // RT-side refusal
    } else if(j.done){
      el.textContent=(j.title?j.title+' ':'')+'complete';
    } else el.textContent='';
    renderTestResults(j);
  }catch(_){}
}
setInterval(pollTestStatus,1000);

/* ---- boot ---- */
updateConn(); pollStatus(); setInterval(pollStatus, 500); pollLogs();
