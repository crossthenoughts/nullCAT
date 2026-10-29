// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
'use strict';
// ============================================================
// nullCAT web UI. Polls /api/status + /api/logs (WS if present).
// ============================================================
const $ = (id) => document.getElementById(id);
const API = '';

/* ---- optional web auth: the server 401s every /api/* call when host.json
   sets webAuthToken. fetch is wrapped ONCE here: the stored token rides
   every /api request as a header, and the first 401 raises the password
   prompt (stored per-browser, like the theme; wrapped in try/catch since
   localStorage can be unavailable). Plain HTTP, so this is a lock on the
   door, not a hostile-network defense. ---- */
const AUTH_KEY='nullcat.authToken';
{ const rawFetch=window.fetch.bind(window);
  window.fetch=async(url,opts)=>{
    const isApi=(typeof url==='string')&&url.startsWith(API+'/api');
    if(isApi){ let tok=null; try{ tok=localStorage.getItem(AUTH_KEY); }catch(_){}
      if(tok){ opts=opts||{}; opts.headers=Object.assign({},opts.headers,{'X-Nullcat-Auth':tok}); } }
    const r=await rawFetch(url,opts);
    if(isApi&&r.status===401) authPrompt();
    return r;
  };
}
let authPromptUp=false;
function authPrompt(){
  if(authPromptUp) return; authPromptUp=true;
  const d=document.createElement('div');
  d.style.cssText='position:fixed;inset:0;background:rgba(0,0,0,.55);display:flex;align-items:center;justify-content:center;z-index:9999';
  d.innerHTML='<div style="background:var(--surface,#fff);border:1.5px solid var(--hair,#ccc);border-radius:8px;padding:22px 26px;max-width:320px">'
    +'<div style="font-weight:600;margin-bottom:8px">Password required</div>'
    +'<div style="font-size:.78rem;margin-bottom:12px;line-height:1.5">This controller has a web password set. Enter it once for this browser.</div>'
    +'<input id="authTok" type="password" style="width:100%;margin-bottom:12px;box-sizing:border-box">'
    +'<button id="authGo" class="btn btn-action" type="button">Unlock</button></div>';
  document.body.appendChild(d);
  const go=()=>{ try{ localStorage.setItem(AUTH_KEY,d.querySelector('#authTok').value); }catch(_){} location.reload(); };
  d.querySelector('#authGo').onclick=go;
  d.querySelector('#authTok').addEventListener('keydown',(e)=>{ if(e.key==='Enter') go(); });
  setTimeout(()=>{ const i=d.querySelector('#authTok'); if(i) i.focus(); },50);
}

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
/* ---- Operate / Setup view switch. Setup holds the build-time surfaces
   (host settings, provisioning, commissioning test, bindings, updater);
   Operate holds everything a tuning evening touches. Pure show/hide via
   body[data-view] and the vw-op / vw-setup tags - no element moves, so
   every id and handler stays exactly where the rest of this file put it. */
function setView(v){
  document.body.dataset.view=v;
  const o=$('vwBtn-op'), s=$('vwBtn-setup');
  if(o) o.classList.toggle('on', v==='op');
  if(s) s.classList.toggle('on', v==='setup');
  try{ localStorage.setItem('nullcat.view', v); }catch(_){}
}
{ let v='op';
  try{ v = (location.hash==='#setup') ? 'setup' : (localStorage.getItem('nullcat.view')||'op'); }catch(_){}
  setView(v==='setup'?'setup':'op');
  const o=$('vwBtn-op'), s=$('vwBtn-setup');
  if(o) o.onclick=()=>setView('op');
  if(s) s.onclick=()=>{ setView('setup'); if(cfgBaseline&&dirtyState().count===0) loadConfig(); }; }

/* Three housings: a6 (the default), light, graphite. The storage key is
   unchanged; a browser that stored the old "dark" gets graphite. */
const THEME_NAMES={a6:'A6',light:'Light',graphite:'Graphite'};
function applyTheme(t){ t = t==='dark'?'graphite':(THEME_NAMES[t]?t:'a6');
  document.documentElement.setAttribute('data-theme', t); document.body.setAttribute('data-theme', t);
  const k=$('thmK'); if(k) k.textContent='Theme · '+THEME_NAMES[t];
  document.querySelectorAll('#thmMenu [data-t]').forEach(b=>b.classList.toggle('on', b.dataset.t===t));
  return t; }
let theme = applyTheme(localStorage.getItem(THEME_KEY)||'a6');

/* ---- transport: poll-only. httplib has no WebSocket server, so the old
   /ws reconnect-every-3s just churned failing connections. Pure polling. ---- */
async function pollStatus(){ try{ const r=await fetch(API+'/api/status');
  if(r.ok){ const s=await r.json(); applyState(s); hapLive(s); fitHeader(); } }catch(_){} }

/* ---- apply status ---- */
/* The header's status window drops its cells while the header would run
   past the window (the values' widths change with the rig, so a fixed
   breakpoint cannot know): the least needed first, the link and its
   clock last. */
const FIT_ORDER=['v-cyc','v-wkc','v-telemetry','v-rate','v-udp','v-slv','conn-status'];
function fitHeader(){
  const top=document.querySelector('.top'); if(!top) return;
  const cells=FIT_ORDER.map(id=>{ const e=$(id); return e&&e.closest('.c'); }).filter(Boolean);
  for(const c of cells) c.classList.remove('fit-hide');
  for(const c of cells){ if(top.scrollWidth<=top.clientWidth+1) break; c.classList.add('fit-hide'); }
}
window.addEventListener('resize',fitHeader);
function applyState(s){
  lastRecvMs=Date.now(); lastStatus=s;
  setLinkDown(false);   // fresh data: clear the link-lost guard (button states recomputed below)
  const running=!!s.loopRunning, op=!!s.masterOp, estop=!!s.estop;

  setV(vLoop, running?'RUN':'STOP', running?'ok':'');
  setV(vEc, op?'OP':'OFF', op?'ok':'warn');
  setSeg(vRate, s.loopHz? s.loopHz.toFixed(0) : '----');
  setSeg(vCyc,  s.loopHz? (1e6/s.loopHz).toFixed(0) : '----');
  setV(vWkc, String(s.wkcErrors ?? 0), (s.wkcErrors>0)?'bad':'ok');
  setSeg(vSlv, String(s.slavesFound ?? 0)); { const n=$('v-slvN'); if(n) n.textContent='/ '+(s.numDrives ?? 0); }
  // Telemetry indicator: RX (receiving) / idle (socket up, no packets) /
  // SOCKET FAILED (bind failed at startup - check telemetryBindAddr/port in
  // host.json; a fallback bind logs loudly in the journal).
  setV(vTelemetry, s.telemetryReceiving?'RX':(s.telemetryInit?'idle':'SOCKET FAILED'), s.telemetryReceiving?'ok':(s.telemetryInit?'':'bad'));
  if(vTelemetry) vTelemetry.title=s.telemetryInit?'':"Telemetry socket failed to bind - check telemetryBindAddr (this machine's address or 0.0.0.0) and the port in host.json, then restart the service.";
  // UDP rate diagnostic: new/arrival Hz + hold% ( - when diag off / no window yet)
  if(vUdp){ const ua=+s.udpArrivalHz, un=+s.udpNewHz, uh=+s.udpHoldPct;
    vUdp.innerHTML = (ua>=0)? segHtml(un.toFixed(0),3)+'<span class="dl">/</span>'+segHtml(ua.toFixed(0),3)+'<span class="dl">Hz</span><span class="vt dim">'+uh.toFixed(0)+'%</span>'
                            : '<span class="vt dim">-</span>'; }
  // Config-page hint: suggest a mode from the measured new-frame rate (inform, never auto-switch).
  { const e=$('cf-udphint'); if(e){ const un=+s.udpNewHz;
    e.textContent = (un<0) ? 'UDP rate: enable diagnostics to measure'
      : (un>500) ? `UDP new ${un.toFixed(0)}Hz - high; Bypass/Interpolate suggested`
      : (un<150) ? `UDP new ${un.toFixed(0)}Hz - low; Interpolate or Filter suggested`
      : `UDP new ${un.toFixed(0)}Hz`; } }

  const j=+s.maxJitterUs||0; jbuf[jhead]=j; jhead=(jhead+1)%SPARK_N; if(jfill<SPARK_N) jfill++;
  if(j>jpeak) jpeak=j;
  setSeg(vJit, j? j.toFixed(0):'--'); setSeg(vJpk, jpeak? jpeak.toFixed(0):'--');
  renderSpark();

  buildDriveCards(s.drives||[], running);
  devPoll(s);
  driveCount.textContent = (s.numDrives ?? (s.drives||[]).length)+' axes';
  // commissioning panel needs the axis list + loop state
  tstDrives = s.drives||[]; tstLoopRunning = running; refreshTestButtons();

  document.body.classList.toggle('estop-active', estop);
  // The trip banner: what stopped the rig, when, and the one way back.
  { const b=$('estopBanner'); if(b){ const t=!!s.estopTripped; b.hidden=!t;
      if(t){ const ago=+s.estopTripAgoSec||0, src=s.estopSource||'e-stop';
        const when = ago<60 ? ago+' s ago' : ago<3600 ? Math.round(ago/60)+' min ago' : Math.round(ago/3600)+' h ago';
        const h=String(src).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
        const steps = running ? 'Stop, then Stop EtherCAT, then Initialize EtherCAT'
                    : op      ? 'Stop EtherCAT, then Initialize EtherCAT'
                    :           'Initialize EtherCAT';
        b.innerHTML = '⚠ <b>E-STOP</b> tripped by the '+h+' '+when+'. Nothing moves until the drives are re-initialized'
          + (s.estopInput ? ': release the e-stop first, then ' : ': ') + '<b>'+steps+'</b>.'; } } }
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
  // E-STOP and Release follow the input, so E-STOP can be pressed again
  // while tripped; the trip itself stays until Initialize.
  btn.estop.disabled=!!(s.estopInput ?? estop); btn.release.disabled=!(s.estopInput ?? estop);
  // Key lamps: green marks the next useful step, amber a recovery action,
  // the E-STOP key sits latched while its input is held. Derived from the
  // enable states above only; the rules themselves are unchanged.
  const faulted=(s.aggregate&&s.aggregate.state==='fault')||(s.drives||[]).some(d=>/fault|fatal/i.test(d.state||''));
  btn.init.classList.toggle('lit', !btn.init.disabled && ecatMode==='init');
  btn.start.classList.toggle('lit', !btn.start.disabled);
  btn.park.classList.toggle('lit', !btn.park.disabled && parked);
  btn.belts.classList.toggle('lit', !btn.belts.disabled && beltsSlack);
  btn.fault.classList.toggle('lit', faulted && !btn.fault.disabled);
  btn.release.classList.toggle('lit', !btn.release.disabled);
  btn.estop.classList.toggle('on', !!(s.estopInput ?? estop));
  { const ab=$('ab'); if(ab) ab.classList.toggle('alert', estop||faulted); }
}
/* Status window writers. setV keeps its (el, text, cls) contract: the value
   takes the display class (ok / warn / bad, else dim) and the lamp before
   it takes the same colour. Segment readouts carry a ghost of unlit
   segments behind the digits (data-g, padded to the field's width). */
function segGhost(el,t){ const w=+(el.dataset.w||(el.dataset.w=(el.dataset.g||'').length)); el.dataset.g=String(t).replace(/[0-9A-Za-z]/g,'8').padStart(w,'8'); }
function setV(el,t,c){ if(!el) return; el.textContent=t;
  const seg=el.classList.contains('seg'); el.className=(seg?'seg':'vt')+(c?' '+c:(seg?'':' dim'));
  if(seg) segGhost(el,t);
  const l=el.previousElementSibling; if(l&&l.classList.contains('lamp')) l.className='lamp'+(c?' '+c:''); }
function setSeg(el,t){ if(!el||el.textContent===t) return; el.textContent=t; segGhost(el,t); }
function segHtml(t,w){ return '<span class="seg" data-g="'+String(t).replace(/[0-9]/g,'8').padStart(w,'8')+'">'+t+'</span>'; }
function setLogo(c){ if(!elLogo) return; const f={'is-offline':'logo-offline.png','is-online':'logo-online.png','is-fault':'logo-fault.png','is-estop':'logo-estop.png'}[c]||'logo-offline.png';
  if(!(elLogo.getAttribute('src')||'').endsWith(f)) elLogo.setAttribute('src',f); elLogo.className='logo-badge '+c; }
function renderSpark(){ let p=''; for(let k=0;k<jfill;k++){ const i=(jhead-jfill+k+SPARK_N)%SPARK_N; const v=jbuf[i];
  p+=(k/(SPARK_N-1)*60).toFixed(1)+','+(18-Math.min(v,150)/150*16).toFixed(1)+' '; } sparkLine.setAttribute('points',p.trim()); }

/* ---- Motion rows. One row per axis: lamp, name and role, then a glass
   strip of state, position bar with target tick, position, velocity,
   torque bar, IGBT and following error; a click opens the row downward
   into the detail grid (target, peaks, duty, speed, guard, accel headroom,
   the raw statusword, the config line) and an "Axis setup" key. The same
   /api/status drives[] fields the cards showed, written onto the same
   class hooks; peaks stay sticky per session as before. ---- */
const AX_OPEN_KEY='nullcat.ui.axes';
let axOpen=[]; try{ axOpen=JSON.parse(localStorage.getItem(AX_OPEN_KEY))||[]; }catch(_){ axOpen=[]; }
if(!Array.isArray(axOpen)) axOpen=[];
const LAMP_OF={'s-op':'ok','s-home':'warn','s-fault':'bad','s-estop':'bad','s-park':'acc','s-off':''};
function axToggle(i,on){ const w=driveGrid.children[i]; if(!w) return; if(on===undefined) on=!w.classList.contains('open');
  w.classList.toggle('open',on); axOpen=axOpen.filter(x=>x!==i); if(on) axOpen.push(i);
  try{ localStorage.setItem(AX_OPEN_KEY,JSON.stringify(axOpen)); }catch(_){} }
function buildDriveCards(drives, loopRunning){
  const rail=$('moRail');
  while(driveGrid.children.length<drives.length){
    const i=driveGrid.children.length;
    const w=document.createElement('div'); w.className='axw'+(axOpen.includes(i)?' open':''); w.dataset.ax=i;
    w.innerHTML=
      `<div class="axr"><span class="lamp dot"></span>`+
        `<span class="nm"><b class="nmt"> - </b><small class="role"><span class="typ"> - </span><span class="dir"></span></small><small class="stt"> - </small></span>`+
        `<div class="rg glass">`+
          `<span class="c-st vt dim stxt"> - </span>`+
          `<span class="bar pos-only"><i class="pf"></i><b class="tt"></b></span><span class="vt dim na trq-only">torque only · CST</span>`+
          `<span class="r pos-only"><span class="seg pos" data-g="88.888">--.---</span></span><span class="r trq-only"><span class="mono cmdtrq"> - </span></span>`+
          `<span class="r c-vel mono vel"> - </span>`+
          `<span class="c-tq tq"><span class="bar t"><i class="tf"></i></span><span class="mono r trq"> - </span></span>`+
          `<span class="r c-tmp mono igbt"> - </span>`+
          `<span class="r c-fe mono ferr"> - </span>`+
        `</div><span class="chev"></span></div>`+
      `<div class="axx"><div class="srow"><span class="sw"></span><span class="dcfg"> - </span></div><div class="flt" style="display:none"></div>`+
        `<div class="glass dg metrics">`+
          `<div class="pos-only"><span class="dl">Target mm</span><span class="dv tgt"> - </span></div>`+
          `<div class="pos-only"><span class="dl">Peak vel mm/s</span><span class="dv velpk"> - </span></div>`+
          `<div class="trq-only"><span class="dl">Cmd %</span><span class="dv cmdtrq2"> - </span></div>`+
          `<div><span class="dl">Peak trq %</span><span class="dv trqpk"> - </span></div>`+
          `<div class="trq-only"><span class="dl">Duty rms 60 s</span><span class="dv rms"> - </span></div>`+
          `<div class="trq-only"><span class="dl">Speed rpm</span><span class="dv rpmv"> - </span></div>`+
          `<div class="trq-only"><span class="dl">Guard</span><span class="dv guardv"> - </span></div>`+
          `<div class="pos-only acc-c"><span class="dl">Accel mm/s² · peak / limit</span><span class="dv acc"> - </span><span class="abar"><span class="fill"></span><span class="lim"></span></span><span class="dl clip"> - </span></div>`+
        `</div>`+
        `<div class="axf"><span class="sp"></span><button type="button" class="key sm nolamp axsetup" title="Open this axis in Setup">Axis setup ›</button></div>`+
      `</div>`;
    w.querySelector('.axr').addEventListener('click',e=>{ if(e.target.closest('button,input,select,a')) return; axToggle(i); });
    w.querySelector('.axsetup').addEventListener('click',()=>{ setView('setup'); setupPage('axes');
      const s=$('axisSel'); if(s){ s.value=String(i); s.dispatchEvent(new Event('change')); } });
    driveGrid.appendChild(w);
    if(rail){ const r=document.createElement('button'); r.type='button'; r.className='rx';
      r.innerHTML='<span class="lamp"></span><span class="glass rb"><i></i></span><span class="rxn"></span>';
      r.onclick=()=>{ setLayout('full',uiLayout.hp); axToggle(i,true); }; rail.appendChild(r); }
  }
  while(driveGrid.children.length>drives.length){ driveGrid.removeChild(driveGrid.lastChild);
    if(rail&&rail.lastElementChild) rail.removeChild(rail.lastElementChild); }

  let anyBad=false;
  drives.forEach((d,i)=>{
    const card=driveGrid.children[i], rx=rail?rail.children[i]:null;
    // Status from the shared model (StatusModel via /api/status `ind`): canonical
    // colour class + verbatim text. Proven identical to the former local derivation
    // by TestStatusModel's 210-combo golden-reference sweep. The raw statusword is
    // still shown separately in the detail.
    const ind = d.ind || {};
    const stext = ind.text || ' - ';
    const sclass = ind.cls || 's-off';
    const lampc = LAMP_OF[sclass]||'';
    if(sclass==='s-fault'||sclass==='s-estop') anyBad=true;
    const isTorque = d.mode==='torque';
    card.className='axw '+sclass+(sclass==='s-fault'?' fault':'')+(isTorque?' torque':'')+(card.classList.contains('open')?' open':'');
    card.querySelector('.dot').className='lamp dot '+lampc;
    card.querySelector('.nmt').textContent  = d.name || ('Drive '+i);
    card.querySelector('.typ').textContent = (TYPE[d.axisType] || ' - ').toLowerCase();
    const dir=card.querySelector('.dir');
    dir.textContent = (typeof d.invertDir==='boolean') ? ' · '+(d.invertDir?'rev':'fwd') : '';
    // status: chosen status text (the strip, and the compact layout's line under the name) + raw statusword hex
    const st=card.querySelector('.stxt'); st.textContent = stext; st.className='c-st vt stxt '+(lampc||'dim');
    card.querySelector('.stt').textContent = stext;
    card.querySelector('.sw').textContent   = ('sw' in d) ? 'SW '+hex4(d.sw) : '';
    // decoded fault line: precise Er name once the 0x203F read lands, else
    // the coarse 603F class. Hidden entirely when not faulted.
    const fEl=card.querySelector('.flt');
    if(fEl){
      if(ind.fault && (ind.faultCode||ind.faultText)){
        fEl.textContent=[ind.faultCode,ind.faultText].filter(Boolean).join(' · ');
        fEl.style.display='';
      } else fEl.style.display='none';
    }
    // Rotary lever: labels read in degrees. Swapped once per kind change.
    const rotCard = d.axisType==='rotary_lever' ? '1':'0';
    if(card.dataset.rot!==rotCard){
      card.dataset.rot=rotCard;
      const rot = rotCard==='1';
      const lbl=(sel,lin,deg)=>{ const el=card.querySelector(sel); if(el){ const dl=el.parentElement.querySelector('.dl'); if(dl) dl.textContent=rot?deg:lin; } };
      lbl('.dv.tgt','Target mm','Target °'); lbl('.dv.velpk','Peak vel mm/s','Peak vel °/s'); lbl('.dv.acc','Accel mm/s² · peak / limit','Accel °/s² · peak / limit');
    }
    // config line: stroke · pitch (static); torque axes have no stroke
    if(isTorque){
      card.querySelector('.dcfg').textContent = 'torque · CST';
      // Torque telemetry (see WebServer: cmdTrq/rms/rpm/igbtC/guard).
      const set=(sel,txt)=>{ const e=card.querySelector(sel); if(e) e.textContent=txt; return e; };
      const cmd = typeof d.cmdTrq==='number'? d.cmdTrq.toFixed(1) : ' - ';
      set('.cmdtrq', cmd); set('.cmdtrq2', cmd);
      const rmsEl=set('.rms', typeof d.rms==='number'? d.rms.toFixed(0)+'%' : ' - ');
      if(rmsEl) rmsEl.style.color = typeof d.rms!=='number' ? ''
        : d.rms>105 ? 'var(--bad)' : d.rms>85 ? 'var(--warn)' : 'var(--ok)';   // vs RATED; >100 = i2t clock
      set('.rpmv', typeof d.rpm==='number'? Math.abs(d.rpm).toFixed(0) : ' - ');
      const GUARD={overspeed:'OVERSPEED TRIP',travel:'TRAVEL TRIP',relaxed:'RELAXED'};
      const gEl=set('.guardv', GUARD[d.guard]||' - ');
      if(gEl) gEl.style.color = d.guard==='relaxed' ? 'var(--warn)' : d.guard ? 'var(--bad)' : '';
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
    // metrics - current in the strip, peaks in the detail (sticky per session)
    const pk=peaks[i]||(peaks[i]={vel:0,trq:0});
    if(typeof d.vel==='number' && Math.abs(d.vel)>pk.vel) pk.vel=Math.abs(d.vel);
    if(typeof d.trq==='number' && d.trq>pk.trq) pk.trq=d.trq;
    setSeg(card.querySelector('.pos'), typeof d.pos==='number'? d.pos.toFixed(3):'--.---');
    card.querySelector('.tgt').textContent   = typeof d.target==='number'? d.target.toFixed(3):' - ';
    card.querySelector('.vel').textContent   = typeof d.vel==='number'? d.vel.toFixed(2):' - ';
    card.querySelector('.velpk').textContent = typeof d.vel==='number'? pk.vel.toFixed(2):' - ';
    card.querySelector('.trq').textContent   = typeof d.trq==='number'? d.trq.toFixed(1):' - ';
    card.querySelector('.trqpk').textContent = typeof d.trq==='number'? pk.trq.toFixed(1):' - ';
    const igbt=card.querySelector('.igbt'); igbt.textContent = typeof d.igbtC==='number'? d.igbtC.toFixed(0) : ' - ';
    // position bar (position over stroke) with the target tick; torque bar
    const stroke=(typeof d.strokeMm==='number'&&d.strokeMm>0)?d.strokeMm:0;
    const pf=card.querySelector('.pf'), tt=card.querySelector('.tt');
    const posPct=(stroke&&typeof d.pos==='number')? Math.max(0,Math.min(100,d.pos/stroke*100)) : 0;
    pf.style.width=posPct.toFixed(1)+'%';
    if(stroke&&typeof d.target==='number'){ tt.style.display='block'; tt.style.left=Math.max(0,Math.min(100,d.target/stroke*100)).toFixed(1)+'%'; }
    else tt.style.display='none';
    const tf=card.querySelector('.tf'); const trqPct=(typeof d.trq==='number')? Math.max(0,Math.min(100,d.trq)) : 0;
    tf.style.width=trqPct.toFixed(1)+'%'; tf.classList.toggle('hot', trqPct>80);
    // Peak WINDOWED commanded accel vs the Amax limit -- the real headroom gauge. It's the
    // macroscopic accel (|dvel| over ~10 ms), NOT per-cycle: the per-cycle value pegs at
    // exactly Amax on every frame-boundary discontinuity, so it read 100% at every Amax.
    // The windowed value dilutes those single-cycle corners and reads BELOW Amax when you
    // have headroom -- so the ratio finally means something. Paired with the accel-clamp
    // clip rate, the braking-clamp bind rate, and peak following-error. All server-latched
    // since loop-start/soft-reset (drive, pause, then read -- motion pauses on focus-loss).
    // '.dv.acc', not '.acc': a parked axis's lamp carries the colour class acc
    // and comes first in the row, and the readout used to land in it.
    const accEl=card.querySelector('.dv.acc'), bar=card.querySelector('.abar'), fill=card.querySelector('.fill');
    const amax=(typeof d.maxAccel==='number' && d.maxAccel>0)? d.maxAccel : 0;
    const clip=(typeof d.accelClipPct==='number')? d.accelClipPct : 0;
    const brake=(typeof d.accelBindPct==='number')? d.accelBindPct : 0;
    if(typeof d.accelPeakMms2==='number' && amax){
      const pct=d.accelPeakMms2/amax*100;
      accEl.textContent = `${d.accelPeakMms2.toFixed(0)} / ${amax.toFixed(0)} (${pct.toFixed(0)}%)`;
      fill.style.width = Math.min(pct/125*100,100).toFixed(1)+'%';   // track = 0..125% Amax, limit marked at 80%
      const hot = pct>=95 || clip>=10;   // at the accel limit, or the clamp is binding a lot
      bar.classList.toggle('hot',hot);
      accEl.style.color = hot ? 'var(--bad)' : '';
    } else { accEl.textContent=' - '; accEl.style.color=''; fill.style.width='0%'; bar.classList.remove('hot'); }
    const clipEl=card.querySelector('.clip');
    clipEl.textContent = (typeof d.accelClipPct==='number')? `${clip.toFixed(1)}% clip · ${brake.toFixed(1)}% brake` : ' - ';
    const ferrEl=card.querySelector('.ferr');
    ferrEl.textContent = (typeof d.followErrPeak==='number')? d.followErrPeak.toFixed(3) : ' - ';
    // the rail's item: lamp, a bar of position (or torque for torque axes), a short name
    if(rx){ rx.querySelector('.lamp').className='lamp '+lampc; rx.querySelector('.rb i').style.height=(isTorque?trqPct:posPct).toFixed(1)+'%';
      const nm=rx.querySelector('.rxn'); const sn=(d.name||('D'+i)).slice(0,5); if(nm.textContent!==sn) nm.textContent=sn; rx.title=d.name||''; }
  });
  const ml=$('moLamp'); if(ml) ml.className='lamp '+(anyBad?'bad':loopRunning?'ok':'');
}

/* ---- layout state: Motion full or rail, Haptics mix or rail (the editor
   state arrives with the Haptics step). Motion on the rail leaves Haptics
   the width; Haptics on the rail forces Motion full. Persists per browser. ---- */
const LAYOUT_KEY='nullcat.ui.layout';
let uiLayout={mo:'full',hp:'mix'};
try{ const l=JSON.parse(localStorage.getItem(LAYOUT_KEY)); if(l&&typeof l==='object') Object.assign(uiLayout,l); }catch(_){}
function applyLayout(){ const ws=$('ws'); if(!ws) return; ws.dataset.mo=uiLayout.mo; ws.dataset.hp=uiLayout.hp; }
/* Haptics available or not (hapInit decides: a torque axis or a shaker, and
   the experimental tickbox). Without it the Haptics panel is hidden and the
   workspace is Motion alone, full width. */
function hapAvail(on){ const ws=$('ws'); if(!ws) return; ws.dataset.nohap=on?'0':'1';
  if(!on&&(uiLayout.mo!=='full'||uiLayout.hp!=='mix')){ if(uiLayout.hp==='ed') hapCloseEd(); else setLayout('full','mix'); } }
function setLayout(mo,hp){
  { const ws=$('ws'); if(ws&&ws.dataset.nohap==='1'){ mo='full'; hp='mix'; } }   // no Haptics panel: Motion has the width
  if(hp==='rail') mo='full';
  // Motion on the rail leaves Haptics the width: the editor opens (on the
  // selected effect, else the first) so the room is used.
  if(mo==='rail'&&hp!=='ed'){ hp='ed'; if(!(hapSel&&hapTiles[hapSel])){ const k=Object.keys(hapTiles)[0]; if(k){ uiLayout.mo=mo; hapOpenFx(k,true); return; } hp='mix'; mo='full'; } else { uiLayout.mo=mo; hapOpenFx(hapSel,true); return; } }
  if(hp==='ed'&&!(hapSel&&hapTiles[hapSel]&&hapTiles[hapSel].tile.classList.contains('ined'))) hp='mix';
  uiLayout.mo=mo; uiLayout.hp=hp; applyLayout();
  try{ localStorage.setItem(LAYOUT_KEY,JSON.stringify(uiLayout)); }catch(_){}
}
{ const PNL={'mo-rail':()=>setLayout('rail',uiLayout.hp),'mo-full':()=>setLayout('full',uiLayout.hp),'hp-rail':()=>setLayout('full','rail'),'hp-mix':()=>setLayout(uiLayout.mo,'mix')};
  document.querySelectorAll('[data-pnl]').forEach(b=>{ b.onclick=()=>{ const f=PNL[b.dataset.pnl]; if(f) f(); }; });
  // At load the effects do not exist yet (hapInit builds them once the
  // config arrives), and the effect state setLayout reads (hapSel,
  // hapTiles) is declared further down: calling setLayout here threw on a
  // stored "editor open" or "Motion on the rail" layout and stopped the
  // whole page. Show the stored layout without the editor; the uiLayout
  // kept as stored lets hapMixerBuild reopen the editor once the tiles exist.
  { const ws=$('ws'); if(ws){ ws.dataset.mo=(uiLayout.hp==='rail')?'full':(uiLayout.mo==='rail'?'rail':'full');
      ws.dataset.hp=(uiLayout.hp==='rail')?'rail':'mix'; } } }

/* ---- connection (freshness) ---- */
/* Link-lost guard: on Disconnected (>=5s = 10 missed polls) grey EVERY button and show the
   banner, so a dead network path LOOKS dead instead of silently eating clicks against a
   frozen page (eth1 carrier-drop incident). Cleared by the next successful applyState:
   fault/restart/shutdown are restored here (applyState never touches them); the rest are
   recomputed by applyState's own state logic every poll. */
let linkDown=false;
function setLinkDown(d){ if(d===linkDown) return; linkDown=d; setTimeout(fitHeader,0);
  const b=$('linkBanner'); if(b) b.hidden=!d;
  if(d) Object.values(btn).forEach(x=>{ if(x) x.disabled=true; });
  else ['fault','restart','shutdown'].forEach(k=>{ if(btn[k]) btn[k].disabled=false; });
}
function updateConn(){ const age=lastRecvMs?(Date.now()-lastRecvMs):Infinity;
  if(age<1500) setV(elConn,'Connected','ok');
  else if(age<5000) setV(elConn,'Stale','warn');
  else { setV(elConn,'Disconnected','bad'); setLogo('is-offline'); setLinkDown(true); } }
setInterval(updateConn,500);
setInterval(()=>{ setSeg(elClock,new Date().toTimeString().slice(0,8)); },1000);

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
/* ---- Setup pages. One page shows at a time; the others stay in the
   document, so every control keeps its id and handler. Selecting the tests
   page does what opening its panel did (build the axis list, poll), and
   the page keeps the old panel's 'collapsed' class while another page is
   up, which is what pollTestStatus reads. The choice persists per browser. */
const SETUP_PAGES={axes:['Rig','Axes'],blend:['Rig','Blend & conditioning'],shk:['Outputs','Shakers / audio'],btn:['Inputs','Button box'],
  sim:['Inputs','Sim channels'],dev:['Devices','Force-feel devices'],prov:['Commissioning','Drive provisioning'],test:['Commissioning','Commissioning tests'],sys:['Host','System']};
let setupCur='axes'; try{ setupCur=localStorage.getItem('nullcat.ui.setup')||'axes'; }catch(_){}
function setupPage(id,atLoad){
  if(!SETUP_PAGES[id]) id='axes';
  setupCur=id; try{ localStorage.setItem('nullcat.ui.setup',id); }catch(_){}
  document.querySelectorAll('#su .supg').forEach(p=>{ const on=p.dataset.pg===id; p.classList.toggle('on',on); if(p.id==='testPanel') p.classList.toggle('collapsed',!on); });
  document.querySelectorAll('#sun [data-su]').forEach(b=>b.classList.toggle('on',b.dataset.su===id));
  const [k,t]=SETUP_PAGES[id]; const kk=$('suKick'), tt=$('suTitle'); if(kk) kk.textContent=k; if(tt) tt.textContent=t;
  { const sh=$('shkHead'); if(sh) sh.hidden=id!=='shk'; }   // the Shakers key lives in that page's head
  if(id==='test'&&!atLoad){ buildTestAxes(); pollTestStatus(); }
}
document.querySelectorAll('#sun [data-su]').forEach(b=>{ b.onclick=()=>setupPage(b.dataset.su); });
// At load the tests page's state (tstDrives) is declared further down, so a
// stored tests page used to stop the whole script here: its build and poll
// run at the end of the boot instead.
setupPage(setupCur,true);
/* Nav summaries and lamps, and the tests page's interlock lamps, from the
   state the page already holds (config copy, bindings, the last status). */
let lastStatus=null;
function setupNavRefresh(){
  const sum=(id,t,l)=>{ const b=document.querySelector('#sun [data-su="'+id+'"]'); if(!b) return;
    const s=b.querySelector('[data-sum]'); if(s&&s.textContent!==t) s.textContent=t;
    const lp=b.querySelector('.lamp'); if(lp) lp.className='lamp'+(l?' '+l:''); };
  const drv=(cfgObj&&cfgObj.drives)||[]; sum('axes', drv.length+(drv.length===1?' axis':' axes')+' · slave order', '');
  const cm=$('cf-condmode'), bt=$('cf-blendt');
  sum('blend', (cm&&cm.selectedIndex>=0?cm.options[cm.selectedIndex].text:'-')+(bt&&bt.value!==''?' · '+bt.value+' s':''), '');
  const sk=$('cf-shakers'), nsh=hapShakerCount(); sum('shk', sk&&sk.checked?(nsh+' ch'):'off', sk&&sk.checked?'ok':'');
  { const key=$('shkKey'); if(key&&sk) key.classList.toggle('on',sk.checked); }
  shkMapSync();
  const nb=(btnMap.bindings||[]).length; sum('btn', nb+' of '+BINDABLE.length+' bound', nb?'ok':'');
  sum('sim', ((cfgObj&&cfgObj.ncxBindings)||[]).length+' slots · not for SimHub', '');
  const nd=cfgObj?devAxes().length:0; sum('dev', nd?nd+(nd>1?' devices':' device'):'none', nd?'ok':'');
  { // force-feel devices run on the Raspberry Pi only: no page for them elsewhere
    const b=document.querySelector('#sun [data-su="dev"]'), off=!!meta.platform&&meta.platform!=='linux';
    if(b&&b.hidden!==off){ b.hidden=off; const g=b.previousElementSibling; if(g&&g.classList.contains('sung')) g.hidden=off;
      if(off&&document.querySelector('#su .supg.on[data-pg="dev"]')) setupPage('axes'); } }
  sum('prov', 'A6-EC · params · role check', '');
  sum('test', tstActive?'running…':(tstLoopRunning?'ready':'needs the loop running'), tstActive?'warn':(tstLoopRunning?'ok':''));
  const au=$('cf-authen'); sum('sys', 'v'+(meta.version||'?')+(au&&!au.checked?' · web password off':''), au&&!au.checked?'warn':'');
  const s=lastStatus||{}; const il={loop:!!s.loopRunning, parked:!!s.parked, notel:!s.telemetryReceiving};
  document.querySelectorAll('#testInterlock [data-il]').forEach(l=>{ const on=!!il[l.dataset.il]; l.className='lamp'+(on?' ok':''); const t=l.nextElementSibling; if(t) t.className='vt'+(on?' ok':' dim'); });
}
setInterval(setupNavRefresh,1000);
// Button bindings: collapsed by default. It is a commissioning-time task, and
// left expanded its full-width panel pushed the config Save bar off screen.
{ const h=$('bindHead'); if(h) h.onclick=()=>toggle($('bindBody'),$('bindArrow')); }
// Sim channels (numbered line): folded by default too, below the shaker
// settings and the bindings (a SimHub setup never opens it).
{ const h=$('ncxHead'); if(h) h.onclick=()=>toggle($('ncxBody'),$('ncxArrow')); }
/* The activity log is a drawer above the action bar, opened by the bar's
   Log key; Auto-scroll is a latching key around the checkbox pollLogs reads. */
{ const k=$('logKey'), d=$('logd'); if(k&&d) k.onclick=()=>{ const o=d.classList.toggle('open'); k.classList.toggle('on',o); };
  const t=$('chk-tail'), tk=$('tailKey'); if(t&&tk) t.onchange=()=>tk.classList.toggle('on',t.checked); }
/* The status window's System cell opens Setup. */
{ const b=$('sysBtn'); if(b) b.onclick=()=>{ setView('setup'); setupPage('sys'); }; }

/* Persisted collapse for the Operate sections (fight the sprawl): the
   choice sticks per browser. Clicks on embedded actions (Reset peaks)
   never toggle. */
function wireCollapse(headId, arrowId, panelId, key){
  const h=$(headId), a=$(arrowId), p=$(panelId); if(!h||!a||!p) return;
  let c=false; try{ c=localStorage.getItem(key)==='1'; }catch(_){}
  const apply=()=>{ p.classList.toggle('collapsed',c); a.textContent=c?'▸':'▾'; };
  apply();
  h.onclick=(e)=>{ if(e.target.closest('.act,button,input,select,label')) return;
    c=!c; try{ localStorage.setItem(key,c?'1':'0'); }catch(_){} apply(); };
}
// The Motion and Effects section headers went with the panel redesign (a
// panel minimises to its rail instead), so nothing collapses on Operate now.

/* ---- theme menu (header) ---- */
document.querySelectorAll('#thmMenu [data-t]').forEach(b=>{ b.onclick=()=>{ theme=applyTheme(b.dataset.t);
  try{ localStorage.setItem(THEME_KEY,theme); }catch(_){} const m=$('thmMenu'); if(m) m.open=false; }; });
document.addEventListener('click',(e)=>{ const m=$('thmMenu'); if(m&&m.open&&!m.contains(e.target)) m.open=false; },true);

/* ---- config: split host.json (per-machine) + rig.json (portable).
   The web owns rig on every platform; host only when there is no native config
   UI (hostOwner === "web"). On "native" the host fields are read-only here.
   Restart to apply. ---- */
let cfgObj=null;          // merged working model for the axis editor (host + rig.global + drives)
let meta={hostOwner:'web'};
function setField(id,v){ const el=$(id); if(!el||v==null) return; if(el.type==='checkbox') el.checked=!!v; else el.value=v; }

// host.json inputs - disabled when a native app owns host (hostOwner==="native").
const HOST_INPUT_IDS=['cf-authen','cf-authtok','cf-sim','cf-nic','cf-hz','cf-wd','cf-dc','cf-bind','cf-wport','cf-sport','cf-sbind',
  'cf-loglvl','cf-logfile','cf-logcon','cf-diag','cf-temppoll','cf-cmdsync','cf-wkccyc','cf-wkcthr','cf-capscan','cf-gpiomode','cf-ledtest',
  'cf-showdev','cf-shakers','cf-audiodev','cf-audioch','cf-audiodev2','cf-audioch2'];
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
    ? 'Rig & axis settings: portable rig config, saved here. Settings marked live apply on Save; the rest apply on the next Stop → Re-initialize, and a save that changes any of those waits for it whole.'
    : 'Rig & axis settings: saved here. Settings marked live (and device feel) apply on Save; the rest apply on the next Stop → Re-initialize, and a save that changes any of those waits for it whole. Host/network settings need a service restart.';
}

/* ---- UI state clarity ----
   Three field states, made visible: EDITED-UNSAVED (amber row markers, Save(N),
   leave warning -- client-side vs the loaded snapshot); SAVED-NOT-APPLIED
   (SERVER-owned pending flags from /api/meta -> persistent pill, survives
   reloads and agrees across clients); APPLIED (neither). */
const CFG_WATCH_IDS=[...HOST_INPUT_IDS.filter(id=>id!=='cf-ledtest'),
  'cf-blendt','cf-blendv','cf-condmode','cf-reqreset'];
let cfgBaseline=null;
/* The apply generation the loaded rig copy belongs to (X-Nullcat-Gen on
   GET /api/rig). Sent back with every save: if the controller rewrote the
   haptics meanwhile (follow car, a pull, a starting point, a profile) the
   save is refused (409) instead of writing this page's stale copy over it. */
let cfgGenLoaded=null;
/* Shaker shaping state (the editor lives with shakerStatus below). Declared
   up here because loadConfig, called at start-up further down, reads it
   (shkSchemaLoad) before the script reaches that section. */
const SHK={ schema:null, shapes:[], n:0, sel:-1, band:1, built:'', previewed:false, bypass:false, editGen:0,
            send:{}, tone:{busy:false,again:false}, toneCh:-1, testKind:[], testT0:[], testHz:[], gr:[], lv:[], grD:[], lvD:[], raf:0, mT:0,
            copying:false, copy:new Set(), drag:null };
function fieldVal(el){ return el.type==='checkbox'?el.checked:el.value; }
function snapshotCfg(){
  const f={}; for(const id of CFG_WATCH_IDS){ const el=$(id); if(el) f[id]=fieldVal(el); }
  return { fields:f, drives:((cfgObj&&cfgObj.drives)||[]).map(d=>JSON.stringify(d)),
           haptics:JSON.stringify((cfgObj&&cfgObj.haptics)||{}),
           ncx:JSON.stringify((cfgObj&&cfgObj.ncxBindings)||[]),
           shk:JSON.stringify((cfgObj&&cfgObj.shakerShaping)||[]) };
}
/* Dirty = anything saveConfig() would post that differs from the loaded
   baseline. Haptics and bindings MUST be in here: they used to be left out,
   so a haptics-only edit left Save greyed out and could never reach the
   controller (a bench session concluded the changes "were not sticking"). */
function dirtyState(){
  const out={count:0, ids:new Set(), axisKeys:new Set(), host:false, haptics:false, shk:false};
  if(!cfgBaseline) return out;
  for(const id of CFG_WATCH_IDS){ const el=$(id); if(!el) continue;
    if(fieldVal(el)!==cfgBaseline.fields[id]){ out.count++; out.ids.add(id);
      if(HOST_INPUT_IDS.includes(id)) out.host=true; } }
  if(JSON.stringify((cfgObj&&cfgObj.haptics)||{})!==cfgBaseline.haptics){ out.count++; out.haptics=true; }
  if(JSON.stringify((cfgObj&&cfgObj.ncxBindings)||[])!==cfgBaseline.ncx) out.count++;
  if(JSON.stringify((cfgObj&&cfgObj.shakerShaping)||[])!==cfgBaseline.shk){ out.count++; out.shk=true; }
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
  const b=$('cfgSave'); if(b){ b.textContent=d.count?`Save (${d.count} change${d.count>1?'s':''})`:'Save to rig.json'; b.disabled=!d.count; }
  // The haptics strip lives in the Operate view, a screen away from the
  // Setup save bar: it carries its own unsaved marker + Save.
  const hb=$('hapSaveBar'); if(hb){ hb.hidden=!d.count;
    const m=$('hapDirtyMsg'); if(m) m.textContent=d.haptics?'unsaved haptics changes (nothing plays until saved)':`${d.count} unsaved change${d.count>1?'s':''}`; }
  shkDirtyUI(d);
  window.__cfgDirty=d.count;
}
window.addEventListener('beforeunload',e=>{ if(window.__cfgDirty){ e.preventDefault(); e.returnValue=''; } });
{ const p=$('cfgPanel'); if(p){ p.addEventListener('input',refreshDirtyUI); p.addEventListener('change',refreshDirtyUI); } }
async function refreshPendingPill(fetchMeta){
  if(fetchMeta){ try{ const r=await fetch(API+'/api/meta'); if(r.ok) meta=await r.json(); }catch(_){} }
  // A tab left open across a version update keeps running the OLD page
  // logic (the server already sends no-store, but nothing re-fetches a
  // running page). Reload automatically when the server version changes,
  // unless there are unsaved edits - then just say so.
  if(meta.version){
    if(!window.__pageVer) window.__pageVer=meta.version;
    else if(meta.version!==window.__pageVer){
      if(!window.__cfgDirty){ location.reload(); return; }
      const p=$('cfgPending');
      if(p){ p.style.display=''; p.textContent='controller updated to v'+meta.version+' - reload this page'; }
      return;
    }
  }
  const pill=$('cfgPending'); if(!pill) return;
  const rig=!!meta.rigPendingRestart, host=!!meta.hostPendingRestart;
  pill.style.display=(rig||host)?'':'none';
  // Say which action is actually owed. Haptics/device saves never light
  // this (they apply live); the server only flags what it has not picked up.
  const restart=(meta.hostOwner==='native')?'restart nullCAT':'restart the service';
  pill.textContent=(rig&&host)?`rig + host saved - re-initialize, then ${restart}`
                  :rig?'rig saved - re-initialize to apply'
                  :`host settings saved - ${restart} to apply`;
}
setInterval(()=>refreshPendingPill(true),15000);
/* ---- The bar's message glass mirrors the Setup head: the save status
   (#cfgStatus, several writers in saveConfig and applyToAll) for 10 s after
   it changes, else the pending pill (#cfgPending: what the controller has
   not picked up yet, until a re-initialize or restart clears it). A command
   refusal has the glass while it shows. Same text and colours; observers
   follow the existing writers rather than touching each one. ---- */
let barNoteUntil=0, barNoteTimer=null;
function barNoteSync(){
  const el=$('barNote'), err=$('cmdErr'), st=$('cfgStatus'), pill=$('cfgPending'); if(!el) return;
  if(err&&!err.hidden){ el.hidden=true; return; }
  const saveOn=!!(st&&st.textContent&&performance.now()<barNoteUntil);
  const pendOn=!!(pill&&pill.style.display!=='none'&&pill.textContent);
  if(saveOn){ el.textContent=st.textContent; el.style.color=st.style.color||''; el.className='vt'; }
  else if(pendOn){ el.textContent=pill.textContent; el.style.color=''; el.className='vt warn'; }
  el.title=(saveOn||pendOn)?el.textContent:''; el.hidden=!(saveOn||pendOn);
}
{ const st=$('cfgStatus'), pill=$('cfgPending'), err=$('cmdErr'), all={childList:true,characterData:true,subtree:true,attributes:true};
  if(st) new MutationObserver(()=>{ barNoteUntil=performance.now()+10000; if(barNoteTimer) clearTimeout(barNoteTimer); barNoteTimer=setTimeout(barNoteSync,10100); barNoteSync(); }).observe(st,all);
  if(pill) new MutationObserver(barNoteSync).observe(pill,all);
  if(err) new MutationObserver(barNoteSync).observe(err,{attributes:true}); }

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
      <span class="bindcur${b?'':' unbound'}${cap?' cap':''}">${cap?'press a button…':(b?(b.label||(b.vendor+':'+b.product+' #'+b.code)):' - ')}</span>
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
    const [mR,hR,rR]=await Promise.all([fetch(API+'/api/meta'),fetch(API+'/api/host'),fetch(API+'/api/rig'),hapLoadSchema(),shkSchemaLoad()]);
    meta=mR.ok?await mR.json():{hostOwner:'web'};
    updInit();
    const host=hR.ok?await hR.json():{};
    const g=(rR.ok?await rR.json():{}); const rig=g.global||{};
    { const gh=rR.ok?rR.headers.get('X-Nullcat-Gen'):null; cfgGenLoaded=(gh!==null&&gh!==''&&isFinite(+gh))?+gh:null; }
    cfgObj=Object.assign({},host,rig,{ drives:g.axes||[], numDrives:(g.axes||[]).length, _configVersion:g.configVersion||2 });
    shkLoad(rig.shakerShaping);
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
    setField('cf-shakers',host.shakersEnabled);
    // The device list form wins when present; the simple pair is device 1.
    const ad=Array.isArray(host.audioDevices)&&host.audioDevices.length?host.audioDevices:[{device:host.audioDevice||'',channels:host.audioChannels||2}];
    setField('cf-audiodev',ad[0].device||''); setField('cf-audioch',ad[0].channels||2);
    setField('cf-audiodev2',ad[1]?(ad[1].device||''):''); setField('cf-audioch2',ad[1]?(ad[1].channels||2):2);
    $('cf-audiodev2')._present=!!ad[1];
    shakerDevicesLoad();
    setField('cf-authen',!!host.webAuthToken); setField('cf-authtok',host.webAuthToken||'');
    // rig.global fields (portable feel/policy)
    setField('cf-blendt',rig.blendTimeSec); setField('cf-blendv',rig.blendMaxVelocityMmS);
    setField('cf-condmode',rig.conditioningMode||'bypass'); updateCondNote();
    setField('cf-reqreset',rig.requireUserFaultReset);
    applyHostOwnership();
    shkMapSync();
    devInit();
    hapInit();
    ncxInit();
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
            conditioningMode:$('cf-condmode').value, requireUserFaultReset:$('cf-reqreset').checked,
            haptics:cfgObj.haptics||{},
            ncxBindings:cfgObj.ncxBindings||[],
            shakerShaping:cfgObj.shakerShaping||[] };
  cfgObj.conditioningMode=g.conditioningMode;   // keep merged view in sync for axis logic
  const rig={ configVersion:cfgObj._configVersion||2, numDrives:(cfgObj.drives||[]).length, global:g, axes:cfgObj.drives||[] };
  const dirty=dirtyState();
  const shkGen=SHK.editGen;   // an edit made while the save is out is not in it
  // What this save sends is the clean baseline afterwards: an edit made
  // while it is out stays unsaved (it used to be taken as saved, and the
  // leave warning and the shaping preview's revert then lost it).
  const sentBaseline=snapshotCfg();
  st.textContent='Saving…'; st.style.color='var(--ink-soft)';
  const hm=$('hapMsg');
  try{
    const hdr=(typeof cfgGenLoaded==='number')?{'X-Nullcat-Gen':String(cfgGenLoaded)}:{};
    const rr=await fetch(API+'/api/rig',{method:'POST',headers:hdr,body:JSON.stringify(rig)}); const rj=await rr.json();
    if(rr.status===409){
      // The controller rewrote the haptics after this page loaded them
      // (follow car, a pull, a starting point, a profile): the page's copy
      // is stale and nothing was saved. Reload it (the edits on the strip
      // are lost; the other sections are re-read too) and say what
      // happened, rather than merge in silence.
      const why='NOT SAVED: '+(rj.error||'the settings changed on the controller')+' The page has reloaded the controller\'s settings; make your edits again and Save.';
      await loadConfig();
      st.textContent='✗ '+why; st.style.color='var(--danger)';
      if(hm) hm.textContent=why;
      return; }
    if(!rj.ok){ st.textContent='✗ rig: '+(rj.error||'save failed'); st.style.color='var(--danger)';
      // Mirror the refusal where the user is likely editing: the Save bar
      // can be a screen away from the Devices card (a bench session lost
      // an evening to a refusal rendered out of view).
      const m=$('devMsg'); if(m) m.textContent='SAVE REFUSED - nothing was saved: '+(rj.error||'save failed');
      if(hm) hm.textContent='SAVE REFUSED - nothing was saved: '+(rj.error||'save failed');
      return; }
    // A save that changed the file moved the generation: this page's copy
    // is the new one (its next save carries it), and its own poll need not
    // reload it. Another page holding the older copy is refused (409).
    if(typeof rj.gen==='number'){ cfgGenLoaded=rj.gen; hapCarGenLive=rj.gen; }
    if(SHK.editGen===shkGen) SHK.previewed=false;   // the controller now plays the saved shaping
    let hj=null;
    // host.json - only when the web owns it (headless; on "native" the
    // desktop app owns it) AND a host field actually changed. Posting it
    // on every save used to bump its mtime and light the restart pill
    // every time, and every restart was a first init.
    if(meta.hostOwner==='web'&&dirty.host){
      // Web password tickbox semantics: unticked -> "" (auth off); ticked
      // needs an actual password (an empty one would lock nobody out and
      // read as protection).
      if($('cf-authen')&&$('cf-authen').checked&&!$('cf-authtok').value){
        st.textContent='✗ Web password: ticked but empty - type a password or untick.';
        st.style.color='var(--danger)'; return; }
      const host={ configVersion:cfgObj._configVersion||2,
        webAuthToken:($('cf-authen')&&$('cf-authen').checked)?$('cf-authtok').value:'',
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
        webShowDevices:$('cf-showdev').checked,
        shakersEnabled:$('cf-shakers').checked, audioDevice:$('cf-audiodev').value.trim(), audioChannels:Math.max(1,Math.min(8,+$('cf-audioch').value||2)),
        audioDevices:shakerDeviceList() };
      const hr=await fetch(API+'/api/host',{method:'POST',body:JSON.stringify(host)}); hj=await hr.json();
      if(!hj.ok){ st.textContent='✗ host: '+(hj.error||'save failed'); st.style.color='var(--danger)'; return; }
    }
    // Say exactly what is owed, from what the server reports it could not
    // apply live: haptics + device feel land immediately; other rig fields
    // wait for a re-initialize; host fields wait for a restart. Native
    // (desktop) reloads config in-process (QFileSystemWatcher), so host
    // changes there never go through here.
    const restart=(meta.hostOwner==='native')?'restart nullCAT':'restart the service';
    const owed=[];
    if(rj.needsInit) owed.push('re-initialize EtherCAT to apply axis/rig settings');
    if(hj&&hj.needsRestart) owed.push(restart+' to apply host settings');
    const applyMsg=owed.length?'Saved ✓ '+owed.join('; ')+'.'
                  :(rj.changed?'Saved ✓ applied live.':'Saved ✓ nothing changed.');
    st.textContent=applyMsg+(clamped?` (${clamped} field${clamped>1?'s':''} clamped to safe range)`:''); st.style.color='var(--ok)';
    { const m=$('devMsg'); if(m&&m.textContent.startsWith('SAVE REFUSED')) m.textContent='Saved.'; }
    if(hm){ hm.textContent=dirty.haptics?(rj.changed?'Saved - haptics applied live.':'Saved.'):''; setTimeout(()=>{ if(hm.textContent.startsWith('Saved')) hm.textContent=''; },4000); }
    cfgBaseline=sentBaseline; refreshDirtyUI();   // what was saved = the new clean baseline
    refreshPendingPill(true);                      // server now reports what is pending
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
  {k:'hapticsMaxMm',label:'Haptic max',type:'num',min:0,max:10,step:0.1,unit:'mm',pos:true,
   tip:'Largest offset a haptic effect may add to this axis\'s position (0 = this axis takes no haptics). Effects are also kept to what the axis can follow at their frequency, using the pos budget share of its velocity and acceleration limits.'},
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
  {k:'torqueMaxPct',label:'Torque max',type:'num',min:0,max:300,step:1,unit:'%',torque:true,tip:'Tension at full-scale telemetry (or at g at max, with the cue from g) - the overall strength of the effect. The drive 0x6072 limit still caps above it.'},
  {k:'beltSlewPctPerSec',label:'Slew cap',type:'num',min:100,max:20000,step:100,unit:'%/s',torque:true,tip:'How fast the belt tension from your motion telemetry (or from g) may change, each control cycle. It shapes the tension cue, limits how far a frame that jumps can move the belt, and paces the ease to minimum when telemetry stops and the relaxer ease. The haptic effects on the belt are not limited by it. Lower is softer; 20000 is the default.'},
  {k:'beltOverspeedRpm',label:'Overspeed',type:'num',min:50,max:6000,step:10,unit:'rpm',torque:true,tip:'Snapped-belt detector: shaft speed sustained above this trips to slack. Set above any speed a worn strap can reach in use.'},
  {k:'beltOverspeedMs',label:'Overspeed time',type:'num',min:20,max:5000,step:10,unit:'ms',torque:true,tip:'How long the overspeed must persist before tripping - haptic flicks and hand pulls reset it.'},
  {k:'beltMaxTravelRevs',label:'Travel cap (0=off)',type:'num',min:0,max:100,step:0.5,unit:'revs',torque:true,tip:'Net winding since tension-up. Catches the SLOW runaway an rpm limit never sees (bare shaft idling along). A strapped belt cannot reach it at any speed.'},
  {k:'beltMaxRpm',label:'Speed fold (0=off)',type:'num',min:0,max:3000,step:50,unit:'rpm',torque:true,tip:'THE enforced speed limit on these drives (their own CST speed objects do not restrain): tension folds to zero above this, capping the slack take-up lunge.'},
  {k:'beltRelaxerSec',label:'Relaxer (0=off)',type:'num',min:0,max:120,step:1,unit:'s',torque:true,tip:'Sustained near-max tension for this long eases to min (at the Slew cap rate) until demand drops - pre-empts the drive thermal fault that would park the whole rig.'},
  {k:'beltRelaxerPct',label:'Relaxer band',type:'num',min:20,max:100,step:1,unit:'%',torque:true,tip:'Fraction of Torque max that counts as near-max for the relaxer dwell.'},
  {k:'beltSource',label:'Belt cue from',type:'select',opts:['motion','g'],torque:true,tip:'motion: the tension your motion software sends on this axis (as before). g: nullCAT pulls the belt straight from the car\'s g (surge: braking and acceleration; sway: cornering; heave: crests and compressions; needs a protocol 1.9 sender such as the SimHub plugin 1.9), shaped by the settings below; the motion software\'s value for this axis is then unused.'},
  {k:'beltGMode',label:'Surge',type:'select',live:true,opts:['decel','accel','both','off'],torque:true,tip:'With the cue from g, the g along the car: decel tightens under braking, accel under acceleration, both under either, off leaves surge out (a belt on sway or heave alone). If it tightens the wrong way round, pick the other one (senders differ on the sign).'},
  {k:'beltGSway',label:'Sway',type:'select',live:true,opts:['off','left','right','both'],torque:true,tip:'With the cue from g, the g across the car: it adds to the pull in left-hand corners, right-hand corners or both. With a belt on each shoulder, give each its own side. If a belt tightens in the wrong corners, pick the other side.'},
  {k:'beltGSwayMix',label:'Sway share',type:'num',live:true,min:0,max:3,step:0.05,torque:true,tip:'With sway on: how much a g of sway pulls against a g of surge (1 = the same).'},
  {k:'beltGHeave',label:'Heave',type:'select',live:true,opts:['off','lift','press','both'],torque:true,tip:'With the cue from g, the vertical g: it adds to the pull over crests (lift: the car goes light), into compressions (press) or both. Only the change counts: gravity and long hills never tighten the belt. If it is the wrong way round, pick the other one.'},
  {k:'beltGHeaveMix',label:'Heave share',type:'num',live:true,min:0,max:3,step:0.05,torque:true,tip:'With heave on: how much a g of heave pulls against a g of surge (1 = the same).'},
  {k:'beltGFromG',label:'g from',type:'num',live:true,min:0,max:2,step:0.01,unit:'g',torque:true,tip:'With the cue from g: below this the belt stays at Torque min (a dead band for the small g of cruising).'},
  {k:'beltGFullG',label:'g at max',type:'num',live:true,min:0.2,max:6,step:0.05,unit:'g',torque:true,tip:'With the cue from g: the g that pulls the belt to Torque max. Lower makes it stronger; road cars brake at about 1 g, race cars 2 to 4 g.'},
  {k:'beltGCurve',label:'g curve',type:'num',live:true,min:0.3,max:3,step:0.05,torque:true,tip:'With the cue from g: 1 is a straight line from g from to g at max. Above 1 is softer at small g and steeper near the top; below 1 the reverse.'},
  {k:'beltGSmoothMs',label:'g smoothing',type:'num',live:true,min:0,max:250,step:5,unit:'ms',torque:true,tip:'With the cue from g: how much the g is smoothed (it arrives about 60 times a second from most senders, about 333 with direct telemetry). 20 to 40 ms takes the steps out at 60; more is softer but later.'},
  {k:'beltGCrashPct',label:'g after a crash',type:'num',live:true,min:0,max:100,step:5,unit:'%',torque:true,tip:'With the cue from g: after a crash (a hit past 8 g that loses 30 km/h or more), if the car is still a second on and its g still rough, as the physics of a wrecked car send noise, the g counts at this share until it settles for a second, you drive off, reset or change car. The crash itself plays as it always did; a reset, and most crashes, change nothing. 100 = off.'},
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
    const e0=effSpec(f,d);
    // a setting that applies on Save (no re-initialize) says so beside its name
    const e=f.live?Object.assign({},e0,{label:e0.label+'<i class="ftag" title="Applies on Save with the loop running, unless the same save also changes a setting that needs a re-initialize">live</i>'}):e0;
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
    if(f.k==='beltGFullG'&&d.beltSource==='g'){ const tg=+((cfgObj&&cfgObj.haptics&&cfgObj.haptics.beltFullG)||0);
      if(tg>0) h+=`<div class="fldtip" style="color:var(--warnInk)">The type of car sets ${tg} g now (Haptics, Rig · car · sim): that one is in force. This one is used when it is 0.</div>`; }
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
// Support bundle: fetched (so the auth header rides along when a web
// password is set) and handed to the browser as a normal file save.
{ const dl=$('cf-logdl'); if(dl) dl.onclick=async()=>{
    try{
      const r=await fetch(API+'/api/logbundle'); if(!r.ok) return;
      const a=document.createElement('a');
      a.href=URL.createObjectURL(await r.blob());
      a.download='nullcat-support-bundle.txt'; a.click();
      setTimeout(()=>URL.revokeObjectURL(a.href),5000);
    }catch(_){}
  }; }
// Provisioning role check: reads drive-resident params (loop stopped) and
// flags any that disagree with the axis's configured role.
{ const rc=$('pc-rolecheck'); if(rc) rc.onclick=async()=>{
    const m=$('pc-rolemsg'), t=$('pc-roletbl');
    const esc=(v)=>String(v).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
    if(m) m.textContent='Reading drives…'; if(t) t.innerHTML='';
    try{
      const r=await fetch(API+'/api/provision/rolecheck'); const j=await r.json();
      if(!j.ok){ if(m) m.textContent=j.error||'check failed'; return; }
      if(m) m.textContent=j.rows.length?'':'No slaves configured to check.';
      if(t&&j.rows.length){
        let bad=0,h='<table class="prov-table" style="table-layout:auto"><tr><th>axis</th><th>slave</th><th>role</th><th>param</th><th>drive has</th><th>should be</th><th></th></tr>';
        for(const w of j.rows){
          h+=w.error!==undefined
            ?`<tr><td>${esc(w.name)}</td><td>${w.slave}</td><td>${w.role}</td><td colspan="4">${esc(w.error)}</td></tr>`
            :`<tr><td>${esc(w.name)}</td><td>${w.slave}</td><td>${w.role}</td><td>${esc(w.param)}</td><td>${w.actual}</td><td>${w.expected}</td><td>${w.match?'OK':'<b>SET ON PANEL</b>'}</td></tr>`;
          if(w.error!==undefined||!w.match) bad++;
        }
        t.innerHTML=h+'</table>';
        if(m) m.textContent=bad?bad+' item(s) need attention - fix on the drive panel, save, power-cycle.':'All drives match their roles.';
      }
    }catch(e){ if(m) m.textContent='check failed: '+e; }
  }; }
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
const devSweep={};  // per-axis {min,max} travel swept since homing (diagnostic row)
const devCapA={};   // two-press travel capture: first end, awaiting the second
const devDot={};    // per-svg live-dot updaters (curve editors)
function devEnabled(){ return meta.platform==='linux' && !!($('cf-showdev')&&$('cf-showdev').checked); }
function devAxes(){ return ((cfgObj&&cfgObj.drives)||[]).map((d,i)=>({d,i})).filter(x=>isDeviceType(x.d.axisType)); }
/* ---- haptics strip: one tile per effect, left to right; params write into
   cfgObj.haptics (rig.global, saved by the normal Save; restart-free apply
   comes later). Routes edit in a drawer below the strip so the strip stays
   short. Waveform previews are drawn from the same math the engine runs. */
/* The effect list comes from the controller (GET /api/haptics/schema, built
   from HapticsRegistry.h), so this page carries no copy of it: HAP_FX is
   the tile list and HAP_DEF the per-effect defaults, both filled by
   hapLoadSchema() during loadConfig(). A tile is built from its schema
   entry: label, kind, the channels it needs, the tunables with ranges (a
   param with opts renders as a choice), and the tip. */
let HAP_FX=[], HAP_DEF={};
// The hover note on a setting the car table sets (tinted on the tile), and
// the one it carries instead while the car has no entry and follow car put
// these settings on their defaults (tinted pink).
const HAP_CARKEY_TIP='Follows the car: a car\'s entry in the car table can set this (follow car, or re-apply), and "save as mine" keeps yours. Untinted settings are general: a new car leaves them as they are.';
// ...and on a setting the type of car sets (a starting point: Road car, Cup
// car, GT3, Open wheeler), tinted in its own colour.
const HAP_TYPEKEY_TIP='Follows the type of car: applying a starting point (Haptics panel, Rig · car · sim: road car, cup car, GT3, open wheeler, rally gravel or tarmac) sets this. A new car leaves it as it is.';
const HAP_CARDEFAULT_TIP='The car table has no entry for this car, so this setting is on its default rather than the last car\'s. Pick a car like it in the preset box and apply it, or tune it and "save for this car".';
let hapCarDefaultOn=false;   // the car tiles are on the defaults (the car has no entry)
function hapCarKeyTip(){ return hapCarDefaultOn?HAP_CARDEFAULT_TIP:HAP_CARKEY_TIP; }
// Pink tint and the defaults note on every car-owned row while the car has
// no entry; the faint car tint and the usual note otherwise.
function hapCarDefaultMark(on){
  hapCarDefaultOn=!!on;
  const strip=$('hapStrip'); if(strip) strip.classList.toggle('cardefault',hapCarDefaultOn);
  const ed=$('hapEd'); if(ed) ed.classList.toggle('cardefault',hapCarDefaultOn);
  document.querySelectorAll('#hapStrip .hr.carkey').forEach(row=>{
    const help=row.dataset.phelp||'';
    row.dataset.help=hapCarKeyTip()+(help?'\n\n'+help:''); row.title=row.dataset.help;
  });
}
/* The sim channel token registry also comes from the schema (NcxTokens.h):
   the bindings editor's list, the order of the status ncxHave[] array, the
   wire's slot count, and the route parts the per-wheel effects take. */
let NCX_TOKENS=[], NCX_DEFAULT=[], NCX_MAX_SLOTS=48, HAP_PARTS=['all'];
async function hapLoadSchema(){
  try{
    const r=await fetch(API+'/api/haptics/schema'); if(!r.ok) return;
    const j=await r.json();
    HAP_FX=(j.effects||[]).map(e=>({
      k:e.key, label:e.label, transient:e.kind==='transient', slip:e.kind==='slip', parts:!!e.parts, fxIdx:e.fxIdx,
      id:e.id, carKeys:e.carKeys||[], typeKeys:e.typeKeys||[], chan:e.channels||[], tip:e.tip||'',
      params:(e.params||[]).map(p=>[p.key,p.label,p.min,p.max,p.step,p.opts,p.help||''])}));
    HAP_DEF={};
    for(const e of j.effects||[]) HAP_DEF[e.key]=Object.assign({routes:[]},e.defaults||{});
    NCX_TOKENS=j.tokens||[];
    NCX_DEFAULT=NCX_TOKENS.map((t,i)=>({token:t,slot:i,scale:1,offset:0}));
    NCX_MAX_SLOTS=+j.maxSlots||48;
    HAP_PARTS=j.parts||['all'];
  }catch(_){}
}
/* A tile's channel entry, as the registry writes it: "a|b" = any of these,
   a trailing '*' = any of the four wheel tokens (slipAngle* covers
   slipAngleFL..RR), a leading '~' = optional (shown dim, never as missing).
   Returns {label, optional, ok(haveArray)}. */
function hapChanSpec(entry){
  const optional=entry.startsWith('~'); const body=optional?entry.slice(1):entry;
  const alts=body.split('|');
  const expand=(a)=>a.endsWith('*')?NCX_TOKENS.filter(t=>t.startsWith(a.slice(0,-1))):[a];
  const altOk=(a,have)=>{ const toks=expand(a); return toks.length>0&&toks.every(t=>!!have[NCX_TOKENS.indexOf(t)]); };
  return {label:body, optional, alts,
    ok:(have)=>alts.some(a=>altOk(a,have)),
    // The alternative actually feeding the law: the first one present
    // (the laws prefer per-wheel data over a magnitude in the same order).
    live:(have)=>alts.find(a=>altOk(a,have))||null,
    // Chip text: the alternative actually feeding the tile (not the whole
    // list), with its value and session peak; a wheel group shows its
    // largest wheel now and its largest peak (slip peaks only count while
    // moving), the number to set peak % from. Flags and gear show no peak.
    text:(have,vals,pks)=>{
      const a=alts.find(x=>altOk(x,have));
      if(!a) return body;
      if(a.endsWith('*')){
        let v=0, pk=0;
        for(const t of expand(a)){ const i=NCX_TOKENS.indexOf(t); if(i<0) continue;
          v=Math.max(v,Math.abs(+vals[i]||0)); pk=Math.max(pk,Math.abs(+pks[i]||0)); }
        return a+' '+hapNum(v)+'/'+hapNum(pk);
      }
      const i=NCX_TOKENS.indexOf(a); if(i<0) return body;
      if(HAP_FLAG_TOKENS.includes(a)) return a;
      const v=+vals[i]||0, pk=+pks[i]||0;
      return a+' '+hapNum(v)+(a==='gear'?'':'/'+hapNum(pk)); }};
}
const HAP_FLAG_TOKENS=['absActive','limiter','tcActive','pitLimiter'];
// Which status hapWheels set a tile's per-wheel line reads (-1 = none):
// 0 lateral severity, 1 longitudinal severity, 2 road mm, 3 on a kerb 0..1,
// 4 flat spot mm, 5 impact ring. Sets 6..9 (ground contact, surface class,
// water, floating) are read by the Road cells and the Surface line.
function hapWheelSet(k){ return k==='slipLat'?0:k==='slipLon'?1:k==='road'?2:k==='kerb'?3:k==='wheels'?4:k==='impacts'?5:-1; }
const HAP_SURF_NAMES=['tarmac','bumpy','kerb','gravel','grass','dirt','cobbles','snow','ice','sand','mud','water'];
// Road: the settings each model reads (the rest are dimmed), and the
// roadNoise texture's, live only while the model has nothing to run on.
const HAP_ROAD_MODEL_KEYS=[['fullMm','fullLearn','surface','surfaceHz','surfaceKmh','rough','suspPct','suspHz','bodyHz','hopHz','damping'],
  ['bodyMm','rough','suspPct','suspHz','bodyHz','hopHz','damping'],['bodyMm']];
const HAP_ROAD_ALL_KEYS=['fullMm','fullLearn','surface','surfaceHz','surfaceKmh','bodyMm','rough','suspPct','suspHz','bodyHz','hopHz','damping'];
const HAP_ROAD_TEX_KEYS=['freqHz','jitter','peakPct'];
// The dimmed settings of the Road and Rumble strip tiles with the reason, from the
// chosen model and what is arriving (null = the generic path trims).
function hapDimFor(k,dv,have,curbsNamed){
  const on=(t)=>{ const i=NCX_TOKENS.indexOf(t); return i>=0&&!!have[i]; };
  const all=(p)=>['FL','FR','RL','RR'].every(w=>on(p+w));
  if(k==='road'){
    const m=Math.round(+dv.model)||0;
    const corners=all('roadHeight')||all('suspVel')||all('suspTravel');
    const fed=(m===1)?(corners||on('speedKmh')):(m===2)?on('accHeave'):(all('suspVel')||all('suspTravel'));
    const dim={}; const name=['suspension','tyre','chassis'][m]||'suspension';
    HAP_ROAD_ALL_KEYS.forEach(x=>{ if(!HAP_ROAD_MODEL_KEYS[m].includes(x)) dim[x]='Not used by the '+name+' model'; });
    if(fed) HAP_ROAD_TEX_KEYS.forEach(x=>{ if(!(m===0&&x==='jitter')) dim[x]='The roadNoise texture: only when the model has nothing to run on'; });
    if(m===0&&Math.round(+dv.fullLearn)===1) dim.fullMm='Learn full mm is on: the value learned for this car plays (shown on the learn full mm row). Yours is kept for when it is off, and plays until the car has been learned.';
    return dim;
  }
  if(k==='abs'){
    const dim={};
    if(!(+dv.ampPct>0)){
      // Amp 0 = no ABS at all (a car without it): nothing plays, through the tyres either.
      ['slipLink','lockLink','scrubLink','fromWheels','lowOff','holdMs','brakeFull'].forEach(x=>{ dim[x]='ABS is off (amp 0): nothing of it plays, through the tyres either'; });
      return dim;
    }
    if(Math.round(+dv.slipLink)!==1){ dim.lockLink='Slip link is off'; dim.scrubLink='Slip link is off'; }
    return dim;
  }
  if(k==='driveline'){
    const dim={};
    if(!(+dv.diff>0)) ['diffType','diffHz','driven'].forEach(x=>{ dim[x]='Diff x is 0: no differential clunk (an open diff)'; });
    if(!(+dv.whine>0)) dim.whinePitch='Whine x is 0: no gear whine';
    return dim;
  }
  if(k==='kerb'){
    const dim={};
    if(all('surface')){ dim.peakPct='The sim sends the surface under each tyre: the curbs channel is not used'; dim.detectMm=dim.heights='The sim sends the surface under each tyre: no detection needed'; }
    else if(curbsNamed) dim.detectMm=dim.heights='The game sends its own kerb channel: no detection needed';
    else if(all('roadHeight')) dim.peakPct='No curbs channel: kerbs come from the road heights (from road heights, detect mm)';
    if(!dim.detectMm&&!(Math.round(+dv.heights)===1)) dim.detectMm='From road heights is off: no kerbs are looked for in the road heights';
    if(!(Math.round(+dv.step)===1)) dim.riseMm='Step is off: only the ribs play';
    return dim;
  }
  return null;
}
// The trims that belong to one input path or the other: when per-wheel
// data is live the magnitude trim is dimmed, and vice versa, so only the
// trim that acts on what is arriving reads as live.
const HAP_TRIM_WHEEL=['peakDeg','peakRatio','fullMm','hpHz'], HAP_TRIM_MAG=['peakPct'];
// Compact number for the readouts: integers as is, else up to 2 decimals.
function hapNum(v){ v=+v||0; return Number.isInteger(v)?String(v):(Math.abs(v)>=100?v.toFixed(0):Math.abs(v)>=10?v.toFixed(1):v.toFixed(2)); }

/* Every axis is a routing destination. A torque axis (belt, device) takes
   a plain gain (x); a position (CSP) axis takes a gain in mm at 100%
   effect amplitude, capped at its hapticsMaxMm and derated by the
   controller to what the axis can follow at the effect's carrier (the
   haptic share of its velocity/acceleration limits). PP-mode axes and axes
   with hapticsMaxMm 0 take no haptics and are not listed. */
function hapRouteAxes(){
  const out=[]; if(!cfgObj||!cfgObj.drives) return out;
  cfgObj.drives.forEach((d,i)=>{
    const torque=(d.mode==='torque'||d.axisType==='shifter'||d.axisType==='pedal');
    if(torque){ out.push({i,name:d.name||('Axis '+(i+1)),kind:'torque',unit:'x',max:2,step:0.1}); return; }
    // A file that predates the field means the controller's default (3 mm);
    // only an explicit 0 opts the axis out.
    const cap=(d.hapticsMaxMm===undefined||d.hapticsMaxMm===null||d.hapticsMaxMm==='')?3:+d.hapticsMaxMm;
    if(d.mode==='pp'||!(cap>0)) return;
    out.push({i,name:d.name||('Axis '+(i+1)),kind:'position',unit:'mm',max:cap,step:0.1,
      maxV:+d.maxVelocityMmS||0,maxA:+d.maxAccelerationMmS2||0,cap:cap}); });
  return out;
}
// Largest peak a position axis can take at a carrier (mm): the controller's
// rule, min(cap, budget*maxV/w, budget*maxA/w^2), so the drawer can say
// "up to X mm at F Hz" before anything is saved or played.
function hapAllowedMm(a,hz){
  const budget=(cfgObj&&cfgObj.haptics&&isFinite(+cfgObj.haptics.positionBudget))?+cfgObj.haptics.positionBudget:0.4;
  let allowed=a.cap;
  if(hz>0){ const w=2*Math.PI*hz; allowed=Math.min(allowed,budget*a.maxV/w,budget*a.maxA/(w*w)); }
  return allowed;
}
// The carrier a hint should use for an effect: its freq, or the engine's
// idle rock (crank rate at ~800 rpm) since that is what a vertical carries.
function hapHintHz(fx,dv){
  if(fx.k==='rpmVibe') return 13;
  if(fx.k==='slipLat') return +dv.slideHz>0?+dv.slideHz:11;
  if(fx.k==='slipLon') return +dv.lockHz>0?+dv.lockHz:9;
  if(fx.k==='road'||fx.k==='kerb'||fx.k==='surface') return 8;   // no single carrier: the bump rate the controller derates at
  if(fx.k==='wheels') return 11;   // the wheel's rate at about 80 km/h
  if(fx.k==='impacts') return 15;  // a post's jolt
  return +dv.freqHz>0?+dv.freqHz:30;
}

// The wave has ONE meaning: playing right now. Idle draws a flat line
// whatever the amplitude (the header dot is the enabled indicator); a
// live Test passes a scrolling phase and the shape appears only then.
/* The tile wave is a flat line when idle and, while the effect plays, a
   picture of THIS effect as configured: height = amp % x master gain
   (x the live level for telemetry-driven effects), cycle density follows
   the carrier frequency on a log scale (9 Hz slow and fat, 45 Hz dense),
   jitter roughens it, and a transient's envelope width follows its length
   in ms with the carrier cycles it really fits. ph = animation phase
   (seconds); level = 0..1 live level, default 1 (Test preview). */
function hapWave(svg,fx,dv,ph,level){
  const W=148,H=26,pts=[];
  const mg=(cfgObj&&cfgObj.haptics&&typeof cfgObj.haptics.masterGain==='number')?cfgObj.haptics.masterGain:1;
  const A=Math.max(0,Math.min(1,(dv.ampPct/100)*mg*(level===undefined?1:level)));
  const live=(ph!==undefined&&A>0), p=ph||0, n=96;
  // Engine has no fixed carrier (freq 0 = follows rpm): draw the canned
  // idle the Test preview plays, 1100 rpm x cylinders / 2.
  const f=(+dv.freqHz>0)?+dv.freqHz:(1100/60*(+dv.cylinders||4)/2);
  const cyc=Math.max(1.5,Math.min(14,2+3*Math.log2(f/8)));
  for(let s=0;s<=n;s++){
    const t=s/n; let y=0;
    if(live){
      if(fx.transient){
        const w=Math.max(0.2,Math.min(1,(+dv.durMs||18)/60));         // envelope width
        const u=t/w, ramp=0.3;
        const env=(u<=0||u>=1)?0:(u<ramp?0.5*(1-Math.cos(Math.PI*u/ramp))
          :(u>1-ramp?0.5*(1-Math.cos(Math.PI*(1-u)/ramp)):1));
        const tc=Math.max(1,Math.min(8,f*(+dv.durMs||18)/1000*1.5));  // carrier cycles inside
        y=env*Math.sin(2*Math.PI*tc*u+p*18);
      }else{
        const jit=dv.jitter?(Math.sin(s*12.9898)*0.5*dv.jitter):0;
        y=Math.sin(2*Math.PI*(cyc+jit*cyc*0.6)*t+p*10);
      }
      y*=A;
    }
    pts.push((t*W).toFixed(1)+','+(H/2-y*(H/2-2)).toFixed(1));
  }
  svg.innerHTML='<polyline points="'+pts.join(' ')+'"/>';
  svg.setAttribute('viewBox','0 0 '+W+' '+H);
}
// Scroll the preview for ms, then settle back to the flat line. tag
// names the source on the wave ('test') while it runs.
function hapAnimate(svg,fx,dv,ms,tag){
  const t0=performance.now();
  const frame=(now)=>{
    const el=now-t0;
    if(el>=ms){ hapWave(svg,fx,dv); hapWaveTag(svg,''); return; }
    hapWave(svg,fx,dv,el/1000);
    if(tag) hapWaveTag(svg,tag);
    requestAnimationFrame(frame);
  };
  requestAnimationFrame(frame);
}
// The source tag in the wave's corner: 'test' during a preview, 'live x'
// (the live level) while sim data drives the effect, empty when idle.
function hapWaveTag(svg,text){
  const el=svg&&svg.parentElement?svg.parentElement.querySelector('[data-wtag]'):null;
  if(el&&el.textContent!==text) el.textContent=text;
}

function hapChipText(dv){
  const n=(dv.routes||[]).filter(x=>x&&x.axis>=0&&x.gain>0).length;
  const m=(dv.routes||[]).filter(x=>x&&x.shaker>=0&&x.gain>0).length;
  if(!n&&!m) return '→ not routed';
  return '→ '+[n?n+(n>1?' axes':' axis'):'',m?m+(m>1?' shakers':' shaker'):''].filter(Boolean).join(' + ');
}
// The device list as the host form has it: device 1 always, device 2
// when it has a name (or the file already had a second entry).
function shakerDeviceList(){
  const out=[{device:$('cf-audiodev').value.trim(),channels:Math.max(1,Math.min(8,+$('cf-audioch').value||2))}];
  const d2=$('cf-audiodev2');
  if(d2&&(d2.value.trim()||d2._present)) out.push({device:d2.value.trim(),channels:Math.max(1,Math.min(8,+$('cf-audioch2').value||2))});
  return out;
}
// Shaker channels the host exposes (0 when shakers are off): the sum over
// the devices, capped at 8.
function hapShakerCount(){
  if(!cfgObj||!cfgObj.shakersEnabled) return 0;
  const ad=Array.isArray(cfgObj.audioDevices)&&cfgObj.audioDevices.length?cfgObj.audioDevices:[{channels:cfgObj.audioChannels||2}];
  return Math.max(0,Math.min(8,ad.reduce((t,d)=>t+Math.max(1,Math.min(8,+d.channels||2)),0)));
}
// Host settings: the playback devices the backend sees, for the picker.
async function shakerDevicesLoad(){
  const dl=$('audioDevList'); if(!dl) return;
  try{ const r=await fetch(API+'/api/shakers/devices'); if(!r.ok) return; const j=await r.json();
    dl.innerHTML=(j.devices||[]).map(d=>'<option value="'+String(d.name).replace(/"/g,'&quot;')+'">'+(d.default?'(default) ':'')+'</option>').join('')+'<option value="null">no hardware</option>'; }catch(_){}
}
// The sound cards' status, one line per card in the glass (its lamp, name,
// the shakers it carries, rate, latency, the card's own buffer, underruns;
// a card not open: its error), Reopen beside it while a card is not open;
// then one strip per shaker (shkStripsBuild). Twice a second.
function shakerStatus(s){
  const line=$('shakerLine'); if(!line||!s) return;
  const k=s.shakers||{}, ro=$('shakerReopen');
  const put=h=>{ if(line._h!==h){ line._h=h; line.innerHTML=h; } };
  if(!k.enabled){
    put('<div class="sk-dv"><span class="lamp"></span><span class="vt dim">shakers off</span></div>');
    if(ro) ro.hidden=true;
    for(let c=0;c<8;c++){ SHK.lv[c]=0; SHK.gr[c]=0; SHK.testHz[c]=0; }
    shkStripsBuild(0); return;
  }
  const devs=k.devices||[];
  const kv=(l,v,cls)=>'<span class="sk-kv">'+(l?'<span class="dl">'+l+'</span>':'')+'<span class="vt'+(cls?' '+cls:'')+'">'+v+'</span></span>';
  // A card with fewer channels than asked for: the shakers past its own
  // are silent, and its line says which.
  put(devs.length?devs.map(d=>{
    const name=esc(d.name||'(default)'), f=d.first|0;
    if(!d.open) return '<div class="sk-dv"><span class="lamp bad"></span><span class="vt">'+name+'</span>'+kv('not open',esc(d.error||'not open'),'bad')+'</div>';
    const want=d.channelsWanted||d.channels, short=d.channels<want;
    let h='<div class="sk-dv"><span class="lamp '+(short?'warn':'ok')+'"></span><span class="vt">'+name+'</span>'
      +kv(d.channels>1?'shakers':'shaker',d.channels>1?(f+1)+'-'+(f+d.channels):String(f+1));
    if(short) h+=kv('',d.channels+' of '+want+' channels: '+(want-d.channels>1?'shakers '+(f+d.channels+1)+'-'+(f+want):'shaker '+(f+want))+' silent','warn');
    h+=kv('rate',d.rate+' Hz')+kv('latency',(d.latencyMs!=null?d.latencyMs:d.bufferMs)+' ms')+kv('card',d.bufferMs+' ms')
      +kv('underruns',d.underruns|0,d.underruns>0?'warn':'');
    if(d.volumeResets>0) h+=kv('volume','turned down '+d.volumeResets+'x by something else, put back to full','warn');
    return h+'</div>';
  }).join(''):'<div class="sk-dv"><span class="lamp'+(k.open?' ok':' bad')+'"></span><span class="vt">'+(k.open?'open':'not open')+'</span></div>');
  if(ro) ro.hidden=!devs.some(d=>!d.open);
  // The meters, from each card's levels (0..1, the output's peak), gain
  // reduction (dB) and test tone (Hz); a shaker on a card not open reads 0.
  for(let c=0;c<8;c++){ SHK.lv[c]=0; SHK.gr[c]=0; SHK.testHz[c]=0; }
  for(const d of devs){
    const f=d.first|0;
    (d.levels||[]).forEach((v,i)=>{ if(f+i<8) SHK.lv[f+i]=+v||0; });
    (d.gr||[]).forEach((v,i)=>{ if(f+i<8) SHK.gr[f+i]=+v||0; });
    (d.testHz||[]).forEach((v,i)=>{ if(f+i<8) SHK.testHz[f+i]=+v||0; });
  }
  shkStripsBuild(k.channels||0);
  // The strips stay (their shaping can be edited with no card open); the
  // test keys need a card.
  document.querySelectorAll('#shakerTests [data-shtest]').forEach(b=>{ b.disabled=!k.open; });
  shkMetersKick();
  shkStripsSync();   // a strip's test tag
  shkLive();
}
// The sound card rows: the shakers each card's channels become (from the
// fields, so what a restart will give), and the quick picks follow the
// fields' ownership (the desktop app owns them on the PC).
function shkMapSync(){
  const d1=$('cf-audiodev'), d2=$('cf-audiodev2'); if(!d1) return;
  const ch=id=>Math.max(1,Math.min(8,+($(id)||{}).value||2));
  const n1=ch('cf-audioch'), has2=!!d2&&!!(d2.value.trim()||d2._present), n2=has2?ch('cf-audioch2'):0;
  const rng=(a,b)=>a>8?'→ none (8 shakers at most)':(Math.min(8,b)>a?'→ shakers '+a+'-'+Math.min(8,b):'→ shaker '+a);
  const m0=$('shkMap0'), m1=$('shkMap1');
  if(m0) m0.textContent=rng(1,n1);
  if(m1) m1.textContent=has2?rng(n1+1,n1+n2):'';
  document.querySelectorAll('[data-shq]').forEach(b=>{ b.disabled=d1.disabled; b.classList.toggle('on',d1.value.trim()===b.dataset.shq); });
  const hint=d1.disabled?'the sound cards are set in the desktop app':'device and channel changes apply after a restart';
  for(const id of ['shkHeadHint','shkCardsHint']){ const e=$(id); if(e) e.textContent=hint; }
  const sk=$('cf-shakers'), t=$('shkKeyT'); if(sk&&t) t.textContent=sk.checked?'Shakers on':'Shakers off';
}
{ document.querySelectorAll('[data-shq]').forEach(b=>{ b.onclick=()=>{ const d=$('cf-audiodev'); if(!d||d.disabled) return;
    d.value=b.dataset.shq; d.dispatchEvent(new Event('input',{bubbles:true})); }; });
  for(const id of ['cf-audiodev','cf-audioch','cf-audiodev2','cf-audioch2']){ const e=$(id); if(e) e.addEventListener('input',shkMapSync); }
  // The Shakers key sits in the page head, outside the settings panel.
  const sk=$('cf-shakers'); if(sk) sk.addEventListener('change',()=>{ shkMapSync(); refreshDirtyUI(); });
  // Open every card again with the same request (shown while one is down).
  const ro=$('shakerReopen'); if(ro) ro.onclick=async()=>{ ro.disabled=true; try{ await postCmd('/api/shakers/reopen'); }finally{ ro.disabled=false; } };
}

/* ---- Shaker shaping: each shaker's EQ and compressor, played at the sound
   card (ShakerShape.h on the controller). Each shaker's strip carries its
   EQ curve and EQ and CMP keys (in or out); clicking the strip opens its
   editor below the strips, the EQ and the compressor side by side. Every
   edit plays on the shaker at once (POST /api/shakers/shape: a preview,
   never saved) and marks the page unsaved; Save writes rig.json's
   global.shakerShaping and the controller applies the saved shaping; this
   page left with an edit unsaved (reloaded, closed, or its settings read
   again) puts the saved shaping back. The limits, band types and the flat
   default come from /api/shakers/schema, the curves from the same formulas
   the controller runs. ---- */
const SHK_TYPE_LABEL={highpass:'High-pass',lowshelf:'Low shelf',bell:'Bell',highshelf:'High shelf',lowpass:'Low-pass'};
const shkClone=o=>JSON.parse(JSON.stringify(o));
const shkHasGain=t=>t==='bell'||t==='lowshelf'||t==='highshelf';
async function shkSchemaLoad(){
  if(SHK.schema) return SHK.schema;
  try{ const r=await fetch(API+'/api/shakers/schema'); if(r.ok) SHK.schema=await r.json(); }catch(_){}
  return SHK.schema;
}
// rig.json's entries (only the shaped shakers, by number) to one whole
// shape per shaker, and back. Starting from the default keeps the key order
// the same, so an untouched page compares equal to what it loaded.
function shkFromEntries(entries){
  const out=[]; for(let i=0;i<8;i++) out.push(shkClone(SHK.schema.default));
  for(const e of (Array.isArray(entries)?entries:[])){
    const n=(e&&e.shaker)|0; if(n<1||n>8) continue; const s=out[n-1];
    if(e.eq&&typeof e.eq==='object'){
      if(typeof e.eq.on==='boolean') s.eq.on=e.eq.on;
      (Array.isArray(e.eq.bands)?e.eq.bands:[]).slice(0,s.eq.bands.length).forEach((b,i)=>{
        if(b&&typeof b==='object') for(const key of Object.keys(s.eq.bands[i])) if(b[key]!==undefined) s.eq.bands[i][key]=b[key]; });
    }
    if(e.comp&&typeof e.comp==='object') for(const key of Object.keys(s.comp)) if(e.comp[key]!==undefined) s.comp[key]=e.comp[key];
  }
  return out;
}
function shkToEntries(shapes){
  const d=JSON.stringify(SHK.schema.default), out=[];
  shapes.forEach((s,i)=>{ if(JSON.stringify(s)!==d) out.push(Object.assign({shaker:i+1},shkClone(s))); });
  return out;
}
// loadConfig: the saved shaping in (normalised, so an untouched page is
// never "unsaved"); a preview this page left playing goes back to it.
function shkLoad(entries){
  if(!SHK.schema){ cfgObj.shakerShaping=Array.isArray(entries)?entries:[]; return; }
  SHK.shapes=shkFromEntries(entries);
  cfgObj.shakerShaping=shkToEntries(SHK.shapes);
  if(SHK.previewed) shkRevert(false);
  SHK.built=''; shkRender();
}
function shkRevert(leaving){
  SHK.previewed=false;
  try{ fetch(API+'/api/shakers/shape',{method:'POST',keepalive:!!leaving,body:JSON.stringify({revert:true})}).catch(()=>{}); }catch(_){}
}
window.addEventListener('pagehide',()=>{ if(SHK.previewed||SHK.bypass){ SHK.bypass=false; shkRevert(true); } });
window.addEventListener('pageshow',e=>{
  if(!e.persisted||!SHK.schema||!cfgBaseline) return;
  if(dirtyState().shk){ SHK.previewed=true; for(let c=0;c<SHK.n;c++) shkSend(c); }
  shkRender();
});
// One shaker's shaping to the card, the newest edit always last (one
// request in flight per shaker; a drag sends as fast as the controller
// answers, never a queue).
async function shkSend(c){
  const st=SHK.send[c]||(SHK.send[c]={busy:false,again:false});
  if(st.busy){ st.again=true; return; }
  st.busy=true;
  try{
    do{ st.again=false;
      await fetch(API+'/api/shakers/shape',{method:'POST',body:JSON.stringify({channel:c,shape:SHK.shapes[c],bypass:SHK.bypass&&c===SHK.sel})});
    }while(st.again);
  }catch(_){}finally{ st.busy=false; }
}
function shkEdited(c){
  SHK.previewed=true; SHK.editGen++;
  cfgObj.shakerShaping=shkToEntries(SHK.shapes);
  refreshDirtyUI();
  shkSend(c);
  if(SHK.toneCh===c&&(SHK.testHz[c]||0)>0) shkToneFollow();   // the band tone follows its band
  shkRender();
}
// A strip's EQ or CMP key: that one in or out (the editor's In/Out).
function shkToggle(c,part){
  const s=SHK.shapes[c]; if(!s) return;
  s[part].on=!s[part].on; shkEdited(c);
}

/* The strips: built once per channel count; shkStripsSync draws their
   state (selected, bypassed, copy picks, the EQ curve and keys),
   shkMetersDraw their meters. */
function shkStripsBuild(n){
  const bank=$('shakerTests'), none=$('shakerNone'); if(!bank) return;
  n=Math.max(0,Math.min(8,n|0));
  if(none) none.hidden=n>0;
  bank.hidden=!n;
  if(bank._n===n) return;
  bank._n=n; SHK.n=n;
  for(const c of [...SHK.copy]) if(c>=n) SHK.copy.delete(c);   // a pick past the shakers there are now
  bank.style.setProperty('--n',Math.max(1,n));
  bank.innerHTML=Array.from({length:n},(_,c)=>{ const N=c+1;
    return '<div class="sk-st" data-skst="'+c+'" tabindex="0" role="button" title="Shaker '+N+': click to tune its EQ and compressor">'
      +'<div class="sk-h"><span class="lamp" data-skl="'+c+'"></span><span class="glass sk-num"><span class="seg" data-g="8">'+N+'</span></span>'
      +'<span class="sk-tag" data-sktag="'+c+'"></span><span class="sp"></span>'
      +'<button class="key sm nolamp ico" type="button" data-shtest="'+c+'" title="Shaker '+N+': 40 Hz for a second on this channel">&#9654;</button></div>'
      +'<div class="glass sk-m"><span class="sk-lv" title="Shaker '+N+': level"><i data-lv="'+c+'"></i></span>'
      +'<span class="sk-gr" title="Shaker '+N+': gain the compressor is taking off (full: 24 dB)"><i data-gr="'+c+'"></i></span>'
      +'<span class="sk-rd"><span class="seg" data-skdb="'+c+'" data-g="-88">--</span><span class="dl">dBFS</span><span class="dl sk-grv" data-skgrv="'+c+'">GR 0.0</span></span></div>'
      +'<div class="glass sk-th" data-skthw="'+c+'" title="Shaker '+N+': its EQ curve"><svg viewBox="0 0 100 30" preserveAspectRatio="none"><line class="z" x1="0" x2="100"/><polyline data-skth="'+c+'"/></svg></div>'
      +'<div class="sk-k"><button class="key sm latch" type="button" data-sheq="'+c+'" title="Shaker '+N+': EQ in or out">EQ</button>'
      +'<button class="key sm latch" type="button" data-shcmp="'+c+'" title="Shaker '+N+': compressor in or out">CMP</button></div></div>'; }).join('');
  if(SHK.sel>=n) shkClose(); else shkRender();
  shkMetersDraw();
}
// A strip's EQ curve, 100 x 30, over the editor's range.
function shkThumb(eq){
  const L=SHK.schema.limits, f0=L.hz[0], f1=L.hz[1], top=L.db[1], bot=L.db[0];
  const Y=db=>((top-Math.max(bot,Math.min(top,db)))/(top-bot)*30).toFixed(2), pts=[];
  for(let i=0;i<=60;i++){ const f=f0*Math.pow(f1/f0,i/60); let db=0; for(const b of eq.bands) if(b.on) db+=shkBandResp(b,f,SHK_FS); pts.push((i/60*100).toFixed(2)+','+Y(db)); }
  return { z:Y(0), pts:pts.join(' ') };
}
function shkStripsSync(){
  const bank=$('shakerTests'); if(!bank) return;
  const open=SHK.sel>=0, cm=open&&SHK.copying;
  bank.classList.toggle('cm',cm);
  bank.querySelectorAll('[data-skst]').forEach(st=>{
    const c=+st.dataset.skst, s=SHK.shapes[c], sel=open&&c===SHK.sel, tgt=cm&&SHK.copy.has(c);
    st.classList.toggle('sel',sel); st.classList.toggle('byp',sel&&SHK.bypass);
    st.classList.toggle('src',cm&&sel); st.classList.toggle('tgt',tgt);
    st.title=cm?(sel?'Copying from this shaker':'Shaker '+(c+1)+': click to copy to it (again: not)'):'Shaker '+(c+1)+': click to tune its EQ and compressor';
    const tag=st.querySelector('[data-sktag]'); if(tag) tag.textContent=cm?(sel?'from':tgt?'copy':''):(sel&&SHK.bypass?'bypass':shkTestPlaying(c)?'test':'');
    const eq=st.querySelector('[data-sheq]'), cp=st.querySelector('[data-shcmp]');
    if(eq){ eq.classList.toggle('on',!!(s&&s.eq.on)); eq.disabled=!s; }
    if(cp){ cp.classList.toggle('on',!!(s&&s.comp.on)); cp.disabled=!s; }
    const w=st.querySelector('[data-skthw]'); if(w) w.classList.toggle('off',!(s&&s.eq.on));
    const pl=st.querySelector('[data-skth]');
    if(pl&&s&&SHK.schema){ const key=JSON.stringify(s.eq.bands); if(pl._k!==key){ pl._k=key; const t=shkThumb(s.eq);
      pl.setAttribute('points',t.pts); const z=st.querySelector('.sk-th .z'); if(z){ z.setAttribute('y1',t.z); z.setAttribute('y2',t.z); } } }
  });
}
// The meters between status readings (twice a second): each falls the way
// the card's own meter does (ShakerDsp.h kMeterReleaseSec, 0.3 s), never
// below the newest reading, and shows a higher one at once. Only the
// drawing: the readings are the card's. Runs while something is falling.
const SHK_METER_REL_SEC=0.3;
function shkMetersKick(){ if(!SHK.raf){ SHK.mT=0; SHK.raf=requestAnimationFrame(shkMetersTick); } }
function shkMetersTick(now){
  SHK.raf=0;
  const dt=SHK.mT?Math.min(0.5,Math.max(0,(now-SHK.mT)/1000)):0, k=Math.exp(-dt/SHK_METER_REL_SEC); SHK.mT=now;
  let falling=false;
  for(let c=0;c<8;c++){
    const lv=SHK.lv[c]||0, gr=SHK.gr[c]||0;
    let l=Math.max(lv,(SHK.lvD[c]||0)*k), g=Math.max(gr,(SHK.grD[c]||0)*k);
    if(l<0.004) l=lv; if(g<0.05) g=gr;   // under the ladders (-48 dB, GR 0.0): down to the reading
    SHK.lvD[c]=l; SHK.grD[c]=g;
    if(l>lv||g>gr) falling=true;
  }
  shkMetersDraw(); shkGrDraw();
  if(falling) SHK.raf=requestAnimationFrame(shkMetersTick);
}
// Level as dBFS (the output's peak; the ladder spans -48 to 0) and the
// compressor's gain reduction (the ladder full at 24 dB), as drawn.
function shkMetersDraw(){
  const bank=$('shakerTests'); if(!bank) return;
  for(let c=0;c<SHK.n;c++){
    const v=SHK.lvD[c]||0, db=v>1e-5?20*Math.log10(v):-120, gr=SHK.grD[c]||0;
    const lv=bank.querySelector('[data-lv="'+c+'"]'); if(!lv) continue;
    lv.style.height=Math.round(Math.max(0,Math.min(1,(db+48)/48))*100)+'%';
    const g=bank.querySelector('[data-gr="'+c+'"]'); if(g) g.style.height=Math.round(Math.max(0,Math.min(1,gr/24))*100)+'%';
    const r=bank.querySelector('[data-skdb="'+c+'"]'); if(r){ const t=db>-48?String(Math.round(db)):'--'; if(r.textContent!==t) r.textContent=t; }
    const gv=bank.querySelector('[data-skgrv="'+c+'"]'); if(gv){ gv.textContent='GR '+gr.toFixed(1); gv.classList.toggle('on',gr>0.2); }
    const l=bank.querySelector('[data-skl="'+c+'"]'); if(l) l.className='lamp'+(db>-1?' bad':db>-42?' ok':'');
  }
}
{ const bank=$('shakerTests');
  if(bank){
    const pick=c=>{
      if(SHK.sel>=0&&SHK.copying){ if(c!==SHK.sel){ if(SHK.copy.has(c)) SHK.copy.delete(c); else SHK.copy.add(c); shkCopySync(); } return; }
      if(!(SHK.sel===c&&!$('shkEd').hidden)) shkOpen(c);
    };
    bank.addEventListener('click',e=>{
      const t=e.target, st=t.closest('[data-skst]'); let b;
      if(st&&SHK.sel>=0&&SHK.copying){ pick(+st.dataset.skst); return; }   // copying: the whole strip picks it
      if((b=t.closest('[data-shtest]'))){ shkTest('burst',+b.dataset.shtest); return; }
      if((b=t.closest('[data-sheq]'))){ shkToggle(+b.dataset.sheq,'eq'); return; }
      if((b=t.closest('[data-shcmp]'))){ shkToggle(+b.dataset.shcmp,'comp'); return; }
      if(st) pick(+st.dataset.skst);
    });
    bank.addEventListener('keydown',e=>{
      if((e.key!=='Enter'&&e.key!==' ')||!e.target.matches('[data-skst]')) return;
      e.preventDefault(); pick(+e.target.dataset.skst);
    });
  }
}

/* The editor. */
function shkOpen(c){
  if(!SHK.schema||!SHK.shapes[c]) return;
  const ed=$('shkEd'); if(!ed) return;
  if(SHK.bypass&&SHK.sel!==c){ SHK.bypass=false; shkSend(SHK.sel); }
  if(SHK.sel!==c) shkCopyEnd();
  SHK.sel=c; ed.hidden=false;
  shkRender();
}
function shkClose(){
  if(SHK.bypass){ SHK.bypass=false; if(SHK.sel>=0) shkSend(SHK.sel); }
  shkCopyEnd();
  SHK.sel=-1; const ed=$('shkEd'); if(ed) ed.hidden=true;
  shkStripsSync();
}
function shkRender(){
  shkStripsSync();
  const ed=$('shkEd'); if(!ed||ed.hidden||SHK.sel<0||!SHK.shapes[SHK.sel]) return;
  const c=SHK.sel, s=SHK.shapes[c];
  $('shkEdN').textContent=c+1;
  $('shkEdPrev').disabled=c<=0; $('shkEdNext').disabled=c>=SHK.n-1;
  ed.classList.toggle('byp',SHK.bypass);
  const bp=$('shkEdBypass'); if(bp) bp.classList.toggle('on',SHK.bypass);
  const io=(id,tid,v)=>{ const e=$(id); if(e) e.checked=v; const t=$(tid); if(t) t.textContent=v?'In':'Out'; };
  io('shkEdOn','shkEdOnT',!!s.eq.on); io('shkEdCompOn','shkEdCompOnT',!!s.comp.on);
  if(!SHK.built){ SHK.built='1'; shkBandsBuild(); shkCompBuild(); }
  shkBandsSync(); shkEqDraw(); shkCompSync(); shkCompDraw();
  shkCopySync();
  shkLive();
}
// The test playing on a shaker: the burst for its second, the sweep or
// the tone while the card reports its frequency (and for a moment after
// the key, before the status shows it).
function shkTestPlaying(c){
  const k=SHK.testKind[c]||'', age=Date.now()-(SHK.testT0[c]||0);
  if(k==='burst') return age<1000?'burst':'';
  if(k==='sweep'||k==='tone') return ((SHK.testHz[c]||0)>0||age<800)?k:'';
  return '';
}
// The editor's gain reduction (its ladder, the number and the curve's
// dot), as drawn (shkMetersTick).
function shkGrDraw(){
  const ed=$('shkEd'); if(!ed||ed.hidden||SHK.sel<0||!SHK.shapes[SHK.sel]) return;
  const gr=SHK.grD[SHK.sel]||0, gf=$('shkGrFill'), gt=$('shkGrTxt');
  if(gf) gf.style.height=Math.round(Math.max(0,Math.min(1,gr/24))*100)+'%';
  if(gt) gt.textContent=gr.toFixed(1)+' dB';
  shkCompDot();
}
// Twice a second from the status: the gain reduction, the test keys and
// readout, its cursor on the curve and the compressor's dot.
function shkLive(){
  const ed=$('shkEd'); if(!ed||ed.hidden||SHK.sel<0||!SHK.shapes[SHK.sel]) return;
  const c=SHK.sel;
  shkGrDraw();
  const kind=shkTestPlaying(c), th=SHK.testHz[c]||0, hz=th>0?' · '+Math.round(th)+' Hz':'';
  document.querySelectorAll('[data-shkt]').forEach(b=>{ if(b.dataset.shkt!=='stop') b.classList.toggle('on',b.dataset.shkt===kind); });
  const now=$('shkTestNow');
  if(now){ now.textContent=kind==='burst'?'burst · 40 Hz':kind==='sweep'?'sweep'+hz:kind==='tone'?'tone'+hz+' · band '+(SHK.band+1):'idle';
    now.className='vt'+(kind?' ok':' dim'); }
  if(th>0||SHK._marked){ SHK._marked=th>0; shkEqDraw(); }
}
function shkDirtyUI(d){
  const m=$('shkEdDirty'), b=$('shkEdSave');
  if(m) m.hidden=!d.shk; if(b) b.disabled=!d.count;
}

/* EQ: the curve, the numbered band points and the five band cards. */
// One band's gain at f, dB: the controller's filter (ShakerShape.h
// bandCoeffs) evaluated on the unit circle.
function shkBandResp(b,f,fs){
  const hz=Math.max(1,Math.min(+b.hz||40,0.45*fs)), q=Math.max(0.05,+b.q||1);
  const w0=2*Math.PI*hz/fs, cw=Math.cos(w0), sw=Math.sin(w0), al=sw/(2*q), A=Math.pow(10,(+b.db||0)/40);
  let b0,b1,b2,a0,a1,a2;
  switch(b.type){
    case 'highpass': b0=(1+cw)/2; b1=-(1+cw); b2=b0; a0=1+al; a1=-2*cw; a2=1-al; break;
    case 'lowpass':  b0=(1-cw)/2; b1=1-cw;    b2=b0; a0=1+al; a1=-2*cw; a2=1-al; break;
    case 'lowshelf': { const sa=2*Math.sqrt(A)*al;
      b0=A*((A+1)-(A-1)*cw+sa); b1=2*A*((A-1)-(A+1)*cw); b2=A*((A+1)-(A-1)*cw-sa);
      a0=(A+1)+(A-1)*cw+sa; a1=-2*((A-1)+(A+1)*cw); a2=(A+1)+(A-1)*cw-sa; break; }
    case 'highshelf': { const sa=2*Math.sqrt(A)*al;
      b0=A*((A+1)+(A-1)*cw+sa); b1=-2*A*((A-1)+(A+1)*cw); b2=A*((A+1)+(A-1)*cw-sa);
      a0=(A+1)-(A-1)*cw+sa; a1=2*((A-1)-(A+1)*cw); a2=(A+1)-(A-1)*cw-sa; break; }
    default: b0=1+al*A; b1=-2*cw; b2=1-al*A; a0=1+al/A; a1=-2*cw; a2=1-al/A;
  }
  const w=2*Math.PI*f/fs, c1=Math.cos(w), s1=Math.sin(w), c2=Math.cos(2*w), s2=Math.sin(2*w);
  const nr=b0+b1*c1+b2*c2, ni=-(b1*s1+b2*s2), dr=a0+a1*c1+a2*c2, di=-(a1*s1+a2*s2);
  return 10*Math.log10((nr*nr+ni*ni)/(dr*dr+di*di));
}
const SHK_FS=48000;
function shkEqGeom(){
  const svg=$('shkEqSvg'), L=SHK.schema.limits;
  const W=Math.max(200,Math.round((svg&&svg.clientWidth)||640)), H=216, f0=L.hz[0], f1=L.hz[1], top=L.db[1], bot=L.db[0];
  // Inset so a point at a limit (10 or 300 Hz, the top or bottom gain) stays whole.
  const px=14, pt=14, pb=20, w=W-2*px, h=H-pt-pb;
  return { W, H, f0, f1, top, bot,
    X:f=>px+Math.log(f/f0)/Math.log(f1/f0)*w,
    Y:db=>pt+(top-Math.max(bot,Math.min(top,db)))/(top-bot)*h,
    fromX:x=>f0*Math.pow(f1/f0,Math.max(0,Math.min(1,(x-px)/w))),
    fromY:y=>top-Math.max(0,Math.min(1,(y-pt)/h))*(top-bot) };
}
function shkHandleY(g,b){ return shkHasGain(b.type)?g.Y(+b.db):g.Y(shkBandResp(b,+b.hz,SHK_FS)); }
function shkEqDraw(){
  const svg=$('shkEqSvg'); if(!svg||SHK.sel<0||!SHK.schema) return;
  const s=SHK.shapes[SHK.sel], g=shkEqGeom(), bands=s.eq.bands;
  svg.setAttribute('viewBox','0 0 '+g.W+' '+g.H);
  let h='';
  // Above half the loop's rate nothing reaches the shakers on this machine.
  const nyq=(+(cfgObj&&cfgObj.controlLoopHz)||0)/2;
  if(nyq>g.f0&&nyq<g.f1) h+='<rect class="shade" x="'+g.X(nyq).toFixed(1)+'" y="0" width="'+(g.W-g.X(nyq)).toFixed(1)+'" height="'+g.H+'"><title>Nothing above '+Math.round(nyq)+' Hz reaches the shakers on this machine (half its loop rate)</title></rect>';
  for(const f of [20,30,50,100,200]) h+='<line class="gv" x1="'+g.X(f).toFixed(1)+'" x2="'+g.X(f).toFixed(1)+'" y1="0" y2="'+g.H+'"/><text class="gt" x="'+g.X(f).toFixed(1)+'" y="'+(g.H-6)+'" text-anchor="middle">'+f+'</text>';
  for(let d=Math.ceil(g.bot/6)*6; d<=g.top; d+=6)
    h+='<line class="gh'+(d===0?' z':'')+'" x1="0" x2="'+g.W+'" y1="'+g.Y(d).toFixed(1)+'" y2="'+g.Y(d).toFixed(1)+'"/>'
      +(d>g.bot&&d<g.top?'<text class="gt" x="7" y="'+(g.Y(d)-4).toFixed(1)+'">'+(d>0?'+':'')+d+'</text>':'');
  h+='<text class="gt" x="'+(g.W-8)+'" y="12" text-anchor="end">Hz · dB</text>';
  const N=160, fAt=i=>g.f0*Math.pow(g.f1/g.f0,i/N);
  // The selected band's own part, filled from 0 dB.
  const sb=bands[SHK.band];
  if(sb&&sb.on){ const z=g.Y(0).toFixed(1), p=[g.X(g.f0).toFixed(1)+','+z];
    for(let i=0;i<=N;i++){ const f=fAt(i); p.push(g.X(f).toFixed(1)+','+g.Y(shkBandResp(sb,f,SHK_FS)).toFixed(1)); }
    p.push(g.X(g.f1).toFixed(1)+','+z); h+='<polygon class="bf" points="'+p.join(' ')+'"/>'; }
  const pts=[];
  for(let i=0;i<=N;i++){ const f=fAt(i); let db=0; for(const b of bands) if(b.on) db+=shkBandResp(b,f,SHK_FS); pts.push(g.X(f).toFixed(1)+','+g.Y(db).toFixed(1)); }
  h+='<polyline class="cv'+(s.eq.on?'':' off')+'" points="'+pts.join(' ')+'"/>';
  const th=SHK.testHz[SHK.sel]||0;
  if(th>=g.f0&&th<=g.f1) h+='<line class="tm" x1="'+g.X(th).toFixed(1)+'" x2="'+g.X(th).toFixed(1)+'" y1="0" y2="'+g.H+'"/>';
  bands.forEach((b,i)=>{ const x=g.X(+b.hz).toFixed(1), y=shkHandleY(g,b);
    h+='<g class="hd'+(b.on?' on':'')+(i===SHK.band?' sel':'')+'"><circle cx="'+x+'" cy="'+y.toFixed(1)+'" r="11"/><text x="'+x+'" y="'+(y+4).toFixed(1)+'">'+(i+1)+'</text></g>'; });
  svg.innerHTML=h;
}
function shkBandsBuild(){
  const host=$('shkEqBands'); if(!host) return;
  const L=SHK.schema.limits, types=SHK.schema.types.map(t=>'<option value="'+t+'">'+(SHK_TYPE_LABEL[t]||t)+'</option>').join('');
  let h='';
  for(let i=0;i<SHK.schema.bands;i++){
    const N=i+1, num=(k,st)=>'<input class="inp" type="number" data-bk="'+k+'" data-b="'+i+'" min="'+L[k][0]+'" max="'+L[k][1]+'" step="'+st+'">';
    h+='<div class="sk-b" data-b="'+i+'">'
      +'<div class="sk-bh"><button class="sk-bn" type="button" data-b="'+i+'" title="Band '+N+': select (the band tone plays it)">'+N+'</button>'
      +'<select data-bk="type" data-b="'+i+'" title="Band '+N+': its kind">'+types+'</select>'
      +'<label class="key sm latch ico" title="Band '+N+' on or off"><input type="checkbox" hidden data-bk="on" data-b="'+i+'"></label></div>'
      +'<div class="sk-bf"><span>Hz</span>'+num('hz',1)+'</div>'
      +'<div class="sk-bf"><span>dB</span>'+num('db',0.5)+'</div>'
      +'<div class="sk-bf"><span>Q</span>'+num('q',0.05)+'</div></div>';
  }
  host.innerHTML=h;
  const pick=b=>{ if(SHK.band===b) return; SHK.band=b; shkBandsSync(); shkEqDraw(); if(SHK.toneCh===SHK.sel&&(SHK.testHz[SHK.sel]||0)>0) shkToneFollow(); };
  host.querySelectorAll('[data-bk]').forEach(el=>{
    const ev=(el.type==='checkbox'||el.tagName==='SELECT')?'change':'input';
    el.addEventListener(ev,()=>{
      if(SHK.sel<0) return;
      const b=+el.dataset.b, k=el.dataset.bk, band=SHK.shapes[SHK.sel].eq.bands[b];
      if(k==='on') band.on=el.checked;
      else if(k==='type') band.type=el.value;
      else { if(el.value==='') return; const v=+el.value; if(!isFinite(v)) return; const lim=SHK.schema.limits[k]; band[k]=Math.max(lim[0],Math.min(lim[1],v)); }
      SHK.band=b; shkEdited(SHK.sel);
    });
    el.addEventListener('focus',()=>pick(+el.dataset.b));
  });
  // The card itself (not its fields) or its number: select the band.
  host.querySelectorAll('.sk-b').forEach(el=>{ el.addEventListener('click',e=>{ if(!e.target.closest('input,select,label')) pick(+el.dataset.b); }); });
}
function shkBandsSync(){
  const host=$('shkEqBands'); if(!host||SHK.sel<0) return;
  const s=SHK.shapes[SHK.sel];
  host.querySelectorAll('[data-bk]').forEach(el=>{
    const band=s.eq.bands[+el.dataset.b], k=el.dataset.bk;
    if(k==='db') el.disabled=!shkHasGain(band.type);
    if(document.activeElement===el&&el.type==='number') return;   // being typed in
    if(k==='on') el.checked=!!band.on;
    else if(k==='type') el.value=band.type;
    else el.value=(k==='hz')?(Math.round(+band.hz*10)/10):(k==='db')?(+band.db).toFixed(1):(+band.q).toFixed(2);
  });
  host.querySelectorAll('.sk-b').forEach(el=>{ const b=+el.dataset.b; el.classList.toggle('sel',b===SHK.band); el.classList.toggle('on',!!s.eq.bands[b].on); });
  host.classList.toggle('off',!s.eq.on);
}
// Drag a numbered point: frequency across, gain up and down (bells and
// shelves); the wheel over the curve changes the selected band's width;
// a double click switches a band on or off.
{ const svg=$('shkEqSvg');
  if(svg){
    const at=e=>{ const r=svg.getBoundingClientRect(), g=shkEqGeom(); return { g, x:(e.clientX-r.left)*g.W/Math.max(1,r.width), y:(e.clientY-r.top)*g.H/Math.max(1,r.height) }; };
    const nearest=p=>{ if(SHK.sel<0) return -1; let best=-1, bd=18*18;
      SHK.shapes[SHK.sel].eq.bands.forEach((b,i)=>{ const dx=p.g.X(+b.hz)-p.x, dy=shkHandleY(p.g,b)-p.y, d=dx*dx+dy*dy; if(d<bd){ bd=d; best=i; } });
      return best; };
    svg.addEventListener('pointerdown',e=>{
      if(SHK.sel<0||!SHK.schema) return;
      const b=nearest(at(e)); if(b<0) return;
      SHK.band=b; SHK.drag={b}; try{ svg.setPointerCapture(e.pointerId); }catch(_){}
      e.preventDefault(); shkBandsSync(); shkEqDraw();
    });
    svg.addEventListener('pointermove',e=>{
      if(!SHK.drag||SHK.sel<0) return;
      const p=at(e), band=SHK.shapes[SHK.sel].eq.bands[SHK.drag.b];
      band.hz=Math.round(p.g.fromX(p.x)*10)/10;
      if(shkHasGain(band.type)) band.db=Math.round(p.g.fromY(p.y)*2)/2;
      shkEdited(SHK.sel);
    });
    const end=()=>{ SHK.drag=null; };
    svg.addEventListener('pointerup',end); svg.addEventListener('pointercancel',end);
    svg.addEventListener('wheel',e=>{
      if(SHK.sel<0||!SHK.schema) return;
      const band=SHK.shapes[SHK.sel].eq.bands[SHK.band]; if(!band) return;
      e.preventDefault();
      const L=SHK.schema.limits.q;
      band.q=Math.round(Math.max(L[0],Math.min(L[1],band.q*(e.deltaY<0?1.12:1/1.12)))*100)/100;
      shkEdited(SHK.sel);
    },{passive:false});
    svg.addEventListener('dblclick',e=>{
      const b=nearest(at(e)); if(b<0) return;
      const band=SHK.shapes[SHK.sel].eq.bands[b]; band.on=!band.on; SHK.band=b; shkEdited(SHK.sel);
    });
    if(window.ResizeObserver) new ResizeObserver(()=>{ if(SHK.sel>=0) shkEqDraw(); }).observe(svg);
  }
}

/* Compressor: the curve (level in against level out, the dot where it is
   working now) and its six settings, a slider and a number each (attack,
   release and ratio on a log slider). */
const SHK_COMP=[['thresholdDb','Threshold','dB',0.5,false],['ratio','Ratio',':1',0.1,true],['kneeDb','Knee','dB',0.5,false],
                ['attackMs','Attack','ms',0.5,true],['releaseMs','Release','ms',5,true],['makeupDb','Makeup','dB',0.5,false]];
function shkCompGain(lv,thr,ratio,knee){
  const over=lv-thr, slope=1/Math.max(1,ratio)-1;
  if(knee>0&&2*Math.abs(over)<=knee){ const k=over+knee/2; return slope*k*k/(2*knee); }
  return over>0?slope*over:0;
}
function shkSlider(k,v,toSlider){
  const L=SHK.schema.limits[k], lg=SHK_COMP.find(r=>r[0]===k)[4];
  if(toSlider) return Math.round(lg?Math.log(v/L[0])/Math.log(L[1]/L[0])*1000:(v-L[0])/(L[1]-L[0])*1000);
  const t=Math.max(0,Math.min(1,v/1000));
  return lg?L[0]*Math.pow(L[1]/L[0],t):L[0]+t*(L[1]-L[0]);
}
function shkCompBuild(){
  const host=$('shkCompRows'); if(!host) return;
  const L=SHK.schema.limits;
  host.innerHTML=SHK_COMP.map(([k,lab,u,st])=>'<div class="pr"><span class="hk">'+lab+' <i>'+u+'</i></span>'
      +'<input type="range" min="0" max="1000" step="1" data-ck="'+k+'">'
      +'<input class="inp" type="number" data-cn="'+k+'" min="'+L[k][0]+'" max="'+L[k][1]+'" step="'+st+'"></div>').join('');
  const put=(k,v)=>{ if(SHK.sel<0) return; const L2=SHK.schema.limits[k], st=SHK_COMP.find(r=>r[0]===k)[3];
    const dec=st<1?String(st).split('.')[1].length:0;
    SHK.shapes[SHK.sel].comp[k]=Math.max(L2[0],Math.min(L2[1],+(Math.round(v/st)*st).toFixed(dec))); shkEdited(SHK.sel); };
  host.querySelectorAll('[data-ck]').forEach(el=>el.addEventListener('input',()=>put(el.dataset.ck,shkSlider(el.dataset.ck,+el.value,false))));
  host.querySelectorAll('[data-cn]').forEach(el=>el.addEventListener('input',()=>{ if(el.value==='') return; const v=+el.value; if(isFinite(v)) put(el.dataset.cn,v); }));
}
function shkCompSync(){
  const host=$('shkCompRows'); if(!host||SHK.sel<0) return;
  const c=SHK.shapes[SHK.sel].comp;
  host.querySelectorAll('[data-ck]').forEach(el=>{ if(document.activeElement!==el) el.value=shkSlider(el.dataset.ck,+c[el.dataset.ck],true); });
  host.querySelectorAll('[data-cn]').forEach(el=>{ if(document.activeElement!==el){ const k=el.dataset.cn, v=+c[k]; el.value=(k==='releaseMs')?Math.round(v):(Math.round(v*10)/10); } });
  host.classList.toggle('off',!c.on);
}
// The plot's scale: level -lo..0 dBFS both ways over 200.
function shkCompGeom(){
  const lo=SHK.schema.limits.thresholdDb[0], S=200;
  return { lo, S, P:db=>(db-lo)/(0-lo)*S, Q:db=>S-(Math.max(lo,Math.min(0,db))-lo)/(0-lo)*S };
}
function shkCompDraw(){
  const svg=$('shkCompSvg'); if(!svg||SHK.sel<0) return;
  const c=SHK.shapes[SHK.sel].comp, {lo,S,P,Q}=shkCompGeom();
  let h='';
  for(let d=lo; d<=0; d+=12) h+='<line class="gv" x1="'+P(d).toFixed(1)+'" x2="'+P(d).toFixed(1)+'" y1="0" y2="'+S+'"/><line class="gh" x1="0" x2="'+S+'" y1="'+Q(d).toFixed(1)+'" y2="'+Q(d).toFixed(1)+'"/>'
    +(d<0&&d>lo?'<text class="gt" x="'+P(d).toFixed(1)+'" y="'+(S-5)+'" text-anchor="middle">'+d+'</text>':'');
  h+='<line class="un" x1="0" y1="'+S+'" x2="'+S+'" y2="0"/>';
  h+='<line class="thr" x1="'+P(c.thresholdDb).toFixed(1)+'" x2="'+P(c.thresholdDb).toFixed(1)+'" y1="0" y2="'+S+'"/>';
  const pts=[]; for(let i=0;i<=100;i++){ const lv=lo+(0-lo)*i/100; pts.push(P(lv).toFixed(1)+','+Q(lv+shkCompGain(lv,c.thresholdDb,c.ratio,c.kneeDb)+c.makeupDb).toFixed(1)); }
  h+='<polyline class="cv'+(c.on?'':' off')+'" points="'+pts.join(' ')+'"/>';
  h+='<circle class="dot" id="shkCompDot" r="5" cx="0" cy="0" style="display:none"/>';
  svg.innerHTML=h;
  shkCompDot();
}
// The dot: where on the curve the compressor works now, from the gain it
// reports taking off (the level in that takes that much off). Nothing
// taken off: no dot (anywhere under the knee looks the same).
function shkCompDot(){
  const dot=$('shkCompDot'); if(!dot||SHK.sel<0||!SHK.schema) return;
  const c=SHK.shapes[SHK.sel].comp, gr=SHK.grD[SHK.sel]||0;
  if(!c.on||SHK.bypass||gr<0.05){ dot.style.display='none'; return; }
  const {P,Q}=shkCompGeom(), off=x=>-shkCompGain(x,c.thresholdDb,c.ratio,c.kneeDb);
  let a=c.thresholdDb-c.kneeDb/2, b=0, x=0;
  if(off(b)>gr){ for(let i=0;i<30;i++){ const m=(a+b)/2; if(off(m)<gr) a=m; else b=m; } x=(a+b)/2; }
  dot.setAttribute('cx',P(x).toFixed(1)); dot.setAttribute('cy',Q(x-off(x)+c.makeupDb).toFixed(1));
  dot.style.display='';
}

/* Copy: the targets are picked on the strips. */
function shkCopyEnd(){ SHK.copying=false; SHK.copy.clear(); shkCopySync(); }
function shkCopySync(){
  const row=$('shkEdCopyRow'), on=SHK.copying&&SHK.sel>=0;
  const k=$('shkEdCopy'); if(k) k.classList.toggle('on',on);
  if(row) row.hidden=!on;
  if(on){
    $('shkEdCopyFrom').textContent=SHK.sel+1;
    const to=[...SHK.copy].filter(c=>c!==SHK.sel&&c<SHK.n).sort((a,b)=>a-b), t=$('shkEdCopyTo');
    if(t){ t.textContent=to.length?to.map(c=>c+1).join(' · '):'none picked'; t.className='vt'+(to.length?'':' dim'); }
    const go=$('shkEdCopyGo'); if(go){ go.disabled=!to.length; go.classList.toggle('lit',!!to.length); }
  }
  shkStripsSync();
}
async function shkTest(kind,ch){
  const c=(ch!=null)?ch:SHK.sel; if(c<0) return;
  const body={channel:c};
  if(kind!=='burst') body.kind=kind;
  if(kind==='tone'){ body.hz=+SHK.shapes[c].eq.bands[SHK.band].hz; SHK.toneCh=c; }
  else if(kind==='sweep'||kind==='stop') SHK.toneCh=-1;
  SHK.testKind[c]=kind; SHK.testT0[c]=Date.now();
  shkLive();
  try{
    const r=await fetch(API+'/api/shakers/test',{method:'POST',body:JSON.stringify(body)});
    if(!r.ok){ let j=null; try{ j=await r.json(); }catch(_){} SHK.testKind[c]=''; shkLive(); showCmdErr((j&&j.error)||('HTTP '+r.status)); }
  }catch(_){ showCmdErr('no response from the controller'); }
}
// The band tone after its band, one request in flight.
async function shkToneFollow(){
  const st=SHK.tone; if(st.busy){ st.again=true; return; }
  st.busy=true;
  try{
    do{ st.again=false; const c=SHK.toneCh; if(c<0) break;
      await fetch(API+'/api/shakers/test',{method:'POST',body:JSON.stringify({channel:c,kind:'tone',hz:+SHK.shapes[c].eq.bands[SHK.band].hz})});
    }while(st.again);
  }catch(_){}finally{ st.busy=false; }
}
{ const on=(id,f)=>{ const e=$(id); if(e) e.onclick=f; };
  on('shkEdPrev',()=>{ if(SHK.sel>0) shkOpen(SHK.sel-1); });
  on('shkEdNext',()=>{ if(SHK.sel<SHK.n-1) shkOpen(SHK.sel+1); });
  on('shkEdClose',()=>shkClose());
  const io=(id,part)=>{ const e=$(id); if(e) e.onchange=()=>{ if(SHK.sel<0) return; SHK.shapes[SHK.sel][part].on=e.checked; shkEdited(SHK.sel); }; };
  io('shkEdOn','eq'); io('shkEdCompOn','comp');
  on('shkEdReset',()=>{ if(SHK.sel<0) return; SHK.shapes[SHK.sel].eq=shkClone(SHK.schema.default.eq); shkEdited(SHK.sel); });
  on('shkEdCompReset',()=>{ if(SHK.sel<0) return; SHK.shapes[SHK.sel].comp=shkClone(SHK.schema.default.comp); shkEdited(SHK.sel); });
  // Hold to bypass: the shaker without its EQ and compressor while held.
  const bp=$('shkEdBypass');
  if(bp){
    const set=v=>{ if(SHK.sel<0||SHK.bypass===v) return; SHK.bypass=v; shkSend(SHK.sel); shkRender(); };
    bp.addEventListener('pointerdown',e=>{ e.preventDefault(); try{ bp.setPointerCapture(e.pointerId); }catch(_){} set(true); });
    for(const ev of ['pointerup','pointercancel','lostpointercapture']) bp.addEventListener(ev,()=>set(false));
    bp.addEventListener('keydown',e=>{ if((e.key===' '||e.key==='Enter')&&!e.repeat){ e.preventDefault(); set(true); } });
    bp.addEventListener('keyup',e=>{ if(e.key===' '||e.key==='Enter') set(false); });
    bp.addEventListener('blur',()=>set(false));
  }
  on('shkEdCopy',()=>{ if(SHK.sel<0) return; if(SHK.copying) shkCopyEnd(); else { SHK.copying=true; SHK.copy.clear(); shkCopySync(); } });
  on('shkEdCopyCancel',()=>shkCopyEnd());
  on('shkEdCopyAll',()=>{ const others=Array.from({length:SHK.n},(_,c)=>c).filter(c=>c!==SHK.sel);
    if(others.every(c=>SHK.copy.has(c))) SHK.copy.clear(); else others.forEach(c=>SHK.copy.add(c)); shkCopySync(); });
  on('shkEdCopyGo',()=>{ if(SHK.sel<0) return;
    for(const c of SHK.copy) if(c!==SHK.sel&&c<SHK.n&&SHK.shapes[c]){ SHK.shapes[c]=shkClone(SHK.shapes[SHK.sel]); shkSend(c); }
    shkCopyEnd(); shkEdited(SHK.sel); });
  document.querySelectorAll('[data-shkt]').forEach(b=>{ b.onclick=()=>shkTest(b.dataset.shkt); });
  on('shkEdSave',()=>saveConfig());
  document.addEventListener('keydown',e=>{
    if(e.key!=='Escape'||SHK.sel<0||document.body.dataset.view!=='setup'||(e.target.closest&&e.target.closest('input,select'))) return;
    if(SHK.copying) shkCopyEnd(); else shkClose();
  });
}
let hapOpenDrawer=null;
// Tiles whose shaker output is a modelled signal with no single carrier
// (the order selector would do nothing there).
const HAP_NO_HARM=['kerb','surface','wheels','impacts'];
function hapDrawerRender(fx){
  const dr=$('hapDrawer'); if(!dr) return;
  if(hapOpenDrawer===fx.k){ dr.hidden=true; hapOpenDrawer=null; return; }
  hapOpenDrawer=fx.k;
  const dv=cfgObj.haptics[fx.k];
  const gain=(axis)=>{ const e=(dv.routes||[]).find(r=>r.axis===axis); return e?e.gain:0; };
  const part=(axis)=>{ const e=(dv.routes||[]).find(r=>r.axis===axis); return (e&&e.part)||'all'; };
  let h='<div class="hd-t">'+fx.label+' · routes (0 = not routed; any or all axes; torque axes x, position axes mm at full amp'
    +(fx.parts?'; part = which wheels this axis carries':'')+'; Save to apply)</div>';
  for(const a of hapRouteAxes()){
    const hint=(a.kind==='position')?' <span class="hk">up to '+hapAllowedMm(a,hapHintHz(fx,dv)).toFixed(2)+' mm @ '+hapHintHz(fx,dv)+' Hz</span>':'';
    const sel=fx.parts?' <select data-part="'+a.i+'" title="Which wheels this axis carries: the strongest of them">'
      +HAP_PARTS.map(pk=>'<option value="'+pk+'"'+(pk===part(a.i)?' selected':'')+'>'+pk.toUpperCase()+'</option>').join('')+'</select>':'';
    h+='<label>'+a.name+' <input type="number" min="0" max="'+a.max+'" step="'+a.step+'" data-axis="'+a.i+'" data-max="'+a.max+'" value="'+gain(a.i)+'"> '+a.unit+sel+hint+'</label>';
  }
  // Shaker channels (when the host has them): gain, the harmonic of the
  // effect's carrier to play (x2 puts a 9 Hz belt effect at 18 Hz on a
  // shaker, phase-locked) where the effect has one carrier, and for the
  // per-wheel effects which wheels the shaker carries (a corner per
  // shaker, an axle, or all).
  const nsh=hapShakerCount();
  const harmOk=!HAP_NO_HARM.includes(fx.k);
  for(let c=0;c<nsh;c++){
    const e=(dv.routes||[]).find(r=>r.shaker===c);
    const ePart=(e&&e.part)||'all';
    const hsel=harmOk
      ?' <select data-harm="'+c+'" title="Harmonic of the carrier this shaker plays: 1 = as is, 2 = double, ...'+(fx.k==='road'?' (the tarmac grain only: the road itself has no single carrier)':'')+'">'
        +[1,2,3,4,6,8].map(k=>'<option value="'+k+'"'+((e?(e.harm||1):1)===k?' selected':'')+'>x'+k+'</option>').join('')+'</select>'
      :' <span class="hk" title="This effect has no single carrier to multiply: it plays on a shaker as it is">as is</span>';
    const psel=fx.parts?' <select data-shpart="'+c+'" title="Which wheels this shaker carries: one corner, an axle, or all">'
      +HAP_PARTS.map(pk=>'<option value="'+pk+'"'+(pk===ePart?' selected':'')+'>'+pk.toUpperCase()+'</option>').join('')+'</select>':'';
    h+='<label>Shaker '+(c+1)+' <input type="number" min="0" max="2" step="0.1" data-shaker="'+c+'" value="'+(e?e.gain:0)+'"> x'+hsel+psel+'</label>';
  }
  h+='<span class="fldtip" id="hapRouteMsg"></span>';
  dr.className='hap-drawer'; dr.innerHTML=h; dr.hidden=false;
  // The drawer stays open across edits so "belt 1, shifter 1" is one
  // visit, not two (it used to close on the first change, dropping the
  // field being typed into). The tile's chip mirrors the routes in place.
  dr.querySelectorAll('input,select').forEach(inp=>{ inp.onchange=()=>{
    // Write to the config the page holds NOW: a reload under an open drawer
    // (a car preset applied, a profile loaded) replaces it, and edits made
    // to the old one never reached Save.
    const cur=(cfgObj&&cfgObj.haptics&&cfgObj.haptics[fx.k])||dv;
    const routes=[];
    dr.querySelectorAll('input[data-axis]').forEach(x=>{
      const g=+x.value; if(!(isFinite(g)&&g>0)) return;
      const r={axis:+x.dataset.axis,gain:Math.min(+x.dataset.max||2,g)};
      const ps=dr.querySelector('select[data-part="'+x.dataset.axis+'"]');
      if(ps&&ps.value&&ps.value!=='all') r.part=ps.value;
      routes.push(r); });
    dr.querySelectorAll('input[data-shaker]').forEach(x=>{
      const g=+x.value; if(!(isFinite(g)&&g>0)) return;
      const hs=dr.querySelector('select[data-harm="'+x.dataset.shaker+'"]');
      const r={shaker:+x.dataset.shaker,gain:Math.min(2,g),harm:hs?(+hs.value||1):1};
      const ps=dr.querySelector('select[data-shpart="'+x.dataset.shaker+'"]');
      if(ps&&ps.value&&ps.value!=='all') r.part=ps.value;
      routes.push(r); });
    cur.routes=routes.slice(0,14);
    const t=hapTiles[fx.k]; if(t){ const c=t.tile.querySelector('.hap-routechip'); if(c) c.textContent=hapChipText(cur); }
    refreshDirtyUI();
  }; });
}

/* ---- NULLCATX bindings editor (Setup). Also the round-trip fix: the web
   save used to omit ncxBindings and the server writes the posted body
   verbatim, so every web Save silently ERASED the bindings from rig.json.
   They are now first-class edited state, always sent with the save. ---- */
function ncxInit(){
  const rows=$('ncxRows'); if(!rows||!cfgObj) return;
  if(!Array.isArray(cfgObj.ncxBindings)||!cfgObj.ncxBindings.length){
    cfgObj.ncxBindings=NCX_DEFAULT.map(b=>Object.assign({},b));
    const m=$('ncxMsg'); if(m) m.textContent='Defaults loaded (they match the SimHub plugin) - Save to persist.';
  }
  const render=()=>{
    rows.innerHTML='';
    cfgObj.ncxBindings.forEach((b,idx)=>{
      const row=document.createElement('div');
      row.className='frow';
      row.innerHTML='<select data-f="token">'+NCX_TOKENS.map(t=>
          '<option'+(t===b.token?' selected':'')+'>'+t+'</option>').join('')+'</select>'
        +' slot <input type="number" min="0" max="'+(NCX_MAX_SLOTS-1)+'" style="width:52px" data-f="slot" value="'+b.slot+'">'
        +' scale <input type="number" step="0.001" style="width:70px" data-f="scale" value="'+b.scale+'">'
        +' offset <input type="number" step="0.001" style="width:70px" data-f="offset" value="'+b.offset+'">'
        +' <button class="btn btn-sm" type="button" data-del="1">✕</button>';
      row.querySelectorAll('[data-f]').forEach(el=>{ el.onchange=()=>{
        const f=el.dataset.f;
        b[f]=(f==='token')?el.value:+el.value;
        refreshDirtyUI(); }; });
      row.querySelector('[data-del]').onclick=()=>{
        cfgObj.ncxBindings.splice(idx,1); render(); refreshDirtyUI(); };
      rows.appendChild(row);
    });
  };
  render();
  const add=$('ncxAdd'); if(add) add.onclick=()=>{
    cfgObj.ncxBindings.push({token:'rpm',slot:0,scale:1,offset:0});
    render(); refreshDirtyUI(); };
  const def=$('ncxDefaults'); if(def) def.onclick=()=>{
    cfgObj.ncxBindings=NCX_DEFAULT.map(b=>Object.assign({},b));
    render(); refreshDirtyUI(); };
}

const hapTiles={};        // per-effect {svg,fx,dv,tile,level,anim}
let hapMutedNow=false;
let hapLastFired=-1;

/* Sim dots. nullCAT remembers, per game, whether each effect's channels
   have ever arrived and whether it has ever played (effectstatus.json),
   so "does this sim feed this effect?" is answered without a lap. 0 =
   never delivered (grey), 1 = delivered but never produced (amber: amp
   0, no route, or a law that never fired), 2 = has produced (green). */
let hapDotsPinned=false;   // a game other than the current one is shown
let hapGamesKnown=[];
let hapProfileLive=null;   // the active profile as the server last reported it
let hapCarLive=null;       // the car table's view of the current car as last reported
let hapCarGenLive=null;    // apply generation: a change means the tiles moved
let hapCarItems=[];        // the whole table for the search box: {key,name,game,source}
function hapSetDot(k,d){
  const t=hapTiles[k]; if(!t) return;
  const el=t.tile.querySelector('[data-sdot]'); if(!el) return;
  el.className='sdot d'+(d|0);
  el.title=d===2?'Sim: channels arrive and this effect has played':d===1
    ?'Sim: channels arrive but this effect has never played (amp 0, no route, or never triggered)'
    :'Sim: no channels seen from this game yet';
}
function hapGamesSeen(cur){
  if(cur&&!hapGamesKnown.includes(cur)){ hapGamesKnown.push(cur); hapGameSelectRender(); }
}
function hapGameSelectRender(){
  const sel=$('hapGameSel'); if(!sel) return;
  const v=sel.value;
  sel.innerHTML='<option value="">current</option>'+hapGamesKnown.map(g=>'<option value="'+g.replace(/"/g,'&quot;')+'"'+(g===v?' selected':'')+'>'+(g||'(sender unnamed)')+'</option>').join('');
}
// Profiles selector: the names from the server, the active one selected
// (or a just-saved one), with a car/game binding noted beside each name.
async function hapProfilesRender(sel,prefer){
  if(!sel) return;
  try{
    const r=await fetch(API+'/api/haptics/profiles'); if(!r.ok) return;
    const j=await r.json();
    const bound={}; for(const k in (j.bindings||{})){ const n=j.bindings[k]; (bound[n]=bound[n]||[]).push(k.replace(/^car:/,'').replace(/^game:/,'game ')); }
    const cur=prefer||j.active||'';
    sel.innerHTML='<option value="">(none)</option>'+(j.profiles||[]).map(n=>'<option value="'+n.replace(/"/g,'&quot;')+'"'+(n===cur?' selected':'')+'>'+n+(bound[n]?' · '+bound[n].join(', '):'')+'</option>').join('');
  }catch(_){}
}
// Starting points selector: the shipped types of car in their order, each
// option's hover its description; the chosen one stays chosen across a
// rebuild of the strip.
let hapPresetPick='';
async function hapPresetsRender(sel){
  if(!sel) return;
  try{
    const r=await fetch(API+'/api/haptics/presets'); if(!r.ok) return;
    const list=(await r.json()).presets||[];
    const esc=(t)=>String(t||'').replace(/&/g,'&amp;').replace(/"/g,'&quot;').replace(/</g,'&lt;');
    sel.innerHTML='<option value="">(pick a type)</option>'+list.map(p=>'<option value="'+esc(p.name)+'" title="'+esc(p.description)+'"'+(p.name===hapPresetPick?' selected':'')+'>'+esc(p.name)+'</option>').join('');
    const o=sel.options[sel.selectedIndex]; sel.title=(o&&o.title)||'';
  }catch(_){}
}
// Car table: the Master tile's car line (what the sim names, the entry
// found for it and where from) and the Engine tile's folded notes.
function hapCarText(c){
  if(!c||!c.car) return '(none)';
  if(!c.source) return c.car+' · no entry'+(c.defaults?(c.described?' · engine from the game':' · defaults in use'):'');
  return (c.name||c.car)+' · '+(c.source==='user'?'yours':'stock')+(c.near?' · near match':'')+(c.applied?'':' · not applied');
}
let hapCarLast=null;   // the car row's last answer, for the session block
function hapCarRow(c){
  hapCarLast=c||null;
  hapCarDefaultMark(!!(c&&c.defaults));
  const now=$('hapCarNow');
  if(now){
    now.textContent=hapCarText(c);
    // What the sim sent, exactly (the name and the id), and the entry it
    // matched: what a car table entry must be keyed by.
    const sent=(c&&c.car)?'\nThe sim sends: "'+c.car+'"'+(c.carId?', id "'+c.carId+'"':'')+(c.game?' ('+c.game+')':'')+(c.key?'; matched '+c.key:'; no entry matches'):'';
    now.title=(((c&&c.defaults)?'The car table has no entry for '+c.car+', so follow car put the engine, limiter, driveline, ABS and TC on their defaults (the pink settings) rather than keep the last car\'s. Pick a car like it in the preset box and apply it, or tune them and save for this car.'
      :(c&&c.near)?'No entry under this exact name: the table\'s '+(c.name||c.key)+' is the nearest (the name the sim gives starts with it). Save for this car to file your own under the sim\'s name.':'')+sent).trim();
  }
  // Forget drops yours for THIS car: a near match is another car's entry.
  const fb=$('hapCarForget'); if(fb) fb.disabled=!(c&&c.source==='user'&&!c.near);
  const pb=$('hapCarPull'); if(pb) pb.disabled=!(c&&c.source);
  const sb=$('hapCarSave'); if(sb) sb.disabled=!(c&&c.car);
  const t=hapTiles.rpmVibe; if(!t) return;
  const det=t.tile.querySelector('[data-car]'); if(!det) return;
  if(!c||!c.source){ det.hidden=true; return; }
  det.hidden=false;
  det.querySelector('[data-carsum]').textContent=(c.name||c.car)+' · '+(c.source==='user'?'yours':'stock');
  det.querySelector('[data-carnotes]').textContent=c.notes||'(no notes)';
}
async function hapCarRefresh(){
  try{ const r=await fetch(API+'/api/cars'); if(!r.ok) return; hapCarRow(await r.json()); }catch(_){}
}
// The search box's text for a picked entry: "name · game" (plus "· yours"
// for your own entries), mapped back to the key on apply.
function hapCarLabel(it){ return it.name+' · '+it.game+(it.source==='user'?' · yours':''); }
async function hapCarListLoad(){
  try{
    const r=await fetch(API+'/api/cars/list'); if(!r.ok) return;
    hapCarItems=(await r.json()).cars||[];
  }catch(_){}
}
// The search box's own suggestion list (the browser's list cut long names
// off, game and all): every word typed must appear in the car's name or
// its game, the game in a column of its own, at most HAP_SUG_MAX rows.
const HAP_SUG_MAX=60;
function hapCarMatches(text){
  // (a picked "name · game" searches as its words: the dot is not one)
  const terms=(text||'').toLowerCase().split(/\s+/).filter(t=>t&&t!=='·');
  if(!terms.length) return [];
  return hapCarItems.filter(i=>{ const h=(i.name+' '+i.game+(i.source==='user'?' yours':'')).toLowerCase();
    return terms.every(t=>h.includes(t)); });
}
function hapCarSuggest(input, box){
  const e=(s)=>String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
  let list=[], sel=-1;
  const close=()=>{ box.hidden=true; sel=-1; };
  const render=()=>{
    if(!list.length){ close(); return; }
    box.innerHTML=list.slice(0,HAP_SUG_MAX).map((it,k)=>'<div class="car-sug-i'+(k===sel?' on':'')+'" data-k="'+k+'"><span class="car-sug-n">'+e(it.name)+'</span><span class="car-sug-g">'+e(it.game)+(it.source==='user'?' · yours':'')+'</span></div>').join('')
      +(list.length>HAP_SUG_MAX?'<div class="car-sug-more">'+(list.length-HAP_SUG_MAX)+' more: type more of the name</div>':'');
    box.hidden=false;
    const on=box.querySelector('.car-sug-i.on'); if(on) on.scrollIntoView({block:'nearest'});
  };
  const take=(k)=>{ const it=list[k]; if(!it) return; input.value=hapCarLabel(it); close(); };
  input.addEventListener('input',()=>{ list=hapCarMatches(input.value); sel=-1; render(); });
  input.addEventListener('focus',()=>{ if(input.value.trim()){ list=hapCarMatches(input.value); render(); } });
  input.addEventListener('keydown',(ev)=>{
    if(box.hidden) return;
    const n=Math.min(list.length,HAP_SUG_MAX);
    if(ev.key==='ArrowDown'){ sel=(sel+1)%n; render(); ev.preventDefault(); }
    else if(ev.key==='ArrowUp'){ sel=(sel<=0?n:sel)-1; render(); ev.preventDefault(); }
    else if(ev.key==='Enter'){ if(sel>=0) take(sel); else if(list.length===1) take(0); ev.preventDefault(); }
    else if(ev.key==='Escape'){ close(); }
  });
  // mousedown, not click: it lands before the input's blur closes the list;
  // anywhere in the box (its scrollbar too) keeps the input's focus
  box.addEventListener('mousedown',(ev)=>{ ev.preventDefault(); const row=ev.target.closest('.car-sug-i'); if(row) take(+row.dataset.k); });
  input.addEventListener('blur',()=>setTimeout(close,150));
}
function hapCarPicked(text){
  const q=(text||'').trim().toLowerCase(); if(!q) return null;
  let it=hapCarItems.find(i=>hapCarLabel(i).toLowerCase()===q);
  if(!it) it=hapCarItems.find(i=>i.name.toLowerCase()===q||i.key.toLowerCase()===q);
  if(!it) it=hapCarItems.find(i=>hapCarLabel(i).toLowerCase().includes(q));
  if(!it){ const m=hapCarMatches(text); if(m.length===1) it=m[0]; }   // the words typed name one car
  return it||null;
}
async function hapGameShow(game){
  hapDotsPinned=!!game;
  if(!game) return;   // back to live dots on the next status
  try{
    const r=await fetch(API+'/api/haptics/status?game='+encodeURIComponent(game)); if(!r.ok) return;
    const j=await r.json();
    hapGamesKnown=j.games||hapGamesKnown; hapGameSelectRender();
    const sel=$('hapGameSel'); if(sel) sel.value=game;
    HAP_FX.forEach(fx=>{ const e=(j.effects||{})[fx.k]||{}; hapSetDot(fx.k,e.produced?2:(e.delivered?1:0)); });
    const gl=$('hapGameNow'); if(gl) gl.textContent=(j.game||'(sender unnamed)')+' (remembered)';
  }catch(_){}
}

// Live status hook (called from applyState): channel-health chips, mute
// state, real-time animation - continuous tiles scroll while their engine
// level is up, transient tiles flick on a fired-counter change.
// The telemetry recording's state on the Master tile.
let hapRecOn=false;
function hapRecRender(s){
  const rec=s&&s.record; if(!rec) return;
  hapRecOn=!!rec.on;
  const b=$('hapRecBtn'), st=$('hapRecState'); if(!b||!st) return;
  b.textContent=rec.on?'stop':'record';
  b.classList.toggle('btn-action',!!rec.on);
  const mb=(+rec.bytes||0)/1048576, t=Math.round(+rec.sec||0);
  st.textContent=rec.on
    ?('REC '+Math.floor(t/60)+':'+String(t%60).padStart(2,'0')+', '+mb.toFixed(1)+' MB, '+(+rec.packets||0)+' packets'+((+rec.dropped||0)>0?(', '+rec.dropped+' dropped'):''))
    :(rec.note?rec.note:(rec.file?('last: '+rec.file+' ('+mb.toFixed(1)+' MB)'):'off'));
}
function hapLive(s){
  shakerStatus(s);
  hapRecRender(s);
  const panel=$('hapPanel'); if(!panel||panel.hidden) return;
  // Direct telemetry: the rig's choice, else what the sender does; the
  // note says what the sender reports (or that it is being asked), and
  // when another sender's channels are set aside for it. A sender that is
  // only direct (a game's own) cannot switch and is never asked.
  { const cb=$('hapDirect'), st=$('hapDirectState'); if(cb&&st){
      const said=s.directSender||'', want=s.directWant||'';
      const aside=s.directSetAside?', another sender set aside':'';
      cb.disabled = said==='only';   // nothing to switch: a tick here would only carry to the next game
      if(document.activeElement!==cb) cb.checked = said==='only' ? true : want ? want==='on' : said==='on';
      // A game paused in a menu sends nothing while another sender's lines
      // are set aside for it (up to five minutes): say so, not "no sender".
      st.textContent = !s.ncxRx ? (s.directSetAside ? '(waiting for the game, another sender set aside)' : '(no sender)')
        : said==='' ? '(the sender does not offer it)'
        : said==='none' ? '(none in this game)'
        : said==='only' ? '(always on with this sender'+aside+')'
        : (want && (want==='on')!==(said==='on')) ? '(asking the sender)'
        : said==='on' ? '(on'+aside+')'
        : '(off)'; } }
  hapMutedNow=!!s.hapMuted;
  const mb=$('hapMuteBtn'), mst=$('hapMuteState');
  if(mb) mb.textContent=hapMutedNow?'UNMUTE':'MUTE';
  if(mst) mst.textContent=hapMutedNow?'MUTED':'';
  const fired=+s.hapFired||0;
  const firedEdge=(hapLastFired>=0&&fired>hapLastFired);
  hapLastFired=fired;
  // Profile switched on the server (a car binding kicked in): refresh the
  // page's copy of the config unless the user has unsaved edits.
  if(typeof s.hapProfile==='string'&&s.hapProfile!==hapProfileLive){
    const first=(hapProfileLive===null); hapProfileLive=s.hapProfile;
    if(!first){ const sel=$('hapProfSel'); if(sel) hapProfilesRender(sel); if(dirtyState().count===0) loadConfig(); }
  }
  // Car table: a new apply generation means the haptics were rewritten
  // (follow car, a pull, a starting point or a profile, from this page or
  // another, or another page's save; a restart starts a new count too); a
  // new match means the car line changed. A clean
  // page reloads its copy; a page with unsaved edits keeps them and is
  // told: its Save will be refused (409) and reload then.
  if(typeof s.hapCarGen==='number'&&s.hapCarGen!==hapCarGenLive){
    const first=(hapCarGenLive===null); hapCarGenLive=s.hapCarGen;
    if(!first){
      if(dirtyState().count===0) loadConfig();
      else if(typeof cfgGenLoaded==='number'&&s.hapCarGen!==cfgGenLoaded){
        const hm=$('hapMsg'); if(hm) hm.textContent='The haptics settings changed on the controller (car table, starting point, profile, a save from another page, or a restart) while you have unsaved edits: Save will reload them and ask you to make your edits again.';
      }
    }
  }
  if(typeof s.hapCarKey==='string'){
    const sig=(s.hapCar||'')+'|'+s.hapCarKey+'|'+(s.hapCarSrc||'')+'|'+(s.hapCarApplied?1:0)+'|'+(s.hapCarDefault?1:0);
    if(sig!==hapCarLive){ hapCarLive=sig; hapCarRefresh(); }
  }
  // Sim dots: the remembered record for the game shown (the current one
  // unless the Master tile's selector picked another, then it is fetched).
  if(Array.isArray(s.hapDots)&&!hapDotsPinned){
    HAP_FX.forEach((fx,i)=>hapSetDot(fx.k,s.hapDots[i]|0));
    const gl=$('hapGameNow'); if(gl) gl.textContent=s.hapGame||'(sender unnamed)';
    hapGamesSeen(s.hapGame);
  }
  for(const k in hapTiles){
    const t=hapTiles[k];
    // The tile's mute (the master's, or its own).
    if(Array.isArray(s.hapMute)&&t.fx.id!==undefined){
      const m=!!s.hapMute[t.fx.id];
      t.tile.classList.toggle('muted',m);
      const mb=t.tile.querySelector('[data-mute]'); if(mb) mb.classList.toggle('on',m);
    }
    // Channel-health chip: token names with a live tick/cross.
    const ch=t.tile.querySelector('[data-chan]');
    if(ch&&t.fx.chan&&Array.isArray(s.ncxHave)){
      let allOk=true, wheelLive=false, magLive=false;
      const vals=Array.isArray(s.ncxVal)?s.ncxVal:[], pks=Array.isArray(s.ncxPk)?s.ncxPk:[];
      ch.textContent=t.fx.chan.map(entry=>{
        const sp=hapChanSpec(entry); const ok=sp.ok(s.ncxHave);
        if(!sp.optional) allOk=allOk&&ok;
        // Which path feeds a two-path entry (per-wheel group or magnitude).
        // The combined-slip group is per wheel but scaled by peak %, so it
        // counts as the magnitude path for the trims.
        if(sp.alts.length>1){ const a=sp.live(s.ncxHave); if(a){ if(a.endsWith('*')&&a!=='wheelSlip*') wheelLive=true; else magLive=true; } }
        return sp.text(s.ncxHave,vals,pks)+(ok?' ✓':(sp.optional?' ·':' ✗'));
      }).join('  ');
      ch.classList.toggle('bad',!allOk);
      // Only the trim that acts on the live path reads as live.
      const special=hapDimFor(k,t.dv,s.ncxHave,!!s.curbsNamed);
      // A dimmed setting says why first, then what it does.
      const titled=(row,why)=>{ const help=row.dataset.help||''; row.title=why?(why+(help?'\n\n'+help:'')):help; };
      t.tile.querySelectorAll('[data-pk]').forEach(row=>{
        const key=row.dataset.pk;
        if(special){ row.classList.toggle('dim',!!special[key]); titled(row,special[key]||''); return; }
        let dim=(wheelLive&&HAP_TRIM_MAG.includes(key))||(magLive&&!wheelLive&&HAP_TRIM_WHEEL.includes(key));
        let why=dim?(wheelLive?'Per-wheel data is live: this trim applies to the single-channel fallback only':'The single channel is live: this trim applies to per-wheel data only'):'';
        // The Lateral baseline: only on the slip-speed path; base deg only when fixed.
        if(k==='slipLat'&&(key==='baseMode'||key==='baseDegF'||key==='baseDegR')){
          if(+s.hapLatSrc!==3){ dim=true; why='Only for a sim that sends a slip speed rather than a slip angle (Automobilista 2, Le Mans Ultimate)'; }
          else if(key!=='baseMode'&&Math.round(+t.dv.baseMode)!==2){ dim=true; why='Used with baseline fixed only'; }
        }
        row.classList.toggle('dim',dim);
        titled(row,why);
      });
    }
    // Per-wheel readout: now/peak per corner from the matching status set.
    const wl=t.tile.querySelector('[data-wheels]');
    if(wl&&Array.isArray(s.hapWheels)&&Array.isArray(s.hapWheelsPk)){
      const k=+wl.dataset.wheels, now=s.hapWheels[k]||[], pk=s.hapWheelsPk[k]||[];
      const f=(v)=>(k===2?(+v||0).toFixed(1):(+v||0).toFixed(2));
      // Four cells in a 2 x 2 grid (FL FR / RL RR): one line was wider
      // than the tile and ran under its neighbour.
      const cells=wl.children, ground=s.hapWheels[6]||[];
      // Road: a tyre in the air shows "air" instead of its travel.
      ['FL','FR','RL','RR'].forEach((n,i)=>{ if(cells[i]) cells[i].textContent=n+' '+((k===2&&ground[i]!==undefined&&+ground[i]<0.5)?'air':f(now[i])+'/'+f(pk[i])); });
      wl.classList.toggle('hot',pk.some(v=>+v>0.05));
    }
    // Lateral: each wheel's slip angle as played and its straight-line
    // baseline, with how long the baseline has learned.
    const al=t.tile.querySelector('[data-ang]');
    if(al&&Array.isArray(s.hapWheels)&&s.hapWheels.length>13){
      const ang=s.hapWheels[12]||[], base=s.hapWheels[13]||[], cells=al.children;
      ['FL','FR','RL','RR'].forEach((n,i)=>{ if(cells[i]) cells[i].textContent=n+' '+(+ang[i]||0).toFixed(1)+'°/'+(+base[i]||0).toFixed(1); });
      const as=t.tile.querySelector('[data-angstate]'), sec=+s.hapLatBaseSec||0, src=+s.hapLatSrc||0, learned=!!s.hapLatLearned, mode=+s.hapLatMode||0;
      const learning=(mode!==2&&!learned);
      if(as){ as.textContent=src===3?('from slip speed: baseline '+(mode===2?'fixed'
            :(learned?(mode===1?'learned, held':'learned'):'learning')+' ('+sec.toFixed(0)+' s of straights)'))
          :src===1?'the sim\'s own slip angle (no baseline needed)':src===2?'combined slip (no angle from this sim)':'angle -';
        as.classList.toggle('warn',src===3&&learning); }
    }
    // Road: full mm learned for this car, on the learn row (whether or not
    // it is ticked, so it can be compared with yours); ticked, the full mm
    // field is greyed and cannot be edited.
    if(k==='road'){
      const lr=t.tile.querySelector('[data-pk="fullLearn"] .hk'), fr=t.tile.querySelector('[data-pk="fullMm"] input');
      const on=Math.round(+t.dv.fullLearn)===1&&(Math.round(+t.dv.model)||0)===0;
      const f=s.hapRoadFull||{}, mm=+f.mm||0, sec=+f.sec||0;
      if(lr){ lr.textContent='learn full mm'+(mm>0?' ('+mm.toFixed(1)+')':sec>0?' (learning '+sec.toFixed(0)+' s)':'');
              lr.classList.toggle('warn',on&&!(mm>0)); }
      if(fr) fr.disabled=on;
    }
    // ABS: what says it is working (the flag, the wheels and their rate).
    const ab=t.tile.querySelector('[data-abssrc]');
    if(ab&&s.hapAbs){
      const fw=Math.round(+t.dv.fromWheels||0), a=s.hapAbs;
      const why=fw===0?' (from wheels off)':(fw===1&&a.seen===false)?' (from wheels: waiting for the sim\'s flag)':'';
      ab.textContent=!(+t.dv.ampPct>0)?'off (amp 0: no ABS, through the tyres either)':(a.flag||a.wheels)?('working: '+[a.flag?'flag':'',a.wheels?('wheels '+(+a.hz||0).toFixed(0)+' Hz'):''].filter(Boolean).join(' + ')):('idle'+why);
      ab.classList.toggle('warn',!!a.wheels);
    }
    // Wheels: the tyres' state at a glance (flat spots, hot discs, a
    // puncture), amber or red while anything is there.
    const ws=t.tile.querySelector('[data-wstate]');
    if(ws&&Array.isArray(s.hapWheels)&&s.hapWheels.length>11){
      const n=['FL','FR','RL','RR'], flat=s.hapWheels[4]||[], heat=s.hapWheels[10]||[], pun=s.hapWheels[11]||[];
      const parts=[]; let lvl=0;
      const flats=n.map((w,i)=>(+flat[i]>0.05)?w+' '+(+flat[i]).toFixed(2):'').filter(x=>x);
      if(flats.length){ parts.push('flat '+flats.join(', ')+' mm'); lvl=1; }
      const hot=n.filter((w,i)=>+heat[i]>0.3);
      if(hot.length){ parts.push('hot discs '+hot.join(' ')); lvl=1; }
      const punct=n.filter((w,i)=>+pun[i]>0.5);
      if(punct.length){ parts.push('puncture '+punct.join(' ')); lvl=2; }
      const on=+t.dv.ampPct>0;
      ws.textContent=on?(parts.length?parts.join(' | '):'tyres OK'):'off (fresh tyres when on)';
      ws.classList.toggle('warn',on&&lvl===1); ws.classList.toggle('bad',on&&lvl===2);
    }
    // Surface: what each tyre is on as nullCAT reads it (class, wet,
    // floating, or in the air).
    const sl=t.tile.querySelector('[data-surf]');
    if(sl&&Array.isArray(s.hapWheels)&&s.hapWheels.length>9){
      const g=s.hapWheels[6]||[], c=s.hapWheels[7]||[], wa=s.hapWheels[8]||[], fl=s.hapWheels[9]||[];
      const cells=sl.children;
      // What the game says, else the ground the tile assumes ("no surface
      // sent"), marked so: gravel there plays all the time, tarmac plays
      // nothing (the Road tile carries tarmac).
      const tokOn=(nm)=>{ const i=NCX_TOKENS.indexOf(nm); return i>=0&&Array.isArray(s.ncxHave)&&!!s.ncxHave[i]; };
      const corners=['FL','FR','RL','RR'];
      const surfIn=corners.every(n=>tokOn('surface'+n));
      const wetIn=tokOn('rain')||tokOn('wet')||corners.every(n=>tokOn('wet'+n));
      const assumed=['tarmac','gravel','dirt','snow','sand'][Math.max(0,Math.min(4,Math.round(+t.dv.assume||0)))];
      corners.forEach((n,i)=>{ if(!cells[i]) return;
        const what=surfIn?(HAP_SURF_NAMES[Math.round(+c[i])]||'tarmac'):(assumed+'*');
        const water=(+fl[i]>0.5)?'float':(+wa[i]>0.3)?'wet':'';
        const txt=(+g[i]<0.5)?'air':((what+' '+water).trim()||'-');
        cells[i].textContent=n+' '+txt; });
      sl.title=surfIn
        ?'What each tyre is on, as nullCAT reads it: the surface, wet or floating, or in the air'
        :(assumed==='tarmac'
          ?'* This game sends no surface type: the tile takes it as tarmac ("no surface sent"), which it has nothing to play on (the Road tile carries tarmac)'+(wetIn?'; puddles still play on a wet road':'')
          :'* This game sends no surface type: the tile takes it as '+assumed+' ("no surface sent") and plays it all the time');
    }
    if(t.fx.transient){
      if(firedEdge&&t.dv.ampPct>0&&!t.anim){
        t.anim=true;
        hapAnimate(t.svg,t.fx,t.dv,Math.max(300,(+t.dv.durMs||18)*4));
        setTimeout(()=>{ t.anim=false; },Math.max(300,(+t.dv.durMs||18)*4)+50);
      }
    }else if(t.fx.fxIdx!==undefined&&Array.isArray(s.hapFx)){
      t.level=+s.hapFx[t.fx.fxIdx]||0;
      // Driven level (before amp/route gating) with its session peak: the
      // input readout for effects without a per-wheel line.
      if(Array.isArray(s.hapIn)){
        const inNow=+s.hapIn[t.fx.fxIdx]||0, inPk=+(s.hapInPk||[])[t.fx.fxIdx]||0;
        t.inText='in '+inNow.toFixed(2)+'/'+inPk.toFixed(2);
      }
      if(t.level>0.05&&!t.anim){
        t.anim=true;
        const t0=performance.now();
        const frame=(now)=>{
          if(t.level<=0.05||!document.body.contains(t.svg)){
            t.anim=false; hapWave(t.svg,t.fx,t.dv); hapWaveTag(t.svg,''); return; }
          hapWave(t.svg,t.fx,t.dv,(now-t0)/1000,t.level);   // live level scales height
          hapWaveTag(t.svg,t.testUntil>now?'test':'live '+t.level.toFixed(2));
          requestAnimationFrame(frame);
        };
        requestAnimationFrame(frame);
      }
      const it=t.tile.querySelector('[data-in]'); if(it&&t.inText) it.textContent=t.inText;
    }
  }
  hapStatesUpdate(s);
}

function hapInit(){
  const panel=$('hapPanel'), strip=$('hapStrip');
  if(!panel||!strip||!cfgObj) return;
  // Experimental gate (the same tickbox as the device section) + needs a
  // torque-capable axis to route anything to.
  const cb=$('cf-showdev');
  if(!(cb&&cb.checked)||(!hapRouteAxes().length&&!hapShakerCount())){ panel.hidden=true; hapAvail(false); return; }
  panel.hidden=false; hapAvail(true);
  cfgObj.haptics=cfgObj.haptics||{};
  // Fill defaults IN PLACE: the route drawer and the tiles hold references
  // to these objects, so re-rendering must never swap them for copies.
  for(const k in HAP_DEF){
    const cur=cfgObj.haptics[k]||(cfgObj.haptics[k]={});
    for(const p in HAP_DEF[k]) if(cur[p]===undefined)
      cur[p]=Array.isArray(HAP_DEF[k][p])?HAP_DEF[k][p].slice():HAP_DEF[k][p];
  }
  // Routes are an axis index or a shaker channel; drop anything else (early
  // 0.9.6 configs carried a since-removed "own axis" -2 default - the
  // server does the same on read). Shaker routes have no axis: filtering on
  // the axis alone dropped them on every reload, and the next Save wrote
  // the haptics back without them.
  for(const k in cfgObj.haptics)
    if(Array.isArray(cfgObj.haptics[k].routes))
      cfgObj.haptics[k].routes=cfgObj.haptics[k].routes.filter(r=>r&&(r.axis>=0||r.shaker>=0));
  if(typeof cfgObj.haptics.masterGain!=='number') cfgObj.haptics.masterGain=1;
  if(typeof cfgObj.haptics.positionBudget!=='number') cfgObj.haptics.positionBudget=0.4;
  if(typeof cfgObj.haptics.axisDelayMs!=='number') cfgObj.haptics.axisDelayMs=0;
  if(typeof cfgObj.haptics.followCar!=='boolean') cfgObj.haptics.followCar=true;
  const sb=$('hapSave'); if(sb&&!sb._wired){ sb._wired=true; sb.onclick=saveConfig; }
  strip.innerHTML='';
  for(const k in hapTiles) delete hapTiles[k];

  // Master trim card: overall gain (rig config, saved) + runtime MUTE.
  {
    const mc=document.createElement('div');
    mc.className='hap-tile';
    mc.innerHTML='<div class="dh"><span class="dot"></span><span class="nm">Master</span></div>'
      +'<div class="hk hchan"></div>'
      +'<svg class="hap-wave"></svg><div class="hrows">'
      +'<div class="hsec" title="The rig: these apply to every effect and every car.">rig</div>'
      +'<div class="hr"><span class="hk">gain x</span>'
      +'<input type="number" min="0" max="2" step="0.05" id="hapMasterGain" value="'+cfgObj.haptics.masterGain+'"></div>'
      +'<div class="hr" title="Share of each position axis\'s velocity and acceleration limits that haptics may use (0 to 1). The motion cue always keeps priority."><span class="hk">pos budget</span>'
      +'<input type="number" min="0" max="1" step="0.05" id="hapPosBudget" value="'+(isFinite(+cfgObj.haptics.positionBudget)?+cfgObj.haptics.positionBudget:0.4)+'"></div>'
      +'<div class="hr" title="Hold the axis effects back by this many ms so they land together with the shakers (a sound card is 15 to 25 ms behind a belt). 0 = off."><span class="hk">axis delay ms</span>'
      +'<input type="number" min="0" max="60" step="1" id="hapAxisDelay" value="'+(isFinite(+cfgObj.haptics.axisDelayMs)?+cfgObj.haptics.axisDelayMs:0)+'"></div>'
      +'<div class="hsec" title="A profile is a named copy of EVERYTHING on this strip: every tile, every route, master gain, pos budget. Save as keeps the saved strip under any name; load puts one back; use for this car loads it by itself whenever that car runs (it wins over the car table).">profiles: the whole strip</div>'
      +'<div class="hr hap-game" title="Profiles: named copies of everything on this strip (settings, routes, the engine). Load copies one over the live set and applies it; Save as snapshots the SAVED set under a name; Use for this car binds the car the sim names now to the shown profile, and nullCAT then loads it by itself whenever that car runs."><span class="hk">profile</span>'
      +'<select id="hapProfSel"><option value="">(none)</option></select>'
      +'<button class="btn btn-sm" type="button" id="hapProfLoad" title="Copy the chosen profile over the live settings and apply it">load</button>'
      +'<button class="btn btn-sm" type="button" id="hapProfSave" title="Snapshot the saved settings under a name">save as</button>'
      +'<button class="btn btn-sm" type="button" id="hapProfBind" title="Load this profile automatically whenever the sim names the current car (or game)">use for this car</button>'
      +'<button class="btn btn-sm" type="button" id="hapProfDel" title="Forget the chosen profile and its bindings">delete</button>'
      +'<span class="hk" id="hapProfMsg"></span></div>'
      +'<div class="hsec" title="Pick the type of car closest to what you drive and apply it: it sets how the tyres let go, how stiff the road and kerbs feel, the wheels and the knocks for that type. Then pull the car (car table) for the engine, gearbox, ABS and TC. Your routes and master stay as they are.">starting point: the type of car</div>'
      +'<div class="hr hap-game" title="Pick the type of car closest to what you drive, apply, then pull the car (car table) for the engine, gearbox, ABS and TC. It sets the slip, road, kerb, wheels and impacts settings for that type, and the belt g at max (over each belt\'s own); your routes, master gain, pos budget and the settings the car table owns stay as they are."><span class="hk">type</span>'
      +'<select id="hapPresetSel"><option value="">(pick a type)</option></select>'
      +'<button class="btn btn-sm" type="button" id="hapPresetApply" title="Set the slip, road, kerb, wheels and impacts settings and the belt g at max for the chosen type of car. Routes, master gain, pos budget and the engine, limiter, driveline, ABS and TC stay as they are.">apply</button>'
      +'<span class="hk" id="hapPresetMsg"></span></div>'
      +'<div class="hr typekey" data-pk="beltFullG" title="The g at which a belt pulling from the car\'s g reaches its torque max, for this type of car: about 1 for a road car, 2 for a GT3. The type of car sets it; change it here and save, and it applies straight away. 0 = each belt\'s own g at max (Setup, the belt\'s axis)."><span class="hk">belt g at max</span>'
      +'<input type="number" min="0" max="6" step="0.05" id="hapBeltFullG" value="'+(isFinite(+cfgObj.haptics.beltFullG)?+cfgObj.haptics.beltFullG:0)+'"></div>'
      +'<div class="hsec" title="The car table: only the engine, limiter, driveline, ABS and TC tiles, per car. Routes, master gain, pos budget and the other tiles stay as you saved them.">car table: engine, limiter, driveline, ABS, TC</div>'
      +'<div class="hr hap-game" title="The car table: how each car\'s engine, limiter, driveline, ABS and TC feel, shipped for the Assetto Corsa and Automobilista 2 rosters plus your own entries. Applying an entry moves those five tiles only; routes, master gain and every other tile stay yours."><span class="hk">this car</span>'
      +'<span class="hk" id="hapCarNow">(none)</span>'
      +'<button class="btn btn-sm" type="button" id="hapCarPull" title="Apply this car\'s entry again to the engine, limiter, driveline, ABS and TC tiles. With follow car on there is no need: it is applied by itself when the sim names the car.">re-apply</button>'
      +'<button class="btn btn-sm" type="button" id="hapCarSave" title="File the SAVED engine, limiter, driveline, ABS and TC settings under this car as your own entry; it then wins over the shipped one. Save your edits first.">save as mine</button>'
      +'<button class="btn btn-sm" type="button" id="hapCarForget" title="Drop your own entry for this car; the shipped one (if any) is used again">forget mine</button>'
      +'<span class="hk" id="hapCarMsg"></span></div>'
      +'<div class="hr hap-game" title="Take the engine, limiter, driveline, ABS and TC settings from the car table whenever the sim names a car it knows. Routes, master gain and the other tiles never move. Off: the table only acts when you press re-apply or apply.">'
      +'<label class="hk"><input type="checkbox" id="hapFollowCar"'+(cfgObj.haptics.followCar!==false?' checked':'')+'> follow car</label></div>'
      +'<div class="hr hap-game" title="Ask the sim\'s sender for its direct telemetry where it has one: a faster feed straight from the game (Assetto Corsa\'s physics at about 333 a second instead of about 60). Takes effect at once, is remembered on the controller, and the sender is asked again whenever it reports otherwise. A sender that does not offer it is never asked.">'
      +'<label class="hk"><input type="checkbox" id="hapDirect"> direct telemetry</label><span class="hk" id="hapDirectState"></span></div>'
      +'<div class="hr hap-game" title="Another car\'s entry: for a car the table does not know (pick the nearest: another GT3, the same engine in a different body), or to try another car\'s numbers. Apply it, tune to taste, then save as mine."><span class="hk">another car</span>'
      +'<span class="car-pick"><input type="text" id="hapCarPick" placeholder="search cars" autocomplete="off"><div id="hapCarSug" class="car-sug" hidden></div></span>'
      +'<button class="btn btn-sm" type="button" id="hapCarApply" title="Apply the picked entry to the engine, limiter, driveline, ABS and TC tiles">apply</button></div>'
      +'<div class="hsec" title="The coloured dot on each tile: grey = that game has never sent the channels, amber = channels arrive but the effect has never played, green = has played. Only a display; nothing here is saved or changes a setting.">sim dots</div>'
      +'<div class="hr hap-game" title="Which game\'s record the tile dots show: the one running now, or a remembered one. Forget clears the record for the game shown."><span class="hk">game</span>'
      +'<span class="hk" id="hapGameNow">(none)</span><select id="hapGameSel"><option value="">current</option></select>'
      +'<button class="btn btn-sm" type="button" id="hapGameForget" title="Forget the remembered dots for the game shown">forget</button></div>'
      +'<div class="hsec" title="Every telemetry packet as it arrives, to a file on the controller (Pi: ~/nullcat-recordings), so a session can be replayed exactly for tuning. Written by a thread of its own: the control loop only hands it a copy.">recording</div>'
      +'<div class="hr hap-game"><button class="btn btn-sm" type="button" id="hapRecBtn" title="Start or stop recording every telemetry packet to a new file on the controller (Pi: ~/nullcat-recordings). Needs 1 GB free; stops by itself at 2 GB or to leave 512 MB free.">record</button>'
      +'<span class="hk" id="hapRecState" title="While recording: time, size, packets written, and any the writer had to leave out (dropped). Stopped: the last file, or why it stopped.">off</span></div>'
      +'</div><div class="hb"><span class="hk" id="hapMuteState"></span>'
      +'<button class="btn btn-sm btn-action" type="button" id="hapMuteBtn">MUTE</button></div>';
    mc.querySelector('#hapRecBtn').onclick=async()=>{
      const on=!hapRecOn;
      try{
        const r=await fetch(API+'/api/record',{method:'POST',body:JSON.stringify({on})});
        const j=await r.json().catch(()=>({}));
        if(!r.ok||j.ok===false){ const st=$('hapRecState'); if(st) st.textContent=j.error||('HTTP '+r.status); }
      }catch(_){}
    };
    mc.querySelector('#hapGameSel').onchange=(ev)=>hapGameShow(ev.target.value);
    mc.querySelector('#hapGameForget').onclick=async()=>{
      const sel=mc.querySelector('#hapGameSel');
      const game=sel.value||(($('hapGameNow')||{}).textContent||'').replace(' (remembered)','').replace('(sender unnamed)','');
      try{ await fetch(API+'/api/haptics/status/clear',{method:'POST',body:JSON.stringify({game:game})}); }catch(_){}
      hapGamesKnown=hapGamesKnown.filter(g=>g!==game); sel.value=''; hapGameSelectRender(); hapDotsPinned=false;
    };
    // Seed the game list from the remembered record.
    fetch(API+'/api/haptics/status').then(r=>r.ok?r.json():null).then(j=>{ if(j){ hapGamesKnown=j.games||[]; hapGameSelectRender(); } }).catch(()=>{});
    // Profiles row.
    const psel=mc.querySelector('#hapProfSel'), pmsg=mc.querySelector('#hapProfMsg');
    const say=(t)=>{ if(pmsg) pmsg.textContent=t; };
    const post=async(path,body)=>{ const r=await fetch(API+path,{method:'POST',body:JSON.stringify(body||{})}); const j=await r.json().catch(()=>({})); if(!r.ok||j.ok===false) throw new Error(j.error||('HTTP '+r.status)); return j; };
    hapProfilesRender(psel);
    mc.querySelector('#hapProfLoad').onclick=async()=>{
      const name=psel.value; if(!name){ say('pick a profile'); return; }
      if(dirtyState().count>0&&!confirm('You have unsaved edits on this page; loading a profile replaces the saved haptics settings and reloads the page state. Continue?')) return;
      try{ await post('/api/haptics/profiles/load',{name}); say('loaded '+name); await loadConfig(); }catch(e){ say(String(e.message||e)); }
    };
    mc.querySelector('#hapProfSave').onclick=async()=>{
      const name=prompt('Profile name (snapshots the SAVED haptics settings; Save the strip first if you have edits):', psel.value||'');
      if(!name) return;
      try{ await post('/api/haptics/profiles/save',{name:name.trim()}); say('saved '+name.trim()); await hapProfilesRender(psel,name.trim()); }catch(e){ say(String(e.message||e)); }
    };
    mc.querySelector('#hapProfBind').onclick=async()=>{
      const name=psel.value; if(!name){ say('pick a profile'); return; }
      try{ const j=await post('/api/haptics/profiles/bind',{name}); say('bound '+(j.key||'')+' to '+name); }catch(e){ say(String(e.message||e)); }
    };
    mc.querySelector('#hapProfDel').onclick=async()=>{
      const name=psel.value; if(!name){ say('pick a profile'); return; }
      if(!confirm('Forget profile "'+name+'" and its car/game bindings?')) return;
      try{ await post('/api/haptics/profiles/delete',{name}); say('deleted '+name); await hapProfilesRender(psel); }catch(e){ say(String(e.message||e)); }
    };
    // Starting points: the shipped types of car, each one's description as
    // its hover. Applying rewrites a few tiles' settings on the server and
    // the page reloads its config copy (unsaved edits would be lost: ask).
    const tsel=mc.querySelector('#hapPresetSel'), tmsg=mc.querySelector('#hapPresetMsg');
    const tsay=(t)=>{ if(tmsg) tmsg.textContent=t; };
    hapPresetsRender(tsel);
    // (A select's options show no hover in most browsers: the chosen
    // type's description is written beside it.)
    tsel.onchange=()=>{ hapPresetPick=tsel.value; const o=tsel.options[tsel.selectedIndex]; tsel.title=(o&&o.title)||''; tsay(tsel.title); };
    mc.querySelector('#hapPresetApply').onclick=async()=>{
      const name=tsel.value; if(!name){ tsay('pick a type'); return; }
      if(dirtyState().count>0&&!confirm('You have unsaved edits on this page; applying a starting point rewrites the saved slip, road, kerb, wheels and impacts settings and the belt g at max and reloads the page state. Continue?')) return;
      try{
        const j=await post('/api/haptics/presets/apply',{name});
        if(typeof j.gen==='number') hapCarGenLive=j.gen;   // this page reloads itself: the status need not again
        await loadConfig();
        const m=$('hapPresetMsg'); if(m) m.textContent='applied '+name+' (the engine, gearbox, ABS and TC come from the car table)';   // the rebuilt tile's
      }catch(e){ tsay(String(e.message||e)); }
    };
    // Car table row + search box. Applying an entry rewrites five tiles on
    // the server and the page reloads its config copy (unsaved edits on
    // the strip would be lost, so ask first).
    const cmsg=mc.querySelector('#hapCarMsg');
    const csay=(t)=>{ if(cmsg) cmsg.textContent=t; };
    const applyEntry=async(key)=>{
      if(dirtyState().count>0&&!confirm('You have unsaved edits on this page; applying a car preset rewrites the saved engine, limiter, driveline, ABS and TC settings and reloads the page state. Continue?')) return;
      try{ const j=await post('/api/cars/pull',key?{key}:{}); csay('applied '+(j.name||j.key||'')); await loadConfig(); await hapCarRefresh(); }
      catch(e){ csay(String(e.message||e)); }
    };
    hapCarSuggest(mc.querySelector('#hapCarPick'), mc.querySelector('#hapCarSug'));
    mc.querySelector('#hapCarPull').onclick=()=>applyEntry('');
    mc.querySelector('#hapCarApply').onclick=()=>{
      const it=hapCarPicked(mc.querySelector('#hapCarPick').value);
      if(!it){ csay('no such car in the table'); return; }
      applyEntry(it.key);
    };
    mc.querySelector('#hapCarSave').onclick=async()=>{
      if(dirtyState().count>0){ csay('save the strip first'); return; }
      try{ const j=await post('/api/cars/save',{}); csay('saved as '+(j.key||'yours')); await hapCarListLoad(); await hapCarRefresh(); }
      catch(e){ csay(String(e.message||e)); }
    };
    mc.querySelector('#hapCarForget').onclick=async()=>{
      if(!confirm('Drop your own entry for this car?')) return;
      try{ await post('/api/cars/forget',{}); csay('forgotten'); await hapCarListLoad(); await hapCarRefresh(); }
      catch(e){ csay(String(e.message||e)); }
    };
    const dc=mc.querySelector('#hapDirect');
    if(dc) dc.onchange=async()=>{
      try{ const j=await post('/api/telemetry/direct',{on:!!dc.checked}); if(j&&j.warning) csay(j.warning); }
      catch(e){ dc.checked=!dc.checked; csay(String(e.message||e)); }
    };
    const fc=mc.querySelector('#hapFollowCar');
    fc.onchange=()=>{ cfgObj.haptics.followCar=!!fc.checked; refreshDirtyUI(); };
    hapCarListLoad();
    const gi=mc.querySelector('#hapMasterGain');
    gi.onchange=()=>{ const v=+gi.value; if(!isFinite(v)) return;
      cfgObj.haptics.masterGain=Math.max(0,Math.min(2,v)); gi.value=cfgObj.haptics.masterGain;
      refreshDirtyUI(); };
    const di=mc.querySelector('#hapAxisDelay');
    di.onchange=()=>{ const v=+di.value; if(!isFinite(v)) return;
      cfgObj.haptics.axisDelayMs=Math.max(0,Math.min(60,v)); di.value=cfgObj.haptics.axisDelayMs;
      refreshDirtyUI(); };
    const bg=mc.querySelector('#hapBeltFullG');
    if(bg) bg.onchange=()=>{ let v=+bg.value; if(!isFinite(v)) return;
      v=(v<=0)?0:Math.max(0.2,Math.min(6,v)); cfgObj.haptics.beltFullG=v; bg.value=v; refreshDirtyUI(); };
    const bi=mc.querySelector('#hapPosBudget');
    bi.onchange=()=>{ const v=+bi.value; if(!isFinite(v)) return;
      cfgObj.haptics.positionBudget=Math.max(0,Math.min(1,v)); bi.value=cfgObj.haptics.positionBudget;
      refreshDirtyUI(); };
    mc.querySelector('#hapMuteBtn').onclick=async()=>{
      const on=!hapMutedNow;
      try{ await fetch(API+'/api/haptics/mute',{method:'POST',body:JSON.stringify({on:on})}); }catch(_){}
    };
    strip.appendChild(mc);
  }

  for(const fx of HAP_FX){
    const dv=cfgObj.haptics[fx.k];
    const tile=document.createElement('div');
    tile.className='hap-tile'+(dv.ampPct>0?' on':''); tile.title=fx.tip; tile.dataset.fx=fx.k;
    let h='<div class="dh"><span class="lamp mxl"></span><span class="dot"></span><span class="nm">'+fx.label+'</span><span class="mxst" data-st="1"></span><span class="sdot" data-sdot="1" title="Sim: no channels seen from this game yet"></span>'
      +'<button class="hmute" type="button" data-mute="1" title="Mute this effect: it goes silent, its settings and readouts stay as they are, and no other tile changes. The master MUTE mutes every tile and UNMUTE unmutes them all: mute all, then unmute one tile to hear it on its own. Not saved: a restart unmutes.">M</button></div>';
    // The chip row is always present (empty for channel-less effects) so
    // the wave line lands at the same height on every tile.
    h+=(fx.chan&&fx.chan.length)
      ?'<div class="hk hchan" data-chan="1">'+fx.chan.map(e=>hapChanSpec(e).label).join(' ')+'</div>'
      :'<div class="hk hchan"></div>';
    // The wave carries a source tag (live input vs Test) so a moving wave
    // with nothing felt reads as "live input, small", not as stuck.
    h+='<div class="hap-wwrap"><svg class="hap-wave"></svg><span class="hap-wtag" data-wtag="1"></span><span class="mxlv" title="What it puts out now, with the recent peak"><i></i><b></b></span></div>';
    // Per-wheel readout (slip severities 0..1, road travel mm): value and
    // session peak per corner. Test drives 1.00 on every wheel, so the
    // line is the direct comparison; the peak holds until Reset peaks.
    const wset=hapWheelSet(fx.k);
    const wtitle=(wset===4)?'Flat spot per wheel, mm: now/peak since Reset peaks'
      :(wset===5)?'Per wheel: the knock still ringing / the hardest knock since Reset peaks (0..1)'
      :(wset===2)?'Per wheel: road travel mm now/peak since Reset peaks; "air" while that tyre is off the ground'
      :'Per wheel: now/peak since Reset peaks (Test = 1.00 on every wheel)';
    if(wset>=0) h+='<div class="hk hwheels" data-wheels="'+wset+'" title="'+wtitle+'"><span>FL -</span><span>FR -</span><span>RL -</span><span>RR -</span></div>';
    if(fx.k==='slipLat') h+='<div class="hk hwheels" data-ang="1" title="Each wheel\'s slip angle as played, degrees, and after the slash its straight-line baseline (learned on the straights and taken off, where the sim sends a slip speed: Automobilista 2, Le Mans Ultimate). A baseline that stays 0 or keeps climbing means it is not learning right."><span>FL -</span><span>FR -</span><span>RL -</span><span>RR -</span></div>'
      +'<div class="hap-game"><span class="hk hwstate" data-angstate="1" title="Where the slip angle comes from. From a slip speed (Automobilista 2, Le Mans Ultimate): whether the straight-line baseline is learning (amber), learned, learned and held (baseline: learn once), or fixed, and how many seconds of straights it has learned over. It learns on steady straights above 60 km/h, off the brake.">angle -</span>'
      +'<button class="btn btn-sm" type="button" data-relearn="1" title="Throw away the learned straight-line baseline and learn it again from the next straights (a new car does this by itself). With baseline fixed it changes nothing.">relearn</button></div>';
    if(fx.k==='abs') h+='<div class="hk hwstate" data-abssrc="1" title="What says ABS is working right now: the sim\'s flag (held over its gaps), the wheels (with from wheels on: their dumps, and the rate they come at), or both.">ABS -</div>';
    if(fx.k==='wheels') h+='<div class="hap-game"><span class="hk hwstate" data-wstate="1" title="The tyres right now: flat spots (mm, from lock-ups), brake discs hot enough to judder, a puncture. Amber or red while there is anything. They last until a different car, ten minutes without the sim, or fresh tyres; with fresh tyres on pit/restart on, also until a restart, a jump back to the pits or new tyres at a stop.">tyres -</span>'
      +'<button class="btn btn-sm" type="button" data-fresh="1" title="Fresh tyres: wipe the flat spots and cool the discs now (after a restart, or a lock-up you want gone). A puncture is the sim\'s and stays while it says so.">fresh tyres</button></div>';
    else if(fx.k==='surface') h+='<div class="hk hwheels" data-surf="1" title="What each tyre is on, as nullCAT reads it: the surface, wet or floating, or in the air"><span>FL -</span><span>FR -</span><span>RL -</span><span>RR -</span></div>';
    else if(!fx.transient&&fx.fxIdx!==undefined) h+='<div class="hk hwheels" data-in="1" title="What this effect is driven with: now/peak since Reset peaks (Test = 1.00)">in -</div>';
    h+='<div class="hrows">';
    for(const [key,lab,min,max,st,opts] of fx.params){
      h+='<div class="hr'+(fx.carKeys.includes(key)?' carkey':fx.typeKeys.includes(key)?' typekey':'')+'" data-pk="'+key+'"><span class="hk">'+lab+'</span>';
      if(opts&&opts.length===2&&/^(off|no)$/i.test(opts[0]))   // an on/off choice: a tickbox (value 0 / 1)
        h+='<input type="checkbox" data-k="'+key+'"'+(Math.round(+dv[key])===1?' checked':'')+'></div>';
      else if(opts)   // a choice, not a number: value = option index
        h+='<select data-k="'+key+'">'+opts.map((o,i)=>'<option value="'+i+'"'+(Math.round(+dv[key])===i?' selected':'')+'>'+o+'</option>').join('')+'</select></div>';
      else
        h+='<input type="number" min="'+min+'" max="'+max+'" step="'+st+'" data-k="'+key+'" value="'+dv[key]+'"></div>';
    }
    h+='</div>';
    // The Engine tile names the car the table matched and folds its notes
    // (the real-world figures the values came from) under it.
    if(fx.k==='rpmVibe') h+='<details class="hcar" data-car="1" hidden><summary data-carsum="1"></summary><div class="hnotes" data-carnotes="1"></div></details>';
    h+='<div class="hb"><button class="hap-routechip" type="button">'+hapChipText(dv)+'</button>'
      +'<button class="btn btn-sm btn-action" type="button" data-test="1">Test</button></div>';
    tile.innerHTML=h;
    // Each setting's hover text: what it does, a range, starting points;
    // a setting the car table sets says so first (or, while the car has no
    // entry, that it is on its default).
    for(const [key,,,,,,help] of fx.params){
      const car=fx.carKeys.includes(key)?hapCarKeyTip():fx.typeKeys.includes(key)?HAP_TYPEKEY_TIP:'';
      if(!help&&!car) continue;
      const row=tile.querySelector('[data-pk="'+key+'"]'); if(row){ const t=car+(car&&help?'\n\n':'')+(help||''); row.dataset.phelp=help||''; row.dataset.help=t; row.title=t; }
    }
    tile.querySelector('[data-mute]').onclick=async()=>{
      const on=!tile.classList.contains('muted');
      try{ await fetch(API+'/api/haptics/mute',{method:'POST',body:JSON.stringify({effect:fx.k,on:on})}); }catch(_){}
    };
    const svg=tile.querySelector('svg'); hapWave(svg,fx,dv);
    tile.querySelectorAll('input,select').forEach(inp=>{ inp.onchange=()=>{
      const box=(inp.type==='checkbox');
      const v=box?(inp.checked?1:0):+inp.value; if(!isFinite(v)) return;
      dv[inp.dataset.k]=(inp.tagName==='SELECT'||box)?v:Math.max(+inp.min,Math.min(+inp.max,v));
      if(inp.tagName!=='SELECT'&&!box) inp.value=dv[inp.dataset.k];
      hapWave(svg,fx,dv);
      tile.classList.toggle('on',dv.ampPct>0);
      refreshDirtyUI();
    }; });
    hapTiles[fx.k]={svg,fx,dv,tile,level:0,anim:false,testUntil:0,inText:''};
    tile.querySelector('.hap-routechip').onclick=()=>hapDrawerRender(fx);
    { const rl=tile.querySelector('[data-relearn]');
      if(rl) rl.onclick=async()=>{ try{ await fetch(API+'/api/haptics/relearn',{method:'POST',body:'{}'}); }catch(_){} };
      const fr=tile.querySelector('[data-fresh]');
      if(fr) fr.onclick=async()=>{ try{ await fetch(API+'/api/haptics/fresh-tyres',{method:'POST',body:'{}'}); }catch(_){} }; }
    tile.querySelector('[data-test]').onclick=async(ev)=>{
      const b=ev.target; b.disabled=true;
      try{
        const r=await fetch(API+'/api/haptics/test',{method:'POST',
          body:JSON.stringify({effect:fx.k})});
        const j=await r.json();
        b.textContent=j.ok?'Sent':'✗';
        const m=$('hapMsg');
        if(j.ok){
          if(m) m.textContent='';
          const ms=fx.transient?Math.max(400,(+dv.durMs||18)*4):2000;
          const t=hapTiles[fx.k]; if(t) t.testUntil=performance.now()+ms;
          hapAnimate(svg,fx,dv,ms,'test');
        }else if(m){
          m.textContent='Test refused: '+(j.error||'no response');
          setTimeout(()=>{ if(m.textContent.startsWith('Test refused')) m.textContent=''; },6000);
        }
      }catch(_){ b.textContent='✗';
        const m=$('hapMsg'); if(m) m.textContent='Test failed: no response from the controller.'; }
      setTimeout(()=>{ b.textContent='Test'; b.disabled=false; },1200);
    };
    strip.appendChild(tile);
  }
  hapCarRefresh();   // the car line and the Engine tile's notes, once the tiles exist
  // A route drawer open across a rebuild shows the config as it is now.
  if(hapOpenDrawer&&hapTiles[hapOpenDrawer]){ const k=hapOpenDrawer; hapOpenDrawer=null; hapDrawerRender(hapTiles[k].fx); }
  hapMixerBuild();   // the tiles as mixer rows, the editor, the fold, the rail
}

/* ---- The mixer, the editor and the fold (presentation over the tiles
   hapInit built: nothing here changes what a tile writes or posts).
   A tile IS its mixer row (CSS); its sim dot is the lamp, its wave the
   mini scope, its M key the mute. A fader mirrors the tile's amp input
   (it writes through that input's own handler). Selecting an effect moves
   its tile into the editor pane, leaves a placeholder row, and brings the
   route drawer under it as the routing section; every lookup hapLive
   makes stays inside the tile, so it keeps updating wherever it sits. ---- */
// The groups, in this order, with a rule between them and no headings:
// mostly what the car table pulls per car, then mostly what the type of
// car sets, then the rest, the H-shifter's detent click last. No names:
// both reach into more than one group, so a name would mislead.
const HAP_CATS=[['Car',['rpmVibe','limiter','driveline','tc','abs']],['Type of car',['slipLon','slipLat','road','wheels','surface','kerb','impacts']],['Other',['gearShift','detentClick']]];
const HAP_ABBR={rpmVibe:'ENG',limiter:'LIM',gearShift:'GEAR',driveline:'DRV',tc:'TC',abs:'ABS',slipLon:'LONG',slipLat:'LAT',wheels:'WHL',road:'ROAD',kerb:'RUMB',surface:'SURF',impacts:'IMP',detentClick:'CLK'};
const HAP_SEL_KEY='nullcat.ui.sel', HAP_FOLD_KEY='nullcat.ui.rigfold';
let hapSel=null; try{ hapSel=localStorage.getItem(HAP_SEL_KEY)||null; }catch(_){}
function hapCatOf(k){ for(const [n,ks] of HAP_CATS) if(ks.includes(k)) return n; return 'Effects'; }
function hapMixerDress(t){
  const tile=t.tile, num=tile.querySelector('input[data-k="ampPct"]'), wrap=tile.querySelector('.hap-wwrap'), nm=tile.querySelector('.dh .nm');
  if(nm&&!nm._wired){ nm._wired=true; nm.onclick=()=>hapOpenFx(t.fx.k); nm.title='Open '+t.fx.label; }
  const chip=tile.querySelector('.hap-routechip'); if(chip&&!chip._dr){ chip._dr=true; const o=chip.onclick; chip.onclick=()=>{ if(o) o(); hapDrawerDress(); }; }
  const svg=tile.querySelector('svg.hap-wave'); if(svg) svg.setAttribute('preserveAspectRatio','none');
  if(!num||!wrap||tile.querySelector('.mxamp')) return;
  const val=document.createElement('span'); val.className='mxval vt'; wrap.appendChild(val);
  const rg=document.createElement('input'); rg.type='range'; rg.className='mxamp'; rg.min=num.min; rg.max=num.max; rg.step=num.step||'1'; rg.title='amp %';
  const sync=()=>{ rg.value=num.value; const on=+num.value>0; val.textContent=on?String(+num.value):'OFF'; val.classList.toggle('dim',!on); };
  rg.oninput=()=>{ num.value=rg.value; num.dispatchEvent(new Event('change')); };
  num.addEventListener('change',sync); sync();
  tile.appendChild(rg);
}
function hapMixerBuild(){
  const strip=$('hapStrip'), edb=$('hapEdBody'); if(!strip) return;
  // a tile from before this rebuild still in the editor: gone with its rebuild
  if(edb) edb.querySelectorAll('.hap-tile').forEach(e=>e.remove());
  const tiles=[...strip.querySelectorAll('.hap-tile[data-fx]')];
  const master=strip.querySelector('.hap-tile:not([data-fx])');
  for(const tile of tiles){ const t=hapTiles[tile.dataset.fx]; if(t) hapMixerDress(t); }
  // groups, in the design's order; anything unlisted under Effects
  const byCat={}; for(const tile of tiles) (byCat[hapCatOf(tile.dataset.fx)]=byCat[hapCatOf(tile.dataset.fx)]||[]).push(tile);
  const order=HAP_CATS.map(c=>c[0]).concat(Object.keys(byCat).filter(c=>!HAP_CATS.some(x=>x[0]===c)));
  // within a group, the order its list gives (not the registry's)
  const rank=k=>{ for(const [,ks] of HAP_CATS){ const i=ks.indexOf(k); if(i>=0) return i; } return 99; };
  for(const c in byCat) byCat[c].sort((a,b)=>rank(a.dataset.fx)-rank(b.dataset.fx));
  let firstGroup=true;
  for(const cat of order){ if(!byCat[cat]) continue;
    if(!firstGroup){ const g=document.createElement('div'); g.className='mxg'; g.dataset.group=cat; strip.appendChild(g); }
    firstGroup=false;
    for(const tile of byCat[cat]) strip.appendChild(tile); }
  // the Master tile: its profile and follow-car rows up into the session
  // block, Mute and Rec into the panel head, the rest in a fold
  if(master){
    const ses=$('hapSesProf'); if(ses){ ses.innerHTML='';   // a previous build's rows: their Master tile is gone
      const pr=master.querySelector('#hapProfSel'); const prow=pr&&pr.closest('.hr');
      if(prow){ prow.classList.add('hap-game');
        { const sec=prow.previousElementSibling; if(sec&&sec.classList.contains('hsec')) sec.hidden=true; }
        // the four profile actions fold into a More menu (the same buttons, same handlers)
        const menu=document.createElement('details'); menu.className='menu';
        menu.innerHTML='<summary class="key sm nolamp">More</summary><div class="mnu"></div>';
        const mn=menu.querySelector('.mnu');
        ['hapProfLoad','hapProfSave','hapProfBind','hapProfDel'].forEach(id=>{ const b=prow.querySelector('#'+id); if(b){ b.className=''; mn.appendChild(b); } });
        const msg=prow.querySelector('#hapProfMsg');
        prow.appendChild(menu); if(msg) prow.appendChild(msg);
        menu.addEventListener('click',e=>{ if(e.target.closest('.mnu button')) menu.open=false; });
        ses.appendChild(prow); }
      const fc=master.querySelector('#hapFollowCar'); const frow=fc&&fc.closest('.hr');
      if(frow){ const lab=fc.closest('label'); if(lab){ lab.className='key sm latch'+(fc.checked?' on':''); fc.hidden=true; fc.addEventListener('change',()=>lab.classList.toggle('on',fc.checked)); }
        if(prow) prow.appendChild(frow.firstElementChild===lab?lab:frow.querySelector('label')||frow); frow.remove(); } }
    // The type of car and the car table's re-apply are what is touched
    // first: up here under the car (the rest of their sections stay in the
    // fold). The same elements, so the same handlers.
    const tbox=$('hapSesType'), cbox=$('hapSesCarRow');
    if(tbox){ tbox.innerHTML='';
      const ts=master.querySelector('#hapPresetSel'), trow=ts&&ts.closest('.hr');
      if(trow){ const lab=trow.querySelector('.hk'); if(lab) lab.textContent='Type';
        const ap=trow.querySelector('#hapPresetApply'); if(ap){ ap.className='key sm nolamp'; ap.textContent='Apply'; }
        tbox.appendChild(trow); } }
    if(cbox){ cbox.innerHTML='';
      const cp=master.querySelector('#hapCarPull'), crow=cp&&cp.closest('.hr');
      if(crow){ const lab=crow.querySelector('.hk'); if(lab) lab.textContent='Car';
        const now=crow.querySelector('#hapCarNow'); if(now) now.hidden=true;   // the glass above names it
        cp.className='key sm nolamp'; cp.textContent='Re-apply';
        const menu=document.createElement('details'); menu.className='menu';
        menu.innerHTML='<summary class="key sm nolamp">More</summary><div class="mnu"></div>';
        const mn=menu.querySelector('.mnu');
        ['hapCarSave','hapCarForget'].forEach(id=>{ const b=crow.querySelector('#'+id); if(b){ b.className=''; mn.appendChild(b); } });
        const msg=crow.querySelector('#hapCarMsg');
        crow.appendChild(menu); if(msg) crow.appendChild(msg);
        menu.addEventListener('click',e=>{ if(e.target.closest('.mnu button')) menu.open=false; });
        cbox.appendChild(crow); } }
    const hk=$('hapHeadKeys'); if(hk){ hk.innerHTML=''; const rb=master.querySelector('#hapRecBtn'), mb=master.querySelector('#hapMuteBtn');
      if(rb){ rb.className='key sm latch red'; rb.textContent='Rec'; hk.appendChild(rb); }
      if(mb){ mb.className='key sm latch amb'; hk.appendChild(mb); } }
    let open=false; try{ open=localStorage.getItem(HAP_FOLD_KEY)==='1'; }catch(_){}
    const fold=document.createElement('div'); fold.className='sxs'; fold.innerHTML='<div class="sx'+(open?' open':'')+'" id="hapRigFold"><button type="button" class="sxh"><span class="arw"></span><span class="sxt">Rig · car · sim</span><span class="sp"></span><span class="sum" id="hapRigSum"></span></button><div class="sxb"></div></div>';
    fold.querySelector('.sxb').appendChild(master);
    fold.querySelector('.sxh').onclick=()=>{ const sx=fold.querySelector('.sx'); const o=sx.classList.toggle('open'); try{ localStorage.setItem(HAP_FOLD_KEY,o?'1':'0'); }catch(_){} };
    strip.appendChild(fold);
    const gf=$('hapGainFader'), gi=master.querySelector('#hapMasterGain');
    if(gf&&gi) gf.oninput=()=>{ gi.value=gf.value; gi.dispatchEvent(new Event('change')); };   // rebound every build: gi is this build's input
  }
  // the rail: one item per effect, in the mixer's grouped order
  const rail=$('hpRail'); if(rail){ rail.innerHTML='';
    for(const tile of strip.querySelectorAll('.hap-tile[data-fx]')){ const k=tile.dataset.fx, t=hapTiles[k]; if(!t) continue;
      const r=document.createElement('button'); r.type='button'; r.className='rx'; r.dataset.fx=k; r.title=t.fx.label;
      r.innerHTML='<span class="lamp"></span><span class="rxn"></span>'; r.querySelector('.rxn').textContent=HAP_ABBR[k]||t.fx.label.slice(0,4).toUpperCase();
      r.onclick=()=>hapOpenFx(k); rail.appendChild(r); } }
  // an effect that was open stays open across the rebuild
  if(hapSel&&hapTiles[hapSel]&&uiLayout.hp==='ed') hapOpenFx(hapSel,true);
  else if(uiLayout.hp==='ed'||uiLayout.mo==='rail') setLayout(uiLayout.mo,'mix');   // (Motion on the rail opens the first effect)
}
function hapOpenFx(k,force){
  const t=hapTiles[k]; if(!t) return;
  if(!force&&hapSel===k&&uiLayout.hp==='ed'){ hapCloseEd(); return; }
  hapCloseEd(true);
  hapSel=k; try{ localStorage.setItem(HAP_SEL_KEY,k); }catch(_){}
  const ph=document.createElement('div'); ph.className='mx-ph'; ph.dataset.ph=k;
  ph.innerHTML='<span class="lamp acc"></span><button type="button" class="mn"></button><span class="hint">editing</span>';
  ph.querySelector('.mn').textContent=t.fx.label; ph.querySelector('.mn').onclick=()=>hapCloseEd();
  t.tile.replaceWith(ph);
  const edb=$('hapEdBody'); if(edb){ edb.appendChild(t.tile); const dr=$('hapDrawer'); if(dr) edb.appendChild(dr); }
  t.tile.classList.add('ined'); t.tile._tip=t.tile.title; t.tile.title='';
  const nm=$('hapEdName'); if(nm) nm.textContent=t.fx.label;
  const kk=$('hapEdKick'); if(kk) kk.textContent=hapCatOf(k);
  const tip=$('hapEdTip'); if(tip){ tip.textContent=t.fx.tip||''; tip.classList.remove('open'); }
  if(hapOpenDrawer!==k) hapDrawerRender(t.fx);
  hapDrawerDress();
  const dr=$('hapDrawer'); if(dr) dr.hidden=false;
  document.querySelectorAll('#hpRail .rx').forEach(r=>r.classList.toggle('sel',r.dataset.fx===k));
  setLayout(uiLayout.mo,'ed');
  hapEdDress();
}
// The editor holds an effect. Before the config arrives there are no tiles
// (and the panel is hidden): a stored "editor open" layout then waits for
// hapMixerBuild to reopen it, and nothing may close it and drop the choice.
function hapEdOpen(){ const t=hapSel&&hapTiles[hapSel]; return !!(t&&t.tile.classList.contains('ined')); }
function hapCloseEd(keepSel){
  const k=hapSel, t=k&&hapTiles[k], ph=document.querySelector('#hapStrip [data-ph]');
  if(t&&ph&&t.tile.classList.contains('ined')){ ph.replaceWith(t.tile); t.tile.classList.remove('ined'); if(t.tile._tip) t.tile.title=t.tile._tip; }
  else if(ph) ph.remove();
  const dr=$('hapDrawer'), strip=$('hapStrip'); if(dr&&strip&&dr.parentElement!==strip.parentElement){ strip.insertAdjacentElement('afterend',dr); }
  if(dr){ dr.hidden=true; } hapOpenDrawer=null;
  if(!keepSel){ hapSel=null; try{ localStorage.removeItem(HAP_SEL_KEY); }catch(_){} document.querySelectorAll('#hpRail .rx').forEach(r=>r.classList.remove('sel')); if(uiLayout.hp==='ed') setLayout(uiLayout.mo,'mix'); }
}
{ const c=$('hapEdClose'); if(c) c.onclick=()=>hapCloseEd();
  const h=$('hapEdHelp'); if(h) h.onclick=()=>{ const t=hapSel&&hapTiles[hapSel]; const on=h.classList.toggle('on'); if(t) t.tile.classList.toggle('help',on); };
  const tip=$('hapEdTip'); if(tip) tip.onclick=()=>tip.classList.toggle('open');
  document.addEventListener('keydown',e=>{ if(e.key==='Escape'&&uiLayout.hp==='ed'&&hapEdOpen()&&document.body.dataset.view==='op') hapCloseEd(); }); }
/* The routing heading hapDrawerRender writes is one line, title and hint
   together; in the editor the hint goes under the title in smaller type. */
function hapDrawerDress(){
  const t=document.querySelector('#hapDrawer .hd-t'); if(!t||t.querySelector('.hd-h')) return;
  const s=t.textContent, i=s.indexOf(' ('); if(i<0) return;
  t.textContent=''; const a=document.createElement('span'); a.textContent=s.slice(0,i);
  const b=document.createElement('span'); b.className='hd-h'; b.textContent=s.slice(i+2).replace(/\)$/,'');
  t.append(a,b);
}
/* The editor's banners and input chips, from what the tile already shows:
   amp 0, no route, and the channel line hapLive writes (a name then
   ✓ received, · optional and idle, ✗ required but not sent). */
function hapEdDress(){
  const bans=$('hapEdBans'), chips=$('hapEdChips'); if(!bans||!chips) return;
  const t=hapSel&&hapTiles[hapSel];
  if(!t||uiLayout.hp!=='ed'){ bans._h=chips._h=''; bans.innerHTML=''; chips.innerHTML=''; return; }
  const ch=t.tile.querySelector('[data-chan]'), txt=ch?ch.textContent.trim():'';
  let items=[];
  if(/[✓✗·]/.test(txt)){ const parts=txt.split(/\s*([✓✗·])\s*/).filter(Boolean); for(let i=0;i+1<parts.length;i+=2) items.push({label:parts[i],mark:parts[i+1]}); }
  else if(t.fx.chan) items=t.fx.chan.map(e=>({label:hapChanSpec(e).label,mark:''}));
  const missing=items.filter(x=>x.mark==='✗').map(x=>x.label);
  const routed=(t.dv.routes||[]).some(r=>r&&r.gain>0);
  const b=[]; if(!(+t.dv.ampPct>0)) b.push('off: amp 0'); if(!routed) b.push('not routed'); if(missing.length) b.push('not sent: '+missing.join(', '));
  const bk=b.join('|'); if(bans._h!==bk){ bans._h=bk; bans.innerHTML='';
    for(const s of b){ const e=document.createElement('span'); e.className='ban'; e.innerHTML='<span class="lamp warn"></span><span class="vt warn"></span>'; e.lastChild.textContent=s; bans.appendChild(e); } }
  const ck=items.map(x=>x.label+x.mark).join('|'); if(chips._h!==ck){ chips._h=ck; chips.innerHTML='';
    for(const x of items){ const e=document.createElement('span'); e.className='chip'; e.innerHTML='<span class="lamp"></span><span></span>';
      e.firstChild.className='lamp '+(x.mark==='✓'?'ok':x.mark==='✗'?'warn':''); e.lastChild.textContent=x.label; e.title=x.mark==='✓'?'received':x.mark==='✗'?'required, not sent':x.mark==='·'?'optional, idle':'no sender yet'; chips.appendChild(e); } }
}
/* Once a second: the session readouts, the head keys' lit state, the
   fold's summary, the rail lamps and the live count, from state the page
   already holds. */
function hapUiRefresh(){
  const panel=$('hapPanel'); if(!panel) return;
  if(panel.hidden){ if(uiLayout.hp==='ed'&&hapEdOpen()) hapCloseEd(); return; }
  const cg=$('hapSesCar'), gg=$('hapSesGame'); if(gg) gg.textContent=(($('hapGameNow')||{}).textContent||'-');
  if(cg){ // the name (it truncates) and what the car table has for it (it never does)
    const c=hapCarLast, tags=[];
    if(c&&c.car){
      if(!c.source) tags.push(['no entry','w']);
      else { tags.push([c.source==='user'?'yours':'stock','']); if(c.near) tags.push(['near match','w']); if(!c.applied) tags.push(['not applied','w']); }
      if(c.defaults) tags.push(['defaults','w']);
    }
    const name=(c&&c.car)?(c.name||c.car):'(none)', key=name+'|'+tags.map(t=>t[0]).join('|');
    if(cg._k!==key){ cg._k=key; cg.textContent='';
      const n=document.createElement('span'); n.className='cn'; n.textContent=name; cg.appendChild(n);
      for(const [t,w] of tags){ const e=document.createElement('span'); e.className='ctag'+(w?' w':''); e.textContent=t; cg.appendChild(e); } }
    cg.title=(($('hapCarNow')||{}).title)||''; }
  const gi=$('hapMasterGain'), gf=$('hapGainFader'), gs=$('hapGainSeg');
  if(gi&&gf&&document.activeElement!==gf) gf.value=gi.value; if(gi&&gs) setSeg(gs,(+gi.value||0).toFixed(2));
  const mb=$('hapMuteBtn'); if(mb) mb.classList.toggle('on',hapMutedNow);
  { const fc=$('hapFollowCar'); const lab=fc&&fc.closest('label'); if(lab&&lab.classList.contains('key')) lab.classList.toggle('on',fc.checked); }
  const rb=$('hapRecBtn'); if(rb) rb.classList.toggle('on',hapRecOn);
  const sum=$('hapRigSum'); if(sum){ const b=$('hapPosBudget'), d=$('hapAxisDelay'), f=$('hapFollowCar');
    sum.textContent='budget '+(b?b.value:'-')+' · delay '+(d?d.value:'-')+' ms · '+(f&&f.checked?'follows car':'manual'); }
  let live=0, any=false;
  for(const k in hapTiles){ const t=hapTiles[k]; const routed=(t.dv.routes||[]).some(r=>r&&r.gain>0);
    const muted=t.tile.classList.contains('muted'); if(t.dv.ampPct>0&&routed&&!muted) live++; if(t.level>0.05&&!muted) any=true;
    const rx=document.querySelector('#hpRail .rx[data-fx="'+k+'"]'); if(rx){ rx.querySelector('.lamp').className='lamp '+((t.st&&t.st.lamp)||''); if(t.st) rx.title=t.fx.label+': '+t.st.text; } }
  const pc=$('hpLive'); if(pc) pc.textContent=live+' live';
  const hl=$('hpLamp'); if(hl) hl.className='lamp '+(hapMutedNow?'warn':any?'ok':'');
  hapEdDress();
}
setInterval(hapUiRefresh,1000);

/* Each tile's state, from the status twice a second: one word and a lamp
   per tile (the mixer row, the editor's head and the rail), its trigger
   count, a level bar under its wave, and the session glass's Now row.
   Muted, off, routed nowhere, no game, no input (a channel it needs is not
   sent), no kerb surface (the Rumble strip in a game that names none, with
   from road heights off), a low input rate (Rumble strip on road heights
   slower than its detector needs), playing, or ready; an event that has not
   happened yet this session (since Reset peaks or the loop start) is armed. */
const HAP_EVENTS=['tc','abs','limiter','kerb','impacts','gearShift','detentClick'];
const HAP_MTR={}; let hapMtrRaf=0, hapMtrT=0;
function hapAgoText(sec){ return sec<60?Math.round(sec)+' s ago':sec<3600?Math.round(sec/60)+' min ago':Math.round(sec/3600)+' h ago'; }
function hapTileState(t,s){
  const id=t.fx.id, has=a=>Array.isArray(a)&&id!==undefined;
  const routed=(t.dv.routes||[]).some(r=>r&&r.gain>0);
  const muted=hapMutedNow||(has(s.hapMute)&&!!s.hapMute[id]);
  const n=has(s.hapCount)?(+s.hapCount[id]||0):0, ago=has(s.hapAgo)?+s.hapAgo[id]:-1;
  const ev=HAP_EVENTS.includes(t.fx.k), cnt=(ev&&n>0)?' · ×'+n:'';
  if(muted) return {st:'muted',lamp:'',cls:'dim',text:'muted',tip:'Muted: silent until unmuted (its M, or the master UNMUTE). Its settings stay as they are.'};
  if(!(+t.dv.ampPct>0)) return {st:'off',lamp:'',cls:'dim',text:'off',tip:'Off: amp 0.'};
  if(!routed) return {st:'unrouted',lamp:'warn',cls:'warn',text:'unrouted',tip:'Turned up but routed nowhere: give it an axis or a shaker in its editor.'};
  // Channels not arriving are unknown, not missing: no game, paused, or
  // no channels from the sender, before anything is called not sent.
  if(!s.telemetryReceiving&&!s.ncxRx) return {st:'nogame',lamp:'',cls:'dim',text:'no game'+cnt,tip:'No telemetry arriving: start the sim and its sender.'};
  if(s.ncxFrozen) return {st:'paused',lamp:'',cls:'dim',text:'paused'+cnt,tip:'The game is paused: its channels are holding still.'};
  if(!s.ncxRx) return {st:'nochan',lamp:'',cls:'dim',text:'no channels'+cnt,tip:'Motion arrives but no effect channels: the sender is not sending them (or they stopped).'};
  const missing=[];
  for(const e of (t.fx.chan||[])){ const sp=hapChanSpec(e); if(!sp.optional&&!sp.ok(s.ncxHave||[])) missing.push(sp.label); }
  if(missing.length) return {st:'noinput',lamp:'warn',cls:'warn',text:'no input · needs '+missing[0],tip:'This game does not send what it needs: '+missing.join(', ')+'.'};
  let via='', viaTip='';
  if(t.fx.k==='kerb'){
    const on=nm=>{ const i=NCX_TOKENS.indexOf(nm); return i>=0&&Array.isArray(s.ncxHave)&&!!s.ncxHave[i]; };
    const surf=['FL','FR','RL','RR'].every(w=>on('surface'+w));
    const heightsOn=Math.round(+t.dv.heights)===1&&+t.dv.detectMm>0;
    // A game that names no kerb surface and sends no kerb channel of its own
    // (by name: the numbered line carries the slot as 0 whatever the game
    // has) has no rumble strip of its own: said, rather than left to look
    // broken.
    if(!surf&&!s.curbsNamed){
      if(!heightsOn) return {st:'nosurface',lamp:'',cls:'dim',text:'no kerb surface'+cnt,short:'no kerb surface',
        tip:'This game does not say which tyre is on a kerb, so there is no rumble strip to play. From road heights finds the kerb\'s edge in the road heights instead: a thump on and off rather than a rumble.'};
      via=' · from road heights';
      viaTip=' This game does not say which tyre is on a kerb: the kerbs come from the road heights, which find the kerb\'s edge (a thump on and off) rather than its ribs\' rumble.';
    }
    // the height detector runs beside a kerbs channel (the stronger of the
    // two), so a slow rate fires at random whatever the channel does
    const viaHeights=!surf&&heightsOn, hz=+s.hapHeightsHz||0;
    if(viaHeights&&hz>0&&hz<150) return {st:'lowrate',lamp:'warn',cls:'warn',text:'low rate · heights '+Math.round(hz)+'/s'+cnt,short:'low rate · '+Math.round(hz)+'/s',
      tip:'Kerbs are found from the road heights, arriving '+Math.round(hz)+' times a second: too few for a rumble strip\'s ribs (about 110 a second at 100 km/h), so it fires at random. With the SimHub plugin, turn kunosDirectHeights on for the game\'s physics rate.'};
  }
  const playing=t.fx.transient?(ago>=0&&ago<1):((+t.level||0)>0.05);
  if(playing) return {st:'playing',lamp:'ok',cls:'ok',text:'playing'+cnt+via,short:'playing'+cnt,tip:'Playing now.'+viaTip};
  if(ev&&!n&&has(s.hapCount)) return {st:'armed',lamp:'',cls:'',text:'armed · none yet'+via,short:'armed · none yet',tip:'Ready, and nothing has set it off yet for this car (since it was named, Reset peaks or the loop start).'+viaTip};
  const when=(ev&&ago>=0)?' · '+hapAgoText(ago):'';
  return {st:'ready',lamp:'',cls:'',text:'ready'+cnt+when+via,short:(ev&&n>0)?'×'+n+when:'ready',tip:'Ready: fed, nothing to play right now.'+viaTip};
}
function hapStatesUpdate(s){
  let playing=0; const warn=[], armed=[];
  for(const k in hapTiles){
    const t=hapTiles[k], st=hapTileState(t,s); t.st=st;
    const l=t.tile.querySelector('.mxl'); if(l) l.className='lamp mxl'+(st.lamp?' '+st.lamp:'');
    const x=t.tile.querySelector('[data-st]');
    if(x){ const tx=t.tile.classList.contains('ined')?st.text:(st.short||st.text);
      if(x.textContent!==tx) x.textContent=tx; x.className='mxst'+(st.cls?' '+st.cls:''); x.title=st.text+'\n\n'+st.tip; }
    // the level bar: a continuous effect's output, an event's flick
    const m=HAP_MTR[k]||(HAP_MTR[k]={now:0,disp:0,pk:0,pkAt:0,n:-1});
    const n=(Array.isArray(s.hapCount)&&t.fx.id!==undefined)?(+s.hapCount[t.fx.id]||0):0;
    m.now=t.fx.transient?((m.n>=0&&n>m.n)?1:0):Math.max(0,Math.min(1,+t.level||0));
    if(st.st==='muted'||st.st==='off') m.now=0;
    m.n=n;
    if(st.st==='playing') playing++;
    else if(st.lamp==='warn') warn.push(t.fx.label+' '+st.text.split(' · ')[0]);
    else if(st.st==='armed'&&(k==='abs'||k==='tc')) armed.push(t.fx.label.replace(/ pulse$/i,''));
  }
  hapMtrKick();
  const ne=$('hapSesNow'); if(ne){
    const live=!!(s.telemetryReceiving||s.ncxRx)&&!s.ncxFrozen;
    const parts=live?[playing+' playing']:[s.ncxFrozen?'paused':'no game'];
    if(live&&armed.length) parts.push(armed.join(', ')+' none yet');
    parts.push(...warn);
    const txt=parts.join(' · '); if(ne.textContent!==txt) ne.textContent=txt;
    ne.className='vt'+(warn.length?' warn':live?'':' dim');
  }
}
// The level bars between status readings: each falls with a 0.3 s release,
// never below the newest reading, a peak tick held for 1.5 s.
function hapMtrKick(){ if(!hapMtrRaf){ hapMtrT=0; hapMtrRaf=requestAnimationFrame(hapMtrTick); } }
function hapMtrTick(now){
  hapMtrRaf=0;
  const dt=hapMtrT?Math.min(0.5,Math.max(0,(now-hapMtrT)/1000)):0, k=Math.exp(-dt/0.3); hapMtrT=now;
  let busy=false;
  for(const key in HAP_MTR){
    const m=HAP_MTR[key], t=hapTiles[key]; if(!t) continue;
    m.disp=Math.max(m.now,m.disp*k); if(m.disp<0.005) m.disp=m.now;
    if(m.disp>=m.pk){ m.pk=m.disp; m.pkAt=now; }
    else if(now-m.pkAt>1500){ m.pk=Math.max(m.disp,m.pk*k); if(m.pk<0.005) m.pk=m.disp; }
    if(m.disp>m.now||m.pk>m.disp) busy=true;
    const bar=t.tile.querySelector('.mxlv');
    if(bar){ bar.firstChild.style.width=(m.disp*100).toFixed(1)+'%'; bar.lastChild.style.left=(m.pk*100).toFixed(1)+'%'; bar.lastChild.style.opacity=m.pk>0.01?'1':'0'; }
  }
  if(busy) hapMtrRaf=requestAnimationFrame(hapMtrTick);
}

function devInit(){
  const row=$('devShowRow'); if(row) row.hidden=(meta.platform!=='linux');
  const cb=$('cf-showdev'); if(cb&&!cb._wired){ cb._wired=true; cb.addEventListener('change',()=>{ devRender(); hapInit(); }); }
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
      <label title="End of usable travel, motor revs in the homed frame. Two-press capture fills this, or type. Err small: a wall just inside the physical stop is fine."><span>Travel min · rev</span><input type="number" step="0.001" id="devF-${n}-stopMinRev" value="${gv('stopMinRev',-0.07)}"></label>
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
      <label><span id="devLayPL-${n}">Throw · rev (blank = auto)</span><input type="number" step="0.001" id="devLayP-${n}" value="" placeholder="auto"></label>
      <button type="button" class="btn btn-sm btn-warn" id="devT-${n}-lay">Derive layout</button>
    </div>
    <div class="cfg-note" style="grid-column:1/-1">The device homes only from its own button (never with the rig): first press homes and rests limp, next press engages. Feel and geometry edits apply LIVE while the device is limp - Save, feel, adjust; if it is engaged when you save, they land on release. Teach by hand while limp: Capture travel is TWO presses - hold the lever at one end, press, hold it at the other end, press again (either order); then Derive layout and Save.</div>
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
      // Presets apply FEEL ONLY. The mechanism's identity - taught travel,
      // neutral, gate positions, homing direction, mirror - is never
      // clobbered by a preset (learned the hard way: a preset once wiped a
      // bench-taught travel). Geometry comes from teach/derive; a fresh
      // axis with no geometry yet takes the preset's as a starting point.
      const dd=cfgObj.drives[i];
      const KEEP=['stopMinRev','stopMaxRev','neutralRev','detents','homeDir','dir'];
      const old=dd.device||{};
      const fresh=JSON.parse(JSON.stringify(p));
      for(const k of KEEP) if(old[k]!==undefined) fresh[k]=old[k];
      dd.device=fresh;
      dd.mode='torque';
      devRender();   // rebuild cards so the curve editors show the preset
      const m=$('devMsg'); if(m) m.textContent='Preset feel applied to axis '+n+' (travel, neutral, gates, and homing kept) - Save to persist.';
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
    // Taught geometry (stops, neutral, gates) lives in the homed frame, so
    // changing either direction setting silently invalidates it - say so.
    const frameWarn=()=>{ const m=$('devMsg'); if(m) m.textContent=
      'Direction changed: taught travel, neutral and gates were captured in the previous homed frame - re-home, sweep and Capture travel again before trusting the feel.'; };
    wire('homeDir',(el,dv)=>{ dv.homeDir=+el.value; frameWarn(); });
    wire('dir',   (el,dv)=>{ dv.dir=+el.value; frameWarn(); });
    wire('detents',(el,dv)=>{ dv.detents=el.value.split(',').map(s=>+s.trim()).filter(v=>isFinite(v)); });
    // Teach: travel is a HARDWARE property, taught once - home the device
    // (it rests limp), then the TWO-PRESS capture (one press per end). Layout is a USE-CASE property, DERIVED from that
    // range by profile (never taught per position: an H-pattern's lateral
    // select is mechanical and invisible here, so every column shares one
    // fore/neutral/aft geometry; a selector splits the range into slots).
    const setF=(k,v)=>{ const f=$(`devF-${n}-${k}`); if(f) f.value=v; };
    const reflect=(dv)=>{ setF('stopMinRev',dv.stopMinRev); setF('stopMaxRev',dv.stopMaxRev);
      setF('neutralRev',dv.neutralRev);
      const df=$(`devF-${n}-detents`); if(df) df.value=(dv.detents||[]).join(', '); };
    const msg=(t)=>{ const m=$('devMsg'); if(m) m.textContent=t; };
    // Two-press travel capture: hold the lever at one end, press; hold it
    // at the other end, press again. Order does not matter (ends are
    // sorted). The button itself carries the state so the flow can never
    // be missed, and a completed capture flashes Derive layout as the
    // next step.
    const cap=$(`devT-${n}-cap`); if(cap) cap.onclick=()=>{
      const lv=devLive[n];
      if(lv===undefined){ msg('Capture travel: home the device first (its own Home button).'); return; }
      if(devCapA[n]===undefined){
        devCapA[n]=lv;
        cap.textContent='CAPTURE OTHER END';
        msg('End 1 captured at '+lv.toFixed(4)+' rev. Hold the lever at the OTHER end and press again.');
        return;
      }
      const a=devCapA[n], b=lv;
      delete devCapA[n]; cap.textContent='CAPTURE TRAVEL';
      if(Math.abs(b-a)<0.002){
        msg('Capture travel: both presses were at the same spot - hold the lever at the opposite end for the second press.');
        return; }
      const dd=cfgObj.drives[i]; dd.device=dd.device||{};
      dd.device.stopMinRev=+Math.min(a,b).toFixed(4);
      dd.device.stopMaxRev=+Math.max(a,b).toFixed(4);
      reflect(dd.device);
      msg('Travel captured: '+dd.device.stopMinRev+' … '+dd.device.stopMaxRev+' rev. Now press Derive layout, then Save.');
      const lay=$(`devT-${n}-lay`);
      if(lay){ lay.classList.add('flash'); setTimeout(()=>lay.classList.remove('flash'),4500); }
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
      if(layPL) layPL.textContent=(v==='sel')?'Slots · count':'Throw · rev (blank = auto)';
      // Mode switch resets the param to that mode's default so a slot count
      // never gets read as a throw (and vice versa): sel needs a count,
      // h defaults to auto (gates derived from the captured travel).
      if(layP){ layP.disabled=(v==='custom');
        if(v==='sel'&&!(+layP.value>=2)) layP.value=5;
        if(v==='h') layP.value=''; } };
    const lay=$(`devT-${n}-lay`); if(lay) lay.onclick=()=>{
      const dd=cfgObj.drives[i]; dd.device=dd.device||{};
      const dv=dd.device;
      const lo=+dv.stopMinRev, hi=+dv.stopMaxRev;
      if(!(hi>lo)){ msg('Derive layout: capture (or type) the travel first.'); return; }
      const mode=laySel?laySel.value:'h';
      if(mode==='custom'){ msg('Custom layout: type gates and neutral directly.'); return; }
      const centre=+((lo+hi)/2).toFixed(4);
      if(mode==='h'){
        // Auto (blank field): gates just inside each end of the CAPTURED
        // travel (7% inset, the selector's philosophy) - the sweep is the
        // truth, no magic constant. A typed throw overrides, clamped
        // inside the stops so a save can never be refused for geometry
        // the derive itself produced.
        const half=(hi-lo)/2, ov=+layP.value;
        const throwR=(layP.value!==''&&isFinite(ov)&&ov>0)
          ? Math.min(Math.abs(ov), half*0.98)
          : half-(hi-lo)*0.07;
        dv.neutralRev=centre;
        dv.detents=[+(centre-throwR).toFixed(4),+(centre+throwR).toFixed(4)];
        msg('H layout derived: neutral at centre, gates '+
            ((layP.value==='')?'just inside each stop':'at the set throw (kept inside the stops)')+
            ' - Save to persist.');
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
  const nx=$('devNcx'); if(nx) nx.textContent=(s.ncxRx?(s.ncxFrozen?'frozen (sim paused: packets arriving, values unchanged; effects idle)':'receiving'):'not receiving (effects idle, plain feel)')
    +(s.ncxRx&&(s.simGame||s.simCar)?` · ${[s.simGame,s.simCar].filter(Boolean).join(' / ')}`:'')
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
      row.appendChild(mkSel('tfr',[[0,'f/r -'],[1,'Front'],[-1,'Rear']],p.fr|0));
      row.appendChild(mkSel('tlr',[[0,'l/r -'],[1,'Left'],[-1,'Right']],p.lr|0));
    } else {
      row.appendChild(document.createElement('span'));
      row.appendChild(document.createElement('span'));
    }
    // Per-axis home. Hexapod use: direction checks with the pushrods
    // DISCONNECTED, and near-park single-leg re-homes. A full solo sweep
    // on a coupled platform binds (see HEXAPOD_SETUP.md).
    const hb=document.createElement('button'); hb.type='button';
    hb.className='thome key sm nolamp'; hb.textContent='Home';
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
fitHeader();   // before the first status too: a page that never connects still fits
if(setupCur==='test'){ buildTestAxes(); pollTestStatus(); }   // a stored tests page (setupPage at load left it to here)
window.__nullcatBooted=true;   // read by the safety net in index.html
