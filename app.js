'use strict';
/* Drive — personal navigation page. No server: everything runs in the browser.
   Data: OpenStreetMap (map, cameras via Overpass), Valhalla (routing), TomTom (traffic, speed limits, search; needs the user's key). */

const $ = id => document.getElementById(id);
const M_PER_MI = 1609.344, MPS_TO_MPH = 2.23694;
const CAM_NEAR_M = 30;      // a plate camera this close to the route line counts as "on the route"
const HAZ_NEAR_M = 50;      // same, for other hazards
const EXCL_HALF_M = 30;     // half-side of the square the router is told to avoid around a camera
const MAX_EXCL = 40;        // router limit: total avoid-area perimeter is capped
const MAX_ITER = 6;         // camera-avoiding re-route attempts
const VALHALLA = 'https://valhalla1.openstreetmap.de/route';
const OVERPASS = ['https://overpass-api.de/api/interpreter', 'https://overpass.kumi.systems/api/interpreter'];

const KIND = {
  alpr:     { label: 'License plate camera', color: '#ff3b4e', tone: 'red' },
  speedcam: { label: 'Speed camera',         color: '#ff9f1a', tone: 'red' },
  police:   { label: 'Police reported',      color: '#3d8bff', tone: 'red',   icon: '\u{1F693}' },
  damage:   { label: 'Damaged road',         color: '#ffb020', tone: 'amber', icon: '⚠️' },
  jam:      { label: 'Traffic slowdown',     color: '#ff3b4e', tone: 'amber' },
  roadwork: { label: 'Roadwork',             color: '#ffb020', tone: 'amber' },
  closure:  { label: 'Road closed',          color: '#b0b0b0', tone: 'amber' },
  accident: { label: 'Accident',             color: '#ff3b4e', tone: 'amber' },
  hazard:   { label: 'Hazard',               color: '#ffb020', tone: 'amber' },
};

// ---------- storage ----------
function load(key, fallback) {
  try { const v = localStorage.getItem(key); return v ? JSON.parse(v) : fallback; } catch (e) { return fallback; }
}
function save(key, val) { try { localStorage.setItem(key, JSON.stringify(val)); } catch (e) { /* private mode */ } }

const settings = Object.assign({ ttKey: '', margin: 5, voice: true, shortcut: 'Radios Off' }, load('gps.settings', {}));

const S = {
  pos: null, prevFix: null, follow: true, started: false,
  cams: [], camBox: null, camBusy: false,
  incidents: [], incAt: 0,
  pins: load('gps.pins', []),
  picks: [], sel: null, route: null, dest: null, planning: false,
  alerted: {}, limit: null, limitAt: null, limitBusy: false, over: false,
  offCount: 0, rerouteAt: 0, rerouting: false, saidMan: -1,
};
const SIM = (() => {
  const m = /[?&]sim=(-?[\d.]+),(-?[\d.]+)/.exec(location.search);
  return m ? { lat: +m[1], lon: +m[2], along: 0, timer: null } : null;
})();

// ---------- geometry ----------
const rad = d => d * Math.PI / 180;
function dist(a, b) {
  const dLat = rad(b[0] - a[0]), dLon = rad(b[1] - a[1]);
  const s = Math.sin(dLat / 2) ** 2 + Math.cos(rad(a[0])) * Math.cos(rad(b[0])) * Math.sin(dLon / 2) ** 2;
  return 2 * 6371000 * Math.asin(Math.sqrt(s));
}
function bearing(a, b) {
  const y = Math.sin(rad(b[1] - a[1])) * Math.cos(rad(b[0]));
  const x = Math.cos(rad(a[0])) * Math.sin(rad(b[0])) - Math.sin(rad(a[0])) * Math.cos(rad(b[0])) * Math.cos(rad(b[1] - a[1]));
  return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
}
function angDiff(a, b) { return Math.abs(((a - b + 540) % 360) - 180); }
// distance from p to segment a-b (metres) and position along it (0..1)
function segDist(p, a, b) {
  const kx = Math.cos(rad(p[0])) * 111320, ky = 110540;
  const ax = (a[1] - p[1]) * kx, ay = (a[0] - p[0]) * ky, bx = (b[1] - p[1]) * kx, by = (b[0] - p[0]) * ky;
  const dx = bx - ax, dy = by - ay, L = dx * dx + dy * dy;
  let t = L ? -(ax * dx + ay * dy) / L : 0;
  t = Math.max(0, Math.min(1, t));
  return { d: Math.hypot(ax + t * dx, ay + t * dy), t };
}
function decodePolyline(str, precision) {
  const f = 10 ** precision, out = [];
  let i = 0, lat = 0, lon = 0;
  while (i < str.length) {
    for (let k = 0; k < 2; k++) {
      let shift = 0, result = 0, b;
      do { b = str.charCodeAt(i++) - 63; result |= (b & 31) << shift; shift += 5; } while (b >= 32);
      const d = (result & 1) ? ~(result >> 1) : (result >> 1);
      if (k === 0) lat += d; else lon += d;
    }
    out.push([lat / f, lon / f]);
  }
  return out;
}
function locate(route, p) {
  let best = { d: Infinity, i: 0, t: 0 };
  const pts = route.pts;
  for (let i = 0; i < pts.length - 1; i++) {
    const r = segDist(p, pts[i], pts[i + 1]);
    if (r.d < best.d) best = { d: r.d, i, t: r.t };
  }
  best.along = route.cum[best.i] + best.t * (route.cum[best.i + 1] - route.cum[best.i]);
  return best;
}
function pointAt(route, along) {
  const cum = route.cum, pts = route.pts;
  let i = 0;
  while (i < cum.length - 2 && cum[i + 1] < along) i++;
  const seg = cum[i + 1] - cum[i], t = seg ? Math.min(1, Math.max(0, (along - cum[i]) / seg)) : 0;
  return { lat: pts[i][0] + t * (pts[i + 1][0] - pts[i][0]), lon: pts[i][1] + t * (pts[i + 1][1] - pts[i][1]), heading: bearing(pts[i], pts[i + 1]) };
}
// Which hazards sit within thr metres of the route line, and how far along it.
function projectHazards(route, list, thr) {
  if (!list.length) return [];
  const CELL = 0.002, PAD = 0.001, grid = new Map();
  for (const h of list) {
    const k = Math.floor(h.lat / CELL) + ',' + Math.floor(h.lon / CELL);
    if (!grid.has(k)) grid.set(k, []);
    grid.get(k).push(h);
  }
  const best = new Map(), pts = route.pts;
  for (let i = 0; i < pts.length - 1; i++) {
    const a = pts[i], b = pts[i + 1];
    const y0 = Math.floor((Math.min(a[0], b[0]) - PAD) / CELL), y1 = Math.floor((Math.max(a[0], b[0]) + PAD) / CELL);
    const x0 = Math.floor((Math.min(a[1], b[1]) - PAD) / CELL), x1 = Math.floor((Math.max(a[1], b[1]) + PAD) / CELL);
    for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
      const cell = grid.get(y + ',' + x);
      if (!cell) continue;
      for (const h of cell) {
        const r = segDist([h.lat, h.lon], a, b);
        if (r.d > thr) continue;
        const prev = best.get(h.id);
        if (!prev || r.d < prev.d) best.set(h.id, { h, d: r.d, along: route.cum[i] + r.t * (route.cum[i + 1] - route.cum[i]) });
      }
    }
  }
  return [...best.values()].sort((p, q) => p.along - q.along);
}

// Mappers sometimes add one pole holding two cameras as two entries; count cameras under 15 m apart once.
function dedupe(hits) {
  const out = [];
  for (const x of hits) if (!out.some(y => y.h.kind === x.h.kind && dist([y.h.lat, y.h.lon], [x.h.lat, x.h.lon]) < 15)) out.push(x);
  return out;
}

// ---------- formatting ----------
function fmtDist(m) {
  return m < 300 ? Math.max(50, Math.round(m * 3.28084 / 50) * 50) + ' ft' : (m / M_PER_MI).toFixed(1) + ' mi';
}
function sayDist(m) {
  return m < 300 ? Math.max(50, Math.round(m * 3.28084 / 50) * 50) + ' feet' : (m / M_PER_MI).toFixed(1) + ' miles';
}
function fmtMin(s) {
  const m = Math.round(s / 60);
  return m < 60 ? m + ' min' : Math.floor(m / 60) + ' h ' + (m % 60) + ' min';
}
function el(tag, props, kids) {
  const e = document.createElement(tag);
  Object.assign(e, props || {});
  for (const k of kids || []) e.append(k);
  return e;
}
let toastTimer;
function toast(msg, ms) {
  const t = $('toast');
  t.textContent = msg; t.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { t.hidden = true; }, ms || 2500);
}

// ---------- sound ----------
let actx = null;
function beep(freq, ms) {
  if (!actx) return;
  try {
    const o = actx.createOscillator(), g = actx.createGain();
    o.frequency.value = freq || 880; g.gain.value = 0.25;
    o.connect(g); g.connect(actx.destination);
    o.start(); o.stop(actx.currentTime + (ms || 160) / 1000);
  } catch (e) { /* ignore */ }
}
function speak(text) {
  if (!settings.voice || !('speechSynthesis' in window)) return;
  const u = new SpeechSynthesisUtterance(text);
  u.rate = 1.05;
  speechSynthesis.speak(u);
}

// ---------- map ----------
const map = L.map('map', { zoomControl: false, preferCanvas: true }).setView([39.5, -98.35], 4);
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19, attribution: '&copy; OpenStreetMap contributors',
}).addTo(map);
const incLayer = L.layerGroup().addTo(map);
const routeLayer = L.layerGroup().addTo(map);
const camLayer = L.layerGroup().addTo(map);
const pinLayer = L.layerGroup().addTo(map);
let meMarker = null;

map.on('dragstart', () => { S.follow = false; $('recenter').classList.add('off'); });
map.on('contextmenu', e => {            // long-press on the map
  const btn = el('button', { textContent: 'Route here' });
  const pop = L.popup().setLatLng(e.latlng).setContent(btn).openOn(map);
  btn.onclick = () => { map.closePopup(pop); planRoutes([e.latlng.lat, e.latlng.lng], 'Dropped point'); };
});

function drawMe() {
  const p = S.pos;
  if (!meMarker) {
    meMarker = L.marker([p.lat, p.lon], {
      icon: L.divIcon({ className: 'me', html: '<i></i>', iconSize: [26, 26], iconAnchor: [13, 13] }),
      interactive: false, zIndexOffset: 1000,
    }).addTo(map);
  }
  meMarker.setLatLng([p.lat, p.lon]);
  const i = meMarker.getElement() && meMarker.getElement().firstChild;
  if (i && p.heading != null) i.style.transform = 'rotate(' + p.heading + 'deg)';
  if (S.follow) map.setView([p.lat, p.lon], Math.max(map.getZoom(), 16), { animate: true });
}
function drawCams() {
  camLayer.clearLayers();
  for (const c of S.cams) {
    L.circleMarker([c.lat, c.lon], { radius: 5, color: '#000', weight: 1, fillColor: KIND[c.kind].color, fillOpacity: 0.9 })
      .bindPopup(c.note ? KIND[c.kind].label + ' (' + c.note + ')' : KIND[c.kind].label).addTo(camLayer);
  }
}
function drawIncidents() {
  incLayer.clearLayers();
  for (const h of S.incidents) {
    if (h.line && h.line.length > 1) L.polyline(h.line, { color: KIND[h.kind].color, weight: 6, opacity: 0.75 }).bindPopup(h.note || KIND[h.kind].label).addTo(incLayer);
    else L.circleMarker([h.lat, h.lon], { radius: 7, color: '#000', weight: 1, fillColor: KIND[h.kind].color, fillOpacity: 0.9 }).bindPopup(h.note || KIND[h.kind].label).addTo(incLayer);
  }
}
function drawPins() {
  pinLayer.clearLayers();
  for (const p of S.pins) {
    const btn = el('button', { textContent: 'Remove pin' });
    btn.onclick = () => { S.pins = S.pins.filter(x => x.id !== p.id); save('gps.pins', S.pins); drawPins(); refreshRouteHazards(); };
    const body = el('div', {}, [el('div', { textContent: KIND[p.kind].label + ' — ' + new Date(p.t).toLocaleDateString() }), btn]);
    L.marker([p.lat, p.lon], { icon: L.divIcon({ className: 'pin', html: KIND[p.kind].icon, iconSize: [28, 28], iconAnchor: [14, 14] }) })
      .bindPopup(body).addTo(pinLayer);
  }
}
function drawRoutes() {
  routeLayer.clearLayers();
  if (S.route) {
    L.polyline(S.route.pts, { color: '#fff', weight: 10, opacity: 0.9 }).addTo(routeLayer);
    L.polyline(S.route.pts, { color: S.route.color || '#3d8bff', weight: 6 }).addTo(routeLayer);
    return;
  }
  const order = S.picks.filter(p => p !== S.sel).concat(S.sel ? [S.sel] : []);
  for (const p of order) {
    const on = p === S.sel;
    L.polyline(p.route.pts, { color: p.color, weight: on ? 8 : 5, opacity: on ? 1 : 0.55 })
      .on('click', () => selectPick(p)).addTo(routeLayer);
  }
}

// ---------- data: cameras ----------
async function fetchCams(box) {           // box = [south, west, north, east]
  const b = box.map(v => v.toFixed(5)).join(',');
  const q = '[out:json][timeout:40];(node["man_made"="surveillance"]["surveillance:type"="ALPR"](' + b + ');node["highway"="speed_camera"](' + b + '););out body;';
  for (const url of OVERPASS) {
    try {
      const r = await fetch(url, { method: 'POST', body: 'data=' + encodeURIComponent(q), headers: { 'Content-Type': 'application/x-www-form-urlencoded' } });
      if (!r.ok) continue;
      const j = await r.json();
      return j.elements.filter(e => e.lat != null).map(e => {
        const t = e.tags || {};
        return { id: 'c' + e.id, kind: t.highway === 'speed_camera' ? 'speedcam' : 'alpr', lat: e.lat, lon: e.lon,
          note: t.manufacturer && t.manufacturer !== 'Unknown' ? t.manufacturer : (t.brand || t.operator || '') };
      });
    } catch (e) { /* try the next mirror */ }
  }
  throw new Error('Camera data unavailable');
}
function inBox(p, box, margin) {
  return box && p.lat > box[0] + margin && p.lat < box[2] - margin && p.lon > box[1] + margin && p.lon < box[3] - margin;
}
async function camsAround() {             // free-driving: keep cameras loaded around the car
  if (S.camBusy || S.planning || !S.pos || inBox(S.pos, S.camBox, 0.03)) return;
  S.camBusy = true;
  const box = [S.pos.lat - 0.12, S.pos.lon - 0.15, S.pos.lat + 0.12, S.pos.lon + 0.15];
  try { S.cams = await fetchCams(box); S.camBox = box; drawCams(); refreshRouteHazards(); }
  catch (e) { S.camBox = box; toast('Could not load camera data'); }   // do not hammer the server; retry after moving
  S.camBusy = false;
}

// ---------- data: TomTom ----------
const INC_KIND = { 1: 'accident', 3: 'hazard', 6: 'jam', 7: 'roadwork', 8: 'closure', 9: 'roadwork', 11: 'hazard', 14: 'hazard' };
async function fetchIncidents() {
  if (!settings.ttKey || !S.pos) return;
  S.incAt = Date.now();
  const p = S.pos;
  const fields = '{incidents{type,geometry{type,coordinates},properties{id,iconCategory,magnitudeOfDelay,events{description},from,to}}}';
  const url = 'https://api.tomtom.com/traffic/services/5/incidentDetails?key=' + encodeURIComponent(settings.ttKey) +
    '&bbox=' + [p.lon - 0.25, p.lat - 0.2, p.lon + 0.25, p.lat + 0.2].map(v => v.toFixed(4)).join(',') +
    '&fields=' + encodeURIComponent(fields) + '&language=en-US&timeValidityFilter=present';
  try {
    const r = await fetch(url);
    if (!r.ok) { if (r.status === 403) toast('TomTom key was rejected'); return; }
    const j = await r.json();
    const out = [];
    for (const inc of j.incidents || []) {
      const pr = inc.properties || {}, kind = INC_KIND[pr.iconCategory];
      if (!kind || !inc.geometry) continue;
      if (kind === 'jam' && !(pr.magnitudeOfDelay >= 2 && pr.magnitudeOfDelay <= 3)) continue;   // skip minor jams
      const c = inc.geometry.type === 'Point' ? [inc.geometry.coordinates] : inc.geometry.coordinates;
      if (!c || !c.length) continue;
      const desc = (pr.events || []).map(e => e.description).filter(Boolean).join(', ');
      out.push({ id: 'i' + pr.id, kind, lat: c[0][1], lon: c[0][0], line: c.map(x => [x[1], x[0]]),
        note: KIND[kind].label + (desc ? ': ' + desc : '') + (pr.from ? ' — ' + pr.from : '') });
    }
    S.incidents = out; drawIncidents(); refreshRouteHazards();
  } catch (e) { /* offline; try again next cycle */ }
}
async function fetchLimit() {
  const p = S.pos;
  if (!settings.ttKey || !p || S.limitBusy) return;
  if (S.limitAt && (Date.now() - S.limitAt.t < 15000 || dist([p.lat, p.lon], S.limitAt.p) < 250)) return;
  S.limitBusy = true; S.limitAt = { t: Date.now(), p: [p.lat, p.lon] };
  try {
    let url = 'https://api.tomtom.com/search/2/reverseGeocode/' + p.lat.toFixed(5) + ',' + p.lon.toFixed(5) +
      '.json?key=' + encodeURIComponent(settings.ttKey) + '&returnSpeedLimit=true&radius=60';
    if (p.heading != null) url += '&heading=' + Math.round(p.heading);
    const r = await fetch(url);
    if (r.ok) {
      const j = await r.json();
      const raw = j.addresses && j.addresses[0] && j.addresses[0].address && j.addresses[0].address.speedLimit;
      const m = raw && /([\d.]+)\s*(MPH|KPH)/i.exec(raw);
      S.limit = m ? Math.round(m[2].toUpperCase() === 'KPH' ? m[1] * 0.621371 : +m[1]) : null;
      $('limit').hidden = !S.limit;
      $('limitVal').textContent = S.limit || '';
    }
  } catch (e) { /* ignore */ }
  S.limitBusy = false;
}
async function search(q) {
  const p = S.pos;
  if (settings.ttKey) {
    let url = 'https://api.tomtom.com/search/2/search/' + encodeURIComponent(q) + '.json?key=' + encodeURIComponent(settings.ttKey) + '&limit=6';
    if (p) url += '&lat=' + p.lat.toFixed(4) + '&lon=' + p.lon.toFixed(4);
    const r = await fetch(url);
    if (r.ok) {
      const j = await r.json();
      return (j.results || []).map(x => ({
        name: (x.poi && x.poi.name ? x.poi.name + ', ' : '') + (x.address && x.address.freeformAddress || ''),
        lat: x.position.lat, lon: x.position.lon }));
    }
  }
  let url = 'https://nominatim.openstreetmap.org/search?format=jsonv2&limit=6&q=' + encodeURIComponent(q);
  if (p) url += '&viewbox=' + [p.lon - 1, p.lat + 1, p.lon + 1, p.lat - 1].map(v => v.toFixed(3)).join(',');
  const r = await fetch(url);
  if (!r.ok) throw new Error('Search failed');
  return (await r.json()).map(x => ({ name: x.display_name, lat: +x.lat, lon: +x.lon }));
}

// ---------- routing ----------
function square(c) {
  const dy = EXCL_HALF_M / 110540, dx = EXCL_HALF_M / (111320 * Math.cos(rad(c[0])));
  return [[c[1] - dx, c[0] - dy], [c[1] + dx, c[0] - dy], [c[1] + dx, c[0] + dy], [c[1] - dx, c[0] + dy], [c[1] - dx, c[0] - dy]];
}
async function vRoute(from, to, excl, alternates) {
  const body = { locations: [{ lat: from[0], lon: from[1] }, { lat: to[0], lon: to[1] }], costing: 'auto', units: 'miles' };
  if (alternates) body.alternates = alternates;
  if (excl && excl.length) body.exclude_polygons = excl.map(square);
  const r = await fetch(VALHALLA, { method: 'POST', body: JSON.stringify(body) });
  const j = await r.json();
  if (!r.ok || !j.trip) throw new Error(j.error || 'No route found');
  return j;
}
function buildRoute(trip, excl) {
  const leg = trip.legs[0], pts = decodePolyline(leg.shape, 6), cum = [0];
  for (let i = 1; i < pts.length; i++) cum.push(cum[i - 1] + dist(pts[i - 1], pts[i]));
  const route = {
    pts, cum, time: trip.summary.time, length: cum[cum.length - 1], excl: excl || [],
    maneuvers: leg.maneuvers.map(m => ({ text: m.instruction, say: m.verbal_pre_transition_instruction || m.instruction, idx: m.begin_shape_index })),
    onRoute: [],
  };
  const hit = dedupe(projectHazards(route, S.cams.filter(c => c.kind === 'alpr'), CAM_NEAR_M));
  route.camList = hit.map(x => x.h);
  route.cams = hit.length;
  return route;
}
const sleep = ms => new Promise(res => setTimeout(res, ms));

async function planRoutes(dest, name) {
  if (!S.pos) { toast('Waiting for your location…'); return; }
  if (S.planning) return;
  endNav(true);
  S.planning = true; S.dest = dest; S.picks = []; S.sel = null;
  const from = [S.pos.lat, S.pos.lon];
  let camsOk = true;
  try {
    sheetStatus('Loading known cameras…');
    const padLat = Math.max(0.04, Math.abs(from[0] - dest[0]) * 0.3), padLon = Math.max(0.05, Math.abs(from[1] - dest[1]) * 0.3);
    const box = [Math.min(from[0], dest[0]) - padLat, Math.min(from[1], dest[1]) - padLon, Math.max(from[0], dest[0]) + padLat, Math.max(from[1], dest[1]) + padLon];
    const covered = S.camBox && S.camBox[0] <= box[0] && S.camBox[1] <= box[1] && S.camBox[2] >= box[2] && S.camBox[3] >= box[3] && S.cams.length;
    if (!covered) {
      while (S.camBusy) await sleep(300);     // a background camera load is running; wait instead of a second request
      try { S.cams = await fetchCams(box); S.camBox = box; drawCams(); }
      catch (e) { await sleep(2500); try { S.cams = await fetchCams(box); S.camBox = box; drawCams(); } catch (e2) { camsOk = S.cams.length > 0; } }
    }

    sheetStatus('Finding the fastest route…');
    const j = await vRoute(from, dest, null, 2);
    const cands = [buildRoute(j.trip, [])].concat((j.alternates || []).map(a => buildRoute(a.trip, [])));
    const fastest = cands.reduce((a, b) => (b.time < a.time ? b : a));

    // Re-route around the cameras found so far, until a route has none or we run out of attempts.
    // Cameras right at the start or destination cannot be avoided; count them but never block them.
    const key = h => h.lat.toFixed(5) + ',' + h.lon.toFixed(5);
    const seen = new Set(), stuck = new Set();
    for (const c of S.cams) if (c.kind === 'alpr' && (dist(from, [c.lat, c.lon]) < 200 || dist(dest, [c.lat, c.lon]) < 200)) stuck.add(key(c));
    let cur = fastest, excl = [], exhausted = false, calls = 0;
    const tryRoute = async ex => {
      calls++;
      sheetStatus('Looking for routes with fewer cameras… (' + calls + ')');
      await sleep(1100);                    // be polite to the free routing server
      return buildRoute((await vRoute(from, dest, ex, 0)).trip, ex.slice());
    };
    while (camsOk && calls < MAX_ITER * 2 && cur.cams > 0) {
      const fresh = [];
      for (const h of cur.camList) { const k = key(h); if (!seen.has(k) && !stuck.has(k)) { seen.add(k); fresh.push(h); } }
      if (!fresh.length) break;
      if (excl.length + fresh.length > MAX_EXCL) { exhausted = true; break; }
      try { const ex = excl.concat(fresh.map(h => [h.lat, h.lon])); cur = await tryRoute(ex); excl = ex; cands.push(cur); continue; }
      catch (e) { /* the batch blocks every way through: test each camera on its own */ }
      let moved = false;
      for (const h of fresh) {
        if (calls >= MAX_ITER * 2) break;
        try { const ex = excl.concat([[h.lat, h.lon]]); cur = await tryRoute(ex); excl = ex; cands.push(cur); moved = true; break; }
        catch (e) { stuck.add(key(h)); }
      }
      if (!moved) break;
    }

    const zero = cands.filter(c => c.cams === 0).reduce((a, b) => (!a || b.time < a.time ? b : a), null);
    const fewer = cands.filter(c => c.cams < fastest.cams && c !== zero)
      .reduce((a, b) => (!a || b.time + 120 * b.cams < a.time + 120 * a.cams ? b : a), null);
    S.picks = [{ name: 'Fastest', color: '#3d8bff', route: fastest }];
    if (fewer) S.picks.push({ name: 'Fewer cams', color: '#ffb020', route: fewer });
    if (zero && zero !== fastest) S.picks.push({ name: 'Zero cams', color: '#2fd07a', route: zero });
    S.picks.forEach(p => { p.route.color = p.color; });
    S.sel = S.picks[0];

    let note = '';
    if (!camsOk) note = 'Camera data could not be loaded, so camera counts are unknown.';
    else if (fastest.cams === 0) note = 'The fastest route already passes no known plate cameras.';
    else if (!zero) note = 'No camera-free route found' + (exhausted ? ' (too many cameras to avoid)' : stuck.size ? ' (some cameras sit on the only way in or out)' : '') + '. Lowest found: ' + Math.min(...cands.map(c => c.cams)) + '.';
    S.planNote = note; S.destName = name || '';
    S.follow = false; $('recenter').classList.add('off');
    drawRoutes(); showPlan();
    map.fitBounds(L.latLngBounds(S.picks.flatMap(p => p.route.pts)), { paddingTopLeft: [20, 90], paddingBottomRight: [20, 260] });
  } catch (e) {
    $('sheet').hidden = true;
    toast(e.message || 'Could not find a route', 4000);
  }
  S.planning = false;
}

function sheetStatus(msg) {
  const s = $('sheet');
  s.replaceChildren(el('p', { className: 'status', textContent: msg }));
  s.hidden = false;
}
function selectPick(p) { S.sel = p; drawRoutes(); showPlan(); }
function showPlan() {
  const base = S.picks[0].route;
  const cards = el('div', { className: 'cards' }, S.picks.map(p => {
    const r = p.route, extra = Math.round((r.time - base.time) / 60);
    const card = el('button', { className: 'card' + (p === S.sel ? ' sel' : '') }, [
      el('span', { className: 'name', textContent: p.name }),
      el('span', { className: 'time', textContent: fmtMin(r.time) }),
      el('span', { className: 'meta', textContent: (r.length / M_PER_MI).toFixed(1) + ' mi' + (extra > 0 ? ' · +' + extra + ' min' : '') }),
      el('span', { className: 'cams', textContent: r.cams + (r.cams === 1 ? ' known cam' : ' known cams') }),
    ]);
    card.style.setProperty('--c', p.color);
    card.onclick = () => selectPick(p);
    return card;
  }));
  const cancel = el('button', { textContent: 'Cancel' });
  cancel.onclick = () => { S.picks = []; S.sel = null; drawRoutes(); $('sheet').hidden = true; };
  const go = el('button', { className: 'go', textContent: 'Go' });
  go.onclick = () => startNav(S.sel.route);
  const kids = [];
  if (S.destName) kids.push(el('p', { className: 'status', textContent: S.destName }));
  kids.push(cards);
  if (S.planNote) kids.push(el('p', { className: 'note', textContent: S.planNote }));
  kids.push(el('div', { className: 'actions' }, [cancel, go]));
  $('sheet').replaceChildren(...kids);
  $('sheet').hidden = false;
}

// ---------- navigation ----------
function refreshRouteHazards() {
  const r = S.route;
  if (!r) return;
  const alpr = S.cams.filter(c => c.kind === 'alpr');
  const rest = S.cams.filter(c => c.kind !== 'alpr').concat(S.incidents, S.pins);
  r.onRoute = dedupe(projectHazards(r, alpr, CAM_NEAR_M)).concat(projectHazards(r, rest, HAZ_NEAR_M)).sort((a, b) => a.along - b.along);
  r.cams = r.onRoute.filter(x => x.h.kind === 'alpr').length;
}
function startNav(route) {
  S.route = route; S.picks = []; S.sel = null;
  S.saidMan = -1; S.offCount = 0; S.follow = true; $('recenter').classList.remove('off');
  refreshRouteHazards(); drawRoutes(); showNav(route.length, route.time);
  speak('Starting route. ' + (route.maneuvers[0] ? route.maneuvers[0].say : ''));
  if (S.pos) { map.setView([S.pos.lat, S.pos.lon], 16); onFix(S.pos); }
  if (SIM) {
    SIM.along = 0;
    clearInterval(SIM.timer);
    SIM.timer = setInterval(() => {
      if (!S.route) { clearInterval(SIM.timer); return; }
      SIM.along += 30;
      const p = pointAt(S.route, SIM.along);
      onFix({ lat: p.lat, lon: p.lon, heading: p.heading, speed: 20, acc: 5 });
    }, 500);
  }
}
function showNav(remainM, remainS) {
  const eta = new Date(Date.now() + remainS * 1000).toLocaleTimeString([], { hour: 'numeric', minute: '2-digit' });
  const end = el('button', { textContent: 'End' });
  end.onclick = () => endNav();
  const cams = S.route.cams;
  $('sheet').replaceChildren(el('div', { className: 'navline' }, [
    el('div', {}, [el('b', { textContent: fmtMin(remainS) + ' · ' + (remainM / M_PER_MI).toFixed(1) + ' mi' }),
      el('span', { textContent: 'Arrive ' + eta + ' · ' + cams + (cams === 1 ? ' known cam' : ' known cams') + ' on route' })]),
    end,
  ]));
  $('sheet').hidden = false;
}
function endNav(quiet) {
  if (SIM) clearInterval(SIM.timer);
  const had = !!S.route;
  S.route = null;
  $('turn').hidden = true; $('alert').hidden = true;
  if (!S.planning || had) { drawRoutes(); }
  if (!quiet) $('sheet').hidden = true;
}
async function reroute() {
  if (S.rerouting || !S.route || !S.dest) return;
  S.rerouting = true; S.rerouteAt = Date.now();
  toast('Rerouting…');
  const old = S.route;
  try {
    const j = await vRoute([S.pos.lat, S.pos.lon], S.dest, old.excl, 0);
    if (S.route === old) {
      const r = buildRoute(j.trip, old.excl);
      r.color = old.color;
      S.route = r; S.saidMan = -1; S.offCount = 0;
      refreshRouteHazards(); drawRoutes();
    }
  } catch (e) { toast('Could not reroute'); }
  S.rerouting = false;
}

// ---------- alerts ----------
function raiseAlert(h, d) {
  const k = KIND[h.kind], a = $('alert');
  $('alertText').textContent = k.label;
  $('alertDist').textContent = fmtDist(d);
  a.className = k.tone === 'amber' ? 'amber' : '';
  a.hidden = false;
  const last = S.alerted[h.id];
  if (!last || Date.now() - last > 5 * 60000) {
    S.alerted[h.id] = Date.now();
    beep(k.tone === 'amber' ? 660 : 990, 200);
    speak(k.label + ' ahead, ' + sayDist(d));
  }
}
function checkAlerts(p, loc) {
  const reach = Math.min(1200, Math.max(400, (p.speed || 0) * 25));
  let near = null;
  if (S.route && loc) {
    for (const x of S.route.onRoute) {
      const ahead = x.along - loc.along;
      if (ahead < -15) continue;
      if (ahead > reach) break;
      near = { h: x.h, d: Math.max(0, ahead) }; break;
    }
  } else {
    const here = [p.lat, p.lon];
    for (const h of S.cams.concat(S.incidents, S.pins)) {
      if (Math.abs(h.lat - p.lat) > 0.012 || Math.abs(h.lon - p.lon) > 0.016) continue;
      const d = dist(here, [h.lat, h.lon]);
      if (d > reach) continue;
      if (p.heading != null && d > 40 && angDiff(bearing(here, [h.lat, h.lon]), p.heading) > 50) continue;
      if (p.heading == null && d > 150) continue;
      if (!near || d < near.d) near = { h, d };
    }
  }
  if (near) raiseAlert(near.h, near.d); else $('alert').hidden = true;
}

// ---------- position ----------
function onFix(p) {
  // derive heading from movement when the device does not report one
  if (p.heading == null || isNaN(p.heading)) {
    p.heading = S.pos ? S.pos.heading : null;
    if (S.prevFix && dist([S.prevFix.lat, S.prevFix.lon], [p.lat, p.lon]) > 8) p.heading = bearing([S.prevFix.lat, S.prevFix.lon], [p.lat, p.lon]);
  }
  if (!S.prevFix || dist([S.prevFix.lat, S.prevFix.lon], [p.lat, p.lon]) > 8) S.prevFix = { lat: p.lat, lon: p.lon };
  const first = !S.pos;
  S.pos = p;
  if (first) map.setView([p.lat, p.lon], 15);
  drawMe();

  const mph = Math.round((p.speed || 0) * MPS_TO_MPH);
  $('speedVal').textContent = mph;
  const over = !!S.limit && mph > S.limit + (+settings.margin || 0);
  $('speed').classList.toggle('over', over);
  if (over && !S.over) { beep(520, 350); speak('Over the limit'); }
  S.over = over;

  camsAround();
  if (Date.now() - S.incAt > 180000) fetchIncidents();
  fetchLimit();

  let loc = null;
  const r = S.route;
  if (r) {
    loc = locate(r, [p.lat, p.lon]);
    const remain = r.length - loc.along;
    if (remain < 35) { speak('You have arrived'); toast('Arrived'); endNav(); return; }
    if (loc.d > 70 && (p.acc || 0) < 60) S.offCount++; else S.offCount = 0;
    if (S.offCount >= 4 && Date.now() - S.rerouteAt > 20000) reroute();

    const next = r.maneuvers.findIndex(m => r.cum[m.idx] > loc.along + 5);
    if (next > 0) {
      const m = r.maneuvers[next], d = r.cum[m.idx] - loc.along;
      $('turnDist').textContent = fmtDist(d);
      $('turnText').textContent = next === r.maneuvers.length - 1 ? 'Destination' : m.text;
      $('turn').hidden = false;
      if (S.saidMan !== next && d < Math.max(200, (p.speed || 0) * 15)) { S.saidMan = next; speak('In ' + sayDist(d) + ', ' + m.say); }
    } else $('turn').hidden = true;
    showNav(remain, r.time * remain / r.length);
  }
  checkAlerts(p, loc);
}

let wakeLock = null;
async function keepAwake() {
  try { if ('wakeLock' in navigator && document.visibilityState === 'visible') wakeLock = await navigator.wakeLock.request('screen'); }
  catch (e) { /* not supported: the user must disable auto-lock */ }
}
document.addEventListener('visibilitychange', () => { if (S.started) keepAwake(); });

function start() {
  S.started = true;
  try { actx = new (window.AudioContext || window.webkitAudioContext)(); actx.resume(); } catch (e) { /* no sound */ }
  if ('speechSynthesis' in window) speechSynthesis.speak(new SpeechSynthesisUtterance(' '));   // unlock speech on iOS
  keepAwake();
  $('start').hidden = true;
  if (SIM) { onFix({ lat: SIM.lat, lon: SIM.lon, heading: null, speed: 0, acc: 5 }); return; }
  if (!('geolocation' in navigator)) { toast('This browser has no location support', 6000); return; }
  navigator.geolocation.watchPosition(
    g => onFix({ lat: g.coords.latitude, lon: g.coords.longitude, speed: g.coords.speed, heading: g.coords.speed > 1.5 ? g.coords.heading : null, acc: g.coords.accuracy }),
    err => toast(err.code === 1 ? 'Location permission was denied. Allow it in Settings > Safari > Location.' : 'Waiting for GPS signal…', 5000),
    { enableHighAccuracy: true, maximumAge: 1000, timeout: 20000 });
}

// ---------- UI wiring ----------
$('startBtn').onclick = start;
if (!settings.ttKey) $('startNote').textContent = 'Tip: add your TomTom key in Settings (the gear) for traffic and speed limits.';

$('recenter').onclick = () => {
  S.follow = true; $('recenter').classList.remove('off');
  if (S.pos) map.setView([S.pos.lat, S.pos.lon], Math.max(map.getZoom(), 16));
};
function dropPin(kind) {
  if (!S.pos) { toast('Waiting for your location…'); return; }
  S.pins.push({ id: 'p' + Date.now(), kind, lat: S.pos.lat, lon: S.pos.lon, t: Date.now() });
  S.alerted[S.pins[S.pins.length - 1].id] = Date.now();   // do not alert on the pin just dropped
  save('gps.pins', S.pins); drawPins(); refreshRouteHazards();
  beep(700, 120); toast(KIND[kind].label + ' pin saved');
}
$('pinPolice').onclick = () => dropPin('police');
$('pinDamage').onclick = () => dropPin('damage');

$('searchForm').onsubmit = async e => {
  e.preventDefault();
  const q = $('q').value.trim(), list = $('results');
  if (!q) return;
  $('q').blur();
  list.replaceChildren(el('li', { textContent: 'Searching…' })); list.hidden = false;
  try {
    const res = await search(q);
    if (!res.length) { list.replaceChildren(el('li', { textContent: 'Nothing found' })); return; }
    list.replaceChildren(...res.map(x => {
      const li = el('li', { textContent: x.name });
      li.onclick = () => { list.hidden = true; planRoutes([x.lat, x.lon], x.name); };
      return li;
    }));
  } catch (err) { list.replaceChildren(el('li', { textContent: 'Search failed. Check your connection.' })); }
};
$('q').addEventListener('input', () => { if (!$('q').value) $('results').hidden = true; });

$('gear').onclick = () => {
  $('setKey').value = settings.ttKey; $('setMargin').value = settings.margin;
  $('setVoice').checked = settings.voice; $('setShortcut').value = settings.shortcut;
  $('settings').showModal();
};
$('runShortcut').onclick = e => {
  e.preventDefault();
  location.href = 'shortcuts://run-shortcut?name=' + encodeURIComponent($('setShortcut').value.trim() || 'Radios Off');
};
$('clearPins').onclick = () => {
  if (!confirm('Delete all your police and road pins?')) return;
  S.pins = []; save('gps.pins', S.pins); drawPins(); refreshRouteHazards();
};
$('settingsForm').onsubmit = () => {
  const hadKey = settings.ttKey;
  settings.ttKey = $('setKey').value.trim();
  settings.margin = Math.max(0, +$('setMargin').value || 0);
  settings.voice = $('setVoice').checked;
  settings.shortcut = $('setShortcut').value.trim() || 'Radios Off';
  save('gps.settings', settings);
  if (settings.ttKey !== hadKey) { S.incAt = 0; S.limitAt = null; if (S.pos) { fetchIncidents(); fetchLimit(); } }
};

drawPins();
