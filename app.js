/* Kinetic Asset — 실시간 데모
 *
 * 파이프라인은 demo/webbench/bench.html 에서 이식했다 (MoveNet 2D → COCO→H36M →
 * MotionBERT-lite 3D, 링 버퍼 재생).  새로 붙인 것은 세 가지다:
 *   1. demo/metrics.py 의 생체 지표를 JS로 옮긴 실시간 계산
 *   2. 개인 기준선 등록(캘리브레이션) — 메모리에만 두고 저장하지 않는다
 *   3. 라이브 분포 vs 기준선 분포의 KL 이탈도 → 건강 자산 점수
 *
 * 서버로 아무것도 보내지 않는다. 전부 브라우저 안에서 끝난다.
 */

const $ = (id) => document.getElementById(id);
const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
const DEV = new URLSearchParams(location.search).has('dev');
if (DEV) document.body.classList.add('dev');

const log = (s) => {
  const el = $('log');
  el.textContent += s + '\n';
  el.scrollTop = 1e9;
  if (DEV) console.log(s);
};
const stat = (s) => { $('stat').textContent = s; };

/* ═══════════════════════════  지표  ═══════════════════════════
   H36M-17: 0 pelvis, 1 r_hip, 2 r_knee, 3 r_ank, 4 l_hip, 5 l_knee, 6 l_ank,
            7 spine, 8 thorax, 9 neck, 10 head, 11 l_sho .. 16 r_wri
   좌표는 root-relative 이고 +y 가 아래다. (demo/metrics.py 와 동일) */

const J = 17;
const PELVIS = 0, RHIP = 1, RKNEE = 2, RANK = 3, LHIP = 4, LKNEE = 5, LANK = 6;
const THORAX = 8;

const BONES = [[0,1],[1,2],[2,3],[0,4],[4,5],[5,6],[0,7],[7,8],[8,9],[9,10],
               [8,11],[11,12],[12,13],[8,14],[14,15],[15,16]];
const COCO_BONES = [[5,7],[7,9],[6,8],[8,10],[5,6],[5,11],[6,12],[11,12],
                    [11,13],[13,15],[12,14],[14,16],[0,5],[0,6]];

const g3 = (p, j) => [p[j*3], p[j*3+1], p[j*3+2]];

/** 관절 b 에서의 내각(도). */
function angleAt(a, b, c) {
  const v1 = [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
  const v2 = [c[0]-b[0], c[1]-b[1], c[2]-b[2]];
  const n1 = Math.hypot(...v1) + 1e-8, n2 = Math.hypot(...v2) + 1e-8;
  const cos = clamp((v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]) / (n1*n2), -1, 1);
  return Math.acos(cos) * 180 / Math.PI;
}

/** 몸통 길이(pelvis→thorax) — 모든 길이의 단위.
    오프라인에서는 클립 전체의 중앙값을 썼다. 실시간에는 미래를 못 보므로 EMA 로 대체한다. */
let torsoEma = 0;
function bodyScale(p) {
  const t = Math.hypot(p[THORAX*3] - p[0], p[THORAX*3+1] - p[1], p[THORAX*3+2] - p[2]);
  torsoEma = torsoEma ? torsoEma + (t - torsoEma) * 0.02 : t;
  return torsoEma + 1e-8;
}

/** 3D 한 프레임에서 지표를 뽑는다. p 는 수평 보정이 끝난 좌표여야 한다 —
    상체 기울기는 중력을 기준으로 재므로 보정 전 좌표를 쓰면 카메라 각도가 그대로 편향이 된다. */
function frameMetrics(p) {
  const s = bodyScale(p);

  const kneeL = 180 - angleAt(g3(p, LHIP), g3(p, LKNEE), g3(p, LANK));
  const kneeR = 180 - angleAt(g3(p, RHIP), g3(p, RKNEE), g3(p, RANK));

  // 상체 기울기: 몸통 벡터와 연직(up = -y)의 사잇각
  const tx = p[THORAX*3] - p[0], ty = p[THORAX*3+1] - p[1], tz = p[THORAX*3+2] - p[2];
  const tn = Math.hypot(tx, ty, tz) + 1e-8;
  const lean = Math.acos(clamp(-ty / tn, -1, 1)) * 180 / Math.PI;

  const footSep = Math.hypot(p[LANK*3] - p[RANK*3],
                             p[LANK*3+1] - p[RANK*3+1],
                             p[LANK*3+2] - p[RANK*3+2]) / s;

  return { knee_l: kneeL, knee_r: kneeR, knee_flex: (kneeL + kneeR) / 2,
           trunk_lean: lean, foot_sep: footSep };
}

/* 추적하는 세 지표 — 포스터 STEP 4 와 같다.
   range 는 히스토그램의 고정 지지집합. 세션이 달라도 같은 칸을 써야 KL 이 비교 가능하다. */
const NB = 24;
const METRICS = [
  { key: 'knee_flex',  name: '무릎 각도',   unit: '°', scale: 1,   dp: 0, range: [0, 120], pair: ['knee_l', 'knee_r'] },
  { key: 'trunk_lean', name: '상체 기울기', unit: '°', scale: 1,   dp: 0, range: [0, 90]  },
  { key: 'foot_sep',   name: '발 간격',     unit: '%', scale: 100, dp: 0, range: [0, 1.6], pre: '상체의 ' },
];

function histPush(h, v, lo, hi) {
  let i = Math.floor((v - lo) * NB / (hi - lo));
  if (i < 0) i = 0; if (i >= NB) i = NB - 1;
  h[i]++;
}
/** KL(P‖Q), add-eps 평활. 빈 칸이 있어도 발산하지 않게 한다. */
function kl(p, q, eps = 0.5) {
  let ps = 0, qs = 0;
  for (let i = 0; i < NB; i++) { ps += p[i] + eps; qs += q[i] + eps; }
  let d = 0;
  for (let i = 0; i < NB; i++) {
    const a = (p[i] + eps) / ps, b = (q[i] + eps) / qs;
    d += a * Math.log(a / b);
  }
  return d;
}

/* ═══════════════════════════  기준선  ═══════════════════════════
   오프라인 데모의 기준선은 NTU Day 01–05 를 모은 히스토그램이었다.
   실시간에는 그런 게 없으므로 사용자가 직접 등록한다. */

/* 기준선은 **저장하지 않는다.** 현장 데모에서는 앞사람 기준선으로 다음 관람객이
   채점되는 사고가 나고, 그 숫자는 그럴듯해 보여서 더 나쁘다.
   등록할 때마다 이전 것을 버리고 메모리에만 들고 있는다 — 새로고침하면 사라진다. */
const LS_KEY = 'kinetic-asset-baseline-v1';   // 예전 버전이 남긴 값을 지우기 위해서만 쓴다
const CAL_MS = 20000;                       // 기준선 등록 20초

let BASE = null;                            // {hists:{key:[..]}, kl0, at, n, win}
let calUntil = 0, calVals = null;
/* 라이브 창 = 등록 구간의 1/2.
   1/4(20초 등록 기준 5초)은 앉았다 일어나는 한 사이클 정도라 표본이 너무 작아
   창마다 이탈도가 크게 튀었다 (NTU 실측 점수 폭 p10~p90 = 12.1점 → 1/2 에서 0). */
const CAL_DIV = 2;

function newHists() {
  const o = {};
  for (const m of METRICS) o[m.key] = new Float64Array(NB);
  return o;
}

/* "오늘의 나" 는 측정 시작 이후 전체가 아니라 **최근 구간의 이동 창**이다.
   누적하면 ① 초반의 나쁜 프레임이 영원히 남고 ② 기준선(20초)보다 표본이 계속
   커져서 두 분포를 공정하게 비교할 수 없다. 창 길이는 kl0 을 잰 블록과 같게 맞춘다. */
let LIVE = newHists(), liveN = 0;
const liveQ = {};                       // key -> 최근 값들의 큐
for (const m of METRICS) liveQ[m.key] = [];

function histPop(h, v, lo, hi) {
  let i = Math.floor((v - lo) * NB / (hi - lo));
  if (i < 0) i = 0; if (i >= NB) i = NB - 1;
  h[i]--;
}
function resetLive() {
  LIVE = newHists(); liveN = 0;
  for (const m of METRICS) liveQ[m.key] = [];
}
/** 이동 창에 한 프레임을 넣고, 창을 넘치면 가장 오래된 값을 뺀다. */
function pushLive(fm) {
  const win = BASE ? BASE.win : 300;
  for (const m of METRICS) {
    const [lo, hi] = m.range, q = liveQ[m.key];
    histPush(LIVE[m.key], fm[m.key], lo, hi);
    q.push(fm[m.key]);
    if (q.length > win) histPop(LIVE[m.key], q.shift(), lo, hi);
  }
  if (liveN < win) liveN++;
}

/* 지표가 관여하는 관절. 이탈이 큰 지표를 몸의 어느 부위에서 재는지 3D 에 표시한다.
   (예전 데모의 RISK_GROUP 을 현재 지표 3종에 맞춰 다시 매핑) */
const METRIC_JOINTS = {
  knee_flex:  [RKNEE, LKNEE],
  trunk_lean: [PELVIS, 7 /* spine */, THORAX],
  foot_sep:   [RANK, LANK],
};
let RISK = {};                          // 관절 번호 -> 0 정상 · 1 관찰 · 2 개입

/* 판정은 **그 지표 자신의 정상 변동 대비 몇 배인가**로 한다.
   이전에는 전체 영점을 지표 수로 나눠 공통으로 썼는데, 지표마다 자연 변동이
   2배까지 다르다 (NTU 실측: 발 간격 0.773 · 무릎 0.609 · 상체 기울기 0.380).
   그래서 변동이 큰 발 간격에 늘 먼저 불이 켜졌다 — 정상인데도 21%가 오탐이었다.
   배수로 바꾸면 지표별로도 사람별로도 눈금이 저절로 맞는다. */
const LV_WARN = 2.0, LV_CRIT = 3.2;         // 정상 변동의 몇 배부터 관찰 / 개입
const ratioOf = (k, kl0m) => k / Math.max(kl0m, 1e-3);
const metricLevel = (k, kl0m) =>
  (ratioOf(k, kl0m) < LV_WARN ? 0 : ratioOf(k, kl0m) < LV_CRIT ? 1 : 2);

/* 총점도 같은 배수 위에서 매긴다. LAMBDA 는 등급 경계를 뱃지와 일치시키는 값 —
   배수 2.0 에서 80점(관찰), 3.2 에서 61점(개입)이 된다. */
const LAMBDA = 0.223;
const scoreOf = (tot, tot0) => clamp(100 * Math.exp(-LAMBDA * Math.max(0, ratioOf(tot, tot0) - 1)), 0, 100);
const statusOf = (s) => (s >= 80 ? ['good', '정상 범위'] : s >= 62 ? ['warn', '관찰 필요'] : ['crit', '개입 권고']);

/* 표시용 평활. 이탈도 자체는 그대로 두고 화면에 띄우는 값만 지수이동평균으로
   눌러준다. 0.5초마다 갱신되므로 계수 0.12 면 시정수 약 4초. 점수와 지표 뱃지가
   같은 값에서 나와야 하므로 지표별 이탈도까지 함께 평활한다. */
const SMOOTH = 0.12;
let klEma = null;
const resetEma = () => { klEma = null; };

function smoothKL(r) {
  if (!r) { resetEma(); return null; }
  if (!klEma) {
    klEma = { per: { ...r.per }, tot: r.tot };
  } else {
    for (const m of METRICS) klEma.per[m.key] += (r.per[m.key] - klEma.per[m.key]) * SMOOTH;
    klEma.tot += (r.tot - klEma.tot) * SMOOTH;
  }
  return klEma;
}

/** 지표별 이탈도와 합계. 기준선이 없으면 null. */
function liveKL() {
  if (!BASE || liveN < BASE.win) return null;
  const per = {};
  let tot = 0;
  for (const m of METRICS) {
    const k = kl(LIVE[m.key], BASE.hists[m.key]);
    per[m.key] = k; tot += k;
  }
  return { per, tot };
}

/* ═══════════════════════════  그리기  ═══════════════════════════ */

function fit(cv) {
  const dpr = Math.min(2, devicePixelRatio || 1);
  const w = cv.clientWidth, h = cv.clientHeight;
  if (!w || !h) return null;
  if (cv.width !== Math.round(w*dpr) || cv.height !== Math.round(h*dpr)) {
    cv.width = Math.round(w*dpr); cv.height = Math.round(h*dpr);
  }
  const g = cv.getContext('2d');
  g.setTransform(cv.width/w, 0, 0, cv.height/h, 0, 0);
  g.clearRect(0, 0, w, h);
  return [g, w, h];
}

/* 좌우 반전 여부는 반드시 한 곳에서만 정한다. 녹화 영상은 거울상이 아니므로 반전하지 않는다. */
const mirrorOn = () => $('mir').checked && !srcVideo;

/* ── 프레임 크롭 ──────────────────────────────────────────────────────
   iPhone 연속성 카메라는 거치 방향과 무관하게 macOS 가 항상 가로 프레임을 내놓는다.
   기기 포맷 목록에 세로가 없으므로 getUserMedia 로는 세로를 받을 수 없다 —
   세로가 필요하면 우리가 자르는 수밖에 없다.

   크롭 상자는 세션 시작 때 한 번 정하고 고정한다. 인물을 따라 움직이면 카메라
   기하가 프레임마다 변해서 관절 지표를 세션끼리 비교할 수 없게 된다 (센터 스테이지를
   끄라는 것과 같은 이유). */
let work = null, workCtx = null, crop = null;

function setupCrop(vw, vh) {
  let sw, sh;
  if ($('fit').value === 'portrait') {
    sh = vh; sw = Math.round(vh * 3 / 4);
    if (sw > vw) { sw = vw; sh = Math.round(vw * 4 / 3); }
  } else {
    sw = vw; sh = vh;
  }
  crop = { sx: Math.round((vw - sw) / 2), sy: Math.round((vh - sh) / 2), sw, sh };
  work = document.createElement('canvas');
  work.width = sw; work.height = sh;
  workCtx = work.getContext('2d');
  log(`입력 ${vw}x${vh} → 사용 ${sw}x${sh}` + (sw < vw || sh < vh ? ' (가운데 크롭)' : ''));
  return [sw, sh];
}

/** 카메라 프레임을 크롭해 작업 캔버스에 옮긴다. 2D 추정도 이 캔버스를 본다 —
    화면에 보이는 것과 모델이 보는 것이 같은 픽셀이어야 한다. */
function grabFrame() {
  workCtx.drawImage($('vid'), crop.sx, crop.sy, crop.sw, crop.sh, 0, 0, crop.sw, crop.sh);
  return work;
}

function draw2d(kp, vw, vh) {
  const f = fit($('c2d')); if (!f) return;
  const [g, w, h] = f;
  const s = Math.min(w/vw, h/vh), ox = (w - vw*s)/2, oy = (h - vh*s)/2;

  g.save();
  // 반전은 컨텍스트에 한 번만 건다 — 영상과 스켈레톤이 갈릴 수 없다
  if (mirrorOn()) { g.translate(w, 0); g.scale(-1, 1); }
  if (work) g.drawImage(work, ox, oy, vw*s, vh*s);

  if (kp) {
    const X = (i) => ox + kp[i][0]*s, Y = (i) => oy + kp[i][1]*s;
    g.lineWidth = 2.5; g.strokeStyle = '#7FB2F0';
    for (const [a, b] of COCO_BONES) {
      if (kp[a][2] < .25 || kp[b][2] < .25) continue;
      g.beginPath(); g.moveTo(X(a), Y(a)); g.lineTo(X(b), Y(b)); g.stroke();
    }
    g.fillStyle = '#E0F0FF';
    for (let i = 0; i < J; i++) {
      if (kp[i][2] < .25) continue;
      g.beginPath(); g.arc(X(i), Y(i), 3.2, 0, 7); g.fill();
    }
  }
  g.restore();
}

/* 시점 각도. 슬라이더로만 움직인다 — 자동 회전은 슬라이더를 더블클릭했을 때만. */
let yaw = 0.6, autoYaw = false;
const PITCH = 0.21;                 // 살짝 내려다본다. 0 이면 바닥 격자가 선 하나로 붕괴한다

function draw3d(p) {
  const f = fit($('c3d')); if (!f) return;
  const [g, w, h] = f;

  g.fillStyle = css('--sky-2');
  g.fillRect(0, 0, w, h);

  if (!p) {
    g.fillStyle = css('--ink-3');
    g.textAlign = 'center';
    g.font = '13.5px ' + css('--font');
    g.fillText(!running ? '측정을 시작하면 3D 자세가 나타납니다'
             : buf.length < F ? '자세를 읽는 중'
             : '3D 자세를 만드는 중', w/2, h/2);
    if (running && buf.length < F) {
      g.fillStyle = css('--rule-2'); g.fillRect(w/2-70, h/2+14, 140, 4);
      g.fillStyle = css('--measured'); g.fillRect(w/2-70, h/2+14, 140*buf.length/F, 4);
    }
    return;
  }

  if (autoYaw) {
    yaw += 0.004;
    const deg = Math.round((((yaw * 180 / Math.PI) + 180) % 360 + 360) % 360 - 180);
    $('yaw').value = deg;
  }
  const cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(PITCH), sp = Math.sin(PITCH);

  const lo = [1e9,1e9,1e9], hi = [-1e9,-1e9,-1e9];
  for (let j = 0; j < J; j++) for (let k = 0; k < 3; k++) {
    const v = p[j*3+k];
    if (v < lo[k]) lo[k] = v;
    if (v > hi[k]) hi[k] = v;
  }
  const mid = [(lo[0]+hi[0])/2, (lo[1]+hi[1])/2, (lo[2]+hi[2])/2];
  const span = Math.max(hi[1]-lo[1], hi[0]-lo[0], 1e-6);
  const s = Math.min(w, h) * 0.72 / span;

  const proj = (x, y, z) => {
    const rx = x*cy + z*sy, rz = -x*sy + z*cy;
    return [w/2 + rx*s, h/2 + (y*cp - rz*sp)*s, rz];
  };

  // 바닥 격자 — 회전을 눈으로 따라가려면 고정된 지면 기준이 필요하다
  const gy = hi[1] - mid[1];
  const R = span * 0.62, STEP = R / 2;
  g.lineWidth = 1; g.strokeStyle = css('--rule');
  for (let i = -2; i <= 2; i++) {
    const t = i * STEP;
    for (const [A, B] of [[[t, gy, -R], [t, gy, R]], [[-R, gy, t], [R, gy, t]]]) {
      const a = proj(...A), b = proj(...B);
      g.beginPath(); g.moveTo(a[0], a[1]); g.lineTo(b[0], b[1]); g.stroke();
    }
  }

  const P = [];
  for (let j = 0; j < J; j++) {
    P.push(proj(p[j*3]-mid[0], p[j*3+1]-mid[1], p[j*3+2]-mid[2]));
  }

  // 이탈이 큰 지표가 관여하는 관절을 상태색으로 짚어준다.
  // 색만으로 알리지 않는다 — 옆 지표 패널에 「관찰 필요 / 개입 권고」 글자가 같이 뜬다.
  const LVC = [null, css('--warn'), css('--crit')];
  const hexA = (hex, a) => {
    const n = parseInt(hex.slice(1), 16);
    return `rgba(${n >> 16 & 255}, ${n >> 8 & 255}, ${n & 255}, ${a.toFixed(3)})`;
  };

  // 깊이 순으로 그려야 앞뒤가 뒤집히지 않는다
  const order = BONES.map((b, i) => [i, (P[b[0]][2] + P[b[1]][2])/2]).sort((a, b) => b[1] - a[1]);
  for (const [i] of order) {
    const [a, b] = BONES[i];
    const t = 1 - clamp((P[a][2]+P[b][2])/2 / span + .5, 0, 1);   // 앞쪽일수록 1
    const lv = Math.min(RISK[a] || 0, RISK[b] || 0);              // 양끝이 다 위험할 때만
    g.lineWidth = 2.2 + t*2.4 + (lv ? 1.2 : 0);
    g.strokeStyle = lv ? hexA(LVC[lv], .45 + t*.55)
                       : `rgba(26, 79, 160, ${(.30 + t*.70).toFixed(3)})`;
    g.lineCap = 'round';
    g.beginPath(); g.moveTo(P[a][0], P[a][1]); g.lineTo(P[b][0], P[b][1]); g.stroke();
  }
  // 마디는 흰 링을 둘러 겹쳐도 서로 분리돼 보이게 한다
  P.forEach((q, j) => {
    const lv = RISK[j] || 0;
    const r = lv ? 5.2 : 3.4;
    g.beginPath(); g.arc(q[0], q[1], r, 0, 7);
    g.fillStyle = css('--surface'); g.fill();
    g.beginPath(); g.arc(q[0], q[1], r - 1.2, 0, 7);
    g.fillStyle = lv ? LVC[lv] : css('--navy'); g.fill();
    if (lv === 2) {                       // 개입 단계는 바깥 링을 하나 더 둘러 눈에 띄게
      g.beginPath(); g.arc(q[0], q[1], r + 2.6, 0, 7);
      g.strokeStyle = hexA(LVC[2], .45); g.lineWidth = 1.6; g.stroke();
    }
  });
}

/* ─── 실시간 지표 패널 ─── */

const TRACE_N = 140;
const traces = {};                       // key -> {vals:[], valsR:[]}
for (const m of METRICS) traces[m.key] = { a: [], b: [] };

// 각 지표를 "어떻게 재는가" — 아이콘이 측정 방식을 그대로 보여준다
const GLYPH = {
  knee_flex: `<svg width="26" height="26" viewBox="0 0 26 26" fill="none" aria-hidden="true">
    <path d="M8 4 L15 12 L8 21" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/>
    <path d="M15 12 m-5.5 -1 a5.5 5.5 0 0 0 0 4.6" stroke="currentColor" stroke-width="1.2" opacity=".55"/>
    <circle cx="8" cy="4" r="1.9" fill="currentColor"/><circle cx="15" cy="12" r="2.2" fill="currentColor"/><circle cx="8" cy="21" r="1.9" fill="currentColor"/></svg>`,
  trunk_lean: `<svg width="26" height="26" viewBox="0 0 26 26" fill="none" aria-hidden="true">
    <path d="M11 22 L11 4" stroke="currentColor" stroke-width="1.2" stroke-dasharray="2.5 2.5" opacity=".55"/>
    <path d="M11 22 L18 5" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"/>
    <circle cx="11" cy="22" r="2.1" fill="currentColor"/><circle cx="18" cy="5" r="2.1" fill="currentColor"/></svg>`,
  foot_sep: `<svg width="26" height="26" viewBox="0 0 26 26" fill="none" aria-hidden="true">
    <circle cx="6" cy="9" r="2.1" fill="currentColor"/><circle cx="20" cy="9" r="2.1" fill="currentColor"/>
    <path d="M6 16 L20 16" stroke="currentColor" stroke-width="1.5" stroke-linecap="round"/>
    <path d="M6 13.2 L6 18.8 M20 13.2 L20 18.8" stroke="currentColor" stroke-width="1.5" stroke-linecap="round"/></svg>`,
};

function buildMetricPanel() {
  $('metrics').innerHTML = METRICS.map((m) => `
    <div class="metric">
      <div class="top">
        <span class="nm">${GLYPH[m.key] || ''}${m.name}<span class="lvl" id="s_${m.key}"></span></span>
        <span class="val empty" id="v_${m.key}">측정 대기</span>
      </div>
      <canvas id="t_${m.key}"></canvas>
    </div>`).join('');
}

function drawTrace(m) {
  const f = fit($('t_' + m.key)); if (!f) return;
  const [g, w, h] = f;
  const T = traces[m.key];
  const [lo, hi] = m.range;
  const X = (i) => (i / (TRACE_N - 1)) * w;
  const Y = (v) => h - 3 - (clamp(v, lo, hi) - lo) / (hi - lo) * (h - 6);

  g.strokeStyle = css('--grid'); g.lineWidth = 1;
  g.beginPath(); g.moveTo(0, h - .5); g.lineTo(w, h - .5); g.stroke();

  const line = (arr, dash) => {
    if (arr.length < 2) return;
    g.setLineDash(dash ? [4, 3] : []);
    g.strokeStyle = css('--measured'); g.lineWidth = 2; g.lineJoin = 'round';
    g.beginPath();
    const off = TRACE_N - arr.length;
    arr.forEach((v, i) => (i ? g.lineTo(X(off + i), Y(v)) : g.moveTo(X(off + i), Y(v))));
    g.stroke();
    g.setLineDash([]);
  };
  if (m.pair) { line(T.a, false); line(T.b, true); } else line(T.a, false);
}

function updateMetricPanel(fm) {
  for (const m of METRICS) {
    const T = traces[m.key];
    if (m.pair) {
      T.a.push(fm[m.pair[0]]); T.b.push(fm[m.pair[1]]);
      if (T.a.length > TRACE_N) { T.a.shift(); T.b.shift(); }
    } else {
      T.a.push(fm[m.key]);
      if (T.a.length > TRACE_N) T.a.shift();
    }
    // 값 읽기 — L/R 은 색이 아니라 실선/점선으로 구분한다
    const fmt = (v) => (v * m.scale).toFixed(m.dp);
    const el = $('v_' + m.key);
    el.className = 'val';
    el.innerHTML = m.pair
      ? `<i class="ls"></i>${fmt(fm[m.pair[0]])}<small>${m.unit}</small>` +
        `&nbsp;&nbsp;<i class="ls dash"></i>${fmt(fm[m.pair[1]])}<small>${m.unit}</small>`
      : `${m.pre ? `<small>${m.pre}</small>` : ''}${fmt(fm[m.key])}<small>${m.unit}</small>`;
    drawTrace(m);
  }
}

/* ─── 분포 비교 ─── */

function buildDists() {
  $('dists').innerHTML = METRICS.map((m) => `
    <div class="dist">
      <div class="t">${m.name}</div>
      <canvas id="d_${m.key}"></canvas>
      <div class="kl" id="k_${m.key}">–</div>
    </div>`).join('');
}

function drawDist(m, klv) {
  const f = fit($('d_' + m.key)); if (!f) return;
  const [g, w, h] = f;
  const pad = { l: 2, r: 2, t: 6, b: 16 };
  const iw = w - pad.l - pad.r, ih = h - pad.t - pad.b;
  const bw = iw / NB;

  // 플롯 영역을 옅게 깔아야 빈 차트가 "고장난 여백"이 아니라 "아직 비어 있음"으로 읽힌다
  g.fillStyle = css('--sky-2');
  g.fillRect(pad.l, pad.t, iw, ih);

  const norm = (a) => { let s = 0; for (let i = 0; i < NB; i++) s += a[i]; return s ? a.map((v) => v/s) : null; };
  const B = BASE ? norm(BASE.hists[m.key]) : null;
  const L = liveN ? norm(LIVE[m.key]) : null;

  if (!B && !L) {
    g.fillStyle = css('--ink-3'); g.textAlign = 'center';
    g.font = '12.5px ' + css('--font');
    g.fillText('기준선 등록 전', pad.l + iw/2, pad.t + ih/2 + 4);
  }

  let mx = 0;
  for (const a of [B, L]) if (a) for (const v of a) if (v > mx) mx = v;
  if (!mx) mx = 1;

  // 기준선은 채운 무채색 면, 오늘은 그 위의 선 — 겹쳐도 둘 다 읽힌다
  if (B) {
    g.fillStyle = 'rgba(111,114,128,.30)';
    for (let i = 0; i < NB; i++) {
      const bh = B[i] / mx * ih;
      if (bh > .4) g.fillRect(pad.l + i*bw + 1, pad.t + ih - bh, Math.max(1, bw - 2), bh);
    }
  }
  if (L) {
    g.strokeStyle = css('--measured'); g.lineWidth = 2; g.lineJoin = 'round';
    g.beginPath();
    for (let i = 0; i < NB; i++) {
      const x = pad.l + i*bw + bw/2, y = pad.t + ih - L[i]/mx*ih;
      i ? g.lineTo(x, y) : g.moveTo(x, y);
    }
    g.stroke();
  }

  g.strokeStyle = css('--rule'); g.lineWidth = 1;
  g.beginPath(); g.moveTo(pad.l, pad.t + ih + .5); g.lineTo(w - pad.r, pad.t + ih + .5); g.stroke();

  g.fillStyle = css('--ink-3'); g.font = '11px ' + css('--mono');
  g.textAlign = 'left';  g.fillText((m.range[0]*m.scale).toFixed(0) + m.unit, pad.l, h - 3);
  g.textAlign = 'right'; g.fillText((m.range[1]*m.scale).toFixed(0) + m.unit, w - pad.r, h - 3);

  // 차트 안에 이미 '기준선 등록 전'이 있다 — 같은 말을 두 번 쓰지 않는다
  $('k_' + m.key).textContent =
    klv == null ? '' : '평소의 ' + ratioOf(klv, BASE.kl0[m.key]).toFixed(1) + '배';
}

function refreshCompare() {
  const r = smoothKL(liveKL());

  RISK = {};
  if (DEV && window.__riskPin) {
    RISK = window.__riskPin;
  } else if (r) {
    for (const m of METRICS) {
      const lv = metricLevel(r.per[m.key], BASE.kl0[m.key]);
      for (const j of METRIC_JOINTS[m.key]) RISK[j] = Math.max(RISK[j] || 0, lv);
      const dot = $('s_' + m.key);
      if (dot) {
        dot.className = 'lvl' + (lv ? ' on lv' + lv : '');
        dot.textContent = lv === 2 ? '개입 권고' : lv === 1 ? '관찰 필요' : '';
      }
    }
  } else {
    for (const m of METRICS) {
      const dot = $('s_' + m.key);
      if (dot) { dot.className = 'lvl'; dot.textContent = ''; }
    }
  }
  for (const m of METRICS) drawDist(m, r ? r.per[m.key] : null);

  const pill = $('scoreP'), st = $('scoreT'), v = $('scoreV');
  if (calUntil) {
    const left = Math.max(0, Math.ceil((calUntil - performance.now()) / 1000));
    v.className = 'v'; v.innerHTML = left + '<small>초</small>';
    pill.className = 'pill warn'; pill.style.color = '';
    st.textContent = '기준선 등록 중';
    $('scoreS').textContent = '평소처럼 움직여 주세요';
    return;
  }
  const stamp = (t) => new Date(t).toLocaleString('ko-KR',
    { month: 'long', day: 'numeric', hour: '2-digit', minute: '2-digit' });

  $('cal').textContent = BASE ? '기준선 다시 등록' : '기준선 등록';

  if (!r) {
    v.className = 'v empty';
    v.style.whiteSpace = 'pre-line';
    pill.className = 'pill'; pill.style.color = 'var(--ink-3)';

    if (!BASE) {
      v.textContent = '기준선을 등록하면\n점수가 나타납니다';
      st.textContent = '기준선 없음';
      $('scoreS').textContent = '측정을 시작한 뒤 [기준선 등록]을 누르세요';
    } else if (running) {
      v.textContent = '움직임을\n모으는 중입니다';
      st.textContent = '분석 중';
      $('scoreS').textContent = `${stamp(BASE.at)} 등록한 기준선과 비교 중`;
    } else {
      v.textContent = '측정을 시작하면\n점수가 나타납니다';
      st.textContent = '측정 대기';
      $('scoreS').textContent = `${stamp(BASE.at)} 등록한 기준선 사용`;
    }
    return;
  }
  pill.style.color = '';
  const sc = scoreOf(r.tot, BASE.tot0);
  const [cl, label] = statusOf(sc);
  v.className = 'v'; v.style.whiteSpace = '';
  v.innerHTML = sc.toFixed(1) + '<small>점</small>';
  pill.className = 'pill ' + cl;
  st.textContent = label;
  $('scoreS').textContent =
    `${stamp(BASE.at)} 등록한 기준선 대비 · 평소 변동의 ${ratioOf(r.tot, BASE.tot0).toFixed(1)}배`;
}

/* ═══════════════════════════  파이프라인  ═══════════════════════════ */

let F = 100;
let running = false, buf = [], nLift = 0, lastLift = 0, lifting = false, e2e = NaN;
let stream = null, sess = null, det = null, srcVideo = false;

const RING = 512;
const ring3d = new Float32Array(RING * J * 3);
const ringG = new Int32Array(RING).fill(-1);
const ringT = new Float64Array(RING);
const held = new Float32Array(J * 3);
let gidx = 0, lastShownG = -1, hasHeld = false, dispG = -1, maxCovG = -1;

const S = (n) => ({ v: [], n,
  push(x) { if (!isFinite(x)) return; this.v.push(x); if (this.v.length > this.n) this.v.shift(); },
  p(q) { if (!this.v.length) return NaN; const a = [...this.v].sort((x, y) => x - y);
         return a[Math.min(a.length-1, Math.floor(q*a.length))]; } });
const stFps = S(60), st2d = S(60), stLift = S(20), cov3d = S(90);

/* COCO-17 → H36M-17 (MotionBERT lib/data/dataset_action.py 와 동일) */
function coco2h36m(k) {
  const y = Array.from({ length: J }, () => [0, 0, 0]);
  const mid = (a, b) => [(a[0]+b[0])/2, (a[1]+b[1])/2, Math.min(a[2], b[2])];
  y[0] = mid(k[11], k[12]);
  y[1] = k[12].slice(); y[2] = k[14].slice(); y[3] = k[16].slice();
  y[4] = k[11].slice(); y[5] = k[13].slice(); y[6] = k[15].slice();
  y[8] = mid(k[5], k[6]);
  y[7] = mid(y[0], y[8]);
  y[9] = k[0].slice();
  y[10] = mid(k[1], k[2]);
  y[11] = k[5].slice(); y[12] = k[7].slice(); y[13] = k[9].slice();
  y[14] = k[6].slice(); y[15] = k[8].slice(); y[16] = k[10].slice();
  return y;
}

/* crop_scale (MotionBERT lib/utils/utils_data.py) — 윈도우 단위로 [-1,1] 정규화 */
function cropScale(win, out) {
  let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity, n = 0;
  for (const fr of win) for (const j of fr) {
    if (j[2] === 0) continue;
    n++;
    if (j[0] < xmin) xmin = j[0]; if (j[0] > xmax) xmax = j[0];
    if (j[1] < ymin) ymin = j[1]; if (j[1] > ymax) ymax = j[1];
  }
  if (n < 4) { out.fill(0); return out; }
  const scale = Math.max(xmax - xmin, ymax - ymin);
  if (scale === 0) { out.fill(0); return out; }
  const xs = (xmin + xmax - scale) / 2, ys = (ymin + ymax - scale) / 2;
  const cl = (v) => (v < -1 ? -1 : v > 1 ? 1 : v);
  let p = 0;
  for (const fr of win) for (const j of fr) {
    out[p++] = cl(((j[0] - xs) / scale - 0.5) * 2);
    out[p++] = cl(((j[1] - ys) / scale - 0.5) * 2);
    out[p++] = cl(j[2]);
  }
  return out;
}

/* 카메라 좌표계 기울기 보정.
   MotionBERT 출력은 카메라 좌표계다. 아이폰을 높이 두고 내려다보면 스켈레톤이 통째로
   기운다. IMU 가 없으니 피사체에서 중력을 추정한다 — 시간 평균한 몸통 방향을 위쪽으로
   보고 그 벡터를 월드 up 으로 보내는 회전(Rodrigues)을 적용한다.
   그리기 전용이 아니다: 상체 기울기는 이 좌표에서 계산해야 한다. */
const levelBuf = new Float32Array(J * 3);
let upEma = null;
function levelPose(p) {
  let tx = p[THORAX*3] - p[0], ty = p[THORAX*3+1] - p[1], tz = p[THORAX*3+2] - p[2];
  const n = Math.hypot(tx, ty, tz) || 1; tx /= n; ty /= n; tz /= n;
  if (!upEma) upEma = [tx, ty, tz];
  else {
    const k = 0.02;
    upEma = [upEma[0] + (tx-upEma[0])*k, upEma[1] + (ty-upEma[1])*k, upEma[2] + (tz-upEma[2])*k];
    const m = Math.hypot(...upEma) || 1;
    upEma = upEma.map((v) => v/m);
  }
  const a = upEma, b = [0, -1, 0];
  const v = [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
  const c = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
  const s2 = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
  if (s2 < 1e-10) return p;
  const k = (1 - c) / s2;
  const R = [
    1 + k*(-v[1]*v[1]-v[2]*v[2]), -v[2] + k*v[0]*v[1],           v[1] + k*v[0]*v[2],
    v[2] + k*v[0]*v[1],           1 + k*(-v[0]*v[0]-v[2]*v[2]), -v[0] + k*v[1]*v[2],
    -v[1] + k*v[0]*v[2],          v[0] + k*v[1]*v[2],            1 + k*(-v[0]*v[0]-v[1]*v[1]),
  ];
  for (let j = 0; j < J; j++) {
    const x = p[j*3], y = p[j*3+1], z = p[j*3+2];
    levelBuf[j*3]   = R[0]*x + R[1]*y + R[2]*z;
    levelBuf[j*3+1] = R[3]*x + R[4]*y + R[5]*z;
    levelBuf[j*3+2] = R[6]*x + R[7]*y + R[8]*z;
  }
  return levelBuf;
}

async function listCams(ask) {
  try {
    if (ask) {
      const s = await navigator.mediaDevices.getUserMedia({ video: true });
      s.getTracks().forEach((t) => t.stop());
    }
    const ds = (await navigator.mediaDevices.enumerateDevices()).filter((d) => d.kind === 'videoinput');
    const sel = $('cam'), cur = sel.value;
    sel.innerHTML = '<option value="">기본 카메라</option>' +
      ds.map((d) => `<option value="${d.deviceId}">${d.label || '카메라'}</option>`).join('');
    sel.value = cur;
    // 연속성 카메라가 목록에 있으면 자동 선택 — 「데스크뷰」는 초광각이라 피한다
    const ip = ds.find((d) => /iphone/i.test(d.label) && !/desk/i.test(d.label));
    if (ip && !cur) sel.value = ip.deviceId;
    log(`카메라 ${ds.length}개` + (ip ? ' (iPhone 자동 선택)' : ''));
  } catch (e) { log('카메라 목록 실패: ' + (e.message || e)); }
}

async function openSource() {
  const vid = $('vid');

  if (srcVideo) {
    vid.srcObject = null;
    vid.src = 'assets/sample.mp4';
    vid.loop = true;
    try {
      await vid.play();
    } catch (e) {
      throw new Error('샘플 영상을 열 수 없다 — assets/sample.mp4 를 넣었는지 확인');
    }
    log(`샘플 영상 재생 — ${vid.videoWidth}x${vid.videoHeight}`);
    return [vid.videoWidth, vid.videoHeight];
  }

  const [rw, rh] = $('res').value.split('x').map(Number);
  const c = { width: { ideal: rw }, height: { ideal: rh }, frameRate: { ideal: 30 } };
  if ($('cam').value) c.deviceId = { exact: $('cam').value };
  stream = await navigator.mediaDevices.getUserMedia({ video: c, audio: false });
  vid.src = '';
  vid.srcObject = stream;
  await vid.play();
  const tr = stream.getVideoTracks()[0];
  log(`카메라: ${tr.label || '(라벨 없음)'} — ${vid.videoWidth}x${vid.videoHeight}`);
  // 기기가 실제로 어떤 해상도를 내줄 수 있는지 기록해 둔다.
  // 연속성 카메라에 세로 포맷이 없다는 걸 추측이 아니라 로그로 확인하려는 것.
  const caps = tr.getCapabilities ? tr.getCapabilities() : {};
  if (caps.width && caps.height) {
    log(`  지원 범위: 가로 ${caps.width.min}–${caps.width.max}, 세로 ${caps.height.min}–${caps.height.max}`);
  }
  if (/iphone/i.test(tr.label)) log('연속성 카메라 — 센터 스테이지가 꺼져 있어야 지표가 유효하다');
  await listCams(false);
  return [vid.videoWidth, vid.videoHeight];
}

function stopRun() {
  running = false;
  if (stream) { stream.getTracks().forEach((t) => t.stop()); stream = null; }
  $('vid').srcObject = null;
  $('start').disabled = false; $('stop').disabled = true;
  $('idle2d').hidden = false;
  document.querySelectorAll('.stage').forEach((s) => s.classList.remove('live'));
  work = null; crop = null;
  stat('');
  draw2d(null, 1, 1);
  draw3d(null);
}

async function run() {
  $('start').disabled = true;
  stat('준비 중');
  try {
    const [rawW, rawH] = await openSource();
    const [vw, vh] = setupCrop(rawW, rawH);
    $('idle2d').hidden = true;
    document.querySelectorAll('.stage').forEach((s) => s.classList.add('live'));

    if (!det) {
      log('MoveNet 로드…');
      for (const b of ['webgl', 'cpu']) {
        try { if (await tf.setBackend(b)) break; } catch (e) { log(`  tfjs ${b} 실패`); }
      }
      await tf.ready();
      det = await poseDetection.createDetector(
        poseDetection.SupportedModels.MoveNet,
        { modelType: poseDetection.movenet.modelType.SINGLEPOSE_LIGHTNING });
      log('MoveNet 준비 (backend: ' + tf.getBackend() + ')');
    }

    if (!sess) {
      ort.env.wasm.wasmPaths = 'https://cdn.jsdelivr.net/npm/onnxruntime-web@1.27.0/dist/';
      if (!self.crossOriginIsolated) log('cross-origin isolated 아님 → WASM 은 단일 스레드');
      ort.env.wasm.numThreads = self.crossOriginIsolated
        ? Math.min(8, navigator.hardwareConcurrency || 4) : 1;
      ort.env.logLevel = 'error';
      F = +$('win').value;
      const url = `models/mb_${$('model').value}_${F}_${$('prec').value}.onnx`;
      const epWant = $('ep').value;
      log(`MotionBERT 로드 (${$('prec').value}, ${epWant})…`);
      stat('처음 한 번 준비 중입니다');
      const t0 = performance.now();
      let epUsed = epWant;
      try {
        sess = await ort.InferenceSession.create(url, { executionProviders: [epWant] });
      } catch (e) {
        log('  ' + epWant + ' 실패 → wasm 폴백');
        sess = await ort.InferenceSession.create(url, { executionProviders: ['wasm'] });
        epUsed = 'wasm';
      }
      $('m_load').textContent = ((performance.now() - t0)/1000).toFixed(1) + ' s';
      $('m_ep').textContent = epUsed + ' / wasm ' + ort.env.wasm.numThreads + 't';
    }

    const inBuf = new Float32Array(F*J*3);
    running = true; nLift = 0; buf = []; e2e = NaN;
    ringG.fill(-1); gidx = 0; lastShownG = -1;
    hasHeld = false; upEma = null; torsoEma = 0; dispG = -1; maxCovG = -1;
    resetLive(); resetEma();          // 이전 세션의 창을 물고 시작하지 않게
    $('stop').disabled = false;
    stat('측정 중');
    let tPrev = performance.now(), lastUi = 0;

    const loop = async () => {
      if (!running) return;
      const tNow = performance.now();
      stFps.push(1000/(tNow - tPrev)); tPrev = tNow;

      const a = performance.now();
      const poses = await det.estimatePoses(grabFrame(), { flipHorizontal: false });
      st2d.push(performance.now() - a);

      let kp = null;
      if (poses.length) kp = poses[0].keypoints.map((k) => [k.x, k.y, k.score ?? 0]);
      if (kp) {
        buf.push({ h: coco2h36m(kp), t: tNow, g: gidx++ });
        if (buf.length > F) buf.shift();
      }
      draw2d(kp, vw, vh);

      const stride = +$('stride').value;
      if ($('autolag').checked && nLift > 2) {
        const need = stride + Math.ceil(stLift.p(.9) * stFps.p(.5) / 1000) + 3;
        const cur = +$('lag').value;
        if (Math.abs(need - cur) > 1) {
          const v = clamp(need, 0, 95);
          $('lag').value = v; $('lagV').textContent = v + ' fr';
        }
      }
      const lag = +$('lag').value;

      /* 리프팅 한 번이 F 프레임 전부의 3D 를 돌려준다. 그중 1프레임만 쓰면 3D 가
         초당 1회만 갱신되므로, 전역 프레임 인덱스로 주소를 매긴 링 버퍼에 윈도우
         전체를 써넣고 카메라 속도로 재생한다. */
      const want = gidx - 1 - lag;
      const slot = ((want % RING) + RING) % RING;
      // want 가 음수면 아직 그 프레임이 존재하지 않는다. ringG 의 초기값도 -1 이라
      // 그냥 비교하면 want=-1 이 빈 슬롯과 일치해버리고, 한 번도 쓰지 않은 ring3d 의
      // 0 좌표를 자세로 읽는다 (무릎 90°/기울기 90°/발 0% 가 그 증상이다).
      if (want >= 0 && nLift > 0 && ringG[slot] === want) {
        held.set(ring3d.subarray(slot*J*3, slot*J*3 + J*3));
        hasHeld = true;
        if (want !== lastShownG) lastShownG = want;
        e2e = tNow - ringT[slot];
        cov3d.push(1);
      } else if (nLift) cov3d.push(0);

      if (hasHeld) {
        const p = $('lvl').checked ? levelPose(held) : held;
        draw3d(p);

        const fm = frameMetrics(p);
        // 등록 중이면 기준선 쪽에, 아니면 라이브 쪽에 쌓는다
        if (calUntil) {
          for (const m of METRICS) calVals[m.key].push(fm[m.key]);
          if (performance.now() >= calUntil) finishCal();
        } else if (BASE) {
          pushLive(fm);
        }

        if (tNow - lastUi > 60) { updateMetricPanel(fm); lastUi = tNow; }
      } else draw3d(null);

      if (!lifting && buf.length === F) {
        if (performance.now() - lastLift >= stride * (1000/Math.max(1, stFps.p(.5)))) {
          lifting = true; lastLift = performance.now();
          const win = buf.map((x) => x.h), g0 = buf[0].g, ts = buf.map((x) => x.t);
          cropScale(win, inBuf);
          const t1 = performance.now();
          sess.run({ kp2d: new ort.Tensor('float32', inBuf, [1, F, J, 3]) }).then((r) => {
            stLift.push(performance.now() - t1);
            const o = r[sess.outputNames[0]].data;
            for (let k = 0; k < F; k++) {
              const gg = g0 + k, sl = ((gg % RING) + RING) % RING;
              ring3d.set(o.subarray(k*J*3, k*J*3 + J*3), sl*J*3);
              ringG[sl] = gg; ringT[sl] = ts[k];
            }
            if (g0 + F - 1 > maxCovG) maxCovG = g0 + F - 1;
            nLift++;
          }).catch((e) => log('리프팅 실패: ' + (e.message || e))).finally(() => { lifting = false; });
        }
      }

      if (DEV) {
        const fps = stFps.p(.5), lift = stLift.p(.5);
        $('m_fps').textContent  = fps.toFixed(1) + ' fps';
        $('m_2d').textContent   = st2d.p(.5).toFixed(1) + ' ms';
        $('m_lift').textContent = isNaN(lift) ? '–' : lift.toFixed(1) + ' ms';
        $('m_duty').textContent = isNaN(lift) ? '–' : (lift / (stride * 1000 / Math.max(1, fps)) * 100).toFixed(0) + ' %';
        $('m_e2e').textContent  = isNaN(e2e) ? '–' : (e2e/1000).toFixed(2) + ' s';
        const c = cov3d.v.length ? cov3d.v.reduce((x,y)=>x+y,0)/cov3d.v.length*100 : NaN;
        $('m_cov').textContent  = isNaN(c) ? '–' : c.toFixed(0) + ' %';
      }

      // 실제 영상 프레임에 동기 — rAF 는 디스플레이 주사율로 돌아서 같은 프레임을 중복 투입한다
      const v = $('vid');
      if (v.requestVideoFrameCallback) v.requestVideoFrameCallback(() => loop());
      else requestAnimationFrame(loop);
    };
    loop();
  } catch (e) {
    log('시작 실패: ' + (e.message || e));
    stat('카메라를 열 수 없습니다');
    $('start').disabled = false;
  }
}

/* ═══════════════════════════  기준선 등록  ═══════════════════════════ */

function finishCal() {
  calUntil = 0;
  const n = calVals[METRICS[0].key].length;
  // 창은 n/2 를 넘을 수 없다 — 넘으면 서로소 짝을 만들 수 없어 영점을 못 잰다
  const win = Math.max(1, Math.min(Math.max(30, Math.floor(n / CAL_DIV)), Math.floor(n / 2)));

  const hists = newHists();
  for (const m of METRICS) {
    const [lo, hi] = m.range;
    for (const v of calVals[m.key]) histPush(hists[m.key], v, lo, hi);
  }

  /* 점수의 영점 — "같은 사람이 같은 날 움직여도 이만큼은 다르다".
     라이브 창과 **같은 길이**의 블록을 그 나머지와 견준다. 위치를 반 칸씩 밀어가며
     여러 번 재고 중앙값을 쓴다 — 한 구간이 튀어도 영점이 흔들리지 않는다.
     이전 구현은 전반부를 '전반부를 포함한 전체'와 견줬는데, 표본이 겹쳐서
     이탈도가 실제 정상 변동의 1/5 로 나왔고 가만히 있어도 개입 권고가 떴다. */
  const per = [];                                  // 구간별 지표별 이탈도
  for (let o = 0; o + win <= n; o += Math.max(1, Math.floor(win / 2))) {
    if (n - win < win / 2) break;                  // 나머지가 너무 짧으면 의미 없다
    const one = {};
    for (const m of METRICS) {
      const [lo, hi] = m.range;
      const blk = new Float64Array(NB), rest = new Float64Array(NB);
      calVals[m.key].forEach((v, i) => histPush(i >= o && i < o + win ? blk : rest, v, lo, hi));
      one[m.key] = kl(blk, rest);
    }
    per.push(one);
  }
  if (!per.length) {                               // 여기까지 오면 표본이 병적으로 적다
    log('기준선 등록 실패 — 수집된 프레임이 너무 적다. 다시 등록해라.');
    stat('기준선 등록 실패 — 다시 시도');
    calVals = null;
    return;
  }
  // 지표마다 따로 중앙값을 잡는다. 한 구간이 튀어도 눈금이 흔들리지 않는다.
  const kl0 = {};
  for (const m of METRICS) {
    const v = per.map((o) => o[m.key]).sort((a, b) => a - b);
    kl0[m.key] = v[Math.floor(v.length / 2)];
  }
  const tot0 = METRICS.reduce((a, m) => a + kl0[m.key], 0);

  BASE = { hists, kl0, tot0, at: Date.now(), n, win };  // 이전 기준선은 여기서 버려진다
  resetLive();
  resetEma();
  calVals = null;
  if (n < 150) log(`경고: 기준선이 ${n}프레임뿐이다 (권장 300 이상). 점수 눈금이 불안정할 수 있다.`);
  log(`기준선 등록 완료 — ${n}프레임, 비교 창 ${win}프레임, 구간 ${per.length}개\n` +
      METRICS.map((m) => `   ${m.name} 영점 ${kl0[m.key].toFixed(3)}` +
        ` (${per.map((o) => o[m.key].toFixed(2)).join(' / ')})`).join('\n'));
  stat('측정 중');
  refreshCompare();
}

$('cal').onclick = () => {
  if (!running) { stat('먼저 [측정 시작]을 누르세요'); return; }
  calVals = {};
  for (const m of METRICS) calVals[m.key] = [];
  calUntil = performance.now() + CAL_MS;
  stat('기준선 등록 중');
  log('기준선 등록 시작 — ' + (CAL_MS/1000) + '초');
};

$('clr').onclick = () => {
  BASE = null; resetLive(); resetEma();
  log('기준선 삭제');
  refreshCompare();
};

/* ═══════════════════════════  배선  ═══════════════════════════ */

$('start').onclick = run;
$('stop').onclick  = stopRun;
$('scan').onclick  = () => listCams(true);
$('src').onclick = () => {
  srcVideo = !srcVideo;
  $('src').textContent = '입력: ' + (srcVideo ? '샘플 영상' : '카메라');
  if (running) { stopRun(); run(); }
};
for (const [r, b] of [['stride', 'strideV'], ['lag', 'lagV']]) {
  $(r).oninput = () => { $(b).textContent = $(r).value + ' fr'; };
}

// 슬라이더를 건드리는 순간 자동 회전을 멈춘다. 슬라이더가 현재 각도를 이미
// 따라다니고 있으므로 넘겨받을 때 화면이 튀지 않는다.
$('yaw').oninput = () => {
  autoYaw = false;
  yaw = +$('yaw').value * Math.PI / 180;
  if (!running) draw3d(null);
};
// 더블클릭하면 다시 자동 회전
$('yaw').ondblclick = () => { autoYaw = true; };


/* ?dev=1 에서만: 위험 표시를 고정해 렌더링을 눈으로 확인한다.
   실제 이탈이 생길 때까지 기다리지 않고 색·크기를 손볼 수 있다.
   __setRisk({2:2, 5:2}) → 무릎 개입 단계. null 이면 해제. */
if (DEV) window.__setRisk = (obj) => { window.__riskPin = obj; };

buildMetricPanel();
buildDists();
// 예전 버전이 localStorage 에 남긴 기준선을 청소한다 (이제 저장하지 않는다)
try { localStorage.removeItem(LS_KEY); } catch {}
refreshCompare();
draw3d(null);

setInterval(() => { if (running || calUntil) refreshCompare(); }, 500);
addEventListener('resize', () => {
  for (const m of METRICS) { drawTrace(m); }
  refreshCompare();
});

// 샘플 영상 유무는 알려만 준다 — 버튼은 남긴다 (파일을 나중에 떨궈도 새로고침 없이 잡히도록)
fetch('assets/sample.mp4', { method: 'HEAD' })
  .then((r) => log(r.ok ? '샘플 영상 준비됨' : 'assets/sample.mp4 없음 — 넣으면 [입력] 토글로 쓸 수 있다'))
  .catch(() => log('assets/sample.mp4 확인 실패'));
