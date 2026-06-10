"""Standalone Flask/SocketIO web dashboard for the Doosan RH-P12-RN(A) gripper.

This is the original threaded dashboard: it spins up a ROS2 node in a background
thread purely to use the gripper TCP bridge, then serves a SocketIO web UI on
port 5000. For the proper ROS2 node version see :mod:`web_dashboard_node`.

This direct-owner mode is kept as a legacy compatibility path. The recommended
operational entrypoint is ``gripper_service_node`` as the single TCP bridge
owner, with future dashboard work moving toward a ROS client architecture.
"""

import threading
import time
import socket
from flask import Flask, render_template_string
from flask_socketio import SocketIO
import rclpy

from dsr_gripper_tcp.gripper_tcp_bridge import DoosanGripperTcpBridge, BridgeConfig
from dsr_gripper_tcp.robot_utils import set_robot_mode_autonomous

app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins="*")

bridge = None
ros_node = None
tcp_lock = threading.Lock()
POLL_INTERVAL = 0.05  # 20 fps

# RH-P12-RN(A) hardware limits / sensible UI ranges
POSITION_MAX = 1150
GOAL_CURRENT_MIN, GOAL_CURRENT_MAX = 50, 1000
PROFILE_VEL_MIN, PROFILE_VEL_MAX = 100, 3000
PROFILE_ACC_MIN, PROFILE_ACC_MAX = 100, 3000


HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<title>Doosan Gripper Dashboard</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  :root {
    --bg: #0e1118;
    --panel: #161b25;
    --panel-2: #1e2532;
    --border: #2a3142;
    --fg: #e6edf3;
    --fg-dim: #8b95a7;
    --accent: #3ea6ff;
    --ok: #2ecc71;
    --warn: #f1c40f;
    --danger: #e74c3c;
    --muted: #4a5365;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 16px;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", "Pretendard", sans-serif;
    background: var(--bg); color: var(--fg);
  }
  header {
    display: flex; align-items: center; justify-content: space-between;
    padding: 12px 18px; background: var(--panel);
    border: 1px solid var(--border); border-radius: 10px; margin-bottom: 14px;
  }
  header h1 { margin: 0; font-size: 18px; font-weight: 600; }
  header .sub { color: var(--fg-dim); font-size: 12px; margin-left: 10px; }
  .conn { display:flex; align-items:center; gap:8px; font-size: 13px; }
  .dot { width:10px; height:10px; border-radius:50%; background: var(--muted); box-shadow: 0 0 8px transparent; }
  .dot.ok    { background: var(--ok);    box-shadow: 0 0 8px var(--ok); }
  .dot.warn  { background: var(--warn);  box-shadow: 0 0 8px var(--warn); }
  .dot.err   { background: var(--danger);box-shadow: 0 0 8px var(--danger); }

  .grid-main { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
  @media (max-width: 980px) { .grid-main { grid-template-columns: 1fr; } }

  .card {
    background: var(--panel); border: 1px solid var(--border);
    border-radius: 10px; padding: 16px;
  }
  .card h2 {
    margin: 0 0 12px 0; font-size: 14px; font-weight: 600;
    color: var(--fg-dim); text-transform: uppercase; letter-spacing: 1px;
  }

  .stat-row { display:grid; grid-template-columns: repeat(2, 1fr); gap: 10px; }
  .stat {
    background: var(--panel-2); border: 1px solid var(--border);
    border-radius: 8px; padding: 12px;
  }
  .stat .label { color: var(--fg-dim); font-size: 11px; text-transform: uppercase; letter-spacing: 0.5px; }
  .stat .value { font-size: 22px; font-weight: 600; font-variant-numeric: tabular-nums; margin-top: 4px; }
  .stat .unit  { color: var(--fg-dim); font-size: 13px; margin-left: 4px; font-weight: 400; }
  .stat .sub   { color: var(--fg-dim); font-size: 11px; margin-top: 2px; }

  .bar-wrap { margin-top: 8px; height: 8px; border-radius: 4px; background: #0a0d13; overflow: hidden; position: relative; }
  .bar { height: 100%; transition: width 0.1s linear; background: var(--accent); }
  .bar.ok    { background: var(--ok); }
  .bar.warn  { background: var(--warn); }
  .bar.err   { background: var(--danger); }
  .bar-marker { position: absolute; top:-2px; bottom:-2px; width: 2px; background: #fff; }

  .chips { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 12px; }
  .chip {
    display: inline-flex; align-items: center; gap: 6px;
    padding: 5px 10px; background: var(--panel-2);
    border: 1px solid var(--border); border-radius: 999px; font-size: 12px;
    color: var(--fg-dim);
  }
  .chip .dot { width: 8px; height: 8px; }
  .chip.active { color: var(--fg); border-color: var(--accent); }

  .control-group { margin-bottom: 16px; }
  .control-group:last-child { margin-bottom: 0; }
  .control-group .head { display:flex; justify-content:space-between; align-items:center; margin-bottom: 6px; }
  .control-group label { color: var(--fg-dim); font-size: 12px; text-transform: uppercase; }
  .control-group .num { font-size: 13px; font-variant-numeric: tabular-nums; color: var(--fg); }

  input[type=range] {
    -webkit-appearance: none; appearance: none;
    width: 100%; height: 6px; background: #0a0d13;
    border-radius: 3px; outline: none;
  }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none; appearance: none;
    width: 18px; height: 18px; border-radius: 50%;
    background: var(--accent); cursor: pointer; border: 2px solid #fff;
  }
  input[type=range]::-moz-range-thumb {
    width: 18px; height: 18px; border-radius: 50%;
    background: var(--accent); cursor: pointer; border: 2px solid #fff;
  }
  input[type=range]:disabled::-webkit-slider-thumb { background: var(--muted); }
  input[type=range]:disabled::-moz-range-thumb { background: var(--muted); }

  .btn-row { display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px; margin-top: 8px; }
  button {
    background: var(--panel-2); color: var(--fg);
    border: 1px solid var(--border); padding: 10px;
    border-radius: 6px; cursor: pointer; font-size: 13px;
    transition: background 0.15s, border-color 0.15s;
  }
  button:hover:not(:disabled) { background: #2a3445; border-color: var(--accent); }
  button.primary { background: var(--accent); border-color: var(--accent); color: #fff; }
  button.primary:hover { background: #2987d4; }
  button.danger  { background: var(--danger); border-color: var(--danger); color: #fff; }
  button.danger:hover  { background: #c0392b; }
  button:disabled { opacity: 0.4; cursor: not-allowed; }

  .toggle {
    position: relative; display: inline-block; width: 56px; height: 28px;
  }
  .toggle input { opacity: 0; width: 0; height: 0; }
  .slider-toggle {
    position: absolute; cursor: pointer; inset: 0;
    background: #2a3142; border-radius: 28px; transition: 0.2s;
  }
  .slider-toggle:before {
    content: ""; position: absolute; height: 22px; width: 22px;
    left: 3px; top: 3px; background: white; border-radius: 50%; transition: 0.2s;
  }
  .toggle input:checked + .slider-toggle { background: var(--ok); }
  .toggle input:checked + .slider-toggle:before { transform: translateX(28px); }
  .toggle-row { display: flex; align-items: center; gap: 12px; }
  .toggle-label { font-size: 13px; }

  .chart-card { grid-column: 1 / -1; }
  .chart-toolbar {
    display: flex; flex-wrap: wrap; gap: 12px; align-items: center;
    margin: 0 0 10px 0; color: var(--fg-dim); font-size: 12px;
  }
  .chart-toggle { display: inline-flex; align-items: center; gap: 6px; }
  .chart-toggle input { accent-color: var(--accent); }
  .swatch { width: 10px; height: 10px; border-radius: 2px; display: inline-block; }
  .chart-wrap { position: relative; height: 280px; }

  .grasping {
    color: var(--ok); font-size: 11px; margin-top: 4px;
    visibility: hidden;
  }
  .grasping.show { visibility: visible; }
</style>
</head>
<body>

<header>
  <div>
    <h1>Doosan Gripper Control</h1>
    <span class="sub">RH-P12-RN(A) · 20 FPS Live</span>
  </div>
  <div class="conn">
    <span class="dot" id="conn-dot"></span>
    <span id="conn-text">Connecting...</span>
  </div>
</header>

<div class="grid-main">

  <!-- ============= LIVE STATUS ============= -->
  <div class="card">
    <h2>Live Status</h2>

    <div class="stat" style="margin-bottom: 10px;">
      <div class="label">Position</div>
      <div class="value">
        <span id="pos">--</span><span class="unit">/ {{ pos_max }}</span>
      </div>
      <div class="bar-wrap">
        <div class="bar ok" id="pos-bar" style="width: 0%"></div>
        <div class="bar-marker" id="pos-goal-marker" style="left: 0%"></div>
      </div>
      <div class="sub" id="pos-sub">goal: --</div>
    </div>

    <div class="stat-row">
      <div class="stat">
        <div class="label">Current (Force)</div>
        <div class="value"><span id="cur">--</span><span class="unit">mA</span></div>
        <div class="bar-wrap"><div class="bar warn" id="cur-bar" style="width: 0%"></div></div>
        <div class="grasping" id="grasping-tag">● Grasping detected</div>
      </div>
      <div class="stat">
        <div class="label">Velocity</div>
        <div class="value" id="vel">--</div>
        <div class="sub" id="vel-dir">idle</div>
      </div>
      <div class="stat">
        <div class="label">Temperature</div>
        <div class="value" id="temp">--<span class="unit">°C</span></div>
        <div class="sub" id="temp-warn">normal</div>
      </div>
      <div class="stat">
        <div class="label">Moving Status</div>
        <div class="value" id="moving">--</div>
        <div class="sub" id="moving-sub">moving_status: --</div>
      </div>
    </div>

    <div class="chips">
      <div class="chip" id="chip-torque"><span class="dot"></span>Torque</div>
      <div class="chip" id="chip-ready"><span class="dot"></span>Ready</div>
      <div class="chip" id="chip-inposition"><span class="dot"></span>In Position</div>
      <div class="chip" id="chip-moving"><span class="dot"></span>Moving</div>
    </div>
  </div>

  <!-- ============= CONTROL ============= -->
  <div class="card">
    <h2>Control</h2>

    <div class="control-group">
      <div class="head">
        <label>Torque</label>
        <div class="toggle-row">
          <span class="toggle-label" id="torque-label">OFF</span>
          <label class="toggle">
            <input type="checkbox" id="torque-toggle">
            <span class="slider-toggle"></span>
          </label>
        </div>
      </div>
      <div class="sub" style="color: var(--fg-dim); font-size: 11px;">
        Torque OFF 시 그리퍼 수동 조작 가능 (모니터링은 유지)
      </div>
    </div>

    <div class="control-group">
      <div class="head">
        <label>Goal Position</label>
        <span class="num"><span id="goal-pos-num">700</span> / {{ pos_max }}</span>
      </div>
      <input type="range" id="goal-pos" min="0" max="{{ pos_max }}" value="700">
      <div class="btn-row">
        <button onclick="setGoal(0)">Open (0)</button>
        <button onclick="setGoal(500)">Half (500)</button>
        <button onclick="setGoal(700)">Close (700)</button>
        <button onclick="setGoal({{ pos_max }})">Full ({{ pos_max }})</button>
      </div>
      <div class="btn-row" style="grid-template-columns: 1fr 1fr;">
        <button class="primary" id="btn-move" onclick="moveToGoal()">Move To Goal</button>
        <button class="primary" id="btn-safe-grasp" onclick="safeGrasp()">Safe Grasp</button>
      </div>
    </div>

    <div class="control-group">
      <div class="head">
        <label>Goal Current (Grip Force)</label>
        <span class="num"><span id="cur-num">400</span> mA</span>
      </div>
      <input type="range" id="goal-cur" min="{{ cur_min }}" max="{{ cur_max }}" value="400">
    </div>

    <div class="control-group">
      <div class="head">
        <label>Current Delta Threshold</label>
        <span class="num"><span id="cur-delta-num">120</span> mA</span>
      </div>
      <input type="range" id="cur-delta" min="0" max="500" value="120">
    </div>

    <div class="control-group">
      <div class="head">
        <label>Profile Velocity</label>
        <span class="num"><span id="vel-num">1500</span></span>
      </div>
      <input type="range" id="profile-vel" min="{{ vel_min }}" max="{{ vel_max }}" value="1500">
    </div>

    <div class="control-group">
      <div class="head">
        <label>Profile Acceleration</label>
        <span class="num"><span id="acc-num">1000</span></span>
      </div>
      <input type="range" id="profile-acc" min="{{ acc_min }}" max="{{ acc_max }}" value="1000">
    </div>
  </div>

  <!-- ============= REAL-TIME CHART ============= -->
  <div class="card chart-card">
    <h2>Real-time Telemetry (last ~10s)</h2>
    <div class="chart-toolbar">
      <label class="chart-toggle">
        <input type="checkbox" data-dataset="0" checked>
        <span class="swatch" style="background:#3ea6ff"></span>Position
      </label>
      <label class="chart-toggle">
        <input type="checkbox" data-dataset="1" checked>
        <span class="swatch" style="background:#8b95a7"></span>Goal
      </label>
      <label class="chart-toggle">
        <input type="checkbox" data-dataset="2" checked>
        <span class="swatch" style="background:#f1c40f"></span>Current
      </label>
      <label class="chart-toggle">
        <input type="checkbox" data-dataset="3" checked>
        <span class="swatch" style="background:#2ecc71"></span>Velocity
      </label>
    </div>
    <div class="chart-wrap"><canvas id="chart"></canvas></div>
  </div>

</div>

<script>
const socket = io();

const POS_MAX = {{ pos_max }};
const CUR_MIN = {{ cur_min }}, CUR_MAX = {{ cur_max }};

// state
let lastGoalPosition = 700;
let lastGoalCurrent  = 400;
let currentDeltaThreshold = 120;
let currentProfile = { goal_current: 400, profile_velocity: 1500, profile_acceleration: 1000 };
let torqueEnabled = false;

// ===== Chart.js =====
const MAX_POINTS = 200;
const chart = new Chart(document.getElementById('chart').getContext('2d'), {
  type: 'line',
  data: {
    labels: [],
    datasets: [
      { label: 'Position',     data: [], borderColor: '#3ea6ff', backgroundColor: 'rgba(62,166,255,0.1)',  yAxisID: 'yPos', tension: 0.2, pointRadius: 0, borderWidth: 2 },
      { label: 'Goal Position',data: [], borderColor: '#8b95a7', borderDash: [4,4], yAxisID: 'yPos', tension: 0, pointRadius: 0, borderWidth: 1.5 },
      { label: 'Current',      data: [], borderColor: '#f1c40f', backgroundColor: 'rgba(241,196,15,0.1)',  yAxisID: 'yCur', tension: 0.2, pointRadius: 0, borderWidth: 2 },
      { label: 'Velocity',     data: [], borderColor: '#2ecc71', backgroundColor: 'rgba(46,204,113,0.1)',  yAxisID: 'yVel', tension: 0.2, pointRadius: 0, borderWidth: 2 }
    ]
  },
  options: {
    animation: false,
    responsive: true,
    maintainAspectRatio: false,
    interaction: { intersect: false, mode: 'index' },
    plugins: {
      legend: { display: false },
      tooltip: { backgroundColor: '#1e2532', borderColor: '#2a3142', borderWidth: 1 }
    },
    scales: {
      x: { ticks: { color: '#8b95a7', maxTicksLimit: 6 }, grid: { color: '#2a3142' } },
      yPos: { type: 'linear', position: 'left',  min: 0, max: POS_MAX, ticks: { color: '#3ea6ff' }, grid: { color: '#2a3142' }, title: { display: true, text: 'Position', color: '#3ea6ff' } },
      yCur: { type: 'linear', position: 'right', ticks: { color: '#f1c40f' }, grid: { drawOnChartArea: false }, title: { display: true, text: 'Current', color: '#f1c40f' } },
      yVel: { type: 'linear', display: false }
    }
  }
});

document.querySelectorAll('.chart-toggle input').forEach((input) => {
  input.addEventListener('change', () => {
    const idx = parseInt(input.dataset.dataset);
    chart.setDatasetVisibility(idx, input.checked);
    chart.update('none');
  });
});

function pushPoint(time, pos, goal, cur, vel) {
  const ds = chart.data.datasets;
  chart.data.labels.push(time);
  ds[0].data.push(pos);
  ds[1].data.push(goal);
  ds[2].data.push(cur);
  ds[3].data.push(vel);
  if (chart.data.labels.length > MAX_POINTS) {
    chart.data.labels.shift();
    ds.forEach(d => d.data.shift());
  }
  chart.update('none');
}

// ===== Connection state =====
socket.on('connect', () => {
  document.getElementById('conn-dot').className = 'dot ok';
  document.getElementById('conn-text').innerText = 'Connected';
});
socket.on('disconnect', () => {
  document.getElementById('conn-dot').className = 'dot err';
  document.getElementById('conn-text').innerText = 'Disconnected';
});

// ===== Live state updates =====
socket.on('state_update', (data) => {
  if (data.status === 'error') {
    document.getElementById('conn-dot').className = 'dot warn';
    document.getElementById('conn-text').innerText = 'Waiting for gripper state...';
    return;
  }
  document.getElementById('conn-dot').className = 'dot ok';
  document.getElementById('conn-text').innerText = 'Connected';

  // Position
  document.getElementById('pos').innerText = data.present_position;
  const posPct = Math.max(0, Math.min(100, (data.present_position / POS_MAX) * 100));
  document.getElementById('pos-bar').style.width = posPct + '%';
  document.getElementById('pos-goal-marker').style.left = ((lastGoalPosition / POS_MAX) * 100) + '%';
  document.getElementById('pos-sub').innerText = 'goal: ' + lastGoalPosition;

  // Current
  document.getElementById('cur').innerText = data.present_current;
  const curPct = Math.max(0, Math.min(100, (Math.abs(data.present_current) / Math.max(1, lastGoalCurrent)) * 100));
  document.getElementById('cur-bar').style.width = curPct + '%';
  const grasping = Math.abs(data.present_current) >= lastGoalCurrent * 0.9 && data.moving === 0;
  document.getElementById('grasping-tag').classList.toggle('show', grasping);

  // Velocity
  document.getElementById('vel').innerText = data.present_velocity;
  const vd = document.getElementById('vel-dir');
  if (data.present_velocity > 5)       vd.innerText = '→ closing';
  else if (data.present_velocity < -5) vd.innerText = '← opening';
  else                                  vd.innerText = 'idle';

  // Temperature
  document.getElementById('temp').innerHTML = data.present_temperature + '<span class="unit">°C</span>';
  const tw = document.getElementById('temp-warn');
  if (data.present_temperature >= 55)      { tw.innerText = '⚠ HIGH';     tw.style.color = 'var(--danger)'; }
  else if (data.present_temperature >= 45) { tw.innerText = 'warm';        tw.style.color = 'var(--warn)';   }
  else                                      { tw.innerText = 'normal';      tw.style.color = 'var(--fg-dim)'; }

  // Moving
  document.getElementById('moving').innerText = data.moving === 1 ? 'Moving' : 'Stopped';
  document.getElementById('moving-sub').innerText = 'moving_status: 0x' + (data.moving_status || 0).toString(16);

  // Status chips
  setChip('chip-torque',     data.torque_enabled, data.torque_enabled ? 'Torque ON' : 'Torque OFF');
  setChip('chip-ready',      data.torque_enabled, data.torque_enabled ? 'Ready' : 'Not Ready');
  setChip('chip-inposition', !!(data.moving_status & 0x01), 'In Position');
  setChip('chip-moving',     data.moving === 1, 'Moving');

  // Sync torque toggle with backend state
  const tgl = document.getElementById('torque-toggle');
  if (tgl.dataset.userToggling !== '1' && tgl.checked !== !!data.torque_enabled) {
    tgl.checked = !!data.torque_enabled;
    document.getElementById('torque-label').innerText = data.torque_enabled ? 'ON' : 'OFF';
  }
  torqueEnabled = !!data.torque_enabled;
  updateControlsEnabled();

  // Chart push
  const t = new Date();
  const ts = t.toLocaleTimeString('en-GB', { hour12: false }) + '.' + String(t.getMilliseconds()).padStart(3,'0').slice(0,2);
  pushPoint(ts, data.present_position, lastGoalPosition, data.present_current, data.present_velocity);
});

function setChip(id, active, text) {
  const el = document.getElementById(id);
  el.classList.toggle('active', !!active);
  el.querySelector('.dot').className = 'dot ' + (active ? 'ok' : '');
  el.lastChild.textContent = text;
}

function updateControlsEnabled() {
  const moveBtn = document.getElementById('btn-move');
  const safeGraspBtn = document.getElementById('btn-safe-grasp');
  moveBtn.disabled = !torqueEnabled;
  safeGraspBtn.disabled = !torqueEnabled;
}

// ===== Controls =====
const goalPos = document.getElementById('goal-pos');
const goalPosNum = document.getElementById('goal-pos-num');
goalPos.addEventListener('input', () => {
  lastGoalPosition = parseInt(goalPos.value);
  goalPosNum.innerText = lastGoalPosition;
});

function setGoal(v) {
  goalPos.value = v;
  lastGoalPosition = v;
  goalPosNum.innerText = v;
}

function moveToGoal() {
  if (!torqueEnabled) return;
  socket.emit('move_cmd', { goal_position: lastGoalPosition });
}

function safeGrasp() {
  if (!torqueEnabled) return;
  socket.emit('safe_grasp_cmd', {
    target_position: lastGoalPosition,
    max_current: lastGoalCurrent,
    current_delta_threshold: currentDeltaThreshold,
    timeout_sec: 8.0,
    profile_velocity: currentProfile.profile_velocity,
    profile_acceleration: currentProfile.profile_acceleration
  });
}

// Torque toggle
const torqueTgl = document.getElementById('torque-toggle');
torqueTgl.addEventListener('change', () => {
  torqueTgl.dataset.userToggling = '1';
  document.getElementById('torque-label').innerText = torqueTgl.checked ? 'ON' : 'OFF';
  socket.emit('torque_cmd', { enabled: torqueTgl.checked });
  setTimeout(() => { torqueTgl.dataset.userToggling = '0'; }, 1500);
});

// ===== Profile sliders with debounce =====
function debounce(fn, ms) {
  let t = null;
  return (...a) => { clearTimeout(t); t = setTimeout(() => fn(...a), ms); };
}

const sendProfile = debounce(() => {
  socket.emit('profile_cmd', { ...currentProfile });
}, 300);

function bindProfileSlider(sliderId, numId, key) {
  const s = document.getElementById(sliderId);
  const n = document.getElementById(numId);
  s.addEventListener('input', () => {
    const v = parseInt(s.value);
    n.innerText = v;
    currentProfile[key] = v;
    if (key === 'goal_current') lastGoalCurrent = v;
    sendProfile();
  });
}

bindProfileSlider('goal-cur',   'cur-num', 'goal_current');
bindProfileSlider('profile-vel','vel-num', 'profile_velocity');
bindProfileSlider('profile-acc','acc-num', 'profile_acceleration');

const curDelta = document.getElementById('cur-delta');
const curDeltaNum = document.getElementById('cur-delta-num');
curDelta.addEventListener('input', () => {
  currentDeltaThreshold = parseInt(curDelta.value);
  curDeltaNum.innerText = currentDeltaThreshold;
});
</script>
</body>
</html>
"""


def reset_socket_on_error():
    global bridge
    if bridge and bridge._socket:
        try:
            bridge._socket.close()
        except Exception:
            pass
        bridge._socket = None


def background_polling_thread():
    while True:
        if bridge is not None:
            try:
                if tcp_lock.acquire(timeout=0.01):
                    try:
                        state = bridge.read_state()
                        socketio.emit('state_update', {
                            "status": "ok",
                            "present_position": state.present_position,
                            "present_current": state.present_current,
                            "present_temperature": state.present_temperature,
                            "present_velocity": state.present_velocity,
                            "moving": state.moving,
                            "moving_status": state.moving_status,
                            "torque_enabled": state.torque_enabled,
                        })
                    finally:
                        tcp_lock.release()
            except (BrokenPipeError, ConnectionError, socket.error):
                reset_socket_on_error()
                socketio.emit('state_update', {"status": "error"})
            except Exception:
                pass
        time.sleep(POLL_INTERVAL)


@app.route('/')
def index():
    return render_template_string(
        HTML_TEMPLATE,
        pos_max=POSITION_MAX,
        cur_min=GOAL_CURRENT_MIN, cur_max=GOAL_CURRENT_MAX,
        vel_min=PROFILE_VEL_MIN, vel_max=PROFILE_VEL_MAX,
        acc_min=PROFILE_ACC_MIN, acc_max=PROFILE_ACC_MAX,
    )


def _run_in_bridge(fn):
    """Run a bridge call on a background thread under the TCP lock."""
    def runner():
        try:
            with tcp_lock:
                fn()
        except Exception:
            reset_socket_on_error()
    threading.Thread(target=runner, daemon=True).start()


@socketio.on('move_cmd')
def handle_move(data):
    if not bridge or 'goal_position' not in data:
        return
    pos = int(data['goal_position'])
    _run_in_bridge(lambda: bridge.move_to(pos, 5.0))


@socketio.on('torque_cmd')
def handle_torque(data):
    if not bridge or 'enabled' not in data:
        return
    enabled = bool(data['enabled'])
    _run_in_bridge(lambda: bridge.set_torque(enabled))


@socketio.on('profile_cmd')
def handle_profile(data):
    if not bridge:
        return
    gc = int(data.get('goal_current', 400))
    pv = int(data.get('profile_velocity', 1500))
    pa = int(data.get('profile_acceleration', 1000))
    _run_in_bridge(lambda: bridge.set_motion_profile(gc, pv, pa))


@socketio.on('safe_grasp_cmd')
def handle_safe_grasp(data):
    """Legacy direct-owner safe grasp approximation for the standalone dashboard."""
    if not bridge:
        return
    target_position = int(data.get('target_position', 700))
    max_current = int(data.get('max_current', 400))
    profile_velocity = int(data.get('profile_velocity', 1500))
    profile_acceleration = int(data.get('profile_acceleration', 1000))
    timeout_sec = float(data.get('timeout_sec', 8.0))
    _run_in_bridge(
        lambda: (
            bridge.set_motion_profile(max_current, profile_velocity, profile_acceleration),
            bridge.move_to(target_position, timeout_sec),
        )
    )


def ros_thread():
    global bridge, ros_node
    rclpy.init(args=None)
    ros_node = rclpy.create_node("gripper_web_backend")

    local_bridge = DoosanGripperTcpBridge(
        node=ros_node,
        config=BridgeConfig(
            controller_host="110.120.1.56",
            namespace="dsr01",
            service_prefix=""
        )
    )
    bridge = local_bridge

    try:
        ros_node.get_logger().info("Setting autonomous mode...")
        set_robot_mode_autonomous(ros_node, "dsr01", "")

        ros_node.get_logger().info("Starting TCP Bridge...")
        local_bridge.start()

        with tcp_lock:
            local_bridge.initialize()

        threading.Thread(target=background_polling_thread, daemon=True).start()

        rclpy.spin(ros_node)
    except Exception as e:
        ros_node.get_logger().error(f"ROS thread error: {e}")
        # Disable the global handle so SocketIO handlers stop trying to use
        # a half-broken bridge (which would spam reconnect attempts).
        bridge = None
    finally:
        try:
            local_bridge.close()
        except Exception:
            pass
        ros_node.destroy_node()
        rclpy.shutdown()


def main():
    threading.Thread(target=ros_thread, daemon=True).start()
    time.sleep(3)
    print("🚀 Web server started! Open http://localhost:5000 in your browser.")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, use_reloader=False)


if __name__ == '__main__':
    main()
