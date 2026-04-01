#!/usr/bin/env python3
"""
moveit_dashboard.py — SpiderBot Web Dashboard
==============================================
Opens a browser-based dashboard at http://localhost:8888
No tkinter/PyQt5 needed — uses Python's built-in http.server

Usage: ros2 run spiderbot moveit_dashboard.py
Then open: http://localhost:8888
"""
import sys, os, math, time, json, threading, subprocess
import http.server, urllib.parse, webbrowser
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

PORT = 8888

JOINTS = [
    'fr1_joint','fr2_joint',
    'fl1_joint','fl2_joint',
    'rr1_joint','rr2_joint',
    'rl1_joint','rl2_joint',
]

POSES = {
    'Home'   : [0.0]*8,
    'Stand'  : [0.0,0.0,  0.0,0.0,  0.0,0.0,  0.0,0.0],
    'Sit'    : [0.0,-0.5, 0.0,-0.5, 0.0,-0.5, 0.0,-0.5],
    'Stretch': [0.5,0.3,  0.5,0.3, -0.5,0.3, -0.5,0.3],
    'Wave FR': [0.0,0.8,  0.0,0.0,  0.0,0.0,  0.0,0.0],
    'Wave FL': [0.0,0.0,  0.0,0.8,  0.0,0.0,  0.0,0.0],
    'Wave RR': [0.0,0.0,  0.0,0.0,  0.0,0.8,  0.0,0.0],
    'Wave RL': [0.0,0.0,  0.0,0.0,  0.0,0.0,  0.0,0.8],
    'Walk A' : [0.4,0.2,  0.0,0.0,  0.0,0.0,  0.4,0.2],
    'Walk B' : [0.0,0.0,  0.4,0.2,  0.4,0.2,  0.0,0.0],
}

# ──────────────────────────────────────────────────────────
# ROS2 Node
# ──────────────────────────────────────────────────────────
class RobotNode(Node):
    def __init__(self):
        super().__init__('spiderbot_dashboard')
        self._ac = ActionClient(self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        self.live    = [0.0]*8
        self.status  = 'Connecting...'
        self.recorded = []
        self._lock   = threading.Lock()
        self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)

    def _js_cb(self, msg):
        with self._lock:
            for i, n in enumerate(JOINTS):
                if n in msg.name:
                    self.live[i] = msg.position[msg.name.index(n)]

    def send(self, positions, duration=1.0):
        if not self._ac.server_is_ready():
            self.status = 'Controller not ready'
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions       = [float(p) for p in positions]
        pt.velocities      = [0.0]*8
        pt.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points   = [pt]
        goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        self._ac.send_goal_async(goal)
        self.status = 'Sent OK'
        return True

    def send_blocking(self, positions, duration=1.0):
        if not self._ac.server_is_ready():
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions       = [float(p) for p in positions]
        pt.velocities      = [0.0]*8
        pt.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points   = [pt]
        goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()
        fut = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        h = fut.result()
        if h and h.accepted:
            rf = h.get_result_async()
            rclpy.spin_until_future_complete(self, rf, timeout_sec=duration+2.0)
            return True
        return False


# ──────────────────────────────────────────────────────────
# HTTP Handler
# ──────────────────────────────────────────────────────────
class Handler(http.server.BaseHTTPRequestHandler):
    node = None  # set before server starts

    def log_message(self, fmt, *args): pass  # suppress access logs

    def do_GET(self):
        if self.path == '/':
            self._serve_html()
        elif self.path == '/state':
            self._serve_state()
        else:
            self.send_response(404); self.end_headers()

    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        body   = self.rfile.read(length)
        try:
            data = json.loads(body)
        except Exception:
            self.send_response(400); self.end_headers(); return

        action = data.get('action', '')
        node   = self.node

        if action == 'send':
            pos = [float(v) for v in data['positions']]
            dur = float(data.get('duration', 1.0))
            threading.Thread(target=node.send,
                             args=(pos, dur), daemon=True).start()
            self._ok({'status': 'sent'})

        elif action == 'pose':
            name = data['name']
            if name in POSES:
                dur = float(data.get('duration', 1.0))
                threading.Thread(target=node.send,
                                 args=(POSES[name], dur), daemon=True).start()
                self._ok({'status': 'pose sent', 'name': name})
            else:
                self._ok({'status': 'unknown pose'})

        elif action == 'record_add':
            pos   = [float(v) for v in data['positions']]
            dur   = float(data.get('duration', 1.0))
            delay = float(data.get('delay', 0.3))
            node.recorded.append({'pos': pos, 'dur': dur, 'delay': delay})
            self._ok({'frames': len(node.recorded)})

        elif action == 'record_clear':
            node.recorded.clear()
            self._ok({'frames': 0})

        elif action == 'record_delete':
            idx = int(data.get('index', -1))
            if 0 <= idx < len(node.recorded):
                node.recorded.pop(idx)
            self._ok({'frames': len(node.recorded)})

        elif action == 'replay':
            loop = bool(data.get('loop', False))
            seq  = list(node.recorded)
            if seq:
                threading.Thread(target=self._do_replay,
                                 args=(seq, loop), daemon=True).start()
                self._ok({'status': 'replaying', 'frames': len(seq)})
            else:
                self._ok({'status': 'empty'})

        elif action == 'save':
            fname = data.get('filename', 'sequence.json')
            path  = os.path.expanduser('~/' + fname)
            with open(path, 'w') as f:
                json.dump({'keyframes': node.recorded}, f, indent=2)
            self._ok({'saved': path})

        elif action == 'load':
            fname = data.get('filename', 'sequence.json')
            path  = os.path.expanduser('~/' + fname)
            try:
                with open(path) as f:
                    d = json.load(f)
                node.recorded = d.get('keyframes', [])
                self._ok({'loaded': path, 'frames': len(node.recorded)})
            except Exception as e:
                self._ok({'error': str(e)})

        elif action == 'get_sequences':
            files = [f for f in os.listdir(os.path.expanduser('~'))
                     if f.endswith('.json')]
            self._ok({'files': files})

        else:
            self._ok({'status': 'unknown action'})

    def _do_replay(self, seq, loop):
        node = self.node
        while True:
            for kf in seq:
                node.send_blocking(kf['pos'], kf['dur'])
                time.sleep(kf.get('delay', 0.3))
            if not loop:
                break
            time.sleep(0.5)
        node.status = 'Replay done'

    def _ok(self, data):
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_state(self):
        node = self.node
        with node._lock:
            live = list(node.live)
        data = {
            'live'    : live,
            'live_deg': [round(math.degrees(v), 1) for v in live],
            'status'  : node.status,
            'frames'  : len(node.recorded),
            'keyframes': node.recorded,
            'ready'   : node._ac.server_is_ready(),
            'poses'   : list(POSES.keys()),
        }
        self._ok(data)

    def _serve_html(self):
        html = HTML_PAGE.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.send_header('Content-Length', str(len(html)))
        self.end_headers()
        self.wfile.write(html)


# ──────────────────────────────────────────────────────────
# HTML Dashboard Page
# ──────────────────────────────────────────────────────────
HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>SpiderBot MoveIt Dashboard</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;700&family=Inter:wght@400;600&display=swap');
  *{box-sizing:border-box;margin:0;padding:0}
  :root{
    --bg:#0d1117;--card:#161b22;--card2:#1c2128;--border:#30363d;
    --text:#e6edf3;--muted:#8b949e;
    --blue:#58a6ff;--green:#3fb950;--red:#f85149;
    --amber:#d29922;--purple:#bc8cff;--teal:#39d353;
    --fr:#58a6ff;--fl:#3fb950;--rr:#d29922;--rl:#bc8cff;
  }
  body{background:var(--bg);color:var(--text);font-family:'Inter',sans-serif;min-height:100vh}
  .topbar{background:var(--card);border-bottom:1px solid var(--border);
    display:flex;align-items:center;padding:0 20px;height:50px;gap:16px}
  .topbar h1{font-family:'JetBrains Mono',monospace;font-size:16px;color:var(--blue)}
  .topbar .sub{font-size:12px;color:var(--muted)}
  .topbar .status{margin-left:auto;font-family:'JetBrains Mono',monospace;
    font-size:11px;color:var(--amber)}
  .main{display:grid;grid-template-columns:1fr 260px;gap:8px;padding:8px;height:calc(100vh - 50px)}
  .left{display:flex;flex-direction:column;gap:8px;overflow-y:auto}
  .right{display:flex;flex-direction:column;gap:8px;overflow-y:auto}
  .card{background:var(--card);border:1px solid var(--border);border-radius:6px;overflow:hidden}
  .card-hdr{background:var(--card2);padding:8px 12px;font-size:12px;font-weight:600;border-bottom:1px solid var(--border)}
  .card-body{padding:10px}

  /* Knob grid */
  .legs{display:flex;gap:8px;flex-wrap:wrap}
  .leg-panel{border:1px solid var(--border);border-radius:6px;padding:10px;
    display:flex;flex-direction:column;align-items:center;gap:6px;min-width:170px}
  .leg-label{font-family:'JetBrains Mono',monospace;font-size:13px;font-weight:700}
  .knob-row{display:flex;gap:10px}
  .knob-wrap{display:flex;flex-direction:column;align-items:center;gap:4px}
  .knob-lbl{font-size:10px;color:var(--muted)}
  canvas.knob{cursor:ns-resize;border-radius:50%}
  .zero-btn{font-size:10px;padding:2px 10px;border:1px solid var(--border);
    border-radius:4px;cursor:pointer;background:var(--card2);color:var(--muted)}
  .zero-btn:hover{border-color:currentColor}

  /* Send button */
  .send-btn{background:var(--blue);color:#000;font-weight:700;font-size:13px;
    border:none;border-radius:6px;padding:10px 28px;cursor:pointer;
    margin-top:8px;align-self:flex-end}
  .send-btn:hover{background:#79c0ff}

  /* Settings bar */
  .settings-row{display:flex;align-items:center;gap:16px;flex-wrap:wrap}
  .settings-row label{font-size:12px;color:var(--muted)}
  .settings-row input[type=range]{width:120px;accent-color:var(--teal)}
  .settings-row span{font-family:'JetBrains Mono',monospace;font-size:12px;color:var(--teal);min-width:32px}
  .prog-bar{width:200px;height:8px;background:var(--card2);border-radius:4px;overflow:hidden;margin-left:8px}
  .prog-fill{height:100%;background:var(--green);border-radius:4px;transition:width 0.3s;width:0%}

  /* Pose buttons */
  .pose-grid{display:flex;flex-direction:column;gap:4px}
  .pose-btn{padding:7px;border:none;border-radius:5px;cursor:pointer;
    font-weight:600;font-size:12px;text-align:left;background:var(--card2);
    transition:filter 0.15s}
  .pose-btn:hover{filter:brightness(1.3)}

  /* Record panel */
  .rec-row{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:8px}
  .rbtn{padding:5px 10px;border:none;border-radius:5px;cursor:pointer;
    font-size:11px;font-weight:700}
  .rbtn:hover{filter:brightness(1.2)}

  /* Timeline */
  .timeline{max-height:160px;overflow-y:auto;font-family:'JetBrains Mono',monospace;
    font-size:10px;background:var(--card2);border-radius:4px;padding:4px}
  .kf-row{padding:3px 6px;border-radius:3px;cursor:pointer;display:flex;justify-content:space-between}
  .kf-row:hover{background:var(--border)}
  .kf-row.active{background:rgba(88,166,255,0.2);color:var(--blue)}
  .kf-del{color:var(--red);font-weight:700;cursor:pointer;padding:0 4px}

  /* IO */
  .io-row{display:flex;gap:6px;align-items:center;margin-top:6px}
  .io-row input{flex:1;background:var(--card2);border:1px solid var(--border);
    border-radius:4px;padding:4px 8px;color:var(--text);font-size:11px}
  .io-btn{padding:4px 12px;border:none;border-radius:4px;cursor:pointer;
    font-size:11px;font-weight:600}
  .live-vals{display:flex;flex-wrap:wrap;gap:4px;font-family:'JetBrains Mono',monospace;font-size:10px}
  .live-chip{background:var(--card2);border-radius:3px;padding:2px 6px;color:var(--teal)}
</style>
</head>
<body>

<div class="topbar">
  <h1>🕷 SPIDERBOT</h1>
  <span class="sub">MoveIt Dashboard</span>
  <span class="status" id="status">Connecting…</span>
</div>

<div class="main">
 <div class="left">

  <!-- Joint Knobs -->
  <div class="card">
   <div class="card-hdr" style="color:var(--blue)">⚙ Joint Control — drag knob up/down or use slider</div>
   <div class="card-body">
    <div class="legs" id="legs"></div>
    <div style="display:flex;align-items:center;gap:12px;margin-top:10px">
     <button class="send-btn" onclick="sendAll()">▶ SEND ALL JOINTS</button>
     <button class="send-btn" style="background:var(--card2);color:var(--muted);font-size:11px"
             onclick="readLive()">⟳ Sync from Robot</button>
    </div>
   </div>
  </div>

  <!-- Settings -->
  <div class="card">
   <div class="card-hdr" style="color:var(--teal)">🗺 Execution Settings</div>
   <div class="card-body">
    <div class="settings-row">
     <label>Duration</label>
     <input type="range" id="dur" min="0.2" max="5" step="0.1" value="1.0"
            oninput="document.getElementById('dur-val').textContent=this.value+'s'">
     <span id="dur-val">1.0s</span>
     <label>Delay</label>
     <input type="range" id="delay" min="0" max="3" step="0.1" value="0.3"
            oninput="document.getElementById('delay-val').textContent=this.value+'s'">
     <span id="delay-val">0.3s</span>
     <div class="prog-bar"><div class="prog-fill" id="prog"></div></div>
     <span id="prog-lbl" style="font-family:'JetBrains Mono',monospace;font-size:11px;color:var(--green)"></span>
    </div>
   </div>
  </div>

  <!-- Live Feedback -->
  <div class="card">
   <div class="card-hdr" style="color:var(--teal)">📡 Live Joint States (degrees)</div>
   <div class="card-body">
    <div class="live-vals" id="live-vals">—</div>
   </div>
  </div>

 </div>
 <div class="right">

  <!-- Poses -->
  <div class="card">
   <div class="card-hdr" style="color:var(--purple)">📐 Named Poses</div>
   <div class="card-body"><div class="pose-grid" id="poses"></div></div>
  </div>

  <!-- Record -->
  <div class="card">
   <div class="card-hdr" style="color:var(--red)">⏺ Record / Replay</div>
   <div class="card-body">
    <div class="rec-row">
     <button class="rbtn" id="rec-btn" style="background:var(--red);color:#fff"
             onclick="toggleRec()">⏺ REC</button>
     <button class="rbtn" style="background:var(--card2);color:var(--muted)"
             onclick="stopRec()">⏹</button>
     <button class="rbtn" style="background:var(--green);color:#000"
             onclick="play(false)">▶ Play</button>
     <button class="rbtn" style="background:var(--amber);color:#000"
             onclick="play(true)">🔁 Loop</button>
    </div>
    <div style="font-family:'JetBrains Mono',monospace;font-size:10px;color:var(--muted);margin-bottom:6px">
     Frames: <span id="frame-cnt">0</span>
    </div>
    <button onclick="addKf()"
      style="width:100%;padding:5px;background:var(--card2);color:var(--blue);
             border:1px solid var(--blue);border-radius:4px;cursor:pointer;font-size:11px;margin-bottom:4px">
      + Add Keyframe
    </button>
    <button onclick="clearRec()"
      style="width:100%;padding:5px;background:var(--card2);color:var(--red);
             border:1px solid var(--red);border-radius:4px;cursor:pointer;font-size:11px">
      🗑 Clear Recording
    </button>
    <div class="io-row" style="margin-top:8px">
     <input type="text" id="fname" value="sequence.json" placeholder="filename.json">
     <button class="io-btn" style="background:var(--teal);color:#000"
             onclick="saveSeq()">💾</button>
     <button class="io-btn" style="background:var(--amber);color:#000"
             onclick="loadSeq()">📂</button>
    </div>
   </div>
  </div>

  <!-- Timeline -->
  <div class="card">
   <div class="card-hdr" style="color:var(--amber)">📋 Keyframe Timeline</div>
   <div class="card-body">
    <div class="timeline" id="timeline"></div>
   </div>
  </div>

 </div>
</div>

<script>
// ─── State ────────────────────────────────────────────────
const JOINTS = ['fr1','fr2','fl1','fl2','rr1','rr2','rl1','rl2'];
const LEGS   = {FR:[0,1],FL:[2,3],RR:[4,5],RL:[6,7]};
const LEG_COL= {FR:'#58a6ff',FL:'#3fb950',RR:'#d29922',RL:'#bc8cff'};
let vals     = new Array(8).fill(0);
let recording= false;
let canvases = {};

// ─── Build Leg Panels ────────────────────────────────────
function buildLegs(){
  const container = document.getElementById('legs');
  for(const [leg,[i1,i2]] of Object.entries(LEGS)){
    const col = LEG_COL[leg];
    const div = document.createElement('div');
    div.className='leg-panel';
    div.style.borderColor=col;
    div.innerHTML=`<div class="leg-label" style="color:${col}">${leg}</div>
      <div class="knob-row" id="knob-row-${leg}"></div>
      <button class="zero-btn" style="color:${col};border-color:${col}"
              onclick="zeroLeg('${leg}')">↺ Zero ${leg}</button>`;
    container.appendChild(div);

    for(const [idx,lbl] of [[i1,'Coxa'],[i2,'Tibia']]){
      const wrap = document.createElement('div');
      wrap.className='knob-wrap';

      const c = document.createElement('canvas');
      c.className='knob'; c.width=90; c.height=90;
      c.title='Drag up/down to rotate';
      canvases[idx]=c;

      // Slider below knob
      const sl = document.createElement('input');
      sl.type='range'; sl.min=-157; sl.max=157; sl.step=1; sl.value=0;
      sl.style.width='80px'; sl.style.accentColor=col;
      sl.oninput = ()=>{
        vals[idx]=sl.value/100;
        drawKnob(idx,col);
        document.getElementById(`val-${idx}`).textContent=
          (Math.round(vals[idx]*100/Math.PI*180))+'°';
      };

      const valLbl = document.createElement('div');
      valLbl.id=`val-${idx}`; valLbl.className='knob-lbl';
      valLbl.textContent='0°'; valLbl.style.color=col;

      // Mouse drag on canvas
      let dragY=null;
      c.addEventListener('mousedown',e=>{dragY=e.clientY; e.preventDefault()});
      window.addEventListener('mousemove',e=>{
        if(dragY===null) return;
        const dy=(dragY-e.clientY)*0.025;
        dragY=e.clientY;
        vals[idx]=Math.max(-1.57,Math.min(1.57,vals[idx]+dy));
        sl.value=Math.round(vals[idx]*100);
        valLbl.textContent=Math.round(vals[idx]*180/Math.PI)+'°';
        drawKnob(idx,col);
      });
      window.addEventListener('mouseup',()=>dragY=null);

      // Touch support
      c.addEventListener('touchstart',e=>{dragY=e.touches[0].clientY;e.preventDefault()},{passive:false});
      c.addEventListener('touchmove',e=>{
        const dy=(dragY-e.touches[0].clientY)*0.025;
        dragY=e.touches[0].clientY;
        vals[idx]=Math.max(-1.57,Math.min(1.57,vals[idx]+dy));
        sl.value=Math.round(vals[idx]*100);
        valLbl.textContent=Math.round(vals[idx]*180/Math.PI)+'°';
        drawKnob(idx,col); e.preventDefault();
      },{passive:false});

      wrap.appendChild(c);
      wrap.appendChild(sl);
      wrap.appendChild(valLbl);
      wrap.insertAdjacentHTML('beforeend',`<div class="knob-lbl">${lbl}</div>`);
      document.getElementById(`knob-row-${leg}`).appendChild(wrap);
      drawKnob(idx,col);
    }
  }
}

function drawKnob(idx,col){
  const c=canvases[idx]; if(!c) return;
  const ctx=c.getContext('2d');
  const cx=45,cy=45,r=36;
  ctx.clearRect(0,0,90,90);

  // Outer bg
  ctx.beginPath(); ctx.arc(cx,cy,r+4,0,Math.PI*2);
  ctx.fillStyle='#1c2128'; ctx.fill();
  ctx.strokeStyle='#30363d'; ctx.lineWidth=1; ctx.stroke();

  // Track arc (225° to -45°, 270° sweep)
  ctx.beginPath();
  ctx.arc(cx,cy,r, 225*Math.PI/180, -45*Math.PI/180);
  ctx.strokeStyle='#30363d'; ctx.lineWidth=5; ctx.lineCap='round'; ctx.stroke();

  // Value arc
  const norm=(vals[idx]+1.57)/3.14;
  const sweep=norm*270;
  if(sweep>0.1){
    ctx.beginPath();
    ctx.arc(cx,cy,r, 225*Math.PI/180,(225-sweep)*Math.PI/180, true);
    ctx.strokeStyle=col; ctx.lineWidth=5; ctx.lineCap='round'; ctx.stroke();
  }

  // Indicator dot
  const ang=(225-norm*270)*Math.PI/180;
  const ix=cx+(r-2)*Math.cos(ang), iy=cy-(r-2)*Math.sin(ang);
  ctx.beginPath(); ctx.arc(ix,iy,5,0,Math.PI*2);
  ctx.fillStyle=col; ctx.fill();

  // Centre
  ctx.beginPath(); ctx.arc(cx,cy,17,0,Math.PI*2);
  ctx.fillStyle='#1c2128'; ctx.fill();
  ctx.strokeStyle='#30363d'; ctx.lineWidth=1; ctx.stroke();

  // Value text
  const deg=Math.round(vals[idx]*180/Math.PI);
  ctx.fillStyle=col; ctx.font='bold 10px JetBrains Mono,monospace';
  ctx.textAlign='center'; ctx.textBaseline='middle';
  ctx.fillText((deg>=0?'+':'')+deg+'°',cx,cy);
}

function setVals(positions){
  positions.forEach((v,i)=>{
    vals[i]=v;
    const sl=document.querySelectorAll(`input[type=range]`)[i];
    if(sl) sl.value=Math.round(v*100);
    const vl=document.getElementById(`val-${i}`);
    if(vl) vl.textContent=Math.round(v*180/Math.PI)+'°';
    const leg=Object.entries(LEGS).find(([,pair])=>pair.includes(i));
    if(leg) drawKnob(i,LEG_COL[leg[0]]);
  });
}

// ─── Build Pose Buttons ──────────────────────────────────
function buildPoses(poses){
  const con=document.getElementById('poses');
  const cols={Stand:'#58a6ff',Sit:'#39d353',Home:'#8b949e',
    Stretch:'#bc8cff',Wave:'#f85149',Walk:'#d29922'};
  poses.forEach(name=>{
    const key=Object.keys(cols).find(k=>name.includes(k))||'Home';
    const btn=document.createElement('button');
    btn.className='pose-btn'; btn.style.color=cols[key];
    btn.textContent=name;
    btn.onclick=()=>sendPose(name);
    con.appendChild(btn);
  });
}

// ─── API calls ───────────────────────────────────────────
async function api(data){
  const r=await fetch('/',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify(data)});
  return r.json();
}

function sendAll(){
  const dur=parseFloat(document.getElementById('dur').value);
  api({action:'send',positions:vals,duration:dur})
    .then(d=>setStatus(d.status||'sent'));
  if(recording) addKf();
}

function sendPose(name){
  const dur=parseFloat(document.getElementById('dur').value);
  api({action:'pose',name,duration:dur}).then(()=>setStatus('Pose: '+name));
}

function zeroLeg(leg){
  const [i1,i2]=LEGS[leg];
  vals[i1]=vals[i2]=0;
  setVals(vals);
}

function readLive(){
  fetch('/state').then(r=>r.json()).then(d=>{
    if(d.live) setVals(d.live);
    setStatus('Synced from robot');
  });
}

// ─── Record ──────────────────────────────────────────────
let recActive=false;
function toggleRec(){
  recActive=!recActive;
  const btn=document.getElementById('rec-btn');
  btn.style.background=recActive?'#f85149':'#21262d';
  btn.textContent=recActive?'⏺ REC ●':'⏺ REC';
  setStatus(recActive?'RECORDING…':'Stopped');
  recording=recActive;
}
function stopRec(){recActive=false;recording=false;
  document.getElementById('rec-btn').style.background='#21262d';
  document.getElementById('rec-btn').textContent='⏺ REC';
  setStatus('Ready');
}

function addKf(){
  const dur=parseFloat(document.getElementById('dur').value);
  const delay=parseFloat(document.getElementById('delay').value);
  api({action:'record_add',positions:vals,duration:dur,delay})
    .then(d=>{ document.getElementById('frame-cnt').textContent=d.frames;
               refreshTimeline(); });
}

function clearRec(){
  api({action:'record_clear'}).then(d=>{
    document.getElementById('frame-cnt').textContent=0;
    document.getElementById('timeline').innerHTML='';
  });
}

function play(loop){
  api({action:'replay',loop}).then(d=>{
    setStatus(d.status==='empty'?'No frames!':'Replaying '+d.frames+' frames…');
    if(d.frames) animateProgress(d.frames);
  });
}

function animateProgress(total){
  let i=0;
  const bar=document.getElementById('prog');
  const lbl=document.getElementById('prog-lbl');
  const iv=setInterval(()=>{
    i++; bar.style.width=(i/total*100)+'%';
    lbl.textContent=i+'/'+total;
    if(i>=total){clearInterval(iv);
      setTimeout(()=>{bar.style.width='0%';lbl.textContent='Done';},1000);}
  },parseFloat(document.getElementById('dur').value)*1000+300);
}

async function refreshTimeline(){
  const d=await fetch('/state').then(r=>r.json());
  const tl=document.getElementById('timeline');
  tl.innerHTML='';
  (d.keyframes||[]).forEach((kf,i)=>{
    const row=document.createElement('div');
    row.className='kf-row';
    const degs=kf.pos.map(v=>Math.round(v*180/Math.PI)).join(' ');
    row.innerHTML=`<span>F${String(i+1).padStart(2,'0')} ${kf.dur}s  [${degs}]</span>
      <span class="kf-del" onclick="delKf(${i})">✕</span>`;
    tl.appendChild(row);
  });
  document.getElementById('frame-cnt').textContent=(d.keyframes||[]).length;
}

function delKf(idx){
  api({action:'record_delete',index:idx}).then(()=>refreshTimeline());
}

function saveSeq(){
  const fname=document.getElementById('fname').value||'sequence.json';
  api({action:'save',filename:fname}).then(d=>setStatus('Saved: '+d.saved));
}
function loadSeq(){
  const fname=document.getElementById('fname').value||'sequence.json';
  api({action:'load',filename:fname}).then(d=>{
    if(d.error) setStatus('Error: '+d.error);
    else{setStatus('Loaded: '+d.frames+' frames'); refreshTimeline();}
  });
}

// ─── Status + polling ────────────────────────────────────
function setStatus(msg){document.getElementById('status').textContent='⬤ '+msg;}

function pollState(){
  fetch('/state').then(r=>r.json()).then(d=>{
    if(d.ready) setStatus(d.status||'Connected ✓');
    const lv=document.getElementById('live-vals');
    const names=['FR Coxa','FR Tibia','FL Coxa','FL Tibia','RR Coxa','RR Tibia','RL Coxa','RL Tibia'];
    lv.innerHTML=d.live_deg.map((v,i)=>
      `<span class="live-chip">${names[i]}: ${v>=0?'+':''}${v}°</span>`
    ).join('');
  }).catch(()=>{});
  setTimeout(pollState,500);
}

// ─── Init ────────────────────────────────────────────────
fetch('/state').then(r=>r.json()).then(d=>{
  buildLegs();
  buildPoses(d.poses||[]);
  setStatus(d.ready?'Connected ✓':'Connecting…');
  pollState();
});
</script>
</body>
</html>
"""

# ──────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = RobotNode()

    # ROS spin in background
    spin_t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_t.start()

    # Wait for controller
    print('Connecting to controller...')
    for _ in range(20):
        if node._ac.server_is_ready():
            print('Controller ready!')
            node.status = 'Connected ✓'
            break
        time.sleep(0.5)
    else:
        print('Warning: controller not ready — starting anyway')

    # HTTP server
    Handler.node = node
    server = http.server.ThreadingHTTPServer(('0.0.0.0', PORT), Handler)

    print(f'\n{"="*50}')
    print(f'  SpiderBot Dashboard running!')
    print(f'  Open in browser: http://localhost:{PORT}')
    print(f'{"="*50}\n')

    # Auto-open browser
    threading.Timer(1.0, lambda: webbrowser.open(f'http://localhost:{PORT}')).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
