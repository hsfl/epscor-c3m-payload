const portSelect = document.getElementById("port-select");
const portManual = document.getElementById("port-manual");
const connectBtn = document.getElementById("connect-btn");
const connStatus = document.getElementById("conn-status");
const consoleEl = document.getElementById("console");
const cmdForm = document.getElementById("cmd-form");
const cmdInput = document.getElementById("cmd-input");
const viewersEl = document.getElementById("viewers");

const captureBar = document.getElementById("capture-bar");
const captureLabel = document.getElementById("capture-label");
const requestBar = document.getElementById("request-bar");
const requestLabel = document.getElementById("request-label");

let connected = false;
const viewerCards = {}; // viewer -> {card, img, meta}
let requestResetTimer = null;

const VIEWER_TITLES = { rpicam: "RPi Camera Module 2", lepton: "FLIR Lepton", boson: "Boson Thermal" };

function logLine(text) {
  const atBottom = consoleEl.scrollHeight - consoleEl.scrollTop - consoleEl.clientHeight < 20;
  consoleEl.textContent += text + "\n";
  const lines = consoleEl.textContent.split("\n");
  if (lines.length > 500) {
    consoleEl.textContent = lines.slice(lines.length - 500).join("\n");
  }
  if (atBottom) consoleEl.scrollTop = consoleEl.scrollHeight;
}

function setConnected(isConnected, port) {
  connected = isConnected;
  connStatus.textContent = isConnected ? `Connected (${port})` : "Disconnected";
  connStatus.className = "status " + (isConnected ? "connected" : "disconnected");
  connectBtn.textContent = isConnected ? "Disconnect" : "Connect";
}

async function loadPorts() {
  const res = await fetch("/api/ports");
  const data = await res.json();
  portSelect.innerHTML = "";
  for (const p of data.ports) {
    const opt = document.createElement("option");
    opt.value = p.device;
    opt.textContent = `${p.device} — ${p.description}`;
    portSelect.appendChild(opt);
  }
  if (data.default) portSelect.value = data.default;
}

connectBtn.addEventListener("click", async () => {
  if (connected) {
    await fetch("/api/disconnect", { method: "POST" });
  } else {
    const port = portManual.value.trim() || portSelect.value;
    if (!port) return;
    const res = await fetch("/api/connect", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ port }),
    });
    if (!res.ok) {
      const err = await res.json();
      logLine(`[connect error] ${err.error}`);
    }
  }
});

document.getElementById("refresh-ports").addEventListener("click", loadPorts);

async function sendCommand(cmd) {
  const res = await fetch("/api/send", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ cmd }),
  });
  if (!res.ok) {
    const err = await res.json();
    logLine(`[send error] ${err.error}`);
  }
  return res.ok;
}

cmdForm.addEventListener("submit", (e) => {
  e.preventDefault();
  const cmd = cmdInput.value.trim();
  if (!cmd) return;
  sendCommand(cmd);
  cmdInput.value = "";
});

document.querySelectorAll(".quick-actions button:not(#stream-toggle)").forEach((btn) => {
  btn.addEventListener("click", () => sendCommand(btn.dataset.cmd));
});

function ensureViewer(viewer) {
  if (viewerCards[viewer]) return viewerCards[viewer];

  const card = document.createElement("div");
  card.className = "viewer-card";
  card.dataset.viewer = viewer;

  const head = document.createElement("div");
  head.className = "viewer-head";
  const h2 = document.createElement("h2");
  h2.textContent = VIEWER_TITLES[viewer] || viewer;
  const temperatures = document.createElement("span");
  temperatures.className = "viewer-temperatures";
  if (viewer === "lepton") h2.appendChild(temperatures);

  const actions = document.createElement("div");
  actions.className = "viewer-actions";

  const rotateBtn = document.createElement("button");
  rotateBtn.className = "rotate-btn";
  rotateBtn.textContent = "⟳ Rotate";
  rotateBtn.addEventListener("click", async () => {
    await fetch("/api/rotate", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ viewer, delta: 90 }),
    });
    updateViewerImage(viewer);
  });

  const closeBtn = document.createElement("button");
  closeBtn.className = "close-btn";
  closeBtn.textContent = "×";
  closeBtn.title = "Close viewer";
  closeBtn.addEventListener("click", () => {
    card.remove();
    delete viewerCards[viewer];
  });

  actions.appendChild(rotateBtn);
  actions.appendChild(closeBtn);
  head.appendChild(h2);
  head.appendChild(actions);

  const img = document.createElement("img");
  const meta = document.createElement("div");
  meta.className = "meta";

  card.appendChild(head);
  card.appendChild(img);
  card.appendChild(meta);
  viewersEl.appendChild(card);

  const entry = { card, img, meta, temperatures };
  viewerCards[viewer] = entry;
  return entry;
}

function updateViewerImage(viewer, sourceLabel, temperatures) {
  const entry = ensureViewer(viewer);
  if (viewer === "lepton" && temperatures !== undefined) {
    entry.temperatures.textContent = temperatures
      ? ` · Min ${temperatures.min.toFixed(1)} °C · Max ${temperatures.max.toFixed(1)} °C`
      : "";
  }
  entry.img.src = `/api/image/${viewer}?t=${Date.now()}`;
  const stamp = new Date().toLocaleTimeString();
  entry.meta.textContent = sourceLabel ? `${sourceLabel} · updated ${stamp}` : `updated ${stamp}`;
}

const streamViewer = document.getElementById("stream-viewer");
const streamImage = document.getElementById("stream-image");
const streamStatus = document.getElementById("stream-status");
const streamMeta = document.getElementById("stream-meta");
const streamClose = document.getElementById("stream-close");
const streamToggle = document.getElementById("stream-toggle");
let streamRotation = 0;
let latestStreamFrame = null;
document.getElementById("stream-rotate").addEventListener("click", () => {
  streamRotation = (streamRotation + 90) % 360;
  if (latestStreamFrame) updateStreamFrame(latestStreamFrame);
});
streamToggle.addEventListener("click", async () => {
  const cmd = streamToggle.dataset.cmd;
  const previousPhase = streamPhase;
  const couldClose = !streamClose.hidden;
  setStreamState(cmd === "stream start" ? "waiting" : "stopping");
  try {
    if (!await sendCommand(cmd)) setStreamState(previousPhase, couldClose);
  } catch (error) {
    logLine(`[send error] ${error.message}`);
    setStreamState(previousPhase, couldClose);
  }
});
streamClose.addEventListener("click", () => {
  if (streamPhase === "stopped" && !streamClose.hidden) streamViewer.hidden = true;
});
let streamLoading = false;
let pendingStreamFrame = null;
let lastStreamTime = 0;
let streamPhase = "stopped";

function setStreamState(phase, canClose = false) {
  streamClose.hidden = !(phase === "stopped" && canClose);
  streamPhase = phase;
  const active = ["waiting", "live", "stopping"].includes(phase);
  streamToggle.textContent = active ? "Stream Stop" : "Stream Start";
  streamToggle.dataset.cmd = active ? "stream stop" : "stream start";
  if (phase !== "stopped" || !streamViewer.hidden) streamViewer.hidden = false;
  streamStatus.textContent = phase === "stopping" ? "waiting for RPi idle" : phase;
  if (phase === "waiting") {
    streamImage.hidden = true;
    streamMeta.textContent = "Waiting for frames…";
  }
}

function updateStreamFrame(frame) {
  latestStreamFrame = frame;
  pendingStreamFrame = frame;
  if (streamLoading) return; // Fetch only the newest frame after the current load.
  streamLoading = true;
  pendingStreamFrame = null;
  streamImage.onload = () => {
    streamLoading = false;
    if (streamPhase === "live") {
      streamImage.hidden = false;
      lastStreamTime = Date.now();
      streamStatus.textContent = "live";
      streamMeta.textContent = `100 × 80 · frame ${frame.seq} · ${frame.packets} packets · ${new Date(frame.ts).toLocaleTimeString()}`;
    }
    if (pendingStreamFrame) updateStreamFrame(pendingStreamFrame);
  };
  streamImage.onerror = () => { streamLoading = false; };
  streamImage.src = `/api/stream/image?rotation=${streamRotation}&t=${Date.now()}`;
}

setInterval(() => {
  if (streamPhase === "live" && Date.now() - lastStreamTime > 10000) {
    streamStatus.textContent = "waiting for next frame";
  }
}, 1000);

function setProgressBar(barEl, labelEl, phase, percent) {
  labelEl.textContent = phase;
  barEl.classList.remove("indeterminate", "done", "error");
  if (phase === "capturing" || phase === "requesting") {
    barEl.classList.add("indeterminate");
  } else if (phase === "done") {
    barEl.classList.add("done");
    barEl.style.width = "100%";
  } else if (phase === "error") {
    barEl.classList.add("error");
    barEl.style.width = "100%";
  } else if (typeof percent === "number") {
    barEl.style.width = `${percent}%`;
  } else if (phase === "idle") {
    barEl.style.width = "0%";
  }
}

function connectEvents() {
  const es = new EventSource("/api/events");
  es.onmessage = (e) => {
    const msg = JSON.parse(e.data);
    handleEvent(msg.event, msg.data);
  };
  es.onerror = () => {
    // EventSource auto-reconnects; nothing to do here.
  };
}

function handleEvent(event, data) {
  switch (event) {
    case "log":
      logLine(data.text);
      break;
    case "connection":
      setConnected(data.connected, data.port);
      break;
    case "capture_progress":
      if (data.phase === "start") setProgressBar(captureBar, captureLabel, "capturing");
      else if (data.phase === "done") setProgressBar(captureBar, captureLabel, "done");
      else if (data.phase === "status" && /ERROR|NO_FRAME/.test(data.text || "")) {
        setProgressBar(captureBar, captureLabel, "error");
      }
      break;
    case "request_progress":
      clearTimeout(requestResetTimer);
      setProgressBar(requestBar, requestLabel, data.phase, data.percent);
      if (data.phase === "done") {
        requestResetTimer = setTimeout(() => setProgressBar(requestBar, requestLabel, "idle"), 1500);
      }
      break;
    case "stream_state":
      setStreamState(data.state, data.can_close);
      break;
    case "stream_frame":
      updateStreamFrame(data);
      break;
    case "image_ready":
      updateViewerImage(data.viewer, data.source, data.temperatures);
      break;
    default:
      break;
  }
}

async function loadInitialState() {
  const res = await fetch("/api/state");
  const state = await res.json();
  setConnected(state.connected, state.port);
  setProgressBar(captureBar, captureLabel, state.capture_state === "capturing" ? "capturing" : state.capture_state);
  setProgressBar(requestBar, requestLabel, state.request_state, state.request_percent);
  setStreamState(state.stream_state, state.stream_can_close);
  if (state.stream_frame) updateStreamFrame(state.stream_frame);
  if (state.viewers.rpicam) updateViewerImage("rpicam");
  if (state.viewers.lepton) updateViewerImage("lepton", null, state.lepton_temperatures);
  if (state.viewers.boson) updateViewerImage("boson");
}

loadPorts();
loadInitialState();
connectEvents();
