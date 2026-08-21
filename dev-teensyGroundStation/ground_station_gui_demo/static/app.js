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

const combinedPreview = document.getElementById("combined-preview");
const previewHint = document.getElementById("preview-hint");
const logoHint = document.getElementById("logo-hint");
const colorbarToggle = document.getElementById("colorbar-toggle");
const outreachForm = document.getElementById("outreach-form");
const visitorNameInput = document.getElementById("visitor-name");
const visitorEmailInput = document.getElementById("visitor-email");
const sendEmailBtn = document.getElementById("send-email-btn");
const savePhotoBtn = document.getElementById("save-photo-btn");
const sendStatus = document.getElementById("send-status");

let combinedReady = false;
let emailConfigured = false;
let requestResetTimer = null;

let connected = false;
const viewerCards = {}; // viewer -> {card, img, meta}

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
}

cmdForm.addEventListener("submit", (e) => {
  e.preventDefault();
  const cmd = cmdInput.value.trim();
  if (!cmd) return;
  sendCommand(cmd);
  cmdInput.value = "";
});

document.querySelectorAll(".quick-actions button").forEach((btn) => {
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
  head.appendChild(h2);
  head.appendChild(rotateBtn);

  const img = document.createElement("img");
  const meta = document.createElement("div");
  meta.className = "meta";

  card.appendChild(head);
  card.appendChild(img);
  card.appendChild(meta);
  viewersEl.appendChild(card);

  const entry = { card, img, meta };
  viewerCards[viewer] = entry;
  return entry;
}

function updateViewerImage(viewer, sourceLabel) {
  const entry = ensureViewer(viewer);
  entry.img.src = `/api/image/${viewer}?t=${Date.now()}`;
  const stamp = new Date().toLocaleTimeString();
  entry.meta.textContent = sourceLabel ? `${sourceLabel} · updated ${stamp}` : `updated ${stamp}`;
}

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
    case "image_ready":
      updateViewerImage(data.viewer, data.source);
      if (data.viewer === "rpicam" || data.viewer === "lepton") refreshCombinedPreview();
      break;
    default:
      break;
  }
}

function setCombinedReady(ready) {
  combinedReady = ready;
  combinedPreview.classList.toggle("hidden", !ready);
  previewHint.classList.toggle("hidden", ready);
  sendEmailBtn.disabled = !ready || !emailConfigured;
  savePhotoBtn.disabled = !ready;
}

function refreshCombinedPreview() {
  const img = new Image();
  img.onload = () => {
    combinedPreview.src = img.src;
    setCombinedReady(true);
  };
  img.onerror = () => setCombinedReady(false);
  img.src = `/api/image/combined?t=${Date.now()}`;
}

async function loadOutreachConfig() {
  const res = await fetch("/api/outreach/config");
  const config = await res.json();
  emailConfigured = config.email_configured;
  sendEmailBtn.disabled = !combinedReady || !emailConfigured;
  sendEmailBtn.title = emailConfigured ? "" : "Email is not configured (see .env.example)";
  const logosMissing = !config.logos.hsfl || !config.logos.c3m;
  logoHint.classList.toggle("hidden", !logosMissing);
  colorbarToggle.checked = config.colorbar_enabled;
}

colorbarToggle.addEventListener("change", async () => {
  await fetch("/api/outreach/colorbar", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ enabled: colorbarToggle.checked }),
  });
  if (combinedReady) refreshCombinedPreview();
});

function setSendStatus(text, kind) {
  sendStatus.textContent = text;
  sendStatus.className = "send-status" + (kind ? ` ${kind}` : "");
}

outreachForm.addEventListener("submit", async (e) => {
  e.preventDefault();
  const name = visitorNameInput.value.trim();
  const email = visitorEmailInput.value.trim();
  if (!email) return;

  sendEmailBtn.disabled = true;
  sendEmailBtn.textContent = "Sending…";
  setSendStatus("", null);

  try {
    const res = await fetch("/api/send-email", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name, email }),
    });
    const data = await res.json();
    if (!res.ok) throw new Error(data.error || "send failed");
    setSendStatus(`Sent to ${email}`, "ok");
  } catch (err) {
    setSendStatus(err.message, "error");
  } finally {
    sendEmailBtn.disabled = !combinedReady || !emailConfigured;
    sendEmailBtn.textContent = "Send Email";
  }
});

savePhotoBtn.addEventListener("click", async () => {
  savePhotoBtn.disabled = true;
  savePhotoBtn.textContent = "Saving…";
  setSendStatus("", null);

  try {
    const res = await fetch("/api/combined/save", { method: "POST" });
    const data = await res.json();
    if (!res.ok) throw new Error(data.error || "save failed");
    setSendStatus(`Saved to ${data.filename}`, "ok");
  } catch (err) {
    setSendStatus(err.message, "error");
  } finally {
    savePhotoBtn.disabled = !combinedReady;
    savePhotoBtn.textContent = "Save Photo";
  }
});

async function loadInitialState() {
  const res = await fetch("/api/state");
  const state = await res.json();
  setConnected(state.connected, state.port);
  setProgressBar(captureBar, captureLabel, state.capture_state === "capturing" ? "capturing" : state.capture_state);
  setProgressBar(requestBar, requestLabel, state.request_state, state.request_percent);
  if (state.viewers.rpicam) updateViewerImage("rpicam");
  if (state.viewers.lepton) updateViewerImage("lepton");
  if (state.viewers.boson) updateViewerImage("boson");
  if (state.viewers.combined) refreshCombinedPreview();
}

loadPorts();
loadInitialState();
loadOutreachConfig();
connectEvents();
