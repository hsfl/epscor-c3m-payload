# Outreach Demo Runbook

Operating guide for `ground_station_gui_demo` at a public outreach event: capture a
visitor's rpicam + Lepton thermal photo, combine them into one keepsake image, and
email it to them. This is a separate copy of the production ground station GUI
(`ground_station_gui/`) — nothing here affects that one.

## Hardware setups

The GUI works with either of these. **No-RF is the recommended booth setup.**

| | **No-RF (recommended)** | **Full radio link** |
|---|---|---|
| Plugged into the laptop | Satellite Teensy, over USB | Ground station Teensy, over USB |
| Satellite Teensy firmware | `outreach_teensy_no_rf/outreach_teensy_no_rf.ino` | `SatellitePayload/satellite_teensy/satellite_teensy.ino` |
| Radio / ground station Teensy | Not used | Required |
| **Clear RPi Storage** | Works | Not supported (Teensy replies "Unknown command") |
| Connected cameras in **Status** | Shown | Not shown |

With the no-RF firmware, the satellite Teensy still splits each image into the same
CRC-checked packets as the radio link, but loops them back internally and prints
exactly what the ground station Teensy would. The GUI can't tell the difference.

## 1. One-time setup (do this before event day)

Do this once, ahead of time, on the laptop that will run the booth. Confirm it all
works before you're standing at a table with a line of visitors.

1. **Install dependencies** (into the existing venv):
   ```bash
   cd dev-teensyGroundStation
   source .venv/bin/activate   # or: .venv/bin/pip install ...
   pip install -r ground_station_gui_demo/requirements.txt
   ```

2. **Create your `.env`** from the template and fill in real values:
   ```bash
   cd ground_station_gui_demo
   cp .env.example .env
   ```
   Edit `.env`:
   - `SMTP_HOST` / `SMTP_PORT` / `SMTP_USE_SSL` — for Gmail: `smtp.gmail.com`, `465`, `true`.
   - `SMTP_USERNAME` / `SMTP_PASSWORD` — a real mailbox + an **app password** (not
     your normal login password — Gmail: Account → Security → App passwords).
   - `SMTP_FROM_NAME` / `SMTP_FROM_EMAIL` — what visitors see as the sender.
   - `EMAIL_SUBJECT` — the email subject line.
   - `PROJECT_WEBSITE_URL` — the link that goes in the email body.

   `.env` is gitignored — it stays on this laptop, never gets committed.

3. **Add the logo files** to `static/logos/`:
   - `hsfl_logo.png`
   - `c3m_logo.png`

   Transparent PNG recommended, roughly 60px tall (see `static/logos/README.md`).
   The app still works fine without these — the combined photo just won't have a
   logo in that corner — so it's OK to do this step later, but don't forget before
   the actual event.

4. **Edit the email wording** if you want, in `templates/email_body.txt`
   (placeholders: `{{ visitor_name }}`, `{{ website_url }}`). To change the
   keepsake image itself (header text, layout, colors), see
   [Customizing the keepsake image](#customizing-the-keepsake-image) below.

5. **Flash the satellite Teensy** (no-RF setup only). Open
   `outreach_teensy_no_rf/outreach_teensy_no_rf.ino` in the Arduino IDE, select
   **Teensy 4.1**, and upload. Or from the repo root:
   ```bash
   arduino-cli compile --upload --fqbn teensy:avr:teensy41 \
     dev-teensyGroundStation/ground_station_gui_demo/outreach_teensy_no_rf
   ```
   `rpi_uart.hpp` in that folder is a copy of
   `SatellitePayload/satellite_teensy/rpi_uart.hpp` (Arduino can't include files
   from outside the sketch folder). If the UART framing changes there, re-copy it.

6. **Update the Pi's camera script.** The Pi needs the `multicam_controller.py`
   that supports `CLEAR` (used by **Clear RPi Storage**). Power the Pi on
   (`rpi on`, wait for `RPI STATUS: IDLE`), then from the repo root:
   ```bash
   # which script does the service run? (note the ExecStart path)
   ssh artemis4@<rpi-ip> "systemctl cat boson-camera.service | grep -E 'ExecStart|WorkingDirectory'"

   # copy the controller + the camera modules it imports
   scp SatellitePayload/multicam_controller/{multicam_controller,lepton_camera,boson_camera,rpi_camera,uvctypes}.py artemis4@<rpi-ip>:~/
   ```
   If `ExecStart` still points at `boson_controller.py`, point it at the new file:
   ```bash
   ssh artemis4@<rpi-ip>
   sudo sed -i 's#/home/artemis4/boson_controller.py#/home/artemis4/multicam_controller.py#' /etc/systemd/system/boson-camera.service
   sudo systemctl daemon-reload
   ```
   Then restart and check:
   ```bash
   sudo systemctl restart boson-camera.service
   journalctl -u boson-camera.service -n 30   # should list "- CLEAR: Delete all queued captures"
   ```
   If the user or service name differs on your Pi, find it with
   `systemctl list-units | grep camera`.

7. **Dry-run without hardware** using the repo's serial simulator, to confirm
   email sending actually works before you're relying on it live:
   ```bash
   # terminal 1 — fake serial port pair
   socat -d -d pty,raw,echo=0,link=/tmp/ttyGS pty,raw,echo=0,link=/tmp/ttySIM

   # terminal 2 — stands in for the Teensy firmware
   cd dev-teensyGroundStation
   .venv/bin/python ground_station_sim/simulate_ground_station.py --port /tmp/ttySIM

   # terminal 3 — the demo app
   cd dev-teensyGroundStation/ground_station_gui_demo
   ../.venv/bin/python gds_app.py
   ```
   Open `http://127.0.0.1:5051`, connect to `/tmp/ttyGS`, click **Capture**, then
   **Request RPiCam** and **Request Lepton**. Once the combined preview shows up,
   send a test email to yourself and confirm it lands (check spam).

   The simulator doesn't report camera names during capture, so the plain
   **Request** button says "No captured cameras yet" here — use the per-camera
   buttons. **Status** and **Clear RPi Storage** aren't simulated either.

### Customizing the keepsake image

The emailed / saved photo is laid out like this:

```
┌──────────────────────────────────────────────────────┐
│ [HSFL logo]   HEADER_TEXT (title)        [C3M logo]  │
│               HEADER_SUBTITLE_TEXT                   │
│ ┌──────────────────────┐ ┌──────────────────┐ ┌──┐   │
│ │   RPi camera photo   │ │  Lepton thermal  │ │▓▓│ ← temperature scale
│ └──────────────────────┘ └──────────────────┘ └──┘   (optional toggle)
└──────────────────────────────────────────────────────┘
```

Almost everything is a constant at the top of **`imaging.py`**:

| To change… | Edit (in `imaging.py`) |
|---|---|
| Event title, e.g. for a new event or year | `HEADER_TEXT` |
| Subtitle line | `HEADER_SUBTITLE_TEXT` |
| Title / subtitle size | `HEADER_FONT_SIZE`, `HEADER_SUBTITLE_FONT_SIZE` |
| Title / subtitle color | `TEXT_COLOR`, `SUBTITLE_COLOR` (RGB tuples) |
| Background color | `BACKGROUND` |
| Photo size (height of both panels; width follows) | `TARGET_PANEL_HEIGHT` |
| Logo size | `HEADER_LOGO_HEIGHT` |
| Spacing | `MARGIN_PX` (outer edge), `GUTTER_PX` (between panels), `HEADER_GAP_PX` (header ↔ photos), `HEADER_LINE_GAP_PX` (title ↔ subtitle) |
| Arrangement (e.g. stack the photos vertically, move the logos) | `compose_visitor_image()` in the same file |

Elsewhere:

| To change… | Edit |
|---|---|
| Logo artwork | Replace `static/logos/hsfl_logo.png` / `c3m_logo.png` (same filenames) |
| Thermal color palette (default `"hot"`) | `cmap="hot"` in `_render_thermal_png()` and `_render_colorbar_png()` in `gds_app.py`; keep both the same so the scale matches the image |
| Temperature scale label | `label = ...` in `_render_colorbar_png()` in `gds_app.py` |
| Show the temperature scale by default | `combined_colorbar_enabled = False` in `gds_app.py` (visitors can also flip the **Show temperature scale** switch) |
| Rotation of each camera | The **⟳ Rotate** button on each viewer panel; it's saved to `rotation_state.json` |
| Attachment filename / email subject | `attachment_filename=` in `api_send_email()` in `gds_app.py` / `EMAIL_SUBJECT` in `.env` |

**After editing, restart the app** (`Ctrl+C`, then `gds_app.py` again); code is
only loaded at startup. Restarting clears the images held in memory, so you'll
need to capture again — or preview the layout with any two images, no hardware
needed:

```bash
cd dev-teensyGroundStation/ground_station_gui_demo
../.venv/bin/python -c "import imaging; open('/tmp/keepsake_preview.png','wb').write(imaging.compose_visitor_image(open('photo.jpg','rb').read(), open('thermal.png','rb').read()))" && open /tmp/keepsake_preview.png
```
Replace `photo.jpg` / `thermal.png` with any two image files (e.g. an earlier
`captures_demo/combined/` image works as a stand-in).

## 2. Event day

1. **Connect the hardware.**
   - *No-RF:* plug the satellite Teensy (running the outreach firmware) into USB.
     The Pi powers on automatically at boot.
   - *Full radio link:* plug the ground station Teensy into USB and power the
     satellite.

2. **Start the app**:
   ```bash
   cd dev-teensyGroundStation/ground_station_gui_demo
   ../.venv/bin/python gds_app.py
   ```
   Open `http://127.0.0.1:5051` in a browser. (The production GUI, if also
   running, uses port 5050 — the two don't conflict.)

3. In the GUI, **pick the Teensy's port** from the dropdown (or type it manually)
   and click **Connect**.

4. **Wait for `RPI STATUS: IDLE`** in the console before capturing. Capture and
   request are refused until then.

5. Click **Status** and confirm the expected cameras are listed, e.g.
   `Connected cameras: lepton (request 0), rpicam (request 2)` (no-RF only).
   If it says "unknown", the Pi booted before the Teensy — send `rpi off`, then
   `rpi on`, and wait for `IDLE` again.

6. On page load, confirm:
   - The **Send Email** button isn't greyed out (if it is, hover it — the
     tooltip will say email isn't configured, meaning `.env` is missing/incomplete).
   - No "logos not added yet" hint (if you skipped step 3 above).

### Teensy LED (no-RF firmware)

| LED | Meaning |
|---|---|
| Off | No power, or the first ~5 s of boot (waiting for USB) |
| Solid | Powered and idle |
| Blinking | Capturing, or downlinking + exporting an image |

## 3. Per-visitor flow

1. Click **Capture**. Wait for the Capture bar to show **done** — it finishes
   once *every* connected camera has been captured.
2. Click **Request**. It requests each camera from that capture, one at a time,
   in capture order (the console logs `[request sequence: lepton, rpicam]` and
   `[request sequence complete]`). A camera that fails doesn't stop the rest.
   - To fetch a single camera instead, use **Request Lepton** / **Request RPiCam**.
3. Wait for the "Send to Visitor" panel's preview image to update — this
   confirms both shots landed and the combined photo is ready.
4. Enter the visitor's **name** and **email**, click **Send Email**.
5. Watch the status line under the button: green = sent, red = shows the error
   (see Troubleshooting below).

If you just want a local copy without emailing, click **Save Photo** next to
**Send Email** — it writes to `dev-teensyGroundStation/captures_demo/combined/`
and shows the saved path in the status line.

There's no Export button: each request exports automatically when its download
finishes. To re-export the last image, type `export` in the command box.

## 4. Freeing space on the Pi

Every capture is saved on the Pi's SD card until it's deleted. If the Pi is
running low on space (or between events), clear it:

1. Click **Clear RPi Storage** (red button).
2. The browser asks *"Delete ALL captured images stored on the Raspberry Pi?
   This cannot be undone."* — click **OK** to delete, **Cancel** to back out.
3. While it runs, the button reads "Clearing RPi Storage…" and Capture / Request
   are disabled.
4. Done when the console shows `--- RPI STORAGE CLEAR COMPLETE ---`. The line
   before it reports what was freed, e.g.
   `RPI STATUS: CLEAR_DONE:24_FILES:1920KB:FREE_12000MB`.

From the command box (or the serial CLI) the equivalent is
`clear_storage confirm` — plain `clear_storage` only prints the warning.

Afterwards there's nothing left to request until the next **Capture**. Images
already downloaded to the laptop (`captures_demo/`) are not affected.

## 5. Troubleshooting

**Send Email button is greyed out**
`.env` is missing or incomplete. Check `GET /api/outreach/config` in a browser —
`email_configured: false` means one of `SMTP_HOST`/`SMTP_PORT`/`SMTP_USERNAME`/
`SMTP_PASSWORD`/`SMTP_FROM_EMAIL` isn't set. Restart the app after editing `.env`
(it's only read at startup).

**Send fails with an SMTP/auth error**
The status line shows the raw error (e.g. Gmail's "Username and Password not
accepted"). Usually means the app password is wrong/expired, or 2FA isn't
enabled on the sending account (Gmail requires it for app passwords). Generate a
fresh app password and update `.env`, then restart.

**Combined preview never shows up**
Needs *both* an RPiCam and a Lepton capture in the current session — check the
individual RPiCam/Lepton viewer panels are both showing images first. If a
sensor cable is loose or a request fails, redo `Request RPiCam` / `Request
Lepton` for that one.

**Request / Download bar turns red**
The request ended without an image. Scroll the console up for the reason:
- `Raspberry Pi is not ready yet` — wait for `RPI STATUS: IDLE`.
- `REQUEST_ERROR:NO_CAPTURE` — that camera has nothing stored (never captured,
  or storage was cleared). Click **Capture** first.
- `ERROR: UART header timeout` — the Pi didn't answer; check it's powered and
  its service is running.

**"No captured cameras yet - press Capture first"**
The **Request** button only knows the cameras from a capture done in this app
session. Click **Capture**, or use the per-camera Request buttons.

**Clear RPi Storage reports `--- RPI STORAGE CLEAR FAILED ---`**
Usually after a ~15 s pause: the Pi is running an old `multicam_controller.py`
without `CLEAR`. Redo setup step 6. If the console shows `CLEAR_ERROR:...`, some
files couldn't be deleted — check permissions on the Pi's `capture_queue/`.

**Status shows "Connected cameras: unknown"**
The Pi only announces its cameras when it boots. Send `rpi off`, then `rpi on`,
and wait for `RPI STATUS: IDLE`.

**No serial ports listed / can't connect**
Same as the base ground station GUI — check the Teensy is powered and plugged
in, try a different USB cable/port, and see the troubleshooting notes in
`dev-teensyGroundStation/README.md`.

**Corners of the combined photo are blank**
Logo file(s) missing from `static/logos/` — check `GET /api/outreach/config`'s
`logos` field, or just look in that folder for `hsfl_logo.png`/`c3m_logo.png`.

## 6. Shutdown

`Ctrl+C` the app. Captured images live in
`dev-teensyGroundStation/captures_demo/` (gitignored, safe to delete or archive
after the event), and no visitor PII is stored anywhere (names/emails are only
used in-memory to send, never written to disk). To wipe the copies stored on the
Pi as well, use **Clear RPi Storage** (section 4) before shutting down.
