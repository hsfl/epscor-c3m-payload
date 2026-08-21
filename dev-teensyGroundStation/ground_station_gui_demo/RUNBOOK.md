# Outreach Demo Runbook

Operating guide for `ground_station_gui_demo` at a public outreach event: capture a
visitor's rpicam + Lepton thermal photo, combine them into one keepsake image, and
email it to them. This is a separate copy of the production ground station GUI
(`ground_station_gui/`) — nothing here affects that one.

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
   (placeholders: `{{ visitor_name }}`, `{{ website_url }}`).

5. **Dry-run without hardware** using the repo's serial simulator, to confirm
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

## 2. Event day

1. **Connect the real hardware**: plug the ground station Teensy into USB.

2. **Start the app**:
   ```bash
   cd dev-teensyGroundStation/ground_station_gui_demo
   ../.venv/bin/python gds_app.py
   ```
   Open `http://127.0.0.1:5051` in a browser. (The production GUI, if also
   running, uses port 5050 — the two don't conflict.)

3. In the GUI, **pick the Teensy's port** from the dropdown (or type it manually)
   and click **Connect**.

4. On page load, confirm:
   - The **Send Email** button isn't greyed out (if it is, hover it — the
     tooltip will say email isn't configured, meaning `.env` is missing/incomplete).
   - No "logos not added yet" hint (if you skipped step 3 above).

## 3. Per-visitor flow

1. Click **Capture**.
2. Click **Request RPiCam**, then **Request Lepton** (order doesn't matter, but
   both are needed).
3. Wait for the "Send to Visitor" panel's preview image to update — this
   confirms both shots landed and the combined photo is ready.
4. Enter the visitor's **name** and **email**, click **Send Email**.
5. Watch the status line under the button: green = sent, red = shows the error
   (see Troubleshooting below).

If you just want a local copy without emailing, click **Save Photo** next to
**Send Email** — it writes to `dev-teensyGroundStation/captures_demo/combined/`
and shows the saved path in the status line.

## 4. Troubleshooting

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

**No serial ports listed / can't connect**
Same as the base ground station GUI — check the Teensy is powered and plugged
in, try a different USB cable/port, and see the troubleshooting notes in
`dev-teensyGroundStation/README.md`.

**Corners of the combined photo are blank**
Logo file(s) missing from `static/logos/` — check `GET /api/outreach/config`'s
`logos` field, or just look in that folder for `hsfl_logo.png`/`c3m_logo.png`.

## 5. Shutdown

`Ctrl+C` the app. No cleanup needed — captured images live in
`dev-teensyGroundStation/captures_demo/` (gitignored, safe to delete or archive
after the event), and no visitor PII is stored anywhere (names/emails are only
used in-memory to send, never written to disk).
