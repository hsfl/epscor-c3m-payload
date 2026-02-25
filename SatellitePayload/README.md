# Raspberry Pi Setup
Instructions for deploying `thermal_camera_controller.py` onto the Raspberry Pi and configuring it to run automatically on boot.

## Prerequisites
### Hardware
- Raspberry Pi (we use RPi Zero 2 W 4GB)
- USB Thermal Camera (FLIR Lepton or compatible UVC device with Y16 format)
- UART connection to Teensy via GPIO 14/15 - `/dev/serial0`

### Software / Python Dependencies
Make sure the following are installed on your RPi:
- Python 3.9+
    * numpy
    * pyserial
    * opencv-python
- [libuvc](https://github.com/libuvc/libuvc)


## 1. Copy files to the RPi

From your development machine, in the repo root:

```bash
scp SatellitePayload/thermal_camera_controller.py <rpi-user>@<rpi-ip>:~/
```
>Enter RPi password if prompted.

You also need `uvctypes.py` in the same directory. This file contains libuvc Python bindings. Make sure [libuvc](https://github.com/libuvc/libuvc) is installed on RPi. In repo root:

```bash
scp SatellitePayload/uvctypes.py <rpi-user>@<rpi-ip>:~/
```

## 2. Install Python Dependencies

On the RPi:

```bash
pip3 install numpy pyserial opencv-python
```
>note: you may need to run `sudo apt install python3-xyz` where xyz is the package you are trying to install.

## 3. Ensure UART is Enabled

The script communicates with the Teensy over `/dev/serial0` (GPIO 14/15). UART must be enabled on the RPi.

Run `raspi-config`:

```bash
sudo raspi-config
```

Navigate to: **Interface Options → Serial Port**
- "Would you like a login shell to be accessible over serial?" → **No**
- "Would you like the serial port hardware to be enabled?" → **Yes**

Reboot when prompted.

## 4. Create the systemd Service

Create the service file:

```bash
sudo nano /etc/systemd/system/thermal-camera.service
```

Paste the following:

```ini
[Unit]
Description=Thermal Camera Controller
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/thermal_camera_controller.py
WorkingDirectory=/home/pi
Restart=always
RestartSec=5
User=pi

[Install]
WantedBy=multi-user.target
```

> `Restart=always` and `RestartSec=5` ensure the script retries every 5 seconds if
> the camera is not yet plugged in or fails to initialize UART at boot.

## 5. Enable and Start the Service

```bash
sudo systemctl daemon-reload
sudo systemctl enable thermal-camera.service
sudo systemctl start thermal-camera.service
```

Verify it is running:

```bash
sudo systemctl status thermal-camera.service
```

You should see `Active: active (running)`.

## 6. Verify End-to-End

We can use the Teensy to verify end-to-end. Once the service is running, the RPi will send status messages to the Teensy over UART. If `DEBUG` mode is loaded onto the Teensy, it will print to serial:

```
STATUS:BOOT
STATUS:CAM_READY
STATUS:IDLE
```

The Teensy will forward these to the ground station as:

```
RPI STATUS: BOOT
RPI STATUS: CAM_READY
RPI STATUS: IDLE
```

The system is ready for thermal capture once `RPI STATUS: IDLE` appears at the ground station. The Teensy will not accept a capture command (`u`) until this status is received.

## Useful Service Commands

```bash
# View live logs
journalctl -u thermal-camera.service -f

# Restart the service
sudo systemctl restart thermal-camera.service

# Stop the service
sudo systemctl stop thermal-camera.service

# Disable autostart
sudo systemctl disable thermal-camera.service
```

If any changes are made to `thermal_camera_controller.py` or `thermal-camera.service`, make sure to daemon-reload and restart the service!

## Troubleshooting

**Script fails on boot but works manually:**
Check that UART is enabled (`raspi-config`) and that `/dev/serial0` exists:
```bash
ls -la /dev/serial*
```

**`uvctypes` import error:**
Ensure `uvctypes.py` is in `/home/pi/` alongside `thermal_camera_controller.py`.

**Camera not detected (`STATUS:NO_CAMERA`):**
- Check USB connection to the thermal camera
- The service will auto-restart every 5 seconds and retry camera initialization
- Run `lsusb` to confirm the camera is visible to the OS

**UART permission denied:**
Add the `pi` user to the `dialout` group:
```bash
sudo usermod -a -G dialout pi
```
Then reboot.
