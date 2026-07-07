# Meta Quest 3 Wired Connection (ADB Reverse Tethering) Setup Guide

This guide covers connecting a Meta Quest 3 to a PC (DGX Spark) via USB-C cable instead of WiFi for a Vuer (WebXR) based VR teleoperation system, using ADB reverse port forwarding to eliminate WiFi latency jitter.

## Background

- Vuer runs as a WebXR browser session, so the Quest Link Cable's PCVR streaming capability isn't needed. A standard USB-C data cable (USB 3.0 rated) is sufficient.
- The data Vuer exchanges (hand/controller pose, button states) is only a few KB/s, so bandwidth is never the bottleneck. What matters is latency **consistency (jitter)** — WiFi occasionally spikes in latency, which shows up as the robot arm stuttering during teleoperation.
- A USB-C Ethernet adapter (true wired LAN) is also possible, but it's an unofficial Meta feature with potential compatibility issues. ADB reverse tethering gives the best return for the setup effort involved.
- **This guide assumes the Quest 3 has no internet access at all** (fully local/offline network), so the connection uses the self-hosted local client rather than the hosted `vuer.ai` frontend.

---

## Part 1 — One-Time Setup

Do this once per machine. No need to repeat it for every session.

### 1. Install ADB on the PC (DGX Spark)

```bash
sudo apt update
sudo apt install android-tools-adb
```

### 2. Enable Developer Mode on Quest 3

Meta Quest mobile app → Headset settings → Developer Mode ON
(First-time setup may require creating an organization under a Meta Developer account.)

### 3. Connect Quest 3 to the PC via USB-C cable (first time)

Use a standard USB 3.0 data cable (charge-only cables may lack data lines).

After connecting, a prompt will appear inside the headset: **"Allow USB debugging?"** → check "Always allow" → Allow. Checking "Always allow" means you won't see this prompt again on future connections.

### 4. udev Rule Setup (Linux, fixes "no permission" error)

On Linux, without a udev rule granting non-root access to the USB device, `adb devices` will show `no permission`.

**Check the Vendor ID:**

```bash
lsusb | grep -i oculus
```

Meta/Oculus Quest 3's Vendor ID is `2833`.

**Create the udev rule file:**

```bash
sudo nano /etc/udev/rules.d/51-android.rules
```

Add the following:

```
SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0666", TAG+="uaccess"
```

**Apply the rule:**

```bash
sudo chmod a+r /etc/udev/rules.d/51-android.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Check/add the plugdev group:**

```bash
groups $USER
```

If `plugdev` isn't listed:

```bash
sudo usermod -aG plugdev $USER
```

**After adding the group, you must log out/back in or reboot** — group changes only take effect in a new session.

### 5. Verify the device is recognized

```bash
adb kill-server
adb start-server
adb devices
```

Expected output:

```
List of devices attached
2833:5013	device
```

If it shows `device` (not `no permission` or `unauthorized`), one-time setup is complete.

### 6. Make the tethering script executable

The `quest_tether.sh` script (see below) handles the steps you'll repeat every session. Make it executable once:

```bash
chmod +x quest_tether.sh
```

---

## Part 2 — Every Session

Do this each time you want to use the Quest 3 tethered to the PC.

### 1. Plug in the USB-C cable

Connect Quest 3 to the PC.

### 2. Start the Vuer server

```bash
grep -rn "port" your_vuer_script.py   # confirm the port if unsure — default is 8012
```

Run your teleoperation/Vuer script as usual.

### 3. Run the tethering script

```bash
./quest_tether.sh
```

This waits for the device and sets up `adb reverse` port forwarding (it resets every time the cable is unplugged, so this needs to run each session).

### 4. Open the connection in the Quest 3 browser

In Meta Quest Browser:

```
https://localhost:8012?ws=wss://localhost:8012
```

Use `localhost` (not the `192.168.x.x` IP used for WiFi connections) in both the page URL and the `ws=` parameter — ADB reverse forwards requests made to `localhost` inside the Quest straight through to the PC.

### 5. Handle certificate warnings

If using a self-signed certificate on your local server, the browser may show a security warning when the WebSocket connects → click **Advanced → Proceed (unsafe)**.

### 6. Enter the WebXR session

Click "Enter VR" → allow hand tracking permission → confirm the client connection appears in the Vuer server terminal log.

### 7. Test perceived latency

Move your hand quickly and compare the robot arm's response delay against the WiFi connection.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `adb devices` shows nothing | Charge-only cable (no data lines) | Replace with a USB 3.0 data cable |
| `unauthorized` | USB debugging prompt not confirmed in headset | Check the prompt while wearing the headset, or run `adb kill-server && adb start-server` |
| `no permission` | Missing udev rule | Follow the udev rule setup steps in Part 1 |
| Mixed-content error in Quest browser | Mixing `ws://` and `wss://` on the local page | Make sure the local Vuer server's WebSocket also uses `wss://`, matching the page's `https://` |
| Port forwarding lost after reconnecting cable | `adb reverse` resets on disconnect | Re-run `./quest_tether.sh` |
