#!/bin/bash
# quest_tether.sh
#
# Run this once per session, after plugging in the Quest 3 via USB-C and
# starting your Vuer server. Sets up ADB reverse port forwarding so the
# Quest can reach the Vuer server via localhost over the USB tether.
#
# See README.md ("Part 2 — Every Session") for full context.

set -e

echo "Waiting for Quest 3 connection..."
adb wait-for-device

echo "Setting up port forwarding..."
adb reverse tcp:8012 tcp:8012
adb reverse tcp:8080 tcp:8080   # remove this line if you don't use a separate data-stream port

echo "Setup complete. Current forwarding list:"
adb reverse --list

echo ""
echo "Open this URL in the Quest browser:"
echo "  https://localhost:8012?ws=wss://localhost:8012"
