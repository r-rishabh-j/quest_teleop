#!/usr/bin/env bash
# Get the Quest ready for teleop over USB. Safe to run any number of times;
# run_sim.sh and run_robot.sh call it.
#
#   ./quest_prep.sh            # check the headset and turn the proximity sensor off
#   ./quest_prep.sh --restore  # turn the proximity sensor back on
#
# With the proximity sensor off the headset acts as worn: it stays awake and
# keeps tracking on a stand. This is the same broadcast MQDH sends for
# "Proximity Sensor: off". It lasts until the headset reboots or --restore.
# With several ADB devices, set ANDROID_SERIAL.
set -euo pipefail

APK=com.rail.oculus.teleop
APK_VERSION=1.1-teleopfix

command -v adb >/dev/null || { echo "[quest] adb not found: sudo apt install adb" >&2; exit 1; }
state=$(adb get-state 2>&1 || true)
case $state in
    device) ;;
    *unauthorized*)
        echo "[quest] Put the headset on and accept 'Allow USB debugging', then rerun." >&2; exit 1 ;;
    *permissions*)
        echo "[quest] No USB permission: add the udev rule for vendor 2833 (docs/TELEOP_SETUP.md)." >&2; exit 1 ;;
    *)
        echo "[quest] No Quest over USB (adb: $state). Check the cable; 'adb devices' must list it." >&2; exit 1 ;;
esac

if [[ ${1:-} == --restore ]]; then
    adb shell am broadcast -a com.oculus.vrpowermanager.automation_disable >/dev/null
    echo "[quest] proximity sensor back on"
    exit 0
fi

version=$(adb shell dumpsys package "$APK" | sed -n 's/.*versionName=//p' | head -1 | tr -d '\r')
if [[ -z $version ]]; then
    echo "[quest] $APK is not installed on the headset." >&2; exit 1
elif [[ $version != "$APK_VERSION" ]]; then
    echo "[quest] warning: $APK is $version, expected $APK_VERSION" >&2
fi

adb shell am broadcast -a com.oculus.vrpowermanager.prox_close >/dev/null
if adb shell dumpsys vrpowermanager | grep -q "Virtual proximity state: CLOSE"; then
    echo "[quest] proximity sensor off; headset stays awake"
else
    echo "[quest] warning: could not confirm the proximity sensor is off" >&2
fi
