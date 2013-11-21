#!/bin/bash
xinput set-int-prop $(xinput --list | grep "OEM RFID Keyboard Emulator" | grep -oEi 'id=([0-9]+)' | grep -oEi '[0-9]+') "Device Enabled" 8 0

