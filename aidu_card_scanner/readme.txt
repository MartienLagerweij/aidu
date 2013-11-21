Create a user group called "rfid" and add yourself to it:

sudo groupadd -f rfid
sudo gpasswd -a YOUR_USERNAME rfid


Now copy the udev rules file 99-rfid.rules to the udev rule folder:

sudo cp 99-rfid.rules /etc/udev/rules.d/99-rfid.rules


This is sufficient to read the RFID keyboard emulator input. You might have to reboot for the changes to take effect.
