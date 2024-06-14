# Setup Wireless Flashing

The process to set up wireless flashing requires ssh/sftp access to the Orange Pi and a laptop.

## Orange Pi Configuration

The most involved part of the wireless flashing setup is configuring the OPi itself.

**Complete the following steps via ssh'ing into your Orange Pi**

### 1. Clone the teensy flash cli repo

```
git clone https://github.com/PaulStoffregen/teensy_loader_cli.git
```

### 2. Build repo

```
cd teensy_loader_cli
sudo apt install gcc
sudo apt install libusb-dev
make
```

### 3. Configure UDEV rules for USB permissions

```
( sudo rm -f /tmp/00-teensy.rules /etc/udev/rules.d/00-teensy.rules /lib/udev/rules.d/00-teensy.rules && wget -O /tmp/00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules &&  sudo install -o root -g root -m 0664 /tmp/00-teensy.rules /lib/udev/rules.d/00-teensy.rules && sudo udevadm control --reload-rules && sudo udevadm trigger && echo "Success" )
```

### 4. Create .hex file destination

```
mkdir teensyCode
```

### 5. Reboot Orange Pi

While it is best practice to do this step, if you are in a pinch, we have at least 1 instance where we did NOT have to reboot the OPi use the wireless flashing

```
sudo reboot
```

# Using Wireless Flashing

On the laptop used to generate the .hex file and initiate file transfer, you have the option to flash a single teensy on an OPi or all teensy's that are online on the network. 

To flash a specific teensy:
```
./flashTeensy.sh <teensy number (1-6)> <path to .hex file>
```

To flash all teensys that are online:
```
./allFlashTeensy.sh <path to .hex file>
```
