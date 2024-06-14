# Ping the Orange Pi's

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <Path to firmware.hex>"
    exit 1
fi

hexFilePath=$1

if ping -c 1 192.168.0.101 &> /dev/null; then
  echo "Orange Pi 1 is online. Flashing..."
  ./flashTeensy.sh 1 "$hexFilePath"
else
  echo "Orange Pi 1 offline."
fi

if ping -c 1 192.168.0.102 &> /dev/null; then
  echo "Orange Pi 2 is online. Flashing..."
  ./flashTeensy.sh 2 "$hexFilePath"
else
  echo "Orange Pi 2 offline."
fi

if ping -c 1 192.168.0.103 &> /dev/null; then
  echo "Orange Pi 3 is online. Flashing..."
  ./flashTeensy.sh 3 "$hexFilePath"
else
  echo "Orange Pi 3 offline."
fi

if ping -c 1 192.168.0.104 &> /dev/null; then
  echo "Orange Pi 4 is online. Flashing..."
  ./flashTeensy.sh 4 "$hexFilePath"
else
  echo "Orange Pi 4 offline."
fi

if ping -c 1 192.168.0.105 &> /dev/null; then
  echo "Orange Pi 5 is online. Flashing..."
  ./flashTeensy.sh 5 "$hexFilePath"
else
  echo "Orange Pi 5 offline."
fi

if ping -c 1 192.168.0.106 &> /dev/null; then
  echo "Orange Pi 6 is online. Flashing..."
  ./flashTeensy.sh 6 "$hexFilePath"
else
  echo "Orange Pi 6 offline."
fi
