# Ping the Orange Pi's
if ping -c 1 orangepi1 &> /dev/null; then
  echo "Orange Pi 1 is online. Flashing..."
  ./flashTeensy.sh 1
else
  echo "Orange Pi 1 offline."
fi

if ping -c 1 orangepi2 &> /dev/null; then
  echo "Orange Pi 2 is online. Flashing..."
  ./flashTeensy.sh 2
else
  echo "Orange Pi 2 offline."
fi

if ping -c 1 orangepi3 &> /dev/null; then
  echo "Orange Pi 3 is online. Flashing..."
  ./flashTeensy.sh 3
else
  echo "Orange Pi 3 offline."
fi

if ping -c 1 orangepi4 &> /dev/null; then
  echo "Orange Pi 4 is online. Flashing..."
  ./flashTeensy.sh 4
else
  echo "Orange Pi 4 offline."
fi

if ping -c 1 orangepi5 &> /dev/null; then
  echo "Orange Pi 5 is online. Flashing..."
  ./flashTeensy.sh 5
else
  echo "Orange Pi 5 offline."
fi

if ping -c 1 orangepi6 &> /dev/null; then
  echo "Orange Pi 6 is online. Flashing..."
  ./flashTeensy.sh 6
else
  echo "Orange Pi 6 offline."
fi
