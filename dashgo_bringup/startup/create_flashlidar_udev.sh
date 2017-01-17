echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="flashlidar"' >/etc/udev/rules.d/flashlidar.rules

echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout",  SYMLINK+="flashlidar"' >/etc/udev/rules.d/flashlidar-V2.rules

service udev reload
sleep 2
service udev restart
