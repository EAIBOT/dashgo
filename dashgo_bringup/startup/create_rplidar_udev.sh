echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="rplidar"' >/etc/udev/rules.d/rplidar.rules

service udev reload
sleep 2
service udev restart
