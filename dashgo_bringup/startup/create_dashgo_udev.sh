echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", MODE:="0666", GROUP:="dialout",  SYMLINK+="dashgo"' >/etc/udev/rules.d/dashgo.rules

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout",  SYMLINK+="dashgo"' >/etc/udev/rules.d/ch34x.rules

service udev reload
sleep 2
service udev restart
