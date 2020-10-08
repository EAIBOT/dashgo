sudo sh create_dashgo_udev.sh
sudo sh create_flashlidar_udev.sh
----------------
cd 
sudo cp dashgo-start /usr/sbin/
sudo chmod +x /usr/sbin/dashgo-start


sudo cp dashgo-stop /usr/sbin/
sudo chmod +x /usr/sbin/dashgo-stop


sudo cp dashgo-restart /usr/sbin/
sudo chmod +x /usr/sbin/dashgo-restart

sudo cp  dashgo.service /lib/systemd/system/


----------
sudo systemctl start dashgo.service
sudo systemctl stop dashgo.service

systemctl is-enabled dashgo.service

sudo systemctl enable dashgo.service
sudo systemctl disable dashgo.service
