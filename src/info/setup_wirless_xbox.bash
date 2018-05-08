echo "Setting up wireless xbox controller settings"
cd /sys/module/bluetooth/parameters
echo "Changing [/sys/module/bluetooth/parameters/disable_ertm] permissions"
sudo chmod o=rw disable_ertm
echo "Setting [disable_ertm] to [Y]"
sudo echo 1 >disable_ertm 
read -p "Done, press enter to exit..." foo 

