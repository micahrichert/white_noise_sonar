sudo apt-get install gcc-arm-none-eabi
sudo apt-get install binutils-arm-none-eabi
#sudo apt-get install cmake build-essential

# this is to access serial port
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER

# copy stlink & morsa udev rules
sudo cp ./49-lowsheen.rules /etc/udev/rules.d/49-lowsheen.rules
sudo udevadm control --reload-rules

sudo ln -s /usr/bin/arm-none-eabi-gcc-ar /usr/bin/arm-none-eabi-ar

