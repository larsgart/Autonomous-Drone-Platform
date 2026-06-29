#!/bin/sh
echo "updating apt-get"
apt-get update && sudo apt-get upgrade && sudo apt-get clean
echo "installing python3-pip"
apt-get install python3-pip
echo "stopping nvgetty.service"
systemctl stop nvgetty.service
echo "disabling nvgetty.service"
systemctl disable nvgetty.service
echo "enabling UART connection"
sed -i '10s/^/#/' /boot/extlinux/extlinux.conf
echo "cloning Autonomous-Drone-Platform repository"
git clone https://github.com/LarsGart/Autonomous-Drone-Platform.git
echo "installing drone package and dependencies"
pip3 install -e Autonomous-Drone-Platform
echo "restarting system"
reboot now
