# Bento-Box system files
*This directory contains the configuration files and setup guide for Bento-Box's system setup.*  
Versioning these files helps keep track of changes, and turns worst-case scenarios (for instance a dead SD card) from disqualifying events into something that can be solved in a few minutes.

> We recommend using whatever OS works best for your hardware,  
> but prefer Debian as it is designed for servers and therefore very stable and lightweight.  
> Docker gets around OS restrictions (such as ROS only distributing for ubuntu).


## Install software

> ℹ️ **internet required!**  
> you can test your connection using `ping google.com`

```shell
# Install software (everything after 'git' is optional)
sudo apt update
sudo apt install docker.io docker-compose dnsmasq git gh can-utils btop tree iperf3 -y  # presume that systemd-networkd is preinstalled, like on raspi
# say yes to auto-start iperf3
```
```shell
# Set up this Repo, Build image, and start container
cd ~/
git clone https://github.com/Bento-Robotics/Bento-Box.git
cd Bento-Box/system-files/  # set up host configs
sudo ./setup.sh
```
```shell

# Set up container
cd Bento-Box/container/
sudo docker compose up -d
cd ~/
```
```shell
# (Optional) Set up git
gh auth login
# follow instructions and log into github
git config --global user.email "bento.robotics@gmail.com"
git config --global user.name "Bento-Box"
```
```shell
# Set up user docker permissions
sudo usermod -aG docker $USER

# Apply changes
sudo reboot
```

## Install configuration
Files are laid out in the same way that they would be in the system.
> e.g. `etc/NetworkManager/xyz` → `/etc/NetworkManager/xyz`

⚠️ **reboot to make changes take effect**, both for automatic and manual

### Automatic
Use `./setup.sh` provided in this directory.
It does the exact same as the manual steps, just automatically.
It will ask for you password so it can use sudo to copy files into /etc.


### WiFi & Ethernet - NetworkManager
> /etc/NetworkManager/system-connections/*
```
sudo chmod 600 /etc/NetworkManager/system-connections/*
sudo chown root /etc/NetworkManager/system-connections/*
sudo chgrp root /etc/NetworkManager/system-connections/*
```

set country `sudo iw reg set DE` (or whatever county you are in)

### CAN - systemd-networkd
> /etc/systemd/network/80-can.network
```
sudo systemctl enable systemd-networkd
```

### DNS - dnsmasq

TODO
