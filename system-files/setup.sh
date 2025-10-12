#!/bin/bash

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

GREEN='\033[1;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# (permissions, user, group, destination)
# source is assumed to be "./destination"
install_thingy() {
  if [ -e $4 ]; then
    printf "${YELLOW}WARN: file '$4' already exists. Overwrite? (y/n)${NC}%s \n"
    read -r YN
  else
    YN="Y"
  fi
  if [ "$YN" = "y" ]||[ "$YN" = "Y" ]; then
    install -D -m $1 -o $2 -g $3 "./$4" $4
    printf "${GREEN}OK: $4${NC}%s \n"
  fi
}

install_thingy 600 root root "/etc/NetworkManager/system-connections/Bento-Box_WiFi.nmconnection"
install_thingy 600 root root "/etc/NetworkManager/system-connections/Bento-Box_Eth.nmconnection"
install_thingy 644 root root "/etc/NetworkManager/dnsmasq.d/00-bento-box.conf"
install_thingy 644 root root "/etc/NetworkManager/conf.d/00-use-dnsmasq.conf"
install_thingy 644 root root "/etc/systemd/network/80-can.network"
# seems to not be needed after DNS was fixed?
#install_thingy 755 root root "/etc/systemd/system/systemd-networkd-wait-online.service.d/override.conf"

# enable CAN networking
systemctl enable systemd-networkd

printf "${GREEN}set wifi region? (y/n)${NC}%s \n"
read -r YN
if [ "$YN" = "y" ]||[ "$YN" = "Y" ]; then
  RET=1
  while [ $RET != 0 ]; do
    printf "${GREEN}what region? (ISO/IEC 3166-1 alpha2, 2 char name)${NC}%s \n"
    read -r REG
    printf "\n"
    # We're running on raspi here, otherwise use `iw reg`
    raspi-config nonint do_wifi_country "${REG}"
    #sudo iw reg set "${REG}"
    RET=$?
  done
  sudo iw reg get | grep country
fi

