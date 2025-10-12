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
    printf "${YELLOW}WARN: file '$4' already exists. Overwrite? (y/n) ${NC}%s"
    read -r YN
  else
    YN="Y"
  fi
  if [ "$YN" = "y" ]||[ "$YN" = "Y" ]; then
    install -D -m $1 -o $2 -g $3 "./$4" $4
    printf "${GREEN}OK: Installed '$4${NC}%s' \n\n"
  fi
}

install_thingy 600 root root "/etc/netplan/00-bento-box.yaml"
install_thingy 600 root root "/etc/dnsmasq.d/00-bento-box.conf"
install_thingy 644 root root "/etc/systemd/network/80-can.network"


printf "${GREEN}set wifi region? (y/n)${NC}%s "
read -r YN
if [ "$YN" = "y" ]||[ "$YN" = "Y" ]; then
  RET=1
  while [ $RET != 0 ]; do
    printf "${GREEN}what region? (ISO/IEC 3166-1 alpha2, 2 char name)${NC}%s "
    read -r REG
    # We're running on raspi here, otherwise use `iw reg`
    raspi-config nonint do_wifi_country "${REG}"
    #sudo iw reg set "${REG}"
    RET=$?
  done
  sudo iw reg get | grep country
fi

# enable CAN networking
systemctl enable systemd-networkd
systemctl start systemd-networkd
printf "${GREEN}OK: enabled & started systemd-networkd \n"
# set up networking
netplan apply
printf "${GREEN}OK: configured networking \n"
# enable DHCP & DNS server
systemctl enable dnsmasq &>/dev/null #STFU sysV
systemctl start dnsmasq
printf "${GREEN}OK: enabled & started dnsmasq \n ${NC}"


