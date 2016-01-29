#!/usr/bin/env bash

# append to bashrc
echo 'source $HOME/quaddard/scripts/setup.bash' >> $HOME/.bashrc

# set up networking
sudo cp interfaces /etc/network/interfaces
sudo cp wpa_supplicant.conf /etc/wpa_supplicant.conf