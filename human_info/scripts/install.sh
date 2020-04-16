#!/bin/sh
bash ../src/project-posenet/install_requirements.sh
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
apt-get update
apt-get install libedgetpu1-std python3-edgetpu
pip3 install -r ../src/requirements.txt
