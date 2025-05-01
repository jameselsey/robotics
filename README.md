# robotics
Robotic experiments!

Joystick
----
Launch using `ros2 launch robotics joystick.launch.py`

Foxglove
----
Make sure you have foxglove installed. You can install it using the following command:
```bash
sudo apt install ros-jazzy-foxglove-bridge
```

Then run the foxglove bridge itself using `make launch-bridge`, this will just run the bridge and
you can then connect like `ws://pi5:8765`

It isn't particularly useful on it's own, so run the "bringup" which will launch all components, so you can control it and see sensor data etc

ros2 setup
---
I took all these commands from the [official ros2 setup guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), I just added the `-y` and put them into a single script so it's easier to run

```bash
locale  # check for UTF-8

sudo apt update 
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install -y ros-dev-tools

sudo apt update
sudo apt upgrade

sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-base \
  '~nros-jazzy-rqt*' \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-tf-transformations 

```

Then add `source /opt/ros/jazzy/setup.bash` into `~/.bashrc` and run `source ~/.bashrc`, this will mean that `ros2` is available in your terminal

Then install the `ros2` command line tools
```bash

Pi Setup
---
When setting this up on a fresh pi, you'll need to install a bunch of packages
```bash
sudo apt install -y \
  joystick \
  jstest-gtk \
  tree \
  python3-gpiozero \
  openssh-server \
  htop
```
We may or may not need these, depending how the bluetooth connection goes
```bash
sudo apt install -y \
  dkms \
  linux-headers-$(uname -r) \
  xboxdrv
```

Then setup an ssh key and grab the public key to put onto github
```bash
ssh-keygen -t ed25519 -C "james.elsey@gmail.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
cat ~/.ssh/id_ed25519.pub 
```
