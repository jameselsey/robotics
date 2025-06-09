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

How to set this project up
=========================

Prerequisites on the Pi5
----
Firstly setup SSH, so we can connect to the pi from our laptop.
```bash
sudo apt update
sudo apt install -y openssh-server
```

Then setup the SSH keys, which we add to github so we can push code to the pi
```bash
ssh-keygen -t ed25519 -C "james.elsey@gmail.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
cat ~/.ssh/id_ed25519.pub 
```
Copy the output of that public key, then add to your github, under `Settings > SSH and GPG keys > New SSH key`



Ros2 Setup
---
I took all these commands from the [official ros2 setup guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), I just added the `-y` and put them into a single script so it's easier to run, it also takes nearly an hour so run this and grab a coffee

```bash
sudo apt update 
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install -y ros-dev-tools

sudo apt update
sudo apt upgrade -y

sudo apt install -y \
  ros-jazzy-desktop \
  '~nros-jazzy-rqt*' \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-tf-transformations \
  ros-jazzy-xacro \
  ros-jazzy-foxglove-bridge \
  joystick \
  jstest-gtk \
  tree \
  python3-gpiozero \
  htop

```

Then run this to add ros to your terminal, this means you can run `ros2` when you login or open a new terminal
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

Robotics codebase 
---

Clone the repo
```bash
cd ~
git clone git@github.com:jameselsey/robotics.git
cd robotics
```

Build and run!
```bash
make build
make lauch
```



Cameras
---
To use a camera, we need to install and configure some additional packages.

```bash
sudo apt update
sudo apt install -y v4l-utils ros-jazzy-v4l2-camera

```





