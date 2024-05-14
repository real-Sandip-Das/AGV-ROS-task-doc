# Documentation for ROS Installation

Resource followed: [Official Documentation Instructions for Noetic on Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)

## Installation Instructions

The Noetic Distribution of ROS was installed on Ubuntu 20.04.6 LTS(Focal Fossa) following the Official instructions

The One-Line Installation instructions were not followed even though it was given in the official documention

The Installation process was executed in a Gnome terminal window, and is summarized below:

```bash
# Configure Ubuntu repositories to allow "restricted", "universe", "multiverse"
sudo add-apt-repository restricted
sudo add-apt-repository universe
sudo add-apt-repository multiverse

# setup packages.ros.org repositories
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get -y install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

#ROS installation
sudo apt-get update
sudo apt-get -y install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc #Replace with ~/.zshrc if you use zsh instead of bash
source ~/.bashrc
sudo apt-get -y install python3-rosdep python3-rosinstall python3-rosinstall-generator \
python3-wstool build-essential #Dependencies for building packages
sudo rosdep init && rosdep update #Initializing rosdep
sudo apt install python3-catkin-tools python3-catkin-lint #Installing catkin
```

Further instructions for installing Turtlesim3 and Turtlebot3 packages have been given in the `README.md` of the repository's home directory for Task 2
