## CU Hyperloop
![texasBullWorm](img/Alaskan_Bull_Worm.webp)

# Welcome to the CU Hyperloop Software Repository
# Table of Contents
1. [Installing Ubuntu 20.04 on UTM for MacOS](#virtual-machine-instructions)
2. [Installing Ubuntu 20.04 on VMware for Windows](#installing-ubuntu-2004-on-vmware-for-windows)
3. [Download Arduino IDE (Non MAC)](#download-arduino-ide-do-not-follow-if-you-have-an-m-chip-mac)
3. [Download Arduino IDE (MAC)](#download-arduino-ide-for-m-chip-mac)
4. [ROS Setup](#ros-setup)
5. [ROS Tutorials](#ROS-Tutorials)

# Virtual Machine Instructions
## Setting Up Ubuntu 20.04 VM on macOS using UTM

### Table of Contents

- [Prerequisites](#prerequisites)
- [Step 1: Download Ubuntu 20.04 ISO](#step-1-download-ubuntu-2004-iso)
- [Step 2: Open UTM](#step-2-open-utm)
- [Step 3: Create a New Virtual Machine](#step-3-create-a-new-virtual-machine)
- [Step 4: Configure the VM](#step-4-configure-the-vm)
- [Step 5: Install Ubuntu](#step-5-install-ubuntu)
- [Step 6: Install Desktop GUI](#step-6-install-desktop-gui)

---

### Prerequisites

- macOS machine
- UTM application installed
- Stable internet connection

---

### Step 1: Download Ubuntu 20.04 Server ISO for ARM

1. Click [this link](https://cdimage.ubuntu.com/releases/20.04/release/ubuntu-20.04.5-live-server-arm64.iso) to start the install. If it does not work visit[official Ubuntu website](https://cdimage.ubuntu.com/releases/20.04/release/) and download the Ubuntu 20.04 Server ISO file for ARM.

---

### Step 2: Download UTM

1. The Mac UTM download can be found [here](https://mac.getutm.app/)
2. Click the "Download" button. Note do not get the app store version it costs 10$
3. Once the UTM application is downloaded open it on your macOS device.

---

### Step 3: Create a New Virtual Machine

1. In UTM, click on the `+` button to create a new virtual machine.
2. Choose `Creating a new virtual machine` and click `Next`.
3. Choose the `Virtualize` option
4. Click `Linux`
5. Under the boot ISO image option press `Browse...`
6. Find the download of the Ubuntu 20.04 ISO and select it then press `open`

### Step 4: Configure the VM

**Hardware**: 

1. Make sure the hardware settings are set to these parameters:
    - Memory: Recommended at least `2048 MB`.
    - CPUs: At least `2`.
    
2. Click `Continue`

**Storage**:

1. Choose a size (Recommended at least `20 GB`).
2. Click `Save`.
3. Now click `Import Drive` and choose the downloaded Ubuntu 20.04 ISO file.
4. Click `Continue`
  
**Shared Directory**:

1. Click `Continue`
  
**Summary**:

1. Name the VM "HyperloopVM"
2. Click `Continue`

---

### Step 5: Install Ubuntu

1. Start your VM by selecting it and clicking `Start`.
2. You should see the Ubuntu installer. Follow the on-screen instructions to complete the installation (all default options will work).
3. Make sure to set the computer password to `cuhyperloop`
4. After the installation is complete, the VM should reboot into an ubunutu terminal. If it does not, close the vm, unmount the install iso and try again, it should work.

### Step 6: Install Desktop GUI
1. Since we installed the server version, you will only see a terminal, no gui.
2. Login to your vm.
3. Run `Install Desktop GUI`
4. Setup a display manager using `sudo apt install slim`
5. Install Ubuntu desktop using `sudo apt install ubuntu-desktop`
6. Install a vanilla gnome experience using `sudo apt install vanilla-gnome-desktop vanilla-gnome-default-settings`
7. Reboot the vm and you should now have a GUI desktop with `sudo reboot`. (If that does not work try closing the vm and opening it again).

---
# Installing Ubuntu 20.04 on VMware for Windows

## Overview
This guide provides step-by-step instructions for installing Ubuntu 20.04 LTS on VMware Workstation Player for Windows.

## Prerequisites
- VMware Workstation Player installed ([Download Here](https://cs-vsphere.int.colorado.edu/ui))
- Ubuntu 20.04 LTS ISO file ([Download here](https://releases.ubuntu.com/focal/))

## Steps

### 1. Open VMware Workstation Player
- Run VMware Workstation Player and click on "Create a New Virtual Machine."

### 2. Select ISO File
- Choose the "Installer disc image file (ISO)" option.
- Browse to the Ubuntu 20.04 LTS ISO file you downloaded.

### 3. Guest Operating System
- Choose `Linux` as the guest operating system.
- Select `Ubuntu 64-bit` as the version.

### 4. Virtual Machine Name and Location
- Name your virtual machine.
- Choose a location on your drive where you want to save it.

### 5. Specify Disk Capacity
- Choose how much disk space to allocate to the virtual machine.
  - Recommended: At least 20GB.

### 6. Customize Hardware (Optional)
- Click on "Customize Hardware" if you wish to allocate more or less RAM, CPU cores, or other hardware settings.

### 7. Finish Configuration
- Click on "Finish" to complete the virtual machine creation process.

### 8. Run Ubuntu
- Start the virtual machine.
- Follow the on-screen instructions to complete the Ubuntu installation.

### 9. Install VMware Tools (Recommended)
- Once Ubuntu is installed, it's recommended to install VMware Tools for better performance and usability.

## Conclusion
You should now have Ubuntu 20.04 running on a VMware virtual machine on your Windows system.

---

## Download Arduino IDE (DO NOT FOLLOW IF YOU HAVE AN M-CHIP MAC)
1. Follow All the directions on this [tutorial link.](https://lucidar.me/en/arduino/how-to-install-arduino-ide-2-0-on-ubuntu/) 
2. If you are adding the shortcut and the exec is correct try this command to grant the exec access to the Arduino ide:
```
chmod +x <path_to_arduino_ide_exc>
```
3. Go the the Teensyduino [website](https://www.pjrc.com/teensy/td_download.html) and follow the Teensyduino download instructions.
4. Go to [here](#install-ide-packages)

## Download Arduino IDE (FOR M-CHIP MAC)
1. Go [here](https://www.arduino.cc/en/software) and install the `Linux Arm 64 Bits`
Arduino IDE 1.8.19. DO NOT INSTALL the new IDE for mac, it will not work.
2. Follow these instructions `https://docs.arduino.cc/software/ide-v1/tutorials/Linux`
3. Go the the Teensyduino [website](https://www.pjrc.com/teensy/td_download.html) and follow the Teensyduino download instructions FOR ARDUINO 1.8.x.
4. Go to [here](#install-ide-packages)


## Install IDE Packages

1. Go to the Arduino lib foulder in GitHub and download libraries.zip
2. Open the Arduino IDE and go to File > Preferences.
3. Look for "Sketchbook location" to find the file location of the Arduino folder.
4. Go into a folder explore and find the Arduino folder.
5. Export the libraries folder from the zip into the Arduino folder.
6. Make sure the imported libraries folder is the only libraries folder in the Arduino folder.
7. Restart the Arduino IDE and all of the libraries should show up under the libraries tab.

# ROS Setup

1. Open your UTM VM and go to the Ubuntu Terminal
2. Tutorial link to ROS website for reference [link](http://wiki.ros.org/noetic/Installation/Ubuntu)

## Installing ROS Noetic on Ubuntu 20.04

### Overview
This guide will walk you through the steps to install ROS Noetic on a Ubuntu 20.04 system.

### Prerequisites
- Ubuntu 20.04 installed on your computer

---

### Steps

#### 1. Update System Packages
Before you start with the installation, it's generally a good idea to make sure your system packages are up-to-date.
```
sudo apt update
sudo apt upgrade
```

#### 2. Setup Sources and Add ROS GPG Key
ROS software is distributed via custom repositories. We need to add these repositories to our package manager, and also add the corresponding GPG keys for package verification.

##### Add ROS Repository
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

##### Add GPG Key
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key F42ED6FBAB17C654
```

#### 3. Update Package List
After adding the repository and GPG key, you should update your package list.

```
sudo apt update
```

#### 4. Install ROS Noetic

##### Full Installation
To install ROS Noetic with all features and packages, run the following command.

```
sudo apt install ros-noetic-desktop-full
```

#### 5. Initialize rosdep
`rosdep` is a command-line tool for installing system dependencies. Initialize it with the following command:

```
sudo rosdep init
rosdep update
```
If rosdep is not found then it needs to be installed manually with this command:
```
sudo apt-get install python3-rosdep
```
Now retry the first part of step 5.

#### 6. Environment Setup
To automatically add ROS environment variables to your shell session every time a new shell is launched:

```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Verification
To verify that ROS was installed correctly, you can run the following command which will show the installed ROS packages:

```
rospack list
```
If that doesn't work install the package manually:
```
sudo apt install rospack-tools
```
Now retry the verification.

# ROS Tutorials
Now go to `http://wiki.ros.org/ROS/Tutorials` and work through all the begineer tutorials.


And that's it! You have successfully installed ROS Noetic on your Ubuntu 20.04 system.


