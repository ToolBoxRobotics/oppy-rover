# oppy-rover Robot_Operating_System_Noetic

## Install the Arduino IDE on Ubuntu

### Method 1: Use Snap (Recommended for simplicity)

1. Open a terminal.
2. Run the following command to install the Arduino IDE via Snap:

   ```bash
   sudo snap install arduino
   ```
4. Launch the IDE by searching for "Arduino" in your applications menu or by typing arduino in the terminal. 

### Method 2: Use APT (Quick installation) 

1. Open a terminal.
2. Update your package list and install the Arduino IDE:

   ```bash
   sudo apt update
   sudo apt install arduino
   ```
4. Launch the application by searching for "Arduino" in your applications menu. 

### Method 3: Download the AppImage (For the latest version) 

1. Go to the official Arduino IDE download page and download the AppImage for your system architecture (e.g., 64-bit).
2. Find the downloaded file (it will likely be in your Downloads folder).
3. Make the file executable. You can do this in two ways:
   1. Via the terminal: chmod +x arduino-ide_*.AppImage
   2. Via the file manager: Right-click the file, select "Properties," go to the "Permissions" tab, and check the box for "Allow executing file as program".
4. Double-click the AppImage file to launch the Arduino IDE. 

### Final step: Grant serial port access

1. After installing via any method, you may need to grant yourself access to the Arduino's serial port.
2. Open a terminal and run the following command:

   ```bash
   sudo usermod -a -G dialout $USER
   ```
4. Log out and log back in for the group changes to take effect. 




## ROS Noetic Rosserial Arduino

#### Install packages
```terminal
sudo apt-get install ros-noetic-rosserial-arduino
rosdep install rosserial_arduino
rosdep update
```

#### Lancez le roscore
```terminal
roscore
```
#### Exécutez l'application client rosserial qui transfère vos messages Arduino au reste de ROS. Assurez-vous d'utiliser le bon port série. 
```terminal
rosrun rosserial_python serial_node.py
```

#### Alternativement, si vous voulez pouvoir réinitialiser votre Arduino par programmation, exécutez en utilisant
```terminal
rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0
```


