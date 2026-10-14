# RTB-Swift to ESP32 Manipulator Control

This project combines advanced robotic simulation in Python with real-time hardware control via an ESP32 microcontroller. The system uses the **Robotics Toolbox for Python (RTB)** and the **Swift** simulator to calculate inverse kinematics (using analytical Jacobian and damped pseudo-inverse) of a 6-DOF example test robot and transmits joint angles to an ESP32 using the UDP protocol (mDNS) over a Wi-Fi network. The ESP32 physically drives the servomotors using a PCA9685 PWM driver.

## Project Structure

The workspace contains both the C++ firmware for the microcontroller and the Python scripts for the PC:

* `platformio.ini` and `src/main.cpp`: ESP32 firmware.
* `requirements.txt`: Python environment dependencies.
* `main.py`: UI rendering engine (Swift) and hardware abstraction (UDP communication).
* `ik_dls_rpy.py`: Mathematical controller (Jacobian calculation and state space error).
* `test_setup.py`: Initial test script to verify the simulator installation.

## Requirements

* [Visual Studio Code](https://code.visualstudio.com/)
* [Python 3.13](https://www.python.org/) (or compatible version)
* **PlatformIO IDE** extension for VS Code
* **Python** extension for VS Code

## Setup Part 1: Hardware (ESP32)

1. Open the project folder directly in VS Code.
2. Ensure the **PlatformIO** extension is installed. VS Code will automatically detect the `platformio.ini` file and initialize the project, downloading the `Adafruit PWM Servo Driver` library in the background.
3. Modify the `src/main.cpp` file by entering the credentials for the Wi-Fi network (`ssid` and `password`).
4. Connect the ESP32 via USB and click the **Upload** button (the right-pointing arrow icon in the bottom bar of PlatformIO) to flash the firmware onto the microcontroller.
5. Use the Serial Monitor to verify the Wi-Fi connection and the successful startup of the mDNS service (`braccio-robot.local`).

## Setup Part 2: Software (PC / Python)

To manage Python dependencies in an isolated and clean manner, it is highly recommended to use the built-in VS Code tool for creating a virtual environment (venv):

1. Open the VS Code Command Palette by pressing `Ctrl+Shift+P` (or `Cmd+Shift+P` on macOS).
2. Search for and select the **Python: Create Environment** command.
3. Choose the **Venv** option.
4. Select the Python 3.13 interpreter installed on the system.
5. VS Code will automatically generate the `.venv` directory and configure the environment for the current workspace.

Once the environment is created, open a new integrated terminal in VS Code (`Terminal -> New Terminal`). The terminal will automatically activate within the newly created virtual environment (indicated by the `(.venv)` prefix at the beginning of the command line).

Proceed to install the necessary libraries by executing the following commands in the terminal:

```bash
python -m pip install --upgrade pip
pip install -r requirements.txt
```

## Usage

1. **Environment Test:**
   Execute `python test_setup.py` to verify the correct operation of the mathematical libraries and the opening of the browser with the 3D rendering of the test robot (Puma560).

2. **Main Controller Startup:**
   Start the inverse kinematics algorithm with the command:
   ```bash
   python ik_dls_rpy.py
   ```
   The Swift simulator will launch in the default web browser. Use the sliders in the right panel to define the position and orientation of the target: the system will instantly calculate the tracking trajectory.

3. **Hardware Control (ESP32):**
   By default, data transmission to the ESP32 is disabled. To send UDP packets to the physical hardware, set the `send_esp32=True` flag within the main script or pass it as a startup parameter. The PC will begin streaming the joint coordinates to the address `braccio-robot.local:4210`.

## Electrical Warning

Under no circumstances should the servomotors (e.g., MG996R) be powered by drawing voltage from the 5V pins of the ESP32 or the computer's USB ports. Always use a dedicated external power supply (e.g., 5V 10A) connected exclusively to the green terminals of the PCA9685 driver board, making strictly sure to share the ground (GND) of the power supply with the ESP32 ground.