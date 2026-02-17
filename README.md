# Customized Arduino-FFB-wheel for Guillemot Force Feedback Racing Wheel
## About this project
### Motivation
It gets more and more difficult to find working Windows 11 drivers for this wheel.

Even the old [Vista drivers](https://support.thrustmaster.com/en/product/ffdrw-en/) which worked fine for me some years ago only work with the iRacing FFB test program (WheelCheck.exe) anymore, but not with any of my games - they simply crash. I already replaced "System32/immpid.dll" with the one from "SysWOW64" and although this fixes the crashes I do not get any Force Feedback effects in games at all (the green effect LED on the Main PCB also does not blink).

There is also another old [driver on archive.org](https://archive.org/details/Guillemot-Thrustmaster-ForceFeedback-wheel-64bit-driver) which is supposed to work according to some forums. But even after disabling the Windows driver signature check the driver was marked as non-functional in the device manager and was not functional.

As a consequence I decided to reverse engineer the hardware and replace the Main PCB of the wheel with an Arduino Pro Micro board. This way it supports Force Feedback via USB HID and does not need any drivers at all.
The nice thing about this wheel is that the Motor Control PCB can be reused and only the Logic Board has to be replaced.

For the firmware I tried multiple projects, e.g.:
- [ranenbg/Arduino-FFB-wheel](https://github.com/ranenbg/Arduino-FFB-wheel)
- [vsulako/AFFBWheel](https://github.com/vsulako/AFFBWheel) (see my [unfinished fork here](https://github.com/tobigun/AFFBWheel))
- [YukMingLaw/ArduinoJoystickWithFFBLibrary](https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary)

Arduino-FFB-wheel was the most complete, so this project was used as the foundation.

### Game Compatibility Matrix
|Game|Store|Status|
|----|----|---|
|Assetto Corsa|Steam|OK
|Assetto Corsa Competizione|Steam|OK
|DiRT Rally 2.0|Steam|OK|
|Forza Horizon 4|Steam|OK. Requires the [Steering Wheel HID Profile](#hid-profiles), otherwise wheel will not be detected.|
|Forza Horizon 5|Steam|OK. Requires the [Generic HID Profile](#hid-profiles), otherwise game will crash at start.|
|NFS Heat|Steam|Only Steering, no FFB effects|
|NFS Hot Pursuit Remastered|Steam|OK|
|NFS Most Wanted|Steam|OK. Seems to support vibrations only, as FFB effects are week.|
|NFS Undercover|Steam|OK|


### Firmware
This repo is a fork of Miloš Ranković's [Arduino-FFB-wheel](https://github.com/ranenbg/Arduino-FFB-wheel), customized for the Guillemot Force Feedback Racing Wheel. As none of the pre-compiled binaries of the original project were suitable for this wheel, this project provides a configuration for the hardware and some minor software adjustments.

**Hardware configuration:**
- Analog X-Axis
- Hat Switch
- PWM motor control in 0-50-100 mode (hardcoded, so no need to configure it via the config software)
- Matrix keypad with 12 buttons
- Usage of 5 analog axis:
  - X-Axis (wheel angle)
  - Throttle
  - Brake
  - 2x analog levers on the wheel
- Force Feedback (of course)

### Original PCBs
The wheel consists of five PCBs:

- Main PCB, i.e. the logic board with the I-Force 2.0 chip:<br>
<img src="./docs/main-pcb-with-connectors.jpg" height="400px"/>

- Motor PCB (contains the PWM H-Bridge):<br>
<img src="./docs/motor-pcb.jpg" height="400px"/>

- Front Button PCB:<br>
<img src="./docs/button-pcb-front.jpg" height="400px"/>
<img src="./docs/button-pcb-back.jpg" height="400px"/>
<img src="./docs/button-pcb.jpg" height="400px"/>

- Gear Shift PCB:<br>
<img src="./docs/gear-pcb.jpg" height="400px"/>

- Power PCB (Note: already modified on the pictures). Contains power connector and switch, pedal connector and a digital switch (74HC4066). The 74HC4066 disables the two analog levers on the wheel when the pedals are connected.<br>
<img src="./docs/power-pcb.jpg" height="200px"/>
<img src="./docs/power-pcb2.jpg" height="200px"/>

All PCBs connected:
<img src="./docs/all-pcbs.jpg" height="400px"/>


### Arduino Main PCB
All of the original PCBs will be reused. The only PCB that will be replaced is the Main PCB. Instead an Arduino Pro Micro is used. I placed it on the Power PCB:
<img src="./docs/arduino-board.jpg" height="400px"/>

#### Wiring
Here is the overview of the connectors that are located on the Main PCB that now have to be connected to the Arduino board instead:
<img src="./docs/main-pcb.jpg" height="600px"/>

<table>
<tr><th colspan="3">Connector</th><th rowspan="2">Signal</th><th rowspan="2">Pin (ESP32-S3)</th><th rowspan="2">Pin (Pro Micro)</th><th rowspan="2">Comment</th></tr>
<tr><th>Name</th><th>Pin</th><th>Color</th></tr>

<tr><td rowspan="6">J5 (Motor PCB Connector)</td>
    <td>1</td><td style="background:green">green</td><td>Motor PWM</td><td>GP42</td><td>D9</td>
    <td rowspan="6">* Motor Enable switch must be connected to GP39/D10 via a BJT transistor to make sure that the motor is disabled when the microcontroller is powered down (i.e. not connected to USB).<br>
    This is necessary as the Motor PCB would erroneously interpret the missing PWM signal to apply maximum force. It would turn the wheel with full force in one direction and try to move it past the end stop. As this is not possible the motor gets quite hot after some minutes.
    <br>See schematic for more info.
    </td></tr>
<tr><td>2</td><td style="background:yellow">yellow</td><td>Motor Enable switch</td><td>[GP41]*</td><td>[D10]*</td></tr>
<tr><td>3</td><td style="background:orange">orange</td><td>GND</td><td colspan="2">GND</td></tr>
<tr><td>4</td><td style="background:red">red</td><td>GND</td><td colspan="2">GND</td></tr>
<tr><td>5</td><td style="background:brown">brown</td><td>GND</td><td colspan="2">GND</td></tr>
<tr><td>6</td><td style="background:black;color:white">black</td><td>+20V (from power supply)</td><td colspan="2">- (unused)</td></tr>

<tr><td rowspan="3">J6 (X-Axis Potentiometer)</td>
    <td>1</td><td style="background:orange">orange</td><td>VCC</td><td colspan="2">VCC</td><td rowspan="3"></td></tr>
<tr><td>2</td><td style="background:white">white</td><td>Analog X-Axis (0 .. VCC)</td><td>A0</td><td>A0</td></tr>
<tr><td>3</td><td style="background:green">green</td><td>GND</td><td colspan="2">GND</td></tr>

<tr><td rowspan="3">J7 (Y-Axis Potentiometer)</td>
    <td>1</td><td style="background:red">red</td><td>74HC4066 Supply Voltage</td><td colspan="2">VCC</td><td rowspan="3"></td></tr>
<tr><td>2</td><td style="background:blue;color:white">blue</td><td>Analog Y-Axis (0 .. VCC)</td><td>A1</td><td>A1</td></tr>
<tr><td>3</td><td style="background: repeating-linear-gradient(45deg,#eee,#eee 4px,#ccc 4px,#ccc 8px);">n.c.</td><td>-</td><td colspan="2">-</td></tr>

<tr><td rowspan="3">J8 (Z-Axis Potentiometer)</td>
    <td>1</td><td style="background:orange">orange</td><td>VCC</td><td colspan="2">VCC</td><td rowspan="3"></td></tr>
<tr><td>2</td><td style="background:brown">brown</td><td>Analog Z-Axis (0 .. VCC)</td><td>A3</td><td>A2</td></tr>
<tr><td>3</td><td style="background:green">green</td><td>GND</td><td colspan="2">GND</td></tr>

<tr><td rowspan="12">J12 (Button Matrix)</td>
    <td>1</td><td style="background:black;color:white">black</td><td>Matrix column 1</td><td>GP38</td><td>D5</td><td rowspan="12"></td></tr>
<tr><td>2</td><td style="background:brown">brown</td><td>Matrix column 2</td><td>GP37</td><td>D14</td></tr>
<tr><td>3</td><td style="background:red">red</td><td>Matrix column 3</td><td>GP36</td><td>D15</td></tr>
<tr><td>4</td><td style="background:orange">orange</td><td>Matrix column 4</td><td>GP35</td><td>D2</td></tr>
<tr><td>5</td><td style="background:yellow">yellow</td><td>Matrix row 1</td><td>GP17</td><td>D6</td></tr>
<tr><td>6</td><td style="background:green">green</td><td>Matrix row 2</td><td>GP33</td><td>D7</td></tr>
<tr><td>7</td><td style="background:blue;color:white">blue</td><td>Matrix row 3</td><td>GP18</td><td>D1 (TX0)</td></tr>
<tr><td>8</td><td style="background:#add8e6">light blue</td><td>Matrix row 4</td><td>GP34</td><td>D4</td></tr>
<tr><td>9</td><td style="background:#d3d3d3">light grey</td><td>Gear Shifter - Up (Matrix column 1)</td><td>GP38</td><td>D5</td></tr>
<tr><td>10</td><td style="background:grey">grey</td><td>Gear Shifter - Down (Matrix column 2)</td><td>GP37</td><td>D14</td></tr>
<tr><td>11</td><td style="background: repeating-linear-gradient(45deg,#eee,#eee 4px,#ccc 4px,#ccc 8px);">n.c.</td><td>-</td><td colspan="2">-</td></tr>
<tr><td>12</td><td style="background:white">white</td><td>Gear Shifter - Matrix row (Matrix row 2)</td><td>GP33</td><td>D7</td></tr>

<tr><td rowspan="4">J13 (Front LED)</td>
    <td>1</td><td style="background:#ff66cc">rose</td><td>LED Anode (+)</td><td>[GP40]*</td><td>[D3]*</td><td rowspan="4">* Connect LED's Anode via 820 Ohms resistor to pin.<br>See schematic for more info.</td></tr>
<tr><td>2</td><td style="background:black;color:white">black</td><td>LED Cathode (-)</td><td colspan="2">GND</td></tr>
<tr><td>3</td><td style="background: repeating-linear-gradient(45deg,#eee,#eee 4px,#ccc 4px,#ccc 8px);">n.c.</td><td>-</td><td colspan="2">-</td></tr>
<tr><td>4</td><td style="background: repeating-linear-gradient(45deg,#eee,#eee 4px,#ccc 4px,#ccc 8px);">n.c.</td><td>-</td><td colspan="2">-</td></tr>

<tr><td rowspan="4">JP101 (Analog Levers)<br>[Optional]</td>
    <td>1</td><td style="background:orange">orange</td><td>VCC</td><td colspan="2">VCC</td>
    <td rowspan="4">Wires can be unsoldered from connector JP101 of the power PCB and directly connected to the microcontroller.
    This way the pedals and levers can be used in parallel (5 instead of 3 axes). The Pins 2+3 on the JP101 connector on the power board should be pulled high then (i.e. connected to Pin 1).<br>See schematic for more info.</td></tr>
<tr><td>2</td><td style="background:red">red</td><td>Analog Axis - Lever left</td><td>A4</td><td>A3</td></tr>
<tr><td>3</td><td style="background:brown">brown</td><td>Analog Axis - Lever right</td><td>A5</td><td>A8</td></tr>
<tr><td>4</td><td style="background:green">green</td><td>GND</td><td colspan="2">GND</td></tr>

<tr><td rowspan="2">Blue Front LED<br>[Optional]</td>
    <td>A</td><td style="background:blue"></td><td>LED Anode (+)</td><td>[GP39]*</td><td>[D0]* (RX1)</td>
    <td rowspan="2">* Connect LED's Anode via 4.7 kOhms resistor to pin.<br>See schematic for more info.</td>
    </tr>
<tr><td>C</td><td style="background:black;color:white"></td><td>LED Cathode (-)</td><td colspan="2">GND</td></tr>

<tr><td rowspan="2">Green Front LED<br>[Optional]</td>
    <td>A</td><td style="background:green"></td><td>LED Anode (+)</td><td colspan="2">[+20V]*</td>
    <td rowspan="2">* Connect LED's Anode via 150 kOhms resistor to +20V.<br>See schematic for more info.</td>
    </tr>
<tr><td>C</td><td style="background:black;color:white"></td><td>LED Cathode (-)</td><td colspan="2">GND</td></tr>

<tr><td rowspan="2">Profile Switch<br>[Optional]</td>
    <td>1</td><td></td><td>Pin 1</td><td>GP7</td><td>D16</td>
    <td rowspan="2">For three pin switches: connect middle pin to D16 and any of the other two pins to GND.<br>
    </td></tr>
<tr><td>2</td><td style="background:black;color:white"></td><td>Pin 2</td><td colspan="2">GND</td></tr>
</table>

Make a little connector hub PCB to hold the motor switch transistor and  to connect all of the GND and VCC pins together. This way you can keep the existing connectors and you will still be able to reconnect them to the original Main PCB. But you can also cut the original connectors and solder the wires directly to the Arduino board.
<img src="./docs/connector-hub.jpg" height="400px"/>

#### HID Profiles
A Profile Switch can be added to select between to HID profiles:
- Switch open: Generic Joystick with generic X/Y/Z/Rx/Rz Axes (default)
- Switch closed: Steering wheel with Steering/Brake/Accelerator/Rx/Rz Axes

Switching profiles is necessary as there is no unified way to detect a Driving Wheel on Windows.
- Some games only detect a driving wheel if a Steering-Wheel Axis is present in the USB HID descriptor (e.g. Forza Horizon 4).
  - It seems that mostly older DirectInput based games require this profile as DirectInput uses a heuristic to detect wheels based on the axes of the controller ([see Wine DirectInput Port](https://github.com/wine-mirror/wine/blob/master/dlls/dinput/joystick_hid.c)).
- Some games require a generic X-Axis and do not work (or crash) when a Steering-Wheel Axis is present.
  - For example, Forza Horizons 5 crashes if a Steering-Wheel Axis is found.

##### Registry Tweak for Generic Profile
Instead of adding a switch to change the HID profile, for some games (like Forza Horizon 4) the Generic Profile also works if the following Windows registry keys are added via `regedit`:
```
Computer\HKEY_CURRENT_USER\System\CurrentControlSet\Control\MediaProperties\PrivateProperties\Joystick\OEM\VID_1B4F&PID_9206
Computer\HKEY_LOCAL_MACHINE\System\CurrentControlSet\Control\MediaProperties\PrivateProperties\Joystick\OEM\VID_1B4F&PID_9206
```
Add or modify the following value/data pair in these keys:
|Name|Type|Data
|----|----|----
|OEMData|REG_BINARY|43 00 88 01 fe 00 00 00

This can be achieved by double clicking on the `FFB_misc_programs/wheel-oemdata-add.reg` file.
The entries can be removed again with the `wheel-oemdata-remove.reg` file.

The bits of OEMData are described [here](https://sourceforge.net/p/timidity/git/ci/master/tree/windrv/mmddk.h#l230) but do not seem to be that important, as long as it starts with `43`.

See [Steam Community article on Forza Horizon 4 by LeadMagnet](https://steamcommunity.com/sharedfiles/filedetails/?id=2909923087) for more info.

> [!IMPORTANT]
> Make sure that the registry entries are removed again for Forza Horizons 5, as this game crahes if the entries are present.

------------------

# Arduino-FFB-wheel
A stand-alone DirectInput USB device is recognized in Windows as a joystick with force feedback functionality, based on BRWheel by Fernando Igor in 2017.

Firmware features:
- supported Arduino boards: Leonardo, Micro, and ProMicro (ATmega32U4, 5V, 16MHz)
- 4 analog axis + 1 for an optical or magnetic encoder, 2 FFB axis (with multichannel PWM or DAC output)
- for 2 FFB axis mode - 2 magnetic encoders may be used (for X and Y axis)
- automatic or manual analog axis calibration
- up to 16 buttons by 4x4 matrix or via **[button box firmware](https://github.com/ranenbg/Arduino-FFB-wheel/tree/master/tx_rw_ferrari_458_wheel_emu_16buttons)** uploaded to Arduino Nano/Uno
- up to 24 buttons by 3x8bit shift register chips
- analog XY H-pattern shifter (configurable to 6 or 8 gears + reverse gear, XY axis invert, reverse gear button invert)
- fully supported 16bit FFB effects (custom force effect not implemented)
- envelope and conditional block effects, start delay, duration, deadband, and direction enable
- FFB calculation and axis/button update rate 500Hz (2ms period)
- many firmware options (external 12bit ADC/DAC, automatic/manual pedal calibration, z-index support/offset/reset, hat switch, button matrix, external shift register, hardware wheel re-center, xy analog H shifter, FFB on analog axis)
- RS232 serial interface for configuring many firmware settings (10ms period)
- fully adjustable FFB output in the form of 4-channel digital 16bit PWM or 2-channel analog 12bit DAC signals
- available PWM modes: PWM+-, PWM+dir, PWM0-50-100, RCM (if 2 FFB axis: 2CH PWM+-, 2CH PWM+dir, 2CH PWM0-50-100, 2CH RCM)
- available DAC modes: DAC+-, DAC+dir, DAC0-50-100 (if 2 FFB axis: 1CH DAC+-, 2CH DAC+dir, 2CH DAC0-50-100)
- load cell support for 24bit HX711 chip (for Y axis only)
- all firmware settings are stored in EEPROM (and automatically loaded at each Arduino powerup)
- original wheel control user interface **[Arduino FFB gui](https://github.com/ranenbg/Arduino-FFB-gui)** for an easy configuration and monitoring of all inputs/outputs 

Detailed documentation and more information about the firmware can be found in txt files inside **[docs](https://github.com/tobigun/Arduino-FFB-wheel/tree/master/brWheel_my/docs)** folder.

# Firmware pinouts and wiring diagrams
![plot](./brWheel_my/wirings/Firmware-v250%20pinout.png)
**Note:** some pin mappings in the image are outdated. Check the wiring table above for the correct ones.
## Optical encoder and LED wiring
![plot](./brWheel_my/wirings/encoder_ffb_clip_led_wiring_diagram.png)
## Button matrix pinouts
![plot](./brWheel_my/wirings/button_matrix_wiring_diagram.png)
**Note:** the digital inputs for the matrix are not correct in the image. Check the wiring table above for the correct ones.

## Firmware upload procedure
You can use **[XLoader](https://github.com/tobigun/Arduino-FFB-wheel/tree/master/XLoader)**:
- set 57600baud, ATmega32U4 microcontroller and select desired HEX
- press the reset button on Arduino (or shortly connect the RST pin to GND)
- select the newly appeared COM port (Arduino in bootloader mode*) and press upload, you will only have a few seconds

*It is possible that some cheap Chinese clones of Arduino Leonardo, Micro, or ProMicro do not have a bootloader programmed. In that case you need to upload the original Arduino Leonardo bootloader first. You can find more details about it here: https://docs.arduino.cc/built-in-examples/arduino-isp/ArduinoISP

## How to compile the source
In order to compile the firmware yourself you may use Windows 11, you need to install Arduino IDE v2.3.6 and Arduino Boards v1.8.7. You must place the libraries (from the `libraries` folder) in your `.../documents/Arduino/Libraries` folder.

## Credits
- Miloš Ranković: [Arduino-FFB-wheel](https://github.com/ranenbg/Arduino-FFB-wheel)
- FFB HID and USB core for Arduino by: Peter Barrett
- BRWheel firmware by: Tero Loimuneva, Saku Kekkonen, Etienne Saint-Paul, and Fernando Igor
https://github.com/fernandoigor/BRWheel/tree/alphatest
