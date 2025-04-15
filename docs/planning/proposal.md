# Final project proposal

- [X] I have reviewed the project guidelines.
- [X] I will be working alone on this project.
- [X] No significant portion of this project will be (or has been) used in other course work.

## Embedded System Description

My embedded system will track the position of a hand in 3D space and how closed or open the fingers on the hand are. This will be accomplished with a combination of a gyroscope and a rotary encoder. The position and closedness of the fingers will be indicated on an LCD display and an LED bar array.

The first input of the system will be a rotary encoder. The output of the encoder will be filtered through an encoder converter and then passed into the master controller. The output from the encoder converter will be used to drive an up/down counter on the master controller.

The second input in the system will be a 6-axis gyroscope. The master controller will read the values on the gyroscope over I2C and calculate the gyroscope's position in 3D space.

The third input to the system will be a SPDT button that will act as a reset switch for the gyroscope and encoder.

Moving on to the outputs of the system, the first output will be an LCD display. The display will show the X, Y, and Z coordinates of the gyroscope and be driven by a slave microcontroller. The master controller will communicate the gyroscope's position to the slave controller over I2C.

The second output will be an LED bar array driven by another slave microcontroller. The master controller will direct the slave controller on how full the LED bar array should be based on the closedness of the finger according to the rotary encoder.

The third outputs will be status LEDs connected to the two slave controllers to indicate when they receive data over I2C.


## Hardware Setup

![circuit diagram](../assets/circuit_diagram.svg)

This project will require one MSP430FR2355 to act as the master controller of the system. Two MSP430FR2310's will act as drivers for an LED bar and an LCD display. A 6 axis gyroscope (MPU-6050) will be required to track the hand in 3D space. A quadrature encoder will be attached to a spring-loaded spool of string to measure the extension of a finger on the hand. A quadrature encoder converter (LS7183) will be used to convert the encoder reading into a usable, up/down signal. A voltage converter (MAXIM MAX1044) is required to drive the contrast of the LCD display. Additionally a SPDT switch will be used as a reset button and two red LEDs will be used to indicate when the LCD and LED bar drivers receive instructions.

## Software overview

![flowchat](../assets/main_flowchart.svg)

A timer interrupt will be used to periodically poll a gyroscope over I2C. Port interrupts will be used to sense when the encoder ticks up or down. Another port interrupt will be used to sense the H-to-L transition on the reset button which will return all the gyroscope and encoder positions to 0. The main loop of the master controller will send updated positions of the gyroscope and encoder to the LED bar and LCD driver when the positions change.

## Testing Procedure

A successful test will involve turning and moving the gyroscope and observing an updated output on the LCD display. Additionally pulling on the string connected to the encoder should update the fullness of the LED bar. At some point throughout the demo the reset button will be pressed and the LCD display should display all zeros and the LED bar will be unlit.


## Prescaler

Desired Prescaler level: 

- [ ] 100%
- [X] 95% 
- [ ] 90% 
- [ ] 85% 
- [ ] 80% 
- [ ] 75% 

### Prescalar requirements 

**Outline how you meet the requirements for your desired prescalar level**

**The inputs to the system will be:**
1.  6 axis gyroscope
2.  rotary encoder
3.  push button

**The outputs of the system will be:**
1.  LED bar
2.  LCD display
3.  LED status light x2 

**The project objective is**

To be able to track a hand in 3D space and track extension of fingers on said hand. This project will be a proof of concept so only one finger will be tracked.

**The new hardware or software modules are:**
New hardware:
1. I2C gyroscope
2. rotary encoder
3. Necessary hardware to mount gryoscope and encoder to hand

New software:
1. Handling and displaying real-time position of gyroscope
2. Up/Down counter on MSP430FR2355 for rotary encoder


The Master will be responsible for:

Handling input from the the rotary encoder and I2C gyroscope, performing calculation on the inputs, and sending the transformed data out to the LED bar and LCD display drivers over I2C.

The Slaves will be responsible for:

Displaying finger extension, as calculated by the master, on the LED bar. Additionally driving the LCD display to show gyroscope position/data sent from the master.



### Argument for Desired Prescaler

I believe this project meets all the requirements for the 95% prescaler due to the 3 inputs and 3 outputs. Although one of the inputs and outputs are basic, I believe the complexity of the other two inputs and outputs makes up for the basic ones. Additional challenge comes from designing and assembling the physical apparatus to hold the gyroscope and encoder. For these reasons, I believe a 95% scalar is appropriate.
