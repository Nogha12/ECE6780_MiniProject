![Main](/assets/images/embedded.jpg)

# ECE 6780 Embedded System Design -- Mini-Project
**University of Utah, Electrical and Computer Engineering**

### Project Members:

- Noah Lomu
- David Venegas
- Skylar Stockham

### Description:

We will develop an LED driver that can interface with an off-the-shelf LED strip. The level of each color will be represented with an 8-bit integer representing 256 different PWM duty cycles (over 16 million colors in total). The LEDs’ color will be controlled by the onboard gyroscope; the color space of the LED strip will be mapped to the 3D space of the gyroscope’s orientation. The intensity of the LEDs can be controlled by the onboard touch sensor.

### Block Diagram:

![BlockDiagram](/assets/images/block.png)

### Milestones:

- **Milestone #1:** The driver can display any color on the LED strip.
- **Milestone #2:** The driver can map a normalized 3D vector to an RGB value.
- **Milestone #3:** The gyroscope can be read and used to control the LEDs.

### Linear Touch Sensor:

![TSC0](/assets/images/TSC/TSC_pad.png)

Capacitive sensing technology is able to detect finger presence near an electrode that is protected from direct touch by a dielectric (for example glass, plastic). The capacitive variation introduced by the finger (or any conductive object) is measured using a proven implementation based on a surface charge transfer acquisition principle.

![TSC1](/assets/images/TSC/Surface_charge_transfer.png)

The surface charge transfer acquisition is a proven, robust and efficient way to measure a capacitance. It uses a minimum number of external components to operate with a single ended electrode type. This acquisition is designed around an analog I/O group composed of up to four GPIOs. Several analog I/O groups are available to allow the acquisition of several capacitive sensing channels simultaneously and to support a larger number of capacitive sensing channels. Within a same analog I/O group, the acquisition of the capacitive sensing channels is sequential. One of the GPIOs is dedicated to the sampling capacitor CS. Only one sampling capacitor I/O per analog I/O group must be enabled at a time. The remaining GPIOs are dedicated to the electrodes and are commonly called channels. For some specific needs (such as proximity detection), it is possible to simultaneously enable more than one channel per analog I/O group.

One pin must be dedicated to the sampling capacitor, while up to three of the remaining pins can be used for touch sense pads:

**Channel:**

PA2: G1_IO3

**Electrodes:**

PA3: G1_IO4

PA7: G2_IO4

PB1: G3_IO3

**Pin GPIO configuration:**
- The capacitor pin must be configured as Open-Drain

- The capacitor pin also must have its Schmitt trigger function disabled.

- The sense pins must be configured as Push-Pull


The surface charge transfer acquisition principle consists of charging an electrode capacitance (CX) and transferring a part of the accumulated charge into a sampling capacitor (CS). This sequence is repeated until the voltage across CS reaches a given threshold (VIH in our case). The number of charge transfers required to reach the threshold is a direct representation of the size of the electrode capacitance.

**The measurement is performed as follows:**

- Manually discharge the capacitor by clearing the IODEF pin in the CR register. The recommended discharge time is 1 ms, but you could use a shorter time, depending on the sampling capacitor size.

- Set IODEF high if the pads are interlaced, or leave it cleared if they’re separate (for push-buttons). This controls all configured touch sensing pins other than the one channel (pad) currently used.

- Select the channel you want to sample in the IOCCR register. Only one channel can be enabled per group (or they will be added together), but you can sample up to all eight groups at once.
  
- Start the measurement loop using the START bit in the CR register.
  
- Wait for the completion flag, or an interrupt and read the counter values.

**For more detail into the implementation code of the Linear Touch Sensor in this project:**

-[TSC.c](<Core/Src/tsc.c>)

-[TSC.h](<Core/Inc/tsc.h>)
