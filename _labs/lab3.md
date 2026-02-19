---
layout: page
title: Lab 3
description: Time of Flight (ToF)
img: assets/img/lab3/tof.jpg
importance: 3
---

### Setup

For the car, I was given 2 ToF sensors. I decided it would be best to put one sensor at the front of the car and one on the right side of the car in order to maximize the coverage of obstacles. So to see obstacles on the back and left of the car, we would have to rotate the car. For the wiring, I decided to use the 2 longer QWIIC connect wires for the ToF sensors since they were more position sensitive than the IMU or the QWIIC breakout board (which will use the short wires). Below is a wiring diagram showing how the all the sensors will be connected. Since we have 2 ToF sensors which will both have the same default I2C address, we will connect one of the ToF's XSHUT inputs to the Artemis' GPIO pin. This allows us programmatically turn off one of the ToF sensors when the Artemis starts up so that we can change the other sensor's default I2C address.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/wiring.png" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

This lab involved a considerate amount of soldering, and the photos below shows the battery soldered with its new JST connector and one of ToFs soldered to its input signal and power wires.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/battery_soldered.jpeg" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/tof_soldered.jpeg" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

### Lab

I first started off by running the example program under `File->Examples->Apollo3->Example05_Wire_I2C` to verify the I2C address of the ToF sensor. As shown in the image below, the program prints an address of 0x29, which is different than the 0x52 address specified by the datasheet. But upon further analysis, the datasheet says the LSB of the address is the read/write bit, and 0x29 is just 0x52 shifted right a bit, so this makes sense.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/address.png" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

Short, Medium, and Long



### 5000 Level Tasks