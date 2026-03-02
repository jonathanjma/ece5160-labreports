---
layout: page
title: Lab 4
description: Motors and Open Loop Control
img: assets/img/lab4/motor_driver.jpg
importance: 4
---

### Setup

To control the 2 motors on the car using the Artemis, we will need 4 analog pins in order to generate the PWM signals that the drivers use to power the motors. I ended up choosing pins A0 and A1 for the first driver and A2 and A3 for the second driver since these pairs of pins are right next to each other and also near the QWICC connector, allowing me to keep the wires organized while being easy to access. Below is the updated circuit diagram with the motor control circuitry added.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab4/full_circuit.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

For the battery, we want to power the Artemis and the motor drivers/motors with seperate batteries for 2 main reasons. First, the motors consume a lot of current, and if they draw too much, this could cause the Artemis to brownout and reset, terminating the program. Second, we are using brushed motors, so any wire which is connected to motors/motor drivers will have lots of electrical noise in it, which could also cause the Artemis to reset.

### Verifying Motor Driver Functionality

After soldering up the first motor driver, I verified the driver's functionality by connecting the battery pins of the motor driver to a power supply. I set the voltage of the power supply to 3.7 since that's the same voltage as the 850mAh battery that the motors will use when the car is fully assembled. Below is a photo of my power supply setup.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab4/power_supply_setup.jpeg" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

To test the PWN signals that I was generating using `analogWrite`, I connected the oscilloscope probe to one of the outputs. The below images were taken when the motor was set to a PWM value of 150, so roughly a 60% duty cycle.

<div class="row">
    <div class="col-sm-6 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab4/scope_setup.jpeg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-6 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab4/scope.jpeg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

After confirming that the driver was functional, I wrote short piece of code to command the motor forward for 2 seconds and backwards for seconds. The 2-motor version of the test code can be found below. The 2 videos below show both the left and right motor moving independently when connected to the power supply.

<div class="row">
    <div class="col-sm-6 mt-3 mt-md-0">
        <iframe width="100%" height="315" src="https://www.youtube.com/embed/7bhCypF-1F0?si=AOcmF1R8KF28yvGw" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>
    <div class="col-sm-6 mt-3 mt-md-0">
        <iframe width="100%" height="315" src="https://www.youtube.com/embed/c8e1Ezj_UQc?si=fjDCudjYyui0vomY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>
</div>

Now that I knew both motor drivers worked, I disconnected the power supply and soldered the motor input wires for both drivers to the leads of the battery connector. Running the following test program, the video below shows both motors spinning forward and backward. One thing to note about the code is that in order to drive forward one motor has to spin backward since the motors are installed on opposite sides of the robot, making the other one "flipped". Also when running this code I noticed that sometimes the wheels on the right side of the robot would move but on the left side they would not. This is because for some reason, the motor for the left side of the robot has to overcome a lot more static friction in order to start moving compared to the motor for the right side.

```c
void loop() {
    analogWrite(0, 150);
    analogWrite(1, 0);
    analogWrite(2, 0);
    analogWrite(3, 150);
    Serial.println("forward");
    delay(2000);

    analogWrite(0, 0);
    analogWrite(1, 150);
    analogWrite(2, 150);
    analogWrite(3, 0);
    Serial.println("backward");
    delay(2000);
}
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/kfySMyaBVs0?si=00mWS6dMJWkxDq11" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

The image below shows the final layout of my car after installing the motor drivers, IMU, ToF sensors, QWIIC breakout board, the Artemis, and the 2 batteries. In the image, the 2 motor drivers are hidden by the Artemis, and the 850mAh battery is located in the battery compartment which is accessed from the other side of the car.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab4/car_layout.png" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

### Calibrating the Motors

Since I noticed that both motors do not spin at the same rate, I implemented a calibration factor by commanding the robot to move forward for 2 seconds and adjusting the scale factor (`LR_conversion` in the code below) until the car drove completely straight. After doing a bunch of testing, my observations shows that in order for the left motor to spin at the same rate as the right motor, the code must multiply the desired PWM value by a factor of 1.6. I used the following Arduino code and commands to move the car forward/backward or turn left/right for 2 seconds at a time. The video below shows the car moving forward after implementing the calibration factor.

```c
case DRIVE_FORWARD: {
    forward();
    delay(2000);
    stop();
    break;
}
case DRIVE_BACKWARD: {
    backward();
    delay(2000);
    stop();
    break;
}
case TURN_LEFT: {
    left();
    delay(2000);
    stop();
    break;
}
case TURN_RIGHT: {
    right();
    delay(2000);
    stop();
    break;
}

...
void forward() {
    analogWrite(FW_LEFT, base_speed * LR_conversion); 
    analogWrite(BW_LEFT, 0);
    analogWrite(FW_RIGHT, base_speed); 
    analogWrite(BW_RIGHT, 0);
}

void backward() {
    analogWrite(FW_LEFT, 0); 
    analogWrite(BW_LEFT, base_speed * LR_conversion);
    analogWrite(FW_RIGHT, 0); 
    analogWrite(BW_RIGHT, base_speed);
}

void left() {
    analogWrite(FW_LEFT, 0); 
    analogWrite(BW_LEFT, base_speed * LR_conversion);
    analogWrite(FW_RIGHT, base_speed); 
    analogWrite(BW_RIGHT, 0);
}

void right() {
    analogWrite(FW_LEFT, base_speed * LR_conversion); 
    analogWrite(BW_LEFT, 0);
    analogWrite(FW_RIGHT, 0); 
    analogWrite(BW_RIGHT, base_speed);
}

void stop() {
    analogWrite(FW_LEFT, 0); 
    analogWrite(BW_LEFT, 0);
    analogWrite(FW_RIGHT, 0); 
    analogWrite(BW_RIGHT, 0);
}
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/rxCUAzYp7ys?si=ZQo9i1c-w-uci8o6" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Next, I tested the lowest PWM values which the robot can move forward and do an on-axis left turn. I found that to move forward, the lowest PWM value is 45, a 17.5% duty cycle, which I show in the video on the left. And to do an on-axis left turn, the lowest PWM value was 135, a 53% duty cycle, which I show in the video on the right.

<div class="row">
    <div class="col-sm-6 mt-3 mt-md-0">
        <iframe width="100%" height="315" src="https://www.youtube.com/embed/DUrZdeiEhus?si=EQZjU55Rsap8xwl6" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>
    <div class="col-sm-6 mt-3 mt-md-0">
        <iframe width="100%" height="315" src="https://www.youtube.com/embed/ZpgPqZvQ26E?si=DiLetgv5oDWFXp16" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>
</div>

To demonstrate open-loop control, I created the following command which would tell the car to follow a fixed sequence of moves: forwards, left spin, backwards, right spin, and forward again. The video below shows the car executing these moves.

```c
case DRIVE_SEQUENCE: {
    base_speed = 70;
    forward();
    delay(1000);
    base_speed = 180;
    left();
    delay(1000);
    base_speed = 70;
    backward();
    delay(750);
    base_speed = 180;
    right();
    delay(1000);
    forward();
    base_speed = 60;
    delay(500);
    stop();
    break;
}
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/PqKLgmr3xVg?si=_1pFp_Sic8q14uVf" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### 5000 Level Tasks

Looking at the oscilloscope reading, `analogWrite` produces a PWM output with a frequency of 184Hz. This seems to be on a low end since for a previous class I implemented PWM control in the kHz frequency. However this should be adaquate since the motor driver's datasheet states that it actually drives the motor at an internal PWM frequency of 50kHz. If we manually configure the timers to generate a faster PWM signal, this would allow us to generate smoother signals to the motor, especially once we start implementing closed loop PID control.

To find the lowest PWM value where we can keep the robot running once it is in motion, I created a command which sets the PWM output to 50, which is just above the minimum PWN we found earlier, for half a second to get the car moving. Then, I set the PWM output to a lower value for 2 seconds. After trying a bunch of values, I found the lowest PWM value to maintain motion is 15, a 5.8% duty cycle. This is shown in the video below.

```c
case FRICTION_SEQUENCE: {
    base_speed = speed1;
    forward();
    delay(500);
    base_speed = speed2;
    delay(2000);
    stop();
    break;
}
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/6aWXOLKctzc?si=6p6ncCrjYf6Q5N_u" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Acknowledgement: I referenced Aidan Derocher's website from Spring 2025 for inspiration