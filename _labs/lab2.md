---
layout: page
title: Lab 2
description: IMU
img: assets/img/lab1/artemis_nano.webp
importance: 1
---

## Lab 2

### Setup

Using the one of the QWIIC connector wires, I connected the IMU to the Artemis board.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/setup.jpeg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

I then ran the basic IMU example program under `File->Examples->SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library->Arduino->Example1_Basics`, the video below shows the accelorometer and gyroscope values updating as expected as I moved the IMU board! When I tilted the IMU so that its axes were parallel with gravity, I saw that the accelorometer measured the acceleration due to gravity on that axis. And when I rotated the IMU along its axes, I saw that the gyroscope measured the angular movement on that axis.

*** insert yt embed

To figure out the use of `AD0_VAL` constant in the code, I consulted the Sparkfun's [IMU webpage](https://learn.sparkfun.com/tutorials/sparkfun-9dof-imu-icm-20948-breakout-hookup-guide/all#hardware-overview), which explains what the `ADR` jumper on the back of the IMU chip does. By default, the jumper is open, and the I2C address of the IMU is 0x69. But when the jumper is closed the IMU's address is 0x68. So the constant in the code tells the library which address the IMU is at, and should be the default value of 1 since the jumper is open.

### Accelorometer

Next, I started to experiment with the IMU's accelorometer. Using the following equations from class, I calculated the IMU's pitch and roll based on measurements from the accelorometer.

$\text{Pitch: } \theta_a = \text{atan2}(a_x, a_z)$

$\text{Roll: } \phi_a = \text{atan2}(a_y, a_z)$

When the IMU is laid flat on the table (z-axis pointing straight up), its pitch and roll is 0.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/pitch_roll_0.jpeg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

When the IMU is held vertically such that its x-axis is pointing straight up, we get a pitch of +90. If the x-axis is pointing straight down we get a pitch of -90.

<div class="row">
    <div class="col-sm-6 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/pitch_pos_90.jpeg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-6 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/pitch_neg_90.jpeg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

When the IMU is held horizontally such that its y-axis is pointing straight up, we get a roll of +90. If the y-axis is pointing straight down we get a roll of -90.

<div class="row">
    <div class="col-sm-6 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/roll_pos_90.jpeg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-6 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/roll_pos_90.jpeg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

As seen from the images above, the accelorometer readings were highly accurate when I aligned the IMU with the side of my computer (off by at most half a degree). Therefore, I decided that a 2 point calibration would not be needed.

Moving the RC car back and forth next to the accelorometer, we see that there is a lot of noise in the pitch and roll data.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/accel_noise.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

**** insert c code

The above graph had 500 data points collected with a roughly 10ms gap between each data point. Using that, we can run a Fourier Transform on the data to see what frequencies are causing noise in our data. The images below show a huge spike in the 1-2 Hz range.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/accel_fft.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

<div class="row">
    <div class="col-sm">
        {% include figure.liquid loading="eager" path="assets/img/lab2/fft_code.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

To implement a low-pass filter, I used the data above to select a cutoff frequency of 3 Hz. This should significantly reduce the amount of noise in the system. For an angle $\theta$, the low pass filter is represented by the following.

$\theta_\text{LPF}[n] = \alpha\theta_\text{RAW} + (1 – \alpha)\theta_\text{LPF}[n-1]$

To compute $\alpha$, we know that $\alpha = \frac{T}{T + RC}$ and that $RC = \frac{1}{f_c \cdot 2\pi}$

So with $f_c = 3$ and $T = 0.01$, we get $RC = 0.053$ and $\alpha = 0.16$

*** insert c code

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/accel_lpf.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

### Gyroscope

Next, I started to experiment with the IMU's gyroscope. Using the following equations from class, I calculated the IMU's pitch, roll, and yaw based on measurements from the gyroscope. The pitch and yaw readings are inverted so that they match the axes of the accelorometer.

$\text{Pitch: } \theta_g = \theta_g - g_y \cdot dt$

$\text{Roll: } \phi_g = \phi_g + g_x \cdot dt$

$\text{Yaw: } \psi_g = \psi_g - g_z \cdot dt$

remove? --------------

As I started collecting data from the gyroscope, I saw that it tended to drift a lot, even while it was sitting still on the table. To attempt to lessen the drift, I collected 500 measurements while the IMU was sitting still and added this as an bias to each gyroscope reading.

<div class="row">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/pitch_drift.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-4 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/yaw_drift.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

-------

Looking at the comparison between the biased gyro values the filtered accelorometer values, we can see that the gyro values drift over time and even miss a few of the big movements which were picked up by the accelorometer. However, the gyro also picks up some angles changes quicker then the filtered accelorometer values. So to get the best of both worlds, we can use complimentary angles, which combines both the gyro and accelorometer readings using the following formula:

$\theta = (\theta + \theta_g)(1 − \alpha) + \theta_{a\_\text{LPF}}\alpha$

This works because we high-pass the gyro readings, suppressing the low-frequency drift, while low-passing the accelerometer data, suppressing its noisy readings. The $\alpha$ we use here is the same value as the one we used above to low-pass the accelerometer. As a result, the complimentary angle readings in the graph is accurate and not susceptible to drift or quick vibrations. I also tried adjusting the sampling frequency from 10ms to 50ms but this caused the gyro to drift more and made the complimentary slightly less accurate as a result.

<div class="row">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/accel_gyro.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-4 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/yaw.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>


### Sample Data


### Stunt