---
layout: page
title: Lab 2
description: Inertial Measurement Unit (IMU)
img: assets/img/lab2/imu.webp
importance: 2
---

### Setup

Using the one of the QWIIC connector wires, I connected the IMU to the Artemis board.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/setup.jpeg" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

I then ran the basic IMU example program under `File->Examples->SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library->Arduino->Example1_Basics`, and the video below shows the accelorometer and gyroscope values updating as expected as I moved the IMU board! When I tilted the IMU so that its axes were parallel with gravity, I saw that the accelorometer measured the acceleration due to gravity on that axis. And when I rotated the IMU along its axes, I saw that the gyroscope measured the angular movement on that axis.

<iframe width="560" height="315" src="https://www.youtube.com/embed/RpYN8xSuHbM?si=ZZQtnZalyajmuyev" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

To figure out the use of `AD0_VAL` constant in the code, I consulted the Sparkfun's [IMU webpage](https://learn.sparkfun.com/tutorials/sparkfun-9dof-imu-icm-20948-breakout-hookup-guide/all#hardware-overview), which explains what the `ADR` jumper on the back of the IMU chip does. By default, the jumper is open, and the I2C address of the IMU is 0x69. But when the jumper is closed the IMU's address is 0x68. So the constant in the code tells the library which address the IMU is at, and should be the default value of 1 since the jumper is open.

### Accelorometer

Next, I started to experiment with the IMU's accelorometer. Using the following equations from class, I calculated the IMU's pitch and roll based on measurements from the accelorometer.

$\text{Pitch: } \theta_a = \text{atan2}(a_x, a_z)$

$\text{Roll: } \phi_a = \text{atan2}(a_y, a_z)$

When the IMU is laid flat on the table (z-axis pointing straight up), its pitch and roll is 0.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/pitch_roll_0.jpeg" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
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
        {% include figure.liquid loading="eager" path="assets/img/lab2/roll_neg_90.jpeg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

As seen from the images above, the accelorometer readings were highly accurate when I aligned the IMU with the side of my computer (off by at most half a degree). Therefore, I decided that a 2 point calibration would not be needed.

### Accelorometer Fourier Transform

Moving the RC car back and forth next to the accelorometer, we see that there is a lot of noise in the pitch and roll data.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/accel_noise.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

The above graph had 500 data points collected with a roughly 10ms gap between each data point. Using that, we can run a Fourier Transform on the data to see what frequencies are causing noise in our data. The images below show a huge spike in the 1-2 Hz range.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/accel_fft.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

I used the following Python code to generate the Fourier Transform, making sure to only select the positive frequencies and to normalize the amplitudes.

<div class="row">
    <div class="col-sm">
        {% include figure.liquid loading="eager" path="assets/img/lab2/fft_code.png" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

To implement a low-pass filter, I used the data above to select a cutoff frequency of 3 Hz. This should significantly reduce the amount of noise in the system. For an angle $\theta$, the low pass filter is represented by the following.

$\theta_\text{LPF}[n] = \alpha\theta_\text{RAW} + (1 – \alpha)\theta_\text{LPF}[n-1]$

To compute $\alpha$, we know from lecture that $\alpha = \frac{T}{T + RC}$ and that $RC = \frac{1}{f_c \cdot 2\pi}$

So with $f_c = 3$ and $T = 0.01$, we get $RC = 0.053$ and $\alpha = 0.16$. The following image shows the low-passed accelorometer readings compared to the raw values, where we can see a substantial decrease in the noise.

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

Looking at the comparison between the raw gyro values the filtered accelorometer values, we can see in the graphs below that the gyro values drift over time and even miss a few of the big movements which were picked up by the accelorometer. However, the gyro also picks up some angles changes quicker then the filtered accelorometer values. So to get the best of both worlds, we can use complimentary angles, which combines both the gyro and accelorometer readings using the following formula:

$\theta = (\theta + \theta_g)(1 − \alpha) + \theta_{a\text{LPF}}\alpha$

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/accel_gyro.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/yaw.png" title="example image" class="img-fluid rounded z-depth-1" width="50%" %}
    </div>
</div> 

Using the complimentary angle works because we high-pass the gyro readings, suppressing the low-frequency drift, while low-passing the accelerometer data, suppressing its noisy readings. The $\alpha$ we use here is the same value as the one we used above to low-pass the accelerometer. As a result, the complimentary angle readings in the graph is accurate and not susceptible to drift or quick vibrations. I also tried adjusting the sampling frequency from 10ms to 50ms but this caused the gyro to drift more and made the complimentary slightly less accurate as a result.

### Sample Data

My code up to this point was able to collect data roughly every 10ms due to a combination of delay statments and serial prints. After refactoring the code to remove these and centralizing the IMU data collection logic to support start/stop data collection commands, there is now only roughly 3 ms between data points. I also had to update the $\alpha$ value for my complimentary and low-pass filter to be 0.085. To see if the IMU could keep up with the speed of my main loop now, I kept a counter to keep track of how many times where the IMU data was not ready when it was requested, but that did not happen a single time!

I decided to use seperate arrays to store the data as floats. Specifically, to support the graphing visuals in this lab I created 10 arrays to store timestamps, raw and low-passed accelerometer data, raw gyro data, and complimentary angle data. Since I sample and store 10 data points 333 times a second and a float is 4 bytes, in 384 KB we can store 28 seconds worth of data. But if I only keep 4 arrays for time, pitch, roll, and yaw, I can store 72 seconds of data. The graph shown in gyroscope section above represents 6 seconds of IMU data which was collected by the Artemis and set over Bluetooth to my computer.

Here is the Arduino code I used in this lab to collect and send the IMU data over Bluetooth.

```c
void collectIMUData() {
    // Fetch IMU data
    IMU.getAGMT();

    float acc_pitch = atan2(IMU.accX(), IMU.accZ()) * 180/M_PI;
    float acc_roll = atan2(IMU.accY(), IMU.accZ()) * 180/M_PI;
    acc_lp_pitch = a*acc_pitch + (1-a)*acc_lp_pitch;
    acc_lp_roll = a*acc_roll + (1-a)*acc_lp_roll;

    float dt = (micros()-prev_time)/1000000.0;
    gyr_pitch = gyr_pitch - (IMU.gyrY() * dt);
    gyr_roll = gyr_roll + (IMU.gyrX() * dt);
    gyr_yaw = gyr_yaw - (IMU.gyrZ() * dt);
    prev_time = micros();

    comp_pitch = a*acc_lp_pitch + (1-a)*(comp_pitch+(IMU.gyrY() * dt));
    comp_roll = a*acc_lp_roll + (1-a)*(comp_roll+(IMU.gyrX() * dt));

    millis_data[cur_points] = millis();
    acc_pitch_data[cur_points] = acc_pitch;
    acc_roll_data[cur_points] = acc_roll;
    acc_lp_pitch_data[cur_points] = acc_lp_pitch;
    acc_lp_roll_data[cur_points] = acc_lp_roll;
    gyr_pitch_data[cur_points] = gyr_pitch;
    gyr_roll_data[cur_points] = gyr_roll;
    gyr_yaw_data[cur_points] = gyr_yaw;
    comp_pitch_data[cur_points] = comp_pitch;
    comp_roll_data[cur_points] = comp_roll;

    cur_points++;
}
```

This function is only called in the main loop function if the following conditions are met. `DATA_POINTS` was defined to be 2000 which is around 6 seconds of data.
```c
if (collect_data && IMU.dataReady() && cur_points < DATA_POINTS) {
    collectIMUData();
}
```

Finally, I created 2 commands which start the collection of IMU data and stop the collection and send the data back via Bluetooth.
```c
case COLLECT_IMU_DATA: {
    collect_data = true;
    cur_points = 0;
    acc_lp_pitch = 0;
    acc_lp_roll = 0;
    gyr_pitch = 0;
    gyr_roll = 0;
    gyr_yaw = 0;
    comp_pitch = 0;
    comp_roll = 0;
    prev_time = micros();

    break;
}
case GET_IMU_DATA: {

    collect_data = false;

    for (int i = 0; i < min(cur_points, DATA_POINTS); i++) {
        tx_estring_value.clear();
        tx_estring_value.append(String(millis_data[i]).c_str());
        tx_estring_value.append(",");
        tx_estring_value.append(acc_pitch_data[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(acc_roll_data[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(acc_lp_pitch_data[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(acc_lp_roll_data[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(gyr_pitch_data[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(gyr_roll_data[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(gyr_yaw_data[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(comp_pitch_data[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(comp_roll_data[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());

        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
    }

    break;
}
```

And in Jupyter notebook, I used the following notification handler to process all of the timestamped data into arrays for the graph visualizations.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab2/data_handler.png" title="example image" class="img-fluid rounded z-depth-1" width="50%" %}
    </div>
</div>

### Stunt

I played with the car in 2 locations- on the tile flooring of the lab and on the carpeted floor of my apartment. On both surfaces the car accelerated and moved very fast. I was also able to get some nice drifts and spins on the smooth tile flooring and some nice flips on the carpeted flooring, which is shown in the video below. 

<iframe width="560" height="315" src="https://www.youtube.com/embed/dNXj7Hg7iQs?si=GzK9QEjnsKl_jizr" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>