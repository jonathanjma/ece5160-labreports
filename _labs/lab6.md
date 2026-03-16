---
layout: page
title: Lab 6
description: Orientation Control
img: assets/img/lab6/pid.jpg
importance: 6
---

### Setup

My debugging setup to handle sending and receiving PID debugging data over Bluetooth was mostly similar to the one in lab 5, but I created two new commands and new functions so that the linear PID code was independent of the rotation PID code.
The `PID_ROTATE` command allows me to set the values of Kp, Ki, Kd, as well as the target angle, and tells the car to start turning using PID control for 10 seconds. `GET_PID_ROTATE_DATA` tells the car to send over the IMU yaw and PWM data values collected during the rotation. `DATA_POINTS` was set to 3000 since the best case loop time is around 3 ms.

```c
case PID_ROTATE: {
    success = robot_cmd.get_next_value(set_point_angle);
    if (!success) return;
    success = robot_cmd.get_next_value(kp_r);
    if (!success) return;
    success = robot_cmd.get_next_value(ki_r);
    if (!success) return;
    success = robot_cmd.get_next_value(kd_r);
    if (!success) return;

    drive_time = 10000;
    drive_start_time = millis();
    pid_rotate = true;
    error_sum = 0;
    last_pid_time = millis();
    cur_points = 0;
    last_error = gyr_yaw - set_point_angle;

    break;
}
case GET_PID_ROTATE_DATA: {
    pid_rotate = false;

    for (int i = 0; i < min(cur_points, DATA_POINTS); i++) {
        tx_estring_value.clear();
        // time,yaw,pwm
        tx_estring_value.append(String(millis_data[i]).c_str());
        tx_estring_value.append(",");
        tx_estring_value.append(gyr_yaw_data[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(pwm_data[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(error_data[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());

        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
    }
    break;
    }
```

Similarly to lab 5, I placed this code in the main loop function, which stops the car after the specified amount of driving time is up and collects PID debug data during the PID rotation.

```c
if (pid_rotate) {
    pid_rotate_control();
    if (cur_points < DATA_POINTS) {
        millis_data[cur_points] = millis();
        gyr_yaw_data[cur_points] = gyr_yaw;
        pwm_data[cur_points] = pwm_output;
        error_data[cur_points] = cur_error;
        cur_points++;
    }
}
```

I used the following notification handler in Python to parse the PID debug data into lists so that it can be visualized in graphs.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab6/handler.png" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

### Digital Motion Processor (DMP)

In lab 2, the IMU was used to calculate the yaw by integrating gyroscope output over time: `gyr_yaw = gyr_yaw - (IMU.gyrZ() * dt)`. However, any inaccuracies or noise in the gyroscope output will quickly cause the error to grow unbounded. But unlike the pitch and roll direction where you can use complementary filter with accelorameter data, gravity stays the same for yaw so this wouldn't work. Instead, we can use the digital motion processor (DMP) which is integrated into our IMU, which fuses data from the gyroscope and accelerometer to perform error/drift correction. Using the [DMP tutorial](https://fastrobotscornell.github.io/FastRobots-2026/tutorials/dmp.html) found on the course website, the code below initializes the DMP module. One thing I had to change in the DMP code was its data rate since the DMP stores all of its data in a FIFO queue. Using its default settings, the yaw measurements were lagging behind since my code wasn't sampling it fast enough, so I changed the data rate parameter to 1, which is 27.5 Hz, or one measurement every 35 ms. 

```c
// IMU DMP setup
bool dmp_success = true;
dmp_success &= (IMU.initializeDMP() == ICM_20948_Stat_Ok);
dmp_success &= (IMU.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

// Set the DMP output data rate (ODR): value = (DMP running rate / ODR ) - 1
// E.g. for a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
dmp_success &= (IMU.setDMPODRrate(DMP_ODR_Reg_Quat6, 1) == ICM_20948_Stat_Ok);
dmp_success &= (IMU.enableFIFO() == ICM_20948_Stat_Ok);
dmp_success &= (IMU.enableDMP() == ICM_20948_Stat_Ok);
dmp_success &= (IMU.resetDMP() == ICM_20948_Stat_Ok);
dmp_success &= (IMU.resetFIFO() == ICM_20948_Stat_Ok);
Serial.print("DMP Initialization ");
Serial.println(dmp_success ? "Success" : "FAILED");
```

Using the IMU Arduino library's example code found in the `Example7_DMP_Quat6_EulerAngles` file, I created the `collectDMPYaw` function which runs in the main loop to check to see if a new yaw measurement is available. One thing I noticed when experimenting is that the during the first 10 seconds after the DMP turns on, the yaw measurement always drifts from 0 to 9.85 degrees and then settles, even though the IMU is flat and the car is stationary. To fix this, I subtract a `DMP_YAW_OFFSET` which is 9.85 from every measurement. Finally, I saw in the datasheet that the maximum maximum rotational velocity the IMU can read is 2000 dps, which is more than enough for this application since the car will not be rotating at 5 revolutions per second.

```c
void collectDMPYaw() {
    // Read any DMP data waiting in the FIFO
    icm_20948_DMP_data_t data;
    IMU.readDMPdataFromFIFO(&data);

    // Is valid data available?
    if (IMU.status == ICM_20948_Stat_Ok || IMU.status == ICM_20948_Stat_FIFOMoreDataAvail) {

        // We have asked for GRV data so we should receive Quat6
        if ((data.header & DMP_header_bitmap_Quat6) > 0) {
            // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
            // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
            // The quaternion data is scaled by 2^30.

            // Scale to +/- 1
            double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
            double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

            double qw = q0;
            double qx = q2;
            double qy = q1;
            double qz = -q3;

            // yaw (z-axis rotation)
            double t3 = +2.0 * (qw * qz + qx * qy);
            double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
            // for some reason the yaw output always drifts to around 9.85 degrees after the first 10 seconds before settling
            gyr_yaw = atan2(t3, t4) * 180.0 / PI - DMP_YAW_OFFSET;
            if (gyr_yaw <= -180) { // fix range from -189.85 to 170.15 -> -180 to 180
                gyr_yaw += 360;
            }
        }
    }
}
```

### Implementing PID Control

For this lab, I decided to implement a PID controller since this would add overshoot prevention and elimiate steady state error. The PID controller was implemented according to the formula below:

$u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau + K_d \frac{de(t)}{dt}$

The code below shows the PID control loop I used. I first calculate the error based on the current target angle. If this error is more than 180 degress, I subtract or add 360 degrees to ensure that the car turns the optimal way. To deal with deadband, the minimum PWM output I allowed was +/- 130 for turning left and right. Similar to lab 5, if the controller outputted any number between that range, it was min/maxed so it would be outside the deadband. To prevent the car from moving back and forth around the set point, I also tell the motors to stop moving if the PWM output is between -3 and 3. Finally, the PWM outputs are also min/maxed between 200 and -200 since beyond this the car would move turn too fast and out of control.

```c
void pid_rotate_control() {
    cur_error = gyr_yaw - set_point_angle;
    // turn the optimal way (deal with wraparound)
    if (cur_error > 180) {
        cur_error -= 360;
    } else if (cur_error < -180) {
        cur_error += 360;
    }
    unsigned long dt_ms = millis() - last_pid_time;

    float cur_derror_dt = (cur_error - last_error) / (dt_ms/1000.0);
    derror_dt = a_dr*cur_derror_dt + (1 - a_dr)*derror_dt;

    // Accumulate and clamp error sum for I
    error_sum += (float)cur_error * ((float)dt_ms/1000.0);
    error_sum = min(error_sum, error_sum_max_r);
    error_sum = max(error_sum, -error_sum_max_r);

    pwm_output = cur_error * kp_r + error_sum * ki_r + derror_dt * kd_r;
    if (pwm_output > 3) {
        // ensure pwm is below motor saturation but above deadband
        pwm_output = min(pwm_output, max_pwm_r);
        pwm_output = max(pwm_output, min_pwm_r);

        left(pwm_output);
    } else if (pwm_output < -3) {
        // same conditions but reversed since its negative
        pwm_output = max(pwm_output, -max_pwm_r);
        pwm_output = min(pwm_output, -min_pwm_r);

        right(-pwm_output);
    } else {
        left(0);
    }

    last_pid_time = millis();
    last_error = cur_error;
}
```

#### Proportional Control

I started by tuning the porportional gain Kp. From lab 4, I know a reasonable PWM value for turning would be 150. Since the maximum angle the car will ever have to rotate is 180 degrees, a reasonable value of Kp to start with would be 150/180 which is 0.85. As I tuned Kp, I found that a higher Kp would be result in a more snappy controller with some overshoot, which could then be mitigated with a derivative term. I ended up settling on a Kp value of 1.0. In the graphs and video below, the car rotates from 0 to 180 degrees, overshoots a bit, but is able to correct itself and settle at the target angle.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab6/p_control.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

<iframe width="560" height="315" src="https://www.youtube.com/embed/lr5Rt8nWxIQ?si=OcOG3V3AoBWcg78_" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

#### Derivative Control

For the derivative term, taking the derivative of the current yaw error seems redundant since usually yaw is this the integral of angular velocity. But since we are using the DMP, the yaw outputs are distinct measurements and not just integrated values. When using the derivative term, there are two main issues which can make our system unstable: derivative kick and noisy derivative calculations. To deal with derivative kick, each time the targat angle changes, I set the `last_error` variable used to calculate the slope to the current error so that for the first iteration, the derivative is practically zero. To address the noisy measurements, I added a low-pass filter to the derivative output.

For the alpha value of my low-pass filter, I initially started with 0.025, but when tuning my derivative term I noticed that while this was great for filtering the high frequency noise, it introduced significant phase lag which would create a bump in the error graph before the car reached the target angle where the error would briefly increase before decreasing again. I ended up choosing an alpha value of 0.25, which makes the derivative term noisier, but also makes the controller more snappy and responsive. The images below show the progression of the derivative signal from just the raw signal, fixing derivative kick, and then adding the low-pass filter.

<div class="row">
    <div class="col-sm-4 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab6/d_kick.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-4 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab6/d_no_lowpass.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-4 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab6/d_lowpass.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

For tuning Kd, I tried to find the middle ground between the overshooting seen in P-only control when D is too low and the jittering around the target angle seen when D is too high. I ended up settling on Kd value of 0.03. In the graph and video below, the car is able to rotate from 0 to 180 degrees with barely an overshoot.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab6/pd_control.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

<iframe width="560" height="315" src="https://www.youtube.com/embed/xxDaO7HtKyk?si=VzMLLuXKXqTmHuf-" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

#### Integral Control

After achieving PD control, I noticed that there was barely any steady state error in my system, which would make the integral term unnecessary. However, it could be useful when running the car on higher friction surfaces like cardboard since I had tuned the PID for low friction surfaces like hard flooring. For tuning Ki, I tried to find a value which would allow the car to make a 180 degress turn on the cardboard surface, which was 0.5. In the graph and video below, the car is able to rotate from 0 to 180 degrees due to help from the integral term.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab6/pid_control.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

<iframe width="560" height="315" src="https://www.youtube.com/embed/8-Q9mnV0jnk?si=wbH03s3vO4AJiV9j" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

However, in my final controller I ended up changing the Ki to 0.25 since I noticed that running a Ki of 0.5 on hard flooring would cause the controller to always overshoot the target.

### Changing the Target Angle

To support changing the target angle while the PID controller is running, I added a new `SET_ANGLE` command which updates the controller's set point. And for controlling the orientation while the car is driving forward or backward, this could be done by running both the linear and rotational PID controllers at the same time and adding together the PWM outputs from both. The video below shows the car moving through a sequence of angles using the command below, as well as how the controller reacts to external disturbances.

```c
case SET_ANGLE: {
    success = robot_cmd.get_next_value(set_point_angle);
    if (!success) return;

    last_error = gyr_yaw - set_point_angle;

    break;
}
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/BVIDLI3-y8Y?si=kDMoeuIzfFXL52pO" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### 5000 Level Task

Whenever the integral term is used for PID control, integrator windup is always an issue since disturbance or high friction can cause the error sum to grow so large that a large PWM output is sent to the motors, causing the car to turn past its target angle. When tuning the Ki term on cardboard, I graphed the error sum and I noticed that the car tends to overshoot the target when the error sum was around 250, so I set the max to be 225 which seemed to resolve the issue. In the graph and video below, the car attempts to rotate from 0 to 180 degrees, but overshoots the target due to integral windup since the error sum unable to shrink fast enough.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab6/no_i_clamp.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

<iframe width="560" height="315" src="https://www.youtube.com/embed/FGiw6cORHOI?si=yPgs3-Mr5yeUk1mE" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Acknowledgement: I referenced Evan Leong's and Aidan Derocher's website from Spring 2025 for inspiration