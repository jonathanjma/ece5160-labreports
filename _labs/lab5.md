---
layout: page
title: Lab 5
description: Linear PID Control and Extrapolation
img: assets/img/lab5/pid.jpg
importance: 5
---

### Setup

To handle sending and receiving PID debugging data over Bluetooth, I created two commands. `PID_DRIVE` allows me to set the values of Kp, Ki, and Kd and tells the car to start driving using PID control for 10 seconds. `GET_PID_DATA` tells the car to send over the distance and PWM data values collected during the drive. `DATA_POINTS` was set to 3000 since the best case loop time is around 3 ms.

```c
case PID_DRIVE: {
    success = robot_cmd.get_next_value(kp);
    if (!success) return;
    success = robot_cmd.get_next_value(ki);
    if (!success) return;
    success = robot_cmd.get_next_value(kd);
    if (!success) return;

    drive_time = 10000;
    drive_start_time = millis();
    pid_drive = true;
    error_sum = 0;
    last_pid_time = millis();
    cur_points = 0;

    break;
}
case GET_PID_DATA: {
    pid_drive = false;

    for (int i = 0; i < min(cur_points, DATA_POINTS); i++) {
        tx_estring_value.clear();
        // time,tof_front,pwm
        tx_estring_value.append(String(millis_data[i]).c_str());
        tx_estring_value.append(",");
        tx_estring_value.append(tof_front_data[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(pwm_data[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());

        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
    }
    break;
}
```

I placed this code in the main loop function, which stops the car after the specified amount of drive time is up and collects PID debug data during a PID drive.
```c
if (drive_start_time != 0 && millis() - drive_start_time > drive_time) {
    stop();
    drive_start_time = 0;
    pid_drive = false;
} else if (pid_drive) {
    pid_control();
    if (cur_points < DATA_POINTS) {
        millis_data[cur_points] = millis();
        tof_front_data[cur_points] = distance_front;
        pwm_data[cur_points] = pwm_output;
        i_data[cur_points] = error_sum;
        cur_points++;
    }
}
```

I used the following notification handler in Python to parse the PID debug data into lists so that it can be visualized in graphs. I found the TQDM library helpful as it shows a nice progress bar of how many data points have been sent over Bluetooth so far.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab5/handler.png" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

### PID Control

For PID control, I started by tuning the porportional gain Kp. From lab 4, I know a reasonable PWM value for driving forward would be 80. Additionally, I will be testing the car from around a distance of 5 feet from the wall, which is around 1200mm. Therefore, a reasonable value of Kp to start testing with would be 80 divided by 1200 which is 0.066.

Also I decided to use the long distance mode of the ToF sensor since the car needed to see beyond the 1.3m limit of short mode. By default, the ToF sensor library uses an intermeasurement period between ToF readings of 100ms and a slightly shorter timing budget. But since I didn't need to full long range and could sacrifice some accuracy, I set the timing budget to 50ms and the intermeasurement period to 55ms since it had to be 5ms higher.

For this lab, I decided to implement a PI controller. Although this would lack the overshoot prevention that the D term provides, my tuning below shows that PI is enough to correct the overshoot and eliminate any steady state error. The PI controller was implemented according to the formula below

$u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau$

The piece of code below shows the PID control loop I used. The set point was set to be 300mm, which is roughly 1 foot. To deal with deadband, the minimum PWM output I allowed was 40 for moving forward and -40 backward. If the controller outputted any number between that range, it was min/maxed so it would be outside the deadband. Later on, I did run into issues where the car kept moving back and forth around the set point, so I updated the logic to tell the motors to stop moving if the PWM output was between -3 and 3, which reduced the severity of this issue. I also min/maxed the PWM outputs between 125 and -125 since I found that beyond this the car would move way to fast and out of control.

```c
void pid_control() {
    int error = distance_front - set_point;
    unsigned long dt_ms = millis() - last_pid_time;

    // Accumulate and clamp error sum for I
    error_sum += (float)error * (float)dt_ms / 1000.0;
    error_sum = min(error_sum, error_sum_max);
    error_sum = max(error_sum, -error_sum_max);

    pwm_output = error * kp + error_sum * ki;
    if (pwm_output > 3) {
        // ensure pwm is below motor saturation but above deadband
        pwm_output = min(pwm_output, max_pwm);
        pwm_output = max(pwm_output, min_pwm);

        forward(pwm_output);
    } else if (pwm_output < -3) {
        // same conditions but reversed since its negative
        pwm_output = max(pwm_output, -max_pwm);
        pwm_output = min(pwm_output, -min_pwm);
        
        backward(-pwm_output);
    } else {
        forward(0);
    }

    last_pid_time = millis();
}
```

After a bunch of tuning runs, I settled on a Kp of 0.06. As you can see in the video below, this would result in the car overshooting the target, backing up to the target, and then oscillating around it. However, lower Kp values would result in a super slow approach which undershoots the target, so I decided to stick with this value. For the Ki term, testing showed that with just Kp, the car was  reaching the target each time after a few oscillations with little to no steady state error. I did have a few tuning runs where steady state error occured, so I graphed the error sum used for Ki and saw that at the time the car usually reached the wall, the error sum was around 200. After a few more runs I ended up settling on a Ki of 0.05. The graph shows the distance reported as the car approaches the 1 foot set point. 

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab5/non_extrapolated.png" title="example image" class="img-fluid rounded z-depth-1" width="50%" %}
    </div>
</div>

<iframe width="560" height="315" src="https://www.youtube.com/embed/9Jn4biEd_-Q?si=_yfDkepEI3y3rv70" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Extrapolation

As I discussed above, the ToF sensor takes a new measurement every 55ms. Looking at the timestamps of the PID debug data, I can see that the cycle time of the main loop is around 5-10ms. And since reading the ToF sensor is non-blocking, it means that my loop is running roughly 5 times as fast as the sensor readings are coming in. In order to improve the performance of my PID controller, I implemented extrapolation which uses the last 2 ToF readings as well as the last time a reading was taken. The last 2 readings give as a slope, and then we can use the time since the last reading to estimate the current position of the car.

```c
if (tof_front.checkForDataReady()) {
    prev2_distance_front_raw = prev_distance_front_raw;
    prev_distance_front_raw = tof_front.getDistance();
    distance_front = prev_distance_front_raw;
    last_measure_time = millis();
    tof_front.clearInterrupt();
} else {
    // extrapolate current distance based on last 2 data points
    // prev2_distance_front_raw is only 0 after the first tof reading
    float slope = (prev_distance_front_raw - prev2_distance_front_raw) / (intermeasurement_ms * 1.0);
    distance_front = prev_distance_front_raw + slope * (millis() - last_measure_time);
}
```

The graph of the extrapolated ToF distance values and the associated PWM output for each timestep is shown below. The video shows the final tuned controller approaching the wall and responding to a few disturbances as I tried to push the car around.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab5/extrapolated.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

<iframe width="560" height="315" src="https://www.youtube.com/embed/MhT6D2W-ZpI?si=uYV2zc7axbeKYpXT" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### 5000 Level Tasks

Whenever the Ki term is used for PID control, integrator windup is always an issue. This occurs when a disturbance, obstacle, or high friction causes the integrated error sum to grow so large and is unable to shrink fast enough that an extremely large PWM output is sent to the motors, causing the car to drive well past its set point. This is demonstrated in the video below. When tuning the Ki term, I graphed the error sum I noticed that at the time the car usually reached the wall, the error sum was around 200, so I set the max to be 250.

<iframe width="560" height="315" src="https://www.youtube.com/embed/L3envAN5Oxo?si=KIzxJ20RUbWw-DGu" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Acknowledgement: I referenced Aidan Derocher's website from Spring 2025 for inspiration