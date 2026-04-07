---
layout: page
title: Lab 8
description: Stunts!
img: assets/img/lab8/flip.jpeg
importance: 8
---

### The Flip

For the stunt, I decided to perform a flip. I created a new command `FLIP`, which starts a 6-state process:
1. Apply a small starting PWM signal to overcome static friction
2. Apply a large PWM signal to get the car moving forward fast
3. Once we are close to the wall, send a backwards PWM signal to initiate the flip
4. Start orientation PID control after we flip over
5. Once we are pointed towards the starting direction, return to the start line
6. Once we reach the start line, brake

Breaking down the code, I first noticed that if I set the forward PWM to be super high initially, the car's wheels would slip, causing the car to turn and miss the flip pad. To address this, I first set the forward PWM to 100 for 500ms to overcome static friction.

```c
case FLIP: {
    perform_flip = true;
    flip_state = 0;
    cur_points = 0;

    LR_conversion = 1.45;
    forward(100); // start slower to overcome static friction first
    flip_time = millis();

    break;
}
```

After static friction is overcome, I set the forward PWM to 205 so the car heads towards the wall as fast as possible. The callibration factor was also adjusted to ensure that the car drives straight at these high speeds. When the Kalman filter distance is less than 900mm (around 3ft), the backward PWM is set to the max value of 255 to initiate the flip. In order to improve the consistency of the flip, I adjusted the callibration factor since I noticed that the left side of the car would always start flipping first. I also added a large weight of screws to the front of the car to make it easier for the car to flip.

While this helped to improve the flip's consistency, there would still be many runs where the car would not be pointing towards the direction where it started (0 degrees). Therefore, I added a state where PID orientation control would be activated using the DMP's yaw measurements to align the car's angle after the flip. To keep things simple, I only use P control and set the min PWM signal to 240 since the flip mat is a super grippy surface. Finally, the car is commanded backwards to return to the starting line and brakes once it is reached.

```c
if (perform_flip) {
    if (flip_state == 0 && millis() - flip_time > 500) { // after static friction overcome
        LR_conversion = 1.25;
        forward(205);
        flip_state++;
    } else if (flip_state == 1 && distance_front < 900) { // start flip when 900mm from wall
        LR_conversion = 0.8;
        backward(255);
        flip_state++;
        flip_time = millis();
    } else if (flip_state == 2 && millis() - flip_time > 750) { // correct heading after flip, assumes flips takes 0.75s
        LR_conversion = 1.45;
        stop();
        pid_rotate = true;
        min_pwm_r = 240; max_pwm_r = 255;
        ki_r = 0; kd_r = 0;
        set_point_angle = 0;
        flip_state++;
    } else if (flip_state == 3 && abs(gyr_yaw) < 10) { // go back to line
        pid_rotate = false;
        backward(140);
        flip_state++;
        flip_time = millis();
    } else if (flip_state == 4 && millis() - flip_time > 1500) {
        brake();
        perform_flip = false;
    }
    if (cur_points < DATA_POINTS) {
        millis_data[cur_points] = millis();
        if (tof_update) {
            tof_front_data[cur_points] = last_tof;
            tof_update = false;
        } else {
            tof_front_data[cur_points] = 0;
        }
        kalman_data[cur_points] = distance_front;
        pwm_data[cur_points] = flip_state;
        gyr_yaw_data[cur_points] = gyr_yaw;
        cur_points++;
    }
}
```

### Flip In Action

The video below shows a successful flip where the car uses the orientation PID to correct its orientation after the flip:

<iframe width="560" height="315" src="https://www.youtube.com/embed/GDrmnILKocw?si=lJUwsVs6L-K6NV4u" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

The graph on the left shows the ToF and Kalman filter distances overlaid with the current state of the flip. The graph on the right shows the yaw angle of the car.

<div class="row">
    <div class="col-sm-6 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab8/distance_state.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-6 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab8/yaw.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

Here is a video of another successful flip:

<iframe width="560" height="315" src="https://www.youtube.com/embed/kvWWl7nr-i4?si=LREIqk7nWF1rWw-Z" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Blooper

Below is a blooper from one of my early test runs as I was tuning the PWM values where the car does a double flip!

<iframe width="560" height="315" src="https://www.youtube.com/embed/QQ0ojeVlxkM?si=okN7Ia0darJNv4R2" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>