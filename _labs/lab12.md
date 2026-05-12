---
layout: page
title: Lab 12
description: Path Planning and Execution
img: assets/img/lab12/navigation.png
importance: 12
---

### Initial Approach

For this lab, my approach was to mainly use the linear and orientation PID controllers I developed in previous labs to navigate to all the waypoints. While this could be less resilient than incorporating the mapping and localization after each waypoint to account for errors in the PID, I decided to use this approach since I that my PID controllers were pretty accurate. While my localization code in lab 11 was actually quite accurate, I was worried that all the mapping spins would cause the DMP to slowly drift. Also I noticed that the angle component of Baye's filter was always off by over 10 degrees, which could cause a lot of error.

My main approach was to use the front ToF sensor combined with the linear PID controller to track how far my car drives on the straight sections and the DMP combined with the orientation PID controller to execute turns. I also changed the timing budget for the ToF sensor from 50ms to the default of 100ms since speed wasn't an issue anymore and I wanted to prioritize accurate and consistent measurements. The annotated map below shows the PID distance setpoints for each straight segment and the PID angle setpoints for each turn.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab12/annotated_map.png" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

Converting these measurements to Python, the code uses the `PID_DRIVE` and `PID_ROTATE` commands to control the car. The final version of this code is shown below. 

```py
def drive(dist):
    ble.send_command(CMD.PID_DRIVE, str(dist) + "|0.08|0|0.007")
    sleep(4.1)

def rotate(angle):
    ble.send_command(CMD.PID_ROTATE, str(angle) + "|1.0|0.25|0.025")
    sleep(1.1)

rotate(-45)
drive(2175)

rotate(0)
drive(1650)

rotate(60)
drive(475)

rotate(0)
drive(350)

rotate(-90)
drive(400)

rotate(-180)
drive(700)

rotate(90)
drive(750)
```

This video shows the first successful run of the car through the course. As you can see, there is a lot of room for improvement as the car overshoots waypoints, does non-precise turns, and spends a lot of time just waiting.

<iframe width="560" height="315" src="https://www.youtube.com/embed/a8Hwvl_vI8M?si=l_y175vowsO07eDY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Adding D Control

I realized the reason my linear PID controller kept overshooting the waypoints was because I had not implement derivative control. This was done by largely copying the same approach I used for derivative control in the orientation PID: calculating the change in error over time since the last loop and then passing it through a low-pass filter.

```c
float cur_derror_dt = (error - last_error) / (dt_ms / 1000.0);
derror_dt = a_dd * cur_derror_dt + (1 - a_dd) * derror_dt;
pwm_output += error_dt * kd_d;
...
last_error = error;
```

After a bunch of tests where I started the car at different distances from a wall, I slowly tuned the value of $K_d$ and was able to produce the graph below where the car barely overshoots the setpoint. The final value of $K_d$ is 0.007, and I was also able to increase $K_p$ to 0.08 which allowed the car to accelerate faster initially. I also tuned the $\alpha$ value of the derivative low-pass filter to 0.1, which made the derivative signal less noisy while still being responsive to changes. 

<div class="row">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab12/dist_pwm_graph.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-4 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab12/d_graph.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

Here is a video of the newly tuned PID controller:

<iframe width="560" height="315" src="https://www.youtube.com/embed/JTanJBbu9VQ?si=TUxDQPZRAqYofeLF" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Other Problems

I also ran into other problems which would cause inconsistency between runs, like unprecise turns which would cause my car to run into the center island or not reach the final waypoint, which are shown in the videos below. I was able to improve the consistency of the turns by increasing the $K_p$ of the orientation controller from 0.75 and 1.0. For the linear PID, I also added a cut out which would stop motor movement if the output of the PID controller was between -1 and 1 in order to improve stability by reducing the oscillations around the setpoint.

<div class="row">
    <div class="col-sm-6 mt-3 mt-md-0">
        <iframe width="100%" height="315" src="https://www.youtube.com/embed/ogN6AduVT0s?si=EE3AztHMW56M8AuY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>
    <div class="col-sm-6 mt-3 mt-md-0">
        <iframe width="100%" height="315" src="https://www.youtube.com/embed/GE7M0V4drlk?si=YT8nf1A2mJPON-7j" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div>
</div>

### Final Run

For the final run, I was able to combine all of the improvements below to get a final run through of the course going through each waypoint. I also reduced the waits between movements to get a run time of around 35 seconds. I was actually quite surprised that my car was able to navigate the course so well without the need to relocalize, which shows the benefit of having precise control.

<iframe width="560" height="315" src="https://www.youtube.com/embed/yQF3Mh9Sw6E?si=gAzdprRCiSOFoot7" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>