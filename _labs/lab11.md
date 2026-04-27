---
layout: page
title: Lab 11
description: Grid Localization using Bayes Filter (Real Robot)
img: assets/img/lab11/localization.jpg
importance: 11
---

### Setup

Before starting, I first checked the functionality of the localization starter code by running the `lab11_sim` notebook. The results are shown below is closely follow the results from lab 10.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab11/sim_still_works.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

In order to implement a Bayes filter using the real robot, I first implemented the `perform_observation_loop` in the `RealRobot` class, which is called when the update loop needs to collect observations about the robot's sorroundings. I first send a `MAP` command which I implemented in lab 9 to collect 18 ToF readings which are seperated by 20 degrees. I also made this function `async` so that I could wait for 25 seconds for the localization spin to be complete before sending the `GET_MAP_DATA` command also from lab 9 to get the ToF measurement data. Since the localization code expects the measurements to be in counter-clockwise order by yaw instead of clockwise, I reverse the order of the list.

```py
async def perform_observation_loop(self, rot_vel=120):
    global time_arr, mapping_arr, dist_arr, yaw_arr, measure_X, measure_Y
    time_arr, mapping_arr, dist_arr, yaw_arr = [], [], [], []

    def on_receive(uuid, bytearr):
        data = ble.bytearray_to_string(bytearr).split(",")
        time_arr.append(int(data[0]))
        mapping_arr.append(int(data[1])/1000.0) # convert from mm to m
        dist_arr.append(int(data[2]))
        yaw = float(data[3])
        yaw_arr.append(yaw if yaw >= 0 else yaw + 360)
    
    ble.send_command(CMD.MAP, "")
    await asyncio.sleep(25)
    
    ble.start_notify(ble.uuid['RX_STRING'], on_receive)
    ble.send_command(CMD.GET_MAP_DATA, "")
    await asyncio.sleep(5)
    mapping_arr.reverse()
    yaw_arr.reverse()
    mapping_arr = [mapping_arr[-1]] + mapping_arr[:-1]
    yaw_arr = [yaw_arr[-1]] + yaw_arr[:-1]

    return np.array(mapping_arr)[np.newaxis].T, np.array(yaw_arr)[np.newaxis].T
```

Below is a video of the robot performing a localization spin at the (-3, -2) coordinate:

<iframe width="560" height="315" src="https://www.youtube.com/embed/AnkN5q0Lt6w?si=4RYIo6lekVyKoF3X" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

#### (-3, 2) Scan

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab11/-3_-2.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

The robot was able to perfectly localize itself at (-3, -2), as the belief dot is drawn directly on top of the ground truth dot.

#### (0, 0) Scan

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab11/0_0.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

The robot was also able to perfectly localize itself at (0, 0).

#### (0, 3) Scan

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab11/0_3.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

The robot was also able to perfectly localize itself at (0, 3), but for this location I had to change the robot's code to use the lab 9 behavior of taking 24 ToF measurements 15 degrees apart. The image below shows the result of only 15 measurements where the robot's position is completely incorrect, and this was repeated over multiple runs.

10th point

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab11/0_3_bad.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

#### (5, -3) Scan

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab11/5_-3.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

The robot was able to perfectly localize itself at (5, -3).

#### (5, 3) Scan

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab11/5_3.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

The robot was mostly able to localize itself at (5, 3). Some reasons for this could be the ToF sensor distances maxxing out.

etc