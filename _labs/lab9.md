---
layout: page
title: Lab 9
description: Mapping
img: assets/img/lab9/map.jpg
importance: 9
---

### Control

For control, I decided to use my orientation PID to rotate the robot to each angle in order to map the room. This was better than open-loop control since it offered more accuracy and consistency. It is also more simple to implement than angular speed control while offering most of its beenfits. To perform my orientation PID scan, I created a new command `MAP` which resets the PID variables and starts a sequence of 25 rotations. Each rotation would be 15 degrees, which would produce a more detailed map than the 15 rotations suggested by the handout. I also created a `GET_MAP_DATA` command to retrieve the map and debug data from the robot.

```c
case MAP: {
    perform_map = true;
    mapping_idx = 0;
    cur_points = 0;

    target_reach_time = millis();
    set_point_angle = 0;
    pid_rotate = true;
    error_sum = 0;
    last_pid_time = millis();
    last_error = gyr_yaw - set_point_angle;
    memset(mapping_data, 0, sizeof(mapping_data));

    break;
}
```

After setting each target angle and resetting the PID controller, the code waits 1 second before repeating this to increment the set point to the next target angle. This gives the robot enough time to reach the new angle and settle down so that we can get an accurate ToF measurement.

```c
if (perform_map) {
    if (millis() - target_reach_time > 1000) {
        target_reach_time = millis();
        if (cur_points < DATA_POINTS) {
            mapping_data[cur_points] = last_tof;
        }
        mapping_idx++;
        set_point_angle = (mapping_idx * 15) % 360;
        if (set_point_angle > 180) set_point_angle -= 360;
        error_sum = 0;
        last_error = gyr_yaw - set_point_angle;
        last_pid_time = millis();
        if (mapping_idx == 25) { // 15 deg increments, 360/15 + 1 = 25
            perform_map = false;
            pid_rotate = false;
        }
    }
    if (cur_points < DATA_POINTS) {
        millis_data[cur_points] = millis();
        gyr_yaw_data[cur_points] = gyr_yaw;
        tof_front_data[cur_points] = last_tof;
        cur_points++;
    }
}
```

The graphs below show the yaw and distance sensor readings while the robot performs a mapping. The yaw graph shows little to no overshoot when performing each incremental rotation and plenty of settling time (seems to be 500-750ms) at each set point.

*** maybe update too after vid
<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab9/yaw_dist.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

*** insert video of pid spin

### Mapping

The below graphs show the results of each of the 5 scans in 2 representations: a polar angle to distance graph and the transformed coordinates of each obstacle in world coordinates. To transform the points from the ToF sensor's frame to the robot's frame and then the world frame, I created 2 matrices $T^R_{TOF}$ and $T^W_R$, where

\begin{equation}
T^R_{TOF} = \begin{bmatrix} 1 & 0 & 0.23 \\\\ 0 & 1 & 0 \\\\ 0 & 0 & 1 \end{bmatrix} \text{ and } 
T^W_R = \begin{bmatrix} \cos\psi & -\sin\psi & X \\\\ \sin\psi & \cos\psi & Y \\\\ 0 & 0 & 1 \end{bmatrix} \nonumber
\end{equation}

$X$ and $Y$ are the x and y coordinate of the robot when it creates the scan and $\psi$ is the raw yaw while from the DMP reflected over the y-axis to align the coordinate frames. This is shown in the code below.

```py
def transform_points(dists, yaws, robot_X, robot_Y):
    world_x_points = []
    world_y_points = []
    for i in range(len(dists)):
        psi = -yaws[i] + np.pi # reflect over y-axis to algin with world
        T_robot_to_world = np.array([[np.cos(psi), -np.sin(psi), robot_X], 
                                     [np.sin(psi), np.cos(psi), robot_Y], 
                                     [0, 0, 1]])
        T_tof_to_robot = np.array([[1, 0, 70/304.8], # x offset mm -> ft
                                   [0, 1, 0], # y offset
                                   [0, 0, 1]])
        point_tof = np.array([[dists[i]/304.8], 
                              [0], 
                              [1]])
        point_world = T_robot_to_world @ T_tof_to_robot @ point_tof
        world_x_points.append(point_world[0, 0])
        world_y_points.append(point_world[1, 0])

    return world_x_points, world_y_points
```

#### (-3, 2) Scan

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab9/map_-3_2.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

#### (0, 0) Scan

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab9/map_0_0.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

#### (0, 3) Scan

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab9/map_0_3.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

#### (5, -3) Scan

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab9/map_5_-3.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

#### (5, 3) Scan

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab9/map_5_3.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

### Combined Map

Below is the color-coded map of all of scans in world coordinates:

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab9/combined_map.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

### Line Map

To convert this map into a format which can be used in the simulator for later labs, I estimated all of the walls in the room and plotted their coordinates on the map. I noticed that the raw data was mostly accurate, although some of the outer walls appear crooked and not straight since the robot rotates slightly off-axis as it scans. The middle island obstacle is also missing its right wall since its not really visible from any of the scanning points.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab9/line_map.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>