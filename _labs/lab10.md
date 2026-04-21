---
layout: page
title: Lab 10
description: Grid Localization using Bayes Filter (Simulation)
img: assets/img/lab10/localization.jpg
importance: 10
---

### Control Computation

```py
def compute_control(cur_pose, prev_pose):
    """ Given the current and previous odometry poses, this function extracts
    the control information based on the odometry motion model.

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose 

    Returns:
        [delta_rot_1]: Rotation 1  (degrees)
        [delta_trans]: Translation (meters)
        [delta_rot_2]: Rotation 2  (degrees)
    """
    delta_rot_1 = np.degrees(np.atan2(cur_pose[1] - prev_pose[1], cur_pose[0] - prev_pose[0])) - prev_pose[2]
    delta_trans = np.linalg.norm(cur_pose[:2] - prev_pose[:2])
    delta_rot_2 = cur_pose[2] - prev_pose[2] - delta_rot_1

    return mapper.normalize_angle(delta_rot_1), delta_trans, mapper.normalize_angle(delta_rot_2)
```

### Motion Model

```py
def odom_motion_model(cur_pose, prev_pose, u):
    """ Odometry Motion Model

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose
        (rot1, trans, rot2) (float, float, float): A tuple with control data in the format 
                                                   format (rot1, trans, rot2) with units (degrees, meters, degrees)

    Returns:
        prob [float]: Probability p(x'|x, u)
    """
    actual_control = compute_control(cur_pose, prev_pose)
    ideal_control = u
    p1 = loc.gaussian(mapper.normalize_angle(actual_control[0] - ideal_control[0]), 0, loc.odom_rot_sigma)
    p2 = loc.gaussian(actual_control[1] - ideal_control[1], 0, loc.odom_trans_sigma)
    p3 = loc.gaussian(mapper.normalize_angle(actual_control[2] - ideal_control[2]), 0, loc.odom_rot_sigma)
    prob = p1 * p2 * p3
                      
    return prob
```

### Prediction Step

```py
def prediction_step(cur_odom, prev_odom):
    """ Prediction step of the Bayes Filter.
    Update the probabilities in loc.bel_bar based on loc.bel from the previous time step and the odometry motion model.

    Args:
        cur_odom  ([Pose]): Current Pose
        prev_odom ([Pose]): Previous Pose
    """
    u = compute_control(cur_odom, prev_odom)
    for cx in range(mapper.MAX_CELLS_X):
        for cy in range(mapper.MAX_CELLS_Y):
            for ca in range(mapper.MAX_CELLS_A):

                # don't check if belief is less than 0.0001
                if loc.bel[cx, cy, ca] > 0.0005:
                    for cx2 in range(mapper.MAX_CELLS_X):
                        for cy2 in range(mapper.MAX_CELLS_Y):
                            for ca2 in range(mapper.MAX_CELLS_A):
                                prev_pose = np.array(mapper.from_map(cx, cy, ca))
                                curr_pose = np.array(mapper.from_map(cx2, cy2, ca2))
                                loc.bel_bar[cx2, cy2, ca2] += odom_motion_model(curr_pose, prev_pose, u) * loc.bel[cx, cy, ca]
```

### Sensor Model

```py
def sensor_model(obs):
    """ This is the equivalent of p(z|x).

    Args:
        obs ([ndarray]): A 1D array consisting of the true observations for a specific robot pose in the map 

    Returns:
        [ndarray]: Returns a 1D array of size 18 (mapper.OBS_PER_CELL) with the likelihoods of each individual sensor measurement
    """
    return loc.gaussian(loc.obs_range_data.reshape(-1), obs.reshape(-1), loc.sensor_sigma)
```

### Update Step

```py
def update_step():
    """ Update step of the Bayes Filter.
    Update the probabilities in loc.bel based on loc.bel_bar and the sensor model.
    """
    for cx in range(mapper.MAX_CELLS_X):
        for cy in range(mapper.MAX_CELLS_Y):
            for ca in range(mapper.MAX_CELLS_A):
                loc.bel[cx, cy, ca] = np.prod(sensor_model(mapper.get_views(cx, cy, ca))) * loc.bel_bar[cx, cy, ca]
    # fix underflow
    loc.bel = loc.bel / np.sum(loc.bel)
```

### Results

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab10/run1.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

<iframe width="560" height="315" src="https://www.youtube.com/embed/o-ejXRAHLso?si=Z7hNCd3_AoQbgLxe" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab10/run2.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

<iframe width="560" height="315" src="https://www.youtube.com/embed/bRvAS0hbRtk?si=XBP162XAvTGLlDBp" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
