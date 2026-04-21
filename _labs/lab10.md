---
layout: page
title: Lab 10
description: Grid Localization using Bayes Filter (Simulation)
img: assets/img/lab10/localization.jpg
importance: 10
---

In order to implement the simulated Bayes filter, I first needed to implement 5 functions: control computation, the motion model, the prediction step, the sensor model, and the update step.

### Control Computation

To compute the control input needed to get from the robot's previous pose to its current pose, we can use the odometry motion model covered in class. The model states that any control input can be broken up into 3 components- the first rotation which rotates the robot in the direction of the target point, a translation which moves it to the target point, and the second rotation which rotates it to the target orientation. We also make sure to normalize the angles since the robot uses angles between -180 and +180.

```py
def compute_control(cur_pose, prev_pose):
    """ Given the current and previous odometry poses, this function extracts
    the control information based on the odometry motion model.

    Args:
        cur_pose  ([Pose]): Current Pose, where Pose is (x, y, theta)
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

The robot's motion model computes P(X' \| X, u), which is the probability that the robot reaches the current state X' given that its previous state was X and the control input was u. This is accomplished by using 3 independent gaussians to compute the likelihood of the first rotation, translation, and second rotation based actual control input and the ideal control input. And since the gaussians are independent, we can multiply them together to get the final probability.

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

In the prediction step, we attempt to localize the robot on a 12x9 grid, where each grid cell also has 18 theta cells. To do this, we compute the probability that the robot is in a certain cell given its previous cell and control input for every possible pair of previous and current poses. This comes out to 1944^2 iterations per step, which can be reduced by skipping the inner loop of cells with a probability if less than 0.0001 since they will not contribute much to the belief.

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
                if loc.bel[cx, cy, ca] > 0.0001:
                    for cx2 in range(mapper.MAX_CELLS_X):
                        for cy2 in range(mapper.MAX_CELLS_Y):
                            for ca2 in range(mapper.MAX_CELLS_A):
                                prev_pose = np.array(mapper.from_map(cx, cy, ca))
                                curr_pose = np.array(mapper.from_map(cx2, cy2, ca2))
                                loc.bel_bar[cx2, cy2, ca2] += odom_motion_model(curr_pose, prev_pose, u) * loc.bel[cx, cy, ca]
```

### Sensor Model

The robot's sensor model computes P(Z \| X), which is the probability of the likelihood of measurement Z given that the robot is in state X. This is accomplished by using gaussians to compute the likelihood of each measurment given the expected measurement values for the state (which is precomputed by the simulation).

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

Finally, the update step corrects the filter's predictions. This is done by iterating through each cell and updating its belief using the probabilities from the sensor model. Finally, we normalize the beliefs to prevent them from becoming smaller over time and vanishing.

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

I then ran the Bayes filter using the simulation across 2 runs, which are shown below. The runs show that the Bayes belief (in blue) closely follows the ground truth data points (in green). The images also show how the Bayes filter is superior to just using the odometry motion model (in red) which does not follow the ground truth at all.

#### Run 1

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab10/run1.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

<iframe width="560" height="315" src="https://www.youtube.com/embed/o-ejXRAHLso?si=Z7hNCd3_AoQbgLxe" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

#### Run 2

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab10/run2.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

<iframe width="560" height="315" src="https://www.youtube.com/embed/bRvAS0hbRtk?si=XBP162XAvTGLlDBp" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


Acknowledgements: I referenced Aidan Derocher's website from Spring 2025 for inspiration