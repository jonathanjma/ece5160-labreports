---
layout: page
title: Lab 7
description: Kalman Filter
img: assets/img/lab7/kf.png
importance: 7
---

### Estimating Drag and Momentum

Before we can implement a Kalman filter, we first need to build a basic model of our system. Using Newton's Laws as seen in lecture, we can derive the dynamics of our system, where $u$ is the control (PWM) input, $m$ is the mass of the car, $d$ is the drag on the car, and $\dot{x}$ the change in distance over time.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab7/newton_laws.png" title="example image" class="img-fluid rounded z-depth-1" width="30%" %}
    </div>
</div>

We can then rewrite these dynamics into the form $\dot{x} = Ax + Bu$ to get our state space equations, where $A$ is the state matrix and $B$ is the control input matrix. In this case our state $x$ is represented by our current position and velocity, so $x = \begin{bmatrix} x \\\\ \dot{x} \end{bmatrix}$. Below are the state space equations for our system.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab7/state_space_eqs.png" title="example image" class="img-fluid rounded z-depth-1" width="30%" %}
    </div>
</div>

To estimage the drag ($d$) and mass ($m$) terms, I performed a step response with the car where I drove the car forward towards the wall at a PWM value of 130, a 50% duty cycle, while recording raw ToF data. I then computed the derivative of the ToF measurements to get the velocity of the car at each timestep, and cut off the noisy data right as the car starts and at the end when it crashes into the wall.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab7/step_response.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

To find the drag, the car needs to have reached a constant speed. However, since the car never reached constant speed during my testing, I calculated this by computing the exponential fit of the velocity data and finding the asymptote, which is shown below. For simplication, $u$ is set to 1, which gives us $d = \frac{u}{\dot{x}} = \frac{1 N}{-2.89 m/s} = -0.346 kg/s$. To find the mass, I first found the 90% rise time by finding the intersection of the exponential fit line with the 90% of the steady state speed, so $t_{0.9} = 1.739 s$. So, $m = \frac{-d \cdot t_{0.9}}{\ln 0.1} = -0.261 kg$.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab7/exp_fit.png" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

### Python Simulation

Putting everything together, I computed the $A$, $B$, and $C$ matricies to be: $A = \begin{bmatrix} 0 & 1 \\\\ 0 & -1.32384312 \end{bmatrix}$, $B = \begin{bmatrix} 0 \\\\ -3.82491338 \end{bmatrix}$, and $C = \begin{bmatrix} 1 & 0 \end{bmatrix}$.

For measurement noise, the ToF sensor's datasheet says it has an accuracy of 20mm, so $\sigma_3 = 20$. For process/model noise, I initially started off with 20 for both position and velocity. However, as shown in the graph on the left, this results in a stepped graph where the distance stays mostly constant between updates and predicts distances in the wrong direction while the car backs up. After a bit of tuning, I settled on $\sigma_1 = 10$ and $\sigma_2 = 70$, which results in the graph on the right. This gives a much smoother graph, and while it is a little jagged when the direction changes, it is able to recover more quickly. So the final covariance matrices are  $\text{Sigma}_u = \begin{bmatrix} 100 & 0 \\\\ 0 & 4900 \end{bmatrix}$ and $\text{Sigma}_z = \begin{bmatrix} 400 \end{bmatrix}$.

<div class="row">
    <div class="col-sm-6 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab7/sigma_initial.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-6 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab7/sigma_tuned.png" title="example image" class="img-fluid rounded z-depth-1"  %}
    </div>
</div>

The Python code below which computes a Kalman filter step was mostly taken from the lecture slides, except that I added an `update` parameter to control whether we only want to predict distance or if we want to update the filter as well.

```py
def kf(mu, sigma, u, y, update):
    mu_p = Ad.dot(mu) + Bd.dot(u) 
    sigma_p = Ad.dot(sigma.dot(Ad.transpose())) + sig_u

    if not update: # only run prediction
        return mu_p, sigma_p
    
    sigma_m = C.dot(sigma_p.dot(C.transpose())) + sig_z
    kkf_gain = sigma_p.dot(C.transpose().dot(np.linalg.inv(sigma_m)))
    y_m = y - C.dot(mu_p)
    mu = mu_p + kkf_gain.dot(y_m)    
    sigma = (np.eye(2) - kkf_gain.dot(C)).dot(sigma_p)

    return mu, sigma
```

The following Python code runs the simulated Kalman filter in the Juypter notebook. Since the cycle time of the Artemis control loop is around 10ms and new ToF measurements are recieved around every 54ms, the simulation predicts 5 distances for every ToF update.

```py
mu = np.array([[dist_arr[0]], [0]])
sigma = sig_u

k_time = list(range(time_arr[0], time_arr[-1], 10)) # assumes all tof data >10ms apart
k_dist = []
j = 0

for time in k_time:
    update = time >= time_arr[j] # whether to predict only or update
    
    mu, sigma = kf(mu, sigma, pwm_arr[j]/130.0, dist_arr[j], update) # make sure to scale u out of 130
    k_dist.append(mu[0][0])
    
    if update:
        j += 1
```

Below is a full graph of the simulated Kalman filter running with ToF and PWM data collected from a run of the linear PID, which shows the filter tracking the raw ToF data very well.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab7/kf_sim.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

### Implementing on Robot

To implement the Kalman filter on my robot, I used the `BasicLinearAlgebra` library to create the all of the necessary matrices and to perfom all the matrix operations in the prediction and update steps. One change from the simulation is that since each control loop cycle has a slightly different execution time (it can vary from 5-15ms), I decided to compute the discretized $Ad$ and $Bd$ matrices dynamically to improve accuracy.

```c
Matrix<2,2> A = {0, 1,
                 0, -1.32384312};
Matrix<2,1> B = {0, 
                 -3.82491338};
Matrix<1,2> C = {1, 0}; 

Matrix<2,2> id = {1, 0,
                  0, 1};

Matrix<2,2> sig_u = {100, 0,
                     0, 4900};
Matrix<1> sig_z = {400};

Matrix<2,1> mu = {1575, 0};
Matrix<2,2> sigma = sig_u;

void kalman_filter(int pwm, int distance, bool update) {
    float dt_s = (float)(millis() - last_kalman_time)/1000.0;
    Matrix<2,2> Ad = id + dt_s * A;
    Matrix<2,1> Bd = dt_s * B;

    float u = pwm;
    Matrix<2,1> mu_p = Ad * mu + Bd * u; 
    Matrix<2,2> sigma_p = Ad * (sigma * ~Ad) + sig_u;

    // only run prediction step
    if (!update) {
        mu = mu_p;
        sigma = sigma_p;
    } else {
        Matrix<1> sigma_m = C * (sigma_p * ~C) + sig_z;
        Matrix<2,1> kkf_gain = sigma_p * (~C * Inverse(sigma_m));
        float y = distance;
        Matrix<1> y_m = y - C * mu_p;
        mu = mu_p + kkf_gain * y_m;
        sigma = (id - kkf_gain * C) * sigma_p;
    }
    last_kalman_time = millis();
}
```

After replacing the extrapolation code with the Kalman filter code, I ran the car's linear PID with the same PID parameters. The filter performed well, and the car behaved basically the same as it did with the extrapolation code. The image on the left shows the filter's output layered on top of the ToF output as well as the PWM values while the image on the right shows a detailed view of the filter's output when the direction changes.

<div class="row">
    <div class="col-sm-8 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab7/kf_on_robot.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-4 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab7/robot_kf_detail.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

<iframe width="560" height="315" src="https://www.youtube.com/embed/XAV1zblo1-U?si=tknYSkmDiIYeN57Y" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Acknowledgement: I referenced Aidan Derocher's and Katarina Duric's website from Spring 2025 for inspiration and to sanity check my values.