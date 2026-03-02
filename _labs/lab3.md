---
layout: page
title: Lab 3
description: Time of Flight (ToF)
img: assets/img/lab3/tof.jpg
importance: 3
---

### Setup

For the car, I was given 2 ToF sensors. I decided it would be best to put one sensor at the front of the car and one on the right side of the car in order to maximize the coverage of obstacles. So to see obstacles on the back and left of the car, we would have to rotate the car. For the wiring, I decided to use the 2 longer QWIIC connect wires for the ToF sensors since they were more position sensitive than the IMU or the QWIIC breakout board (which will use the short wires). Below is a wiring diagram showing how the all the sensors will be connected. Since we have 2 ToF sensors which will both have the same default I2C address, we will connect one of the ToF's XSHUT inputs to the Artemis' GPIO pin. This allows us programmatically turn off one of the ToF sensors when the Artemis starts up so that we can change the other sensor's default I2C address.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/wiring.png" title="example image" class="img-fluid rounded z-depth-1" width="75%" %}
    </div>
</div>

This lab involved a considerate amount of soldering, and the photos below shows the battery soldered with its new JST connector and one of ToFs soldered to its input signal and power wires.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/battery_soldered.jpeg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/tof_soldered.jpeg" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

### ToF Sensor Testing

I first started off by running the example program under `File->Examples->Apollo3->Example05_Wire_I2C` to verify the I2C address of the ToF sensor. As shown in the image below, the program prints an address of 0x29, which is different than the 0x52 address specified by the datasheet. But upon further analysis, the datasheet says the LSB of the address is the read/write bit, and 0x29 is just 0x52 shifted right a bit, so this makes sense.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/address.png" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

Comparing the short, medium, and long distance modes, we see that each mode has its own pros and cons. As seen in the images from the datasheet below, the short mode sees the shortest distance, but also performs the best under strong ambient light. It also allows for 20ms sensor readings, which allows for fast loop times. Medium mode doubles the distance compared to short mode, and long mode triples the distance. However, these modes require longer sensor readings and under strong ambient light they perform worse than short mode. Ultimately, I choose short mode since the fast measurements are crucial for the car and most obstacles should be within 1.3 meters.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/distance_stats.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/timing_stats.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

To test the ToF sensor's accuracy, repeatability, and ranging time, I used the following setup where I taped the sensor to the back of my laptop I measured the distance between it and a wall with a ruler.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/test_setup.jpeg" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

The graph below shows the sensor's distance measurements compared to a ruler. This shows that the distance measurements almost exactly match the ruler, even past the 1300mm max distance stated by the datasheet.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/accuracy.png" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

For repeatability, I did 3 sets of measuremenst where I had the sensor collect 32 samples at 3 different distances. 
- At 300mm, the standard devitation was 0.83mm
- At 600mm, the standard devitation was 1.13mm
- At 900mm, the standard devitation was 1.18mm

This shows that as the distance increases, the repeatability decreases slightly. And for ranging time, using the example code I get a ranging time of 50ms for all distances below 135cm. Above 135cm, the ranging time is 30ms, but that is probably because the sensor is just estimating the distance.

### Integrating the Sensor

To connect two ToF sensors to the Artemis, we need to utilize the XSHUT pin we talked about earlier. So when we initialize the sensors, we first pull this pin low to turn off ToF2, then initialize ToF1 and change its default address, then pull this pin high to turn on ToF2, which uses its default address.

```c
pinMode(SHUTDOWN_PIN, OUTPUT);
digitalWrite(SHUTDOWN_PIN, LOW); // turn off tof2 to set tof1 address
bool tof1_status = tof1.begin();
Serial.print("ToF1 Initialization ");
Serial.println(tof1_status != 0 ? "FAILED" : "Success");
tof1.setI2CAddress(23);
tof1.setDistanceModeShort();

digitalWrite(SHUTDOWN_PIN, HIGH); // turn on tof2
bool tof2_status = tof2.begin();
Serial.print("ToF2 Initialization ");
Serial.println(tof2_status != 0 ? "FAILED" : "Success");
tof2.setDistanceModeShort();
```
Below is photo of my setup with both ToF sensors and the IMU connected to the QWIIC breakout board.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/full_setup.jpeg" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

To evaluate the speed of my main loop with the ToF sensors, I used the following code. This code uses continuous mode so we only need to call `startRanging()` once at the start. The screenshot below shows the output of this code, and we can see that the loop time is roughly 3-5ms.

```c
tof1.startRanging();
tof2.startRanging();

...

Serial.print("System time: ");
Serial.println(millis());
if (tof1.checkForDataReady()) {
    int distance1 = tof1.getDistance();
    tof1.clearInterrupt();
    Serial.print("ToF1 Distance: ");
    Serial.println(distance1);
}
if (tof2.checkForDataReady()) {
    int distance2 = tof2.getDistance();
    tof2.clearInterrupt();
    Serial.print("ToF2 Distance: ");
    Serial.println(distance2);
}
```

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/loop_times.png" title="example image" class="img-fluid rounded z-depth-1" width="20%" %}
    </div>
</div>

To allow the collection of timestamped ToF data, I wrote the following function. 

```c
void collectToFData() {

    int distance1 = tof1.getDistance();
    tof1.clearInterrupt();
    int distance2 = tof2.getDistance();
    tof2.clearInterrupt();

    millis_data[cur_points] = millis();
    tof1_data[cur_points] = distance1;
    tof2_data[cur_points] = distance2;

    cur_points++;
    if (cur_points == DATA_POINTS) {
        tof1.stopRanging();
        tof2.stopRanging();
    }
}
```

This function is only called in the main loop function if the following conditions are met. `DATA_POINTS` was defined to be 100 which is around 10 seconds of data.
```c
if (collect_tof_data && tof1.checkForDataReady() && tof2.checkForDataReady() && cur_points < DATA_POINTS) {
    collectToFData();
}
```

Finally, I created 2 commands which start the collection of ToF data and stop the collection and send the data back via Bluetooth.
```c
case COLLECT_TOF_DATA: {
    collect_tof_data = true;
    cur_points = 0;

    tof1.startRanging();
    tof2.startRanging();

    break;
}
case GET_TOF_DATA: {
    collect_tof_data = false;

    for (int i = 0; i < min(cur_points, DATA_POINTS); i++) {
        tx_estring_value.clear();
        // time,tof1,tof2
        tx_estring_value.append(String(millis_data[i]).c_str());
        tx_estring_value.append(",");
        tx_estring_value.append(tof1_data[i]);
        tx_estring_value.append(",");
        tx_estring_value.append(tof2_data[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());

        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
    }

    break;
}
```

The plot to the left shows data points from the 2 ToF sensors graphed versus time, while the plot to the right shows IMU pitch and roll to show that my previous code was not affected.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/tof_graph.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab3/imu_graph.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

### 5000 Level Tasks

There are various kinds of distance sensors, including amplitude-based, IR triangulation, and ToF. Amplitude-based is the cheapest and has a small form factor, but it has a limited range of 10cm and doesn't work when there is a lot of ambient light. IR triangulation gives us more range and is not sensitive to surface textures, but it is more bulky and a low sample rate. Finally, ToF gives us the most range and is not sensitive to ambient light, but it requires more complex processing and has the lowest sample rate.

I tested the ToF sensor across various colors and textures to compare its performance. For colors, I held the sensor 300mm away from my computer screen and I filled my screen with various colors. Here are the results, which show that color has little to no effect on the measurements.
- White: 300mm
- Red: 300mm
- Green: 300mm
- Blue: 300mm

For textures, I also held the sensor 300mm away from the following materials. Here are the results, which show that texture has some effect on the measurements.
- Drywall: 300mm
- Wood: 290mm
- Glass: 310mm
- Carpet: 300mm

Acknowledgement: I referenced Aidan Derocher's website from Spring 2025 for inspiration