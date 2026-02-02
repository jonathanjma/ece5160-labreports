---
layout: page
title: Lab 1
description: Artemis and Bluetooth
img: assets/img/lab1/artemis_nano.webp
importance: 1
---

## Lab 1A

### Setup

Following the instructions on the lab handout, I was able to latest versions of Arduino IDE and Sparkfun Appollo3 boards manager. However, I soon ran into an issue running the Blink example where my computer was not able to communicate with the Artemis' bootloader. This was fixed by following the directions to update CH340 driver on my Mac.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/initial_setup.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Artemis Board Shows Up in the Arduino IDE!
</div>

### Blink

After running the blink example program under `File->Examples->01.Basics->Blink`, the video below shows it working as expected!

<iframe width="560" height="315" src="https://www.youtube.com/embed/vkfAYiTNDPk?si=B3hR1s9mWefxINJr" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Serial

After running the serial example program under `File->Examples->Apollo3->Example4_Serial`, I was able to get the expected output over the serial line on startup and after echoeing strings.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/serial.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

### Temperature

After running the temperature sensor example under `File->Examples->Apollo3->Example2_analogRead`, I was able to recieve raw sensor readings from the Artemis' temperature sensor. The screenshot on the left shows the value of the sensor before I started warming it up and the one on the right shows the value of the sensor after I had held my finger on it for a while.

<div class="row">
    <div class="col-sm-6 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/temp1.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-6 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/temp2.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

### Microphone

After running the microphone example under `File->Examples->PDM->Example1_MicrophoneOutput`, I was able to receive microphone frequency readings from the on-board microphone. The video below shows the microphone's readings as I whistled.

<iframe width="560" height="315" src="https://www.youtube.com/embed/GHye9XGb2pM?si=y1pCQCg8UTtR7v2f" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### (5000 Level) Tuner

To create my simple tuner, I added a few if statements after the line in the example where the loudest frequency is printed. This code should be able to detect the notes middle C, middle F, and middle A# as long as they are played +/- 10 units from the reference frequencies in the code. In the video below, this was tested using an app which generated the sounds for these notes.

```c
float middle_c = 526;
float middle_f = 698;
float middle_a_sharp = 926;

if (abs(ui32LoudestFrequency - middle_c) < 10) {
Serial.printf("Note detected: Middle C\n");
} else if (abs(ui32LoudestFrequency - middle_f) < 10) {
Serial.printf("Note detected: Middle F\n");
} else if (abs(ui32LoudestFrequency - middle_a_sharp) < 10) {
Serial.printf("Note detected: Middle A Sharp\n");
}
```

<iframe width="560" height="315" src="https://www.youtube.com/embed/cBvwQnpaSKs?si=HvQFE2R3wJS5ts-A" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Lab 1B

### Prelab

### Task 1

```c
case ECHO: {
    char char_arr[MAX_MSG_SIZE];
    success = robot_cmd.get_next_value(char_arr);
    if (!success) return;

    tx_estring_value.clear();
    tx_estring_value.append("Echo: ");
    tx_estring_value.append(char_arr);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    Serial.print("Sent back: ");
    Serial.println(tx_estring_value.c_str());
    
    break;
}
```

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/task1.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

### Task 2

```c
case SEND_THREE_FLOATS: {
    float f1, f2, f3;
    success = robot_cmd.get_next_value(f1);
    if (!success) return;
    success = robot_cmd.get_next_value(f2);
    if (!success) return;
    success = robot_cmd.get_next_value(f3);
    if (!success) return;

    Serial.print("Three Floats: ");
    Serial.print(f1);
    Serial.print(", ");
    Serial.print(f2);
    Serial.print(", ");
    Serial.println(f3);;

    break;
}
```

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/task2_cmd.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="row">
    <div class="col-sm">
        {% include figure.liquid loading="eager" path="assets/img/lab1/task2_resp.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

### Task 3

```c
case GET_TIME_MILLIS: {
    tx_estring_value.clear();
    tx_estring_value.append("T:");
    tx_estring_value.append(String(millis()).c_str());
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    Serial.print("Sent back: ");
    Serial.println(tx_estring_value.c_str());

    break;
}
```

### Task 4

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/task4.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

### Task 5

```c
case TIME_LOOP: {

    unsigned long start = millis();
    int count = 0;
    while (millis() - start < 1000) { // 1 seconds
        tx_estring_value.clear();
        tx_estring_value.append("T:");
        tx_estring_value.append(String(millis()).c_str());
        tx_characteristic_string.writeValue(tx_estring_value.c_str());

        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
        count++;
    }

    tx_estring_value.clear();
    tx_estring_value.append("bps:");
    tx_estring_value.append((count*8)/1.0); // each message is 8 bytes
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    Serial.print("Sent back: ");
    Serial.println(tx_estring_value.c_str());

    break;
}
```

<div class="row">
    <div class="col-sm-6 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/task5_start.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm-6 mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/task5_bps.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

### Task 6

```c
case SEND_TIME_DATA: {
    for (int i = 0; i < DATA_POINTS; i++) {
        millis_data[i] = millis();
    }

    for (int i = 0; i < DATA_POINTS; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("T:");
        tx_estring_value.append(String(millis_data[i]).c_str());
        tx_characteristic_string.writeValue(tx_estring_value.c_str());

        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
    }

    break;
}
```

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/task6.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

### Task 7

```c
case GET_TEMP_READINGS: {
    for (int i = 0; i < DATA_POINTS; i++) {
        millis_data[i] = millis();
        temp_data[i] = getTempDegF();
    }

    for (int i = 0; i < DATA_POINTS; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("T:");
        tx_estring_value.append(String(millis_data[i]).c_str());
        tx_estring_value.append(" F:");
        tx_estring_value.append(temp_data[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());

        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
    }

    break;
}
```

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/task7.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

### Task 8

discussion

### (5000 Level) Task 1

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/data_rate_test.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="row">
    <div class="col-sm">
        {% include figure.liquid loading="eager" path="assets/img/lab1/data_rate_graph.png" title="example image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>

### (5000 Level) Task 2



## Discussion
