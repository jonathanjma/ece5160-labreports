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

Following the instructions in the handout, I was able to get the MAC address of my Artemis board and generate a new UUID for the Bluetooth Low Energy service, which is shown in the images below. Now, when I run the connect function in the Jupyter notebook, the library should be able to find/connect to my own Artemis board and update/receive characteristics.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/mac_address.png" title="example image" class="img-fluid rounded z-depth-1" width="70%" %}
    </div>
</div>
<div class="row">
    <div class="col-sm">
        {% include figure.liquid loading="eager" path="assets/img/lab1/mac_uuid.png" title="example image" class="img-fluid rounded z-depth-1" width="70%" %}
    </div>
</div>

### Task 1

To support the `ECHO` command, I added the following C code to the switch statement in the `handle_command()` function to extract the string from the command and write it to the string characteristic. The image shows that the board successfully echoes the string `Hello from my Mac!`.

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
        {% include figure.liquid loading="eager" path="assets/img/lab1/task1.png" title="example image" class="img-fluid rounded z-depth-1" width="70%" %}
    </div>
</div>

### Task 2

To support the `SEND_THREE_FLOATS` command, I added the following C code which extracts 3 floats from the commands and prints them to the serial. The images show the 3 floats which are sent from the Jupyter notebook and the serial output after they are processed by the Artemis.

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
        {% include figure.liquid loading="eager" path="assets/img/lab1/task2_cmd.png" title="example image" class="img-fluid rounded z-depth-1" width="70%" %}
    </div>
</div>
<div class="row">
    <div class="col-sm">
        {% include figure.liquid loading="eager" path="assets/img/lab1/task2_resp.png" title="example image" class="img-fluid rounded z-depth-1" width="70%" %}
    </div>
</div>

### Task 3

To support the `GET_TIME_MILLIS` command, I added the following C code which calls the `millis()` function and write it to the string characteristic. This code is tested in Task 4.

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

The image below shows the notification handler I created to recieve the string characteristic generated by sending the `GET_TIME_MILLIS` command.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/task4.png" title="example image" class="img-fluid rounded z-depth-1" width="70%" %}
    </div>
</div>

### Task 5

To conduct this test, I created a new `TIME_LOOP` command which attempts write as many values to the string characteristic as possible in a one seconds period. And based on that, it calculates the bytes for second. The images show the output of sending this command. Over the 1 second period, the Artemis sent 112 packets, each of which was 8 bytes long, giving a bytes per second of 896.

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

To support the `SEND_TIME_DATA` command, I added the following C code which iterates through the globally defined `millis_data` array for `DATA_POINTS` iterations, which is defined to be 30. After that, it writes all of these timestamps to the string characteristic one after another, which are recieved by the notification handler as shown in the image.

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
        {% include figure.liquid loading="eager" path="assets/img/lab1/task6.png" title="example image" class="img-fluid rounded z-depth-1" width="50%" %}
    </div>
</div>

### Task 7

To support the `GET_TEMP_READINGS` command, I added the following C code which iterates through the globally defined `millis_data` and `temp_data` arrays for 30 iterations. After that, it writes all of the data to the string characteristic one after another, which are parsed by the notification handler into two lists as shown in the image.

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

The differences between collecting and sending each data point one at a time and seperating the collection and sending into 2 loops are huge. The first method is slow for sending large streams of data which are collected every loop and could lead to data loss while sending each packet, but could work for slow streams of data which are only updated once in a while. The second method would be preferred for sending large streams of data without data loss, but takes a lot more memory and is less real time since all the data is collected before it is sent.

To test the second method, I used the `SEND_TIME_DATA` command and increased the number of datapoints to 100. Looking at the data, I saw that the timestamp of the first packet is 41918 and the 100th packet is 41920. This means that 100 data points are collected in 2ms, so the data resolution is 0.02ms. Meanwhile for the first method which we tried in task 5, the timestemp of the first packet is 73611 and the 30th packet is 74600. This means that 30 data points are collected in 989ms, so the data resolution is 33ms.

When I upload code to the Artemis, the Arduino IDE says there is approximately 362KB of free RAM. Assuming each data point is a 4 byte (32 bit) integer, we would be able to store around 90k data points before we run out of memory.

### (5000 Level) Task 1

To test the data rate, I used the `ECHO` command to test replies between 5 bytes and 125 bytes in 5 byte increments. I then used the notification handler to calculate the time it takes the reply to be recieved and the resulting bytes per second. Minus the outliers, the graph shows a linear increase in the bytes per second as the packet size increases. This suggests that the overhead stays relatively constant. This causes short replies to have a smaller bytes per seconds since the overhead is spread over fewer bytes, whereas for larger replies the overhead is spread over more bytes.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/data_rate_test.png" title="example image" class="img-fluid rounded z-depth-1" width="50%" %}
    </div>
</div>
<div class="row">
    <div class="col-sm">
        {% include figure.liquid loading="eager" path="assets/img/lab1/data_rate_graph.png" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

### (5000 Level) Task 2

To test the reliability, I sent `ECHO` commands with the maximum packet size allowed by the Bluetooth API, which is 150 bytes. I sent 99 of these commands one after another, and all of them were received by my laptop after being echoed by the Artemis.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.liquid loading="eager" path="assets/img/lab1/reliability_test.png" title="example image" class="img-fluid rounded z-depth-1" width="60%" %}
    </div>
</div>

## Discussion

Through this lab I learned a lot about the basics of the Artemis platform, Bluetooth Low Energy communication, and how to compare different ways of sending information between my laptop and the Artemis. I didn't run into any major problems besides the CH340 driver issue during initial setup.
