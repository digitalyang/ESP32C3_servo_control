# GATT Server for Servo

![Version](https://img.shields.io/badge/version-1.0.0-blue)
![Platform](https://img.shields.io/badge/platform-ESP32--C3-orange)
![Framework](https://img.shields.io/badge/framework-ESP--IDF%20v5.1.2-green)
![License](https://img.shields.io/badge/license-MIT-lightgrey)
![Servos](https://img.shields.io/badge/servos-4CH-yellow)
![BLE](https://img.shields.io/badge/BLE-GATT%20Server-blue)

## Introduction

This is a GATT server for a servo motor. It allows the user to control the angle of the servo motor using a BLE connection.It uses the ESP32-S2 chip and the Bluetooth LE stack.Among them, the servo motor has various motion modes. Each motion mode has default motion parameters, which can all be modified via Bluetooth.

## Project structure

The project is structured as follows:

``` console
gatt_server_servo
├── main
│   ├── gatt_demo.c
│   ├── servo_controller.c
│   include
│       ├── servo_controller.h
| readme.md
```

## How to use

If user want to compile this project or download the program to user development board, please use the official Espressif SDK or the Espressif SDK extension for VSCode. User can download the SDK from <https://idf.espressif.com/zh-cn/index.html>  
However, the first compilation may take a while.

To use this GATT server, follow these steps:
example for nRF Connect app:

1. Compile and flash the code to the ESP32-S2 board.
2. Open the Serial Monitor and wait for the following message: "GATT server started successfully".
3. Connect to the ESP32-S2 board using a BLE app such as LightBlue or nRF Connect.
4. Search for the GATT server using the server name "ESP_GATT_DEMO"
5. Select the GATT server and click on "Connect".
6. Click on "Unkown Services" to view the available services.And user can rename the service name to "Servo" in the app if user want.
7. Click on "Unkown characteristic" to view the available characteristics.And user can rename the characteristic name to "set-motion" in the app if user want.
8. Click on "set-motion" to write the commanded value to the characteristic.For example, user can write "07-00-11-60-14" to set the servo motor to vibration mode. Note that all the data written is in hexadecimal.Then I will demonstrate the functions corresponding to various control codes.

## Control codes

### codedata structure

The code data structure is as follows:

```c
typedef struct {
    speed_mode_t mode;
    int repeat_count;
    ledc_channel_t servo_channel;
    int range;
    int speed;
    int delay_ms;
    SemaphoreHandle_t semaphore; // synchronization
} motion_stage_t;
```

user can modify the parameters of `mode`, `servo_switch`, `delay_ms`, `speed`, and `range` to change the motion behavior of the servo motor.The structure of data written to Bluetooth is similar.  
For example

```c
  07  -    00       -      11   -   60   -   14 
   ^        ^               ^        ^        ^ 
 mode  servo_switch    delay_ms    range    speed 

```

`mode`: means that users can set the servo motor's actions.  
`servo_switch`: means that servo turns on or off.  However, it defaults to `00` when the server not in stop mode.  
`delay_ms`: means that users can set the delay time between each action.  
`range`: means that users can set the step range of the servo motor.  
`speed`: means that users can set the `step angle` of the servo motor.  

Servo motor modes are defined as follows.

| write data        | function                                              |
|-------------------|-------------------------------------------------------|
| 00                | stop                                                  |
| 0001              | start                                                 |
| 01                | test motion                                           |
| 02-xx-xx-yy-zz    | control the servo motor in the upper right corner, xx is the delay time, yy is the range, and zz is the step angle.  |
| 03-xx-xx-yy-zz    | control the servo motor in the upper left corner, xx is the delay time, yy is the range, and zz is the step angle.  |
| 04-xx-xx-yy-zz    | control the servo motor in the lower left corner, xx is the delay time, yy is the range, and zz is the step angle.  |
| 05-xx-xx-yy-zz    | control the servo motor in the lower right corner, xx is the delay time, yy is the range, and zz is the step angle.  |
| 06-xx-xx-yy-zz    | control all four servo motors to swing at low speed, xx is the delay time(no debug) |
| 07-xx-xx-yy-zz    | control all four servo motors to Custom modes      (no debug)     |

### Note

note 1 : When the write data is `02-xx`, `03-xx`, `04-xx`, `05-xx`, `06-xx`,`07-xx`,if `xx` is `0`, the delay time is `500ms`. And the mode of the servo motor is `speed_mode_t speed_mode = SPEED_MODE_LOW`.  

note 2 : When the write data is `07-xx-xx-xx-xx`, user can modify the `repeat_count`, `delay_ms`, `range`, `speed` to change the motion behavior of the all four servo motors.  

note 3 : If user want to turn off the servo motor, the write data should be `00`. When the servo motor is in stop mode, if user want to turn on the servo motor, the write data `must` be `0001`.Otherwise, the servo motor will not perform any actions as the process has been terminated.

## Custom modes example (debuging)

This is some example code for custom modes.

```c
07 // default low speed swing mode
07-00-11-60-14 // vibration mode, it will delay time is 11ms, range is 60, step angle is 14.
07-00-11-08-5  // swing mode, it will delay time is 11ms, range is 8, step angle is 5.
07-00-11-08-01 // low speed swing mode, it will delay time is 11ms, range is 10, step angle is 1.
```

## Frequency calculation

* The period T is defined by the following formula.  
$$T = \frac{\theta_{step\_range}}{\theta_{step\_angle}}\times t_{step\_delay} \times 2$$

* The frequency of the servo motor is calculated as follows:
  $$f=\frac{1}{T}$$

## ESP32 flash command

```c
 E:\esp32\Espressif\python_env\idf5.3_py3.11_env\Scripts\python -m esptool --chip esp32c3 -b 460800 --before default_reset --after hard_reset  --port COM7 write_flash --flash_mode dio --flash_size 2MB --flash_freq 80m 0x0 bootloader/bootloader.bin 0x10000 gatt_server_demos.bin 0x8000 partition_table/partition-table.bin

```

## ESP32 monitor command

```c
 & 'E:\esp32\Espressif\python_env\idf5.3_py3.11_env\Scripts\python.exe' 'E:\esp32\Espressif\frameworks\esp-idf-v5.3.1\tools\idf_monitor.py' -p COM7 -b 115200 --toolchain-prefix riscv32-esp-elf- --make '''E:\esp32\Espressif\python_env\idf5.3_py3.11_env\Scripts\python.exe'' ''E:\esp32\Espressif\frameworks\esp-idf-v5.3.1\tools\idf.py''' --target esp32c3 'e:\esp32vscode\workspace\gatt_server_servo\build\gatt_server_demos.elf'

 ```
