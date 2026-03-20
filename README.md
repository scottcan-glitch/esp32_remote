General:

    C firmware for open or closed loop (PI) speed control for brushed DC motor. Compatible with quadrature encoder using esp32 16bit hardware counters which are software extended. Also has functionality for fusing IMU data (mpu9250 + Madgwick filter). Python API for polling imu/encoder data, and setting the mode of the esp32 state machine. CLI for setting PI gains and calibrating IMU.

Flashing:

    platformio project for flashing esp32 from raspi!
        Build the (c) code!:
            pio run
        Flash the ESP32 over USB:
            pio run -t upload
        If connectected over usb, check which comm with:
            ls /dev/ttyUSB*
            ls /dev/ttyACM*
        should show something like
            /dev/ttyUSB0
        then use minicom to print serial comms
            minicom -b 115200 -D /dev/ttyUSB0
        to exit minicom:
            ctr+a, x, ENTER
