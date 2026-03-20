API for controlling esp32 state machine.

    send_esp_commands is the lowest level API commands. Running this script will run a
    command line interface where easy tuning of PI controller gains and other debugging
    can be done in line.

    Motor.py is higher level API abstraction. Connects and controls motor. Import this.

    test_send_esp_commands.py is a testing script only