import send_esp_commands as sendEsp
import time
import sys

motor = sendEsp.ESP32Motor('COM10')

'''motor.read_serial(num_lines=1)
time.sleep(10)
sys.exit()'''

#motor.open_loop(pwm=200, cw_direction=True)
time.sleep(2)  # Wait for ESP32 to be ready
#motor._send_command("IE")
motor.get_pi_values()
motor.reset_integral()

motor.set_p_gain(0.35)
motor.set_i_gain(0.1)

motor.open_loop(200,False)
time.sleep(10)


motor.closed_loop(speed_cps=3000, cw_direction=False)
start = time.time()
while time.time() - start < 20:
    motor.get_encoder()
    motor.read_serial(num_lines=3)
    time.sleep(1.05)


motor.stop()
sys.exit()
motor.open_loop(pwm=200, cw_direction=True)

for i in range(30):
    time.sleep(1.05)
    motor.get_encoder()