
"""
ESP32 Motor Controller API

Controls the ESP32 motor state machine via serial communication.

Usage:
    from send_esp_commands import ESP32Motor
    
    motor = ESP32Motor('/dev/ttyUSB0')  # or 'COM6' on Windows
    motor.closed_loop(speed_cps=100, cw_direction=True)
    motor.open_loop(pwm=150, cw_direction=False)
    motor.stop()
    motor.close()
"""

import serial
import time
from typing import Optional


class ESP32Motor:
    """API for controlling ESP32 motor controller via serial."""
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        Initialize serial connection to ESP32.
        
        Args:
            port: Serial port (e.g., '/dev/ttyUSB0' or 'COM6')
            baudrate: Communication speed (default: 115200)
            timeout: Read timeout in seconds (default: 1.0)
        """
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(0.5)  # Wait for connection to stabilize
        self._clear_buffer()
        print(f"Connected to ESP32 on {port}")
    
    def _clear_buffer(self):
        """Clear any pending data in the serial buffer."""
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
    
    def _send_command(self, cmd: str) -> Optional[str]:
        """
        Send command to ESP32 and read response.
        
        Args:
            cmd: Command string to send
            
        Returns:
            Response string from ESP32, or None if no response
        """
        # Clear any accumulated debug output before sending command
        self.ser.reset_input_buffer()
        
        if not cmd.endswith('\n'):
            cmd += '\n'
        
        self.ser.write(cmd.encode())
        time.sleep(0.05)  # Brief delay for processing
        
        # Read only the immediate response line
        if self.ser.in_waiting > 0:
            response = self.ser.readline().decode('utf-8', errors='ignore').strip()
            return response
        return None
    
    def closed_loop(self, speed_cps: int, cw_direction: bool = True) -> Optional[str]:
        """
        Run motor in closed-loop speed control mode.
        
        Args:
            speed_cps: Target speed in counts per second
            cw_direction: True for clockwise, False for counter-clockwise
            
        Returns:
            Response from ESP32
            
        Example:
            motor.closed_loop(100, True)   # 100 cps clockwise
            motor.closed_loop(-50, False)  # 50 cps counter-clockwise
        """
        direction = 1 if cw_direction else 0
        cmd = f"M{speed_cps}D{direction}"
        response = self._send_command(cmd)
        print(f"Closed Loop: {speed_cps} cps, {'CW' if cw_direction else 'CCW'} -> {response}")
        return response
    
    def open_loop(self, pwm: int, cw_direction: bool = True) -> Optional[str]:
        """
        Run motor in open-loop PWM control mode.
        
        Args:
            pwm: PWM duty cycle (0-255)
            cw_direction: True for clockwise, False for counter-clockwise
            
        Returns:
            Response from ESP32
            
        Example:
            motor.open_loop(150, True)  # PWM 150 clockwise
        """
        if not 0 <= pwm <= 255:
            raise ValueError(f"PWM must be 0-255, got {pwm}")
        
        direction = 1 if cw_direction else 0
        cmd = f"OL{pwm}D{direction}"
        response = self._send_command(cmd)
        print(f"Open Loop: PWM={pwm}, {'CW' if cw_direction else 'CCW'} -> {response}")
        return response
    
    def stop(self) -> Optional[str]:
        """
        Stop the motor immediately.
        
        Returns:
            Response from ESP32
        """
        response = self._send_command("STOP")
        print(f"Motor Stopped -> {response}")
        return response
    
    def set_p_gain(self, kp: float) -> Optional[str]:
        """
        Set proportional gain for closed-loop control.
        
        Args:
            kp: Proportional gain value
            
        Returns:
            Response from ESP32
        """
        cmd = f"P{kp}"
        response = self._send_command(cmd)
        print(f"Set P gain -> {response}")
        return response
    
    def set_i_gain(self, ki: float) -> Optional[str]:
        """
        Set integral gain for closed-loop control.
        
        Args:
            ki: Integral gain value
            
        Returns:
            Response from ESP32
        """
        cmd = f"I{ki}"
        response = self._send_command(cmd)
        print(f"Set I gain -> {response}")
        return response
    
    def reset_integral(self) -> Optional[str]:
        """
        Reset the integral term in the PI controller.
        Useful when switching speeds or if integral has saturated.
        
        Returns:
            Response from ESP32
        """
        response = self._send_command("RESET")
        print(f"Reset Integral -> {response}")
        return response
    
    def get_pi_values(self) -> Optional[str]:
        """
        Get current PI controller parameters and state.
        
        Returns:
            Response from ESP32 with Kp, Ki, and Integral values
        """
        response = self._send_command("G")
        print(f"PI Values -> {response}")
        return response
    
    def invert_direction(self) -> Optional[str]:
        """
        Toggle motor direction inversion.
        Use this if the motor spins opposite to commanded direction.
        
        Returns:
            Response from ESP32
        """
        response = self._send_command("INVERT")
        print(f"Invert Direction -> {response}")
        return response
    
    def get_encoder(self) -> Optional[str]:
        """
        Get current encoder position and speed.
        
        Returns:
            Response from ESP32 with encoder values
        """
        response = self._send_command("E")
        if response:
            print(f"Encoder -> {response}")
        return response
    
    def set_pi_update_rate(self, rate_ms: int) -> Optional[str]:
        """
        Set PI controller update rate in milliseconds.
        
        Args:
            rate_ms: Update interval in ms (5-500)
            
        Returns:
            Response from ESP32
            
        Example:
            motor.set_pi_update_rate(50)  # 20Hz update rate
        """
        response = self._send_command(f"PI_RATE{rate_ms}")
        print(f"Set PI Rate -> {response}")
        return response
    
    def set_encoder_update_rate(self, rate_ms: int) -> Optional[str]:
        """
        Set encoder speed calculation update rate in milliseconds.
        
        Args:
            rate_ms: Update interval in ms (5-500)
            
        Returns:
            Response from ESP32
            
        Example:
            motor.set_encoder_update_rate(10)  # 100Hz update rate
        """
        response = self._send_command(f"ENC_RATE{rate_ms}")
        print(f"Set Encoder Rate -> {response}")
        return response
    
    def get_update_rates(self) -> Optional[str]:
        """
        Get current PI and encoder update rates.
        
        Returns:
            Response from ESP32 with both rates
        """
        response = self._send_command("GR")
        print(f"Update Rates -> {response}")
        return response
    
    def read_serial(self, num_lines: int = 10, timeout: float = 5.0, query_encoder: bool = True) -> list:
        """
        Read serial output from ESP32 (useful for debugging).
        
        Args:
            num_lines: Number of lines to read
            timeout: Maximum time to wait for all lines
            query_encoder: If True, queries encoder speed before reading
            
        Returns:
            List of received lines
        """
        lines = []
        start_time = time.time()
        
        # Query encoder speed first if requested
        if query_encoder:
            self.get_encoder()
            time.sleep(0.1)
        
        while len(lines) < num_lines and (time.time() - start_time) < timeout:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    lines.append(line)
                    print(line)
        
        return lines
    
    def close(self):
        """Close the serial connection."""
        if self.ser.is_open:
            self.ser.close()
            print("Serial connection closed")


# ===================
# Example usage and CLI interface
# ===================
if __name__ == "__main__":
    import sys
    
    # Determine port based on OS
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        # Default ports
        import platform
        if platform.system() == 'Windows':
            port = 'COM6'
        else:
            port = '/dev/ttyUSB0'
    
    try:
        motor = ESP32Motor(port)
        
        print("\nESP32 Motor Controller")
        print("=" * 50)
        
        while True:
            print("\nCommands:")
            print("  1. Closed Loop Control")
            print("  2. Open Loop Control")
            print("  3. Stop Motor")
            print("  4. Set P Gain")
            print("  5. Set I Gain")
            print("  6. Reset Integral")
            print("  7. Get PI Values")
            print("  8. Set PI Update Rate")
            print("  9. Set Encoder Update Rate")
            print(" 10. Get Update Rates")
            print(" 11. Read Serial Output")
            print(" 12. Exit")
            
            choice = input("\nSelect option: ").strip()
            
            if choice == '1':
                speed = int(input("Enter speed (cps): "))
                direction = input("CW direction? (y/n): ").lower() == 'y'
                motor.closed_loop(speed, direction)
                
            elif choice == '2':
                pwm = int(input("Enter PWM (0-255): "))
                direction = input("CW direction? (y/n): ").lower() == 'y'
                motor.open_loop(pwm, direction)
                
            elif choice == '3':
                motor.stop()
                
            elif choice == '4':
                kp = float(input("Enter P gain (Kp): "))
                motor.set_p_gain(kp)
                
            elif choice == '5':
                ki = float(input("Enter I gain (Ki): "))
                motor.set_i_gain(ki)
                
            elif choice == '6':
                motor.reset_integral()
                
            elif choice == '7':
                motor.get_pi_values()
                
            elif choice == '8':
                rate = int(input("Enter PI update rate in ms (5-500): "))
                motor.set_pi_update_rate(rate)
                
            elif choice == '9':
                rate = int(input("Enter encoder update rate in ms (5-500): "))
                motor.set_encoder_update_rate(rate)
                
            elif choice == '10':
                motor.get_update_rates()
                
            elif choice == '11':
                print("\nReading serial output (Ctrl+C to stop)...")
                motor.read_serial(num_lines=20)
                
            elif choice == '12':
                break
            
            else:
                print("Invalid option")
        
        motor.close()
        
    except KeyboardInterrupt:
        print("\nExiting...")
        if 'motor' in locals():
            motor.close()
    except Exception as e:
        print(f"Error: {e}")
        if 'motor' in locals():
            motor.close()
