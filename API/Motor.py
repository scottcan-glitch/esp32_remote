"""
Motor Controller Abstraction Layer

High-level interface for ESP32-based motor control. Provides simplified functions
for connecting, running, and configuring the motor controller.

Usage:
    import Motor
    
    # Motor is auto-connected on import
    Motor.RunMotorClosedLoop(3000, True)  # 3000 cps clockwise
    Motor.StopMotor()
    Motor.DisconnectMotor()
"""

import esp32_remote.API.send_esp_commands as sendEsp
import time
import sys
from typing import Optional, Tuple

# Global motor instance - auto-connect on import
_motor: Optional[sendEsp.ESP32Motor] = None
_default_port = 'COM10'
_default_p_gain = 0.35
_default_i_gain = 0.1


def _ensure_connected():
    """Internal: Ensure motor is connected, raise error if not."""
    global _motor
    if _motor is None:
        raise RuntimeError("Motor not connected. Call ConnectMotor() first.")
    if not _motor.ser.is_open:
        raise RuntimeError("Motor serial connection is closed.")


def ConnectMotor(port: str = None, p_gain: float = None, i_gain: float = None) -> bool:
    """
    Connect to ESP32 motor controller and configure default gains.
    
    Args:
        port: COM port (default: COM10)
        p_gain: Proportional gain (default: 0.35)
        i_gain: Integral gain (default: 0.1)
    
    Returns:
        True if connection successful
        
    Raises:
        Exception if connection fails
    """
    global _motor, _default_port, _default_p_gain, _default_i_gain
    
    if port is None:
        port = _default_port
    if p_gain is None:
        p_gain = _default_p_gain
    if i_gain is None:
        i_gain = _default_i_gain
    
    try:
        _motor = sendEsp.ESP32Motor(port)
        _motor.set_p_gain(p_gain)
        _motor.set_i_gain(i_gain)
        _motor.reset_integral()
        print(f"Motor connected on {port} with P={p_gain}, I={i_gain}")
        return True
    except Exception as e:
        print(f"Failed to connect motor: {e}")
        _motor = None
        raise


def DisconnectMotor():
    """
    Stop motor and close serial connection.
    """
    global _motor
    if _motor is not None:
        try:
            _motor.stop()
        except Exception as e:
            print(f"Warning: Error stopping motor during disconnect: {e}")
        try:
            _motor.close()
        except Exception as e:
            print(f"Warning: Error closing serial connection: {e}")
        _motor = None
        print("Motor disconnected")


def RunMotorClosedLoop(speed: int, direction: bool = True):
    """
    Run motor in closed-loop speed control mode.
    
    Args:
        speed: Target speed in counts per second (cps)
        direction: True for clockwise, False for counter-clockwise
        
    Raises:
        RuntimeError if motor not connected
        
    Example:
        RunMotorClosedLoop(3000, True)   # 3000 cps clockwise
        RunMotorClosedLoop(1500, False)  # 1500 cps counter-clockwise
    """
    _ensure_connected()
    _motor.closed_loop(speed_cps=speed, cw_direction=direction)


def RunMotorOpenLoop(speed_pwm: int, direction: bool = True):
    """
    Run motor in open-loop PWM control mode.
    
    Args:
        speed_pwm: PWM duty cycle (0-255)
        direction: True for clockwise, False for counter-clockwise
        
    Raises:
        RuntimeError if motor not connected
        ValueError if speed_pwm not in range 0-255
        
    Example:
        RunMotorOpenLoop(200, True)  # PWM 200 clockwise
    """
    _ensure_connected()
    if not 0 <= speed_pwm <= 255:
        raise ValueError(f"PWM must be 0-255, got {speed_pwm}")
    _motor.open_loop(pwm=speed_pwm, cw_direction=direction)


def StopMotor():
    """
    Stop the motor immediately.
    
    Raises:
        RuntimeError if motor not connected
    """
    _ensure_connected()
    _motor.stop()


def SetControllerGains(p_gain: float, i_gain: float, 
                       encoder_update_rate_ms: Optional[int] = None,
                       controller_update_rate_ms: Optional[int] = None):
    """
    Set PI controller gains and optionally update rates.
    
    Args:
        p_gain: Proportional gain
        i_gain: Integral gain
        encoder_update_rate_ms: Encoder speed calculation rate in ms (5-500)
        controller_update_rate_ms: PI controller update rate in ms (5-500)
        
    Raises:
        RuntimeError if motor not connected
        ValueError if update rates out of range
        
    Example:
        SetControllerGains(0.5, 0.15)  # Set gains only
        SetControllerGains(0.5, 0.15, 10, 50)  # Set gains and update rates
    """
    _ensure_connected()
    
    if encoder_update_rate_ms is not None:
        if not 5 <= encoder_update_rate_ms <= 500:
            raise ValueError(f"Encoder update rate must be 5-500 ms, got {encoder_update_rate_ms}")
        _motor.set_encoder_update_rate(encoder_update_rate_ms)
    
    if controller_update_rate_ms is not None:
        if not 5 <= controller_update_rate_ms <= 500:
            raise ValueError(f"Controller update rate must be 5-500 ms, got {controller_update_rate_ms}")
        _motor.set_pi_update_rate(controller_update_rate_ms)
    
    _motor.set_p_gain(p_gain)
    _motor.set_i_gain(i_gain)
    _motor.reset_integral()


def GetControllerGains() -> Optional[str]:
    """
    Get current PI controller parameters and state.
    
    Returns:
        String with current Kp, Ki, and Integral values
        
    Raises:
        RuntimeError if motor not connected
        
    Example:
        gains = GetControllerGains()
        print(gains)  # "Kp=0.350 Ki=0.100 Integral=0.000"
    """
    _ensure_connected()
    return _motor.get_pi_values()


def PrintControllerGains():
    """
    Print current PI controller parameters to console.
    
    Raises:
        RuntimeError if motor not connected
    """
    gains = GetControllerGains()
    if gains:
        print(f"Controller Gains: {gains}")
    else:
        print("No response from controller")


def GetEncoderPosition() -> Optional[str]:
    """
    Get current encoder position and speed.
    
    Returns:
        String with encoder position and speed
        
    Raises:
        RuntimeError if motor not connected
        
    Example:
        encoder = GetEncoderPosition()
        print(encoder)  # "Encoder: Pos=12345 Speed=3021.5 cps"
    """
    _ensure_connected()
    return _motor.get_encoder()


def ResetIntegral():
    """
    Reset the integral term in the PI controller.
    Useful when switching speeds or if integral has saturated.
    
    Raises:
        RuntimeError if motor not connected
    """
    _ensure_connected()
    _motor.reset_integral()


def InvertDirection():
    """
    Toggle motor direction inversion.
    Use if motor spins opposite to commanded direction.
    
    Raises:
        RuntimeError if motor not connected
    """
    _ensure_connected()
    _motor.invert_direction()


def IsConnected() -> bool:
    """
    Check if motor is connected and serial port is open.
    
    Returns:
        True if connected, False otherwise
    """
    global _motor
    if _motor is None:
        return False
    try:
        return _motor.ser.is_open
    except:
        return False


def GetUpdateRates() -> Optional[str]:
    """
    Get current PI controller and encoder update rates.
    
    Returns:
        String with both update rates
        
    Raises:
        RuntimeError if motor not connected
    """
    _ensure_connected()
    return _motor.get_update_rates()


# Auto-connect on import with default settings
try:
    ConnectMotor()
except Exception as e:
    print(f"Warning: Auto-connect failed: {e}")
    print("Call ConnectMotor() manually with correct COM port")
    _motor = None
    