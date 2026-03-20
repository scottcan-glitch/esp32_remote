"""
Test script for Motor.py abstraction layer

Tests basic motor control functions with the ESP32 controller.
"""

import Motor
import time

def test_connection():
    """Test motor connection status"""
    print("\n=== Testing Connection ===")
    if Motor.IsConnected():
        print("✓ Motor is connected")
        return True
    else:
        print("✗ Motor is not connected")
        print("Attempting to connect...")
        try:
            Motor.ConnectMotor()
            print("✓ Connection successful")
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False

def test_gains():
    """Test getting and setting controller gains"""
    print("\n=== Testing Controller Gains ===")
    try:
        # Get current gains
        print("Current gains:")
        Motor.PrintControllerGains()
        
        # Set new gains
        print("\nSetting gains to P=0.5, I=0.15...")
        Motor.SetControllerGains(0.5, 0.15)
        Motor.PrintControllerGains()
        
        # Reset to defaults
        print("\nResetting to default gains P=0.35, I=0.1...")
        Motor.SetControllerGains(0.35, 0.1)
        Motor.PrintControllerGains()
        
        print("✓ Gain tests passed")
        return True
    except Exception as e:
        print(f"✗ Gain tests failed: {e}")
        return False

def test_encoder():
    """Test encoder reading"""
    print("\n=== Testing Encoder ===")
    try:
        for i in range(3):
            encoder_data = Motor.GetEncoderPosition()
            print(f"  Read {i+1}: {encoder_data}")
            time.sleep(0.5)
        print("✓ Encoder tests passed")
        return True
    except Exception as e:
        print(f"✗ Encoder tests failed: {e}")
        return False

def test_open_loop():
    """Test open-loop motor control"""
    print("\n=== Testing Open-Loop Control ===")
    try:
        # Low speed test
        print("Running at PWM 100 for 2 seconds...")
        Motor.RunMotorOpenLoop(100, True)
        time.sleep(2)
        
        # Read encoder while running
        Motor.GetEncoderPosition()
        
        # Stop
        print("Stopping motor...")
        Motor.StopMotor()
        time.sleep(1)
        
        print("✓ Open-loop tests passed")
        return True
    except Exception as e:
        print(f"✗ Open-loop tests failed: {e}")
        Motor.StopMotor()
        return False

def test_closed_loop():
    """Test closed-loop motor control"""
    print("\n=== Testing Closed-Loop Control ===")
    try:
        # Test at 1500 cps
        print("Running at 1500 cps for 3 seconds...")
        Motor.RunMotorClosedLoop(1500, True)
        
        # Monitor speed
        for i in range(3):
            time.sleep(1)
            Motor.GetEncoderPosition()
        
        # Change speed
        print("\nChanging to 2500 cps...")
        Motor.ResetIntegral()  # Reset integral when changing speed
        Motor.RunMotorClosedLoop(2500, True)
        
        time.sleep(2)
        Motor.GetEncoderPosition()
        
        # Stop
        print("\nStopping motor...")
        Motor.StopMotor()
        time.sleep(1)
        
        print("✓ Closed-loop tests passed")
        return True
    except Exception as e:
        print(f"✗ Closed-loop tests failed: {e}")
        Motor.StopMotor()
        return False

def test_update_rates():
    """Test setting update rates"""
    print("\n=== Testing Update Rates ===")
    try:
        print("Current rates:")
        rates = Motor.GetUpdateRates()
        print(f"  {rates}")
        
        print("\nSetting encoder rate to 20ms, controller rate to 50ms...")
        Motor.SetControllerGains(0.35, 0.1, 
                                encoder_update_rate_ms=20,
                                controller_update_rate_ms=50)
        
        rates = Motor.GetUpdateRates()
        print(f"  New rates: {rates}")
        
        print("✓ Update rate tests passed")
        return True
    except Exception as e:
        print(f"✗ Update rate tests failed: {e}")
        return False

def run_all_tests():
    """Run all motor tests"""
    print("="*60)
    print("ESP32 Motor Controller Test Suite")
    print("="*60)
    
    results = []
    
    # Connection test (required for all others)
    if not test_connection():
        print("\n✗ Cannot proceed without motor connection")
        return
    
    # Run tests
    results.append(("Gains", test_gains()))
    results.append(("Encoder", test_encoder()))
    results.append(("Update Rates", test_update_rates()))
    
    # Motor motion tests (ask for confirmation)
    print("\n" + "="*60)
    response = input("Run motor motion tests? (y/n): ").lower()
    if response == 'y':
        results.append(("Open-Loop", test_open_loop()))
        results.append(("Closed-Loop", test_closed_loop()))
    else:
        print("Skipping motor motion tests")
    
    # Summary
    print("\n" + "="*60)
    print("Test Summary:")
    print("="*60)
    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {name:20s} {status}")
    
    passed_count = sum(1 for _, p in results if p)
    total_count = len(results)
    print(f"\nPassed {passed_count}/{total_count} tests")
    
    # Cleanup
    print("\nDisconnecting motor...")
    Motor.DisconnectMotor()
    print("Done!")

if __name__ == '__main__':
    try:
        run_all_tests()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        Motor.StopMotor()
        Motor.DisconnectMotor()
    except Exception as e:
        print(f"\n\nUnexpected error: {e}")
        Motor.StopMotor()
        Motor.DisconnectMotor()
