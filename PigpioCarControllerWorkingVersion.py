# -*- coding: utf-8 -*-
"""
Remote Car Controller using pigpio.

This class controls a DC motor (via PWM) using a Two-Input driver (like DRV8833)
and a steering servo on a remote Raspberry Pi.

NOTE: The pigpio daemon must be running on the Raspberry Pi:
      sudo systemctl start pigpiod
"""
import pigpio
import sys
import time

# Servo pulse widths (in microseconds)
SERVO_MIN_PULSE = 500
SERVO_MAX_PULSE = 2500
SERVO_CENTER = 1500

# PWM Frequency (1000Hz is often better for DC motor torque than the default 8000Hz)
MOTOR_PWM_FREQ = 1000

class CarController:
    """
    Controls the drive motor and steering servo via the pigpio daemon.
    
    Uses two pins (IN1, IN2) for motor control suitable for drivers like DRV8833.
    """
    # min_mod ensures the duty cycle is high enough to overcome motor inertia.
    def __init__(self, motor_in1_pin, motor_in2_pin, turn_pin, ip=None, min_mod=50, max_mod=255):
        """
        Initializes connection and sets up GPIO pins.

        Args:
            motor_in1_pin (int): GPIO pin connected to IN1 of the motor driver.
            motor_in2_pin (int): GPIO pin connected to IN2 of the motor driver.
            turn_pin (int): GPIO pin for servo pulse width.
            ip (str, optional): IP address of the remote Raspberry Pi. 
            min_mod (int): Minimum effective duty cycle for PWM (0-255).
            max_mod (int): Maximum duty cycle for PWM (0-255).
        """
        print(f"Attempting to connect to pigpio daemon at: {ip if ip else 'localhost'}")
        
        # Connect to remote or local Pi
        self.pi = pigpio.pi(ip)

        if not self.pi.connected:
            self.pi.stop() 
            raise ConnectionError(
                "Could not connect to pigpio daemon. "
                "Ensure pigpiod is running on the Pi and the IP is correct."
            )

        print("Connection successful.")
        
        # Renamed for clarity with DRV8833
        self.motor_in1_pin = motor_in1_pin
        self.motor_in2_pin = motor_in2_pin
        self.turn_pin = turn_pin
        
        self.duty_cycle = 0 # Current effective speed (positive value)
        self.min_mod = min_mod
        self.max_mod = max_mod
        
        # Set up GPIO pins as outputs
        self.pi.set_mode(self.motor_in1_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.motor_in2_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.turn_pin, pigpio.OUTPUT) 
        
        # --- NEW: Set PWM frequency for better motor torque ---
        self.pi.set_PWM_frequency(self.motor_in1_pin, MOTOR_PWM_FREQ)
        self.pi.set_PWM_frequency(self.motor_in2_pin, MOTOR_PWM_FREQ)
        # ---------------------------------------------------

        # Initialize servo to center position (pigpio defaults to 50Hz for servos)
        self.pi.set_servo_pulsewidth(self.turn_pin, SERVO_CENTER) 

    def turn(self, direction, angle=0):
        """
        Controls the steering servo.

        Args:
            direction (int): 1 for max right, -1 for max left, 0 for center.
        """
        if direction == 1:
            # Max Right (Servo max pulse)
            self.pi.set_servo_pulsewidth(self.turn_pin, SERVO_MAX_PULSE)
            print(f"Steering: Right ({SERVO_MAX_PULSE}us)")
        elif direction == -1:
            # Max Left (Servo min pulse)
            self.pi.set_servo_pulsewidth(self.turn_pin, SERVO_MIN_PULSE)
            print(f"Steering: Left ({SERVO_MIN_PULSE}us)")
        elif angle!= 0 :
            self.pi.set_servo_pulsewidth(self.turn_pin, angle)  
        else:
            # Center (Stop turning)
            self.pi.set_servo_pulsewidth(self.turn_pin, SERVO_CENTER)
            print(f"Steering: Center ({SERVO_CENTER}us)")

    def test_servo_pulse(self, pulse_width):
        """ Sets a custom pulse width to the servo for testing purposes. """
        # Ensure pulse is within a reasonable range
        pulse_width = max(0, min(3000, pulse_width))
        self.pi.set_servo_pulsewidth(self.turn_pin, pulse_width)
        print(f"Servo Pulse Test: Set to {pulse_width}us.")
    
    def test_servo_sweep(self):
        """ 
        Performs an automatic sweep across the servo's pulse range 
        to test its responsiveness.
        """
        STEP = 100  # Step size in microseconds
        DELAY = 0.1  # Delay in seconds
        
        print("\n--- Starting Servo Sweep Test ---")

        # Sweep forward (500us to 2500us)
        for pulse in range(SERVO_MIN_PULSE, SERVO_MAX_PULSE + 1, STEP):
            self.pi.set_servo_pulsewidth(self.turn_pin, pulse)
            print(f"Sweep: {pulse}us")
            time.sleep(DELAY)
            
        # Sweep backward (2500us to 500us)
        for pulse in range(SERVO_MAX_PULSE, SERVO_MIN_PULSE - 1, -STEP):
            self.pi.set_servo_pulsewidth(self.turn_pin, pulse)
            print(f"Sweep: {pulse}us")
            time.sleep(DELAY)

        # Return to center, keeping the signal active (restores "hum" if present)
        self.pi.set_servo_pulsewidth(self.turn_pin, SERVO_CENTER)
        print("--- Servo Sweep Test Complete, returned to center signal ---")


    def accelerate(self, duty_input):
        """
        Controls the motor speed and direction using Two-Input mode.

        Args:
            duty_input (int): Desired duty cycle (-255 to 255). Negative values 
                              indicate reverse.
        """
        # Determine raw duty cycle and direction
        duty = abs(duty_input)
        is_forward = duty_input >= 0
        
        # Always stop the motor first (low on both pins)
        self.pi.set_PWM_dutycycle(self.motor_in1_pin, 0)
        self.pi.set_PWM_dutycycle(self.motor_in2_pin, 0)

        # Apply bounds checking
        if duty != 0:
             duty = min(duty, self.max_mod)
             duty = max(duty, self.min_mod)
        
        self.duty_cycle = duty # Store the effective positive speed

        # --- Apply Two-Input Logic ---
        if self.duty_cycle > 0:
            if is_forward:
                # Forward: PWM on IN1, LOW on IN2
                self.pi.set_PWM_dutycycle(self.motor_in1_pin, self.duty_cycle)
                self.pi.write(self.motor_in2_pin, pigpio.LOW)
                print("Direction: Forward")
            else:
                # Reverse: LOW on IN1, PWM on IN2
                self.pi.write(self.motor_in1_pin, pigpio.LOW)
                self.pi.set_PWM_dutycycle(self.motor_in2_pin, self.duty_cycle)
                print("Direction: Reverse")
        else:
            # Stop condition (duty_cycle is 0)
            self.stop_motor_only()
            print("Direction: Neutral (Motor Stopped)")

        print(f"Drive Speed (PWM): {self.duty_cycle}")
    
    def stop_motor_only(self):
        """ Stops only the drive motor and ensures pins are set LOW for safety. """
        # Stop PWM first
        self.pi.set_PWM_dutycycle(self.motor_in1_pin, 0)
        self.pi.set_PWM_dutycycle(self.motor_in2_pin, 0)
        
        # Explicitly set the pins to LOW output to ensure a defined, non-PWM state
        self.pi.write(self.motor_in1_pin, pigpio.LOW)
        self.pi.write(self.motor_in2_pin, pigpio.LOW)
        
        self.duty_cycle = 0

    def stop(self):
        """
        Stops the motor and fully disables the servo signal.
        """
        self.stop_motor_only()
        # Disable the steering servo signal (pulse width 0) to stop the hum
        self.pi.set_servo_pulsewidth(self.turn_pin, 0) 
        print("Car Stopped and Servo Disabled.")

    def end_connection(self):
        """
        Stops the car, sets all control pins back to input mode (no drive), 
        and terminates the pigpio connection.
        """
        self.stop()
        
        # --- NEW: Explicitly return pins to INPUT mode to eliminate residual voltage ---
        self.pi.set_mode(self.motor_in1_pin, pigpio.INPUT)
        self.pi.set_mode(self.motor_in2_pin, pigpio.INPUT)
        self.pi.set_mode(self.turn_pin, pigpio.INPUT)
        # -----------------------------------------------------------------------------
        
        self.pi.stop()
        print("pigpio connection terminated.")


# --- Main Interactive Loop ---

# These pins were: DRIVE_PIN=17 and DIRECTION_PIN=27.
# We are now treating them as MOTOR_IN1 and MOTOR_IN2.
MOTOR_IN1_PIN = 17 
MOTOR_IN2_PIN = 27 
TURN_PIN = 21 

if __name__ == "__main__":
    
    ip = None
    try:
        ip_input = input("Enter Raspberry Pi IP address (leave blank for local connection): ").strip()
        if ip_input:
            ip = ip_input
    except EOFError:
        print("\nInput interrupted. Exiting.")
        sys.exit(0)

    try:
        # Pass the new motor pin structure to the controller
        car = CarController(MOTOR_IN1_PIN, MOTOR_IN2_PIN, TURN_PIN, ip)
    except ConnectionError as e:
        print(f"\nFATAL ERROR: {e}")
        sys.exit(1)
    
    # Simple command line interface
    while True:
        try:
            print("\n" + "="*30)
            print(f"Current Speed: {car.duty_cycle}")
            print("""
Available Commands:
1. Turn Steering
2. Accelerate / Reverse
3. Stop (Motor & Steering)
4. End Connection
5. Test Servo Pulse (Manual)
6. Test Servo Sweep (Auto)
""")
            command = int(input("Enter command number (1-6): ").strip())
        
        except ValueError:
            print("Invalid command. Please enter a number (1-6).")
            continue
        except EOFError:
            print("\nInput interrupted. Ending connection.")
            car.end_connection()
            break

        if command == 1:
            try:
                print("\nSteering Options:")
                print("1. Turn Right (Max)")
                print("2. Turn Left (Max)")
                print("3. Center Steering (Stop)")
                specific = int(input("Which direction? ").strip())
                
                if specific == 1:
                    car.turn(1)
                elif specific == 2:
                    car.turn(-1)
                elif specific == 3:
                    car.turn(0)
                else:
                    print("Invalid steering option.")
            except ValueError:
                print("Invalid input. Please enter 1, 2, or 3.")

        elif command == 2:
            try:
                # User can enter positive (forward) or negative (reverse) speed
                specific = int(input(f"How fast? (0 to {car.max_mod} for FWD, 0 to -{car.max_mod} for REV): ").strip())
                car.accelerate(specific)
            except ValueError:
                print("Invalid speed. Please enter an integer number.")

        elif command == 3:
            car.stop()

        elif command == 4:
            confirm = input("Are you sure you want to end connection? (Y/n) ").strip()
            if confirm.upper() == "Y" or not confirm:
                car.end_connection()
                print("Exiting application.")
                break
        
        elif command == 5:
            try:
                print("\nServo Test:")
                pulse = int(input("Enter pulse width in microseconds (e.g., 1500 for center, 1000 or 2000 for sides): ").strip())
                car.test_servo_pulse(pulse)
            except ValueError:
                print("Invalid pulse width. Please enter an integer.")

        elif command == 6:
            car.test_servo_sweep()
        
        else:
            print("Unknown command. Please enter a number between 1 and 6.")

    sys.exit(0)
