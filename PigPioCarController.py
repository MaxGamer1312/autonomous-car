# -*- coding: utf-8 -*-
"""
Spyder Editor

Class to control raspberry pi car.
"""
import pigpio
SERVO_CENTER=1500

class CarController:
    def __init__(self, drive_pin, direction_pin, turn_pin, ip= None, duty_cycle=0, min_mod=0, max_mod=255):
        #if isinstance(ip, str):
        self.pi = pigpio.pi(ip)  # Connect to remote Pi
        #else:
#            self.pi = pigpio.pi()    # Connect locally

        if not self.pi.connected:
            raise ConnectionError("Could not connect to pigpio daemon.")

        self.drive_pin = drive_pin
        self.direction_pin = direction_pin
        self.turn_pin = turn_pin
        
        # This might be a hardware thing, dunno if different compotentsare used
        self.duty_cycle = duty_cycle
        self.min_mod = min_mod
        self.max_mod = max_mod
        
        # Set up GPIO pins
        self.pi.set_mode(self.drive_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.turn_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.direction_pin, pigpio.OUTPUT) # *** FIX 1: Must set direction pin mode ***
       
        
        # Initialize servo to center position
        self.pi.set_servo_pulsewidth(self.turn_pin, SERVO_CENTER) 

    #Might change to take a speed variable.
    def turn(self, direction):
        if direction ==  1:
            self.pi.set_servo_pulsewidth(self.turn_pin, 2500)
        elif direction ==  -1 :
            
            self.pi.set_servo_pulsewidth(self.turn_pin, 500)
        else:
            # Stop turning
            self.pi.set_servo_pulsewidth(self.turn_pin, SERVO_CENTER)

    def accelerate(self, duty=None):
        if duty is not None:
            self.pi.write(self.direction_pin, pigpio.HIGH) 
            if duty < 0 :
                duty = abs(duty)
                self.pi.write(self.direction_pin, pigpio.LOW) 
            # Makes sure it's lower than max
            duty=min(duty,self.max_mod)
            # Makes sure it's higher than min
            self.duty_cycle=max(duty,self.min_mod)
            
        
        self.pi.set_PWM_dutycycle(self.drive_pin, self.duty_cycle)
    

    def stop(self):
         # set speed to 0
         self.pi.set_PWM_dutycycle(self.drive_pin, 0)
         # sets the servo to center 
         self.pi.set_servo_pulsewidth(self.turn_pin, SERVO_CENTER) 

         self.pi.write(self.direction_pin,pigpio.LOW)
         self.duty_cycle = 0 # reset stored speed
    def end_connection(self):
        self.stop(self)
        self.pi.stop()
        #11 13 16 18 all seem to connect to the motor. Unsure which is direction, and which is drive.
drive_pin=11 # pin for the motor contolling wheels
direction_pin=13 # pin for the direction of the motor. not sure if needed.
turn_pin=40 # pin for the servo

command="" 
#if input("ip?")[0]== "Y": 
ip = input("Enter ip here : ")
car= CarController(drive_pin, direction_pin, turn_pin,ip)
#else :
 #   car= CarController(drive_pin, direction_pin, turn_pin)
while True :
    print("""Current commands : 
          1. turn
          2. accelerate
          3. stop 
          4. end
          """)
    command = int(input("Next command"))
    if (command==1):
        print("""
              1. clockwise
              2. counter clockwise
              """)
              
        specfic = int(input("Which direction?"))
        if specfic == 1 :
            car.turn(1)
        else :
            car.turn(-1)
    elif (command == 2): 
        specfic = int(input("How fast?"))
        if (specfic < 0):
            car.accelerate()    
        else :
            car.accelerate(specfic)
    elif (command == 3) :
        car.stop()
    else:
        command = input("End connection?")
        if command[0].upper()== "Y":
            car.end_connection()
            print("Connection terminated")
        
        


    
    
