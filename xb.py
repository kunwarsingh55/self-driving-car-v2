
# Script for debugging and testing car manually

import RPi.GPIO as GPIO
import pigpio
import time
from time import sleep
import pygame

pygame.init()
controller = pygame.joystick.Joystick(0)
controller.init()

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

servo = 12
Ena= 26
In1= 6
In2= 5


GPIO.setup(Ena,GPIO.OUT)
GPIO.setup(In1,GPIO.OUT)
GPIO.setup(In2,GPIO.OUT)
pwmA =GPIO.PWM(Ena,100);
pwmA.start(0);

pwm = pigpio.pi() 
pwm.set_mode(servo, pigpio.OUTPUT)
pwm.set_PWM_frequency( servo, 50 )

axis_start = -0.99
axis_end = 0.99
axis_range = axis_end - axis_start

throttle_start = 0
throttle_end = 100
throttle_range = throttle_end - throttle_start
# output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)

steering_start = 1000
steering_end = 2000
steering_range = steering_end - steering_start


def getJS(name=''):
 
    global buttons
    # retrieve any events ...
    for event in pygame.event.get():                                # Analog Sticks
        if event.type == pygame.JOYAXISMOTION:
            #print("AXIS", event.axis , "Value = " , event.value)
            
            # throttle
            # axis range = -1 to 1
            # car throttle range = 0 to 100
            if event.axis == 5:
                current_axis = event.value
                throttle_input = throttle_start + ((throttle_range) / (axis_range)) * (current_axis - axis_start)
                print("Throttle Input : ",int(throttle_input))

                pwmA.ChangeDutyCycle(int(throttle_input));
                GPIO.output(In1,GPIO.LOW);
                GPIO.output(In2,GPIO.HIGH);
                
            if event.axis == 2:
                current_axis = event.value
                throttle_input = throttle_start + ((throttle_range) / (axis_range)) * (current_axis - axis_start)
                print("Reverse Throttle Input : ",int(throttle_input))
                pwmA.ChangeDutyCycle(int(throttle_input));
                GPIO.output(In1,GPIO.HIGH);
                GPIO.output(In2,GPIO.LOW);
                
            if event.axis == 0:
                current_axis = event.value
                steering_input = steering_start + ((steering_range) / (axis_range)) * (current_axis - axis_start)
                print("Steering Input : ",int(steering_input))
                pwm.set_servo_pulsewidth( servo, steering_input ) ;
                
            
                        
    
        elif event.type == pygame.JOYBUTTONDOWN:                    # When button pressed
            print(event.dict, event.joy, event.button, 'PRESSED')
            pwmA.ChangeDutyCycle(27);
            GPIO.output(In1,GPIO.LOW);
            GPIO.output(In2,GPIO.HIGH);
            
            
            
        elif event.type == pygame.JOYBUTTONUP:                      # When button released
            print(event.dict, event.joy, event.button, 'released')
            
            
                    
 
    
def main():
    #print(getJS()) # To get all values
    #sleep(0.05)
    getJS('share') # To get a single value
    sleep(0.05)
 
 
if __name__ == '__main__':
  while True:
    main()


