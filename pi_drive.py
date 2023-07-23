from lane_methods import mask_img, detect_edges, crop_roi, \
                        detect_lines, draw_lines, lane_search_area, \
                        group_lines, line_to_point, steering_angle
import cv2
import time
import RPi.GPIO as GPIO
import pigpio
import time
from time import sleep



# motor driver pins
Ena= 26 
In1= 6
In2= 5

# servo ppin
servo = 12


# GPIO SETUP
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(Ena,GPIO.OUT)
GPIO.setup(In1,GPIO.OUT)
GPIO.setup(In2,GPIO.OUT)

# speed control with pwmA
pwmA =GPIO.PWM(Ena,100)
pwmA.start(0)

# servo setup
pwm = pigpio.pi() 
pwm.set_mode(servo, pigpio.OUTPUT)
pwm.set_PWM_frequency( servo, 50 )



# range for steering input
axis_start = 50
axis_end = -50
axis_range = axis_end - axis_start

# range for throttle
throttle_start = 0
throttle_end = 100
throttle_range = throttle_end - throttle_start

# range for car servo
steering_start = 900
steering_end = 2100
steering_range = steering_end - steering_start


# list to keep last 'n' outputs. Idea is to average all of them and then produce a final output,
#  this helps in smoothing out any jerks and sudden changes in servo and speed
steering_last_five = []
throttle_last_three = []

# for caliberating steering of car 
steering_factor = 1

# starting throttle value, will be 0
th = 0

def main():
    cam = cv2.VideoCapture(0)

    while True:

        ret, frame = cam.read()
        frame = cv2.resize(frame, (640, 480))
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # for debug, sh = True
        sh = False

        # Masking
        frame_masked = mask_img(frame_hsv, show=sh)
        

        # Extract Edges
        frame_edges = detect_edges(frame_masked, show=sh)


        # Crop top half, Only lower half is region of interest
        frame_cropped = crop_roi(frame_edges, show=sh)


        # Detect line segments
        frame_lines = detect_lines(frame_cropped)
        #draw_lines(frame, frame_lines)


        # Group lines into left and right catgry
        frame_lines_grouped = group_lines(frame, frame_lines)
        
        #this is to view where is search area for lanes, helps in debugging
        #lane_search_area(frame, 0.7) 

        # Draw Lines
        draw_lines(frame, frame_lines_grouped)


        steerin_input_raw = steering_angle(frame, frame_lines_grouped, show=True)
        print("Steering Input Raw: ",int(steerin_input_raw))
       
        
        # caliberating with some factor
        current_axis = steerin_input_raw * steering_factor
        print("Steering Input Raw Corrected : ",int(current_axis))

        # mapping steering output to servo angle
        steering_input = steering_start + ((steering_range) / (axis_range)) * (current_axis - axis_start)
        print("Steering Input Final: ",int(steering_input), '\n\n')

        st = int(steering_input)

        # appending it to the list for averaging with past 5 values
        steering_last_five.append(st)
        if len(steering_last_five) > 5:
            steering_last_five.pop(0)
        st = int(sum(steering_last_five) / len(steering_last_five))

        
    
        
        # if only one lane line detected, means car need to take hard right or left    
        # also steering_angle() returns slope of the one lane if there is only one lane.
        # this slope range is different from when two lanes are deected.
        # So we define new range that is here 70 to 20. and map servo angle accordingly 
        if(len(frame_lines_grouped) == 1):
            print("ONE LANE SLOPE : ", steerin_input_raw)
            axs = 70
            axe = 20
            axr = axe - axs
            st = steering_start + ((steering_range) / (axr)) * (steerin_input_raw - axs)
            print("ONE LANE STEER :", st)
            
            # also we have to slow down the car as there is steep turn when there is only one lane marking
            th = th - 2
            throttle_input = th 
            print("Throttle Down : ", throttle_input)
        
        else:
            # if two name marking visible, means road is mostly straingt,
            # so we can increase throttle input
            th = th + 3
            throttle_input = th
            print("Throttle Up : ", throttle_input)

        # making sure angle remain in range of what wervo can handel
        if st < 900:
            st = 900
        if st > 2100:
            st = 2100 

        # assigning the final values     
        pwm.set_servo_pulsewidth( servo, st ) ;
        pwmA.ChangeDutyCycle(int(throttle_input));

        


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    pwmA.ChangeDutyCycle(int(0));
    cam.release()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main()



