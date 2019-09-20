import RPi.GPIO as GPIO
from gpiozero import AngularServo,DistanceSensor,LED,Buzzer
import time,os                        
GPIO.setmode(GPIO.BCM)                
   
GPIO.setwarnings(False)

TRIG = 14                                
ECHO = 15                                

print ("Distance measurement in progress")
GPIO.setup(17,GPIO.OUT)  # Sets up pin 11 to an output (instead of an input)
p = GPIO.PWM(17, 50)     # Sets up pin 11 as a PWM pin
p.start(0)               # Starts running PWM on the pin and sets it to 0


GPIO.setup(TRIG,GPIO.OUT)                 
GPIO.setup(ECHO,GPIO.IN)
GPIO.setup(4,GPIO.IN)
buzzer=Buzzer(2)
#Servo_motor=AngularServo(17,min_angle=0,max_angle=90)
led_red=LED(27)
led_amber=LED(22)
led_green=LED(10)
try:
    while True:
      sensor=GPIO.input(4)
      GPIO.output(TRIG, False)                
      print ("Waitng For Sensor To Settle")
      time.sleep(0.1)                            

      GPIO.output(TRIG, True)                 
      time.sleep(0.00001)                     
      GPIO.output(TRIG, False)                 

      while GPIO.input(ECHO)==0:               
        pulse_start = time.time()
      while GPIO.input(ECHO)==1:               
        pulse_end = time.time()                

      pulse_duration = pulse_end - pulse_start 
      distance = pulse_duration * 17150        
      distance = round(distance, 2)            

      if distance <=5 :
        buzzer.beep(0.2,0.2,1)
        led_green.on()
        led_amber.off()
        led_red.off()
        print ("Distance:",distance ,"cm"  )
        os.system('/home/pi/Desktop/dev/pushbullet.sh "Alert dustbin is full"')
      elif distance ==20:
          led_green.off()
          led_red.off()
          led_amber.on()
          print("halve")
      elif distance ==40 :
          led_red.on()
          led_amber.off()
          led_green.off()
          print("empty")
          
      
         

      elif sensor==0:
        print("motion detected")
        led_red.on()
        led_amber.off()
#        Servo_motor.angle=0
        p.ChangeDutyCycle(1)
        
        time.sleep(12)
        #GPIO.output(2,GPIO.HIGH)
        
      elif sensor==1:
        print("no motion detected")
        led_amber.on()
        led_red.off()
        #GPIO.output(2,GPIO.LOW)
#        Servo_motor.angle=90
        p.ChangeDutyCycle(13)
           
                            
            
      else:
          led_red.off()
          led_amber.off()
          led_green.off()
          
          print ("Out Of Range")                  

       
except:
    GPIO.cleanup()
