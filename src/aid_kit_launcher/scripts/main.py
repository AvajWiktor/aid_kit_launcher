#!/usr/bin/python3
import rospy
#import RPi.GPIO as GPIO
from time import sleep
import serial
#from std_msgs.msg import Bool
#from std_srvs.srv import SetBoolRequest,SetBoolResponse,SetBool
from aid_kit_launcher.srv import Launcher
"""
#Servo works in range 2.5-12.5 duty which means 0-180 degrees

global duty #Pwm duty cycle
global pwm #Pwm instance
duty=2.5 #Set servo 0 position duty value

GPIO.setmode(GPIO.BOARD) #This mode let us using board pinout numerate
GPIO.setwarnings(False) #XD 
print("GPIO set mode") 
GPIO.setup(33, GPIO.OUT) #Seting up pwm pin as gpio out
pwm = GPIO.PWM(33, 50) #Creating pwm instance
pwm.start(duty) #Start pwm with 0 position value 

#Service responsible for handling whether servo is open or closed
#True - Open, False - Closed
def move_servo(req):
	global duty
	global pwm
	try:
		if req.data:
			duty = 12.0		
		else:
			duty = 2.5
		pwm.ChangeDutyCycle(duty) #Set new Pwm duty cycle
		
		return True, "Successful operation"
	except ValueError:
		return False, ValueError
	"""
global ser
ser = serial.Serial(port='/tmp/printer', baudrate=256000)

def execute(req):
	global ser
	if req.command == 1:
		print("deploy aidkit\n")
		if ser.isOpen():
			ser.close()
		ser.open()
		ser.write(b'deploy\r\n')
		print(ser.read())
		ser.close()
		return True
	else:
		print("Rotate camera\n")
		#rotate camera by req.command angle
		return True

rospy.init_node('launcher_service', anonymous=True) #Initialize ros node
s = rospy.Service('send_command',Launcher ,execute) #Create rosservice instance with callback
print ("Servo Service is ready")
rospy.spin() #Wait for ctrl+C
