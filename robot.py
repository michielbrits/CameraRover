#!/usr/bin/env python3
import paho.mqtt.client as mqtt
from time import sleep
import RPi.GPIO as GPIO
import pigpio
import json
import threading

pi = pigpio.pi()
GPIO.setmode(GPIO.BCM)

class Motor:
  def __init__(self, pin1, pin2):
    self.pin1 = pin1
    self.pin2 = pin2
  def Forward(self, speedfactor):
    pi.set_PWM_dutycycle(self.pin1, round(speedfactor))
    pi.set_PWM_dutycycle(self.pin2, 0)
  def Backward(self, speedfactor):
    pi.set_PWM_dutycycle(self.pin1, 0)
    pi.set_PWM_dutycycle(self.pin2, round(speedfactor))
  def Stop(self):
    pi.set_PWM_dutycycle(self.pin1, 0)
    pi.set_PWM_dutycycle(self.pin2, 0)
  def Brake(self):
    pi.set_PWM_dutycycle(self.pin1, 255)
    pi.set_PWM_dutycycle(self.pin2, 255)

class L293D:
  def __init__(self, motor1, motor2):
    self.motor1 = motor1
    self.motor2 = motor2
  def Drive(self, factorx, factory):
    if(factorx <= 0 and factorx > -0.5 and factory > 0):
      self.LeftForward(factorx*2)
    elif(factorx >= 0 and factorx < 0.5 and factory > 0):
      self.RightForward(factorx*2)
    elif(factorx <= 0 and factorx > -0.5 and factory < 0):
      self.LeftBackward(factorx*2)
    elif(factorx >= 0 and factorx < 0.5 and factory < 0):
      self.RightBackward(factorx*2)
    elif(factorx <= 0 and factory > 0):
      self.LeftForwardFast(factorx)
    elif(factorx >= 0 and factory > 0):
      self.RightForwardFast(factorx)
    elif(factorx <= 0 and factory < 0):
      self.LeftBackwardFast(factorx)
    elif(factorx >= 0 and factory < 0):
      self.RightBackwardFast(factorx)
    else:
      self.Stop()
  def Stop(self):
    self.motor1.Stop()
    self.motor2.Stop()
  def LeftForward(self, factorx):
    self.motor1.Forward(255)
    self.motor2.Forward(255*abs((-factorx)-1))
  def RightForward(self, factorx):
    self.motor1.Forward(255*abs(factorx-1))
    self.motor2.Forward(255)
  def LeftBackward(self, factorx):
    self.motor1.Backward(255)
    self.motor2.Backward(255*abs((-factorx)-1))
  def RightBackward(self, factorx):
    self.motor1.Backward(255*abs(factorx-1))
    self.motor2.Backward(255)
  def LeftForwardFast(self, factorx):
    self.motor1.Forward(255)
    self.motor2.Backward(255*(abs(factorx)-0.5)*2)
  def RightForwardFast(self, factorx):
    self.motor1.Backward(255*(factorx-0.5)*2)
    self.motor2.Forward(255)
  def LeftBackwardFast(self, factorx):
    self.motor1.Backward(255)
    self.motor2.Forward(255*(abs(factorx)-0.5)*2)
  def RightBackwardFast(self, factorx):
    self.motor1.Forward(255*(abs(factorx)-0.5)*2)
    self.motor2.Backward(255)

M1 = Motor(21, 16)
M2 = Motor(26, 13)
M3 = Motor(24, 23)
M4 = Motor(27, 17)

#just to make sure no gpio's have been left high
M1.Stop()
M2.Stop()
M3.Stop()
M4.Stop()

H1 = L293D(M1,M2)
H2 = L293D(M3,M4)

def is_json(myjson):
  try:
    json_object = json.loads(myjson)
  except ValueError as e:
    print("Not a valid json")
    return False
  return True

def processjson(message):
  jsonmessage = message.payload.decode("utf-8")
  if(is_json(jsonmessage)):
   j = json.loads(jsonmessage)
   yvalue = j['y']
   xvalue = j['x']
   H1.Drive(xvalue, yvalue)
   H2.Drive(xvalue, yvalue)

def breakh():
  print("nothing received anymore (1s). Executing EMERGENCY brake!")
  H1.Stop()
  H2.Stop()
t = threading.Timer(0.25, breakh)
def on_message(client, userdata, message):
  try:
    processjson(message)
    print(message.payload.decode("utf-8"))
    global t
    t.cancel()
    t = threading.Timer(0.25, breakh)
    t.start()
  except Error as e:
    print(e)

def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected MQTT disconnection. Will auto-reconnect")
broker_address="raspberrypi"
print("creating new instance")
client = mqtt.Client("control1") #create new instance
client.on_message=on_message #attach function to callback
client.on_disconnect = on_disconnect
print("connecting to broker")
client.connect(broker_address,port=1883) #connect to broker
print("Subscribing to topic","robots/CameraRover")
client.subscribe("robots/CameraRover", qos=1)
print("Publishing message to topic","robots/CameraRover")
client.loop_forever() #start the loop
#sleep(360000) # wait
