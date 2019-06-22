## Calvin Barajas
## Dynamixel Robot Arm With OpenCV Vision 2019 rev 001
## Sacramento, California
## SacRobotics.com
## Project Description: We begin using computer vision
## with this robotic arm. OpenCV is implemented along with
## the instruction packets to control the servos.
## Thank You: pyimagesearch.com, electrondust.com.

# general libraries
import RPi.GPIO as GPIO
from time import sleep
import serial
# vision libraries
import cv2
import numpy as np
from picamera import PiCamera
import imutils
from imutils import contours

# Packet Format: [HDR, HDR, ID, LENGTH, WRITE_INST, GOAL_POS(P1),
#  LOW_BYTE(P2), HIGH_BYTE(P3), SPEED1(P4), SPEED2(P5), CHKSUM]
HDR  = 'FF' # Headers are always 0xFF (255).
ID1 = '01' # Servo ID in hex (0x01).
ID2 = '02' 
ID3 = '03'
ID4 = '04'
LENGTH  = '07' # All instructions use 5 parameters + 2 (default) = 0x07.
WRITE_INST = '03' # Write to the servo's register table.
GOAL_POS = '1E' # Obtain this from control table.
RELAY = 17 # For controlling 5v mechanical relay and magnetic end effector.
CHANNEL = 18 # HIGH and LOW signal determines who will TX and who will RX.
area = 0 # Keep track of h x width of image (in pixels). Small or Large image.
GPIO.setmode(GPIO.BCM) # Use BCM naming convention.
GPIO.setup(CHANNEL, GPIO.OUT) # Used for HIGH/LOW signal transmission (TX/RX).
GPIO.setup(RELAY, GPIO.OUT) # HIGH to turn off relay, LOW to turn on relay.
ser = serial.Serial("/dev/ttyS0", baudrate=1000000, timeout=3.0)

# ==========================================================================
# ==== RELAY ON WHEN OBJECT IS "GRABBED" AND RELAY OFF TO RELEASE OBJECT ===
# ==========================================================================

def relay_on(relay):
  GPIO.output(relay, GPIO.LOW)
  print('=== RELAY ON===')
  sleep(1)
    
def relay_off(relay):
  GPIO.output(relay, GPIO.HIGH)
  print('=== RELAY OFF===')
  sleep(1)

relay_off(RELAY) # Turn off in case program is terminated while it's on.

# ==========================================================================
# ========================== COMPUTER VISION CODE ==========================
# pyimagesearch.com/2016/03/21/ordering-coordinates-clockwise-with-python-and-opencv/
# ==========================================================================

def order_points(pts):
  # Find the 4 corners of a bounding box around the object and sort them
  # in a clockwise orientation.
  rect = np.zeros((4, 2), dtype="float32") # Create a 4x2 array and fill with 0s.
  s = pts.sum(axis=1) # Add the elements row-by-row.
  rect[0] = pts[np.argmin(s)] # TL = top left corner.
  rect[2] = pts[np.argmax(s)] # BR = bottom right corner.
  diff = np.diff(pts, axis=1) # Subtract the elements row-by-row.
  rect[1] = pts[np.argmin(diff)] # TR = top right.
  rect[3] = pts[np.argmax(diff)] # BL = bottom left.
  return rect # Return array of organized coordinates now.

def take_photo():
  camera = PiCamera() # Open port to camera.
  camera.resolution = (300, 300) # Take small photo.
  camera.zoom = (0.25, 0.25, 0.5, 0.5) # Zoom in to avoid photographing platform edges.
  camera.start_preview()
  sleep(2) # 2 seconds is minimum delay for camera to prepare.
  camera.capture('/home/pi/Desktop/img/image3.jpg') # Take photo.
  camera.stop_preview()
  camera.close() # Close open connection to camera port.
  image = cv2.imread('/home/pi/Desktop/img/image3.jpg') # Load image for analysis.
  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # Convert image to grayscale.
  edged = cv2.Canny(gray, 25, 100) # Apply edge detector with threshold (converts to B&W).
  edged = cv2.dilate(edged, (5,5), iterations=5) # Pixel expansion.
  cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  cnts = imutils.grab_contours(cnts)
  (cnts, _) = contours.sort_contours(cnts)
  colors = ((0,0,255),(240,0,159),(255,0,0),(255,255,0)) # Bounding box line colors.

  for (i, c) in enumerate(cnts):
    if cv2.contourArea(c) < 100:
      continue
    global area # Use this variable to determine if object is large or small.
    area += cv2.contourArea(c)
    print('contour area: ', cv2.contourArea(c))
    print('area (inside): ', area)
    box = cv2.minAreaRect(c)
    box = cv2.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
    box = np.array(box, dtype="int")
    cv2.drawContours(image, [box], -1, (0, 255, 0), 2)
    rect = order_points(box)
    for ((x, y), color) in zip(rect, colors):
      cv2.circle(image, (int(x), int(y)), 5, color, -1)
    cv2.putText(image, "Object #{}".format(i + 1), (int(rect[0][0] - 15), int(rect[0][1] - 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    cv2.imshow("window", image)
    cv2.waitKey(3000)
    cv2.destroyAllWindows()

# ==========================================================================
# ========================== INSTRUCTION PACKETS ===========================
# ==========================================================================

def arm_neutral():
  # ========= THE MOST LOAD BALANCED POSITION ================
  # For load balancing the arm as it turns at the waist/shoulder.
  # If this is not specified, the arm wobbles to much when it's extended.
  
  # SERVO 01:         ELBOW - LEAN WAY BACK
  # GOAL POSITION:    300[0x12C]
  # SPEED:            100[0x064]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 01 07 03 1E 2C 01 64 00 45"))
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)

  # SERVO 02:         WRIST - ALMOST PERPENDICULAR TO ELBOW
  # GOAL POSITION:    400[0x190]
  # SPEED:            100[0x064]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 02 07 03 1E 90 01 64 00 E0"))
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)
  
  # SERVO 03:         FINGER - MOVE UP (CLOSER TO CANDY DISH)
  # GOAL POSITION:    075[0x04B]
  # SPEED:            100[0x064]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 03 07 03 1E 4B 00 64 00 25"))
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)

def arm_photo():
  # ============ POSITION THE ARM TO TAKE PHOTOGRAPH ================

  arm_neutral() # Place arm in starting (neutral) position.

  # https://github.com/T-Kuhn/ScrewPicker/blob/master/ax12/ax12.py
  # =================================================================
  # Sample code below uses bitwise operators. This code allows you to
  # set GOAL_POSITION and MOVING_SPEED much more conveniently. That way,
  # you don't have to calculate hex values and checksum manually in
  # the instruction packet.
  # =================================================================
  # -----SERVO 04: SHOULDER - MOVE BETWEEN CANDY BOWLS (MIDDLE)------
  GOAL_POSITION = 650 # You can change goal position "on the fly."
  MOVING_SPEED = 80 # You can change moving speed "on the fly" also.
  
  # Create 2 element array with HIGH and LOW bytes.
  gp = [GOAL_POSITION&0xFF,GOAL_POSITION>>8] # gp = goal position.
  # Create 2 element array with HIGH and LOW bytes.
  ms = [MOVING_SPEED&0xFF,MOVING_SPEED>>8] # ms = moving speed.
  # Create an array that matches the instruction packet order.
  cs = [ID4, LENGTH, WRITE_INST, GOAL_POS, hex(gp[0])[2:].zfill(2), #cs = checksum.
   hex(gp[1])[2:].zfill(2), hex(ms[0])[2:].zfill(2), hex(ms[1])[2:].zfill(2)]
  # Add all the bytes and perform bitwise operation to get actual checksum value.
  chksum = (hex(~(int(cs[0],16)+int(cs[1],16)+int(cs[2],16)+int(cs[3],16)+
   int(cs[4],16)+int(cs[5],16)+int(cs[6],16)+int(cs[7],16))&0xFF))[2:]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("{} {} {} {} {} {} {} {} {} {} {}".
   format(HDR,HDR,cs[0],cs[1],cs[2],cs[3],cs[4],cs[5],cs[6],cs[7],chksum)))
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)

  # =======================================================================
  # Below is the "original" code that was used to control the servos. It's
  # very basic and the problem with this code is that calculating hex values
  # for speed, position, and checksum have to be done manually (time consuming).
  # You can replicate above code to all the instruction packets below (just
  # make sure you update the ID).
  # =======================================================================
  
  # SERVO 02:         WRIST - PERPENDICULAR TO ELBOW
  # GOAL POSITION:    500[0x1F4]
  # SPEED:            150[0x096]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 02 07 03 1E F4 01 96 00 4A"))
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)

  # SERVO 01:         ELBOW - 90 DEGREES (STRAIGHT UP)
  # GOAL POSITION:    500[0x1F4]
  # SPEED:            150[0x096]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 01 07 03 1E F4 01 96 00 4B"))
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)

  # SERVO 03:         FINGER - MOVE DOWN TO MEET TABLE
  # GOAL POSITION:    180[0x0B4]
  # SPEED:            150[0x096]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 03 07 03 1E B4 00 96 00 8A"))
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)

def arm_pickup():
  # ============= PICKUP OBJECT OFF THE TABLE ==================

  arm_neutral() # Place arm in starting (neutral) position

  # SERVO 04:         SHOULDER - MOVE SLIGHTLY TO THE RIGHT
  # GOAL POSITION:    545[0x221]
  # SPEED:            150[0x096]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 04 07 03 1E 21 02 96 00 1A")) 
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)

  # SERVO 03:         FINGER - MOVE DOWN TO MEET TABLE
  # GOAL POSITION:    180[0x0B4]
  # SPEED:            150[0x096]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 03 07 03 1E B4 00 96 00 8A"))
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)

  # SERVO 02:         WRIST - MOVE UP SLIGHTLY
  # GOAL POSITION:    440[0x1B8]
  # SPEED:            150[0x096]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 02 07 03 1E B8 01 96 00 86"))
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)
  
  # SERVO 01:         ELBOW - LEAN FORWARD SLIGHTLY
  # GOAL POSITION:    525[0x20D]
  # SPEED:            150[0x096]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 01 07 03 1E 0D 02 96 00 31"))
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)

  relay_on(RELAY) # Turn 5v relay on.

  # SERVO 04:         SHOULDER - SWEEP BACK AROUND TO TOUCH OBJECT
  # GOAL POSITION:    620[0x26C]
  # SPEED:            030[0x01E]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 04 07 03 1E 6C 02 1E 00 47")) 
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)

  arm_neutral() # Place arm in starting (neutral) position.

def arm_bidirectional():
  # ===== SAME MOVEMENT WHETHER ARM MOVES TOWARDS "LARGE" BOWL OR "SMALL" BOWL =====
  
  # SERVO 02:         WRIST - PERPENDICULAR TO ELBOW
  # GOAL POSITION:    500[0x1F4]
  # SPEED:            100[0x064]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 02 07 03 1E F4 01 64 00 7C"))
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)

  # SERVO 03:         FINGER - MOVE UP (CLOSER TO CANDY DISH)
  # GOAL POSITION:    140[0x08C]
  # SPEED:            150[0x096]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 03 07 03 1E 8C 00 96 00 B2"))
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)

  # SERVO 01:         ELBOW - 90 DEGREES (STRAIGHT UP)
  # GOAL POSITION:    400[0x190]
  # SPEED:            100[0x064]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 01 07 03 1E 90 01 64 00 E1"))
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)

  relay_off(RELAY) # release object

def arm_large():
  # ============= PLACE OBJECT IN LARGE CANDY BOWL ==================

  # SERVO 04:         SHOULDER - GO TO LARGE CANDY BOWL
  # GOAL POSITION:    345[0x159]
  # SPEED:            150[0x096]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 04 07 03 1E 59 01 96 00 E3")) 
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)

  arm_bidirectional()

def arm_small():
  # ============= PLACE OBJECT IN LARGE CANDY BOWL ==================

  # SERVO 04:         SHOULDER - GO TO LARGE CANDY BOWL
  # GOAL POSITION:    925[0x39D]
  # SPEED:            150[0x096]
  GPIO.output(CHANNEL, GPIO.HIGH)
  ser.write(bytearray.fromhex("FF FF 04 07 03 1E 9D 03 96 00 9D")) 
  sleep(0.1)
  GPIO.output(CHANNEL, GPIO.LOW)
  sleep(1)

  arm_bidirectional()

for x in range(3):
  arm_photo() # Move the arm and get it ready to take a photo.
  take_photo() # Take a photo of the object on platform.
  arm_pickup() # Pickup object off the platform.
  if (area > 10000): # If the h x width of the pixels is larger than 10,000 pixels.
    print('area large: ', area)
    arm_large() # Place object in large bowl.
  else:
    print('area small: ', area)
    arm_small()
  area = 0 # Reset global variable for calculating object size (large or small).

ser.close()
GPIO.cleanup()



