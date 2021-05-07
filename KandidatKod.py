from pupil_apriltags import Detector
import sys
sys.path.append('/usr/local/lib/python3.7/dist-packages')
import matplotlib.pyplot as plt
import cv2
import numpy as np
from time import sleep
import yaml
import sys
sys.path.append('/usr/lib/python3/dist-packages')
import serial
import time

def length(length):
    """ Calculates the amount of frames between two corners in x-axle, if the apriltag is bend it takes the corners between x, y which has most frames"""
    length_x = length[:, 0]
    length_y = length[:, 1]
    lengths = [length_x, length_y]
    size = []
    for length in lengths:
        length = np.sort(length)[::-1]      # Sorts them with largest first
        length = length[0] - length[2]      # Takes the first and the third because they are beside each other in x-led
        size.append(length)
    if size[0] >= size[1]:
        return size[0]
    return size[1]


def func_2(x):                      # funktion för avståndsmätning
    #return 9.48485461 * np.exp(-0.02868767*x)+0.90513495 # värden för 1280x740
    return 9.472138612 * np.exp(-0.01217216*x)+0.7222774


def func_2_to_middle_line(x):       # funktion för vinkelmätning
    #return 258.800987 * np.exp(-1.43926458*x)+ 18.26819076
    return 494.37127535 * np.exp(-1.22146463*x)+48.78180416

at_detector = Detector(
	families='tag36h11',
	nthreads=1,
	quad_decimate=1.0,
	quad_sigma=0.0,
	refine_edges=1,
	decode_sharpening=0.25,
	debug=0)


""" Fix so the frames on the camera is constant"""
video = cv2.VideoCapture(0)
codec=cv2.VideoWriter_fourcc('M', 'J', 'P', 'G' )
video.set(6,codec)
video.set(5, 30)
video.set(3, 1920)
video.set(4,1080)

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.flush()

while(True):
    # Capture frame-by-frame
    ret, frame = video.read()

    # Change the frame to gray scale
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # See what the height and width of the frame is
    height_of_image, width_of_image = img.shape  # width x and height y
    #print(img.shape)
    # text = [881, 0, 640, 0, 496, 360, 0, 0, 1]     # camera parameters 1280x740
    text = [1321, 0, 960, 0, 743, 540, 0, 0, 1]
    cameraMatrix = np.array(text).reshape((3, 3))
    
    camera_params = (
        cameraMatrix[0, 0], # f_x: focal length = brännvidd (ofta som "synvinkel" = 72 grader), här anges det i pixlar
        cameraMatrix[1, 1], # f_y: focal length = A (=half image size) / tan(synvinkel/2)
        cameraMatrix[0, 2], # c_x: focal center in pixels, center of image
        cameraMatrix[1, 2], # c_y: focal center
    )

    # Detect the apriltags
    frame = at_detector.detect(img, True, camera_params, 0.19) 
    color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    length_from_camera=0
    for tag in frame:
        #print('corners:',tag.corners)      # användes för att mäta på olika avstånd för att få fram funktionerna
        #print('center:',tag.center)

        """ Draw green box around found tags"""
        for idx in range(len(tag.corners)):
            cv2.line(color_img, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)),(0, 255, 0),)

        """ Write tag ID next to found tags"""
        cv2.putText(
            color_img,
            str(tag.tag_id),
            org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10, ),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.8,
            color=(0, 0, 255),
        )

        """ Takes the corners and calculates the distance in frames between 2 of them """
        corners = tag.corners
        length_corners = length(corners)
        length_from_camera = func_2(length_corners)                     # avståndet som skickas till arduinon
        #print(length_from_camera)   

        """ Distance from middle line """
      #  # Middle line is at frame 1280/2 because it is predetermined earlier in the code
        check_16 = 16                                                   # avståndet som vi förflyttat bilden åt vänster vid uppmätning vid olika avstånd
        frame_for_16cm = func_2_to_middle_line(length_from_camera)      # funktion som räknar avstånd till mitten
        center_x = tag.center[0]
        spot = ((center_x-960)/frame_for_16cm)*check_16/100             # meter from middle

        """ Angle to to middle """
        angle = np.sin(spot/length_from_camera)                           # räknar ut vinkeln
        #print(np.degrees(angle))

        str1 =str(10*length_from_camera)[0:2]
        str2 =str(100*(angle + np.pi/3))[0:5]

        int1 = int(10*(length_from_camera))
        int2 = int(100*(angle))
        #print(int1)
        
        Bstring = [str(int1) , str(int2)] # + "\n"
        string = ",".join(Bstring) + '\n'
        #string1= str(3) + str(1) + "\n"
        #print(string)
        str5 = bytes(str1,'utf-8')

    #print(str5)
        #LengthToCameraBytesArray =(20).to_bytes(8, 'big')
        #print(LengthToCameraBytesArray)
        #AngleToCameraBytesArray =int2.to_bytes(8, 'big')
        #print(int1)
        ser.write(bytes([int1]))
        #ser.write(0xff)
        ser.write(bytes([int2]))
        #ser.write(0xfe)
        
        #ser.write(string.encode('utf-8'))
        #ser.write(str1.encode('utf-8'))            char david[4];   
        #ser.write(str2.encode('utf-8'))
        #line=ser.readline().decode('utf-8','replace').rstrip()
       
        #line=ser.readline().decode('utf-8').rstrip()
       # print(line)
        #time.sleep(0)