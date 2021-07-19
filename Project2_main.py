'''
Script to detect tag pose(camera frame), transfrom pose to world frame, compute IK,
and send IK results to arduino

Authors: Abhishek Rathod, Aniket Somwanshi, Joseph Peltroche

Date Updated: 05/12/21
'''


import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
import serial
from project2_ik import inverse_k2


# initial serial for arduino communication
ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
ser.flush()

# The calibration results are loaded from their location
calib_path  = "/home/pi/adv-mech-p3/images/" 
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

# setting font for printing in frame
font = cv2.FONT_HERSHEY_PLAIN

# generating aruco markers from dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()

# coordinates of the corners in the object(aruco tag) frame
object_points = np.float64(np.array([[-25, 25, 0], [25, 25, 0], [25, -25, 0], [-25, -25, 0]]))

# axis to draw the marker pose from center of marker
axis = np.float32([[50, 0, 0], [0, 50, 0], [0, 0, -50]]).reshape(-1, 3)

# axis to draw cube in 3d
axis2 = np.float32([[-25, 25, 0], [25, 25, 0], [25, -25, 0], [-25, -25, 0], [-25, 25, 25], [25, 25, 25], [25, -25, 25], [-25, -25, 25]])
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

counter = 0 # number of times data is sent to the arduino. Used to ensure data is sent only once when the tag is not moving
# counter is also the flag used to wait for the arduino to confirm it is ready to pick up another object

# to get the position of the tag in the camera frame, the camera frame is first flipped

def camera_2_world(pose):
    
    # code to transorm to world frame
    
    H = np.array([[-0.0169,0.9529,-0.3030, 333.],
                 [0.0432,-0.3034,-0.9519,240.0],
                 [-0.9989, -0.0030, 0.0463, 10.],
                 [0, 0, 0, 1.0]])
    
    H_t = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 454.],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    pose = np.array([pose[0], pose[1], pose[2], 1]).reshape(4,1)
    
    pose_world = H @ pose
    return pose_world


def draw(img, corners, imgpts):
    '''
    this function is used to draw a projection of the cube once the tag is detected.
    
    Inputs:
    img: the frame
    corners: detected corners of the tag
    imgpts: projection of cube in camera plane taking into account distortion also
    
    Output:
    frame with cube drawn
    '''
    imgpts = np.int32(imgpts).reshape(-1,2) 

    img = cv2.drawContours(img, [imgpts[:4]], -1, (0, 255, 0), 2)
    
    for i, j in zip(range(4), range(4, 8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 2)
        
    img = cv2.drawContours(img, [imgpts[4:]], -1, (0, 0, 255), 2)
    
    return img

# variable to store previous frame translation vector. used to detect whether tag is moving or not
tvec_prev = np.zeros([3,1])

frame1 = cv2.VideoCapture(0) # initialize video stream
while True:
    # get value from arduino 
    arduino_flag = ser.read()
    if arduino_flag != b'': # if arduino is sending something then set counter to 0
        counter = 0
    
    while counter == 0:  
        ret, frame = frame1.read() # reads frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # frame to gray
        corners, ids, rejecetedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters) # detects markers and stores in id
        
        frame_markers = aruco.drawDetectedMarkers(frame,corners, ids)
        corners = np.array(corners)

        if np.all(ids is not None): # if a tag is detected then do the following
            waste_or_recycle = [0 if ids[0] % 2 == 0 else 1] # based on id detected it classifies the object
            str_waste_or_recycle = ["waste" if ids[0] % 2 == 0 else "recycle"]
            ret = aruco.estimatePoseSingleMarkers(corners, 50, camera_matrix, camera_distortion)
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            moving_id = np.abs(tvec - tvec_prev)
            x = tvec[0]
            y = tvec[1]
            z = tvec[2]
                                    
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0]) # use rodrigues formula to transform rotation vector to matrix
            R_tc    = R_ct.T
            
            pos_camera = -R_tc*np.matrix(tvec).T # inverse of marker to camera. this gives camera to marker transformation
            
            
            tvec_world = camera_2_world(tvec)
            
            if (moving_id < 0.5).all() and counter < 1: # only if the object is still and arduino is ready or the data hasnt been sent yet
                str_moving = " not moving"
                cv2.putText(frame, str_moving, (0, 300), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
                # code to send world pose to arduino
                print("sending...")
                
                m1_theta = (np.arctan2(tvec_world[0], tvec_world[2])) * 180/np.pi
                m1_theta = int(np.abs(m1_theta -180))
                angles = inverse_k2([tvec_world[0], tvec_world[2]])
                angles = [int(a) for a in angles]
                angles_str = "m2=%4.0f  m3=%4.0f  m4=%4.0f"%(angles[0], angles[1], angles[2])
                
                #              m1              m2           m3                  m4              recycling    
                string = [str(m1_theta), str(angles[0]), str(angles[1]), str(angles[2]), str(waste_or_recycle[0])]
                cv2.putText(frame, string[0], (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(frame, angles_str, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                send = ','.join(string)
                send = send + "\n"
                ser.write(send.encode('utf-8'))
                counter = counter + 1 # increment counter so the 
                frame1.release()
                frame1 = cv2.VideoCapture(0)
                break

            str_tvec_world = "Position in WF x=%4.0f  y=%4.0f  z=%4.0f"%(tvec_world[0], tvec_world[1], tvec_world[2])
            cv2.putText(frame, str_tvec_world, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            cv2.putText(frame, str_waste_or_recycle[0], (0, 250), font, 3, (255, 0, 0), 3, cv2.LINE_AA)  
            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, np.float64(rvec), np.float64(tvec), 50)

            # code below is used to draw a cube where the tag is detected. Used for validation of results
            #corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            #imgpts, jac = cv2.projectPoints(axis2, np.float64(rvec), np.float64(tvec), camera_matrix, camera_distortion)
            #frame = draw(frame, corners, imgpts)
            #cv2.imshow('frame', frame)
            
            tvec_prev = tvec # to compare with next frame result
        cv2.imshow('frame', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            frame.release()
            cv2.destroyAllWindows()
            break