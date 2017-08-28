#!/usr/bin/env python

"Import everything from b3m33aro"
from b3m33aro import *
from b3m33aro import detector
import numpy as np
import cv2
import time
"Inside this library I got my function for checking obstacle."
#from obstmark03 import *

"Positive number >> forward, otherwise is backward"
linear_vel = 0.2
"Negative number >> clockwise, otherwise counterclockwise"
angular_vel = -0.5
flag1=False
flag2=False

pGain=0.0095 # proportional gain



def markers_rgb_center(aux_00,aux_image):
    x_points=[]
    y_points=[]
    #It would be 4 points
    for each_point in aux_00:
        x,y=each_point[0],each_point[1]
        x_points.append(x)
        y_points.append(y)
    x1=np.mean(x_points)
    y1=np.mean(y_points)

    rgb_aux=aux_image.shape
    #print(rgb_aux,type(rgb_aux))#It is a tuple
    return (x1,y1,rgb_aux[0]/2,rgb_aux[1]/2)

def db_markers(image1):
	global id_markers
	global flag1

	try:
		markers = detector.detect_markers(image1)
		id_markers=[]
		for each_elem in markers:
			id_markers.append(each_elem[1])
		print('Unsorted:',id_markers,len(id_markers))
		id_markers.sort()
		print('Sorted:',id_markers,len(id_markers))
		time.sleep(1)
		flag1=True
	except Exception as e:
		print(str(e))
	return flag1
def proportional_control(temp1,temp2):

    ">>Proportional control<<"
    
    set_point,process_variable=temp1,temp2
    global pGain
    global control_variable
    error_signal=set_point-process_variable
    control_variable=pGain*error_signal

    print('error_signal',error_signal)


    "Limit the control variable to within +-0.5"
    if (control_variable>0.3):
        control_variable=0.3
    elif (control_variable<-0.3):
        control_variable=-0.3

    if abs(error_signal)<=10:
        aux_flag=False
    else:
        aux_flag=True
    return aux_flag


def main():
    turtle = Turtlebot(True,True,True)
    global flag1
    global flag2

    "__________First_Part______________"
    while(True):
        image = turtle.get_rgb_image()
        if image is None:
            continue
        "Once at the beginning"
        while (not flag1):
        	db_markers(image)

        "______________Second_Part_________________"
        markers_1 = detector.detect_markers(image)
        for each_one,j in zip(markers_1,range(len(markers_1))):
            print('______%i _IDmarker:%i________'%(j,each_one[1]))
            for x in range (len(id_markers)):
                if each_one[1]==id_markers[x]:#New data compare with my DBdata
                    id_markers[x]=255#if so update my DBdata
                    turtle.reset_odometry()
                    time.sleep(0.2)
                    ang_odo_start=turtle.get_odometry()
                    print(ang_odo_start,turtle.get_odometry()[2])
                    flag2=True
                    break
                else:
                    continue
            "____________Third_Part________________"
            print('New one will be processed')
            print('IDmarkers',id_markers,'Len:',len(id_markers))

            while (flag2):
                try:
                    image = turtle.get_rgb_image()
                    if image is None:
                        continue
                    markers_2 = detector.detect_markers(image)
                    for each_one_new in markers_2:
                        if each_one_new[1]==each_one[1]:
                            aux_00=each_one_new[0]#Take the corners
                            "Center of the markers and Center of the RGB-D (Kinect)"
                            mark_x,mark_y,rgb_x,rgb_y=markers_rgb_center(aux_00,image)
                            break
                    flag2=proportional_control(rgb_x,mark_x)
                    turtle.cmd_velocity(0,control_variable)

                    if not (flag2):

                        turtle.play_sound(5)
                        time.sleep(0.5)
                        turtle.play_sound(5)
                        time.sleep(0.5)
                        turtle.play_sound(5)
                        time.sleep(0.5)

                        ang_odo_end=turtle.get_odometry()[2]
                        t=turtle.get_odometry()
                        turtle.reset_odometry()
                        time.sleep(0.2)
                        tmp_ang=0

                        if ang_odo_end<0:
                            angular_vel=0.3
                        elif ang_odo_end>0:
                            angular_vel=-0.3
                        print(t,ang_odo_end)
                        time.sleep(2)

                        while abs(tmp_ang)<abs(ang_odo_end):
                            turtle.cmd_velocity(0,angular_vel)
                            tmp_ang=turtle.get_odometry()[2]


                        turtle.reset_odometry()
                        print('Ready for the next marker!')
                        time.sleep(5)
                except Exception as e:
                    print(str(e))
        if np.mean(id_markers)==255:
            print('It will start a nes cycle!')
            flag1=False
            print('---------------------------------------------------------------')
            print("Here we go again, with our first step in making data base of my Id markers")
            time.sleep(3)



if __name__ == "__main__":
    main()
