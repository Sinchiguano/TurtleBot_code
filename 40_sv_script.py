#By:Sinchiguano Cesar

"Import everything from b3m33aro"
from b3m33aro import *
from b3m33aro import detector
import numpy as np
import cv2
import time
import math


"Positive number >> forward, otherwise is backward"
linear_vel = 0.3#0.2
"Negative number >> clockwise, otherwise counterclockwise"
angular_vel = -0.3
"Variables"
flag1=False#for the db_markers
flag2=True
flag3=True
#pGain=0.0095 # proportional gain
#pGain=0.045#Start to oscillate
#pGain=0.0045#so so it arrives into the middle of the
#pGain=0.025#Very good oscillation
pGain=0.0050
iGain=0.0000035
dGain=0.00045
error_signal_sum=0
last_error=0

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
	global total_markers
	try:
		markers = detector.detect_markers(image1)
		id_markers=[]#list
		total_markers=0
		for each_elem in markers:
			id_markers.append(each_elem[1])
		print('Unsorted:',id_markers,len(id_markers))
		total_markers=len(id_markers)
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
    global iGain
    global dGain
    global control_variable
    global error_signal_sum
    global last_error

    error_signal=set_point-process_variable
    Derivative=error_signal-last_error
    control_variable=pGain*error_signal+iGain*error_signal_sum+dGain*Derivative
    error_signal_sum+=error_signal#sum of error
    last_error=error_signal
    if(error_signal_sum>5000):
    	error_signal_sum=5000
    if (error_signal_sum<-5000):
    	error_signal_sum=5000
    #print('error_signal',error_signal,'error_sum:',error_signal_sum,'Control:',control_variable)
    "Limit the control variable to within +-0.5"
    if (control_variable>0.3):
        control_variable=0.3
    elif (control_variable<-0.3):
        control_variable=-0.3
    if abs(error_signal)<=10:#Error was 4.0 in my final code
        aux_flag=False
    else:
        aux_flag=True
    #aux_flag=False##just to test
    return aux_flag

def check_obstacle():
    flag=True
    turtle = Turtlebot(pc=True)
    state=False
    while (flag):
        pc = turtle.get_point_cloud()
    	if pc is None:
     	    continue
        mask=pc[:,:,1] < 0.2
        # mask point too far
        mask = np.logical_and(mask, pc[:,:,2] < 3)
        if np.count_nonzero(mask) <= 0:
            continue
        mask = np.logical_and(mask, pc[:,:,1] > -0.2)
        data = np.sort(pc[:,:,2][mask])
        if data.size > 50:
            dist = np.percentile(data, 10)#I changed it was 10
            if dist < 0.6:
                state=True#I found some obstacle so I will change my state to true
        flag=False#It means that it will run once, will break the while loop
    return state

def initial_position(save_position):
    turtle = Turtlebot(True,True,True)
    ang_odo_end=save_position[2]#I took the value of the angular position
    ang_odo_end=1*ang_odo_end
    turtle.get_odometry()
    time.sleep(0.2)
    tmp_ang=0
    auxfer=0
    if ang_odo_end<0:#If it is negative i will move in a positive way
        angular_vel=0.3
    elif ang_odo_end>0:#Otherwise
        angular_vel=-0.3
    while abs(tmp_ang)<abs(ang_odo_end):
        turtle.cmd_velocity(0,angular_vel)
        tmp_ang=turtle.get_odometry().tolist()[2]
    turtle.reset_odometry()
    time.sleep(0.03)
    print('Ready for the next marker!','Odometry:',save_position)

def db_track(marker_done):
    turtle = Turtlebot(True,True,True)
    for x in range (total_markers):
        if marker_done==id_markers[x]:#New data compare with my DBdata
            id_markers[x]=255#if so update my DBdata
            turtle.reset_odometry()
            time.sleep(0.2)
    print('Id_markers dB:',id_markers)
def backward_control(some_x,some_y,some_save_pose):
    turtle=Turtlebot(True,True,True)
    CurPose_x_ori=some_save_pose[0]
    CurPose_y_ori=some_save_pose[1]
    CurPose_x_sum=np.add(some_x,CurPose_x_ori)
    CurPose_y_sum=np.add(some_y,CurPose_y_ori)
    CurPose_x_sub=np.subtract(some_x,CurPose_x_ori)
    CurPose_y_sub=np.subtract(some_y,CurPose_y_ori)
    ang_y_x = np.arctan2(some_y,some_x)
    print('Ang_y_x:',ang_y_x,'Ang_y_x in degrees',ang_y_x*180/math.pi)
    return ang_y_x,CurPose_x_sub,CurPose_y_sub
def parameters_backward(ang_y_x,CurPose_theta):
    veloangle=0
    aux_theta=0
    if abs(ang_y_x)>abs(CurPose_theta):
        if CurPose_theta<0:
            aux_theta=-(abs(ang_y_x)-abs(CurPose_theta))
            veloangle=-0.2
            print('Case 1.1:',aux_theta,aux_theta*180/math.pi)
        elif CurPose_theta>0:
            aux_theta=+(abs(CurPose_theta)-abs(ang_y_x))
            veloangle=+0.2
            print('case 1.2:',aux_theta,aux_theta*180/math.pi)
        print('1 aux_theta:',aux_theta,aux_theta*180/math.pi)#it was -
        print('CurPose_theta:',CurPose_theta*180/math.pi)
    elif(abs(ang_y_x)<abs(CurPose_theta)):
        if CurPose_theta>0:
            aux_theta=-(abs(CurPose_theta)-abs(ang_y_x))
            veloangle=-0.2
            print('2.1:',aux_theta,aux_theta*180/math.pi)
        elif CurPose_theta<0:
            aux_theta=+(abs(CurPose_theta)-abs(ang_y_x))
            veloangle=+0.2
            print('2.2:',CurPose_theta,CurPose_theta*180/math.pi)
        print('2 aux_theta:',aux_theta,aux_theta*180/math.pi)
        print('CurPose_theta:',CurPose_theta*180/math.pi)
    return aux_theta,veloangle

def navigation_turtle(somedata):
    turtle = Turtlebot(True,True,True)
    flag3=True
    flag_flag=True
    aux_flag=True
    state_state=True
    Found_it=False
    turtle.reset_odometry()
    time.sleep(0.2)
    "Variables for coming back"
    CurPose_x = []
    CurPose_y = []
    CurPose_theta=[]
    list_path=[]
    while (aux_flag):
        image = turtle.get_rgb_image()
        if image is None:
            continue
        markers= detector.detect_markers(image)
        for each_one in markers:
            if each_one[1]==somedata:#from my dB
                marker_done=each_one[1]
                aux_00=each_one[0]#Take the corners
                "Center of the markers and Center of the RGB-D (Kinect)"
                mark_x,mark_y,rgb_x,rgb_y=markers_rgb_center(aux_00,image)
                Found_it=True
                break
            else:
                Found_it=False
                continue
        if not Found_it:
            state_state=False
            aux_flag=False
            marker_done=None
            continue
        "###########################"
        flag_flag=proportional_control(rgb_x,mark_x)

        if (flag3 and flag_flag):
            turtle.cmd_velocity(0,control_variable)#another approach
        elif (flag3 and not flag_flag):
            flag3=False
            save_position=turtle.get_odometry()
            #turtle.reset_odometry()
            #time.sleep(0.3)
            print('Initial position toward my target:',save_position)
            print('Navigation active!')
        else:
            if not(check_obstacle()):
                turtle.cmd_velocity(linear_vel,control_variable)
                print('From',save_position,'toward:',turtle.get_odometry())
                list_path.append(turtle.get_odometry()[2])
            elif(check_obstacle()):
            	print('I have done from:',save_position,'toward:',turtle.get_odometry())
            	list_path.append(turtle.get_odometry()[2])
            	CurPose_x_y_theta=turtle.get_odometry()
                CurPose_x=turtle.get_odometry().tolist()[0]
                CurPose_y=turtle.get_odometry().tolist()[1]
                CurPose_theta=turtle.get_odometry().tolist()[2]
                CurPose_h=math.hypot(CurPose_x,CurPose_y)
                print('H_dist:',math.hypot(CurPose_x,CurPose_y))
                print('End point x and y:',CurPose_x,CurPose_y,'Angle:',CurPose_theta,CurPose_theta*180/math.pi)
                for i in range(6):
                    turtle.play_sound(6)
                    time.sleep(0.5)
                    i+=1
                "_________________Backward_____________________"
                ang_y_x,aux_x,aux_y=backward_control(CurPose_x,CurPose_y,save_position)
                tmp_x=0
                turtle.reset_odometry()
                time.sleep(0.3)
                aux_theta,veloangle=parameters_backward(ang_y_x,CurPose_theta)
                extra_aux=0
                while(abs(extra_aux*180/math.pi)<abs(aux_theta*180/math.pi)):
                   	turtle.cmd_velocity(0,veloangle)
                	extra_aux=turtle.get_odometry()[2]
                while(tmp_x<(90*CurPose_x/100)):
                    turtle.cmd_velocity(-linear_vel,0)
                    tmp_x=abs(turtle.get_odometry().tolist()[0])
                end_position=turtle.get_odometry()
                print('Final pose after coming back:',end_position,end_position[2]*180/math.pi)
                initial_position(end_position)
                #initial_position(save_position)
                aux_flag=False
    return state_state,marker_done

def main():
    turtle = Turtlebot(True,True,True)
    global flag1
    global flag2
    list_missed=[]
    "__________First_Part______________"
    while(True):
        image = turtle.get_rgb_image()
        if image is None:
            continue
        "Once at the beginning"
        while (not flag1):
        	flag1=db_markers(image)
        "______________Second_Part_______________"
        counter=0
        for  cicle in range (total_markers):
            print('______%i _IDmarker:_______'%(id_markers[cicle]))
            counter+=1
            "___________________________Fourth_Part____________________"
            state_state,marker_done=navigation_turtle(id_markers[cicle])
            if state_state:
                db_track(marker_done)
            elif (not state_state and marker_done==None):
            	list_missed.append(id_markers[cicle])
                continue
        print('Counter:',counter)
        print('list_missed:',list_missed)
        print(list_missed==[])
        print(list_missed!=[])
        if  (np.mean(id_markers)!=255):
            filter=(id_markers!=255)
            id_markers[filter]#update
            list_missed=[]
            continue
        if np.mean(id_markers)==255:
        	try:
        		print('All markers have been reached successfully!')
        		start_key=raw_input('Enter the letter s and push the key Enter to start again\n')
        		if start_key=='s':
        			flag1=False
        			print("Here we go again, with our first step in making data base of my Id markers")
        			time.sleep(2)
        	except Exception as e:
                    print(str(e))
        if  (np.mean(id_markers)!=255):
            filter=(id_markers!=255)
            print('The following marker(s) was(were) not reached successfully!',id_markers[filter])
            try:
                start_key=raw_input('Try again a new cicle, enter the key S and push enter:\n')
                if start_key=='s':
                    flag1=False
                    print("Here we go again, with our first step in making data base of my Id markers")
                    time.sleep(2)
            except Exception as e:
                print(str(e))
if __name__ == "__main__":
    main()
