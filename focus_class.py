#robot as an object

import os.path
import os, sys
import json
import subprocess
from subprocess import Popen, PIPE, STDOUT
import sys
import time
from datetime import datetime,timedelta

from dorna import Dorna

rest_cmd  = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed":1000, "j0" : 0. , "j1" : 145., "j2" : -90, "j3" : 0.0 , "j4" : 0.0}} 
angle_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "a" : -90, "b": 0}}
start_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "speed" : 1000, "j0" : -7 , "j1" : 50., "j2" : -50.}}
vertical_cmd = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed": 500, "j0" : 0. , "j1" : 90., "j2" : 0., "j3" : 0.0 , "j4" : 0.0}}
reset_j0 = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed":1000 ,"j0" : 0. }}

class focal_test(object):
    rest_cmd  = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed":1000, "j0" : 0. , "j1" : 145., "j2" : -90, "j3" : 0.0 , "j4" : 0.0}}
    angle_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "a" : -90, "b": 0}}
    start_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "speed" : 1000, "j0" : -7 , "j1" : 50., "j2" : -50.}}
    vertical_cmd = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed": 500, "j0" : 0. , "j1" : 90., "j2" : 0., "j3" : 0.0 , "j4" : 0.0}}
    reset_j0 = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed":1000 ,"j0" : 0. }}


    def __init__(self,robot,center = [0.,0.,0.,0.,0.],rel_pos = [0.,0.,0.], j0_offset = 0.,
                logfile = None):
        self.robot = robot
        self.center = center
        self.rel_pos = rel_pos
        self.j0_offset = j0_offset
        self.logfile = logfile




    def connect(self,port =  "dev/ttyACM0"):
        self.robot.connect(port)
        return None

    def checkout(self):
        self.robot.disconnect()
        self.robot.terminate()
        return None


    #### Robot motion and dorna parameter methods

    def set_center(self,coords = None):     

        if coords == None:
            self.center  = json.loads(self.robot.position("xyz"))
        else:
            self.center = coords

        print("Center of field :" + str (self.center))

        self.logfile.write("\nField Center : " + str(self.center))

        return self.center

    def find_rel(self):

        robot_pos = json.loads(self.robot.position("xyz"))
        x = robot_pos[0] - self.center[0] 
        y = robot_pos[1] - self.center[1]
        z = robot_pos[2] - self.center[2]

        self.rel_pos  = [x,y,z]

        print("relative position : " + str(self.rel_pos))

        return self.rel_pos




    def ax_rotation(self,offset):

        """rotates around z and redefines x, y coordinates of the robot.
        j0 offset: degrees in counterclockwise direction looking from above. 

        """    
    
        self.robot.play(focal_test.rest_cmd)
        self.robot.play({"command" : "move", "prm":{"movement" : 0, "speed":1000, "path": "joint", "j0" : j0_offset }})
        self.j0_offset += offset
        self.completion()
        robot.set_joint({"j0" : 0 })

        return None

    def completion(self):


        status = self.robot.device()
        parsed_status = json.loads(status)

        while parsed_status['state'] != 0:
            time.sleep(0.5)
            status = robot.device()
            parsed_status = json.loads(status)

        #print("Command is done. On to the next!")

        return None

    def xyz_motion(self,coord,value,speed = 2500):

        """
        Moves the fiber in x y z directions
        input:
            speed: (float) unit length/minute
            coord: (string) "x", "y" or "z" for direction of motion
            value: (float)
        """
        self.completion()
        if coord == "x":
            grid_x = {"command" : "move" , "prm" : {"movement" :  1 ,"speed": speed, "path" : "line", "x" : value}}
            self.robot.play(grid_x)
        elif coord  == "z":
            grid_z = {"command" : "move" , "prm" : {"movement" :  1 ,"speed": speed, "path" : "line", "z" : value}}
            self.robot.play(grid_z)
        elif coord == "y":
            grid_y = {"command" : "move" , "prm" : {"movement" :  1 ,"speed": speed, "path" : "line", "y" : value}}
            self.robot.play(grid_y)

        else:
            print("Invalid coordinate command")

        
        return None

    def joint_motion(self,coord,value,speed = 2540 ,movement = 1):
        """
        Shorthand function to move robot joints
        input:
            robot: (Dorna object)
            coord: (string) "j0", "j1", "j2", "j3" or "j4" for direction of motion
            value: (float) angle in degrees
            speed: (float) degrees/min
            movement: 1 for relative, 0 for absolute
        """
        self.completion()
        if coord == "j0":
            cmd = {"command" : "move" , "prm" : {"movement" :  movement ,"speed" : speed, "path" : "joint", "j0" : value}}
            self.robot.play(cmd)
        elif coord  == "j1":
            cmd = {"command" : "move" , "prm" : {"movement" :  movement ,"speed" : speed, "path" : "joint", "j1" : value}}
            self.robot.play(cmd)
        elif coord == "j2":
            cmd = {"command" : "move" , "prm" : {"movement" :  movement,"speed" : speed, "path" : "joint", "j2" : value}}
            self.robot.play(cmd)
        elif coord == "j3":
            cmd = {"command" : "move" , "prm" : {"movement" :  movement,"speed" : speed, "path" : "joint", "j3" : value}}
            self.robot.play(cmd)
        elif coord == "j4":
            cmd = {"command" : "move" , "prm" : {"movement" :  movement,"speed" : speed, "path" : "joint", "j4" : value}}
            self.robot.play(cmd)


        else:
            print("Invalid coordinate command")

        
        return None

    def find_pos(self):
        """
        outputs xyz position as list
        """

        position = json.loads(self.robot.position("xyz"))

        print("x,y,z,a,b =" + str(position))

        return position


    ### Nuvu control methods

    def set_log(self):

        """
        Creates log file for image taking, sets up camserver image path
        """

        data_dir = '/home/fireball2/SpectroAlign/' + datetime.now().strftime("%y%m%d") +'/'
    
        command = "mkdir -p " + data_dir
        p= Popen(command, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        self.logfile = open(data_dir+'logfile', 'a')

        command = 'cam path ' + data_dir
        p = Popen(command, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        output = p.stdout.read()
        self.logfile.write('Data directory=' + data_dir + "\n")
        return self.logfile

    def expose(self,exptime, burst = 1):
        # Take images
        command = 'cam imno' 
        p = Popen(command, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        output = p.stdout.read()
        self.logfile.write('\nimage%06d.fits' % int(output))
        command0 = 'cam burst='+str(burst)
        print(command0)
        # Run command
        p = Popen(command0, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        # Write to the log file
        output = p.stdout.read()
        self.logfile.write('\nBurst param= %s' % str(output))
        #subprocess.call(command0,shell=True)
        
        # Set the exposure time
        command1 = 'cam exptime='+str(exptime)
        print(command1)
        # Run command 
        p = Popen(command1, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        # Write to the log file
        output = p.stdout.read()
        self.logfile.write('\nExposure time= %s' % str(output))
        #subprocess.call(command1,shell=True)

        # command2 = './cam emgain='+str(emgain)
        # print command2
        # Run command
        # p = Popen(command2, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        # Write to the log file
        # output = p.stdout.read()
        # logfile.write('EM gain= %s' % str(output))
        #subprocess.call(command2,shell=True)
        
        command3 = 'cam expose'
        print(command3)
        # Run command
        p = Popen(command3, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        time.sleep(exptime)
        return None

        ### Combined routines (exposures + motion)

    def focus_test(self, steps=10, dz = 0.5 , exptime = 1., burst =1, x_offset = 0., y_offset = 0.):
        """
        Takes through focus image bursts as the robot moves upwards in z.

        input:
            
            
            steps: (int) number changes in position
            dz: (float) separation between steps in mm
            exptime =  (float) exposure in seconds
            burst = (int) number of images per exposure
            x_offset = (float) displacement from original x position in mm
            y_offset = (float) displacement from original y position in mm
        """


        print("Sweeping focus on Z")

        start_coord = json.loads(self.robot.position("xyz"))
        return_cmd = {"command": "move", "prm": {"movement": 0, "path": "line" , "xyz" : start_coord}}

        if x_offset != 0.:
            self.xyz_motion( "x", x_offset)
        if y_offset != 0.:
            self.xyz_motion("y", y_offset)


        for i in range(steps):
            
            
            z_cmd = {"command" : "move" , "prm" : {"movement" :  0 , "path" : "line", "z" : start_coord[2]+(dz*i)}}
            self.robot.play(z_cmd)
            self.completion()
            print("z =  " + str(start_coord[2]+dz))
            self.find_rel()
            print(self.rel_pos)
            self.logfile.write('\nRelative position [mm] %s' % str(self.rel_pos))
            self.expose(exptime,burst)

        print("Sweep complete, returning to start position")
        
        self.robot.play(return_cmd)
        self.completion()
        return None















