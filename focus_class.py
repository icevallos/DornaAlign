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

## these are some useful commands for the robot
## I type them here so I can use them quickly on the command line
rest_cmd  = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed":1000, "j0" : 0. , "j1" : 145., "j2" : -90, "j3" : 0.0 , "j4" : 0.0}} 
angle_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "a" : -90, "b": 0}}
start_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "speed" : 1000, "j0" : -7 , "j1" : 50., "j2" : -50.}}
vertical_cmd = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed": 1000, "j0" : 0. , "j1" : 90., "j2" : 0., "j3" : 0.0 , "j4" : 0.0}}
reset_j0 = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed":1000 ,"j0" : 0. }}

class SpectroAlign(object):
    rest_cmd  = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed":1000, "j0" : 0. , "j1" : 145., "j2" : -90, "j3" : 0.0 , "j4" : 0.0}}
    angle_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "a" : -90, "b": 0}}
    start_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "speed" : 1000, "j0" : -7 , "j1" : 50., "j2" : -50.}}
    vertical_cmd = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed": 500, "j0" : 0. , "j1" : 90., "j2" : 0., "j3" : 0.0 , "j4" : 0.0}}
    reset_j0 = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed":1000 ,"j0" : 0. }}


    def __init__(self,robot = None ,center = None,rel_pos = [0.,0.,0.], j0_offset = 0.,
                data_dir = None):

        """
        This creates the python object, we can specify initial conditions for the robot
        Typically, we assume we're starting from the rest position, pointed away from the table.
        """
        if robot == None:
            self.robot = Dorna()
        else:
            self.robot = robot
        

        if center == None:
            self.center = [0.,0.,0.,0.,0.]
            self.center_set = False
        else:
            self.center = center
            self.center_set = True

        if data_dir == None:
            self.data_dir = '/home/fireball2/SpectroAlign/' + datetime.now().strftime("%y%m%d") +'/'
        else:
            self.data_dir =  data_dir

        self.logfile  =  open(self.data_dir + "logfile", 'a')

        self.rel_pos = rel_pos
        self.j0_offset = j0_offset


    def __repr__(self):
        return self.robot.device()


    def connect(self,port =  "/dev/ttyACM0"):
        """
        connects the robot arm.
        input:
            port: (string) port address for robot
        """
        self.robot.connect(port)

        return self.robot.device()

    def checkout(self):
        """
        Disconnect robot and terminate robot dorna object
        """
        self.robot.disconnect()
        self.robot.terminate()

        return self.robot.device()

    def calibrate(self,position = [0.,90.,0.,0.,0.]):
        """
        Defines current joint position of the robot in the Dorna joint system.
        defaults to calibrating on the stretched upwards position
        input:
            position: (list of floats) [j0,j1,j2,j3,j4] in degrees
        returns:
            new joint values
        """
        self.completion()
        self.robot.calibrate(position)
        self.robot.save_config()
        return self.robot.position()


    #### Robot motion and dorna parameter methods

    def play(self, cmd):
        """
        passes Dorna command to embeded Dorna object
        input:
            cmd: (dict) in Dorna format for motions
        output:
            dorna play command output
        """

        output = self.robot.play(cmd)
        self.find_rel()
        return output

    def home(self, joint):
        """
        passes joint name to be homed
        input:
            joint: (string) name of joint
        output:
            dorna play command output
        """

        output = self.robot.home(joint)
        return output



    def set_center(self,coords = None):
        """
        Sets center of field coordinates at current robot position, or at specified coords.
        input:
            coords: (list of floats) x,y,z,a,b position of arm in dorna coordinates
        output:
            (list of floats) position of new center in dorna xyzab format
        """     

        if coords == None:
            self.center  = json.loads(self.robot.position("xyz"))
        else:
            self.center = coords

        print("Fiber Center :" + str (self.center))

        self.logfile.write("\nSet Fiber Center [mm]: " + str(self.center))
        self.center_set = True

        return self.center

    def find_rel(self):
        """
        Calculates and updates relative position to center of field
        """

        robot_pos = json.loads(self.robot.position("xyz"))
        x = robot_pos[0] - self.center[0] 
        y = robot_pos[1] - self.center[1]
        z = robot_pos[2] - self.center[2]

        self.rel_pos  = [x,y,z]

        print("relative position : " + str(self.rel_pos))

        return self.rel_pos

    def to_center(self):
        """
        Returns the robot to the center of field as recorded.
        """
        if self.center_set:
            center_cmd =  {"command": "move", "prm": {"movement": 0, "path": "line" , "xyz" : self.center }}
            self.play(center_cmd)
        else:
            print("Center coordinate has not been set")

        return None



    def ax_rotation(self,offset):

        """rotates around z and redefines x, y coordinates of the robot.
        j0 offset: degrees in counterclockwise direction looking from above. 

        """    
    
        self.play(SpectroAlign.rest_cmd)
        self.play({"command" : "move", "prm":{"movement" : 0, "speed":1000, "path": "joint", "j0" : offset }})
        self.j0_offset += offset
        self.completion()
        self.robot.set_joint({"j0" : 0 })

        return None

    def completion(self):
        """
        Wait until previous robot motion is completed.
        """


        status = self.robot.device()
        parsed_status = json.loads(status)

        while parsed_status['state'] != 0:
            time.sleep(0.5)
            status = self.robot.device()
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
            self.play(grid_x)
        elif coord  == "z":
            grid_z = {"command" : "move" , "prm" : {"movement" :  1 ,"speed": speed, "path" : "line", "z" : value}}
            self.play(grid_z)
        elif coord == "y":
            grid_y = {"command" : "move" , "prm" : {"movement" :  1 ,"speed": speed, "path" : "line", "y" : value}}
            self.play(grid_y)

        else:
            print("Invalid coordinate command")

        self.completion()
        self.find_rel()

        
        return None

    def joint_motion(self,coord,value,speed = 1000 ,movement = 1):
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
            self.play(cmd)
        elif coord  == "j1":
            cmd = {"command" : "move" , "prm" : {"movement" :  movement ,"speed" : speed, "path" : "joint", "j1" : value}}
            self.play(cmd)
        elif coord == "j2":
            cmd = {"command" : "move" , "prm" : {"movement" :  movement,"speed" : speed, "path" : "joint", "j2" : value}}
            self.play(cmd)
        elif coord == "j3":
            cmd = {"command" : "move" , "prm" : {"movement" :  movement,"speed" : speed, "path" : "joint", "j3" : value}}
            self.play(cmd)
        elif coord == "j4":
            cmd = {"command" : "move" , "prm" : {"movement" :  movement,"speed" : speed, "path" : "joint", "j4" : value}}
            self.play(cmd)


        else:
            print("Invalid coordinate command")

        
        return None

    def find_pos(self):
        """
        returns position in dorna xyz coordinates as list
        output:
            (list of floats) xyzab coordinates
        """

        position = json.loads(self.robot.position("xyz"))

        print("x,y,z,a,b =" + str(position))

        return position


    ### Nuvu control methods

    def set_log(self):

        """
        Creates log file for image taking, sets up camserver image path

        output
            (textio) pointer to file.
        """

        #data_dir = '/home/fireball2/SpectroAlign/' + datetime.now().strftime("%y%m%d") +'/'
    
        command = "mkdir -p " + data_dir
        p= Popen(command, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        #logfile = open(data_dir+'logfile', 'a')

        command = 'cam path ' + data_dir
        p = Popen(command, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        output = p.stdout.read()
        self.logfile.write('Data directory=' + self.data_dir + "\n")
        return None

    def expose(self,exptime, burst = 1):
        # Take images
        command = 'cam imno' 
        p = Popen(command, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        imno1 = p.stdout.read()
        #self.logfile.write('\nimage%06d.fits' % int(imno1))
        command0 = 'cam burst='+str(burst)
        print(command0)
        # Run command
        p = Popen(command0, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        # Write to the log file
        bpar = p.stdout.read()
        #self.logfile.write('\nBurst param= %s' % str(bpar))
        #subprocess.call(command0,shell=True)
        
        # Set the exposure time
        command1 = 'cam exptime='+str(exptime)
        print(command1)
        # Run command 
        p = Popen(command1, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
       
        expout = p.stdout.read()
        #self.logfile.write('\nExposure time= %s' % str(expout))

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

         # Write to the log file
        for i in range(burst):
            self.logfile.write('\nimage%06d.fits' % int(imno1+i))
            self.logfile.write('\t%s' % str(expout))
            self.logfile.write('\t%s' % str(bpar))
            self.logfile.write('\t%s' % str(burst))
            self.logfile.write('\t%s' % str(self.rel_pos[0]))
            self.logfile.write('\t%s' % str(self.rel_pos[1]))
            self.logfile.write('\t%s' % str(self.rel_pos[2]))
            self.logfile.write
            

        time.sleep(exptime*burst)
        return None

        ### Combined routines (exposures + motion)

    def focus_test(self, steps=10, dz = 0.5 , exptime = 1., burst =1, dx0 = 0., dy0 = 0.,dz0 = 0.):
        """
        Takes through focus image bursts as the robot moves upwards in z.

        input:
            
            
            steps: (int) number changes in position
            dz: (float) separation between steps in mm
            exptime =  (float) exposure in seconds
            burst = (int) number of images per exposure
            dx0 = (float) displacement from original x position in mm
            dy0 = (float) displacement from original y position in mm
            dz0 = (float) displacement from original z position in mm
        """


        print("Sweeping focus on Z")

        start_coord = json.loads(self.robot.position("xyz"))
        return_cmd = {"command": "move", "prm": {"movement": 0, "path": "line" , "xyz" : start_coord}}

        if dx0 != 0.:
            self.xyz_motion( "x", dx0)
        if dy0 != 0.:
            self.xyz_motion("y", dy0)
        if dz0 != 0.:
            self.xyz_motion("z",dz0)


        for i in range(steps):
            z_cmd = {"command" : "move" , "prm" : {"movement" :  0 , "path" : "line", "z" : start_coord[2]+(dz*i)}}
            self.play(z_cmd)
            self.completion()
            print("z =  " + str(start_coord[2]+dz))
            
            print('\nRelative position [mm] %s' % str(self.rel_pos))

            self.expose(exptime,burst)

        print("Sweep complete, returning to start position")
        
        self.play(return_cmd)
        self.completion()
        return None

















