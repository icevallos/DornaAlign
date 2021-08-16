#robot as an object
## to test using macbook

import os.path
import os, sys
import json
import subprocess
from subprocess import Popen, PIPE, STDOUT, run
import sys
import time
from datetime import datetime,timedelta
from laser_serial import LaserIO

import io
import serial


from dorna import Dorna

## these are some useful commands for the robot
## I type them here so I can use them quickly on the command line
rest_cmd  = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed":1000, "j0" : 0. , "j1" : 145., "j2" : -90, "j3" : 0.0 , "j4" : 0.0}} 
angle_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "a" : -90, "b": 0}}
start_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "speed" : 1000, "j0" : -7 , "j1" : 50., "j2" : -50.}}
vertical_cmd = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed": 1000, "j0" : 0. , "j1" : 90., "j2" : 0.}}
reset_j0 = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed":1000 ,"j0" : 0. }}

centerfield = [334.97,-5.0013,306.99,-90,0] #xyz center of field for zn lamp spot.

class SpectroAlign(object):
    rest_cmd  = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed":1000, "j0" : 0. , "j1" : 145., "j2" : -90, "j3" : 0.0 , "j4" : 0.0}}
    angle_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "a" : -90, "b": 0}}
    start_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "speed" : 1000, "j0" : -7 , "j1" : 50., "j2" : -50.}}
    vertical_cmd = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed": 500, "j0" : 0. , "j1" : 90., "j2" : 0., "j3" : 0.0 , "j4" : 0.0}}
    reset_j0 = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed":1000 ,"j0" : 0. }}


    def __init__(self,robot = None ,center = None,rel_pos = [0.,0.,0.,0.,0.], j0_offset = 0.,
                data_dir = None,  laser = None):

        """
        This creates the python object, we can specify initial conditions for the robot
        Typically, we assume we're starting from the rest position, pointed away from the table.
        """
        if center == None:
            self.center = [0.,0.,0.,0.,0.]
            self.center_set = False
        else:
            self.center = center
            self.center_set = True

        if data_dir == None:
            self.data_dir =  "/home/fireball2/SpectroAlign/" + datetime.now().strftime("%y%m%d") +'/' #'/Users/ignacio/' + datetime.now().strftime("%y%m%d") +'/'
        else:
            self.data_dir =  data_dir

        self.log_str  =  self.data_dir + "logfile"

        #self.logfile  =  open(self.data_dir + "logfile", 'a')

        self.rel_pos = rel_pos
        self.j0_offset = j0_offset

        if robot == None:
            self.robot = Dorna()
        else:
            self.robot = robot

        self.imno = 0

        if laser == None:
            self.laser = None
            self.laser_set = False
        else:
            self.laser = laser
            self.laser_set = True

        self.z_laser  = 0.



    def __repr__(self):
        return self.robot.device()

    ####### Laser readout tools

    def laser_read(self, set_target  = False):
        '''
        launches laser_main program and reads measurement
        output: (float) laser distance measured [mm]

        Set target records value to Z attribute
        '''

        if (self.laser_set):
            out = self.laser.measure()
            return out

            if set_target:
                self.z_laser = out

        else:
            print("Laser not configured")
            return None



        '''
        cd_cmd = "cd /home/fireball2/laser"
        subprocess.run(cd_cmd)

        laser_cmd = "./laser_main"

        result = subprocess.run(laser_cmd, stdout = PIPE)
        print(result.stdout.decode())
        i = result.rfind("M0,") + 3
        if i == -1:
            print("measurement not found")
            return None
        j = result[i:].find[","]

        distance  = float(result[i:i+j])
        return distance '''

    def setup_laser(self):
        '''
        Open serial connection to laser, creates laser_serial object
        '''
        if (self.laser_set):
            print("Laser port already open")
            return None


        self.laser = LaserIO()
        self.laser_set  = True
        self.z_laser = self.laser.measure()
        return None



        '''
        readout  = self.laser_read()
        if readout == None:
            print("Laser not found")
            return None
        else:
            self.z_laser = readout
            self.laser = True
            return None'''

    def z_correct(self):
        '''
        adjusts z position and relative coordinates to account for drift
        '''
        if self.laser_set == True:
            readout = self.laser_read()
            z_offset = self.z_laser - readout
            print('Laser offset correction: %s mm' % str(self.z_offset))
            self.xyz_v2(z = z_offset)
            self.set_center()
            return None
        else:
            print("Laser not configured!")
            return None

    def connect(self,port =  "/dev/ttyACM0"): ##"/dev/cu.usbmodem14101" for mac
        """
        connects the robot arm.
        input:
            port: (string) port address for robot
        """
        self.robot.connect(port)

        return self.robot.device()

    def set_j3(self):
        """
        moves j3/j4 to zero position, starting from fiber holder 
        in the calibration position (touching the top of the arm)

        """

        self.robot.set_joint({"j3" : 152.0 , "j4" : 0.0})
        self.joint_v2(j3 =  -152)


        return None



    def checkout(self):
        """
        Disconnect robot and terminate robot dorna object
        """
        self.robot.disconnect()
        self.robot.terminate()
        if self.laser_set:
            self.laser.close()

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
        self.robot.set_joint(position)
        self.robot.save_config()
        return self.robot.position()

    def log_line(self, line):
        """
        Write line to logfile
        """

        logfile  =  open(self.log_str, 'a')
        logfile.write(line)
        logfile.close()

        return None



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
        self.completion()
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

        #print("Fiber Center :" + str (self.center))

        #self.log_line("\nSet Fiber Center [mm]: " + str(self.center))
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
        a = robot_pos[3]
        b = robot_pos[4]

        self.rel_pos  = [x,y,z,a,b]

        #print("relative position : " + str(self.rel_pos))

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

    def to_xyz(self,coords):
        """
        Moves robot to given position in xyz
        """

        motion_cmd = {"command": "move", "prm": {"movement": 0, "path": "line" , "xyz" : coords}}
        self.play(motion_cmd)
        return None

    def to_joint(self,coords):
        """
        Moves robot to given position in joint coord.
        """

        motion_cmd = {"command": "move", "prm": {"movement": 0, "path": "line" , "joint" : coords}}
        self.play(motion_cmd)
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

    def xyz_v2(self,x = None ,y = None, z = None, speed = 2500, movement = 1, y_angle = True):
        """
        Moves the fiber in x y z directions
        input:
            speed: (float) unit length/minute
            dx, dy, dz "x", "y" or "z" for direction of motion
            value: (float)
        """
        self.completion()
        

        start_joint = json.loads(self.robot.position(space = "joint"))  
        start_xyz = json.loads(self.robot.position(space = "xyz")) 
        input_xyz = [x,y,z]
        print(input_xyz)

        if movement == 0:
            for i in range(3):
                if input_xyz[i] == None:
                    input_xyz[i] =  start_xyz[i]
        elif movement == 1:
            for i in range(3):
                if input_xyz[i] == None:
                    input_xyz[i] =  0.
        else:
            print("Invalid movement parameter!")
            return None

        #print(input_xyz)
        motion = {"command" : "move" , "prm" : {"movement" :  movement ,"speed": speed,
             "path" : "line", "x" : input_xyz[0], "y" : input_xyz[1],"z" : input_xyz[2]}}


        self.play(motion)

        self.completion()

        end_joint = json.loads(self.robot.position(space = "joint"))

        base_twist = (start_joint[0] - end_joint[0])

        if base_twist != 0:
            
            line_up = {"command" : "move" , "prm" : {"movement" :  0, "path" : "joint", "j4" : - end_joint[0] ,"speed" : 2000 }}
            self.play(line_up)
            self.completion()
        return None

    def joint_v2(self, j0 = None, j1 = None, j2 = None, j3 = None, j4 = None, speed = 1500, movement = 1):

        """
        input:
            jx: (float) value along direction of motion
            value: (float) angle in degrees
            speed: (float) degrees/min
            movement: 1 for relative to current location, 0 for absolute cooridnates

        """
        self.completion()

        input_joint = [j0,j1,j2,j3,j4]
        start_joint = json.loads(self.robot.position(space = "joint"))
        print("joints: " + str(input_joint))

        if movement == 0:
            for i in range(5):
                if input_joint[i] == None:
                    input_joint[i] = start_joint[i]

        elif movement == 1:
            for i in range(5):
                if input_joint[i] == None:
                        input_joint[i] = 0.0
                
        else:
            print("Invalid movement parameter!")
            return None
        
        motion = {"command" : "move" , "prm" : {"movement" :  movement ,"speed": speed,
             "path" : "joint", "j0" :input_joint[0] , "j1" :input_joint[1], "j2" :input_joint[2], "j3" :input_joint[3], "j4" :input_joint[4]}}
        self.play(motion)

        self.completion()
        self.find_rel()

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

    def joint_motion(self,coord,value,speed = 1500 ,movement = 1):
        """
        Shorthand function to move robot joints
        input:
            
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

        self.completion()
        self.find_rel()
        
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
    
        command = "mkdir -p " + self.data_dir
        p= Popen(command, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        #logfile = open(data_dir+'logfile', 'a')

        #command = 'cam path ' + self.data_dir
        #p = Popen(command, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        #output = p.stdout.read()

        self.log_line('Data directory=' + self.data_dir + "\n")
        
        return None

    def expose(self,exptime, burst = 1):
        # Take images
        command = 'cam imno' 
        #p = Popen(command, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        imno1 = self.imno
        #self.logfile.write('\nimage%06d.fits' % int(imno1))
        command0 = 'cam burst='+str(burst)
        print(command0)
        # Run command
        #p = Popen(command0, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        # Write to the log file
        bpar = str(burst)
        #bpar = int(p.stdout.read().decode())
        #self.logfile.write('\nBurst param= %s' % str(bpar))
        #subprocess.call(command0,shell=True)
        
        # Set the exposure time
        command1 = 'cam exptime='+str(exptime)
        print(command1)
        # Run command 
        #p = Popen(command1, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        expout = "exptime"
        #expout = float(p.stdout.read().decode())
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
        #p = Popen(command3, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)

         # Write to the log file
        for i in range(burst):
            self.log_line('\nimage%06d.fits' % (imno1+i))
            self.log_line('\t%s' % str(expout))
            self.log_line('\t%s' % str(bpar))
            self.log_line('\t%s' % str(burst))
            self.log_line('\t%s' % str(self.rel_pos[0]))
            self.log_line('\t%s' % str(self.rel_pos[1]))
            self.log_line('\t%s' % str(self.rel_pos[2]))
            if self.laser_set:
                self.log_line('\t%s' % str(self.laser_read()))

            

        time.sleep(exptime*burst) 
        self.imno += burst
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
            
            print('Relative position [mm] %s' % str(self.rel_pos))

            if self.laser_set:
                ext_z = self.laser_read()



            self.expose(exptime,burst)

        print("Sweep complete, returning to start position")
        
        self.play(return_cmd)
        self.completion()
        return None

    def repeat_grid(self,gx = 5, gy = 5):
        """
        Grid motions to test repeatability
        """

        start_coord = json.loads(self.robot.position("xyz"))
        return_cmd = {"command": "move", "prm": {"movement": 0, "path": "line" , "xyz" : start_coord}}

        self.laser_read(set_target = True)

        
        print("moving in grid")

        for i in range(3):
            for j in range(3):

                self.xyz_v2(x = start_coord[0]+((i-2)*gx), y = start_coord[1]+((j-2)*gy), movement = 0)
                self.focus_test(steps = 5)
                self.z_correct()

        



        self.play(return_cmd)



        return None




















