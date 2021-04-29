#!/usr/bin/python

import os.path
import os, sys
import json
import subprocess
from subprocess import Popen, PIPE, STDOUT
import sys
import time
from datetime import datetime,timedelta



def completion(robot):


    status = robot.device()
    parsed_status = json.loads(status)

    while parsed_status['state'] != 0:
        time.sleep(1)
        status = robot.device()
        parsed_status = json.loads(status)

    #print("Command is done. On to the next!")

    return None

def xyz_motion(robot,coord,value):

    """
    Moves the fiber in x y z directions
    input:
        robot: (Dorna object)
        coord: (string) "x", "y" or "z" for direction of motion
        value: (float)
    """
    completion(robot)
    if coord == "x":
        grid_x = {"command" : "move" , "prm" : {"movement" :  1 , "path" : "line", "x" : value}}
        robot.play(grid_x)
    elif coord  == "z":
        grid_z = {"command" : "move" , "prm" : {"movement" :  1 , "path" : "line", "z" : value}}
        robot.play(grid_z)
    elif coord == "y":
        grid_y = {"command" : "move" , "prm" : {"movement" :  1 , "path" : "line", "y" : value}}
        robot.play(grid_y)

    else:
        print("Invalid coordinate command")

    
    return None

def joint_motion(robot,coord,value, movement = 1):
    """
    Shorthand function to move robot joints
    input:
        robot: (Dorna object)
        coord: (string) "j0", "j1", "j2", "j3" or "j4" for direction of motion
        value: (float) angle in degrees
        movement: 1 for relative, 0 for absolute
    """
    completion(robot)
    if coord == "j0":
        cmd = {"command" : "move" , "prm" : {"movement" :  movement , "path" : "joint", "j0" : value}}
        robot.play(cmd)
    elif coord  == "j1":
        cmd = {"command" : "move" , "prm" : {"movement" :  movement , "path" : "joint", "j1" : value}}
        robot.play(cmd)
    elif coord == "j2":
        cmd = {"command" : "move" , "prm" : {"movement" :  movement, "path" : "joint", "j2" : value}}
        robot.play(cmd)
    elif coord == "j3":
        cmd = {"command" : "move" , "prm" : {"movement" :  movement, "path" : "joint", "j3" : value}}
        robot.play(cmd)
    elif coord == "j4":
        cmd = {"command" : "move" , "prm" : {"movement" :  movement, "path" : "joint", "j4" : value}}
        robot.play(cmd)


    else:
        print("Invalid coordinate command")

    
    return None



def expose(exptime, logfile, burst = 1):
    # Take images
    command = 'cam imno' 
    p = Popen(command, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
    output = p.stdout.read()
    logfile.write('\nimage%06d.fits' % int(output))
    command0 = 'cam burst='+str(burst)
    print(command0)
    # Run command
    p = Popen(command0, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
    # Write to the log file
    output = p.stdout.read()
    logfile.write('Burst param= %s' % str(output))
    #subprocess.call(command0,shell=True)
    
    # Set the exposure time
    command1 = 'cam exptime='+str(exptime)
    print(command1)
    # Run command 
    p = Popen(command1, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
    # Write to the log file
    output = p.stdout.read()
    logfile.write('Exposure time= %s' % str(output))
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
    #subprocess.call(command3,shell=True)

def focus_test(robot, logfile, steps=10, dz = 0.5 , exptime = 1., burst =1, x_offset = 0., y_offset = 0.):
    """
    Takes through focus image bursts as the robot moves upwards in z.

    input:
        robot: (Dorna object)
        logfile: text file
        steps: (int) number changes in position
        dz: (float) separation between steps in mm
        exptime =  (float) exposure in seconds
        burst = (int) number of images per exposure
        x_offset = (float) displacement from original x position in mm
        y_offset = (float) displacement from original y position in mm


    """


    print("Sweeping focus on Z")

    start_coord = json.loads(robot.position("xyz"))
    return_cmd = {"command": "move", "prm": {"movement": 0, "path": "line" , "xyz" : start_coord}}

    if x_offset != 0.:
        xyz_motion(robot, "x", x_offset)
    if y_offset != 0.:
        xyz_motion(robot,"y", y_offset)


    for i in range(steps):
        
        
        z_cmd = {"command" : "move" , "prm" : {"movement" :  0 , "path" : "line", "z" : start_coord[2]+(dz*i)}}
        robot.play(z_cmd)
        completion(robot)
        print("z =  " + str(start_coord[2]+dz))
        print(robot.position("xyz"))
        logfile.write('Robot position %s' % str(json.loads(robot.position("xyz"))))
        expose(exptime,logfile,burst)

    print("Sweep complete, returning to start position")
    
    robot.play(return_cmd)
    robot.completion()
    return None


def log_setup():

    """
    Creates log file for image taking, sets up camserver image path
    """
    
    # Need to change the data directory
    data_dir = '/home/fireball2/SpectroAlign/' + datetime.now().strftime("%y%m%d") +'/'
    
    command = "mkdir -p " + data_dir
    p= Popen(command, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
    logfile = open(data_dir+'logfile', 'a')

    command = 'cam path ' + data_dir
    p = Popen(command, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
    output = p.stdout.read()
    logfile.write('Data directory=' + data_dir)
    

    # Exposure time is the parameter that we can choose. It should be longer than 1s
    #exptime = 1
    
    # Emgain is 0 for now.
    # emgain = 0

    # for em in emgain:
    # for et in exptime: 
    # Return the current image number?
    


    return logfile
        
    #expose(exptime,burst,logfile)

def ax_rotation(robot,j0_offset):

    """rotates around z and redefines x, y coordinates of the robot.
    j0 offset: degrees in counterclockwise direction looking from above. 

    """

    rest_cmd  = {"command" : "move", "prm":{"movement" : 0, "path": "joint", "j0" : 0. , "j1" : 145., "j2" : -90, "j3" : 0.0 }}
    
    
    robot.play(rest_cmd)
    robot.play({"command" : "move", "prm":{"movement" : 0, "path": "joint", "j0" : j0_offset }})
    completion(robot)
    robot.set_joint({"j0" : 0 })

    return None

def record_pos(robot):
    """
    records xyz position as list
    """

    position = json.loads(robot.position("xyz"))
    print("x,y,z,a,b =" + str(position))

    return position



# -----------------------------------------------------------------------------
# The main function starts here
# -----------------------------------------------------------------------------
if __name__ == "__main__":

    #expose(1)
    print("Loading Dorna commands")
    #logfile = log_setup()
    

    rest_cmd  = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed":500, "j0" : 0. , "j1" : 145., "j2" : -90, "j3" : 0.0 , "j4" : 0.0}} 
    angle_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "a" : -90, "b": 0}}
    start_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "j0" : -7 , "j1" : 50., "j2" : -50.}}
    vertical_cmd = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed": 500, "j0" : 0. , "j1" : 90., "j2" : 0., "j3" : 0.0 , "j4" : 0.0}}
    reset_j0 = {"command" : "move", "prm":{"movement" : 0, "path": "joint","speed":500 ,"j0" : 0. }}






