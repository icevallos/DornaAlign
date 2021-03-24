#!/usr/bin/python

import os.path
import os, sys
# import numpy as np
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
    Moves the fiber in x y z directions, corrects for horizontal angle in y motions
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

def focus_test(robot, logfile, steps=10, dz = 0.5 , exptime = 1, burst =1):

    print("Sweeping focus on Z")

    start_coord = json.loads(robot.position("xyz"))
    return_cmd = {"command": "move", "prm": {"movement": 0, "path": "line" , "xyz" : start_coord}}


    for i in range(steps):
        
        
        z_cmd = {"command" : "move" , "prm" : {"movement" :  0 , "path" : "line", "z" : start_coord[2]+(dz*i)}}
        robot.play(z_cmd)
        completion(robot)
        print("z =  " + str(start_coord[2]+dz))
        print(robot.position("xyz"))
        logfile.write('Robot position %s' % str(json.loads(robot.position("xyz"))))
        Spectro_align_test(exptime,logfile,burst)

    print("Sweep complete, returning to start position")
    
    robot.play(return_cmd)
    robot.completion()
    return None


def log_setup():
    
    # Need to change the data directory
    data_dir = '/home/fireball2/SpectroAlign/' + datetime.now().strftime("%y%m%d") +'/'
    logfile = open(data_dir+'logfile', 'a')
    
    command = 'cam path ' + data_dir
    p = Popen(command, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
    output = p.stdout.read()
    logfile.write('Data directory=' + output)
    

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
    angle_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "a" : -90, "b": 0}}
    start_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "j0" : 0 , "j1" : 80., "j2" : -120.}}
    
    robot.play(rest_cmd)
    robot.play({"command" : "move", "prm":{"movement" : 0, "path": "joint", "j0" : j0_offset }})
    completion(robot)
    robot.set_joint({"j0" : 0 })

    return None




# -----------------------------------------------------------------------------
# The main function starts here
# -----------------------------------------------------------------------------
if __name__ == "__main__":

    #expose(1)
    print("Loading Dorna commands")
    logfile = log_setup()
    

    rest_cmd  = {"command" : "move", "prm":{"movement" : 0, "path": "joint", "j0" : 0. , "j1" : 145., "j2" : -90, "j3" : 0.0 , "j4" : 0.0}} 
    angle_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "a" : -90, "b": 0}}
    start_cmd = {"command" : "move" , "prm" : {"movement" : 0 , "path" : "joint", "j0" : 0 , "j1" : 80., "j2" : -120.}}
    vertical_cmd = {"command" : "move", "prm":{"movement" : 0, "path": "joint", "j0" : 0. , "j1" : 90., "j2" : 0., "j3" : 0.0 , "j4" : 0.0}}
    reset_j0 = {"command" : "move", "prm":{"movement" : 0, "path": "joint", "j0" : 0. }}






