# -*- coding: utf-8 -*-
"""
Created on Wed Feb  8 09:44:45 2017

@author: cch-student
"""

import os
import subprocess 
import sys 
import threading
import Queue
import commands
import time
import socket
import md5

class MltThrd(threading.Thread):
    def __init__(self, cmd, queue):
        threading.Thread.__init__(self)
        self.cmd = cmd
        self.queue = queue

    def run(self):
        # execute the command, queue the result
        subprocess.call(self.cmd, shell=True)        
        #(status, output) = commands.getstatusoutput(self.cmd)
        #self.queue.put((self.cmd, output, status))


def subprocess_cmd(command):
    process = subprocess.Popen(command,stdout=subprocess.PIPE, shell=True)
    proc_stdout = process.communicate()[0].strip()
    print proc_stdout

def md5checksum(str1,str2):
    m1 = md5.new()
    m2 = md5.new()
    m1.update(str1)
    m2.update(str2)
    if (m1.digest() == m2.digest()):
        return True
    else:
        return False

def createDocker(image,ip):
    print "Docker file does not exist. Creating docker file..."
    os.mkdir(image)
    tmppath = os.getcwd() + '/' + image
    print tmppath
    os.chdir(tmppath)
    file = open('Dockerfile','w')
    file.write("FROM ros:kinetic-robot\n")
    file.write("\n")
    file.write("#ros-kinetic-navigation\n")
    file.write("RUN apt-get update\n")
    file.write("RUN apt-get install -y \\ \n")
    file.write("\tros-kinetic-navigation \\ \n")
    file.write("\tros-kinetic-roslint \\ \n")
    file.write("\tros-kinetic-image* \\ \n")
    file.write("\tros-kinetic-teleop-twist-keyboard \\ \n")
    file.write("\tros-kinetic-gmapping\n")
    
    file.write("\n")
    file.write("EXPOSE 22 80\n")
    file.write("CMD export ROS_MASTER_URI=http://172.17.0.1:11311\n")
    file.write("CMD export ROS_IP=172.17.0.2")
    file.close()
    tarcode = "tar zcf Dockerfile.tar.gz Dockerfile"
    subprocess.call(tarcode,shell=True)
    buildcode = "curl -v -X POST -H " + '"Content-Type:application/tar"'+ " --data-binary '@Dockerfile.tar.gz' http://" + ip + ":2375/build?t=" + image + md5.new(image).hexdigest()
    subprocess.call(buildcode ,shell=True)       
#os.system("ls -l")
#subprocess.call(["ls", "-l"])
#os.chdir(r"/home/cch-student/temp123")
try:
    ip = sys.argv[1]
except:
    print "IP address not entered!exiting script"
    exit()
    
try:
    image = sys.argv[2]
except:
    print "Image name not entered!exiting script"
    exit()

dockercmd_img = "docker -H tcp://" + ip + ":2375 images"
#dockercmd = "ls -l"
subprocess.call(dockercmd_img, shell=True)
#image = raw_input("Enter docker image name from list:")
dockercmd_getimages = "curl -v http://" + ip + ":2375/images/search?term=" + image + " | jq '.[] | .name' >> tmpfile"
subprocess.call(dockercmd_getimages,shell=True)
pathtmpfile = os.path.abspath("tmpfile")
tmpfile = open(pathtmpfile,"r")
img_nm = tmpfile.read()
img_nm = img_nm[1:-2]
print img_nm
os.remove(pathtmpfile)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(('8.8.8.8', 0))
ros_ip = s.getsockname()[0]


dockercmd_runimg = "docker -H tcp://" + ip + ":2375 run -it --rm -u " + '"$(id -u):$(id -g)"' + " -v $HOME/.ros:/.ros -v $HOME/.config/catkin:/.config/catkin:ro -v $(pwd):$(pwd) -w $(pwd) " + image + md5.new(image).hexdigest()
docker_exec_str = "docker -H tcp://" + ip + ":2375 " + "exec $(docker -H tcp://" + ip + ":2375 ps -q) "
dockercmd_getconid = docker_exec_str + "ls"
dockercmd_rosmaster = docker_exec_str + "export ROS_MASTER_URI=http://" + ip + ":11311"
dockercmd_rosip = docker_exec_str + "export ROS_IP="+ros_ip
dockercmd_teleop = docker_exec_str + ""

var = md5checksum(image,img_nm)
if (var == False):
    createDocker(image,ip)
    
cmds = [dockercmd_runimg]
#cmds.append(dockercmd_getconid)
#cmds.append(dockercmd_rosmaster)
#cmds.append(dockercmd_rosip)
#print cmds[0]
try:
    #subprocess_cmd('docker -H tcp://'+ip+':2375 run -it --rm -u "$(id -u):$(id -g)" -v $HOME/.ros:/.ros -v $HOME/.config/catkin:/.config/catkin:ro -v $(pwd):$(pwd) -w $(pwd) '+image)
    #process = subprocess.Popen('/bin/bash', stdin=subprocess.PIPE, stdout=subprocess.PIPE,shell=True)
    #out, err = process.communicate(commands)    
    result_queue = Queue.Queue()   
    #subprocess.call(cmds[0], shell=True)
    for cmd in cmds:
        thread = MltThrd(cmd,result_queue)
        thread.start()
        time.sleep(2)
        
except:
    
    exit()
    
#subprocess.call(cmds[1], shell=True)
