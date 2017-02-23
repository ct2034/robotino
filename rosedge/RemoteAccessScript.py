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

def createDockerImg(image,ip,rospackage):
    
    # reading base docker commands    
    print "Docker file does not exist. Creating docker file..."
    base_path = os.path.abspath("basedocker") + "/Dockerfile"
    rd_base = open(base_path,"r")
    contents = rd_base.readlines()
    rd_base.close()
    
    # reading docker file of specific launch files from ros packages
    ros_cmd = "rospack find " + rospackage + " >> tmpfile"
    subprocess.call(ros_cmd,shell=True)
    pathtmpfile = os.path.abspath("tmpfile")
    tmpfile = open(pathtmpfile,"r")
    pkg_path = tmpfile.read()
    tmpfile.close()
    pkg_path = pkg_path[:-1]
    os.remove(pathtmpfile)
    pkg_path = pkg_path + "/Dockerfile/Dockerfile"
    print "!!!!!!!!!!"    
    print pkg_path
    dock_read = open(pkg_path,"r")
    tmp_contents = dock_read.readlines()
    dock_read.close()
    idx = 9
    for i in range (len(tmp_contents)):
        contents.insert(idx,tmp_contents[i])
        idx = idx + 1
    
    
    #pkgdocker = op
    os.mkdir(rospackage+"_docker")
    tmppath = os.getcwd() + '/' + rospackage+"_docker"
    print tmppath
    os.chdir(tmppath)
    
    file = open('Dockerfile','w')
    contents = "".join(contents)
    file.write(contents)
    file.close()
    tarcode = "tar zcf Dockerfile.tar.gz Dockerfile"
    subprocess.call(tarcode,shell=True)
    buildcode = "curl -v -X POST -H " + '"Content-Type:application/tar"'+ " --data-binary '@Dockerfile.tar.gz' http://" + ip + ":2375/build?t=" + image
    subprocess.call(buildcode ,shell=True)       

def createDockerContainer(image,ip):
    
    cmd = "curl -XPOST -H " + '"Content-Type: application/json"' + "http://" + ip + ":2375/containers/create?name=" + image + "_cont -d '"
    container_str1 = '{"Hostname":"3e93a4b05cf6",\
                        "Domainname":"","User":"1000:1000" \
                        ,"AttachStdin":true,"AttachStdout":true, \
                        "AttachStderr":true,"Tty":true,"OpenStdin":true,\
                        "StdinOnce":true,"Env":["PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",\
                        "LANG=en_US.UTF-8","ROS_DISTRO=kinetic"],"Cmd":["bash"],"Image":'
    
    container_str2 = '"Volumes":null,"WorkingDir":"/home/cch-student"\
                        ,"Entrypoint":["/ros_entrypoint.sh"],"OnBuild":null, \
                        "Labels":{}},"NetworkSettings":{"Bridge":"",\
                        "SandboxID":"7905e2dfc3d956cb1666df0abd9fea98ad1d3608d5b2e864cb3e355488b1d8bb",\
                        "HairpinMode":false,"LinkLocalIPv6Address":"","LinkLocalIPv6PrefixLen":0,\
                        "Ports":null,"SandboxKey":"/var/run/docker/netns/7905e2dfc3d9",\
                        "SecondaryIPAddresses":null,"SecondaryIPv6Addresses":null,\
                        "EndpointID":"","Gateway":"","GlobalIPv6Address":"","GlobalIPv6PrefixLen":0,\
                        "IPAddress":"","IPPrefixLen":0,"IPv6Gateway":"","MacAddress":"",\
                        "Networks":{"bridge":{"IPAMConfig":null,"Links":null,"Aliases":null,\
                        "NetworkID":"fec89e99f0ea2921ddc66d23edbc98d7ce7fecdb74ba512faec6a8450efeb036",\
                        "EndpointID":"","Gateway":"","IPAddress":"","IPPrefixLen":0,"IPv6Gateway":"",\
                        "GlobalIPv6Address":"","GlobalIPv6PrefixLen":0,"MacAddress":""}}}}'
    
    container_id = " > contmpfile"                
    dockercontainer_cmd = cmd + container_str1 + image + container_str2 + container_id
    print dockercontainer_cmd
    subprocess.call(dockercontainer_cmd,shell=True)
    pathtmpfile = os.path.abspath("contmpfile")
    tmpfile = open(pathtmpfile,"r")
    cont_id = tmpfile.read()
    os.remove(pathtmpfile)
    dockercontainerstart_cmd = "curl -XPOST http://" + ip + ":2375/containers/" +  cont_id + "/start"
    subprocess.call(dockercontainerstart_cmd,shell=True)
                                 


try:
    ip = sys.argv[1]
except:
    print "IP address not entered!exiting script"
    exit()
    
try:
    rospackage = sys.argv[2]
except:
    print "Ros package name not entered!exiting script"
    exit()
    
try:
    roslaunchfile = sys.argv[3]
except:
    print "Ros launch file name not entered!exiting script"
    exit()

dockercmd_img = "docker -H tcp://" + ip + ":2375 images"
subprocess.call(dockercmd_img, shell=True)
#image = raw_input("Enter docker image name from list:")
image = rospackage + "_dockerfile"
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


dockercmd_runimg = "docker -H tcp://" + ip + ":2375 run -it --rm -u " + '"$(id -u):$(id -g)"' + " -v $HOME/.ros:/.ros -v $HOME/.config/catkin:/.config/catkin:ro -v $(pwd):$(pwd) -w $(pwd) " + image
docker_exec_str = "docker -H tcp://" + ip + ":2375 " + "exec $(docker -H tcp://" + ip + ":2375 ps -q) "
dockercmd_getconid = docker_exec_str + "ls"
dockercmd_rosmaster = docker_exec_str + "export ROS_MASTER_URI=http://" + ip + ":11311"
dockercmd_rosip = docker_exec_str + "export ROS_IP="+ros_ip
dockercmd_teleop = docker_exec_str + ""

var = md5checksum(image,img_nm)
if (var == False):
    createDockerImg(image,ip,rospackage)
    createDockerContainer(image,ip)
    
cmds = [dockercmd_runimg]

try:
    
    result_queue = Queue.Queue()   
    #subprocess.call(cmds[0], shell=True)
    for cmd in cmds:
        thread = MltThrd(cmd,result_queue)
        thread.start()
        time.sleep(2)
        
except:
    
    exit()
