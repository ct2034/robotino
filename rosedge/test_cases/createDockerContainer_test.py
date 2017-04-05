# -*- coding: utf-8 -*-
"""
Created on Wed Apr  5 15:00:10 2017

@author: cch-student
"""
import subprocess
import docker

def createDockerContainer(image, ip, port, roscommand, rospackage, roslaunchfile, dockercmd_rosmaster, dockercmd_rosip):

    ip_str = "tcp://" + ip + ":" + port
    cli = docker.Client(base_url=ip_str)
    container = cli.create_container(image,
                                     hostname='3e93a4b05cf6',
                                     user='1000:1000',
                                     stdin_open=True,
                                     tty=True,
                                     entrypoint='/ros_entrypoint.sh',
                                     command=["bash"],
                                     environment=[
                                         "PATH/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"],
                                     working_dir='/',
                                     volumes='$HOME/.config/catkin:/.config/catkin:ro')
    cli.start(container['Id'])
    bash_cmd = "docker -H tcp://" + ip + ":" + port +\
        " exec -it " + container['Id'] + " /bin/bash"
    bash_cmd = '/bin/bash'
    print bash_cmd
    return container['Id']
    
def test_answer():
    
    assert createDockerContainer()