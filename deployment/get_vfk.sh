#/bin/bash
rm -rf temp
rm -rf vfk_msb_client
mkdir -p temp
mkdir -p vfk_msb_client

sudo apt-get -q update

sudo apt-get -y install unzip python-pip wget

VERSION='1.4.0-SNAPSHOT'

#wget  -q -O temp/vfk-ros.zip https://repo.virtualfortknox.de/filerepository/vfk_public/non-java-client-libs/$VERSION/vfk.msb.client.library.websocket.ros-$VERSION.zip --user vfk_repo_public_read --password eNljCqWs
#unzip -q -o temp/vfk-ros.zip -d temp

wget  -q -O temp/vfk-python.zip https://repo.virtualfortknox.de/filerepository/vfk_public/non-java-client-libs/$VERSION/vfk.msb.client.library.websocket.python-$VERSION.zip --user vfk_repo_public_read --password eNljCqWs
unzip -q -o temp/vfk-python.zip -d temp

find temp/vfk.msb.client/ -type f -exec sed -i 's/\r//g' {} \;

#cp -r temp/vfk.msb.client/vfk.msb.client.library/vfk.msb.client.library.websocket.ros/vfk_msb_client .
cp -r temp/vfk.msb.client/vfk.msb.client.library/vfk.msb.client.library.websocket.python/* vfk_msb_client

#rm -rf temp

#chmod +x vfk_msb_client/scripts/msb_ros_bridge.py

#sudo pip install git+https://github.com/baalexander/rospy_message_converter.git
sudo pip install iso8601
sudo pip install ws4py

sudo bash -c 'echo "description \"running an msb client...\"
author \"cch\"

# When to start the service
start on runlevel [2345]

# When to stop the service
stop on runlevel [016]

# Automatically restart process if crashed
respawn

# Specify working directory
chdir $(pwd)

# Specify the process/command to start, e.g.
exec python $(pwd)/vfk_msb_client/example_main.py" > /etc/init/msb_service.conf'

sudo start msb_service