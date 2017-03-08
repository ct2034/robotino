FROM ros:kinetic-robot

#ros-kinetic-navigation
RUN apt-get update && \
	apt-get install -y \ 
	ros-kinetic-navigation \ 
	ros-kinetic-roslint \ 
	ros-kinetic-image* 



EXPOSE 22 80

ADD ./ros-entrypoint.sh /

ENTRYPOINT ["/ros-entrypoint.sh"]
CMD [ "rosrun", "teleop_twist_keyboard teleop_twist_keyboard.py" ]

