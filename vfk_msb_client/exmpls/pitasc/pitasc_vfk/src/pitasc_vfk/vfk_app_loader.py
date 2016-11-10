#! /usr/bin/env python2

import rospy

from std_msgs.msg import String, Empty
from pitasc_vfk.msg import AppList, AppDescription, LoadProgress


class PitascVFK(object):

    def __init__(self):
        
        ## This is a dirty 15 minutes GoF state pattern state machine
        self.states = {
            "selection":State_Selection(),
            "loading":State_Loading(),
            "running":State_Running()
        }

        self.current_state = None


    def run(self):
        ## Wait for topics
        rospy.sleep(1.0) # give the topics some time, TODO

        ## Set initial state
        self.set_state(self.states["selection"])

        ## Spin
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            ## Update
            self.current_state.on_update()

            ## Transition?
            if self.current_state.state == "done":
                self.set_state(self.states[self.current_state.next_state])

            rate.sleep()

    def set_state(self, state):
        """Switch states
        """
        if self.current_state is not None:
            self.current_state.on_exit()

        self.current_state = state
        self.current_state.on_enter()


class State_Selection(object):

    def __init__(self):
        self.state = "idle"
        
        self.pub_applist = rospy.Publisher('pitasc_app_list', AppList, queue_size=1)        
        self.sub_selection = rospy.Subscriber("pitasc_load_app", String, self.callback_load_app)

    def on_enter(self):
        rospy.loginfo("Start: Selection")
        self.next_state = ""
        self.state = "running"

        app1 = AppDescription(app_id="id1", app_name="my first app")
        app2 = AppDescription(app_id="id2", app_name="a second app")
        app_list = AppList(apps=[app1, app2])
        self.pub_applist.publish(app_list)

    def on_update(self):
        pass

    def on_exit(self):
        rospy.loginfo("Exit: Selection")
        self.state = "idle"

    def callback_load_app(self, msg):
        rospy.loginfo("Loading app: {}".format(msg.data))
        self.next_state = "loading"
        self.state = "done"


class State_Loading(object):

    def __init__(self):
        self.state = "idle"
        
        self.pub_progress = rospy.Publisher('pitasc_load_progress', LoadProgress, queue_size=10)
        self.sub_starting = rospy.Subscriber("pitasc_start_app", String, self.callback_start_app)

    def on_enter(self):
        rospy.loginfo("Start: Loading")
        self.next_state = ""
        self.state = "running"

        self.progress = LoadProgress(skill_names=["skill1", "skill2", "skill3"], progress_values=[0.0, 0.0, 0.0], done=False)

    def on_update(self):

        ## Fake progress
        self.progress.progress_values[0] = self.fake_increment(self.progress.progress_values[0], 0.1)
        self.progress.progress_values[1] = self.fake_increment(self.progress.progress_values[1], 0.05)
        self.progress.progress_values[2] = self.fake_increment(self.progress.progress_values[2], 0.02)

        ## Fake done
        if self.progress.progress_values[2] > 0.999:
            self.progress.done = True

        ## Send
        self.pub_progress.publish(self.progress)

    def on_exit(self):
        rospy.loginfo("Exit: Loading")
        self.state = "idle"

    def fake_increment(self, value, stepsize):
        value = value + stepsize
        if value > 1.0:
            value = 1.0
        return value

    def callback_start_app(self, msg):
        rospy.loginfo("Starting app: {}".format(msg.data))
        self.next_state = "running"
        self.state = "done"


class State_Running(object):

    def __init__(self):
        self.state = "idle"
        
        self.fake_timer = 0.0

    def on_enter(self):
        rospy.loginfo("Start: Running")
        self.next_state = ""
        self.state = "running"

    def on_update(self):
        self.fake_timer = self.fake_timer + 0.05
        if self.fake_timer > 1.0:
            rospy.loginfo("Application finished!")
            self.next_state = "selection"
            self.state = "done"

    def on_exit(self):
        rospy.loginfo("Exit: Running")
        self.state = "idle"


if __name__ == '__main__':
    
    rospy.init_node('pitasc_vfk_app_loader')
    
    vfk = PitascVFK()
    rospy.loginfo("Started pitasc vfk")
    vfk.run()

    rospy.loginfo("Shutting down")

# eof
