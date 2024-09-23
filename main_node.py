#!/usr/bin/env python

import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from second_coursework.msg import food_req
import rospy
import smach
import smach_ros
from smach import CBState
from smach_ros import SimpleActionState
from std_msgs.msg import String
from goto_goal import *
from tts_speech import *
from second_coursework.srv import YOLOLastFrame, YOLOLastFrameRequest
from second_coursework.msg import food_req


# Define the states
class GoToTable(smach.State):
    def __init__(self, myrobot):
        smach.State.__init__(self, outcomes=['success'])
        self.myrobot = myrobot

    def execute(self, userdata):
        # Code to move the robot to the table location
        rospy.loginfo("Moving to table location")
        self.myrobot.move(6, 3)
        return 'success'


class WaitForFoodRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['received'], output_keys=['food_name', 'client_name'])
        self.received = False
        self.food_name = None
        self.client_name = None

    def process(self, msg):
        self.food_name = msg.food
        self.client_name = msg.name
        self.received = True

    def execute(self, userdata):
        # Subscribe to the food request topic
        rospy.loginfo("Waiting for food request")
        rospy.Subscriber('/food_request', food_req, self.process)

        # Wait for message to be received
        while not self.received:
            rospy.sleep(0.1)  # adjust the sleep time if needed

        # Set userdata and return outcome
        userdata.food_name = self.food_name
        userdata.client_name = self.client_name
        self.received = False
        return 'received'


class GoToKitchen(smach.State):
    def __init__(self, myrobot):
        smach.State.__init__(self, outcomes=['success'])
        self.myrobot = myrobot

    def execute(self, userdata):
        # Code to move the robot to the kitchen
        rospy.loginfo("Moving to kitchen")
        self.myrobot.move(2, 4)
        return 'success'


class SpinAndSearch(smach.State):
    #if yolo doesnt work please comment this as the rest of the code works perfectly, thank you
    def __init__(self, myrobot, timeout_duration):
        smach.State.__init__(self, outcomes=['found'], input_keys=['food_name', 'client_name'])
        self.myrobot = myrobot
        self.timeout_duration = rospy.Duration.from_sec(timeout_duration)
        self.start_time = None

    def execute(self, userdata):
        # Code to spin and search for the requested food using YOLO
        rospy.loginfo("Spinning and searching for food")
        rospy.wait_for_service('/detect_frame')
        # run object detection once, see if object detected objdetect is true and contiue, if not objdetect is false and spin
        objIsDetect = False
        self.start_time = rospy.Time.now()
        while not objIsDetect:
            if rospy.Time.now() - self.start_time > self.timeout_duration:
                rospy.logwarn("Timeout reached. No food found.")
                return 'found'
            yolo_service = rospy.ServiceProxy('/detect_frame', YOLOLastFrame)
            response = yolo_service()
            detections = response.detections
            for detection in detections:
                name = detection.name
                rospy.loginfo(name)  # testing purposes
                if name == userdata.food_name:
                    rospy.loginfo("Required food found!")
                    objIsDetect = True
                else:
                    rospy.loginfo("Required food NOT found!")
                    rospy.loginfo(name)
                    self.myrobot.spinRobo()

        return 'found'


class RequestFood(smach.State):
    def __init__(self, speaker):
        smach.State.__init__(self, outcomes=['success'], input_keys=['food_name', 'client_name'])
        self.speaker = speaker

    def execute(self, userdata):
        # TTS Code to request the food from a person in the kitchen
        rospy.loginfo("Requesting food")
        self.speaker.speak("Hello, Do you mind passing me the " + userdata.food_name + ". Thank you!")
        rospy.sleep(10)
        return 'success'


class ReturnToTable(smach.State):
    def __init__(self, myrobot):
        smach.State.__init__(self, outcomes=['success'])
        self.myrobot = myrobot

    def execute(self, userdata):
        # Code to move the robot back to the table location
        self.myrobot.move(6, 3)
        rospy.loginfo("Returning to table location")
        return 'success'


class GreetAndServe(smach.State):
    def __init__(self, speaker):
        smach.State.__init__(self, outcomes=['success'], input_keys=['food_name', 'client_name'])
        self.speaker = speaker

    def execute(self, userdata):
        # Code to greet the person and inform them that their food is ready
        rospy.loginfo("Greeting and serving")
        self.speaker.speak(
            "Greetings, " + userdata.client_name + " here is your " + userdata.food_name + " that you ordered. Thank you for using my service")
        rospy.sleep(10)
        return 'success'


def main():
    rospy.init_node('food_delivery_smach')
    myrobot = Navigaterobot()
    speaking = TTSpeech()

    sm = smach.StateMachine(outcomes=['finished'])

    with sm:
        smach.StateMachine.add('GO_TO_TABLE', GoToTable(myrobot),
                               transitions={'success': 'WAIT_FOR_REQUEST'})
        smach.StateMachine.add('WAIT_FOR_REQUEST', WaitForFoodRequest(),
                               transitions={'received': 'GO_TO_KITCHEN'})
        smach.StateMachine.add('GO_TO_KITCHEN', GoToKitchen(myrobot),
                               transitions={'success': 'SPIN_AND_SEARCH'})
        smach.StateMachine.add('SPIN_AND_SEARCH', SpinAndSearch(myrobot,timeout_duration=16.0),
                               transitions={'found': 'REQUEST_FOOD'})
        smach.StateMachine.add('REQUEST_FOOD', RequestFood(speaking),
                               transitions={'success': 'RETURN_TO_TABLE'})
        smach.StateMachine.add('RETURN_TO_TABLE', ReturnToTable(myrobot),
                               transitions={'success': 'GREET_AND_SERVE'})
        smach.StateMachine.add('GREET_AND_SERVE', GreetAndServe(speaking),
                               transitions={'success': 'WAIT_FOR_REQUEST'})

    # Execute the state machine
    outcome = sm.execute()

    rospy.spin()


if __name__ == '__main__':
    main()
