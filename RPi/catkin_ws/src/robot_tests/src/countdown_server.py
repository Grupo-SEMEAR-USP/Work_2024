#!/usr/bin/env python
import rospy
import actionlib
from robot_actions.msg import MoveToShelfAction, MoveToShelfFeedback, MoveToShelfResult

class MoveToShelfServer():
    def __init__(self):
        self._action_server = actionlib.SimpleActionServer("move_to_shelf", MoveToShelfAction, self.execute, False)
        self._action_server.start()

    def execute(self, goal):
        rospy.loginfo("Começando a se movimentar para ID: {0}\n".format(goal.shelf_id))
        feedback = MoveToShelfFeedback()
        result = MoveToShelfResult()

        if (goal.shelf_id == 1):
            dist = 3
        if (goal.shelf_id == 2):
            dist = 2
        if (goal.shelf_id == 3):
            dist = 5
        

        # Simulate movement with feedback
        for i in range(dist, 0, -1):
            rospy.sleep(1)  # Simulate time passing
            feedback.distance_to_shelf = float(i) * 10.0  # Simulate distance closing
            self._action_server.publish_feedback(feedback)
            rospy.loginfo("Movimentado... Distancia do alvo: {0}".format(feedback.distance_to_shelf))

        result.success = True  # Indicate success

        rospy.loginfo("Chegou ao destino!\n")

        self._action_server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('move_to_shelf_server')
    server = MoveToShelfServer()
    rospy.spin()
