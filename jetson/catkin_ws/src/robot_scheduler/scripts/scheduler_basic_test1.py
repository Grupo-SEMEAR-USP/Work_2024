#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import Int32, String
from robot_arm_control.msg import MoveToPoseCommand

interest_blocks = ["1", "2"]
counter = 0

class Task1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing Task 1: Sleep for 3 seconds')
        time.sleep(3)
        return 'completed'

class Task2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'failed'])
        self.pub_align = rospy.Publisher('/align_table', String, queue_size=10)
        self.feedback_received = None
        self.subscriber = None

    def feedback_cb(self, data):
        global active_task
        if active_task != 'TASK2':
            return
        rospy.loginfo(f"Feedback recebido Task2: {data.data}")
        self.feedback_received = data.data

    def execute(self, userdata):
        global active_task
        rospy.loginfo('Executing Task 2: Publicar "start" no tópico /align_table')
        active_task = 'TASK2'
        self.pub_align.publish("start")

        self.feedback_received = None
        self.subscriber = rospy.Subscriber('/align_table/feedback', Int32, self.feedback_cb)

        timeout = rospy.Time.now() + rospy.Duration(10)
        while self.feedback_received is None and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        if self.subscriber:
            self.subscriber.unregister()

        active_task = None

        if self.feedback_received == 1:
            return 'completed'
        else:
            return 'failed'

class Task3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'failed'])
        self.pub_align = rospy.Publisher('/move_time', String, queue_size=10)
        self.feedback_received = None
        self.subscriber = None

    def feedback_cb(self, data):
        global active_task
        if active_task != 'TASK3':
            return
        rospy.loginfo(f"Feedback recebido Task3: {data.data}")
        self.feedback_received = data.data

    def execute(self, userdata):
        global active_task
        rospy.loginfo('Executing Task 3: Publicar "pra ir para frente" no tópico /move_time')
        active_task = 'TASK3'
        self.pub_align.publish("frente,1.5")
        self.feedback_received = None
        self.subscriber = rospy.Subscriber('/move_time/feedback', Int32, self.feedback_cb)

        timeout = rospy.Time.now() + rospy.Duration(10)
        while self.feedback_received not in [0, 1] and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        if self.subscriber:
            self.subscriber.unregister()

        active_task = None

        if self.feedback_received == 1:
            return 'completed'
        else:
            return 'failed'

class Task4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'failed'])
        self.pub_align = rospy.Publisher('/edge_detect', String, queue_size=10)
        self.feedback_received = None
        self.subscriber = None

    def feedback_cb(self, data):
        global active_task
        if active_task != 'TASK4':
            return
        rospy.loginfo(f"Feedback recebido Task4: {data.data}")
        self.feedback_received = data.data

    def execute(self, userdata):
        global active_task
        rospy.loginfo('Executing Task 4: Publicar "alinha a borda da mesa" no tópico /edge_detect')
        active_task = 'TASK4'
        time.sleep(1)
        self.pub_align.publish("start")
        self.feedback_received = None
        self.subscriber = rospy.Subscriber('/edge_detect/feedback', Int32, self.feedback_cb)

        timeout = rospy.Time.now() + rospy.Duration(15)
        while self.feedback_received is None and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        if self.subscriber:
            self.subscriber.unregister()

        active_task = None

        if self.feedback_received == 1:
            return 'completed'
        else:
            return 'failed'

class Task5(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'failed'])
        self.pub_align = rospy.Publisher('/search_block', String, queue_size=10)
        self.feedback_received = None
        self.subscriber = None

    def feedback_cb(self, data):
        global active_task
        if active_task != 'TASK5':
            return
        rospy.loginfo(f"Feedback recebido Task5: {data.data}")
        self.feedback_received = data.data

    def execute(self, userdata):
        global active_task, counter, interest_blocks
        rospy.loginfo(f"Executing Task 5: Procurando o bloco {interest_blocks[counter]}")

        time.sleep(1)
        active_task = 'TASK5'

        self.pub_align.publish("start," + interest_blocks[counter])
        rospy.loginfo(f"Publicando busca para o bloco: {interest_blocks[counter]}")

        self.feedback_received = None
        self.subscriber = rospy.Subscriber('/search_block/feedback', Int32, self.feedback_cb)

        timeout = rospy.Time.now() + rospy.Duration(30)
        while self.feedback_received not in [0, 1] and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        if self.subscriber:
            self.subscriber.unregister()

        active_task = None

        if self.feedback_received == 1:
            return 'completed'
        else:
            return 'failed'

class Task6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['completed', 'failed'])
        self.pub_align = rospy.Publisher('/block_align', String, queue_size=10)
        self.feedback_received = None
        self.subscriber = None

    def feedback_cb(self, data):
        global active_task
        if active_task != 'TASK6':
            return
        rospy.loginfo(f"Feedback recebido Task6: {data.data}")
        self.feedback_received = data.data

    def execute(self, userdata):
        global active_task, counter, interest_blocks
        rospy.loginfo(f"Executing Task 6: Alinhando ao bloco {interest_blocks[counter]}")

        time.sleep(1)
        active_task = 'TASK6'

        self.pub_align.publish(interest_blocks[counter])
        rospy.loginfo(f"Publicando alinhar com o bloco: {interest_blocks[counter]}")

        counter += 1

        self.feedback_received = None
        self.subscriber = rospy.Subscriber('/block_align/feedback', Int32, self.feedback_cb)

        timeout = rospy.Time.now() + rospy.Duration(10)
        while self.feedback_received not in [0, 1] and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        if self.subscriber:
            self.subscriber.unregister()

        active_task = None

        if self.feedback_received == 1:
            return 'completed'
        else:
            counter -= 1
            return 'failed'


def main():
    rospy.init_node('scheduler_node')

    sm = smach.StateMachine(outcomes=['all_tasks_completed', 'all_tasks_failed'])
    sm.userdata.blocks_searched = 0

    with sm:
        smach.StateMachine.add('TASK1', Task1(), 
                               transitions={'completed': 'TASK3', 'failed': 'all_tasks_failed'})
        
        smach.StateMachine.add('TASK2', Task2(), 
                               transitions={'completed': 'TASK4', 'failed': 'all_tasks_failed'})

        smach.StateMachine.add('TASK3', Task3(), 
                               transitions={'completed': 'TASK4', 'failed': 'all_tasks_failed'})

        smach.StateMachine.add('TASK4', Task4(), 
                               transitions={'completed': 'TASK5', 'failed': 'all_tasks_failed'})

        smach.StateMachine.add('TASK5', Task5(), 
                               transitions={'completed': 'TASK6', 'failed': 'all_tasks_failed'})

        smach.StateMachine.add('TASK6', Task6(), 
                               transitions={'completed': 'CHECK_BLOCK_COUNT', 'failed': 'TASK6'})
        
        def check_block_count(userdata):
            time.sleep(1) 
            if userdata.blocks_searched < len(interest_blocks) - 1:
                userdata.blocks_searched += 1
                return 'search_more'
            else:
                return 'completed'

        smach.StateMachine.add('CHECK_BLOCK_COUNT', smach.CBState(check_block_count, 
                                                                  outcomes=['search_more', 'completed'], 
                                                                  input_keys=['blocks_searched'], 
                                                                  output_keys=['blocks_searched']),
                               transitions={'search_more': 'TASK2',
                                            'completed': 'all_tasks_completed'})

    outcome = sm.execute()
    exit()

if __name__ == '__main__':
    main()
    rospy.spin()
