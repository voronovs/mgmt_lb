import sys
sys.path.append('C:\\opt\\ros\\melodic\\x64\\lib')
sys.path.append('C:\\opt\\ros\\melodic\\x64\\lib\\site-packages')
sys.path.append('C:\\opt\\ros\\melodic\\x64\\catkin_ws\\devel\\lib\\site-packages')

import random
import time
INSTALLED = False
try:
    import rospy
    import actionlib
    import cw_main.msg
    INSTALLED = True
except:
    pass

class DiagnosticProxy():
    
    def __init__(self, callback_fun=None):
        self.installed = INSTALLED
        if not INSTALLED:
            print('No CW_ROS ENVIRONMENT INSTALLED!')
            return
        self.node_name = 'cw_diagnostic_proxy{}'.format(random.randint(0, 2147483647))
        node = rospy.init_node(self.node_name)
        self.callback_fun = callback_fun
        self.client = actionlib.SimpleActionClient('/cw/robot', cw_main.msg.cmdAction)
        self.done = False
        # indicates if actionserver is exist and ready to process command
        self.ready = self.client.wait_for_server(timeout=rospy.Duration(15))
        print('Connected to server: {}'.format(self.ready))
       
    def do_diagnostic(self):
        if not INSTALLED:
            return False
        if not self.ready:
            print('Not connected to ActionServer!')
            return False
        goal = cw_main.msg.cmdGoal(['robot'], 'do_diagnostic', [], 1200)
        self.client.send_goal(goal, active_cb=self.callback_active,
                               feedback_cb=self.callback_feedback,
                               done_cb=self.callback_done)
        return True
        
    def halt(self):
        if not INSTALLED:
            return False
        if not self.ready:
            print('Not connected to ActionServer!')
            return False
        goal = cw_main.msg.cmdGoal(['robot'], 'halt', [], 10)
        self.client.send_goal(goal, active_cb=self.callback_active,
                               feedback_cb=self.callback_feedback,
                               done_cb=self.callback_done)
        return True
        
    def interrupt(self):
        self.client.cancel_goal()
        self.halt()
        self.kill()
    
    def callback_done(self, state, result):  # called when action is done
        self.done = True
        if self.callback_fun != None:
            self.callback_fun()
        self.kill()
        return True
            
    def callback_active(self):
        pass
        
    def callback_feedback(self, feedback):
        pass
        
    def kill(self):
        rosnode.kill_nodes([self.node_name])