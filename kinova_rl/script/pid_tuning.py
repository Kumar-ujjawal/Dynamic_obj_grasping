#!/usr/bin/env python3

import rospy
from dynamic_reconfigure.server import Server
from kinova_rl.cfg import JacoPIDTuningConfig
from controller_manager_msgs.srv import ListControllers
from std_msgs.msg import Float64

class JacoPIDTuner:
    def __init__(self):
        rospy.init_node('jaco_pid_tuner', anonymous=True)
        
        # Wait for controller manager services
        rospy.wait_for_service('/j2s7s300/controller_manager/list_controllers')
        rospy.loginfo("Node started!")
        
        self.list_controllers = rospy.ServiceProxy('/j2s7s300/controller_manager/list_controllers', ListControllers)
        rospy.loginfo("Controllers service proxy created")
        
        # Initialize the dynamic reconfigure server
        self.dyn_reconfig_srv = Server(JacoPIDTuningConfig, self.reconfigure_callback)
        rospy.loginfo("Dynamic reconfigure server initialized")

    def reconfigure_callback(self, config, level):
        rospy.loginfo("Reconfigure callback triggered. Updating PID gains...")
        
        controllers = self.list_controllers().controller
        
        # Update PID gains for joint controllers
        for i in range(7):
            controller_name = f"joint_{i+1}_velocity_controller"
            self.update_gains(controller_name, 
                              config[f"joint{i+1}_p"], 
                              config[f"joint{i+1}_i"], 
                              config[f"joint{i+1}_d"])

        # Update PID gains for finger controllers (assuming 3 finger joints)
        for i in range(3):
            controller_name = f"finger_{i+1}_position_controller"
            self.update_gains(controller_name, 
                              config[f"finger{i+1}_p"], 
                              config[f"finger{i+1}_i"], 
                              config[f"finger{i+1}_d"])

        rospy.loginfo("PID gains updated")
        return config

    def update_gains(self, controller_name, p, i, d):
        param_name = f"/j2s7s300/{controller_name}/pid/set_parameters"
        rospy.set_param(param_name + "/p", p)
        rospy.set_param(param_name + "/i", i)
        rospy.set_param(param_name + "/d", d)
        rospy.loginfo(f"Updated gains for {controller_name}: P={p}, I={i}, D={d}")

if __name__ == '__main__':
    try:
        tuner = JacoPIDTuner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
