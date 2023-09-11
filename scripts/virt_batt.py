#!/usr/bin/env python3 

import rospy 
from std_msgs.msg import Bool 
from sensor_msgs.msg import BatteryState
from virtual_battery_v2.msg import xyz
from vb_service.srv  import CallTopic, CallTopicResponse 

class VBNode:
    def __init__(self):
        self.battery_msg = BatteryState()
        self.battery_msg.present = True 
        self.battery_level = 60
        self.charging = False
        
        #Node
        rospy.init_node("virtual_battery_v2")
        print("virtual_battery_v2 initialized!")
        

        #Publisher & Subscriber
        self.battery_pub = rospy.Publisher("/batt_state", BatteryState, queue_size=10)
        self.status_pub = rospy.Publisher("/status_pub", xyz, queue_size=10)
        self.battery_sub = rospy.Subscriber("/virtual_charging", Bool, self.updateCb)


        #Parameters
        self.increase_rate = rospy.get_param(rospy.search_param('increase_rate'), default=1.0) 
        self.decrease_rate = rospy.get_param(rospy.search_param('decrease_rate'), default=1.0) 


        #Timers
        self.decreaser = rospy.Timer(rospy.Duration(int(self.decrease_rate)), self.decreaseTimerCb)
        self.increaser = rospy.Timer(rospy.Duration(int(self.increase_rate)), self.increaseTimerCb)                
        self.publishTimer = rospy.Timer(rospy.Duration(0.1), self.publishTimerCb) 


        self.increaseTimerCbCalled = False
        self.decreaseTimerCbCalled = False


    # Timer Callbacks
    def increaseTimerCb(self, event):
        self.increaseTimerCbCalled = True
        if self.charging:
            if self.battery_level < 100:
                self.battery_level += 1
                print(self.battery_level)
        

    def decreaseTimerCb(self, event):
        self.decreaseTimerCbCalled = True
        if not self.charging:
            if self.battery_level > 0:                    
                self.battery_level -= 1
                print(self.battery_level)



    def publishTimerCb(self, event):
        self.publishBattState()
        self.publishMsg() 
        


    def publishMsg(self):
        msg = xyz()
        msg.increase_rate = self.increase_rate
        msg.decrease_rate = self.decrease_rate
        msg.current_batt_level = self.battery_level
        if self.increaseTimerCbCalled == True:
            msg.is_charging = True 
        elif self.decreaseTimerCbCalled == True:
            msg.is_charging = False
        else:
            msg.is_charging = None 
        self.status_pub.publish(msg)


    def publishBattState(self):
        self.battery_msg.percentage = self.battery_level


    # Timer callback that sent to Subscriber
    def updateCb(self, msg):        
        self.charging = msg.data
        # print(f"msg.data:{msg.data}")
        print(f"self.charging:{self.charging}")
        self.publishMsg()



def handler(req):
    """
    The parameter taken when the service is called is: "inc_dec" (meaning increase or decrease)
    Call the service as described below:
    rosservice cal /call_topic "inc_dec: true" or
    rosservice cal /call_topic "inc_dec: false"
    """
    print(f"req.inc_dec:{req.inc_dec}")
    response = CallTopicResponse()

    if req.inc_dec:
        bttry.charging = True

    elif req.inc_dec == False:
        bttry.charging = False 

    else:
        print("inc_dec parameter is unrecognized.")
        bttry.charging = None 
   
    bttry.battery_pub.publish(bttry.battery_msg)

    return response



if __name__ == "__main__":
    try:
        bttry = VBNode() 
        
        rospy.Service("call_topic", CallTopic, handler)
        print("service is initialized.")
        
        rospy.spin() 

    except rospy.ROSInterruptException:
        pass 



