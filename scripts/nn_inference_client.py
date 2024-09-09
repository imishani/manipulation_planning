#!/usr/bin/env python3


import rospy
from manipulation_planning.srv import TriggerNN

# Function to call the service with a simple trigger
def nn_trigger_client():
    rospy.wait_for_service('nn_trigger')
    try:
        trigger_nn = rospy.ServiceProxy('nn_trigger', TriggerNN)
        response = trigger_nn(True)  # Sending trigger signal
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

if __name__ == "__main__":
    rospy.init_node('nn_inference_client')

    # Send trigger to server
    rospy.loginfo("Sending trigger to neural network inference server")
    success = nn_trigger_client()
    
    if success:
        rospy.loginfo("Service completed successfully")
    else:
        rospy.loginfo("Service failed")
