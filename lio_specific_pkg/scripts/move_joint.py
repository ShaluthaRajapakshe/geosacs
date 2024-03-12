import rospy
from std_msgs.msg import Bool

next_js = [0, 0, 0, 0, 0, 0, 0]
kill = False

def joint_states_cb(msg):
    global next_js
    next_js = list(msg.position)

def call_move_joint(joint_states):
    if joint_states == [0, 0, 0, 0, 0, 0]: return
    #start_time = rospy.Time.now()
    move_joint([1,2,3,4,5,6], joint_states, velocity=25, acceleration=90, block=False)
    #delta_t = rospy.Time.now()-start_time
    #print(delta_t.to_sec())
    

def call_gripper(state):
    if state == 1:
        # tool_place()
        move_gripper(30, velocity=100, current=None, block=False)
    elif state == -1:
        # tool_pick()
        move_gripper(0, velocity=100, current=0.5, block=False)
    else : return

def app_state_cb(msg):
    global kill
    if msg.data == "stop": 
        print("stop application")
        kill = True
        



ros_handler.subscribe_to_rostopic("/ik_interface/joint_states_lio", joint_states_cb, wait_for_data=False)
ros_handler.subscribe_to_rostopic("/myp_manager/app_state", app_state_cb, wait_for_data=False)

while not kill:
    current_js = next_js
    call_move_joint(current_js[:6])
    move_gripper(current_js[6], velocity=30, current=None, block=False)
    wait(0.05)
    