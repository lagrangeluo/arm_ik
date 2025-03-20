import os
import time
import subprocess
import rospy
import signal
from std_msgs.msg import String


#node_name = "/arm_task_manager"

node_list = ["hand_eye_tf","left_hand_eye_tf","green_to_red_tf","robot_state_publisher"]

# 2. 等待 A 彻底消失
def wait_for_node_death(node_name, timeout=10):
    start_time = time.time()
    while time.time() - start_time < timeout:
        result = subprocess.run(["rosnode", "list"], capture_output=True, text=True)
        if node_name not in result.stdout:
            return True  # A 已经被杀死
        time.sleep(0.1)  # 每 0.1 秒检查一次
    return False  # 超时未能杀死

def task_cmd_callback(msg):
    if msg.data == "stop":
        rospy.loginfo("Got stop cmd ros msg")

        processes = [subprocess.Popen(["rosnode","kill",node_name]) for node_name in node_list]
        for p in processes:
            p.wait()
        
        for node_name in node_list:

            if wait_for_node_death(node_name):
                print(f"节点 {node_name} 已经被成功杀死...")
                # 启动 A 在新终端
                #os.system("gnome-terminal -- bash -c 'source ~/.bashrc&& startD; exec bash'")
            else:
                print(f"警告: 超时未能确认 {node_name} 被杀死，仍然尝试重新启动...")

        subprocess.Popen(["gnome-terminal", "--", "bash", "-i","-c", "source ~/.bashrc && startD; exec bash"])

def signal_handler(signal,frame):
    print('You pressed Ctrl + C!')
    rospy.signal_shutdown('Shutting down...')

if __name__=='__main__' :
    rospy.init_node("task_monitor")
    rospy.loginfo("start monitor")
    cmd_sub = rospy.Subscriber("/task_cmd",String,task_cmd_callback)
    rospy.spin()

