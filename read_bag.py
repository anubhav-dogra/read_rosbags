import rospy
import rosbag
import matplotlib.pyplot as plt
from geometry_msgs.msg import TransformStamped, WrenchStamped
import os, sys
class ROSBagReader:
    def __init__(self, filename):
        self.bag_name = filename
        self.ee_pose_data = []
        self.wrench_tool_data= []

    def read_bag(self):
        with rosbag.Bag(self.bag_name) as bag:
            for topic, msg, t in bag.read_messages(topics=['/tool_link_ee_pose', '/cartesian_wrench_tool']):
                if topic == '/tool_link_ee_pose':
                    self.ee_pose_data.append((t.to_sec(), msg.transform.translation.z))
                elif topic == '/cartesian_wrench_tool':
                    self.wrench_tool_data.append((t.to_sec(), msg.wrench.force.z))
    
    def plot_data(self):
        fig, axs = plt.subplots(2,1,figsize=(10,8),sharex=True)

        ee_pose_data = list(zip(*self.ee_pose_data))
        axs[0].plot(ee_pose_data[0], ee_pose_data[1], label='Z position of the Tip')
        axs[0].set_title('Probe tip Pose in Z')
        axs[0].legend()

        wrench_tool_data = list(zip(*self.wrench_tool_data))
        axs[1].plot(wrench_tool_data[0], wrench_tool_data[1], label='Z force of the Tool')
        axs[1].set_title('Tool wrench in Z')
        axs[1].legend()

        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    rospy.init_node("read_bag", anonymous=True)
    home = os.path.expanduser("~")
    filename = sys.argv[1]
    bag_name = home + '/Recordings/bags/'+ filename +'.bag'
    reader = ROSBagReader(bag_name)
    reader.read_bag()
    reader.plot_data()

