import pickle
from collections import Counter
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

with open ('object_list.txt', 'rb') as fp:
    object_list = pickle.load(fp)

def init_node():
    global pub
    rospy.init_node("coordinates_of_detections")
    rospy.loginfo("publishing the results for 30 secs...")
    pub = rospy.Publisher("coordinates_of_detections", Float64MultiArray, queue_size=4)

def node_publish(object_list):
    length_list = len(object_list)
    for i in range(len(object_list)):
        msg = Float64MultiArray()
        x = object_list[i][1]
        y = object_list[i][2]
        z = object_list[i][3]
        dist = object_list[i][4]
        data_list = [length_list, i, x, y, z, dist]
        msg.data = data_list
        pub.publish(msg)
        rospy.sleep(0.5)


def empty_publish():
    msg = Float64MultiArray()
    msg.data = [-1, 0, 0, 0]
    pub.publish(msg)
    rospy.sleep(1)


counter_list = []
objects = Counter(objects[0] for objects in object_list).keys()
objects = list(objects)
number_of_objects = len(objects)

organized_object_list = []
for i in range(number_of_objects):
    organized_object_list.append([])
    for obj in object_list:
        if(obj[0] == objects[i]):
            organized_object_list[i].append(obj)

# print(len(organized_object_list))
final_object_list = []
for i in range(number_of_objects):
    final_object_list.append([])
    dc_x = 0
    dc_y = 0
    dc_z = 0
    distance = 0
    for obj in organized_object_list[i]:
        dc_x = dc_x + obj[1][0]
        dc_y = dc_y + obj[1][1]
        dc_z = dc_z + obj[1][2]
        distance = distance + obj[2]
    dc_x = dc_x / len(organized_object_list[i])
    dc_y = dc_y / len(organized_object_list[i])
    dc_z = dc_z / len(organized_object_list[i])
    distance = distance / len(organized_object_list[i])
    final_object_list[i] = [objects[i], dc_x, dc_y, dc_z, distance]

for i in range(len(final_object_list)):
    print(final_object_list[i][1])

init_node()

while True:
    node_publish(final_object_list)