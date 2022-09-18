
# coding=utf-8
import sys
import os 
import inspect
current_dir=os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
os.chdir(current_dir)

sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages/nmea_msgs')
from nmea_msgs.msg import Sentence
import rospy
from parser_headinga import ParserHeading
from libnmea_navsat_driver.driver import RosNMEADriver




def nmea_sentence_callback(nmea_sentence, driver):
    """Process a NMEA sentence message with a RosNMEADriver.

    Args:
        nmea_sentence (nmea_msgs.msg.Sentence): NMEA sentence message to feed to the driver.
        driver (RosNMEADriver): Driver to feed the sentence.
    """
    try:
        # 对sentence 切片，判断是Heading 或 GGA
        rospy.loginfo(nmea_sentence.sentence)
        data_ls = nmea_sentence.sentence.split(",")
        rospy.loginfo(data_ls[0])
        if data_ls[0] == "$GPGGA":
            rospy.loginfo("收到GPGGA")
            driver.add_sentence(
                nmea_sentence.sentence,
                frame_id=nmea_sentence.header.frame_id,
                timestamp=nmea_sentence.header.stamp)
        elif data_ls[0] == "#HEADINGA":
            # rospy.loginfo("Heading data")
            # rospy.loginfo(data_ls)
            rospy.loginfo("收到Heading")
            driver.add_heading_sentence(
                nmea_sentence.sentence,
                frame_id=nmea_sentence.header.frame_id,
                timestamp=nmea_sentence.header.stamp)
 

        else:
            rospy.logwarn("数据有误")
    except ValueError as e:
        rospy.logwarn(
            "Value error, likely due to missing fields in the NMEA message. "
            "Error was: %s. Please report this issue at github.com/ros-drivers/nmea_navsat_driver, "
            "including a bag file with the NMEA sentences that caused it." %
            e)


def main():
    """Create and run the nmea_topic_driver ROS node.

    Creates a NMEA Driver and feeds it NMEA sentence strings from a ROS subscriber.

    :ROS Subscribers:
        - nmea_sentence (nmea_msgs.msg.Sentence)
            NMEA sentence messages to feed to the driver.
    """
    rospy.init_node('nmea_topic_driver')

    driver = RosNMEADriver()

    rospy.Subscriber("nmea_sentence", Sentence, nmea_sentence_callback,   driver)

    rospy.spin()


