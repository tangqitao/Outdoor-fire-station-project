# coding=utf-8
import rospy
from geometry_msgs.msg import PoseStamped

class ParserHeading(object):
    """ROS driver for Heading."""
    def __init__(self):
        self.haeding_pub = rospy.Publisher('HEADING', PoseStamped, queue_size=1)


    def check_checksum(self, headingString):
        return True

    def parasSentence(self, headingString):
        headingLs = headingString.split(";")
        if len(headingLs) != 2:
            return False
        heading_data = headingLs[1].split(",")
        rospy.loginfo(heading_data)
    def add_sentence(self, headingString):
        """
        :param
            headingString: Headinga 格式数据类型 string

        Returns:
            bool: True if the string is successfully processed, False if there is an error.
        """

        # 校验数据贞
        if not self.check_checksum(headingString):
            rospy.logwarn("Received a sentence with an invalid checksum. " +
                          "Sentence was: %s" % repr(headingString))
            return False
        self.parasSentence(headingString)
