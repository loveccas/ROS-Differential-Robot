#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigator', anonymous=True)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待move_base服务器连接...")
        self.client.wait_for_server()
        rospy.loginfo("move_base服务器已连接！")

        # 定义航点列表（x, y, yaw），yaw单位：弧度（0=朝X轴正方向）
        self.waypoints = [
            (1.0, 0.0, 0.0),    # 航点1：(x=1, y=0, 朝向正前方)
            (1.0, 1.0, 1.57),   # 航点2：(x=1, y=1, 朝向正左方，π/2弧度)
            (0.0, 1.0, 3.14),   # 航点3：(x=0, y=1, 朝向正后方，π弧度)
            (0.0, 0.0, 0.0)     # 航点4：回到原点
        ]

    def send_goal(self, x, y, yaw):
        """发送单个导航目标点"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # 目标点在地图坐标系
        goal.target_pose.header.stamp = rospy.Time.now()

        # 设置位置
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        # 设置朝向（四元数，从yaw角转换）
        from tf.transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, yaw)  # 仅绕Z轴旋转yaw角
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        rospy.loginfo(f"发送目标点：({x}, {y}, yaw={yaw:.2f})")
        self.client.send_goal(goal)
        # 等待目标完成（超时30秒）
        success = self.client.wait_for_result(rospy.Duration(30.0))
        if success:
            state = self.client.get_state()
            rospy.loginfo(f"目标点到达，状态码：{state}")
            return True
        else:
            rospy.logwarn("目标点超时未到达！")
            self.client.cancel_goal()  # 取消当前目标
            return False

    def navigate_waypoints(self):
        """按顺序导航所有航点"""
        for i, (x, y, yaw) in enumerate(self.waypoints):
            rospy.loginfo(f"开始导航至航点 {i+1}/{len(self.waypoints)}")
            if not self.send_goal(x, y, yaw):
                rospy.logerr("导航失败，终止航点序列")
                return
            rospy.sleep(1)  # 到达后停留1秒
        rospy.loginfo("所有航点导航完成！")

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        navigator.navigate_waypoints()
    except rospy.ROSInterruptException:
        rospy.loginfo("导航被中断")