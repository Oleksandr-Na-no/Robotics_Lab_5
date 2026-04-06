import rclpy
from rclpy.node import Node
import numpy as np
import time
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import tf_transformations


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")
        
        self.last_print_time = 0

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("goal_x", 3.0)
        self.declare_parameter("goal_y", 3.0)

        self.goalX = float(self.get_parameter("goal_x").value)
        self.goalY = float(self.get_parameter("goal_y").value)

        self.kaU = 0.4
        self.krU = 5
        self.kThetaU = 4.0
        self.gStarU = 1.2
        self.epsOrientU = np.pi / 6
        self.epsControlU = 0.2

        self.OdometryMsg = Odometry()
        self.LidarMsg = LaserScan()

        self.initialTime = time.time()
        self.msgOdometryTime = time.time()
        self.msgLidarTime = time.time()

        self.controlVel = TwistStamped()

        self.ControlPublisher = self.create_publisher(
            TwistStamped,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )

        self.PoseSubscriber = self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.odom_callback,
            10,
        )

        self.LidarSubscriber = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.scan_callback,
            10,
        )

        self.period = 0.05
        self.timer = self.create_timer(self.period, self.ControlFunction)

    def OrientationError(self, theta, thetaD):
        if (thetaD > np.pi / 2) and (thetaD <= np.pi):
            if (theta > -np.pi) and (theta <= -np.pi / 2):
                theta += 2 * np.pi

        if (theta > np.pi / 2) and (theta <= np.pi):
            if (thetaD > -np.pi) and (thetaD <= -np.pi / 2):
                thetaD += 2 * np.pi

        return thetaD - theta

    def odom_callback(self, msg):
        self.OdometryMsg = msg
        self.msgOdometryTime = time.time()

        # перевіряємо, чи пройшла 1 секунда від останнього виводу
        if self.msgOdometryTime - self.last_print_time >= 1.0:
            self.last_print_time = self.msgOdometryTime
            # отримуємо координати з msg
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            print(f"Координати робота: x={x:.2f}, y={y:.2f}, z={z:.2f}")

    def scan_callback(self, msg):
        self.LidarMsg = msg
        self.msgLidarTime = time.time()

    def ControlFunction(self):
        ka = self.kaU
        kr = self.krU
        kTheta = self.kThetaU
        gStar = self.gStarU

        xd = self.goalX
        yd = self.goalY

        x = self.OdometryMsg.pose.pose.position.x
        y = self.OdometryMsg.pose.pose.position.y

        quat = self.OdometryMsg.pose.pose.orientation
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w]
        )
        theta = yaw

        lidarRanges = np.array(self.LidarMsg.ranges)
        angleMin = self.LidarMsg.angle_min
        angleIncrement = self.LidarMsg.angle_increment

        vectorD = np.array([[x - xd], [y - yd]])
        gradUa = ka * vectorD
        AF = -gradUa

        indicesNotInf = np.where(~np.isinf(lidarRanges))[0]

        if len(indicesNotInf) > 0:
            diffArray = np.diff(indicesNotInf)
            splitIndices = np.where(diffArray > 1)[0] + 1
            partitionedArray = np.split(indicesNotInf, splitIndices)

            angles = angleMin + indicesNotInf * angleIncrement + theta
            distances = lidarRanges[indicesNotInf]

            xo = x + distances * np.cos(angles)
            yo = y + distances * np.sin(angles)

            minDist = []
            minDistAngles = []

            for i in range(len(partitionedArray)):
                tmpArray = lidarRanges[partitionedArray[i]]
                minIndex = np.argmin(tmpArray)
                minDist.append(tmpArray[minIndex])
                minDistAngles.append(
                    angleMin + angleIncrement * partitionedArray[i][minIndex]
                )

            xoMin = []
            yoMin = []

            for i in range(len(minDist)):
                xoMin.append(x + minDist[i] * np.cos(minDistAngles[i] + theta))
                yoMin.append(y + minDist[i] * np.sin(minDistAngles[i] + theta))

            gradUr = []

            for i in range(len(minDist)):
                gradUrI = np.array([[0.0], [0.0]])
                gVal = np.sqrt((x - xoMin[i]) ** 2 + (y - yoMin[i]) ** 2)

                if gVal <= gStar and gVal != 0:
                    pr = kr * ((1 / gStar) - (1 / gVal)) * (1 / (gVal ** 3))
                    gradUrI = pr * np.array([[x - xoMin[i]], [y - yoMin[i]]])

                gradUr.append(gradUrI)

            RF = np.array([[0.0], [0.0]])

            for i in range(len(gradUr)):
                RF = RF + gradUr[i]

            RF = -RF
            F = AF + RF
        else:
            F = AF

        thetaD = math.atan2(F[1, 0], F[0, 0])

        eorient = self.OrientationError(theta, thetaD)

        if np.linalg.norm(vectorD, 2) < self.epsControlU:
            thetavel = 0.0
            xvel = 0.0
        else:
            if abs(eorient) > self.epsOrientU:
                thetavel = kTheta * eorient
                xvel = 0.0
            else:
                thetavel = kTheta * eorient
                xvel = np.linalg.norm(F, 2)

            if abs(xvel) > 2.6:
                xvel = 2.5

        self.controlVel.twist.linear.x = xvel
        self.controlVel.twist.linear.y = 0.0
        self.controlVel.twist.linear.z = 0.0
        self.controlVel.twist.angular.x = 0.0
        self.controlVel.twist.angular.y = 0.0
        self.controlVel.twist.angular.z = thetavel

        self.ControlPublisher.publish(self.controlVel)

        # print("Received pose:")
        timeDiff = self.msgOdometryTime - self.initialTime

        dist_to_goal = np.linalg.norm(vectorD, 2)

        if len(indicesNotInf) > 0:
            closest_obstacle_dist = np.min(lidarRanges[indicesNotInf])
        else:
            closest_obstacle_dist = float('inf')

        # print(f"Time,x,y,theta:({timeDiff:.3f},{x:.3f},{y:.3f},{theta:.3f})")
        # print(f"Distance to goal: {dist_to_goal:.3f}")
        # print(f"Closest obstacle distance: {closest_obstacle_dist:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()