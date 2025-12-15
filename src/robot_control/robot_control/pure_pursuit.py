import numpy as np
import array
import math
import rclpy

from rclpy.node import Node

from std_msgs.msg import Int16, Empty, Float32MultiArray
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

import time



"""
    thetad desird direction. Angle between map x and control force
    theta = yaw. angle between robot direction and map 
"""
#Topics
#Used to control the robot
topic1="/SpeedControl"
#Used to resieve position measurements
topic2="/map"
#Used to receive lidar scans
topic3="/scan"

class ControllerNode(Node):
    def __init__(self, ka_u, kr_u, g_star,eps_control_u, jump_thresh, lookahead):



        super().__init__("controller_node")

        self.lidar_offset = math.pi / 2 

        self.lookahead = lookahead
        #Car params
        self.wheel_base=0.17
        self.max_actual_angle = math.radians(35)

        self.upper_speed_limit_forwards = 0.3
        self.lower_speed_limit_forwards = 0.1
        self.upper_speed_limit_backwards = -0.3
        self.lower_speed_limit_backwards = -0.1

        self.max_steeirng_angle = 180
        self.min_steering_angle = 0

        #initial speed params
        self.speed_task=0
        self.angle_task=90

        self.last_speed=self.speed_task
        self.last_angle=self.angle_task

         # TF
        self.global_frame = "map"
        self.robot_frame = "base_link"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        #store parameters
        #thresh for obstacle separation
        self.jump_thresh=jump_thresh
        #atractive force parameter
        self.ka=ka_u
        #repulsive force parameter
        self.kr=kr_u
        #parameter for obstacle expansion
        self.g_star=g_star
        #parameter for goal point radius
        self.eps_goal=eps_control_u

        self.LidarMsg = LaserScan()

        self.initialTime = time.time()

        self.msgOdometryTime = time.time()

        self.msgLidarTime = time.time()

        #desired position
        self.goal = None
        #callback to set goal
        self.create_subscription(Empty, "/go_forward", self.trigger_cb, 10)

        self.ControlPublisher = self.create_publisher(Float32MultiArray, topic1, 10)

        self.LidarSubscriber = self.create_subscription(LaserScan, topic3, self.SensorCallbackLidar, 10)

        #Control message time period
        self.period = 0.1

        self.timer = self.create_timer(self.period, self.ControlLoop)


    def trigger_cb(self, msg):
        pose = self.get_pose()
        if not pose:
            self.get_logger().warn("NO TF — cannot set goal")
            return

        x, y, theta = pose
        gx = x + self.lookahead * math.cos(theta)
        gy = y + self.lookahead * math.sin(theta)

        self.goal = (gx, gy)
        self.get_logger().info("Goal set")

    def get_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
        except Exception:
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        return t.x, t.y, theta
    
    def SensorCallbackLidar(self, msg):
        self.LidarMsg=msg

    def orientationError(self, theta, thetaD):
        # This function solves discontinuety problem between pi and -pi caused by atan2 function. 
        # Situation: we need to head to the direction thetad = -160 and robot orientation is theta = 160. I
        # Instead of going from theta -320 degrees to reach thetad alpha = thetad - theta = -320, we recalculate it. 
        # -320 > pi so -320 + 360 = 40 and this is the direction our robot needs to rotate
        alpha = thetaD - theta
        while alpha > math.pi:
            alpha -= 2 * math.pi
        while alpha < -math.pi:
            alpha += 2 * math.pi
        return alpha


    def ControlLoop(self):
        if not self.goal:
            self.publish(0.0, 90)
            return
        #Coordinates of the goal point D
        xd, yd = self.goal
        #Coordinates of the corrent position point B
        x, y, theta = self.get_pose()

        LidarRange = np.array(self.LidarMsg.ranges)

        #LidarRanges is an array containing distance measurements to obstacle points
        #If the ray is reflected from an obstacle point, the range is finite
        #otherwise the value is infinite
        # typically, lidar ranges look like this
        #LidarRanges-[inf, inf, inf, ..., inf, 3.2,3.21,3.3,3.4, inf, inf, inf, ..., inf, 4,2]
        #The non-zero entries correspond to the distance to obstacle points and we need to  analyze them

        angle_min = self.LidarMsg.angle_min
        angle_increment = self.LidarMsg.angle_increment

        #Atrractive force calculation

        vectorD = np.array([[x-xd], [y-yd]])
        gradUa = self.ka*vectorD
        AF = -gradUa

        #extract the indeces of measurements that are not infinite

        """change this. wrong in real world"""
        indices_not_inf = np.where(~np.isinf(LidarRange))[0]

        #Checking if there is an obstacle in lidar range. ID there is none, then we do not need to compute the repulsive force
        obstacleYES =~np.any(np.isinf(indices_not_inf))

        if (obstacleYES):

            #here we need to identify the obstacles from the range measurements
            #we need to segment the indices according to the obstacle they belong to.
            #The idea is that obstacle can be identified by identifying inf (non-reflected)
            #rays between htem. 
                                    # diff_array = np.diff(indices_not_inf)


                                    # split_indices = np.where(np.abs(diff_array) > 1)[0]+1

                                    # partitioned_arrays = np.split(indices_not_inf, split_indices)

            max_r = self.LidarMsg.range_max
            LidarRange[~np.isfinite(LidarRange)] = np.inf
            LidarRange[LidarRange <= 0.0] = np.inf
            LidarRange[LidarRange > max_r] = np.inf

            indices = np.arange(len(LidarRange))
            valid = ~np.isinf(LidarRange)
            valid_idx = indices[valid]

            # Differences in index (angular continuity)
            idx_diff = np.diff(valid_idx)

            # Differences in range (distance discontinuity)
            range_valid = LidarRange[valid]
            range_diff = np.abs(np.diff(range_valid))


            split_points = np.where((idx_diff > 1) | (range_diff > self.jump_thresh))[0] + 1
            partitioned_arrays = np.split(valid_idx, split_points)

            #calclute angles of all rays
            angles = angle_min+indices_not_inf*angle_increment+theta

            #calculate the position of all obstacle points of all obstacles
            distances = LidarRange[indices_not_inf]

            #compute obstacle point positions

            xo=x*np.ones(distances.shape)+distances*np.cos(angles)
            yo=y*np.ones(distances.shape)+distances*np.sin(angles)

            

            #compute minimal distances to obstacles and corresponding angles

            min_distances = []

            min_distances_angles=[]

            for i in range (len(partitioned_arrays)):
                tmpArray=LidarRange[partitioned_arrays[i]]
                min_index = np.argmin(tmpArray)
                min_distances.append(min(tmpArray))
                min_distances_angles.append(angle_min+angle_increment*partitioned_arrays[i][min_index])


            #comoute the ccordinates of the obstacle point O in the fixed frame
            xo_min=[]
            yo_min=[]

            for i in range(len(min_distances)):
                xo_min.append(x+min_distances[i]*np.cos(min_distances_angles[i]+theta + self.lidar_offset))
                yo_min.append(y+min_distances[i]*np.sin(min_distances_angles[i]+theta + self.lidar_offset))

            #compute gradiant value for every obstacle

            g_values =[]
            gradUr=[]

            for i in range(len(min_distances)):
                gradUr_i=np.array([[0],[0]])
                g_val = np.sqrt((x-xo_min[i])**2+(y-yo_min[i])**2)
                g_values.append(g_val)

                if (g_val <= self.g_star):
                    pr = self.kr*((1/self.g_star)-(1/g_values[i]))*(1/((g_values[i])**3))
                    gradUr_i = pr*np.array([[x-xo_min[i]],[y-yo_min[i]]])

                gradUr.append(gradUr_i)

            #compute teh repulsive force 

            RF = np.array([[0],[0]])

            for i in range (len(gradUr)):
                RF=RF+gradUr[i]

            RF=-RF

            #Compute the control force

            if (obstacleYES):
                F=AF+RF
            else:
                F=AF
        

            #Calculate the deisred orientation thetad
            thetaD = math.atan2(F[1,0],F[0,0])

            alpha = self.orientationError(theta, thetaD)

            alpha_deg= math.degrees(alpha)

            if (np.linalg.norm(vectorD,2)<self.eps_goal):
                speed=0
                angle=0
                self.get_logger().info("GOAL REACHED")
                self.publish(speed, angle+90)
                return
            
            else:
                # if not ((alpha_deg <90 and alpha_deg >80) or (alpha_deg < -80 and alpha_deg >-90)):
                #     if alpha_deg >= -80 and alpha_deg <=0:
                #         angle = alpha_deg* 90 / 80
                #         speed = 0.1 + alpha_deg/80 *(self.upper_speed_limit_forwards-self.lower_speed_limit_forwards)

                #     elif alpha_deg <= -90:
                #         angle = -alpha_deg+90
                #         speed = -(0.1 + alpha_deg/90 *(self.upper_speed_limit_forwards-self.lower_speed_limit_forwards))

                #     elif alpha_deg >=0 and alpha_deg < 80:
                #         angle = -alpha_deg* 90 / 80
                #         speed = 0.1 + alpha_deg/80 *(self.upper_speed_limit_forwards-self.lower_speed_limit_forwards)
                #     elif alpha_deg >=90:
                #         angle = alpha_deg+90
                #         speed = -(0.1 + alpha_deg/90 *(self.upper_speed_limit_forwards-self.lower_speed_limit_forwards))
                #     self.last_angle = angle
                #     self.last_speed = speed
                # elif (alpha_deg <90 and alpha_deg >80) or (alpha_deg < -80 and alpha_deg >-90):
                #     #keep last value
                #     angle = self.last_angle
                #     speed = self.last_speed
                
                # Fmag  = np.linalg.norm(F)
                # scale = Fmag / (Fmag + 1.0)
                # speed *= scale
                # self.publish(speed, angle+90)
                #Compute heading error
                alpha = self.orientationError(theta, thetaD)
                alpha_deg_raw = math.degrees(alpha)

                # ---------------- Driving mode decision ----------------
                reverse = False     # <-- reverse DISABLED

                # ---------------- Steering clamp ----------------
                max_wheel_deg = 35.0
                alpha_deg = max(-max_wheel_deg, min(max_wheel_deg, alpha_deg_raw))

                # ---------------- Servo mapping
                # LEFT = 0, RIGHT = 180 ----------------
                angle_cmd = 90.0 - (alpha_deg / max_wheel_deg) * 90.0
                angle_cmd = max(0.0, min(180.0, angle_cmd))

                # ---------------- Speed mapping ----------------
                turn_ratio = abs(alpha_deg) / max_wheel_deg

                speed = (self.upper_speed_limit_forwards -
                    turn_ratio * (
                        self.upper_speed_limit_forwards -
                        self.lower_speed_limit_forwards
                    )
                )

                # ---------------- Prevent pivoting (keep minimum crawl) ----------------
                speed = max(speed, 0.15)

                # ---------------- Force-based scaling ----------------
                Fmag = np.linalg.norm(F)
                scale = Fmag / (Fmag + 1.0)
                speed *= scale
                if speed > self.upper_speed_limit_forwards:
                    speed = self.upper_speed_limit_forwards

                self.publish(speed, angle_cmd)





            
    def publish(self, speed, angle):

        msg = Float32MultiArray()
        #msg.data = [float(speed), float(angle+90)]
        msg.data = [float(speed), float(angle)]
        
        self.ControlPublisher.publish(msg)




def main(args=None):

    lookahead = 2.5
    #select control parameters
    ka_u = 4.0
    kr_u =  1.8
    gstar_u = 0.5
    eps_control_u = 0.1
    #k_theta =4
    # Threshold for segmenting obstacles by range jump
    jump_thresh = 0.4  # meters, tune this
    
    rclpy.init()
    node = ControllerNode(ka_u, kr_u, gstar_u,eps_control_u, jump_thresh, lookahead)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
