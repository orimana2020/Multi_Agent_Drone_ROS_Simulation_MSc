#! /usr/bin/env python
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Quaternion, Point,Twist, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import std_msgs.msg
import time, math
import params

class Flight_manager(object):
    def __init__(self):
        self.linear_velocity_limit = params.linear_velocity
        self.drone_num = params.drone_num
        self.traj = MultiDOFJointTrajectory()
        self.header = std_msgs.msg.Header()
        self.header.stamp = rospy.Time()
        self.header.frame_id = 'frame'
        self.traj.joint_names.append('base_link')
        self.traj.header = self.header  
        self.velocities = Twist()    
        self.accelerations = Twist()  
        self.pubs = []
        self.goal = []
        for drone_idx in range(self.drone_num):
            command_pub = rospy.Publisher('/ardrone%d/command/trajectory' %drone_idx, MultiDOFJointTrajectory, queue_size=10)
            self.pubs.append(command_pub)
            rospy.Subscriber('/ardrone%d/ground_truth/pose' %drone_idx , Pose)
            self.get_pos(drone_idx)
            self.goal.append([])
        while self.pubs[0].get_num_connections() == 0 and not rospy.is_shutdown():
            print("There is no subscriber available, trying again in 1 second.")
            time.sleep(1)

    
    def rotate_inplace(self, angle):
        quaternion = quaternion_from_euler(ai=0, aj=0, ak=angle, axes='sxyz') #roll pictch yaw
        return Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

    def get_yaw(self,start,goal):
        x1,y1,_ = start
        x2,y2,_ = goal
        if x2>=x1:
            yaw_rad = math.atan2((y2-y1) , (x2-x1))
        else:
            yaw_rad = math.atan2((y2-y1) , (x2-x1)) - math.pi
        quaternion = quaternion_from_euler(ai=0, aj=0, ak=yaw_rad, axes='sxyz') #roll pictch yaw
        return Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

    def time_estimator(self,pnt1, pnt2):
        x1,y1,z1 = pnt1
        x2,y2,z2 = pnt2
        dist = ((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)**0.5 # m
        return dist / self.linear_velocity_limit # sec

    def publish_traj_command(self, waypoints, drone_idx):
        self.traj.points = []
        self.header.stamp = rospy.Time()
        timer = 0
        
        transforms = Transform(translation=Point(waypoints[0][0], waypoints[0][1], waypoints[0][2]))
        point0 = MultiDOFJointTrajectoryPoint([transforms], [self.velocities], [self.accelerations], rospy.Time(timer))
        self.traj.points.append(point0)

        for i in range(1, len(waypoints)):
            rotation = self.get_yaw(start=waypoints[i-1], goal=waypoints[i])
            transforms = Transform(translation=Point(waypoints[i][0], waypoints[i][1], waypoints[i][2]), rotation=rotation)
            timer += self.time_estimator(waypoints[i],waypoints[i-1])
            point = MultiDOFJointTrajectoryPoint([transforms], [self.velocities], [self.accelerations], rospy.Time(timer))
            self.traj.points.append(point)
        self.pubs[drone_idx].publish(self.traj)
        self.goal[drone_idx] = waypoints[-1]



    def get_pos(self, drone_idx):
        try:
            pose = rospy.wait_for_message('/ardrone%d/ground_truth/pose' %drone_idx , Pose, timeout=3)
            self.pos = pose.position
            _,_,self.current_yaw = euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
        except:
            print('can not subscribe position')
            return None

    def reached_goal(self, drone_idx):
        try:
            self.get_pos(drone_idx)
            dist2goal = ((self.pos.x - self.goal[drone_idx][0])**2 + (self.pos.y - self.goal[drone_idx][1])**2 +(self.pos.z - self.goal[drone_idx][2])**2 )**0.5
            if dist2goal < 0.1:
                return 1
            else:
                return 0
        except:
            return 0

    def _take_off(self, height, drone_idx):
        self.get_pos(drone_idx)
        self.goal[drone_idx] = [self.pos.x, self.pos.y, height]
        waypoints = [self.goal[drone_idx]]
        self.publish_traj_command(waypoints, drone_idx)
    
    def take_off_swarm(self, height, drone_num):
        for drone_idx in range(drone_num):
            self._take_off(drone_idx=drone_idx, height=height)
        rospy.sleep(5)



        
