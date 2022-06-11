import rospy
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import Twist,PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np
from tf.transformations import euler_from_quaternion
import math as m

path=None
pose=[0,0,0]
goal=[0,0]
sensor_range=2.0 #Maximum sensing range
min_range=0.2 # Minimum sensing range
V_max=0.5 # Maximum linear velocity of the robot (m/s)
w_max=1.0 # Maximum yaw rate of the robot (rad/s)
obstacles=[]

def get_goal(msg):
    global goal
    # Update goal position by using Rviz tool 2d Nav Goal
    goal=[msg.pose.position.x,msg.pose.position.y]
    print ("Goal is found")

def get_pose(msg):
    global pose
    #Get x and y position
    x=msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    #Get 3d orientation info as quaternion form
    quat = msg.pose.pose.orientation
    #Convert quaternion angles to euler angles
    roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    # Update robot pose
    pose=[x,y,yaw]

def normalize_angle(ang):
    return m.atan2(m.sin(ang),m.cos(ang))

def robot_model(V,w,T):
    dt=0.2 #time interval
    #Create a variable to store x,y points and yaw angles
    xy=[[pose[0],pose[1]]]
    thetas=[pose[2]]
    for i in range(int(T/dt)): # Update x,y,theta for a specific interval
        # Fill here !!! 
        xy = xy + [[xy[-1][0] + m.cos(thetas[-1] + w*dt/2)*V*dt, xy[-1][1] + m.sin(thetas[-1] + w*dt/2)*V*dt]]
        thetas = thetas + [thetas[-1] + w*dt]
        pass
    # return with trajectory xy points in [[x1,y1],[x2,y2],...,[xn,yn]] form and last angle of the trajectory
    return xy[1:],thetas[-1]

def get_laser(msg):
    global obstacles
    # Filter lidar readings if measurement is bigger than sensor range or less than minimum range
    range_filter = [np.nan if (i > sensor_range or i < min_range) else i for i in msg.ranges]
    #Create corresponding angle array for lidar measurements
    angles = np.linspace(0 , 2*m.pi , 360).reshape(-1, 1) + pose[2]
    #Normalize these angles
    angles = np.arctan2(np.sin(angles), np.cos(angles)).reshape(-1, 1)
    #Convert lidar measurements to numpy array
    ranges = np.array(range_filter).reshape(-1, 1)
    # Convert lidar readings to x,y coordinates of odom frame
    obsx = (ranges * np.cos(angles)) + pose[0]
    obsy = (ranges * np.sin(angles)) + pose[1]
    coords = np.hstack((obsx, obsy))
    # Filter obstacle coordinates which includes NaN values
    obstacles = coords[~np.isnan(coords)].reshape(-1, 2)

def distance_to_obstacles(x,y):
    if obstacles.shape[0]==0: # If all obstacles are out of the sensor range
        return 2.0
    else:
        return min(np.hypot(obstacles[:,0]-x,obstacles[:,1]-y)) # Minimum distance between a point(x,y) and obstacles

def find_best_velocity(admissible_vels):
    #Goal angle in odom coordinates
    goal_angle=m.atan2(goal[1]-pose[1],goal[0]-pose[0])
    # Calculate velocity scores of each (V,w) pair
    scores=[]
    for (V,w,last_angle,dist) in admissible_vels:
        k_vel,k_angle,k_safety=1,0.3,0.3
        #Angle difference end of the trajectory angle and the goal angle
        angle_diff=normalize_angle(goal_angle-last_angle)
        #Score calculation
        score=k_vel*V+k_angle*((m.pi-abs(angle_diff))/m.pi)+k_safety*(dist)
        scores.append(score)
    #Find the maximum scored velocity pair
    best_score_index=scores.index(max(scores))
    best_pair=admissible_vels[best_score_index]
    return best_pair[0],best_pair[1]# V and w is the first and the second element of best pair

if __name__ == '__main__':
    #Initialize a ros node
    rospy.init_node('dwa_obstacle_avoidance', anonymous=True)

    # ROS publishers and subscribers
    rospy.Subscriber("/move_base_simple/goal",PoseStamped,get_goal)
    rospy.Subscriber("/odom", Odometry, get_pose)
    rospy.Subscriber("/scan", LaserScan, get_laser)
    pub1=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
    pub2 = rospy.Publisher("/forward_sim", Path, queue_size=1)

    # Rate parameter
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        if m.hypot(goal[1]-pose[1],goal[0]-pose[0])>0.3: # If goal is not reached
            admissible_velocities=[]
            # For all executable combinations ((V,w) pairs) for mobile robot
            for V in np.linspace(0.0,V_max,6):
                for w in np.linspace(-w_max,w_max,11):
                    #Simulate the control signal get xy coordinates and last angle of the trajectory
                    xy,last_angle=robot_model(V,w,2.0)
                    #Calculate distance to obstacles of each point (x,y) in xy point list of the trajectory
                    distances=[]
                    for [x,y] in xy:
                        xydist=distance_to_obstacles(x, y)
                        distances.append(xydist)
                    if min(distances)>0.2: # If minimum distance of the trajectory is less than robot diameter (V,w) is marked as safe
                        admissible_velocities.append([V,w,last_angle,min(distances)]) # Save the current control signal (V,w) for evaluation
            #Find the best scored control pair among the alternatives
            V_best,w_best=find_best_velocity(admissible_velocities)
        else:
            #Stop if goal is reached
            V_best,w_best=0,0

        #Create a robot control signal (V_best,w_best) and send to the robot
        vel_cmd = Twist()
        vel_cmd.linear.x = V_best
        vel_cmd.angular.z= w_best
        pub1.publish(vel_cmd)

        #Simulate the best control signal for visualization of the its trajectory
        xy, _ = robot_model(V_best, w_best, 2.0)
        # Create a path message and fill and publish xy coordinates of the best velocity pair trajectory
        pth_msg = Path()
        pth_msg.header.frame_id = "odom"
        pth_msg.poses = []
        for i in range(len(xy)):
            b = PoseStamped()
            b.pose.position.x = xy[i][0]
            b.pose.position.y = xy[i][1]
            b.pose.position.z = 0.0
            pth_msg.poses.append(b)
        pub2.publish(pth_msg)

        rate.sleep()