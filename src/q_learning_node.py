#!/usr/bin/env python3

import subprocess
import rospkg
import rospy
import numpy as np
import pandas as pd
import random
import os
import tf
import math
from geometry_msgs.msg import Twist, Pose, Quaternion

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SetModelState as SetModelStateService
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_node import QlearningVisualizer

class MultiRobotQLearning:
    def __init__(self):
        try:
            #We are basically initializing the node,which will coordinate four TurtleBot3 robots, handling their Q-learning, movement, in a Gazebo simulation.
            rospy.init_node('multi_robot_qlearning', anonymous=True)
            
            # We define the robot's configurations like where do they spawn each time and how to identify them and control them(using command velocity string)

            self.robots = [
                {"name": "robot1", "namespace": "/robot1", "initial_pose": (2.0, 2.0, 3.14), "cmd_vel_topic": "/robot1/cmd_vel"},
                {"name": "robot2", "namespace": "/robot2", "initial_pose": (-2.0, -2.0, 0.0), "cmd_vel_topic": "/robot2/cmd_vel"},  # Example: 90 degrees
                {"name": "robot3", "namespace": "/robot3", "initial_pose": (2.0, -2.0, 1.57), "cmd_vel_topic": "/robot3/cmd_vel"},  # Example: -90 degrees
                {"name": "robot4", "namespace": "/robot4", "initial_pose": (-2.0, 2.0, -1.57), "cmd_vel_topic": "/robot4/cmd_vel"}
            ]
            
            # Spawn robots
            self.spawn_robots()
            
            # Initialize robot-specific attributes
            self.robot_states = {}
            self.robot_cmd_pubs = {}
            self.robot_odom_subs = {}
            self.robot_scan_subs = {}
            
            # Initialize robots
            for robot in self.robots:
                self.initialize_robot(robot)
            
            # Q-learning parameters
            self.epsilon = 0.5
            self.epsilon_min = 0.01
            self.epsilon_decay = 0.995
            self.alpha = 0.5
            self.gamma = 0.95
            self.actions = [-1.57, -0.785, 0, 0.785, 1.57]
            self.linear_speed = 0.50
            
            # Q-table initialization and loading it 
            self.q_table_file = 'q_table_data.csv'
            self.q_table = self.load_q_table()
            
            
            # Visualization
            self.visualizer = QlearningVisualizer()
            
            
            # It waits until the gazebo is being opened and to get initialized ,to get the packages get loaded
            rospy.wait_for_service('/gazebo/set_model_state')

            # This service is used to move, reset, or modify a robot's position inside Gazebo after it has been spawned.
            # Creates a proxy to interact with the service.
            self.set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelStateService)
            
        except Exception as e:
            rospy.logerr(f"Initialization error: {e}")
            raise

    def spawn_robots(self):
        try:
            # Which helps locate ROS packages
            rospack = rospkg.RosPack()

            # Used to find the TurtleBot3 description package 
            turtlebot3_description_path = rospack.get_path("turtlebot3_description")

            # It constructs the path to the URDF file
            xacro_file = f"{turtlebot3_description_path}/urdf/turtlebot3_burger.urdf.xacro"

            for robot in self.robots:
                try:
                    # Convert Xacro to URDF
                    robot_description = subprocess.check_output(["xacro", xacro_file]).decode("utf-8")
                    
                    # It waits until the gazebo is being opened and to get initialized ,to get the packages get loaded
                    rospy.wait_for_service('/gazebo/spawn_urdf_model')

                    # It takes care of calling the service, passing data, and receiving responses, so you don't have to handle everything manually.
                    # Using a proxy, you don't need to create a request object manually every time—you just call the service like a function
                    # This service is used to spawn (create) a robot in the simulation using its URDF (Unified Robot Description Format).
                    spawn_service = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
                    
                    # A Pose() object is created to set the robot's initial position and orientation.
                    initial_pose = Pose()
                    x, y, yaw = robot["initial_pose"]
                    initial_pose.position.x = x
                    initial_pose.position.y = y
                    quaternion = quaternion_from_euler(0, 0, yaw)
                    initial_pose.orientation.x = quaternion[0]
                    initial_pose.orientation.y = quaternion[1]
                    initial_pose.orientation.z = quaternion[2]
                    initial_pose.orientation.w = quaternion[3]
                    
                    # Calls the spawn service to place the robot in Gazebo.
                    spawn_service(robot["name"], robot_description, robot["namespace"], initial_pose, "world")
                    rospy.loginfo(f"{robot['name']} spawned successfully at ({x}, {y})")
                
                except Exception as spawn_error:
                    rospy.logerr(f"Error spawning {robot['name']}: {spawn_error}")

        except Exception as e:
            rospy.logerr(f"Robot spawning error: {e}")
            raise

    def initialize_robot(self, robot):
        """Initialize individual robot attributes"""
        try:
            # Sets up publishers for controlling the robot by sending velocity commands to the robot.
            self.robot_cmd_pubs[robot["name"]] = rospy.Publisher(robot["cmd_vel_topic"], Twist, queue_size=1)
            
            # Sets up subscribers to receive odometry(to receive robot position updates) and laser scan data(to receive obstacle detection data)
            self.robot_odom_subs[robot["name"]] = rospy.Subscriber(f"{robot['namespace']}/odom", Odometry, 
                                                                   lambda msg, name=robot["name"]: self.odom_callback(msg, name))
            self.robot_scan_subs[robot["name"]] = rospy.Subscriber(f"{robot['namespace']}/scan", LaserScan, 
                                                                   lambda msg, name=robot["name"]: self.scan_callback(msg, name))
            
            # Initializes the robot's state information for tracking its position and exploration.
            self.robot_states[robot["name"]] = {
                "x": robot["initial_pose"][0],
                "y": robot["initial_pose"][1],
                "orientation": 0,
                "min_distance": float('inf'),
                "state_visits": {},
                "explored_grids": set(),
                "prev_state": None,
                "prev_action": None
            }
        
        except Exception as e:
            rospy.logerr(f"Error initializing robot {robot['name']}: {e}")
            raise

    def load_q_table(self):
        try:
            if os.path.exists(self.q_table_file):
                return pd.read_csv(self.q_table_file)
            return self.create_empty_q_table()
        except Exception as e:
            rospy.logerr(f"Q-table loading error: {e}")
            return self.create_empty_q_table()

    def create_empty_q_table(self):
        return pd.DataFrame(columns=['robot', 'grid_x', 'grid_y', 'orientation', 
                                     'action_1', 'action_2', 'action_3', 'action_4', 'action_5'])

    # Additional methods like discretize_state, calculate_reward, etc. would be added here
    # Implement similar logic as in the original Q-learning script, but with multi-robot support

    def discretize_state(self, robot_name):
        robot_state = self.robot_states[robot_name]
        grid_x = round(robot_state['x'] / 0.25)
        grid_y = round(robot_state['y'] / 0.25)
        
        # Discretize orientation into 8 states
        angle = math.degrees(robot_state['orientation']) % 360
        if 0 <= angle < 22.5 or 337.5 <= angle < 360:
            orientation = 'N'
        elif 22.5 <= angle < 67.5:
            orientation = 'NE'
        elif 67.5 <= angle < 112.5:
            orientation = 'E'
        elif 112.5 <= angle < 157.5:
            orientation = 'SE'
        elif 157.5 <= angle < 202.5:
            orientation = 'S'
        elif 202.5 <= angle < 247.5:
            orientation = 'SW'
        elif 247.5 <= angle < 292.5:
            orientation = 'W'
        else:
            orientation = 'NW'
        
        # Create state tuple with robot name
        state = (robot_name, grid_x, grid_y, orientation)
        
        # Update visit frequency
        robot_state['state_visits'][state] = robot_state['state_visits'].get(state, 0) + 1
        
        # Add grid position to explored set
        robot_state['explored_grids'].add((grid_x, grid_y))
        
        return state

    def get_q_values(self, state):
        robot_name, grid_x, grid_y, orientation = state
        state_row = self.q_table[
            (self.q_table['robot'] == robot_name) & 
            (self.q_table['grid_x'] == grid_x) & 
            (self.q_table['grid_y'] == grid_y) & 
            (self.q_table['orientation'] == orientation)
        ]
        
        if len(state_row) == 0:
            rospy.loginfo(f"New state discovered for {robot_name}: {state}")
            new_row = pd.DataFrame({
                'robot': [robot_name],
                'grid_x': [grid_x], 
                'grid_y': [grid_y], 
                'orientation': [orientation],
                'action_1': [0], 'action_2': [0], 'action_3': [0], 
                'action_4': [0], 'action_5': [0]
            })
            self.q_table = pd.concat([self.q_table, new_row], ignore_index=True)
            self.save_q_table()
            return [0, 0, 0, 0, 0]
        
        return state_row.iloc[0, 4:].values

    def save_q_table(self):
        try:
            self.q_table.to_csv(self.q_table_file, index=False)
        except Exception as e:
            rospy.logerr(f"Failed to save Q-table: {e}")

    def choose_action(self, state):
        if random.random() < self.epsilon:
            action = random.choice(range(len(self.actions)))
            rospy.loginfo(f"Choosing random action: {action+1}")
            return action
        
        q_values = self.get_q_values(state)
        action = np.argmax(q_values)
        rospy.loginfo(f"Choosing best action: {action+1} (Q-value: {q_values[action]:.3f})")
        return action

    def calculate_reward(self, robot_name):
        robot_state = self.robot_states[robot_name]
        
        # Calculate exploration-based reward
        exploration_ratio = len(robot_state['explored_grids']) / 324 
        exploration_reward = 100.0 * exploration_ratio
        
        # Collision penalty
        collision_penalty = -200 if robot_state['min_distance'] < 0.15 else 0
        
        # Calculate revisit penalty
        current_state = self.discretize_state(robot_name)
        visit_count = robot_state['state_visits'].get(current_state, 0)
        revisit_penalty = 0
        
        if visit_count > 1:
            revisit_penalty = -5.0 * (visit_count - 1)
        
        total_reward = exploration_reward + collision_penalty + revisit_penalty
        
        rospy.loginfo(f"{robot_name} Reward breakdown - " 
                    f"Exploration: {exploration_reward:.2f}, " 
                    f"Collision: {collision_penalty}, " 
                    f"Revisit: {revisit_penalty:.2f}")
        return total_reward

    def update_q_value(self, state, action, reward, next_state, robot_name):
        # Find or create state row in Q-table
        state_row = self.q_table[
            (self.q_table['robot'] == robot_name) &
            (self.q_table['grid_x'] == state[1]) &
            (self.q_table['grid_y'] == state[2]) &
            (self.q_table['orientation'] == state[3])
        ]
        
        if state_row.empty:
            new_row = pd.DataFrame({
                'robot': [robot_name],
                'grid_x': [state[1]],
                'grid_y': [state[2]],
                'orientation': [state[3]],
                'action_1': [0], 'action_2': [0], 'action_3': [0],
                'action_4': [0], 'action_5': [0]
            })
            self.q_table = pd.concat([self.q_table, new_row], ignore_index=True)
            state_idx = self.q_table.index[-1]
        else:
            state_idx = state_row.index[0]

        # Calculate Q-value update
        current_q = self.q_table.loc[state_idx, f'action_{action + 1}']
        next_q_values = self.get_q_values(next_state)
        next_max_q = max(next_q_values)
        new_q = current_q + self.alpha * (reward + self.gamma * next_max_q - current_q)

        # Update Q-table
        self.q_table.loc[state_idx, f'action_{action + 1}'] = new_q
        self.save_q_table()

    # 2. Modify the reset_position method to use the robot's initial orientation
    def reset_position(self, robot_name):
        try:
            # Find the robot's configuration
            robot_config = next(robot for robot in self.robots if robot['name'] == robot_name)
            
            # Prepare reset state message
            initial_state = ModelState()
            initial_state.model_name = robot_name
            initial_state.reference_frame = 'world'
            
            # Set initial pose with the specified orientation
            x, y, yaw = robot_config['initial_pose']
            initial_state.pose.position.x = x
            initial_state.pose.position.y = y
            quaternion = quaternion_from_euler(0, 0, yaw)
            initial_state.pose.orientation = Quaternion(*quaternion)
            
            # Call the service to reset the robot's position
            self.set_model_state_service(initial_state)
            
            # Reset robot-specific internal states
            robot_state = self.robot_states[robot_name]
            robot_state['x'] = x
            robot_state['y'] = y
            robot_state['orientation'] = yaw
            robot_state['min_distance'] = float('inf')
            robot_state['prev_state'] = None
            robot_state['prev_action'] = None
            
            # Clear the robot's visualization
            self.visualizer.clear_visualization()
            
            return True
        except Exception as e:
            rospy.logerr(f"Error resetting {robot_name}: {e}")
            return False

    def decay_epsilon(self):
        """Decay epsilon value with a minimum threshold"""
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
        rospy.loginfo(f"Epsilon decayed to: {self.epsilon:.4f}")

    def move_robot(self, robot_name, action_idx):
        cmd_pub = self.robot_cmd_pubs[robot_name]
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = self.actions[action_idx]
        cmd_pub.publish(cmd)

    def odom_callback(self, msg, robot_name):
        # Update the robot's position and orientation based on odometry data
        robot_state = self.robot_states[robot_name]
        robot_state['x'] = msg.pose.pose.position.x
        robot_state['y'] = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        robot_state['orientation'] = yaw

    def scan_callback(self, msg, robot_name):
        # Update the robot's minimum distance based on laser scan data
        robot_state = self.robot_states[robot_name]
        robot_state['min_distance'] = min(msg.ranges)

    def run(self):
        # Sets the loop frequency to 10 Hz (i.e., it runs 10 times per second).
        rate = rospy.Rate(10)

        # Keeps running until ROS is stopped (e.g., you press Ctrl+C).
        while not rospy.is_shutdown():
            try:
                # Multi-robot learning loop
                for robot in self.robots:
                    try:
                        robot_name = robot['name']
                        current_state = self.discretize_state(robot_name)
                        reward = self.calculate_reward(robot_name)
                        
                        # Update visualizations for each robot
                        self.visualizer.publish_explored_cells(
                            self.robot_states[robot_name]['explored_grids']
                        )
                        self.visualizer.publish_q_values(
                            self.q_table[self.q_table['robot'] == robot_name]
                        )
                        self.visualizer.update_robot_path(
                            (self.robot_states[robot_name]['x'], 
                             self.robot_states[robot_name]['y'])
                        )

                        # Collision detection
                        if self.robot_states[robot_name]['min_distance'] < 0.15:
                            rospy.logwarn(f"{robot_name} collision detected! Resetting...")
                            self.reset_position(robot_name)
                            continue

                        # Q-table update
                        prev_state = self.robot_states[robot_name]['prev_state']
                        prev_action = self.robot_states[robot_name]['prev_action']
                        
                        if prev_state and prev_action is not None:
                            self.update_q_value(prev_state, prev_action, reward, current_state, robot_name)

                        # Action selection and execution
                        action_idx = self.choose_action(current_state)
                        self.move_robot(robot_name, action_idx)

                        # State tracking update
                        self.robot_states[robot_name]['prev_state'] = current_state
                        self.robot_states[robot_name]['prev_action'] = action_idx

                    except Exception as robot_error:
                        rospy.logerr(f"Error processing robot {robot_name}: {robot_error}")

                # Epsilon decay
                self.decay_epsilon()
                rate.sleep()

            except Exception as e:
                rospy.logerr(f"Multi-robot learning error: {e}")


if __name__ == '__main__':
    try:
        multi_robot = MultiRobotQLearning()
        multi_robot.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Multi-robot Q-learning terminated")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
