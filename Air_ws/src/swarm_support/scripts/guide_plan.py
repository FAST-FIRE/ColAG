#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float64, Float32MultiArray
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point, Twist
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from visualization_msgs.msg import Marker, MarkerArray
from custom_msgs.msg import blind_info

import numpy as np
import math
import tf
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


class GuideManager():

    def __init__(self):
        self.drone_id_ = rospy.get_param('~drone_id', 0)
        self.ugv_num_ = rospy.get_param('~ugv_num', 1)
        self.uav_num_ = rospy.get_param('~uav_num', 1)
        self.replan_period_ = rospy.get_param('~replan_period', 1.0)
        self.v_max_ = rospy.get_param('~v_max', 1.0)
        self.a_max_ = rospy.get_param('~a_max', 1.0)
        self.time_reslolution_ = rospy.get_param('~time_reslolution', 100)

        self.data = {}

        self.collision_id_ = []
        self.collision_pos_ = np.zeros([self.ugv_num_, 3])
        # self.collision_pos_[0,:] = np.array([-2,0,0])
        # self.collision_pos_[1,:] = np.array([3,-1,0])
        # self.collision_pos_[2,:] = np.array([4,1,0])
        # self.collision_pos_[3,:] = np.array([5,2,0])
        # self.collision_pos_[4,:] = np.array([3,4,0])

        self.collision_time_ = np.zeros(self.ugv_num_)
        # self.collision_time_ = [1200,900,700,800,500]

        self.record_id_ = np.zeros(self.ugv_num_).astype(int)
        # self.record_id_[2] = 1

        self.waypoints_ = np.zeros([self.ugv_num_, 3])
        self.pos_ = np.zeros(3)
        self.vel_ = np.zeros(3)

        # self.timer_replan_ = rospy.Timer(rospy.Duration(self.replan_period_), self.guidePlanCallback)
        self.ugv_info_sub_ = rospy.Subscriber('blind_info',
                                              blind_info,
                                              self.UGVinfoCallback,
                                              queue_size=200)
        self.self_odom_sub_ = rospy.Subscriber('self_odom',
                                               Odometry,
                                               self.selfOdomCallback,
                                               queue_size=10,tcp_nodelay=True)

        self.waypoint_pub_ = rospy.Publisher('guide_waypoint',
                                             Path,
                                             queue_size=10)
        self.waypoint_marker_pub_ = rospy.Publisher('guide_waypoint_visual',
                                                    Marker,
                                                    queue_size=10)
        self.plan_flag_ = False
        # goal_State

        # self.goal_pub = rospy.Publisher'(goal_state', MarkerArray, queue_size=10)
        # if(self.putn_id>0):
        #     sub_topic_name = "/putn_" +str(0) + "/gpr_path/surf_predict_pub";
        #     self.__sub_multiglobalpath = rospy.Subscriber( sub_topic_name ,Float32MultiArray, self.__multiglobalpath_cb, queue_size=10)
        # else:
        #     self.__sub_goal_state = rospy.Subscriber('gpr_path/surf_predict_pub', Float32MultiArray, self._global_path_callback2, queue_size=10)

    def computeTimeCost(self, c1, c2):
        distance = (c1[0] - c2[0]) * (c1[0] - c2[0]) + (c1[1] - c2[1]) * (
            c1[1] - c2[1])  #+(c1[2]-c2[2])*(c1[2]-c2[2])
        distance = np.sqrt(distance)
        time = distance / self.v_max_
        return int(time * self.time_reslolution_)

    #compute cost accoding to the current speed
    def computeTimeCostFromNow(self, c1):
        distance = (c1[0] - self.pos_[0]) * (c1[0] - self.pos_[0]) + (
            c1[1] - self.pos_[1]) * (c1[1] - self.pos_[1])  #+(c1[2]-self.pos_[2])*(c1[2]-self.pos_[2])
        distance = np.sqrt(distance)
        if distance == 0:
            return 0
        #compute Tangential component ,consider the time to max speed
        direct = (c1[0:1] - self.pos_[0:1]) / distance
        v_tangent = self.vel_[0:1].dot(direct)
        time1 = (self.v_max_ - v_tangent) / self.a_max_
        distance1 = (self.v_max_ * self.v_max_ -
                     v_tangent * v_tangent) / 2 * self.a_max_
        if (distance1 > distance):
            time = (np.sqrt(v_tangent * v_tangent + 2 * self.a_max_ * distance)
                    - v_tangent) / self.a_max_
        else:
            time2 = (distance - distance1) / self.v_max_
            time = time1 + time2
        return int(time * self.time_reslolution_)

    def quart_to_rpy(self, x, y, z, w):
        r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        p = math.asin(2 * (w * y - z * x))
        y = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return r, p, y

    def updateTimeMatrix(self):
        # num = record_id_.sum();
        num = len(self.collision_id_)
        self.data['time_matrix'] = np.zeros([num + 1, num + 1]).astype(int)
        for i in range(num + 1):
            for j in range(i, num + 1):
                if i == j:
                    self.data['time_matrix'][i][j] = 0
                    # print("0\n")
                elif i < j:
                    if i == 0:
                        time = self.computeTimeCostFromNow(self.collision_pos_[self.collision_id_[j - 1]])
                        self.data['time_matrix'][i][j] = time
                    else:
                        time = self.computeTimeCost(
                            self.collision_pos_[self.collision_id_[i - 1]],
                            self.collision_pos_[self.collision_id_[j - 1]])
                        # print("%f\n"%(time))
                        self.data['time_matrix'][i][j] = time
                        self.data['time_matrix'][j][i] = time
        # print("time_matrix::::::::::::::::::::::::::::")
        # print(self.data['time_matrix'])

    def updateTimeConstraint(self):
        num = len(self.collision_id_)
        self.data['time_windows'] = np.zeros([num + 1, 2]).astype(int)
        time_now = rospy.Time.now()
        # print(time_now)
        for i in range(num):
            self.data['time_windows'][i + 1][1] = int((self.collision_time_[self.collision_id_[i]] - time_now.to_sec()) * self.time_reslolution_)
        
        # for i in range(num):
        #     self.data['time_windows'][i + 1][1] = 10001

        # print("time_windows:::::::::::::::::::::::::::")
        # print(self.data['time_windows'])

    def drawWaypoints(self):
        waypoints_visual = Marker()
        waypoints_visual.header.frame_id = 'world'
        waypoints_visual.header.stamp = rospy.Time.now()
        waypoints_visual.action = Marker.ADD
        waypoints_visual.type = Marker.SPHERE_LIST
        waypoints_visual.id = 0
        waypoints_visual.color.r = 0.0
        waypoints_visual.color.g = 0.0
        waypoints_visual.color.b = 0.5
        waypoints_visual.color.a = 1
        waypoints_visual.scale.x = 0.5
        waypoints_visual.scale.y = 0.5
        waypoints_visual.scale.z = 0.5
        waypoints_visual.pose.orientation.w = 1.0
        num = len(self.collision_id_)
        for i in range(num):
            x = self.collision_pos_[self.collision_id_[i]][0]
            y = self.collision_pos_[self.collision_id_[i]][1]
            z = self.collision_pos_[self.collision_id_[i]][2]
            pt = Point(x, y, z)
            waypoints_visual.points.append(pt)
        self.waypoint_marker_pub_.publish(waypoints_visual)

    def pubWaypoints(self):
        waypoints_msg = Path()
        waypoints_msg.header.frame_id = 'world'
        waypoints_msg.header.stamp = rospy.Time.now()
        rows, cols = self.waypoints_.shape
        if rows > 0:
            for i in range(rows):
                temp_pose_msg = PoseStamped()
                temp_pose_msg.pose.position.x = self.waypoints_[i][0]
                temp_pose_msg.pose.position.y = self.waypoints_[i][1]
                temp_pose_msg.pose.position.z = self.waypoints_[i][2]
                temp_pose_msg.header.stamp = rospy.Time.from_sec(
                    (rospy.Time.now().to_sec() + i))
                temp_pose_msg.header.frame_id = "world"
                waypoints_msg.poses.append(temp_pose_msg)
            self.waypoint_pub_.publish(waypoints_msg)

    def guidePlanCallback(self):

        def timeCallback(from_index, to_index):
            """Returns the travel time between the two nodes."""
            # Convert from routing variable Index to time matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            time = self.data['time_matrix'][from_node][to_node]
            # print("%dto%dtime%f"%(from_node,to_node,time))
            return time

        if (self.plan_flag_):
            start_time = rospy.Time.now()

            time_bias = 0
            last_id = -1
            for i in range(len(self.collision_id_)):
                if (self.data['time_windows'][i+1][1] <= time_bias):
                    time_bias = self.data['time_windows'][i+1][1]
                    last_id = i

            # print ("time_bias::::::::",time_bias,"last_id::::::::",last_id)
            # print("time_matrix::::::::::::::::::::::::::::")
            # print(self.data['time_matrix'])
            # print("time_windows:::::::::::::::::")
            # print(self.data['time_windows'])

            if(time_bias <= 0 and last_id >= 0):
                time_bias = self.computeTimeCostFromNow(self.collision_pos_[self.collision_id_[last_id]]) - time_bias
                print("time_bias::::::::",time_bias,"last_id::::::::",last_id)
                for i in range(len(self.collision_id_)):
                    self.data['time_windows'][i+1][1] = self.data['time_windows'][i+1][1] + time_bias + 1
            
            iter_time = 0
            solution_flag = False
            while (iter_time < 100):
                print("iter time::::::::::::::::::::::::::::",iter_time)
                print("time_matrix::::::::::::::::::::::::::::")
                print(self.data['time_matrix'])
                print("time_windows:::::::::::::::::")
                print(self.data['time_windows'])

                manager = pywrapcp.RoutingIndexManager(
                    len(self.data['time_matrix']), self.data['num_vehicles'],
                    self.data['depot'])
                routing = pywrapcp.RoutingModel(manager)

                transit_callback_index = routing.RegisterTransitCallback(timeCallback)

                routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
                time = 'Time'
                routing.AddDimension(
                    transit_callback_index,
                    30000,  # allow waiting time
                    500000,  # maximum time per vehicle
                    False,  # Don't force start cumul to zero.
                    time)
                time_dimension = routing.GetDimensionOrDie(time)

                # Add time window constraints for each location except depot.
                for location_idx in range(len(self.data['time_windows'])):
                    if location_idx == self.data['depot']:
                        continue
                    index = manager.NodeToIndex(location_idx)
                    time_dimension.CumulVar(index).SetRange(
                        int(self.data['time_windows'][location_idx][0]),
                        int(self.data['time_windows'][location_idx][1]))
                # Add time window constraints for each vehicle start node.
                depot_idx = self.data['depot']
                for vehicle_id in range(self.data['num_vehicles']):
                    index = routing.Start(vehicle_id)
                    time_dimension.CumulVar(index).SetRange(
                        int(self.data['time_windows'][depot_idx][0]),
                        int(self.data['time_windows'][depot_idx][1]))
                # [END time_windows_constraint]

                for i in range(self.data['num_vehicles']):
                    routing.AddVariableMinimizedByFinalizer(
                        time_dimension.CumulVar(routing.Start(i)))
                    routing.AddVariableMinimizedByFinalizer(
                        time_dimension.CumulVar(routing.End(i)))
                    # Setting first solution heuristic.

                # [START parameters]
                search_parameters = pywrapcp.DefaultRoutingSearchParameters()
                search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
                # [END parameters]

                # Solve the problem.
                # [START solve]
                solution = routing.SolveWithParameters(search_parameters)
                # [END solve]

                if solution:
                    # plan_output ='[guide Planner] solved in %f sec'%((end_time-start_time).to_sec())
                    self.getSolution(self.data, manager, routing, solution)
                    self.drawWaypoints()
                    self.pubWaypoints()
                    solution_flag = True
                    break
                else:
                    iter_time = iter_time + 1
                    for i in range(len(self.collision_id_)):
                        self.data['time_windows'][i+1][1] = int(self.data['time_windows'][i+1][1] * 1.5 + 0.999)

            if(not solution_flag):
                print("no solution")
            else:
                end_time = rospy.Time.now()
                rospy.loginfo('[guide Planner] solved in %f sec' %((end_time - start_time).to_sec()))
            self.plan_flag_ = False

    def getSolution(self, data, manager, routing, solution):
        time_dimension = routing.GetDimensionOrDie('Time')
        total_time = 0
        num = len(self.collision_id_)
        self.waypoints_ = np.zeros([num, 3])

        for vehicle_id in range(data['num_vehicles']):
            index = routing.Start(vehicle_id)
            plan_output = 'Route for vehicle %d:\n' % (vehicle_id)
            i = -1
            while not routing.IsEnd(index):
                if i >= 0:
                    time_var = time_dimension.CumulVar(index)
                    plan_output += '%d Time(%.3f,%.3f) -> ' % (
                        self.collision_id_[manager.IndexToNode(index) - 1],
                        solution.Min(time_var), solution.Max(time_var))
                    # print(manager.IndexToNode(index))
                    self.waypoints_[i] = self.collision_pos_[self.collision_id_[manager.IndexToNode(index) - 1]]
                i = i + 1
                index = solution.Value(routing.NextVar(index))

            time_var = time_dimension.CumulVar(index)

            plan_output += 'Time of the route: {}s'.format(
                solution.Min(time_var) / 100.0)
            print(plan_output)
            total_time += solution.Min(time_var)
        # print('Total time of all routes: %f min' % (total_time))

    def UGVinfoCallback(self, data):
        ###make the matrix dynamically change with new id info
        id = data.id
        # print(id)
        # print(self.record_id_)
        self.data = {}
        self.data['num_vehicles'] = self.uav_num_
        self.data['depot'] = 0

        if (self.plan_flag_ == True):
            return
        if data.collision_time == -1:  #it means no collis with this id
            self.record_id_[id] = 0
        else:
            self.collision_time_[id] = data.collision_time + data.header.stamp.to_sec()
            self.collision_pos_[id, :] = np.array([data.collision_x, data.collision_y, 1.0])
            self.record_id_[id] = 1

        self.collision_id_ = []
        for i in range(np.size(self.record_id_)):
            if self.record_id_[i] == 1:
                self.collision_id_.append(i)

        if(len(self.collision_id_) == 0):
            return
        
        # incrementally change the cost and constraint, matrix size is the ugv is needed to serve
        self.updateTimeMatrix()
        self.updateTimeConstraint()

        # self.drawWaypoints()
        self.plan_flag_ = True
        self.guidePlanCallback()

    def selfOdomCallback(self, data):
        self.pos_[0] = data.pose.pose.position.x
        self.pos_[1] = data.pose.pose.position.y
        self.pos_[2] = data.pose.pose.position.z
        self.vel_[0] = data.twist.twist.linear.x
        self.vel_[1] = data.twist.twist.linear.y
        self.vel_[2] = data.twist.twist.linear.z


if __name__ == '__main__':
    rospy.init_node("guide_planner")
    guide_planner = GuideManager()
    rospy.spin()
