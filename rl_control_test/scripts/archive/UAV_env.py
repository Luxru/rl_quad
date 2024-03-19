import re
import time
from typing import List, Tuple
from loguru import logger
import numpy as np
import scipy
from scipy.spatial import HalfspaceIntersection,QhullError

#下面的这些根据你那边的需要来调试
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from quadrotor_msgs.msg import RLControl
from rl_control_test.srv import Sfc,SfcRequest,SfcResponse
from rl_control_test.msg import Polytope,Halfspace
from std_srvs.srv import Empty
from tf.transformations import quaternion_matrix

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35


class Hpolys:
    def __init__(self,hpolys:List[Polytope]) -> None:
        self.__hpolys = []
        for p in hpolys:
            hss = []
            for hs in p.halfspaces:
                hs:Halfspace
                hss.append([hs.h0,hs.h1,hs.h2,hs.h3])
            self.__hpolys.append(hss)
            self.__index = 0
    
    def __iter__(self):
        return iter(self.__hpolys)

    def __getitem__(self,index:int):
        if isinstance(index, slice):
            return self.__hpolys[index.start:index.stop:index.step]
        return self.__hpolys[index]

    def __len__(self):
        return len(self.__hpolys)
    
    def get_avg_dist(self,x,y,z,vx,vy,vz)->float:
        #ANGLE = 40
        halfspaces = self.__hpolys[self.__index]
        halfspaces = np.array(halfspaces)
        v1 = halfspaces[:,:-1]
        v2 = np.array([vx,vy,vz])
        p = np.array([x,y,z])
        # arccos [0, pi],when x>=0, arccos(x) in  [0, pi/2]
        #ang = np.arccos(np.dot(v1,v2)/(np.linalg.norm(v1,axis=1)*np.linalg.norm(v2)))*180/np.pi
        ang = np.arccos(np.dot(v1,v2)/(np.linalg.norm(v1,axis=1)*np.linalg.norm(v2)))
        # arctan2  [-pi, pi]
        # ang = np.arctan2(np.linalg.norm(np.cross(v1,v2),axis=1),np.dot(v1,v2))/np.pi*180    
        # i = np.where(np.abs(ang - 90)<ANGLE)[0]
        # if (len(i)==0):
        #     return 0
        dist = (halfspaces[:,:-1].dot(p)+halfspaces[:,-1])*np.sin(ang)/np.sqrt((halfspaces[:,:-1]*halfspaces[:,:-1]).sum(axis=1))
        avg_dist = -np.average(dist)
        assert not np.isnan(avg_dist)
        return avg_dist

    def is_in_hpolys(self,x,y,z)->bool:
        in_poly = False
        #logger.debug("Test if in convexhull")
        for index,halfspaces in enumerate(self.__hpolys):
            if in_poly:
                break
            halfspaces = np.array(halfspaces)
            p = np.array([x,y,z])
            ep = halfspaces[:,:-1].dot(p)+halfspaces[:,-1]
            in_hull = (ep<=0).all()
            if(not in_hull):
                i = np.where(ep>0)[0]
                # i:int
                dist = ep[i]/np.sqrt((halfspaces[i,:-1]*halfspaces[i,:-1]).sum(axis=1))
                if((dist<5e-1).all()):
                    in_hull = True
            if in_hull:
                self.__index = index
                in_poly = True
        return in_poly
   
class Quad:
    def __init__(self) -> None:
        # Launch the simulation with the given launchfile name
        rospy.init_node("gym", anonymous=True)
        # Set up the ROS publishers and subscribers
        # self.unpause = rospy.ServiceProxy("/quadrotor_simulator_so3/unpause_step", Empty)
        # self.pause = rospy.ServiceProxy("/quadrotor_simulator_so3/pause_step", Empty)
        self.__reset()
        # current-> next waypoint

        self.__nav_vector = np.ndarray(3)

        self.__quad_step = rospy.ServiceProxy("/quadrotor_simulator_so3/step", Empty)
        
        self.__so3cmd_pub = rospy.Publisher("/so3_cmd",RLControl,queue_size=50)
        self.__odom_sub = rospy.Subscriber("/state_ukf/odom",Odometry,self.__update_odom_callback,queue_size=10)
        self.__imu_sub = rospy.Subscriber("/quadrotor_simulator_so3/imu",Imu,self.__update_imu_callback)
        #self.cloud_sub = rospy.Subscriber("/local_cloud",PointCloud2,self.cloudCallback)
        self.__sfc_srv = rospy.ServiceProxy("/hpoly_srv",Sfc)

    def get_state(self):
        assert self.__isinit,"Get when state is not inited"
        orientation = self.__odom.pose.pose.orientation
        twist = self.__odom.twist.twist
        linear_acceleration = self.__imu.linear_acceleration
        return [self.__odom_x,self.__odom_y,self.__odom_z,#位置（3维）
                orientation.x,orientation.y,orientation.z,orientation.w,#姿态4元素（4维）
                twist.linear.x,twist.linear.y,twist.linear.z,#速度（3维）
                linear_acceleration.x,linear_acceleration.y,linear_acceleration.z,#加速度（3维）
                self.__nav_vector[0],self.__nav_vector[1],self.__nav_vector[2],#路径点和目标位置的差值（3维）
                self.__goal_x,self.__goal_y,self.__goal_z #目标点的位置（3维）
                ]

    def __reset(self):
        self.__isinit = False
        self.__odom:Odometry = None
        self.__imu:Imu = None
        self.__sfc_res:SfcResponse = None
        self.__hpolys:Hpolys = None 
        self.__route:List[Odometry] = None


        #start->1->2->3->end
        self.__current_index = 1

        self.__odom_x = 0
        self.__odom_y = 0
        self.__odom_z = 0

        self.__goal_x = 0.0
        self.__goal_y = 0.0
        self.__goal_z = 0.0


    def __get_cmd(self,action)->RLControl:
        command = RLControl()
        command.force = action[0]
        command.m1 = action[1]
        command.m2 = action[2]
        command.m3 = action[3]
        return command
    
    def __update_sfc(self,res:SfcResponse):
        self.__sfc_res = res
        self.__hpolys = Hpolys(self.__sfc_res.polys)
        self.__route:List[Point] = self.__sfc_res.route
        goal:Point = self.__route[-1]
        self.__goal_x = goal.x
        self.__goal_y = goal.y
        self.__goal_z = goal.z
        self.__update_state()

    def __update_odom_callback(self,msg:Odometry):
        self.__odom = msg
        self.__odom_x = self.__odom.pose.pose.position.x
        self.__odom_y = self.__odom.pose.pose.position.y
        self.__odom_z = self.__odom.pose.pose.position.z
        self.__update_state()

    def __update_imu_callback(self,msg:Imu):
        self.__imu = msg
        self.__update_state()

    def __update_state(self)->None:
        self.__isinit = self.__imu != None and self.__odom !=None and self.__sfc_res != None
        if(not self.__isinit):
            return
        if not (self.__current_index<len(self.__route)):
            #reach goal
            return
        current_goal:Point = self.__route[self.__current_index]
        self.__nav_vector = np.array([
            current_goal.x-self.__odom_x,
            current_goal.y-self.__odom_y,
            current_goal.z-self.__odom_z
        ])
        if(np.sqrt((self.__nav_vector*self.__nav_vector.T).sum())<GOAL_REACHED_DIST):
            self.__current_index+=1
            return

    def ready(self)->bool:
        return self.__isinit

    def __dist_between_route(self,i:int,j:int)->float:
        assert self.__isinit,"Get when state is not inited"
        assert i>=0
        assert j<len(self.__route)
        assert i<j
        p:Point = self.__route[i]
        q:Point = self.__route[j]
        v = np.array([q.x-p.x,q.y-p.y,q.z-p.z])
        return np.sqrt(np.dot(v,v))

    def get_goal_index(self)->int:
        assert self.__isinit,"Get when state is not inited"
        return len(self.__route)-1

    def get_current_index(self)->int:
        assert self.__isinit,"Get when state is not inited"
        return self.__current_index

    def get_waypoint(self,index:int)->Tuple[float,float,float]:
        assert self.__isinit,"Get when state is not inited"
        assert index >= 0 
        assert index < len(self.__route)
        p = self.__route[index]
        return (p.x,p.y,p.z)

    def get_current_pose(self)->Tuple[float,float,float]:
        assert self.__isinit,"Get when state is not inited"
        return (self.__odom_x,self.__odom_y,self.__odom_z)

    def get_current_orientation(self)->Tuple[float,float,float,float]:
        assert self.__isinit,"Get when state is not inited"
        ori = self.__odom.pose.pose.orientation
        return (ori.x,ori.y,ori.z,ori.w)
    
    def get_current_progress(self)->Tuple[float,float]:
        #return [0,1]
        assert self.__isinit,"Get when state is not inited"
        goal_vec = np.array([self.__goal_x-self.__odom_x,
                        self.__goal_y-self.__odom_y,
                        self.__goal_z-self.__odom_z
                        ])
        quad_goal_dist:float = np.sqrt(np.dot(goal_vec,goal_vec))
        goal_dist = self.__dist_between_route(0,self.get_goal_index())
        quad_waypoint_dist:float  = np.sqrt(np.dot(self.__nav_vector,self.__nav_vector))
        waypoint_dist = self.__dist_between_route(self.__current_index-1,self.__current_index)
        goal_progress = 1-quad_goal_dist/goal_dist
        waypoint_progress =1- quad_waypoint_dist/waypoint_dist
        return (quad_goal_dist,quad_waypoint_dist)
        return (goal_progress,waypoint_progress)

    def get_current_sfc_dist(self)->float:
        assert self.__isinit,"Get when state is not inited"
        sfc_dist = self.__hpolys.get_avg_dist(self.__odom_x,self.__odom_y,self.__odom_z,self.__nav_vector[0],self.__nav_vector[1],self.__nav_vector[2])
        return sfc_dist

    def get_current_velocity(self)->float:
        assert self.__isinit,"Get when state is not inited"
        v = np.array([self.__odom.twist.twist.linear.x,self.__odom.twist.twist.linear.y,self.__odom.twist.twist.linear.z])
        return np.sqrt(np.dot(v,v))

    def get_current_body_rate(self)->float:
        assert self.__isinit,"Get when state is not inited"
        w = np.array([self.__imu.angular_velocity.x,self.__imu.angular_velocity.y,self.__imu.angular_velocity.z])
        return np.sqrt(np.dot(w,w))

    def get_current_nav_vector(self):
        assert self.__isinit,"Get when state is not inited"
        return self.__nav_vector

    def act(self,action)->None:
        self.__so3cmd_pub.publish(self.__get_cmd(action))
        return

    def observe_collision(self):
        #out_poly = not self.__hpolys.is_in_hpolys(self.__odom_x,self.__odom_y,self.__odom_z)
        crash = self.__odom_z<=1e-1
        return crash
        if out_poly:
            logger.warning("Out convexhull")
        if crash:
            logger.warning("Crash detected")
        return out_poly or crash

    def done(self)->bool:
        v = np.array([
            (self.__goal_x-self.__odom_x),
            (self.__goal_y-self.__odom_y),
            (self.__goal_z-self.__odom_z),
        ])
        return np.sqrt((v*v.T).sum())<GOAL_REACHED_DIST

    def step(self):
        pass
        # rospy.wait_for_service('/quadrotor_simulator_so3/step')
        # try:
        #     self.__quad_step()
        # except (rospy.ServiceException) as e:
        #     print("/quadrotor_simulator_so3/step service call failed")
    

    def reset(self)->None:
        #1、随机刷新地图，每20次刷新一次；
        #2、随机生成目标点和起始位置，z设置为0，起始位置到目标位置的距离要大于10米；
        #3、执行全局路径规划，并生成一系列的路径点（位置）；
        #4、初始化无人机的状态信息，实时获取状态信息，位置（3维）、姿态4元素（4维）、速度（3维）、加速度（3维）；
        #（1）路径点和目标位置的差值（3维）、 路径点的位置（3维）；
        #（2）将多面体作为环境的感知信息，以便无人机可以实时的观测周围的信息；
        #5、
        rospy.wait_for_service('/hpoly_srv')
        try:
            sfc_req = SfcRequest()
            sfc_res:SfcResponse = self.__sfc_srv(sfc_req)
            self.__reset()
            if sfc_res.ret_code == sfc_res.ret_success:
                self.__update_sfc(sfc_res)
            else:
                raise rospy.ServiceException(f"Invalid return value: ret_code{sfc_res.ret_code}")
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))


#需要参考港科大那边中，获取状态信息，位置（3维）、姿态4元素（4维）、速度（3维）、加速度（3维）
#动作是；力（1维），角速度（3维）
#奖励函数要跟能耗、碰撞、通行时间有关
class UAVEnv:
    """Superclass for all Gazebo environments."""

    def __init__(self):
        self.quad = Quad()
        self.__last_reward_index = 1
        self.__last_dist = (0,0)
        self.__last_vel = 0
        self.__last_w = 0

    # def Planner_global(self):
    #     #应该获得一些列的全局路径点，并给出第一个路径点的位置，捕获的全局路径点是否包含姿态呢？
    #     x, y, z = 0, 0, 0
    #     return x, y, z

    # Perform an action and read a new state
    def step(self, action):
        self.quad.act(action)
        self.quad.step()

        collision = self.quad.observe_collision()
        target = self.quad.done()

        #Input: 需要修改
        state = self.quad.get_state()

        #!TODO 
        reward = self.get_reward(target,collision)
        return state, reward, collision or target 

    def reset(self):
        self.quad.reset()  
        self.quad.step()
        #位置、姿态（四元素）、速度、角速度、加速度，目标点的位置差异、角度差异(这个怎么定义呢？)、如果沿着
        while(not self.quad.ready()):
            time.sleep(0.1)
        state = self.quad.get_state()
        return state

    #这个我来设置
    def get_reward(self,target, collision):
        current_index = self.quad.get_current_index()
        goal_index = self.quad.get_goal_index()
        sfc_dist = self.quad.get_current_sfc_dist()
        goal_progress,waypoint_progress= self.quad.get_current_progress()
        vel = self.quad.get_current_velocity()
        w = self.quad.get_current_body_rate()
        px,py,pz = self.quad.get_current_pose()
        ox,oy,oz,ow = self.quad.get_current_orientation()
        
        #计算机头与下一个目标点的朝向
        vq = None
        if (current_index==goal_index):
            vq = self.quad.get_current_nav_vector()
        else:
            v = self.quad.get_waypoint(current_index+1)
            vq = np.array([v[0]-px,v[1]-py,v[2]-pz])
        R = quaternion_matrix([ox,oy,oz,ow])[:3,:3]
        vx = R.dot(np.array([1,0,0]))
        ang = np.arccos(np.dot(vx,vq)/(np.linalg.norm(vx)*np.linalg.norm(vq)))

        #ang = ang*180/np.pi
        la,lb = 1,0
        r_prog = la*(self.__last_dist[0]-waypoint_progress)+lb*(self.__last_dist[1]-goal_progress)

        #希望角度小
        lc,ld = 0.02,-10
        r_perc =  lc*np.exp(ld*ang)

        #希望不突变
        le,lf = -2e-4,-14-4
        r_cmd = le*np.abs(w-self.__last_w)+lf*np.abs(vel-self.__last_vel)

        
        lg,lh = 1,-1
        #r_crash = 5 if collision else lg*np.exp(lh*(sfc_dist+pz))
        r_crash = 5 if collision else 0
        r_target = 20 if target else 0
        r_pass = 10 if self.__last_reward_index!=current_index else 0

        #reward = r_prog+r_perc+r_cmd+r_target+r_pass-r_crash
        reward = r_prog+r_perc+r_cmd - r_crash
        self.__last_reward_index = current_index
        self.__last_dist = (waypoint_progress,goal_progress)
        self.__last_vel = vel
        self.__last_w = w

        assert not np.isnan(reward).any()
        return reward
        
