#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32MultiArray, ColorRGBA
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
import threading
import numpy as np
from visualization_msgs.msg import Marker
import casadi as ca


def distance_global(c1, c2):
    return np.sqrt((c1[0] - c2[0]) * (c1[0] - c2[0]) + (c1[1] - c2[1]) * (c1[1] - c2[1]))


class Local_Planner():
    def __init__(self):
        # 从ROS参数服务器获取重规划周期参数：
        # /local_planner/replan_period：参数的完整名称
        # /local_planner：命名空间
        # replan_period：具体参数名
        # 0.05：默认值（50ms）
        self.replan_period = rospy.get_param('/local_planner/replan_period', 0.05)

        self.z = 0
        self.N = 25
        self.goal_state = np.zeros([self.N, 2])# N×2的零矩阵

        self.curr_state = None
        self.global_path = None

        self.last_input = []
        self.last_state = []
        self.mpc_success = None
        self.ob = []


        #线程锁初始化
        self.curr_pose_lock = threading.Lock()
        self.global_path_lock = threading.Lock()
        self.obstacle_lock = threading.Lock()


        #发布、订阅
        self.__timer_replan = rospy.Timer(rospy.Duration(self.replan_period), self.__replan_cb)

        self.__sub_curr_state = rospy.Subscriber('/curr_state', Float32MultiArray, self.__curr_pose_cb, queue_size=10)

        self.__sub_obs = rospy.Subscriber('/obs_predict_pub', Float32MultiArray, self.__obs_cb, queue_size=10)

        self.__sub_goal = rospy.Subscriber('/global_path', Path, self.__global_path_cb, queue_size=25)


        self.__pub_local_path = rospy.Publisher('/local_path', Path, queue_size=10)

        self.__pub_local_plan = rospy.Publisher('/local_plan', Float32MultiArray, queue_size=10)

        self.__pub_start = rospy.Publisher('/cmd_move', Bool, queue_size=10)

    def __replan_cb(self, event):#event由rospy.Timer传入
        if self.choose_goal_state():# 选择目标状态，如果成功返回True

            states_sol, input_sol = self.MPC_ellip()#调用mpc

            # 创建并发布移动命令
            cmd_move = Bool()
            # 如果距离终点大于0.1米，则继续移动
            cmd_move.data = distance_global(self.curr_state, self.global_path[-1]) > 0.1
            self.__pub_start.publish(cmd_move)
            self.__publish_local_plan(input_sol, states_sol)#调用__publish_local_plan函数，发布预测状态可视化

    def __curr_pose_cb(self, msg):
        self.curr_pose_lock.acquire()
        self.curr_state = np.zeros(2)
        self.curr_state[0] = msg.data[0]
        self.curr_state[1] = msg.data[1]
        self.curr_pose_lock.release()


    def __obs_cb(self, msg):
        self.obstacle_lock.acquire()
        self.ob = []# 清空障碍物列表，准备接收新的障碍物数据
        size = int(len(msg.data) / 5)#len(msg.data)是5*障碍物数量*N（卡尔曼滤波预测步数）
        for i in range(size):
            self.ob.append(np.array(msg.data[5*i:5*i+5]))
        self.obstacle_lock.release()


    def __global_path_cb(self, path):
        self.global_path_lock.acquire()
        size = len(path.poses)# 获取路径点的数量
        if size > 0:# 如果路径不为空
            self.global_path = np.zeros([size, 2])# 创建一个size×3的零矩阵存储路径点
            for i in range(size):
                self.global_path[i, 0] = path.poses[i].pose.position.x
                self.global_path[i, 1] = path.poses[i].pose.position.y
        self.global_path_lock.release()

    def __publish_local_plan(self, input_sol, state_sol):
        local_path = Path()# 创建一个空的Path消息对象，nav_msgs.msg.Path
        local_plan = Float32MultiArray()# 创建一个空的Float32MultiArray(浮点数数组)消息对象，std_msgs.msg.Float32MultiArray
        
        
        local_path.header.stamp = rospy.Time.now()

        local_path.header.frame_id = "world"


        for i in range(self.N):
            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = state_sol[i, 0]
            this_pose_stamped.pose.position.y = state_sol[i, 1]
            this_pose_stamped.pose.position.z = self.z
            this_pose_stamped.pose.orientation.x = 0
            this_pose_stamped.pose.orientation.y = 0
            this_pose_stamped.pose.orientation.z = 0
            this_pose_stamped.pose.orientation.w = 1
            this_pose_stamped.header.seq = i
            this_pose_stamped.header.stamp = rospy.Time.now()
            this_pose_stamped.header.frame_id = "world"
            #把每一个预测的状态装到local_path里面去，应该是要在rviz中显示
            local_path.poses.append(this_pose_stamped)

            for j in range(2):
                if len(input_sol[i]) != 0:
                    local_plan.data.append(input_sol[i][j])
                    #local_plan.data是1*50的一维数组
                    #把每一个预测的输入装到local_plan里面去,传给controller.py,由其发布第一个输入

        self.__pub_local_path.publish(local_path)
        self.__pub_local_plan.publish(local_plan)

    def choose_goal_state(self):
        self.curr_pose_lock.acquire()
        self.global_path_lock.acquire()

        #用来检查是否已经收到全局路径和当前状态
        if self.global_path is None or self.curr_state is None:
            self.curr_pose_lock.release()
            self.global_path_lock.release()
            return False# 返回False表示无法选择目标状态

        waypoint_num = self.global_path.shape[0]# shape[0]返回矩阵的行数，即路径点数量

        # 找到距离当前位置最近的路径点
        # 使用列表推导式计算当前位置到每个路径点的距离,np.argmin返回最小值的索引，即最近点的索引
        num = np.argmin(np.array([distance_global(self.curr_state, self.global_path[i]) for i in range(waypoint_num)]))

        scale = 1
        num_list = []
        for i in range(self.N):
            num_path = min(waypoint_num - 1, num + i * scale)
            self.goal_state[i] = self.global_path[num_path]

        self.curr_pose_lock.release()
        self.global_path_lock.release()
        return True# 返回True表示成功选择了目标状态


# - exceed_ob 和 h 都是定义在 MPC_ellip 函数内部的嵌套函数
# - 这样做的原因和好处是：
# - 封装性：这些函数只在 MPC_ellip 中使用，不需要被其他函数调用
# - 访问作用域：内部函数可以访问外部函数的变量（比如 self 、 opti 等）
    def MPC_ellip(self):
        self.curr_pose_lock.acquire()
        self.global_path_lock.acquire()
        self.obstacle_lock.acquire()

        opti = ca.Opti()# 创建优化问题实例
        # 采样时间
        T = 0.1
        #CBF约束参数
        gamma_k = 0.3

        v_max = 1.2#最大速度
        v_min = -1.2#最小速度
        

        # 创建一个2维的参数向量，用于表示机器人的初始状态 [x, y]
        # 后面通过opti.set_value(opt_x0, self.curr_state)来赋值
        opt_x0 = opti.parameter(2)


        # 在 opt_states = opti.variable(self.N + 1, 2) 这行代码中：
        # 1. 创建了一个 (N+1) × 2 的决策变量矩阵
        # 2. 这不是简单的空矩阵，而是告诉优化器：
        # - 这些变量的值需要通过求解优化问题来确定
        # - 它们是优化过程中要寻找的最优解的一部分

        # opt_states 存放的是预测的机器人状态序列
        # - 是一个 (N+1) × 2 的矩阵
        # - N+1 行：表示从当前时刻到未来 N 个时刻的状态
        # opt_states = [
        #     [x₀, y₀],  # 当前状态（t=0）
        #     [x₁, y₁],  # 第1个预测状态（t=1）
        #     [x₂, y₂],  # 第2个预测状态（t=2）
        #     ...,
        #     [xₙ, yₙ]   # 第N个预测状态（t=N）
        # ]
        opt_states = opti.variable(self.N + 1, 2)
        opt_controls = opti.variable(self.N, 2)
        vx = opt_controls[:, 0]
        vy = opt_controls[:, 1]

        #状态空间方程
        def f(x_, u_): return ca.vertcat(u_[0], u_[1])

        #判断障碍物是否需要避开
        def exceed_ob(ob_):
            l_long_axis = ob_[2]
            l_short_axis = ob_[3]
            long_axis = np.array([np.cos(ob_[4]) * l_long_axis, np.sin(ob_[4]) * l_long_axis])

            ob_vec = np.array([ob_[0], ob_[1]])
            center_vec = self.goal_state[self.N-1] - ob_vec
            dist_center = np.linalg.norm(center_vec)
            cos_ = np.dot(center_vec, long_axis.T) / (dist_center * l_long_axis)

            if np.abs(cos_) > 0.1:
                tan_square = 1 / (cos_ ** 2) - 1
                d = np.sqrt((l_long_axis ** 2 * l_short_axis ** 2 * (1 + tan_square) / (
                    l_short_axis ** 2 + l_long_axis ** 2 * tan_square)))
            else:
                d = l_short_axis

            cross_pt = ob_vec + d * center_vec / dist_center

            vec1 = self.goal_state[self.N-1] - cross_pt
            vec2 = self.curr_state - cross_pt
            theta = np.dot(vec1.T, vec2)#向量点积

            return theta > 0 #返回值为True和False，True表示不需要避开，False表示需要避开

        #障碍物CBF约束
        # - now_state_ ：当前位置状态
        # - 类型：CasADi符号变量
        # - 维度：2维向量 [x, y]
        # - 含义：机器人的当前位置

        # - ob_ ：障碍物参数
        # - 类型：CasADi符号变量
        # - 维度：5维向量 [x, y, a, b, θ]
        # - 含义：
        # - ob_[0], ob_[1]：障碍物中心位置坐标 (x, y)
        # - ob_[2]：长轴长度 a
        # - ob_[3]：短轴长度 b
        # - ob_[4]：椭圆旋转角度 θ
        def h(now_state_, ob_):
            
            safe_dist = 0.5 

            c = ca.cos(ob_[4])
            s = ca.sin(ob_[4])
            a = ca.MX([ob_[2]])
            b = ca.MX([ob_[3]])

            ob_vec = ca.MX([ob_[0], ob_[1]])
            center_vec = now_state_ - ob_vec.T

            dist = b * (ca.sqrt((c ** 2 / a ** 2 + s ** 2 / b ** 2) * center_vec[0] ** 2 + (s ** 2 / a ** 2 + c ** 2 / b ** 2) *
                                center_vec[1] ** 2 + 2 * c * s * (1 / a ** 2 - 1 / b ** 2) * center_vec[0] * center_vec[1]) - 1) - safe_dist
            
            return dist

        # - 函数输入：

        # - x : 状态向量或控制向量
        # - A : 权重矩阵（通常是对角矩阵）
        # - 函数计算： x^T * A * x

        # - ca.mtimes 是CasADi库的矩阵乘法函数
        # - 返回一个标量值
        def quadratic(x, A):
            return ca.mtimes([x, A, x.T])

        #初值约束，，，，不知道为什么要给一个初值约束
        # ini.T 是 NumPy/CasADi 中表示矩阵转置（transpose）的操作符。
        # 1. opt_x0 是一个 3×1 的列向量（因为是通过 opti.parameter(3) 创建的）
        # 2. .T 将其转置为 1×3 的行向量
        opti.subject_to(opt_states[0, :] == opt_x0.T)

        # 对输入vx和vy的约束
        opti.subject_to(opti.bounded(v_min, vx, v_max))
        opti.subject_to(opti.bounded(v_min, vy, v_max))

        #状态转移矩阵，这个库是写成约束
        for i in range(self.N):
            x_next = opt_states[i, :] + T * f(opt_states[i, :], opt_controls[i, :]).T
            opti.subject_to(opt_states[i + 1, :] == x_next)

        # - self.ob 是一个列表，存储了所有障碍物的预测轨迹信息
        # - self.N 是预测步长（在代码中设为25）
        # - 对于每个障碍物，都存储了N个时刻的预测位置（即每个障碍物有25个预测点）
        # - 所以 len(self.ob) 实际上是：障碍物数量 × N
        # num_obs表示障碍物数量
        num_obs = int(len(self.ob)/self.N)

        # [i, :] 表示：
        # i ：选择第i行
        # : ：选择这一行的所有列
        #下面是cbf约束，可以看出CBF约束对每个障碍物和每个预测状态都是有效的
        for j in range(num_obs):
            if not exceed_ob(self.ob[self.N*j]):#需要避开
                for i in range(self.N-1):
                    opti.subject_to(h(opt_states[i + 1, :], self.ob[j * 25 + i + 1]) >=
                                    (1 - gamma_k) * h(opt_states[i, :], self.ob[j * 25 + i]))

        obj = 0# 初始化目标函数值为0

        # - R = diag([0.1, 0.02]) 是一个2×2的对角矩阵
        # - 0.1是对线速度v的惩罚权重
        # - 0.02是对角速度ω的惩罚权重
        R = np.diag([0.1, 0.02])

        for i in range(self.N):
            # 创建状态误差的权重矩阵Q，权重随时间步增加而增加
            # [1.0+0.05*i, 1.0+0.05*i] 分别对应x,y两个状态量的权重
            Q = np.diag([1.0+0.05*i,1.0+0.05*i])
            obj += quadratic(opt_states[i+1, :] - self.goal_state[i], Q) + quadratic(opt_controls[i, :], R)
            

        opti.minimize(obj)
        opts_setting = {'ipopt.max_iter': 2000, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-3,
                        'ipopt.acceptable_obj_change_tol': 1e-3}
        opti.solver('ipopt', opts_setting)
        opti.set_value(opt_x0, self.curr_state)#把current_state赋值给opt_x0

        # C++/Java 用 catch，Python 用 except
        try:
            sol = opti.solve()# 尝试求解优化问题
            u_res = sol.value(opt_controls)# 获取最优的控制输入和状态序列
            state_res = sol.value(opt_states)
            # 保存本次成功的结果，供下次求解失败时使用
            self.last_input = u_res
            self.last_state = state_res
            self.mpc_success = True

        except:
            # 优化问题无解时的处理
            rospy.logerr("Infeasible Solution")# 输出错误日志

            if self.mpc_success:# 如果上一次是成功的
                self.mpc_success = False # 标记这次失败
                  
            else:# 如果上一次也是失败的
                for i in range(self.N-1):# 将控制输入和状态向前移动一步
                    self.last_input[i] = self.last_input[i+1]
                    self.last_state[i] = self.last_state[i+1]
                    self.last_input[self.N-1] = np.zeros([1, 2])# 最后一个控制输入设为零

            u_res = self.last_input
            state_res = self.last_state
        self.curr_pose_lock.release()# 释放锁，其他线程可以访问数据
        self.global_path_lock.release()# 释放锁，其他线程可以访问数据
        self.obstacle_lock.release()# 释放锁，其他线程可以访问数据
        return state_res, u_res

# if __name__ == '__main__':用于判断当前脚本是被直接运行还是被当做模块导入。
if __name__ == '__main__':
    rospy.init_node("local_planner")#初始化节点
    local_planner = Local_Planner()#创建Local_Planner对象

    rospy.spin()
