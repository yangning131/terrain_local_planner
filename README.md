# Wish_plan
计划：生成边界

预期效果：承接上游lexroute生成的path 及边界  输入下游优化   接收障碍物做碰撞检测 
接收动态障碍物  定期调用

先验证 接收path

1.增加前向后向转向的选择 parame 禁止转向选择  
bug :rs曲线与搜索曲线接口处有过调 原因rs曲线本身有倒车  当前解决方法 只用混合搜索 加入了选择参数

2.cartesian 接收path及机器人odom    收到的path有更新或者新path有碰撞时用优化

3.走廊迭代次数和大小变化

4.Path 传输  车辆参数



5.新障碍物碰撞检测方法   已更改部分 碰撞检测方法有问题
6.里程计及path更新      抹去走过的path
7.cmd计算或者输出path

8.hybrid steering_angle_discrete_num  更改增加扩展数量

9.更改hybrid 初始化位置 优化速度  ;  释放 珊格数据结构

10.重新做碰撞检测  定位低障碍物位置  用z值标记  优化约束住low ob