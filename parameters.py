


times_threshold_value = 10  # 两个阶段划分的阈值
time_to_target = 15
distance_1 = 0.3  # exploration阶段的移动步长
distance_2 = 0.2  # exploitation阶段的移动步长
candidates_number = 24
source_v_x = 0.1  # 源在x轴方向的运动速度
source_v_y = 0.1  # 源在y轴方向的运动速度
track_point = [0.0, 0.0]  # 追踪方的初始位置
random_distance = 1.0  # 刚开始随机方向移动每次移动的距离
v_track = 0.5  # 追踪方移动速度
random_times = 3  # 刚开始随机移动的次数
threshold_distance = 0.1
rssi_noise = 0.6
track_point_1 = [0.0, 0.0]
track_point_2 = [0.2, 0.4]
track_point_3 = [-0.2, 0.4]
collision_threshold_value = 0.4  # 任意两架无人机之间的最小距离，避免碰撞
communication_threshold_value = 1.2  # 无人机之间的最大距离,保证通信
left_boundary = -2.5
right_boundary = 2.5
k_for_calculate = 3  # 探索阶段每一轮飞行的计算迭代次数
