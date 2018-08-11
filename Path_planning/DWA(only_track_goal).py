# controller
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from animation import Animation_robot
import math
import sys

# 基本関数
# 正規化
def min_max_normalize(data):

    data = np.array(data)
    
    max_data = max(data)
    min_data = min(data)

    if max_data - min_data == 0:
        data = [0.0 for i in range(len(data))]
    else:
        data = (data - min_data) / (max_data - min_data)

    return data
# 角度補正用
def angle_range_corrector(angle):

    if angle > math.pi:
        while angle > math.pi:
            angle -=  2 * math.pi
    elif angle < -math.pi:
        while angle < -math.pi:
            angle += 2 * math.pi
    
    return angle

# 円を書く
def write_circle(center_x, center_y, angle, circle_size=0.2):#人の大きさは半径15cm
    # 初期化
    circle_x = [] #位置を表す円のx
    circle_y = [] #位置を表す円のy

    steps = 100 #円を書く分解能はこの程度で大丈夫
    for i in range(steps):
        circle_x.append(center_x + circle_size*math.cos(i*2*math.pi/steps))
        circle_y.append(center_y + circle_size*math.sin(i*2*math.pi/steps))
    
    circle_line_x = [center_x, center_x + math.cos(angle) * circle_size]
    circle_line_y = [center_y, center_y + math.sin(angle) * circle_size]
    
    return circle_x, circle_y, circle_line_x, circle_line_y

# ルール
# x, y, thは基本的に今のstate
# g_ はgoal
# traj_ は過去の軌跡
# 単位は，角度はrad，位置はm
# 二輪モデルなので入力は速度と角速度

# path
class Path():
    def __init__(self, u_th, u_v): 
        self.x = None
        self.y = None
        self.th = None
        self.u_v = u_v
        self.u_th = u_th
        
class Two_wheeled_robot(): # 実際のロボット
    def __init__(self, init_x, init_y, init_th):
        # 初期状態
        self.x = init_x
        self.y = init_y
        self.th = init_th
        self.u_v = 0.0
        self.u_th = 0.0

        # 時刻歴保存用
        self.traj_x = [init_x]
        self.traj_y = [init_y]
        self.traj_th = [init_th]
        self.traj_u_v = [0.0]
        self.traj_u_th = [0.0]

    def update_state(self, u_th, u_v, dt): # stateを更新

        self.u_th = u_th
        self.u_v = u_v
        
        next_x = self.u_v * math.cos(self.th) * dt + self.x
        next_y = self.u_v * math.sin(self.th) * dt + self.y
        next_th = self.u_th * dt + self.th

        self.traj_x.append(next_x)
        self.traj_y.append(next_y)
        self.traj_th.append(next_th)

        self.x = next_x
        self.y = next_y
        self.th = next_th

        return self.x, self.y, self.th # stateを更新

class Simulator_DWA_robot(): # DWAのシミュレータ用
    def __init__(self):
        # self.model 独立二輪型
        # 加速度制限
        self.max_accelation = 1.0
        self.max_ang_accelation = 100 * math.pi /180
        # 速度制限
        self.lim_max_velo = 1.6 # m/s
        self.lim_min_velo = 0.0 # m/s
        self.lim_max_ang_velo = math.pi/2
        self.lim_min_ang_velo = -math.pi/2

    # 予想状態を作成する
    def predict_state(self, ang_velo, velo, x, y, th, dt, pre_step): # DWA用(何秒後かのstateを予測))
        next_xs = []
        next_ys = []
        next_ths = []

        for i in range(pre_step):
            temp_x = velo * math.cos(th) * dt + x
            temp_y = velo * math.sin(th) * dt + y
            temp_th = ang_velo * dt + th

            next_xs.append(temp_x)
            next_ys.append(temp_y)
            next_ths.append(temp_th)

            x = temp_x
            y = temp_y
            th = temp_th

        # print('next_xs = {0}'.format(next_xs))

        return next_xs, next_ys, next_ths # 予想した軌跡

# DWA
class DWA():
    def __init__(self):
        # 初期化
        # simulation用のロボット
        self.simu_robot = Simulator_DWA_robot()

        # 予測時間(s)
        self.pre_time = 3
        self.pre_step = 30

        # 探索時の刻み幅
        self.delta_velo = 0.02
        self.delta_ang_velo = 0.02

        # サンプリングタイム(変更の場合，共通する項があるので，必ずほかのところも確認する)
        self.samplingtime = 0.1

        # 重みづけ
        self.weight_angle = 0.1
        self.weight_velo = 0.2
        self.weight_obs = 0.1

        # すべてのPathを保存
        self.traj_paths = []

    def calc_input(self, g_x, g_y, state): # stateはロボットクラスでくる
        # Path作成
        paths = self._make_path(state)
        # Path評価
        opt_path = self._eval_path(paths, g_x, g_y)
        
        return paths, opt_path

    def _make_path(self, state): 
        # 角度と速度の範囲算出
        min_ang_velo, max_ang_velo, min_velo, max_velo = self._calc_range_velos(state)

        # 障害物の範囲
        # min_ang_velo, max_ang_velo, min_velo, max_velo = self._calc_range_obs_velos(min_ang_velo, max_ang_velo, min_velo, max_velo)

        # 全てのpathのリスト
        paths = []

        ang_velo_range = np.arange(min_ang_velo, max_ang_velo, self.delta_ang_velo)

        # 角速度と速度の組み合わせを全探索
        for ang_velo in np.arange(min_ang_velo, max_ang_velo, self.delta_ang_velo):
            for velo in np.arange(min_velo, max_velo, self.delta_velo):

                path = Path(ang_velo, velo)

                next_x, next_y, next_th \
                    = self.simu_robot.predict_state(ang_velo, velo, state.x, state.y, state.th, self.samplingtime, self.pre_step)

                path.x = next_x
                path.y = next_y
                path.th = next_th
            
                # 作ったpathを追加
                paths.append(path)
        
        # 時刻歴Pathを保存
        self.traj_paths.append(paths)

        return paths

    def _calc_range_velos(self, state): # 角速度と角度の範囲決定①
        # 角速度
        range_ang_velo = self.samplingtime * self.simu_robot.max_ang_accelation
        min_ang_velo = state.u_th - range_ang_velo
        max_ang_velo = state.u_th + range_ang_velo
        # 最小値
        if min_ang_velo < self.simu_robot.lim_min_ang_velo:
            min_ang_velo = self.simu_robot.lim_min_ang_velo
        # 最大値
        if max_ang_velo > self.simu_robot.lim_max_ang_velo:
            max_ang_velo = self.simu_robot.lim_max_ang_velo

        # 速度
        range_velo = self.samplingtime * self.simu_robot.max_accelation
        min_velo = state.u_v - range_velo
        max_velo = state.u_v + range_velo
        # 最小値
        if min_velo < self.simu_robot.lim_min_velo:
            min_velo = self.simu_robot.lim_min_velo
        # 最大値
        if max_velo > self.simu_robot.lim_max_velo:
            max_velo = self.simu_robot.lim_max_velo

        return min_ang_velo, max_ang_velo, min_velo, max_velo

    '''
    def _calc_range_obs_velos(self, min_ang_velo, max_ang_velo, min_velo, max_velo): # 角速度と角度の範囲決定障害物考慮
        # 角速度
        range_ang_velo = self.samplingtime * self.simu_robot.max_ang_accelation
        min_ang_velo = state.u_th - range_ang_velo
        max_ang_velo = state.u_th + range_ang_velo
        # 最小値
        if min_ang_velo < self.simu_robot.lim_min_ang_velo:
            min_ang_velo = self.simu_robot.lim_min_ang_velo
        # 最大値
        if max_ang_velo > self.simu_robot.lim_max_ang_velo:
            max_ang_velo = self.simu_robot.lim_max_ang_velo

        # 速度
        range_velo = self.samplingtime * self.simu_robot.max_accelation
        min_velo = state.u_v - range_velo
        max_velo = state.u_v + range_velo
        # 最小値
        if min_velo < self.simu_robot.lim_min_velo:
            min_velo = self.simu_robot.lim_min_velo
        # 最大値
        if max_velo > self.simu_robot.lim_max_velo:
            max_velo = self.simu_robot.lim_max_velo

        return min_ang_velo, max_ang_velo, min_velo, max_velo
    '''

    def _eval_path(self, paths, g_x, g_y):
        score_heading_angles = []
        score_heading_velos = []
        score_obstacles = []

        # 全てのpathで評価を検索
        for path in paths:
            # (1) heading_angle
            score_heading_angles.append(self._heading_angle(path, g_x, g_y))
            # (2) heading_velo
            score_heading_velos.append(self._heading_velo(path))
            # (3)obstacle
            score_obstacles.append(self._obstacle(path))

        # 正規化
        for scores in [score_heading_angles, score_heading_velos, score_obstacles]:
            scores = min_max_normalize(scores)
            # print('angle_to_goal = {0}'.format(scores))

        score = 0.0
        # 最小pathを探索
        for k in range(len(paths)):
            temp_score = 0.0

            temp_score = self.weight_angle * score_heading_angles[k] + \
                         self.weight_velo * score_heading_velos[k] # + \
                         # self.weight_obs * score_obstacles[k]
        
            if temp_score > score:
                opt_path = paths[k]
                score = temp_score
                
        return opt_path

    def _heading_angle(self, path, g_x, g_y):
        # 終端の向き
        last_x = path.x[-1]
        last_y = path.y[-1]
        last_th = path.th[-1]

        # 角度計算
        angle_to_goal = math.atan2(g_y-last_y, g_x-last_x)

        # score計算
        score_angle = angle_to_goal - last_th

        # ぐるぐる防止
        score_angle = abs(angle_range_corrector(score_angle))
        
        # 最大と最小をひっくり返す
        score_angle = math.pi - score_angle

        # print('score_sngle = {0}' .format(score_angle))

        return score_angle

    def _heading_velo(self, path): 

        score_heading_velo = path.u_v

        return score_heading_velo

    def _obstacle(self, path):#  obastacles):
        # 障害物回避（エリアに入ったらその線は使わない）/ 今回はなし
        score_obstacle = 0.0
        return score_obstacle

class Const_goal():# goal作成プログラム
    def __init__(self):
        # self.human_trajectory = ...的な
        self.traj_g_x = []
        self.traj_g_y = []

    def calc_goal(self, time_step): # 本当は人の値が入ってもよいかも
        if time_step <= 25:
            g_x  = 2.5
            g_y = 2.5
        else:
            g_x = -2.5
            g_y = -2.5

        self.traj_g_x.append(g_x)
        self.traj_g_y.append(g_y)

        return g_x, g_y

class Main_controller():# Mainの制御クラス
    def __init__(self):
        self.robot = Two_wheeled_robot(0.0, 0.0, 0.0)
        self.goal_maker = Const_goal()
        self.controller = DWA()

        # ここを変えたら他もチェック
        self.samplingtime = 0.1

    def run_to_goal(self):
        goal_flag = False
        time_step = 0

        while not goal_flag:
            g_x, g_y = self.goal_maker.calc_goal(time_step)

            # 入力決定
            paths, opt_path = self.controller.calc_input(g_x, g_y, self.robot)

            u_th = opt_path.u_th
            u_v = opt_path.u_v
            
            # 入力で状態更新
            self.robot.update_state(u_th, u_v, self.samplingtime)

            # goal判定
            dis_to_goal = np.sqrt((g_x-self.robot.x)*(g_x-self.robot.x) + (g_y-self.robot.y)*(g_y-self.robot.y))
            if dis_to_goal < 0.5:
                goal_flag = True
            
            time_step += 1

        return self.robot.traj_x, self.robot.traj_y, self.robot.traj_th, \
                self.goal_maker.traj_g_x, self.goal_maker.traj_g_y, self.controller.traj_paths

def main():
    animation = Animation_robot()
    animation.fig_set()

    controller = Main_controller()
    traj_x, traj_y, traj_th, traj_g_x, traj_g_y, traj_paths = controller.run_to_goal()

    ani = animation.func_anim_plot(traj_x, traj_y, traj_th, traj_paths, traj_g_x, traj_g_y) # , obstacles)

if __name__ == '__main__':
    main()