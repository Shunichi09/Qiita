import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import math
import sys

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

class Path_anim():
    def __init__(self, axis):
        self.path_img, = axis.plot([], [], color='c', linestyle='dashed', linewidth=0.15)

    def set_graph_data(self, x, y):
        self.path_img.set_data(x, y)

        return self.path_img, 
    
class Obstacle_anim():
    def __init__(self, axis):
        self.obs_img, = axis.plot([], [], color='k')

    def set_graph_data(self, obstacle):
        angle = 0.0
        circle_x, circle_y, circle_line_x, circle_line_y = \
                write_circle(obstacle.x, obstacle.y, angle, circle_size=obstacle.size)

        self.obs_img.set_data(circle_x, circle_y)

        return self.obs_img, 

class Animation_robot():
    def __init__(self):
        self.fig = plt.figure()
        self.axis = self.fig.add_subplot(111)

    def fig_set(self):
        # 初期設定 軸
        MAX_x = 12
        min_x = -12
        MAX_y = 12
        min_y = -12

        self.axis.set_xlim(min_x, MAX_x)
        self.axis.set_ylim(min_y, MAX_y)

        # 軸
        self.axis.grid(True)

        # 縦横比
        self.axis.set_aspect('equal')

        # label
        self.axis.set_xlabel('X [m]')
        self.axis.set_ylabel('Y [m]')

    def plot(self, traj_x, traj_y): # ただのplot
        self.axis.plot(traj_x, traj_y)

        plt.show()
    
    def func_anim_plot(self, traj_x, traj_y, traj_th, traj_paths, traj_g_x, traj_g_y, traj_opt, obstacles):
        # selfにしておく
        self.traj_x = traj_x
        self.traj_y = traj_y
        self.traj_th = traj_th
        self.traj_paths = traj_paths
        self.traj_g_x = traj_g_x
        self.traj_g_y = traj_g_y
        self.traj_opt = traj_opt
        self.obstacles = obstacles

        # trajお絵かき
        self.traj_img, = self.axis.plot([], [], 'k', linestyle='dashed')

        # 円と向き
        self.robot_img, = self.axis.plot([], [], 'k')

        self.robot_angle_img, = self.axis.plot([], [], 'k')

        # goalを追加
        self.img_goal, = self.axis.plot([], [], '*', color='b', markersize=15)

        # dwa # 何本線引くかは考える
        self.dwa_paths = []
        self.max_path_num = 100
        for k in range(self.max_path_num):
            self.dwa_paths.append(Path_anim(self.axis))

        # opt_traj
        self.traj_opt_img, = self.axis.plot([], [], '#ff7f00', linestyle='dashed', marker='.' , markersize=0.5)

        # 障害物
        self.obs = []
        self.obstacles_num = len(obstacles)
        for k in range(len(obstacles)):
            self.obs.append(Obstacle_anim(self.axis))
        
        # ステップ数表示
        self.step_text = self.axis.text(0.05, 0.9, '', transform=self.axis.transAxes)


        animation = ani.FuncAnimation(self.fig, self._update_anim, interval=100, \
                              frames=len(traj_g_x))


        print('save_animation?')
        shuold_save_animation = int(input())

        if shuold_save_animation: 
            animation.save('basic_animation.gif', writer='imagemagick')

        plt.show()


    def _update_anim(self, i):
        # 全体
        self.dwa_imgs = []
        # DWApath用
        self.dwa_path_imgs = []
        # 障害物用
        self.obs_imgs = []

        self.traj_img.set_data(self.traj_x[:i+1], self.traj_y[:i+1])
        # 円を書く
        circle_x, circle_y, circle_line_x, circle_line_y = write_circle(self.traj_x[i], self.traj_y[i], self.traj_th[i], circle_size=0.2)

        self.robot_img.set_data(circle_x, circle_y)

        self.robot_angle_img.set_data(circle_line_x, circle_line_y)

        self.img_goal.set_data(self.traj_g_x[i], self.traj_g_y[i])

        self.traj_opt_img.set_data(self.traj_opt[i].x, self.traj_opt[i].y)

        count = 0
        # path_num = np.random.randint(0, len(self.traj_paths[i]), (1, 100))
        # print(path_num)
        
        for k in range(self.max_path_num):
            path_num = math.ceil(len(self.traj_paths[i])/(self.max_path_num)) * k
            
            if path_num >  len(self.traj_paths[i]) - 1:
                path_num = np.random.randint(0, len(self.traj_paths[i]))

            self.dwa_path_imgs.append(self.dwa_paths[k].set_graph_data(self.traj_paths[i][path_num].x, self.traj_paths[i][path_num].y))

        # obstacles
        for k in range(self.obstacles_num):
            self.obs_imgs.append(self.obs[k].set_graph_data(self.obstacles[k]))      

        self.step_text.set_text('step = {0}'.format(i))

        for img in [self.traj_img, self.robot_img, self.robot_angle_img, self.img_goal, self.step_text, self.dwa_path_imgs, self.obs_imgs, self.traj_opt_img]:
            self.dwa_imgs.append(img)


        return self.dwa_imgs
