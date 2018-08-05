# made by Shunichi Sekiguchi

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import math

class RRT():
    def __init__(self, init, goal):
        # 初期設定
        self.init_x = init[0]
        self.init_y = init[1]
        self.goal_x = goal[0]
        self.goal_y = goal[1]
        self.goal = goal

        # パラメータ群
        # 伸ばし幅
        self.d = 0.1
        # 何回かに一回はゴールを選ぶ
        self.g_rate = 0.1
        # どこまでゴールに近づければいいか
        self.g_range = 0.1

        # 探索範囲
        self.MAX_x = 3
        self.MAX_y = 3
        self.min_x = -3
        self.min_y = -3

        # ノードを作成する
        # これはただのノード
        self.Nodes = np.array([[self.init_x, self.init_y]])
        # これはpath
        self.path_x = np.empty((0,2), float)
        self.path_y = np.empty((0,2), float)
        # samples
        self.samples = np.empty((0,2), float)

        self.nearest_node =None
        self.new_node =None

    def search(self):
        # 行動決定（何回かに一回はGoalを選ぶ）
        temp = np.random.randint(0, 10)
        if temp > 0:
            # random点を打つ(-1.5, 1.5の範囲で選ぶ)
            s_x = (np.random.rand() * self.MAX_x) - self.MAX_x/2
            s_y = (np.random.rand() * self.MAX_y) - self.MAX_y/2

            sample = np.array([s_x, s_y])
        else:
            # goalを選ぶ
            s_x = self.goal_x
            s_y = self.goal_y

            sample = np.array([s_x, s_y])

        self.samples = np.append(self.samples, [[s_x, s_y]], axis=0)

        # ノード探索
        distance = float('inf')
        self.nearest_node = None

        for i in range(self.Nodes.shape[0]):
            node = self.Nodes[i, :]
            part_MSE = (sample - node) * (sample - node)
            RMSE = math.sqrt(sum(part_MSE))

            # 距離が小さかったら追加
            if RMSE < distance:
                distance = RMSE
                self.nearest_node = node

        # 新ノードを作成
        pull = sample - self.nearest_node
        grad = math.atan2(pull[1], pull[0])

        d_x = math.cos(grad) * self.d
        d_y = math.sin(grad) * self.d

        self.new_node = self.nearest_node + np.array([d_x, d_y])
        
        return self.nearest_node, self.new_node
        
    def check_goal(self):
        dis = np.sqrt(sum((self.new_node - self.goal) *  (self.new_node - self.goal)))
        goal_flag = False
        if dis < self.g_range:
            print('GOAL!!')
            goal_flag = True
        
        return goal_flag

    def path_make(self):
        # 新ノードを追加
        self.Nodes = np.vstack((self.Nodes, self.new_node))

        self.path_x = np.append(self.path_x, np.array([[self.nearest_node[0], self.new_node[0]]]), axis=0)
        self.path_y = np.append(self.path_y, np.array([[self.nearest_node[1], self.new_node[1]]]), axis=0)

        return self.Nodes, self.path_x, self.path_y, self.samples

class Figures():
    def __init__(self):
        self.fig = plt.figure()
        self.axis = self.fig.add_subplot(111)

    def fig_set(self):
        # 初期設定
        MAX_x = 1.5
        min_x = -1.5
        MAX_y = 1.5
        min_y = -1.5

        self.axis.set_xlim(MAX_x, min_x)
        self.axis.set_ylim(MAX_y, min_y)

        # 軸
        self.axis.grid(True)

        # 縦横比
        self.axis.set_aspect('equal')

        # label
        self.axis.set_xlabel('X [m]')
        self.axis.set_ylabel('Y [m]')

    def plot(self, path_x, path_y, Nodes):
        self.axis.plot(path_x, path_y)
        self.axis.plot(Nodes[:, 0], Nodes[:, 1], '.', color='k')

        plt.show()

    def anim_plot(self, path_x, path_y, Nodes, samples, goal):
        imgs = []
        finish_buffa = 20# 終了後もそのまま数秒表示したいので

        for i in range(path_x.shape[1] + finish_buffa):
            img = []
            if i >= path_x.shape[1]:
                i = path_x.shape[1] - 1
                img_text = self.axis.text(0.05, 0.9, 'step = {0}'.format(i), transform=self.axis.transAxes)
                img.append(img_text)
                img_goal_text = self.axis.text(0.35, 0.5, 'GOAL!!', transform=self.axis.transAxes, fontsize=30)
                img.append(img_goal_text)

            # step数を追加
            img_text = self.axis.text(0.05, 0.9, 'step = {0}'.format(i), transform=self.axis.transAxes)
            img.append(img_text)
            # goalを追加
            img_goal = self.axis.plot(goal[0], goal[1], '*', color='r')
            img.extend(img_goal)
            # sampleを追加
            img_sample = self.axis.plot(samples[i, 0], samples[i, 1], '*', color='b')
            img.extend(img_sample)
            # nodeを追加
            img_nodes = self.axis.plot(Nodes[:i+2, 0], Nodes[:i+2, 1], '.', color='k')
            img.extend(img_nodes)

            # pathを追加
            for k in range(i+1):
                img_path = self.axis.plot(path_x[:, k], path_y[:, k], color='b')
                img.extend(img_path)

            print('i = {0}'.format(i))
            imgs.append(img)
        
        animation = ani.ArtistAnimation(self.fig, imgs)

        print('save_animation?')
        shuold_save_animation = int(input())

        if shuold_save_animation: 
            animation.save('basic_animation.gif', writer='imagemagick')

        plt.show()

    
def main():
    # figures
    figure = Figures()
    figure.fig_set()

    # pathmake
    goal = [1.0, 1.0]
    init = [0.0, 0.0]
    path_planner = RRT(init, goal)
    # iterations = 100

    while True:
        path_planner.search()
        goal_flag = path_planner.check_goal()
        Nodes, path_x, path_y, samples = path_planner.path_make()
        if goal_flag :
            break
        
    # img用に処理
    path_x = path_x.transpose()
    path_y = path_y.transpose()

    figure.anim_plot(path_x, path_y, Nodes, samples, goal)
    
if __name__ == '__main__':
    main()


'''
def anim_set(self, path_x, path_y, Nodes):
        # step
        self.step_text = self.axis.text(0.05, 0.9, '', transform=self.axis.transAxes)
        # node
        self.node_img, = self.axis.plot([], [], '.', label='node', color='k')
        # path
        self.path_img, = self.axis.plot([], [], label='path', color='k')
        # data
        self.path_x = path_x
        self.path_y = path_y
        self.Nodes = Nodes
        # pathをためておくよう
        self.path_imgs = [self.path_img,]

    def init_anim(self): # 初期化
        self.path_img.set_data([], [])

        return self.path_img,
    
    def update_anim(self, i):
        # nodes
        self.node_img.set_data(self.Nodes[:i+1, 0], self.Nodes[:i+1, 1])
        # path(作成)
        img, = self.axis.plot([], [], label='path', color='k')
        self.path_imgs.append(img, )

        #  self.path_img.set_data(self.path_x[i+1, :], self.path_y[i+1, :])
        self.path_imgs[i].set_data(self.path_x[i+1, :], self.path_y[i+1, :])

        # step
        self.step_text.set_text('step = {0}'.format(i))

        return self.node_img,  self.path_imgs, self.step_text,

    def show_ani(self):
        animation = ani.FuncAnimation(self.fig, self.update_anim, init_func=self.init_anim, interval=100, frames=1000 ,blit=True)
        plt.show()
    '''