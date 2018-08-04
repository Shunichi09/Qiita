import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import math

class RRT():
    def __init__(self, init_x, init_y):
        # 初期設定
        self.x = init_x
        self.y = init_y

        # パラメータ群
        # 伸ばし幅
        self.d = 0.01

        # 探索範囲
        self.MAX_x = 10
        self.MAX_y = 10
        self.min_x = -10
        self.min_y = -10

        # ノードを作成する
        # これはただのノード
        self.Nodes = np.array([[init_x, init_y]])
        # これはpath
        self.path_x = np.empty((0,2), float)
        self.path_y = np.empty((0,2), float)
        # samples
        self.samples = np.empty((0,2), float)

        self.nearest_node =None
        self.new_node =None

    def search(self):
        # random点を打つ(-5, 5の範囲で選ぶ)
        s_x = (np.random.rand() * self.MAX_x) - self.MAX_x/2
        s_y = (np.random.rand() * self.MAX_y) - self.MAX_y/2

        sample = np.array([s_x, s_y])
        self.samples = np.append(self.samples, [[s_x, s_y]], axis=0)

        print('sample = {0}'.format(sample))

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
        

    def path_make(self):
        # 新ノードを追加
        self.Nodes = np.vstack((self.Nodes, self.new_node))
        # pathを追加
        # print(self.path_x)
        # print(np.array([self.nearest_node[0], self.new_node[0]]))

        self.path_x = np.append(self.path_x, np.array([[self.nearest_node[0], self.new_node[0]]]), axis=0)
        self.path_y = np.append(self.path_y, np.array([[self.nearest_node[1], self.new_node[1]]]), axis=0)

        # print('self.Nodes, self.path_x, self.path_y = {0}, {1}, {2}'.format(self.Nodes, self.path_x, self.path_y))

        return self.Nodes, self.path_x, self.path_y, self.samples

    def path_back(self):
        pass



class Figures():
    def __init__(self):
        self.fig = plt.figure()
        self.axis = self.fig.add_subplot(111)
    
    def fig_set(self):
        # 初期設定
        MAX_x = 1
        min_x = -1
        MAX_y = 1
        min_y = -1

        self.axis.set_xlim(MAX_x, min_x)
        self.axis.set_ylim(MAX_y, min_y)

        # 軸
        self.axis.grid(True)

        # 縦横比
        self.axis.set_aspect('equal')

        # label
        self.axis.set_xlabel = ['X [m]']
        self.axis.set_ylabel = ['Y [m]']

    def plot(self, path_x, path_y, Nodes):
        self.axis.plot(path_x, path_y)
        self.axis.plot(Nodes[:, 0], Nodes[:, 1], '.', color='k')

        plt.show()

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

    def init_anim(self): # 初期化
        self.path_img.set_data([], [])

        return self.path_img,
    
    def update_anim(self, i):
        # nodes
        self.node_img.set_data(self.Nodes[:i+1, 0], self.Nodes[:i+1, 1])
        # path
        print(self.path_x[i+1, :])
        self.path_img.set_data(self.path_x[i+1, :], self.path_y[i+1, :])
        # step
        self.step_text.set_text('step = {0}'.format(i))

        return self.node_img, self.path_img, self.step_text, 

    def show_ani(self):
        animation = ani.FuncAnimation(self.fig, self.update_anim, init_func=self.init_anim, interval=100, frames=1000 ,blit=True)
        plt.show()

def main():
    # figures
    figure = Figures()
    figure.fig_set()

    # pathmake
    path_planner = RRT(0.0, 0.0)
    iterations = 1000

    for k in range(iterations):
        path_planner.search()
        Nodes, path_x, path_y, samples = path_planner.path_make()

    # img用に処理
    # path_x = path_x.transpose()
    # path_y = path_y.transpose()
    # samples = samples.transpose()

    print(path_x)

    # path お試しplot
    # plt.plot(path_x[:, :50], path_y[:, :50])
    # plt.plot(path_x[:, :100], path_y[:, :100], 'r')
    # plt.plot(Nodes[:, 0], Nodes[:, 1], 'o')
    # plt.show()

    # figure.plot(path_x, path_y, Nodes)
    figure.anim_set(path_x, path_y, Nodes)
    figure.show_ani()

if __name__ == '__main__':
    main()
