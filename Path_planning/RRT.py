import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani


class Path_planning():
    def __init__(self):
        # Pathプランナー
        self.path_planner = RRT(init_x, init_y)

        # アニメーション作成
        self.animation = anim = Animation(axis)

        # 



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
        self.MAX_y = -10
        self.min_x = 10
        self.min_y = -10

        # ノードを作成する
        self.Nodes = [self.x, self.y]

    def search(self):
        # random点を打つ
        s_x = np.random.rand() * self.MAX_x
        s_y = np.random.rand() * self.MAX_y

        sample = [s_x, s_y]

        # ノード探索
        distance = 'inf'
        nearest_node = None

        for node in self.Nodes:
            part_MSE = (sample - node) * (sample - node)
            RMSE = np.sqrt(sum(part_MSE))

            # 距離が小さかったら追加
            if RMSE < distance:
                distance = RMSE
                nearest_node = node

        return nearest_node

class Animation():
    def __init__(self, fig, xlim, ylim, ):
        self.fig = fig
        self.axis = self.fig.add_subplot(111)

        # 初期設定
        self.MAX_x = 12.5
        self.min_x = -12.5
        self.MAX_y = 12.5
        self.min_y = -12.5

        self.axis.set_xlim([self.MAX_x, self.min_x])
        self.axis.set_ylim([self.MAX_y, self.min_y])

        # 軸
        self.axis.legend(True)

        # 縦横比
        self.axis.set_aspect('equal')

        # label
        self.axis.set_xlabel = ['X [m]']
        self.axis.set_ylabel = ['Y [m]']

    def animation
    # ステップ数表示
        step_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

        # 初期設定
        ball_img, = ax.plot([], [], label='Predicit', color="b")
                



def update_anim(i):# このiが更新するたびに増えていきます
    
    ball_x, ball_y= ball.state_update()

    ball_img, = drawer.draw_circle(ball_x, ball_y)

    # 時間
    step_text.set_text('step = {0}'.format(i))

    return ball_img, 

def main():

    






print('save_animation?')
shuold_save_animation = int(input())

if shuold_save_animation == True: 
    animation.save('basic_animation.gif', writer='imagemagick')

plt.show()
