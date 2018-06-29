import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import math
# ボールが跳ねるアニメーション

class Ball():
    # どのボールでも共通です
    max_x = 5
    min_x = -5
    max_y = 5
    min_y = -5

    def __init__(self, init_x, init_y, v_x, v_y, ball_size, e, dt):
        # 初期位置
        self.x = init_x
        self.y = init_y
        self.v_x = v_x
        self.v_y = v_y
        self.size = ball_size
        self.e = e#反発係数です
        self.dt = dt

    def state_update(self):
        self.x += self.v_x * self.dt 
        self.y += self.v_y * self.dt

        # 範囲外に出ないように
        if self.x > self.max_x:
            self.x = self.max_x
            self.velocity_x_update()
        elif self.x < self.min_x:
            self.x = self.min_x
            self.velocity_x_update()

        if self.y > self.max_y:
            self.y = self.max_y
            self.velocity_y_update()
        elif self.y < self.min_y:
            self.y = self.min_y
            self.velocity_y_update()

        return self.x, self.y

    def velocity_x_update(self):
        self.v_x = -self.e * self.v_x
    
    def velocity_y_update(self):
        self.v_y = -self.e * self.v_y

class Drawing():
    def __init__(self, ax):
        self.color = []
        self.ax = ax
        self.ball_img, = ax.plot([], [], color="b")

    def draw_circle(self, center_x, center_y, circle_size=0.2):#人の大きさは半径15cm
        # 初期化
        self.circle_x = [] #位置を表す円のx
        self.circle_y = [] #位置を表す円のy

        steps = 100 #円を書く分解能はこの程度で大丈夫
        for i in range(steps):
            self.circle_x.append(center_x + circle_size*math.cos(i*2*math.pi/steps))
            self.circle_y.append(center_y + circle_size*math.sin(i*2*math.pi/steps))
        
        self.ball_img.set_data(self.circle_x, self.circle_y)

        return self.ball_img, 

def update_anim(i):# このiが更新するたびに増えていきます
    
    ball_x, ball_y= ball.state_update()

    ball_img, = drawer.draw_circle(ball_x, ball_y)

    # 時間
    step_text.set_text('step = {0}'.format(i))

    return ball_img, 

#############################################################
#########################__MAIN__############################
## plot 初期化
# グラフ仕様設定
fig = plt.figure()

# Axes インスタンスを作成
ax = fig.add_subplot(111)

# 軸
# 最大値と最小値⇒軸の範囲設定
max_x = 5
min_x = -5
max_y = 5
min_y = -5

ax.set_xlim(min_x, max_x)
ax.set_ylim(min_y, max_y)

# 軸の縦横比, 正方形，単位あたりの長さが等しくなります
ax.set_aspect('equal')

# 軸の名前設定
ax.set_xlabel('X [m]') 
ax.set_ylabel('Y [m]') 

# その他グラフ仕様
ax.grid(True) # グリッド

# 凡例
ax.legend()

# ステップ数表示
step_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

# 初期設定
ball_img, = ax.plot([], [], label='Predicit', color="b")

# ボール作ります
ball = Ball(0, 0, 2, 3, 0.2, 0.75, 0.05)
drawer = Drawing(ax)

animation = ani.FuncAnimation(fig, update_anim, interval=50, frames=100)

print('save_animation?')
shuold_save_animation = int(input())

if shuold_save_animation == True: 
    animation.save('basic_animation.gif', writer='imagemagick')

plt.show()