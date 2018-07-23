import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import math
import random
from msvcrt import getch
import signal
import sys
import os

# シューティングゲーム的なね
class Ball():
    # どのボールでも共通です
    max_x = 5
    min_x = -5
    max_y = 5
    min_y = -5

    def __init__(self):
        # 初期設定
        self.x = 0.1*random.randint(-50, 50)
        self.y = 5.0 # ここは固定
        self.v_x = 0.0 # 真下に落ちます 
        self.v_y = 0.1*random.randint(-50, -15)
        self.dt = 0.05

    def state_update(self):
        self.x += self.v_x * self.dt
        self.y += self.v_y * self.dt
        delete_flag = False

        # 範囲外に出たら，消滅
        if self.y < self.min_y:
            delete_flag = True

        return self.x, self.y, delete_flag

class Player():
    max_x = 5
    min_x = -5
    max_y = 5
    min_y = -5

    def __init__(self):
        # 初期設定
        self.x = 0.0
        self.y = -3.0 # ここは固定

    def state_update(self):
        # keybord関連
        # 右に動く
        signal.signal(signal.SIGINT, self.righter)

        if self.x > max_x:
            self.x = min_x

        return self.x, self.y
    
    def righter(self, signal, frame):
        self.x += 0.05

    def gameover_judge(self, balls_x, balls_y):
        gameover_flag = False
        for i in range(len(balls_x)):
            if abs(balls_x[i] - self.x) < 0.3 and abs(balls_y[i] - self.y) < 0.3:
                gameover_flag = True
                break

        return gameover_flag

class Drawing_ball():# 描画するクラス
    def __init__(self, ax):
        color_list = ['r', 'b', 'k', 'c', 'y', 'm', 'g']
        color_num = random.randint(0, len(color_list)-1)
        self.ball_img, = ax.plot([], [], color=color_list[color_num])
        self.ax = ax

    def draw_circle(self, center_x, center_y):
        # ボールの大きさ
        circle_size = 0.25

        steps = 50 #円を書く分解能はこの程度で大丈夫
        self.circle_x = [] #位置を表す円のx
        self.circle_y = [] #位置を表す円のy
            
        for j in range(steps):

            self.circle_x.append(center_x + circle_size*math.cos(j*2*math.pi/steps))
            self.circle_y.append(center_y + circle_size*math.sin(j*2*math.pi/steps))

        return self.circle_x, self.circle_y

    def set_graph_data(self):
        self.ball_img.set_data(self.circle_x, self.circle_y)

        return self.ball_img, 

class Drawing_player():# 描画するクラス
    def __init__(self, ax):
        self.player_head_img, = ax.plot([], [], color='k')
        self.player_body_img, = ax.plot([], [], color='k')
        self.ax = ax

    def draw_head_body(self, center_x, center_y):
        head_size = 0.25
        body_size = 0.5
        circle_size = [head_size, body_size]
        center_xs = [center_x, center_x]
        center_ys = [center_y+body_size+head_size, center_y]

        steps = 50 #円を書く分解能はこの程度で大丈夫
        self.circle_x = [[], []] #位置を表す円のx
        self.circle_y = [[], []] #位置を表す円のy
        for k in range(2):
            for j in range(steps):
                self.circle_x[k].append(center_xs[k] + circle_size[k]*math.cos(j*2*math.pi/steps))
                self.circle_y[k].append(center_ys[k] + circle_size[k]*math.sin(j*2*math.pi/steps))

        return self.circle_x, self.circle_y

    def set_graph_data(self):
        self.player_head_img.set_data(self.circle_x[0], self.circle_y[0])
        self.player_body_img.set_data(self.circle_x[1], self.circle_y[1])
        player_imgs = [self.player_head_img, self.player_body_img]

        return player_imgs,

def update_anim(i):# このiが更新するたびに増えていきます
    balls_imgs = []
    balls_x = []
    balls_y = []

    if i % 50 == 0: #ボール追加します
        balls.append(Ball())
        balls_drawers.append(Drawing_ball(ax))

    count = 0

    for ball in balls:
        # ボールの状態をアップデート
        temp_x, temp_y, delete_flg = ball.state_update()

        if delete_flg == True: #ボールが枠外に行ってたら消去
            balls.pop(count)
            balls_drawers.pop(count)
        else:
            balls_x.append(temp_x)
            balls_y.append(temp_y)
            balls_drawers[count].draw_circle(temp_x, temp_y)
            balls_imgs.append(balls_drawers[count].set_graph_data())

            count += 1

    # playerの状態をupdatee
    temp_x, temp_y = player.state_update()
    # 衝突判定
    gameover_flag = player.gameover_judge(balls_x, balls_y)    
    if gameover_flag == True:
        gameover_text.set_text('GAME OVER')

    player_drawer.draw_head_body(temp_x, temp_y)
    player_imgs = player_drawer.set_graph_data()

    # 時間
    step_text.set_text('step = {0}'.format(i))

    return balls_imgs, player_imgs

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

# ステップ数表示
step_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

# gameover
gameover_text = ax.text(0.15, 0.5, '', transform=ax.transAxes, fontsize=30)

# ボールだけ作ってる
balls_size = []
balls = []
balls_drawers = []

# player作成
player = Player()
player_drawer = Drawing_player(ax)

# アニメーション描画
animation = ani.FuncAnimation(fig, update_anim, interval=50, frames=500)

print('save_animation?')
shuold_save_animation = int(input())

if shuold_save_animation == True: 
    animation.save('anim.mp4', writer="ffmpeg")

plt.show()