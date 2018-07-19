# 初めに
制御や機械学習を行う際に結果の可視化は重要になります
今までMatlabを用いてお絵かきしてましたが，Pythonでも同じように書けると聞いたので，お絵かきしてみました

# 目的
Pythonでアニメーションを書く

# FuncAnimation
この関数は逐一，アニメーションを更新するものです
一例ですが，下記プログラムでは，あるfigをupdate_animで，新たに繰り返しかいています．ほかの引数は，
init_funcでfigを初期化します
framesで何回繰り返すか
intervalはそのままで，何秒間隔（ms）でupdateするか
blitはこれをTrueにすると早くなります，たぶんグラフィックの表示の仕方をいじってるんだと思います，公式サイトも見てみてください
なお，update_animは返り値として，fig_imageを返すことが必要になります

```py
animation = ani.FuncAnimation(fig, update_anim, init_func=init_anim, frames= A, interval=65, blit=True)
```

# プログラム
今回はボールが跳ねるアニメーションを作ります
ボールクラスでボールを作っていきます
以下プログラムです

```py

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

animation = ani.FuncAnimation(fig, update_anim, interval=50, frames=1000)

print('save_animation?')
shuold_save_animation = int(input())

if shuold_save_animation == True: 
    animation.save('basic_animation.mp4', writer='ffmpeg')

plt.show()
```

保存できません!!という方
これは，animationを書くものが入ってないからだと思います
anacondaを使用しているので
condaでhttps://anaconda.org/menpo/ffmpeg
をインストールしてみてください
もし，gifで書き出したい場合は
ffmepgでは無理なので，ImageMagick を入れてみてください
ただしちょっとめんどくさいです
ようはセルフインストールなので（Windowsだけです）
詳細は

https://kiito.hatenablog.com/entry/2013/11/27/233507

この辺を参考にお願いします
エラーとか出るかもですが，書き出せます

これで，書けました
![basic_animation.gif](https://qiita-image-store.s3.amazonaws.com/0/261584/e0894673-7882-30ba-7670-c46756991129.gif)


しかしなんかこう，面白くないので，以下のルールを追加しましょう
 * ボールは一定間隔で出現する
 
クラスにその要素を追加します
 
```py
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import math
import random
import sys
# ボールが跳ねるアニメーション

class Ball():
    # どのボールでも共通です
    max_x = 5
    min_x = -5
    max_y = 5
    min_y = -5

    def __init__(self):
        # 初期設定
        self.x = 0.1*random.randint(-30, 30)
        self.y = 0.1*random.randint(-30, 30)
        self.v_x = 0.1*random.randint(-50, 50)
        self.v_y = 0.1*random.randint(-50, 50)
        self.e = 0.1*random.randint(5, 10)#反発係数です
        self.dt = 0.05

    def state_update(self, ball_size):

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

class Drawing_ball():# 描画するクラス
    def __init__(self, ax):
        color_list = ['r', 'b', 'k', 'c', 'y', 'm', 'g']
        color_num = random.randint(0, len(color_list)-1)
        self.ball_img, = ax.plot([], [], color=color_list[color_num])
        self.ax = ax

    def draw_circle(self, center_x, center_y, balls_size):
        circle_size = balls_size

        steps = 100 #円を書く分解能はこの程度で大丈夫
        self.circle_x = [] #位置を表す円のx
        self.circle_y = [] #位置を表す円のy
            
        for j in range(steps):

            self.circle_x.append(center_x + circle_size*math.cos(j*2*math.pi/steps))
            self.circle_y.append(center_y + circle_size*math.sin(j*2*math.pi/steps))

        return self.circle_x, self.circle_y

    def set_graph_data(self):
        self.ball_img.set_data(self.circle_x, self.circle_y)

        return self.ball_img, 

def update_anim(i):# このiが更新するたびに増えていきます
    balls_x = []
    balls_y = []
    balls_img_x = []
    balls_img_y = []
    balls_imgs = []

    if i % 50 == 0: #ボール追加します
        balls_size.append(0.1*random.randint(2, 10))  
        balls.append(Ball())
        balls_drawers.append(Drawing_ball(ax))

    balls_num = int(i/50) + 1

    for j in range(balls_num):
        # ボールの状態をアップデート
        temp_x, temp_y = balls[j].state_update(balls_size[j])

        balls_x.append(temp_x)
        balls_y.append(temp_y)

        temp_x, temp_y = balls_drawers[j].draw_circle(balls_x[j], balls_y[j], balls_size[j])

        balls_img_x.append(temp_x)
        balls_img_y.append(temp_y)

        balls_imgs.append(balls_drawers[j].set_graph_data())
    
    # 時間
    step_text.set_text('step = {0}'.format(i))

    return balls_imgs

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

# ボールだけ作ってる
balls_num = 1
balls_size = []
balls = []
balls_drawers = []

# アニメーション描画
animation = ani.FuncAnimation(fig, update_anim, interval=50, frames=500)

print('save_animation?')
shuold_save_animation = int(input())

if shuold_save_animation == True: 
    animation.save('basic_animation.gif', writer='imagemagick')

plt.show()
```

はい
![anim.gif](https://qiita-image-store.s3.amazonaws.com/0/261584/e067e26e-1028-c13e-5949-a0d66e3659ae.gif)

ボールがどんどんでてきます（重いです．．．）
ここに，いろいろ動作（跳ね返る等）追加できたら面白いかもしれません
時間があればやろうかな

githubに挙げておきました
https://github.com/Shunichi09/Qiita/tree/master/Animation
確認お願いします！
