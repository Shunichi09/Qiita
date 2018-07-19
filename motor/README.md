# はじめに
Laser[ROS+MATLABで物体検出してみた](https://qiita.com/MENDY/items/7f655c1e7f5d8027b42a)では物体検出までやってみましたが，今度はmatlabはまったく関係なく，ROSでモータを回すということをやります

# 目的
ROSでRaspiからモータを回す

# 用意するもの
・Raspi(Ubuntu16.04)/type B
・電源
・Maxon Motor
・ESCON
のみ！
RaspiにはGPIOがあるので良いですね
mbedなどなどはいりません

# プログラム構成
一般的に，当たり前ですがモータはモータドライバを用いて制御をします
今回はこのESCONがモータドライバになってモータをコントロールします
Raspiがやるのは，モータドライバに指示をだすところ！
モータドライバへの指示はPMWでだします
ので，RaspiからいくつのPWMでだしてね！！(duty比ですが）と指示をだせばモータは回るということです．

長々と説明しましたが，要は
Raspi(PWM)→ESCON→モータ(Maxon)ってことです

さらに，プログラム構成ですが，ROSをかじった人ならわかると思いますが，普通，cmd_velという形で（Geometry_msgs/Twist)で指示をもらうはずです！（速度，角速度）
よってそれを回転数に変換
回転数からduty比を計算
duty比をESCONに送信
ESCONがPWMを送ってくれる
というわけです
二輪想定なので、左右のモーターに送ります

イメージ図
![図mtor1.png](https://qiita-image-store.s3.amazonaws.com/0/261584/f24b28b4-659c-cea7-31be-c9cb8f699e0d.png)


# 事前準備
ESCONを使う場合は、事前にいろいろ設定する必要があります。
ESCONの設定用ソフトウェアをPCにインストールして、USBのケーブルで接続してください
プロパティを押して、Maxonの注文書や型番を見ながらポチポチします

# 接続
ESCONの説明書を見ながら接続します
こんな感じ
![図1cccc.png](https://qiita-image-store.s3.amazonaws.com/0/261584/ee826253-ffe7-e5ed-ffb1-f940e0328d7c.png)

ここで、Raspiのピン番号を決めてください！
![image.png](https://qiita-image-store.s3.amazonaws.com/0/261584/3b684cd3-eb33-e120-ce6e-2af77aef3f6e.png)
ただし！
GPIOのライブラリとしてRPi.GPIO
を使う場合、GND以外は，どのピンでも基本変わりません(ソフトウェアPWMなので)
他のライブラリを使う場合は注意してください

# コード
さてコードですが，まずTwistを変換するノードを作ります
2輪を想定していますので，タイヤ幅や，車軸の長さなどは適宜お願いします

```php:twist2int64.py
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64

class Twist2int64():
    def __init__(self):
	    self.command_left = Int64()
        self.command_right = Int64()
        self.received_twist = None
        rospy.init_node('Twist2int64')
        rospy.Subscriber('motor/twist/cmd_vel', Twist, self.callback)
        self.pub_right = rospy.Publisher('right_motor/cmd_vel', Int64, queue_size=10)#name, topic_type, size
        self.pub_left = rospy.Publisher('left_motor/cmd_vel', Int64, queue_size=10)#name, topic_type, size

    def main_twist2int64(self):
        
        rospy.spin()

    def callback(self, message):
        self.received_twist = message #input data
        self.command_right, self.command_left = self.twist2rpm(self.received_twist)
        self.pub_right.publish(self.command_right)
        self.pub_left.publish(self.command_left)

    def twist2rpm(self, received_data):#convert to speed
        #(m/s, rad/s)
        wheeles_size = 0.075#wheel size
        axle_length = 0.35#axle_size(2d)

        v = received_data.linear.x#(m/s)
        omega = received_data.angular.z#(rad/s)

        v_r = (omega*axle_length + 2*v)/2
        v_l = (omega*axle_length - 2*v)/(-2)

        v_r = v_r/(wheeles_size * 2 * 3.14) #wheel_speed(1/s)
        v_l = v_l/(wheeles_size * 2 * 3.14) #wheel_speed(1/s)
        r_rpm = 60 * v_r * 19 #gear rate
        l_rpm = 60 * v_l * 19 #gear rate

        return r_rpm, l_rpm

#Main Program
Convert = Twist2int64()
Convert.main_twist2int64()

#pub memo
	#rostopic pub motor/twist/cmd_vel geometry_msgs/Twist '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

```

さらにそれをduty比にしてPWMを送るプログラムです

```php:motor_right.py
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO#GPIOの設定
import time
import rospy
from std_msgs.msg import Int64

class motor_command():
    def __init__(self):#初期化
        #初期変数		
        self.RotationDirect = True #Forward
        self.Stop = False #Forward
        self.Enable = True
        self.pin_num_PWM = 18#pinの名前
        self.pin_num_RotationDirect = 15
        self.pin_num_Enable = 14
        self.pin_num_Stop = 23
        self.freq = 50 # Hz (PWM のパルスを一秒間に 50 個生成)
        #ノード立ち上げ（ROS）
        rospy.init_node('maxon_test')
        rospy.Subscriber('right_motor/cmd_vel', Int64, self.callback)
        rospy.loginfo('Ready to receive motor command')

        #初期設定（GPIO）
        GPIO.setmode(GPIO.BCM)
        # GPIOをデジタル出力に設定 GPIOのセットアップ
        GPIO.setup(self.pin_num_PWM, GPIO.OUT) #PWM
        GPIO.setup(self.pin_num_RotationDirect, GPIO.OUT) #rotation direct
        GPIO.setup(self.pin_num_Enable, GPIO.OUT) #enable
        GPIO.setup(self.pin_num_Stop, GPIO.OUT) #stop`
        # PWM 設定
        GPIO.output(self.pin_num_RotationDirect, GPIO.LOW)
        GPIO.output(self.pin_num_Enable, GPIO.LOW)
        GPIO.output(self.pin_num_Stop, GPIO.HIGH)
        self.pwm = GPIO.PWM(self.pin_num_PWM, self.freq)
        duty = 0.0 # デューティー比 0.0 で出力開始 (パルス内に占める HIGH 状態の時間が 0.0 %)
        self.pwm.start(duty)

    def motor_main(self):
        rospy.spin()
        #お片付け
        GPIO.cleanup()
        rospy.loginfo('回転終了')
        self.pwm.stop()#終了
		
    def callback(self, data):
        self.rpm_command = data.data#rpmで取得
        rospy.loginfo(self.rpm_command)
        duty_command = self.rpm2duty(self.rpm_command)#rpm→duty比へ（制約考慮）
        rospy.loginfo('motor command is '+ str(duty_command))
        #PINいじり
        if duty_command >= 0:#正逆回転
            self.RotationDirect = True
        else:
            self.RotationDirect = False#逆回転
            rospy.loginfo('Reverse')
            duty_command = -duty_command

        if self.RotationDirect:
            GPIO.output(self.pin_num_RotationDirect,GPIO.HIGH)
        else:
            GPIO.output(self.pin_num_RotationDirect,GPIO.LOW)

        if self.Stop:
            GPIO.output(self.pin_num_Stop, GPIO.HIGH)
        else:
            GPIO.output(self.pin_num_Stop, GPIO.LOW)

        if self.Enable:
            GPIO.output(self.pin_num_Enable, GPIO.HIGH)
        else:
            GPIO.output(self.pin_num_Enable, GPIO.LOW)
	
        rospy.loginfo('Change Command')

        self.pwm.ChangeDutyCycle(duty_command)#回転量を変える

    def rpm2duty(self, rpm):
        max_rpm = 6000 #規定値90%
        max_duty = 90
        min_rpm = 0 #規定値10%
        min_duty = 10
        rpm_rate = (max_rpm-min_rpm)/(max_duty-min_duty)#1%に対するrpm
        if rpm >= 0:
            duty = rpm / rpm_rate + min_duty
        else:
            duty = rpm / rpm_rate - min_duty  
        #速度制約
        if duty > 90:
            rospy.loginfo('Speed limit!!')
            duty = 90
        elif duty < -90:
            rospy.loginfo('Speed limit!!')
            duty = -90
        return duty

if __name__ == '__main__':#これがメイン実行file
    Motor_Command = motor_command()
    Motor_Command.motor_main()
        #pubする場合の参考
        #rostopic pub right_motor/cmd_vel std_msgs/Int64 "data: 10
```

今回は二輪ロボットを想定しているので左右のモーターに指示を送ります
これは左ですね！

安定化電源の電圧は決められたものにしてください
普通は12Vです

これらのプログラムを用いて，

```
roscore
```

で後は2つのプログラムを順に回してください！！

ただ，これだけでは回らないので

```
rostopic pub motor/twist/cmd_vel geometry_msgs/Twist '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
```

でpublishしてください！

# 最後に
Rapiを使ってモータを回しました
次はエンコーダをとってみましょう！
しかし，Maxonモータはエンコーダのパルス数が多いので．．．．
Raspiで対応できないかもしれないですね．．．
工夫案ある方募集中です

今日のは
https://github.com/Shunichi09/Qiita/tree/master/motor
に上がってます
確認お願いします

https://twitter.com/ShunichiSekigu1


