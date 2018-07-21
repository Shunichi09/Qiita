# はじめに
授業で、Xtion Pro Live を使うタイミングがあったので，使い方まとめてみます！
![image.png](https://qiita-image-store.s3.amazonaws.com/0/261584/550fca89-efe9-a4ab-ffcc-1dbb5cb7a8cc.png)
Xtion　公式ホームページ，https://www.asus.com/jp/3D-Sensor/Xtion_PRO_LIVE/ 

githubにもあがってます！
https://github.com/Shunichi09/Qiita/tree/master/Xtion

# 目的
Xtion Pro Live の使い方の解説
ROS化とOpencvとの連携を軸に！


# 環境
- Xtion pro live
- Ubuntu 16.04 LTS
- Python 3.6.5
- OpenCV 3.3.1
- ROS Kinetic

# まず用意すべきもの
ROSでXtionのような、USBカメラを使いたい場合は、

openni2

をインストールします
openniでもいいんですが、こっちの方がXtion pro liveにはみたいですね！
http://wiki.ros.org/openni2_launch

```
sudo apt-get install ros-kinetic-openni2
```

あとはopencv!!
pipでできます

さらにROSイメージをopencvのイメージに変換するために、bridgeが以下の図のように必要になります
http://wiki.ros.org/cv_bridge
<img width="178" alt="キャプチャ.PNG" src="https://qiita-image-store.s3.amazonaws.com/0/261584/0bbe7a75-096a-4230-0b22-1f3f7d7a2c72.png">

```
sudo apt-get update
sudo apt-get install python-cv-bridge
```

準備完了です！

# OpenCVを用いて画面にカメラのRGB画像を表示させる

通常ただ表示させるだけなら

```
 rosrun image_view image_view image:=/image_raw
```

このプログラムで表示できますが，これだけではopencv機能が使用できません

なので
cvbridgeを用いて変更して使えるようにします

```py:hyouzi.py
#!/usr/bin/env python
# -*- coding: utf-8 -*-                                                                   

# 動作確認済み
# roslaunch openni2_launch openni2.launch 

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import copy

class Xtion:
    def __init__(self):
        ## ROS関係
        # Publisher
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        # これは、ROS_imageをOpencv_imageに変換するのに必要
        self.bridge = CvBridge()
        # Subscriber
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

    def callback(self,data):# topicを受信するたびに、このコールバック関数が呼ばれる
        # Topicの呼び出し
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.original_image = copy.copy(self.cv_image)

        except CvBridgeError as e:
            print('Cv_Bridge_Error')

        # RGBからgrey_scaleに変換                                                           
        gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        # 画面サイズ変更
        # ウインドウのサイズを変更                                                               
        half_image = cv2.resize(self.original_image, (0,0), fx=0.5, fy=0.5)

        # ウインドウ表示                                                                         
        cv2.imshow("Origin Image", half_image)
        cv2.imshow('Reasult Image', gray_image)
        cv2.waitKey(3)
    
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(half_image, "bgr8"))
        except CvBridgeError as e:
            print('CV_Bridge_Error')

def main(args):
    xtion_disp = Xtion()
    rospy.init_node('image_disp_pub', anonymous=True) # ノード立ち上げ

    try:
        rospy.spin()
    
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

```

これで完了！
オリジナルの画像とグレー画像も表示されると思います
顔認識などのリアルタイム処理もこれでできます！
今度プログラムをあげます！

たぶんこんな感じになる
<img width="195" alt="無題.jpg" src="https://qiita-image-store.s3.amazonaws.com/0/261584/1842b5f7-d785-c22c-ff37-83b2920a3075.jpeg">

いろいろ隠してあるのはご理解ください

# 補足
上記に示した環境において！（他環境ではわかりません）
ROSとopenCVの混在にちょっとだけつまったので解説を
opencvを入れる時にROSがもともと入ってると，インストールされるのに
単にubuntuでpython3系で使いたいときにROSのPathが邪魔して，opencvが使えない（Pathが通らない）という問題が発生します
pythonの優先順位が勝手に変更されてるみたいですね
[参考までに](http://ossyaritoori.hatenablog.com/entry/2017/06/22/ROS%E3%81%A8%E3%81%AE%E7%AB%B6%E5%90%88%E3%82%92%E9%81%BF%E3%81%91%E3%81%AA%E3%81%8C%E3%82%89Ubuntu%E3%81%A7OpenCV%E7%92%B0%E5%A2%83%E3%82%92%E5%86%8D%E6%A7%8B%E7%AF%89)
つまり，ROS上でOpenCVを使う場合は何にも問題ないですが
単に，UbuntuPCでPython3系でopencvを使いたい場合，使えません
以下のエラーが出ると思います！

```
ImportError: /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so: undefined symbol: PyCObject_Type
```

ROSの方には入ってないよ！と怒られる．．．
なので単にUbuntuでpython3系でopencvを使用したい場合は，仕方ないので
以下のプログラムを追加してください

```
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
```

意外と海外の方も苦戦しているようでしたので，もし同じところで詰まった方の手助けになれれば幸いです
