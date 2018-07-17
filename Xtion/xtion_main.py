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

class Images():
    def __init__(self, image):
        self.image = image
    
    def size_change(self):
        # サイズゲット
        image_h, image_w = self.image.shape[:2]
        # 縮小
        size = (image_h/2, image_w/2)
        self.image = cv2.resize(self.image, size)

    def show_image(self):
        cv2.imshow("Origin Image", self.image)

    def resize(self):
        size = (200, 200)
        self.resize_image = cv2.resize(self.image, size)
        return self.resize_image
    
    def togray(self):
        self.gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        return self.gray_image


class Human_and_face_detect:
    def __init__(self):
        ## ROS関係
        # Nodeを立ち上げる
        # Publisher
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        # これは、ROS_imageをOpencv_imageに変換するのに必要
        self.bridge = CvBridge()
        # Subscriber
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        ## 顔認識クラス
        self.face_detec = Face_detec()
        ## 顔識別クラス
        self.face_classify = Face_classify()
        self.face_classify.train() # 学習させる
        ## オプティカルフロー
        self.optical_flow = Optical_flow_lk()
        # self.optical_flow = Optical_flow_farneback()
        # counter
        self.init_flag = True

    def callback(self,data):# topicを受信するたびに、このコールバック関数が呼ばれる
        # Topicの呼び出し
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.original_image = copy.copy(self.cv_image)

        except CvBridgeError as e:
            print('Cv_Bridge_Error')

        # RGBからgrey_scaleに変換                                                           
        self.gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        # 顔をdetect
        face_image, detected_flag, faces_point = \
                    self.face_detec.detect(self.gray_image, self.original_image)
        
        # if 顔があったら
        person_flags = []
        if detected_flag == True:
            # 顔を比較、顔識別
            person_flags, confidences = self.face_classify.classify(face_image)

        detected_image = self.face_detec.write_face(person_flags)
        person_flags = True
        
        # optical flow（顔の特徴点を使う）
        result_image, optical_flow_image = self.optical_flow.flow(self.gray_image, self.cv_image, detected_image, faces_point, self.init_flag)

        # 画面サイズ変更
        # ウインドウのサイズを変更                                                               
        half_image = cv2.resize(self.original_image, (0,0), fx=0.6, fy=0.6)
        half_detected_image = cv2.resize(detected_image, (0,0), fx=0.6, fy=0.6)
        half_opticaled_flow_image = cv2.resize(optical_flow_image, (0,0), fx=0.6, fy=0.6)
        half_result_image = cv2.resize(result_image, (0,0), fx=0.6, fy=0.6)

        # ウインドウ表示                                                                         
        cv2.imshow("Origin Image", half_image)
        cv2.imshow("Face datected Image", half_detected_image)
        cv2.imshow('Optical flow Image', half_opticaled_flow_image)
        cv2.imshow('Reasult Image', half_result_image)
        cv2.waitKey(3)

        # Counterを加算
        self.init_flag = False
    
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(detected_image, "bgr8"))
        except CvBridgeError as e:
            print('CV_Bridge_Error')


# Optical_flowを行うクラス
class Optical_flow_lk():
    def __init__(self):
        # lk法パラメータ
        self.lk_params = dict(winSize = (15, 15), maxLevel = 2, \
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        # 特徴量パラメータ
        self.feature_params = dict(maxCorners = 100, qualityLevel = 0.3, minDistance = 7, blockSize = 7)
        # 色
        self.color = np.random.randint(0,255,(100,3))
        # 前の画像
        self.pre_gray_image = None
        # 今の画像
        self.gray_image = None
        # 後で使う
        self.p0 = None
        self.init_mask = None
        self.original_init_mask = None
        # 追跡開始フラグ
        self.tracking_flag = False
        self.tracking_count = 0 

    def flow(self, gray_image, original_image, detected_image, faces_point, init_flag):
        if init_flag == False:# オプティカルフロー実行
            self.gray_image = gray_image          
            p1, st, err = cv2.calcOpticalFlowPyrLK(self.pre_gray_image, self.gray_image, self.p0, None, **self.lk_params)
            # 書く(顔が先に入ってる奴)
            optical_flow_image = copy.copy(detected_image)
            # 顔が入ってない奴
            original_flow_image = copy.copy(original_image)

            try:#センサの値がとれたら！
                good_new = p1[st > 0]
                good_old = self.p0[st > 0]
                # x, y, w, zにする
                self.w = 0
                
                for point in faces_point:
                    if point[2] > self.w:#基本的には一番おおきいサイズ
                        self.x = point[0]
                        self.y = point[1]
                        self.w = point[2]
                        self.h = point[3]

                for i,(new,old) in enumerate(zip(good_new,good_old)):
                    a,b = new.ravel()
                    c,d = old.ravel()
                    mergin = 50

                    # 関係なく描写するやつ
                    original_mask = cv2.line(self.original_init_mask, (a,b),(c,d), self.color[i].tolist(), 2)
                    original_flow_image = cv2.circle(original_flow_image,(a,b),5,self.color[i].tolist(),-1)

                    if self.tracking_flag == True:
                        mergin = 100

                    # 特徴点が顔の中に入ってたら[y:y+h, x:x+w]
                    if a > self.x - mergin and a < self.x + self.w + mergin and \
                                            b > self.y - mergin and b < self.y + self.h + mergin:
                        mask = cv2.line(self.init_mask, (a,b),(c,d), self.color[i].tolist(), 2)
                        optical_flow_image = cv2.circle(optical_flow_image,(a,b),5,self.color[i].tolist(),-1)

                        self.tracking_count += 1
                    else:
                        # tracking 失敗
                        # self.tracking_flag = False
                        self.tracking_count -= 1
                        if self.tracking_count < 0:
                            self.tracking_count = 0
                    
                    # トラックするかどうか
                    if self.tracking_count > 1:
                        # tracking 開始
                        self.tracking_flag = True
                    else:
                        # tracking 開始
                        self.tracking_flag = False

                original_flow_image = cv2.add(original_flow_image, original_mask)

                opticaled_flow_image = cv2.add(optical_flow_image, mask)

                # 1つ前の特徴量
                self.p0 = good_new.reshape(-1, 1, 2)

            except:
                opticaled_flow_image = detected_image

            # print(self.tracking_count)

            # 1つ前の保存する
            self.pre_gray_image = gray_image
            # optical_flowを常に更新
            if self.tracking_flag == False:
                self.init_mask = np.zeros_like(original_image)
            
            return opticaled_flow_image, original_flow_image # 顔あり顔なし
        
        else:
            print('initialized!!')
            # 1つ前の保存する
            self.pre_gray_image = gray_image
            # 1つ前の特徴量
            self.p0 = cv2.goodFeaturesToTrack(self.pre_gray_image, mask = None, **self.feature_params)
            # Maskで使うもの
            self.init_mask = np.zeros_like(detected_image)
            # Originakで使うもの
            self.original_init_mask = np.zeros_like(original_image)

            return detected_image, original_image
            
# Optical_flowを行うクラス（farneback）
class Optical_flow_farneback():
    def __init__(self):
        # 色
        self.color = np.random.randint(0,255,(100,3))
        # 前の画像
        self.pre_grey_image = None
        # 今の画像
        self.grey_image = None
        # 書き出すときのbase image
        self.base_image = None

    def flow(self, grey_image, original_image, init_flag):
        if init_flag == False:# オプティカルフロー実行
            self.grey_image = grey_image

            flow = cv2.calcOpticalFlowFarneback(self.pre_grey_image,self.grey_image,\
                                                 None, 0.5, 3, 15, 3, 5, 1.2, 0)

            mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
            self.base_image[...,0] = ang*180/np.pi/2
            self.base_image[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
            optical_flow_image = cv2.cvtColor(self.base_image,cv2.COLOR_HSV2BGR)
            
            return optical_flow_image

        else:
            # 1つ前の保存する
            self.pre_grey_image = grey_image
            # 書き出す用の作成
            self.base_image = np.zeros_like(original_image) 

            return original_image

# 顔認識を行うクラス
class Face_detec():
    def __init__ (self):
        self.face_cascade_path = './library/haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(self.face_cascade_path)

    def detect(self, gray_image, original_image):
        # Copyしておく
        self.detected_image = copy.copy(original_image)
        face_image = []
        detected_flag = False
        # 初期化
        self.faces = None

        self.faces = self.face_cascade.detectMultiScale(gray_image, minNeighbors=5, minSize=(30,30))

        for point in self.faces:
                x = point[0]
                y = point[1]
                w = point[2]
                h = point[3]

                # 画像を切り抜く ## XXX:ここ怪しい！！！！！！！！
                face_image.append(gray_image[y:y+h, x:x+w])
                
                # cv2.imshow('a', gray_image[y:y+h, x:x+w])
                # 顔検出済み
                detected_flag = True
        
        return face_image, detected_flag, self.faces
    
    def write_face(self, person_flags):
        try:
            i = 0
            for point in self.faces:
                x = point[0]
                y = point[1]
                w = point[2]
                h = point[3]
                person_flags[i] = True
                if person_flags[i] == True:# 人物識別された場合
                    self.detected_image = cv2.rectangle(self.detected_image, (x,y), (x+w, y+h), (0, 0, 255), 2)
                    self.detected_image = cv2.putText(self.detected_image, 'KOYANAGI!!', \
                                        (x, y-10), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2, cv2.LINE_AA)
                else:
                    self.detected_image = cv2.rectangle(self.detected_image, (x,y), (x+w, y+h), (255, 0, 0), 2)
                    self.detected_image = cv2.putText(self.detected_image, 'No_Koyanagi', \
                                        (x, y-10), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2, cv2.LINE_AA)
                
                i += 1
        except:
            print('No faces!!')

        return self.detected_image

# 顔識別を行うクラス
class Face_classify():
    def __init__(self):
        # 学習機作成
        self.recognizer = cv2.face.FisherFaceRecognizer_create()
        # self.recognizer = cv2.face.LBPHFaceRecognizer_create()
        self.train_images = []
        self.train_labels = []
        self.test_images = []
        self.test_labels = []
    
    def read_train(self, train_num, point):
        self.train_num = train_num
        for i in range(1, train_num+1):
            if i < point:# 正解データと不正解データを分ける
                train_image = Images(cv2.imread('./train/sample (' + str(i) + ').png', 0))
                self.train_images.append(train_image.resize())
                self.train_labels.append(1)
            else:
                train_image = Images(cv2.imread('./train_uso/sample (' + str(i-point+1) + ').png', 0))
                self.train_images.append(train_image.resize())
                self.train_labels.append(0)
    
    def train(self):
        # トレーニング実施
        self.read_train(180, 90)
        self.recognizer.train(self.train_images, np.array(self.train_labels))

    def classify(self, face_image):
        labels = []
        confidences = []
        for i in range(len(face_image)):
            temp_face_image = Images(face_image[i])
            input_image = temp_face_image.resize()
            # 予測
            label, confidence = self.recognizer.predict(input_image)
            
            if confidence < 100:#信頼度低い場合
                labels.append(0)
                confidences.append(confidence)
            else:
                labels.append(label)
                confidences.append(confidence)
        
        return labels,confidences

def main(args):
    human_detecter = Human_and_face_detect()
    rospy.init_node('image_converter', anonymous=True)

    try:
        rospy.spin()
    
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
