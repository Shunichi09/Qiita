import numpy as np
import cv2

'''
for i in range(1, 14):
    img1 = cv2.imread('./redata/test/test (' + str(i) + ').png', 0)
    
    for j in range(1 + i, 14):
        img2 = cv2.imread('./redata/test/test (' + str(j) + ').png', 0)

        # cv2.imshow("loaded", img1)
            # キーを待つ
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        
        # resizeする（200 * 200）
        size = (200, 200)
        img1 = cv2.resize(img1, size)
        img2 = cv2.resize(img2, size)
        
        #特徴抽出機の生成
        detector = cv2.xfeatures2d.SIFT_create()
        # detector = cv2.AKAZE_create()

        #kpは特徴的な点の位置 destは特徴を現すベクトル
        kp1, des1 = detector.detectAndCompute(img1, None)
        kp2, des2 = detector.detectAndCompute(img2, None)

        #特徴点の比較機
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des1,des2, k=2)

        #割合試験を適用
        good = []
        match_param = 0.7
        for m,n in matches:
            if m.distance < match_param*n.distance:
                good.append([m])

        #cv2.drawMatchesKnnは適合している点を結ぶ画像を生成する
        img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good, None,flags=2)
        cv2.imwrite('shift_result(' + str(i) +').png', img3)

        if len(good) > 2 :
            print(str(i) + str(j) + ' = Same people!!')
'''

# 
recognizer = cv2.face.LBPHFaceRecognizer_create()

recognizer = cv2.face.FisherFaceRecognizer_create()

images = []
labels = []

# トレーニング画像を取得
for i in range(1, 90):
    temp = cv2.imread('./redata/train_t/sample (' + str(i) + ').png', 0)
    size = (200, 200)
    temp = cv2.resize(temp, size)

    images.append(temp)
    labels.append(1)

for i in range(1, 90):
    temp = cv2.imread('./redata/train_iso/sample (' + str(i) + ').png', 0)
    size = (200, 200)
    temp = cv2.resize(temp, size)

    images.append(temp)
    labels.append(0)


# トレーニング実施
recognizer.train(images, np.array(labels))

# テスト
test_images = []
test_labels = []

# テスト画像を取得
for i in range(1, 8):
    temp = cv2.imread('./redata/test/test (' + str(i) + ').png', 0)
    size = (200, 200)
    temp = cv2.resize(temp, size)

    test_images.append(temp)
    test_labels.append(1)


for i in range(8, 16):
    temp = cv2.imread('./redata/test/test (' + str(i) + ').png', 0)
    size = (200, 200)
    temp = cv2.resize(temp, size)

    test_images.append(temp)
    test_labels.append(0)


i = 0

while i < len(test_labels):
    # テスト画像に対して予測実施
    label, confidence = recognizer.predict(test_images[i])
    # 予測結果をコンソール出力
    print("Test labels: {}, Predicted Label: {}, Confidence: {}".format(test_labels[i], label, confidence))
    # テスト画像を表示
    cv2.imshow("test image", test_images[i])
    cv2.waitKey(1000)

    i += 1

# 終了処理
cv2.destroyAllWindows()

