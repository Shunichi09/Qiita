# はじめに
ROSとMATLABが共演できるのにもかかわらず，意外と知られていないのと参考になるプログラムが少ないので書いてみます
しかもROSを普通のPCに入れるのは大変です　RaspiであればすぐできるのでRaspi+PCでやってみます

# 目的
ROSとMatlabの併用

# 詳細
ROSとMATLABを同時に扱えると例えばROSの入ったRaspiでLaserの値を取得，それをMATLAB上で取得して，ロボットのモータの指令値を生成（Matlabの制御ライブラリツールを使える→ここ重要）して，RaspiのGPIOを介して，Raspiからモータを動かすなんてこともできる

# 用意するもの
・Raspi(Ubuntu mate 16.04)
・PC(Matlabの入ったPC(2017a以上推奨））
・UST-10(Hokuyo電機）

![IMG-5295.JPG](https://qiita-image-store.s3.amazonaws.com/0/261584/c1865448-e48b-8e19-68c2-8a9794397173.jpeg)


こんな感じで配線


# コード
①Raspiでroscreを立ち上げます

```
roscore
```

②RaspiでLRSを使えるライブラリを起こします
インストール方法は探せばあるのでぐぐってください
（ROSのいいところはこういう風にライブラリを使えることにありますね）

③Matlabのコードはこんな感じ
Main file

```php:main.m
%% 初期化作業
clear
close all
% ROS初期化
% raspiのIPを入力（変更する場合）
% ROSCOREのIPをいれてください
rosinit('http://'ここにIP':11311')

%% 各ノード立ち上げ

% Subscriber
% Topic : /Scan
% Topic Type : MatlabはSubscribeする場合はTopic名のみで受信可能
% Callback_func : laser_func
% Topicを受信し次第@laser_funcを実行

matlab_sub_node = rossubscriber('/scan', @laser_func);%レーザの値を取得してくるSubscriber

%% またはreceiveでも値を得ることが可能

%% 片付け
rosshutdown
```

laser_func関数では

```php:laser_func.m
%% Laser_func
function laser_func(obj, message, src);

%% global変数(Publishノード）
global matlab_pub_node

%% massageを受信
distance = ((message.Ranges);%距離データ（）
distance_num = length(distance);%データの数
Max_angle = message.AngleMax;%最大角度
min_angle = message.AngleMin;%最小角度
angle_step = message.AngleIncrement;%ステップ角度
angle = min_angle: angle_step :Max_angle;%角度行列

%Laserのスペック
max_det_dis = 25;

%外れ値処理（値がとれずに35となった場合）
for n = 1:1:distance_num
    if distance(n) > max_det_dis && n > 1%端以外の外れ値は1つ前の角度の距離と同様
        distance(n) = distance(n-1);        
    elseif n == 1 && distance(n) > max_det_dis%端は0に
        distance(n) = 0;
    end
end

%% ここから下に処理を書いてください %%%%%%%%%%%%%%
%distanceは距離データ行列
%angleは角度行列



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 描画
refresh%リセット
 
for n_all = 1:1:distance_num
    X_all_Data(n_all) = distance(n_all) * cos(angle(n_all));
    Y_all_Data(n_all) = distance(n_all) * sin(angle(n_all));
end

plot(X_all_Data, Y_all_Data, 'LineWidth', 3, 'Color', [0 0 1]);
hold on
%描画範囲
XLIM = max(X_all_Data);

YLIM = max(Y_all_Data);
xlim([-XLIM XLIM]);
ylim([-YLIM YLIM]);
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
```

MatlabのROSライブラリはreceiveを用いてもTopicの値を取得できますがあまりROSっぽくないので，callback関数を用いてます
実行してみると，
描画してくれました！！
次回はこれを使ってものを検出するアルゴリズムをやってみます！
なおプログラムはgithubに上がってます

https://github.com/Shunichi09/Qiita/tree/master/Laser/laser

Twitterも確認お願いします
https://twitter.com/ShunichiSekigu1
