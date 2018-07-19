# はじめに
前回の[ROS＋MATLABでレーザーの値を取得して，描画してみた](https://qiita.com/MENDY/items/9274ba347bb8b524ec38)からの続きで，次はレーザーを用いて物体検出をしてみようと思います

# 目的
ROSとMATLABの共演

# 用意するもの
・Raspi(Ubuntu mate 16.04)
・PC(Matlabの入ったPC(2017a以上推奨))
→Robotics System Toolboxを入れてください
・UST-10(Hokuyo電機)

# アルゴリズム
レーザーのデータは角度と距離でPublishされます．
物体を検出したい場合は，距離と角度だけで考えなければなりません．
いたって簡単に
![図1.png](https://qiita-image-store.s3.amazonaws.com/0/261584/2bfb0dbf-a5be-5510-9a41-191f49bc54c3.png)

こんな感じです．
つまり，配列を見ていってへっこんだところを見れば良いです！
閾値とか見る範囲の補正は必要になりますが基本はこのアルゴリズムで十分．
ちなみに今回の僕のプログラムは見る範囲を1mにしています．
つまり1mと比較して，値がいくらか小さくなっているところは，物体があるということになります．
LRSの性能にもよりますがこれは10mまでみれるのでもったいないですねお試しです

# コード
## 関数
前回のコードもまとめてのせますが，detect_obsという関数を新たにつくります．この関数は距離情報と角度情報をうけとって，物体の位置（閾値が変わった位置，物体の検出が終わった位置）とその個数を受け取ります．

```php:detect_obs.m
function [obs_s_set, obs_f_set, obs_num] = detect_obs(distance, angle)

%物体検出パラメータ
distance_num = length(distance);%データの数
%1m以内を考慮する
basic_dis = 1 * ones(1, distance_num);
%検出サイズ
obs_size  = 0.15;
margin = 0.5;%誤差
%検出閾値（これを超えてたらなんかある）
dis_dif = 0.3;

%basicとの差を見る
dif = basic_dis - distance;

%記録用
j = 1;

%判定
obs_flag = false;

%大きくなったところを記録
for i = 30 : 1: distance_num-30
    if dif(i) < 0
       dif(i) = 0;        
    end
    
    if abs(dif(i)) > dis_dif
        obs_flag = true;
        start_p = i;
    end
    
    if obs_flag == true
        if abs(dif(i)) < dis_dif
            obs_flag = false;
            fin_p = i-1;
            %開始点と終了点の位置
            D_x_st = distance(start_p) * cos(angle(start_p));
            D_y_st = distance(start_p) * sin(angle(start_p));
            D_x_en = distance(fin_p) * cos(angle(fin_p));
            D_y_en = distance(fin_p) * sin(angle(fin_p));
            
            %物体の大きさ推定
            obs_temp_size = sqrt((D_x_st - D_x_en)^2+(D_y_st - D_y_en)^2);
            %狙い通りの大きさだったらカウント
            if obs_temp_size < obs_size + margin && obs_temp_size > obs_size - margin
                obs_s_set(j, :) = [D_x_st, D_y_st];
                obs_f_set(j, :) = [D_x_en, D_y_en];
                j = j + 1;
             end
        end
    end
end

obs_num = j -1;
if obs_num == 0
    obs_s_set = 0;
    obs_f_set = 0;
end
```

うまくいかない場合はパラメータや閾値をいじってください

描画です

```php:Laser_func.m
%% Laser_func
function laser_func(obj, message, src);

%% global変数(Publishノード）
global matlab_pub_node

%% massageを受信
distance = ((message.Ranges)');%距離データ（）
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

%% 物体検出
%物体検出
%obs_s_set, obs_f_set, obs_num = detect_obs(distance, angle);
[obs_s_set, obs_f_set, obs_num] = detect_obs(distance, angle)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 描画
refresh%リセット
 
for n_all = 1:1:distance_num
    X_all_Data(n_all) = distance(n_all) * cos(angle(n_all));
    Y_all_Data(n_all) = distance(n_all) * sin(angle(n_all));
end

plot(X_all_Data, Y_all_Data, 'LineWidth', 3, 'Color', [0 0 1]);
hold on

% 障害物描画
obs_size = 0.2;
if obs_num > 0
    for i_3 = 1: 1: obs_num
        %センターポジション取得
        Position_X_Data = (obs_f_set(i_3, 1) + obs_s_set(i_3, 1))/2;
        Position_Y_Data = (obs_f_set(i_3, 2) + obs_s_set(i_3, 2))/2;
        for n_ob = 0:1:100
            X_Data(n_ob+1) = Position_X_Data + obs_size * cos((n_ob)*2*pi/100);
            Y_Data(n_ob+1) = Position_Y_Data + obs_size * sin((n_ob)*2*pi/100);
        end
        plot(X_Data, Y_Data, 'LineWidth', 3, 'Color', [0 0.3 0]);
        clear X_Data Y_Data;
        hold on
    end
end

%描画範囲
XLIM = max(X_all_Data);
YLIM = max(Y_all_Data);
xlim([-XLIM XLIM]);
ylim([-YLIM YLIM]);
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

```

メイン関数です

```php:main.m
%% 初期化作業
clear
close all
%ROS初期化
%raspiのIPを入力（変更する場合）
rosinit('http://192.168.1.145:11311')

%% 各ノード立ち上げ
% Publisher Global 宣言
global matlab_pub_node

% Publisher
% Topic : /cmd_vel
% Topic Type : geometry_msgs/Twist
matlab_pub_node = rospublisher('/cmd_vel', 'geometry_msgs/Twist');

% Subscriber
% Topic : /Scan
% Topic Type : MatlabはSubscribeする場合はTopic名のみで受信可能
% Callback_func : laser_func
% Topicを受信し次第@laser_funcを実行（同フォルダ内にあります）
matlab_sub_node = rossubscriber('/scan', @laser_func);%レーザの値を取得してくるSubscriber

%% またはreceiveでも値を得ることが可能

%% 片付け
%rosshutdown
```

簡単です！
MATLABは非常に高価なのでそういう意味では邪道ですが，簡単に遊んでみたいときには非常に有用ですね

結果です
実際の映像
![unnamedwww (1).jpg](https://qiita-image-store.s3.amazonaws.com/0/261584/dcdd8df5-8f18-fcd5-2789-e2275b898819.jpeg)

Matlab上だと，こんなかんじ
![unnamed222.jpg](https://qiita-image-store.s3.amazonaws.com/0/261584/244b9bc5-596b-e6ee-325e-229cc0f35425.jpeg)

Github上にあげました
確認おねがいします
https://github.com/Shunichi09/Qiita/tree/master/Laser/laser_detec

Twitterもはじめました
https://twitter.com/ShunichiSekigu1
興味があればフォローお願いします
次はモータを動かします
