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