%% Laser_func
function control_func(obj, message, src);

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


%% message送信する場合ここ！
command = rosmessage(matlab_pub_node);

send(matlab_pub_node, command);

