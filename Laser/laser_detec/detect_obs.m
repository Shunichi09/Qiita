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

