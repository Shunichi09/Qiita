import numpy as np
import math
import matplotlib.pyplot as plt

# thはラジアンです

# 回転行列の作成
# base_angle 何度回転させるか
def make_rot_mat(base_th):
    rot_matrix = [[np.cos(base_th), np.sin(base_th)], \
                  [-1*np.sin(base_th), np.cos(base_th)]]
    rot_matrix = np.array(rot_matrix)
    return rot_matrix

# 回転速度行列の作成
# 2つの行列を作成
def make_vel_rot_mat(base_th, base_ome):
    #第1項
    vel_rot_mat_1 = [[-1*np.sin(base_th), np.cos(base_th)], \
                     [-1*np.cos(base_th), -1*np.sin(base_th)]]
    vel_rot_mat_1 = base_ome * np.array(vel_rot_mat_1)
    #第2項
    vel_rot_mat_2 = [[np.cos(base_th), np.sin(base_th)], \
                     [-1*np.sin(base_th), np.cos(base_th)]]
    vel_rot_mat_2 = np.array(vel_rot_mat_2)

    return vel_rot_mat_1, vel_rot_mat_2

# 逆回転行列の作成
# baseはもともと回転させた角度
def make_inv_rot_mat(base_th):
    inv_rot_matrix = [[np.cos(base_th), -1 * np.sin(base_th)], \
                    [np.sin(base_th), np.cos(base_th)]]
    inv_rot_matrix = np.array(inv_rot_matrix)
    return inv_rot_matrix

# 逆回転速度行列の作成
# もとに戻す
def make_inv_vel_rot_mat(base_th, base_ome):
    #第1項
    inv_vel_rot_mat_1 = [[-1*np.sin(base_th), -1*np.cos(base_th)], \
                         [np.cos(base_th), -1*np.sin(base_th)]]
    inv_vel_rot_mat_1 = base_ome*np.array(inv_vel_rot_mat_1)
    #第2項
    inv_vel_rot_mat_2 = [[np.cos(base_th), -1*np.sin(base_th)], \
                         [np.sin(base_th), np.cos(base_th)]]
    inv_vel_rot_mat_2 = np.array(inv_vel_rot_mat_2)

    return inv_vel_rot_mat_1, inv_vel_rot_mat_2

def rotation_parallele(x, y, th, v_x, v_y, ome, base_x, base_y, base_th, base_v_x, base_v_y, base_ome):
    # ①下準備
    X = np.array([x, y, th, v_x, v_y, ome])
    base_X = np.array([base_x, base_y, base_th, base_v_x, base_v_y, base_ome])

    # 回転行列の作成
    rot_mat = make_rot_mat(base_th)
    vel_rot_mat_1, vel_rot_mat_2 = make_vel_rot_mat(base_th, base_ome)

    # ②平行移動
    para_X = X - base_X

    # ③回転移動
    rot_xy = np.dot(rot_mat, para_X[:2].reshape(2,1))
    rot_v_xy = np.dot(vel_rot_mat_1, para_X[:2].reshape(2,1)) + np.dot(vel_rot_mat_2, para_X[3:5].reshape(2,1))

    # 入れ直し
    para_rot_x, para_rot_y = rot_xy[0, 0], rot_xy[1, 0]
    para_rot_th = para_X[2]
    para_rot_v_x, para_rot_v_y = rot_v_xy[0, 0], rot_v_xy[1, 0]
    para_rot_ome = para_X[5]

    print('座標変換前：\n x　= {0} \n y = {1} \n th = {2} \n v_x = {3} \n v_y = {4} \n ome = {5}'\
            .format(x, y, th, v_x, v_y, ome))
    print('基準状態：\n x　= {0} \n y = {1} \n th = {2} \n v_x = {3} \n v_y = {4} \n ome = {5}'\
            .format(base_x, base_y, base_th, base_v_x, base_v_y, base_ome))
    print('座標変換後：\n x　= {0} \n y = {1} \n th = {2} \n v_x = {3} \n v_y = {4} \n ome = {5}'\
            .format(para_rot_x, para_rot_y, para_rot_th, para_rot_v_x, para_rot_v_y, para_rot_ome))
    
    return para_rot_x, para_rot_y, para_rot_th, para_rot_v_x, para_rot_v_y, para_rot_ome

if __name__ == '__main__':
    x, y, th = 1.0, 2.0, 0.5 # thはラジアンです
    v_x, v_y, ome = 0.5, 0.75, 0.5 # omeはrad/sです

    base_x, base_y, base_th = 1.3, 2.5, 0.5
    base_v_x, base_v_y, base_ome = 0.5, 0.75, 0.5

    rotation_parallele(x, y, th, v_x, v_y, ome, base_x, base_y, base_th, base_v_x, base_v_y, base_ome)
