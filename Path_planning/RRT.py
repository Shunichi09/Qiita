import numpy as np
import matplotlib.pyplot as pyplot

class RRT():
    def __init__(self):
        # パラメータ群
        # 伸ばし幅
        self.d = 0.01

        # 探索範囲
        self.MAX_x = 10
        self.MAX_y = -10
        self.min_x = 10
        self.min_y = -10

        # ノードを作成する
        Nodes = []

    def search(self):
        # random点を打つ
        s_x = random()
        s_y = random()

        sample = [s_x, s_y]

        # ノード探索
        for node in Nodes:
            distance = (sample - node) * (sample - node)

        # 
