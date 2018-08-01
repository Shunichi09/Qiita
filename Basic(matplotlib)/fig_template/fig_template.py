import numpy as np
import matplotlib.pyplot as plt

# 全体の体裁
plt.rcParams['font.family'] = 'Times New Roman' #全体のフォントを設定
plt.rcParams['font.size'] = 20 #フォントサイズを設定
plt.rcParams['lines.linewidth'] = 3 

class Formal_ploter():
    def __init__(self, x, y, x_label_name, y_label_name):
        # figure作成
        self.fig = plt.figure(figsize=(8,6),dpi=100) # ここでいろいろ設定可能

        # subplot追加
        self.axis = self.fig.add_subplot(111)

        # dataの収納
        self.x = x
        self.y = y

        # 最大値と最小値
        self.x_max = max(self.x)
        self.x_min = min(self.x)
        self.y_max = max(self.y)
        self.y_min = min(self.y)

        # 軸（label_name）
        self.axis.set_xlabel(x_label_name)
        self.axis.set_ylabel(y_label_name)

        # 軸（equal系）
        self.axis.axis('equal')

        # 軸（limit）# 調整可能
        self.axis.set_xlim([self.x_min, self.x_max])
        self.axis.set_ylim([self.y_min, self.y_max])

        # grid
        self.axis.grid(True)

        # color
        self.colors = ['r', 'b', 'k', 'm']

    def plot(self):
        self.axis.plot(self.x, self.y)
        plt.show()

class Formal_mul_ploter():
    def __init__(self, x, y, x_label_name, y_label_name, y_names):
        # figure作成
        self.fig = plt.figure(figsize=(8,6),dpi=100) # ここでいろいろ設定可能

        # subplot追加
        self.axis = self.fig.add_subplot(111)

        # dataの収納
        self.x = x
        self.y = y

        # legendの時に使う
        self.y_names = y_names

        # 最大値と最小値
        self.x_max = np.max(self.x)
        self.x_min = np.min(self.x)
        self.y_max = np.max(self.y)
        self.y_min = np.min(self.y)

        # 軸（label_name）
        self.axis.set_xlabel(x_label_name)
        self.axis.set_ylabel(y_label_name)

        # 軸（equal系）
        # self.axis.axis('equal')

        # 軸（limit）# 調整可能
        # self.axis.set_xlim([self.x_min, self.x_max])
        # self.axis.set_ylim([self.y_min, self.y_max])

        # grid
        self.axis.grid(True)

        # color 好きなカラーリスト作っとく
        self.colors = ['r', 'b', 'k', 'm']

    def plot(self):
        for i in range(len(self.y)):
            self.axis.plot(self.x, self.y[i], label=self.y_names[i], color=self.colors[i])

        self.axis.legend()

        plt.show()

def main():
    x = np.arange(0, 50)
    y = np.random.rand(50)
    x_label_name = '$\it{PY_{thon}}$'
    y_label_name = '$\it{PY_{thon}}$'

    ploter = Formal_ploter(x, y, x_label_name, y_label_name)
    ploter.plot()

    x = np.arange(0, 50)
    y1 = np.random.rand(50)
    y2 = np.random.rand(50)
    y3 = np.random.rand(50)
    y4 = np.random.rand(50)

    y = [y1, y2, y3, y4]

    y_names = ['y1', 'y2', 'y3', 'y4']

    x_label_name = '$\it{PY_{thon}}$'
    y_label_name = '$\it{PY_{thon}}$'

    ploter = Formal_mul_ploter(x, y, x_label_name, y_label_name, y_names)
    ploter.plot()

if __name__ == '__main__':
    main()
