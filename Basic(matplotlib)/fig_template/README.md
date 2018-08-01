# はじめに
中間発表に向けて，グラフの体裁を整える必要があったので．その時のメモ

github：https://github.com/Shunichi09/Qiita/tree/master/Basic(matplotlib)

# 環境
- python3.6.6
- Windows 10
- anaconda
- matplotlib 2.2.2
- numpy 1.14.3

# 体裁
グラフの体裁にこだわるときは以下をこだわることが多い

- 軸とかタイトルのフォント
    - Times New Roman
    - 大きさ（パワポに貼るときは20pt以上）
    - 変数は斜体に

- グラフ内
    - Grid on　(格子だと見やすいと勝手に思ってる)
    - 単位を忘れない
    - グラフの線の太さは（3pt）

以上のことを達成しやすい，テンプレートを作成した

# プログラム
**個別で設定しなきゃなのかなともったらまとめ設定が存在するんかい！！**

```py
plt.rcParams['font.family'] = 'Times New Roman' #全体のフォントを設定
plt.rcParams['font.size'] = 20 #フォントサイズを設定
plt.rcParams['lines.linewidth'] = 3 # 線の太さ設定
```

これでまとめて設定できてしまうという．．．
なんだい

以下は少し自分用に加工したもの
その下は複数の入力にも対応できるようにしたもの

```py
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
        self.axis.axis('equal') # こうすると軸が正方形に

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

def main():
    x = np.arange(0, 50)
    y = np.random.rand(50)
    x_label_name = '$\it{PY_{thon}}$'
    y_label_name = '$\it{PY_{thon}}$'

    ploter = Formal_ploter(x, y, x_label_name, y_label_name)
    ploter.plot()

if __name__ == '__main__':
    main()

```

実行するとこうなる

![Figure_1.pngrff.png](https://qiita-image-store.s3.amazonaws.com/0/261584/38c72774-2677-8e95-45d3-33f3cb523a8a.png)

複数の場合は以下のプログラム

```py

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
```

![aaaaaa.png](https://qiita-image-store.s3.amazonaws.com/0/261584/583dc90a-67a6-f803-dfa6-c1a194892b74.png)

いい感じ！


