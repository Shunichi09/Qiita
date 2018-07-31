# はじめに
pythonで3Dplotをする機会があったので

github:https://github.com/Shunichi09

# 環境
- python 3.6.5
- matplotlib 2.2.2
- numpy 1.14.3

# 基本的な考え方
まずxとyだけ考えてみましょう
![image.png](https://qiita-image-store.s3.amazonaws.com/0/261584/b2a792d0-3779-873b-12b0-3a92c44a129d.png)
こんな平面を考えてみます．

ではそこにz方向に値を伸ばしたいときにどうすればよいでしょうか？

格子状にすればよいような気がしますね
値ののるとこを格子にすればよいので，これで行けそうです
つまりこういうことです

![image.png](https://qiita-image-store.s3.amazonaws.com/0/261584/6cb9ae5d-debc-2be8-fa5d-b9aa3489f352.png)

この格子上に値がのると．．．

![image.png](https://qiita-image-store.s3.amazonaws.com/0/261584/eb13dac6-ea65-4ee3-bfc5-77c87e542af8.png)

なんか行けそうです
つまり，作業としては

1. xとyを作成（1次元）
2. 1で作ったものをmeshにする（平面の軸が完成するイメージ）
3. Zを2次元で作成（これでさっき作った平面に値がのります）
4. plot

という流れなわけです

では参考のプログラムです

# plot_surface
ここでは，surfaceplotという関数を使用します

注意点は

```py
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
```

ともにimportしてください！
じゃないと使えません

```py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 3Dgraphを作成
class Ploter_3D():
    def __init__(self, x, y, z):
        self.x = x # 1次元可能
        self.y = y # 1次元可能
        self. z = z # 2次元配列でほしい
        # グラフ作成
        self.fig = plt.figure()
        self.axis = self.fig.add_subplot(111, projection='3d')
    
    def plot_3d(self):
        self.axis.set_xlabel('xlabel')
        self.axis.set_ylabel('ylabel')
        self.axis.set_zlabel('zlabel')

        X, Y = np.meshgrid(self.x, self.y)
        Z = self.z

        self.axis.plot_surface(X, Y, Z)

        plt.show()

def main():
    # 軸の作成
    x = np.array([10, 20, 30, 40]) # 1次元のデータ
    y = np.array([10, 20, 30, 40]) # 2次元のデータ
    # 格子に乗る値
    z = np.array([[1, 2, 3, 4], [5, 6, 7, 8,], [9, 10, 11, 12], [13, 14, 15, 16]])
    ploter = Ploter_3D(x, y, z)
    ploter.plot_3d()
    
if __name__ == '__main__':
    main()
```

で実行すると
なんかかけた！！

![aa.png](https://qiita-image-store.s3.amazonaws.com/0/261584/4d4c0352-d951-8b0b-fdd9-757da574a43c.png)

しかしどうもいろがださい

https://qiita.com/cesolutionsym/items/dbceb692f6d5d6af9a9e

こんな記事があったので参考にしながら
色を変えてみます

![fig.png](https://qiita-image-store.s3.amazonaws.com/0/261584/610d9a38-944c-d7b9-9ce0-fc8965fc86a4.png)

いやかわったけど笑

どうせなら乱数とかでやってみる

```py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

# 3Dgraphを作成
class Ploter_3D():
    def __init__(self, x, y, z):
        self.x = x # 1次元可能
        self.y = y # 1次元可能
        self. z = z # 2次元配列でほしい
        # グラフ作成
        self.fig = plt.figure()
        self.axis = self.fig.add_subplot(111, projection='3d')
    
    def plot_3d(self):
        self.axis.set_xlabel('xlabel')
        self.axis.set_ylabel('ylabel')
        self.axis.set_zlabel('zlabel')

        X, Y = np.meshgrid(self.x, self.y)
        Z = self.z

        self.axis.plot_surface(X, Y, Z, cmap=cm.GnBu)

        plt.show()

def main():
    # 軸の作成
    x = np.array([i for i in range(50)]) # 1次元のデータ
    y = np.array([i for i in range(50)]) # 2次元のデータ
    z = np.random.rand(50, 50)
    ploter = Ploter_3D(x, y, z)
    ploter.plot_3d()
    
if __name__ == '__main__':
    main()
```

![kakko.png](https://qiita-image-store.s3.amazonaws.com/0/261584/980b954b-2576-9f4a-db6b-6935a7d9feaa.png)

なんかアイスみたいになりました笑
