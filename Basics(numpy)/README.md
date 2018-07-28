# はじめに
毎回，毎回numpyの連結で悩むことがあるので，一回ちゃんと整理しようと思って自分用メモです

# 環境
- python3.6.6
- Windows 10
- anaconda
- numpy 1.14.3

# 悩み
- 連結する方向にいつも悩む

# 参考
https://deepage.net/features/numpy-axis.html

# 連結
こんな時に連結したい
ロボットの状態をx,y,thで分けて計算してて，そっから統合してstateにしたいとか．．．

## 連結で使えるコマンド
- hstack
- vstack
- stack
- concatenate

# プログラムと動作の考察

簡単なプログラムです

```py
import numpy as np

class Connect_array():
    def __init__(self):
        self.a = np.array([[1, 3, 5], [2, 4, 6]])
        self.b = np.array([[7, 9, 11], [8, 10, 12]])

    def Hstack(self):
        print('a = {0}, b = {1}'.format(self.a, self.b))
        connected_array = np.hstack((self.a, self.b))
        print('hstack(a,b) = {0}'.format(connected_array))

    def Vstack(self):
        print('a = {0}, b = {1}'.format(self.a, self.b))
        connected_array = np.vstack((self.a, self.b))
        print('vstack(a,b) = {0}'.format(connected_array))

    def Stack(self):
        print('axis number ?')
        axis_num = int(input())
        print('a = {0}, b = {1}'.format(self.a, self.b))
        connected_array = np.stack((self.a, self.b), axis=axis_num)
        print('stack(a,b) = {0} (axis={1})'.format(connected_array, axis_num))

    def Concatenate(self):
        print('axis number ?')
        axis_num = int(input())
        print('a = {0}, b = {1}'.format(self.a, self.b))
        connected_array = np.concatenate((self.a, self.b), axis=axis_num)
        print('Concatenate(a,b) = {0} (axis={1})'.format(connected_array, axis_num))


def main():
    connect_array = Connect_array()

    connect_array.Hstack()
    
    connect_array.Vstack()

    connect_array.Stack()

    connect_array.Concatenate()


if __name__ == '__main__':
    main()
```


## hstack
実行結果

```py
# a = [[1, 3, 5], [2, 4, 6]]
# b = [[7, 9, 11], [8, 10, 12]]
# hstack(a,b) = [[1 , 3, 5, 7, 9, 11], [2, 4, 6, 8, 10, 12]]
```

つまり，axis=1の方向に連結されてます

## vstack

```py
# a = [[1, 3, 5], [2, 4, 6]]
# b = [[7, 9, 11], [8, 10, 12]]
# vstack(a,b) =  [[1, 3, 5], [2, 4, 6], [7, 9, 11], [8, 10, 12]]
```

axis=0 方向に連結

## stack

stackは特徴的で，次元がふえます！そのまま保持してくっつくイメージですね
ある意味でいちばん，連結っぽいか？

```py
# axis=0
# a = [[1, 3, 5], [2, 4, 6]]
# b = [[7, 9, 11], [8, 10, 12]]
# stack(a,b) =  [[[1, 3, 5], [2, 4, 6]], [[7, 9, 11], [8, 10, 12]]]
```

```py
# axis=1
# a = [[1, 3, 5], [2, 4, 6]]
# b = [[7, 9, 11], [8, 10, 12]]
# stack(a,b) =  [[[1, 3, 5], [7, 9, 11]], [[2, 4, 6], [8, 10, 12]]]
```

## concatenate
これは，hstackとかと同じ動きをします

```py
# axis=0
# a = [[1, 3, 5], [2, 4, 6]]
# b = [[7, 9, 11], [8, 10, 12]]
# concatenate(a,b) =  [[1, 3, 5], [2, 4, 6], [7, 9, 11], [8, 10, 12]]
```

```py
# axis = 1
# a = [[1, 3, 5], [2, 4, 6]]
# b = [[7, 9, 11], [8, 10, 12]]
# concotenate(a,b) = [[1 , 3, 5, 7, 9, 11], [2, 4, 6, 8, 10, 12]]
```

# いやaxis=1とはちょっと気になった
軸とか次元とかについていろいろ調べているとでてくるのがこの図
![image.png](https://qiita-image-store.s3.amazonaws.com/0/261584/f288a42e-cdea-bbe1-e94c-81382c78fd84.png)


確かにこれはあっていると思います
**どっちに連結させるかという行列目線のお話ですね**

つまり，例えば，hstackとすれば，axis=1，列方向に連結することができますよ
逆に，vstackとすれば，axis=0，行方向に連結できますよ
ということを表してます

しかし，なにか気持ち悪い

軸に沿っているのだけみると，どう考えても，1次元目の要素に沿っているので，なんか間違っているような気がしてしまうんです
まぁこんな風に横一列に並べて書いたら，
![image.png](https://qiita-image-store.s3.amazonaws.com/0/261584/e2a919e7-b4f3-6780-5909-49d9200d67cc.png)

ちょっとよくわからなくなってきましたね笑横並びの図は忘れてください

でもよくよく考えて上図を眺めてたら

確かに成分は1次元目のものを追っているような気がするんですが
軸としては2次元目（axis=1）を示しています
つまり，新たなもの増やしたいなぁっておもったときに，すべての要素にまたがって増やす必要があります
よって，axis=1なんです

逆にaxis=0のときは，何も増やさなくていいですね，というかすべての要素にまたがる必要はありません
よってaxis=0なのか

と理解しました

ただ，違う見方をすれば，すべての要素にかかわってるのがaxis=0という見方もできるかとおもいます
足し算するときみたいなね　下の図みたいに

![image.png](https://qiita-image-store.s3.amazonaws.com/0/261584/bfeabe31-442b-1cef-0311-415be0b06e36.png)



なので，これはあくまで，僕の考え方です

- 新しいものを増やすときにすべての要素にまたがなくていいもの
- 足し算するときに各々の要素を見る

三次元でもみても同じ結果になりそうです
![image.png](https://qiita-image-store.s3.amazonaws.com/0/261584/22292f80-d780-459f-45f6-8c51855c0832.png)


足し算を考えてみると
![image.png](https://qiita-image-store.s3.amazonaws.com/0/261584/94d6b28c-6275-0a85-0c7e-0db849d1e248.png)

いけてそうですね
一番おおくまたいでるやつがaxis=0になってます

要素で考えてもいけそうです！

ちなみにこれらの考え方は

```
sum
```

とかそういう系の関数にも使えますので！！

