# はじめに
データをまとめる，相関係数まとめたり，散布図みたり，重回帰分析する機会があったので自分的メモ

github：https://github.com/Shunichi09/Qiita/tree/master/Static

# 相関係数
参考になったサイト
- https://sci-pursuit.com/math/statistics/correlation-coefficient.html#1

使ったプログラム
numpyのcorrcoefがおすすめです
速いし，列ごとでも行ごとでも設定できるので！
普通データは列で管理されると思うので．．．

```py
np.corroef(data, rowvar=False)
```

これで計算してくれます，行ごと（そんなことあんまりないかもですが）にしたい場合は，rowbar=Trueにしてください

# 散布図
いやこれはもうSeabornが圧倒的におすすめです
なんか色かっこいいし，ちょっとイケてる風になります
カテゴリー別でも分けてくれるし，回帰直線引いてくれるしいいことしかありません

参考になったサイト
- https://note.nkmk.me/python-seaborn-pandas-pairplot/

```py
sns.pairplot(data, hue='category')# カテゴリーで色分けしてくれる
```

# 回帰分析
これに関しては意見が分かれるとは思いますが
詳細に見たい場合は，statsmodelが良いです
結果を細かく出力してくれます

しかし，連続で回帰分析してその値を抽出したいときは
sklearnの方が楽かもしれません

参考になったサイト
- Sklearn：https://pythondatascience.plavox.info/scikit-learn/%E7%B7%9A%E5%BD%A2%E5%9B%9E%E5%B8%B0
- Sklearn：https://qiita.com/ynakayama/items/88b4dd5d0dd9b1604006
- Stats：https://qiita.com/yubais/items/24f49df99b487fdc374d
- Stats：http://wcs.hatenablog.com/entry/2016/11/08/231703

```py
# sklearnの場合
# 説明変数の列抜き出し
X = np_data[:, 'input your column']

# 目的変数の列抜き出し
Y = np_data[:, 'input your column']

# 結果出してくれる（切片とか正規化とかその他もろもろ設定可能）
clf = linear_model.LinearRegression(fit_intercept=True, normalize=False, copy_X=True)
 
# 予測モデルを作成
clf.fit(X, Y) # これでfit（学習）させる 
clf.coef_(X, Y) # 偏回帰係数を出してくれる  
clf.score(X, Y) # 決定係数を出してくれる
clf.predict(X) # 予測してくれる 説明変数が入力
```

こうすれば一瞬です
結果も保存できるので，appendして平均とるとかもすぐできますね

```py
# statsmodelの場合
# 説明変数の列抜き出し
X = np_data[:, 'input your column']

# 目的変数の列抜き出し
Y = np_data[:, 'input your column']

X_y = sm.add_constant(X) # 切片を追加
 
model = smf.OLS(Y, X)# 逆なので注意！

result = model.fit()# これで結果が見える

print(result.summary())
```

結果は詳細にでます
t値とかp値とかその他もろもろ

なので，時と場合に応じて使い分けていこうかと


# 最後に
まとめたやつです
参考までに

```py
import numpy as np
import pandas as pd
import statsmodels.formula.api as smf
import statsmodels.api as sm 
from sklearn import linear_model
import matplotlib.pyplot as plt
import seaborn as sns
import string

# 統計のテンプレート

# CSVの読み込み
file_name = 'PATH + file_name.csv'
data = pd.read_csv(file_name, header=None, engine='python')

# NUMPYに変換する場合
np_data = data.values

# 注意
# データはすべて列で管理されるべし

# 対散布図（多変数の場合）引数はpandasです
sns.pairplot(data, hue='category')# カテゴリーで色分けしてくれる

# 相関係数（多変数の場合）引数はnumpy これで列ごとの相関を見れる（Trueで行ごと）
np.corrcoef(np_data, rowvar=False)

# 回帰
# sklearnの場合
# 説明変数の列抜き出し
X = np_data[:, 'input your column']

# 目的変数の列抜き出し
Y = np_data[:, 'input your column']

# 結果出してくれる（切片とか正規化とかその他もろもろ設定可能）
clf = linear_model.LinearRegression(fit_intercept=True, normalize=False, copy_X=True)
 
# 予測モデルを作成
clf.fit(X, Y) # これでfit（学習）させる 
clf.coef_(X, Y) # 偏回帰係数を出してくれる  
clf.score(X, Y) # 決定係数を出してくれる
clf.predict(X) # 予測してくれる 説明変数が入力

# statsmodelの場合
# 説明変数の列抜き出し
X = np_data[:, 'input your column']

# 目的変数の列抜き出し
Y = np_data[:, 'input your column']

X_y = sm.add_constant(X) # 切片を追加
 
model = smf.OLS(Y, X)# 逆なので注意！

result = model.fit()# これで結果が見える

print(result.summary())

```
