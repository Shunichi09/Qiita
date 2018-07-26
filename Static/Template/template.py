import numpy as np
import pandas as pd
import sys
import math
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
X = np_data[:, 'input your rows']

# 目的変数の列抜き出し
Y = np_data[:, 'input your rows']

# 結果出してくれる（切片とか正規化とかその他もろもろ設定可能）
clf = linear_model.LinearRegression(fit_intercept=True, normalize=False, copy_X=True)
 
# 予測モデルを作成
clf.fit(X, Y)
clf.coef_(X, Y)
clf.score(X, Y)# 
clf.predict(X)#予測してくれる 説明変数が入力

# statsmodelの場合

