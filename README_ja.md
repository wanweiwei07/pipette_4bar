# pipette_4bar

[English](README.md) | [中文](README_zh.md) | **日本語**

1 台のモータで駆動する**二重四節リンク機構**の寸法設計です。2 つのリンク（fb1、fb2）は同じモータフランジを共有し、先端は x = -30 mm の鉛直線に沿って交互に動きます。一方が下がるとき、もう一方は上がります。プログラムはすべてのリンク長とモータ回転角を最適化し、結果を [`one`](https://github.com/wanweiwei07/one) ライブラリでアニメーション表示します。

![motion](docs/images/motion.gif)

## 1. 座標系と名称

![schematic](docs/images/schematic.png)

| 記号 | 意味 |
|---|---|
| O1 = (0, 0) | モータフランジ軸、原点。fb1 と fb2 の crank はともにここに固定 |
| O2 = (0, f) | rocker の回転軸、+y 軸上（f = frame 長さ） |
| A | crank と connecting rod の関節 |
| B | connecting rod と rocker の関節 |
| P | 先端（作用点）、rocker に固定（rocker は三角形 O2-B-P） |
| fb1, fb2 | 2 つの四節リンク。fb2 の crank 角 = fb1 の crank 角 + δ（同じフランジのため一定） |

単位は最適化では mm、表示では m です。

## 2. 設計要件

| 要件 | 値 | コード中のパラメータ |
|---|---|---|
| 先端が鉛直線に沿って動く | x = -30 mm | `X_LINE` |
| モータ正転：fb1 先端が -y 方向へ移動 | ≥ 25 mm、単調 | `STROKE1` |
| モータ正転：fb2 先端が上昇（+y） | ≥ 25 mm、単調 | `STROKE2` |
| fb1 先端に対する fb2 先端の始点 | y₂ − y₁ = −25 mm | `START_OFFSET` |
| モータ逆転 | 逆：fb1 が上昇、fb2 が -y | （自動） |
| 2 本の crank がともに +x を向く | 行程中 crank 先端 x ≥ 0 | `W_CRANK_DIR` |
| 関節 A、B の位置 | −15 ≤ x ≤ 20 mm | `JOINT_X_RANGE` |
| 運動中の関節が rocker 回転軸 O2 より上に出ない | y_A, y_B ≤ y_O2（− 余裕） | `JOINT_Y_MARGIN`（= 0） |
| 伝達角 | 40° ≤ μ ≤ 140° | `MU_MIN` |

## 3. 設計変数

リンクごと（×2）：`[f, a, b, c, p, β, σ]`

| 変数 | 意味 | 範囲 |
|---|---|---|
| f | frame O1-O2 | 70 – 100 mm |
| a | crank O1-A | 10 – 30 mm |
| b | connecting rod A-B | 10 – 80 mm |
| c | rocker O2-B | 10 – 50 mm |
| p | O2 から P までの距離 | 10 – 50 mm |
| β | O2B から O2P への角度 | −180° – 180° |
| σ | 組立分岐 | ±1 |

共通変数：モータ開始角 θ₀、crank 角度差 δ、モータ回転方向（CW/CCW）、モータ行程角（45° – 180°）。

## 4. 手法

- **運動学**：各モータ角で、B は円(A, b) と円(O2, c) の分岐 σ 側の交点。P は O2B を β 回転し p に拡縮して求める。組み立てられない構成は除外。
- **評価関数**（fb1 と fb2 の和）：直線度 max|x_P + 30| に、非単調な動き、行程不足、始点差の誤差、伝達角、crank の向き、関節 x 範囲、O2 より上の関節のペナルティを加える。
- **ソルバ**：`scipy.optimize.differential_evolution`（Sobol 初期化、並列処理）。1 回約 10 秒。

## 5. 結果（seed = 0）

| | f | a | b | c | p | β | σ |
|---|---|---|---|---|---|---|---|
| fb1 | 77.09 | 15.67 | 72.95 | 15.15 | 31.30 | −32.4° | +1 |
| fb2 | 77.09 | 16.69 | 73.62 | 17.54 | 31.30 | −156.3° | −1 |

モータ：開始角 29.3°、時計回りに 46.1°；fb2 の crank 角度差 δ = −17.4°。

| 項目 | fb1 | fb2 |
|---|---|---|
| 先端の移動 | 89.59 → 64.59（−25.00 mm） | 64.59 → 89.59（+25.00 mm） |
| 最大 \|x − (−30)\| | 1.30 mm | 1.30 mm |
| 伝達角 | 75.7° – 127.1° | 89.2° – 138.7° |
| 関節 A の x | 13.66 – 15.67 mm | 13.81 – 16.69 mm |
| 関節 B の x | −14.97 – −8.50 mm | 11.91 – 17.54 mm |
| 最も高い関節の y（O2 は 77.09） | 74.77 mm | 77.05 mm |

![poses](docs/images/poses.png)

![performance](docs/images/performance.png)

## 6. 注意点

- **直線度の限界。** P は rocker 上にあるため、軌跡は常に O2 を中心とする円弧になります。長さ s の直線区間を円弧で近似したときの最小偏差は約 s² / (16 p)：15 mm で 0.47 mm、**25 mm で 1.30 mm**、30 mm で約 1.9 mm（推定）。他の変数ではこの誤差を減らせません。より直線に近い軌跡が必要な場合は、P を connecting rod 上に置く必要があります。
- **回転軸の共有。** 2 つの先端が同じ線上の同じ区間を動くため、最適解では f₁ = f₂、p₁ = p₂ となり、2 つの rocker は回転軸 O2 を共有し、先端は同じ円弧を通ります。干渉を避けるため、異なる平面（z 層）に配置する必要があります。表示では fb2 を fb1 の 6 mm 後方に描画しています。
- **現在は余裕を設けていません。** `JOINT_Y_MARGIN` は 0 で、最適化は許された範囲を使い切ります：fb2 の関節 B は O2 の高さまで 0.04 mm、fb1 の関節 B は x = −14.97 に達します。実機の関節部品のための空間が必要な場合は、`JOINT_Y_MARGIN` を設定（および `JOINT_X_RANGE` を狭める）してください。
- **複数解。** 乱数シードが異なると、評価値が同じでリンク長の異なる解が得られます。一意な設計が必要な場合は、副次的な目的（モータ行程角の最小化、伝達角の最大化、小型化など）を追加してください。

## 7. 使い方

必要環境：Python ≥ 3.12、`numpy`、`scipy`、[`one`](https://github.com/wanweiwei07/one)（表示には `pyglet` と `imgui-bundle` も必要）。

```bash
# Windows の例；`one` をインポート可能にする
set PYTHONPATH=E:\src_code\one
py -3.12 four_bar_opt.py
```

結果を表示し（第 5 節参照）、`four_bar_result.npy` に保存して、表示ウィンドウを開きます：

![viewer](docs/images/viewer.png)

| 表示要素 | 意味 |
|---|---|
| 金 / 紫 / 茶 | fb1 の crank / connecting rod / rocker |
| キャロットオレンジ / ピンク / スレートグレー | fb2 の crank / connecting rod / rocker |
| 黒 | frame と関節 |
| マゼンタ | 先端 P とその軌跡 |
| オレンジの線 | 目標線 x = −30 |
| シルバーの線 | 関節 x の範囲 −15 と 20 |
| **Front view** ボタン | 正面図：−z 方向から xy 平面を見る、y が上、機構全体に合わせて拡縮 |
| **Center view** ボタン | 中央表示：現在の視線方向を保ったまま中央に寄せて拡縮 |

モータは行程内を往復します。マウス：左/右ドラッグで回転、中ドラッグで平行移動、ホイールで拡大縮小。

要件を変更するには、[four_bar_opt.py](four_bar_opt.py) 冒頭の定数（`X_LINE`、`STROKE1`、`STROKE2`、`START_OFFSET`、`JOINT_X_RANGE`、`MU_MIN`、`FB_BOUNDS` など）を編集します。再最適化せずに保存済みの結果を再生する場合：

```python
import numpy as np, four_bar_opt as m
x = np.load("four_bar_result.npy")
m.report(x)
m.visualize(x)
```

> Windows では最適化にマルチプロセス（`workers=-1`）を使います。`optimize()` を呼ぶスクリプトは必ず `if __name__ == '__main__':` の中で呼んでください。そうしないと各ワーカーがスクリプト全体を再実行します。
