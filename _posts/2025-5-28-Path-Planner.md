---
title: "差動駆動およびホロノミックロボット向け経路計画インターフェース"
date: 2025-05-28
categories: projects
mathjax: true
---

このプロジェクトでは、差動駆動およびホロノミック [Vex](https://www.vexrobotics.com/) ロボットの滑らかな経路生成における [Cubic Hermite Spline](https://en.wikipedia.org/wiki/Cubic_Hermite_spline) の応用について詳述します。経路生成は [Desmos](https://www.desmos.com/) の使用により簡略化されています。また、本プロジェクトでは、初速度と最終速度、初期加速度と最終加速度、移動距離を与えたときの速度プロファイルを生成する簡単な方法も提案しています。これは高校最終学年の間に取り組んだプロジェクトであり、多くの改善の余地が残されていますが、その一部を本記事で紹介します。

研究期間：2022年8月ー2023年6月

# リンク

[GitHubリポジトリ](https://github.com/Meilan39/Vex-Library-Public)  
[プレゼンテーション](https://drive.google.com/file/d/1s2wVjT6lOR31UDFTtDkMZKHKzBM_V8rS/view)

2023年VEX世界大会に出場したロボットがこのプログラムで計画した経路を自律制御で実行する様子。

[実際に使用されている動画](https://drive.google.com/file/d/109tNWCAwsQDnNOmf9EIANeVorjwlhJYq/view?usp=sharing)

開始点・終了点および速度を可視化できる通常のPath Plannerバージョン。

[Path Planner](https://www.desmos.com/calculator/zqpztqvvoi)

開始点・中間点・終了点および速度を可視化できるPath Planner Plusバージョン。Path Planner Plusは、長いまたは複雑な経路で追加の中間点が必要な場合に最適です。

[Path Planner Plus](https://www.desmos.com/calculator/dg7kybaxyb)

# 使用ツール

- C++
- Desmos

# 動機

VEX Robotics Competitionでは、「自律制御時間」と呼ばれる時間帯があり、ロボットは事前にプログラムされたルーチンを実行して得点します。多くのチームは、エンコーダ距離や時間ベースのアプローチを用いて、個別の直線経路を連続で実行しています。しかし、この方法では、目的地と現在地の間に障害物がある場合、複数の経路を順に実行する必要があり、非効率で大きな誤差が生じる可能性があります。このプロジェクトの動機は、2点間の連続的かつ柔軟な経路を簡単に生成できるインターフェースを作成することでした。

# 背景

### 三次エルミート補間

基本的なアイデアは、開始点と終了点、およびそれぞれに対応する「速度」ペアを使用して柔軟な経路を生成することです。開始点と終了点は経路の端点を定義し、「速度」はそれらの向きが経路の曲率に与える影響を示します。「速度」はロボットの実際の速度を意味するのではなく、端点の向きが経路形状に与える「重み」として扱われます。この問題は補間問題であり、ここではエルミートスプライン補間を用いています。エルミートスプラインは、数学的にはよく知られるベジエ曲線と同等とみなすことができますが、開始点と終了点の向きを明示的に定義できる点で、ロボット経路計画においてより適しています。一方、ベジエ曲線では、中間点が物理的意味を持たずに曲率を定義します。

上述のように、エルミートスプラインは以下の4つの情報から生成されます：

$P_0$ ::= 開始点の x-y 座標  
$P_1$ ::= 終了点の x-y 座標  
$P_2$ ::= 開始点の「速度」  
$P_3$ ::= 終了点の「速度」

次に、パラメータ $t \in \mathbf{R}$ に関する一般的な三次多項式とその導関数を考えます。係数 $a, b, c, d \in \mathbf{R^2}$ は2次元ベクトルです。

$$ \begin{align*} 
P(t) &= at^3+bt^2+ct+d \\
P'(t) &= 3at^2+2bt+c
\end{align*} $$

関数 $P(t)$ は2次元ベクトル、すなわちx-y座標を出力し、$P(0)$ が開始点、$P(1)$ が終了点になるように定義します。特に、$P(t)$ および $P'(t)$ に 0 および 1 を代入すると次のようになります。

$$ \begin{align*}
P(0) = P_0 &= d\\
P(1) = P_1 &= a + b + c + d\\
P'(0) = P_2 &= c\\
P'(1) = P_3 &= 3a + 2b + c
\end{align*} $$

これを行列形式で書き直すと：

$$ 
\begin{bmatrix} P_0 \\ P_1 \\ P_2 \\ P_3 \end{bmatrix}
=
\begin{bmatrix}
0 & 0 & 0 & 1 \\
1 & 1 & 1 & 1 \\
0 & 0 & 1 & 0 \\
3 & 2 & 1 & 0
\end{bmatrix}
\begin{bmatrix} a \\ b \\ c \\ d \end{bmatrix}
$$

この逆行列を使って：

$$ 
\begin{bmatrix}
0 & 0 & 0 & 1 \\
1 & 1 & 1 & 1 \\
0 & 0 & 1 & 0 \\
3 & 2 & 1 & 0
\end{bmatrix}^{-1}
\begin{bmatrix} P_0 \\ P_1 \\ P_2 \\ P_3 \end{bmatrix}
=
\begin{bmatrix} a \\ b \\ c \\ d \end{bmatrix}
$$

ガウス消去法で逆行列を求めると：

$$ 
\begin{bmatrix} a \\ b \\ c \\ d \end{bmatrix}
=
\begin{bmatrix}
2 & -2 & 1 & 1 \\
-3 & 3 & -2 & -1 \\
0 & 0 & 1 & 0 \\
1 & 0 & 0 & 0
\end{bmatrix}
\begin{bmatrix} P_0 \\ P_1 \\ P_2 \\ P_3 \end{bmatrix}
$$

したがって、係数に基づいたパラメトリック方程式は次のようになります：

$$ \begin{align*}
P(t) &=
\begin{bmatrix} t^3 & t^2 & t & 1 \end{bmatrix}
\begin{bmatrix}
2 & -2 & 1 & 1 \\
-3 & 3 & -2 & -1 \\
0 & 0 & 1 & 0 \\
1 & 0 & 0 & 0
\end{bmatrix}
\begin{bmatrix} P_0 \\ P_1 \\ P_2 \\ P_3 \end{bmatrix} \\
&= 
\begin{bmatrix} 
2t^3-3t^2+1 & -2t^3+3t^2 & t^3-2t^2+t & t^3-t^2 
\end{bmatrix}
\begin{bmatrix} P_0 \\ P_1 \\ P_2 \\ P_3 \end{bmatrix} \\
\end{align*} $$

ゆえに、

$$ P(t) = P_0 h_1 + P_1 h_2 + P_2 h_3 + P_3 h_4 $$

ここで、

$h_{1}(t) = 2t^3-3t^2+1$  
$h_{2}(t) = -2t^3 + 3t^2$  
$h_{3}(t) = t^3-2t^2+t$  
$h_{4}(t) = t^3-t^2$  

$h_{1}(t)$ から $h_{4}(t)$ は「エルミート基底関数」と呼ばれます。

### 速度プロファイル

実際には、補間関数は各試合前のロボット初期化時間中に離散的にサンプリングされます。サンプリング時には、経路の1次・2次導関数の離散的な近似も計算可能です。この情報は、特に高速移動するロボットが急カーブで転倒しないように速度をスケーリングする際に有用です。

ここでの目標は、直感的なパラメータを柔軟な速度プロファイルに変換できる関数を定義することです。関数は以下の6つのパラメータで定義されます：

$d$ ::= プロファイルの全体距離  
$m$ ::= 最大速度  
$k_1$ ::= 加速定数  
$k_2$ ::= 減速定数  
$s_1$ ::= 初期速度  
$s_2$ ::= 最終速度  

これらの6つのパラメータで、比較的自由度の高い速度プロファイルを定義することができます。

まず、シグモイド関数の一般形を定義します。ただし、$a_i, b_i$ は定数です。

$$ f(x) = \frac{1}{1 - a_ie^{-b_ix}} \left( a_i > 0, b_i > 0 \right)$$

曲線の「急峻さ」は、指数係数 $b_i$ によって決まるため、

$$ b_i = k_1 $$

次に、$f(0) = s_1$ となるように、$a_i$ を以下のように導出します。

$$ s_1 = f(0) = \frac{1}{1 + a_i}
\implies a_i = \frac{1}{s_1} - 1 $$

同様に、プロファイルの減速側は、反転シグモイド関数で表され、$a_f, b_f$ を定数とします。

$$ g(x) = \frac{1}{1 - a_fe^{b_fx}} \left( a_f > 0, b_f > 0 \right)$$

このグラフの $g(0)$ が $x=d$ に一致するよう、右に $d$ だけシフトします。

$$ g(x) = \frac{1}{1 - a_fe^{b_f(x-d)}} \left( a_f > 0, b_f > 0 \right)$$

最後に、2つの関数を掛け合わせ、最大速度 $m$ を掛けることで、最終的なプロファイルが得られます。

![Velocity Profile](/assets/2025-5-28-Path-Planner/profile.svg "プレゼン資料の一部")

$$ mf(x)g(x) = 
\frac{m}{
  \{ 1 + ( \frac{m}{s_1} - 1 ) e^{-k_1x} \} 
  \{ 1 + ( \frac{m}{s_2} - 1 ) e^{k_2(x - d)} \} 
}
$$

### ホロノミック制御の詳細

ホロノミックドライブとは、理論上すべての方向に移動・同時に回転可能なロボットの駆動機構です。VEXロボティクスでは、シンプルさから「X-drive（クロスドライブ）」が主流です。ただし、高校レベルでは制御が難しいため、その性能は十分には活用されていません。

例として、左スティック入力 $(l_x, l_y)$、右スティック入力 $(r_x, r_y)$ を持つコントローラーを用いた場合：

$$
\mathbf{v} = 
\begin{bmatrix} l_x \\ l_y \end{bmatrix}
\ \ \ \ 
\omega = r_x
$$

ここで $\mathbf{v}$ は**フィールド基準のロボット速度**を、$\omega$ は角速度を表します。「フィールド基準」とは、ロボットの向きに関係なく、スティックを一定方向に倒すと常にフィールド上でその方向に進むことを意味します。

このような制御方式は、**フィールドセンター方式（field-centric）**と呼ばれます。ロボットの向きを測定するためにはジャイロスコープを使用し、ここではその角度を $\theta$ とします。入力ベクトル $\mathbf{v}$ を $-\theta$ 回転（反時計回り）することで、フィールド基準の入力をロボット基準に変換します：

$$
\mathbf{v_{robot}} =
\begin{bmatrix}
  \cos(-\theta) & - \sin(-\theta) \\
  \sin(-\theta) & \cos(-\theta)
\end{bmatrix}
\begin{bmatrix} l_x \\ r_x \end{bmatrix}
$$

たとえばロボットが90度右を向いているとき、左スティックを前に倒すと、フィールド上では右に動くことになります。これを打ち消すには、入力を90度反時計回りに回転させます。こうすることで、入力方向とフィールド上の進行方向が一致します。

この処理は、「**ワールド座標系の入力をロボット座標系に変換する**」ということでもあります。

![X-drive](/assets/2025-5-28-Path-Planner/xdrive.svg "プレゼン資料の一部")

次に、$\mathbf{v}$ を $\mathbf{v_{robot}}$ に置き換え、各ホイールの動作を決定します。X-driveは全方向駆動が可能なため、各ホイールの進行方向に対応する単位ベクトルとの内積をとって回転量を算出します。具体的には次のように定義します：

$$
\mathbf{v_{fl}} = \begin{bmatrix} \cos(45) \\ \sin(45) \end{bmatrix}
\mathbf{v_{fr}} = \begin{bmatrix} \cos(135) \\ \sin(135) \end{bmatrix}
\mathbf{v_{rl}} = \begin{bmatrix} \cos(135) \\ \sin(135) \end{bmatrix}
\mathbf{v_{rr}} = \begin{bmatrix} \cos(45) \\ \sin(45) \end{bmatrix}
$$

内積は、ベクトル同士の方向の一致度を測るもので、各ホイールの相対速度を決定するのに使えます。角速度 $\omega$ も考慮すると、各ホイールの速度は次のように定まります：

$$ \begin{align*}
v_{fl} &= l_x * \cos(45)\ \ + l_y * \sin(45)\ \ + \omega \\  
v_{fr} &= l_x * \cos(135) + l_y * \sin(135) - \omega \\    
v_{rl} &= l_x * \cos(135) + l_y * \sin(135) + \omega \\    
v_{rr} &= l_x * \cos(45)\ \  + l_y * \sin(45)\ \  - \omega
\end{align*} $$

その後、これらの値を最大値で割って正規化することで、モーターの出力が1を超えないようにします。これは、内積に基づく相対的な関係を維持する上で非常に重要です。

この実装では、ジャイロスコープとともにオドメトリエンコーダーを用いて、ロボットの自己位置推定を行います。エンコーダーの速度を離散的に積分し、毎回の位置変化を算出します。これらの変位ベクトルはロボット座標系で表されているため、ジャイロスコープ角を用いて回転させることで、フィールド基準の位置を求めます。移動距離も、変位ベクトルの大きさを積分することで計算可能であり、これは経路追従に使用されます。

### Desmos UI

[Path Planner](https://www.desmos.com/calculator/zqpztqvvoi)  
[Path Planner Plus](https://www.desmos.com/calculator/dg7kybaxyb)

上記の2つのDesmosグラフは、軌道生成のためのユーザーインターフェースとして機能します。ロボットの初期および最終座標や「速度」を表す点は、画面上でドラッグ可能です。エルミートスプラインの特性に従い、初期および最終の「速度」を長くすればするほど、その角度が経路に与える影響（制御力）が大きくなります。

Path Planner Plusのグラフは、拡張された軌道生成方式のUIとして機能し、2つのエルミートスプラインによる分割補間です。経路に中間点と「速度」を1つ追加することで、中央付近の経路の挙動をより詳細に制御できます。

# 実装

以下のコードは、上記理論に基づいた直接的かつ素朴な実装例です。

```c++
  /// @brief エルミート補間式にある処理位置（x）を問い、そ地点の姿勢を返す
  /// @param path エルミート補間式の定義
  /// @param previous 前回の処理位置の姿勢
  /// @param x 処理位置（0から１）
  /// @return 処理位置（x）のロボット姿勢
  Pose CubicHermiteInterpolation(Path path, Pose previous, float x) {
    // エルミート補間多項式の表現
    float h1 = 2*(x*x*x) - 3*(x*x) + 1;
    float h2 = -2*(x*x*x) + 3*(x*x);
    float h3 = (x*x*x) - 2*(x*x) + x;
    float h4 = (x*x*x) - (x*x);
    // エルミート補間定義に従い処理位置（x）の姿勢を導く
    Pose current;
    current.x = path.p0.x * h1 + path.p1.x * h2 + path.t0.x * h3 + path.t1.x * h4;
    current.y = path.p0.y * h1 + path.p1.y * h2 + path.t0.y * h3 + path.t1.y * h4;
    // 姿勢の角度（進行方向）を導出
    current.w = Vector {current.x - previous.x, current.y - previous.y}.getAngle(); 
    return current;
  }
```

$t$ の値だけがループ内で変化するため、入力行列とパラメータをあらかじめまとめておくことで、各係数の再計算を回避できます：

$$
P(t) =
\begin{bmatrix} t^3 & t^2 & t & 1 \end{bmatrix}
\begin{bmatrix}
2P_0 & -2P_1 & 1P_2 & 1P_3 \\
-3P_0 & 3P_1 & -2P_2 & -1P_3 \\
0 & 0 & 1P_1 & 0 \\
1P_0 & 0 & 0 & 0
\end{bmatrix}
$$

ホロノミックドライブでは、「速度」方向とは別に姿勢角を独立して定義することができます。経路の開始を0、終了を1として、途中で向くべき角度のリスト（HolonomicPose）を渡すことで、ロボットの姿勢制御が可能になります。たとえば {0.5, 180} は、経路の中間点（0.5の位置）で180度を向くべきことを意味します。

これらの距離と角度のペアを線形補間し、経路離散化時に滑らかに角度が変化するようにします。以下がそのコードです：

```c++
  /// @param dist 特定する処理位置 (0 から 1)
  /// @param angle　ロボットの角度
  struct HolonomicPose {
    float dist;
    float angle;
  };

  /// @brief ホロノミック姿勢の std::vector をある処理位置　x で補間。
  /// 目的姿勢を提示された経路位置に厳密に達成する為に滑らか且つ徐々に近づいていく必要がある。
  /// この関数は提示された処理位置を用いて全ての処理位置のあるべき姿勢を導く役割を果たす。
  /// @param orientation ホロノミック姿勢の　std::vector （処理位置０と１の姿勢は必ず定義されている）
  /// @param x 処理位置
  /// @return 補間値
  float InterpolateHolonomicPose(std::vector<HolonomicPose> orientation, float x ) {
    if ( ! orientation.empty() ) { //　ホロノミック姿勢が示されているか
      int s = 0; //　イテレータ初期化
      //　std::vector から現在の処理位置（x）が入る区間を探る
      //　区間の先頭と後尾の角度と処理位置の差を取ることで直線補間を行うことができる
      while (orientation.at(s).dist < x) { s++; if (s == orientation.size() - 1) break; } //　std::vector を探索
      float angleError = wrap(orientation.at(s-1).angle, orientation.at(s).angle);        //　区間の最短角度差を求める
      float distError = orientation.at(s).dist - orientation.at(s-1).dist;                //　区間の処理位置の差を求める
      x -= orientation.at(s-1).dist;                                                      //  
      return bound( (x / distError) * angleError + orientation.at(s-1).angle );           //  直線補間を行う
    } else {
      return -1; //　ホロノミック姿勢が示されてない場合（ー１）を返す
    }
  }
```

次に、経路を離散化する処理を行います。「明瞭度（clarity）」は、デフォルトで100点に設定されており、経路、速度プロファイル、および姿勢角度をそれぞれサンプリングしてWaypointとして格納します。

```c++
    struct Waypoint {
      PathType type;
      float dist;
      Pose heading;
    };

    /// @brief 軌道を生成する関数
    /// @param path エルミート補間式の定義
    /// @param orientation ホロノミック姿勢の　std::vector 
    /// @param clarity 明瞭度を示す（一つの経路は100と定められている）
    /// @param profile 速度プロフィール
    std::vector<Waypoint> generate(Path path, std::vector<HolonomicPose> orientation, int clarity, StaticProfile profile) {
        float segment = 1.0 / clarity; //　処理位置の一つ一つの区間の長さを導く
        float dist = 0; //　経路の長さを初期化
        // 現在姿勢と前回姿勢を宣言
        Pose previous {path.p0.x, path.p0.y, path.t0.getAngle()}; //　点Aの姿勢に設定　
        Pose current {0, 0, 0}; //　初期化
        //　軌道となる経由地の配列を作成
        std::vector<Waypoint> waypoints;
        //　明瞭度の分繰り返される（イテレータは1から始める）
        for (int i = 1; i <= clarity; i++) {
            // 現在処理位置を求める
            float x = segment * i;
            // 処理位置を元に現在の姿勢を求める
            current = CubicHermiteInterpolation(path, previous, x);
            // 現在と前回の姿勢の差を（previous）に導入
            previous = previous.getError(current);
            // 現在角度と前回角度の差を比例拡大して逆数を取ります（この値は経路の曲率が高いほど小さくなる）
            // 速度プロフィールの現在処理値値を計算（区分的補間の場合、二番目の補間の際　index　が50となっている）
            // 上記の値はどちらとも0から1の範囲で、掛け合わせることで現在処理位置での速度を導ける。
            float speed = (1 / (autonomous_rotation_scaler * fabs(previous.w) + 1)) * (profile.get(i + index));
            // 経由地に代入していく
            Waypoint waypoint;
            waypoint.dist = length + dist + previous.getVector().getMagnitude(); //　各経由地間の距離の合計
            // ロボットを最終的に動かす関数がコントローラの入力を予想している為、アナログスティックの出力の近似
            // アナログスティックの出力の模倣は、進行方向と同じ角度の単位ベクトルで、その方向に全速力で進むことを意味する
            // 速度にかけることで適切な速度規制を可能とする
            waypoint.heading.x = cosf(previous.getVector().getAngle() / RadToDeg) * speed; 
            waypoint.heading.y = sinf(previous.getVector().getAngle() / RadToDeg) * speed;
            // この処理位置で以前定義した「ホロノミック姿勢補間関数」を呼び出しあるべき角度を保存                
            waypoint.heading.w = InterpolateHolonomicPose(orientation, aIndex + x);
            waypoints.push_back(waypoint);// 経由地を軌道に加える
            // 次のループに備える
            dist = dist + previous.getVector().getMagnitude(); // 今回の経由地間を合計距離にたす
            previous = current; // 今回の姿勢を前回の姿勢に代入
        }
        // 初期姿勢と最終姿勢を定義。ホロノミック姿勢が示されていたら従って代入
        this -> initialPose = Pose {path.p0.x, path.p0.y, orientation.empty() ? 0 : orientation.front().angle};
        this ->   finalPose = Pose {path.p1.x, path.p1.y, orientation.empty() ? 0 : orientation.back().angle };
        this ->      length = dist;    // 経路の最終的長さは経由地間の距離の合計である
        this ->      aIndex = 1;       // 区分的補間を行う場合ホロノミック姿勢をつける為
        this ->       index = clarity; // 区分的補間を行う場合速度プロフィールを継げる為
        return waypoints;              // 軌道を呼び出し主に返す
    }
```

ホロノミック車台のフィールドセンター方式コードも次に示します。

```c++
  void arcadeDrive( Vector translation, float w ) {
      // 運転士視点操作の場合得られた横断ベクトルをロボットの角度の分、逆回転
      if (fieldCentric) translation.rotate( -pose.w );
      float fr = ( translation.y * FR_component.y) + ( translation.x * FR_component.x) - w;
      float fl = ( translation.y * FL_component.y) + ( translation.x * FL_component.x) + w;
      float rl = ( translation.y * RL_component.y) + ( translation.x * RL_component.x) + w;
      float rr = ( translation.y * RR_component.y) + ( translation.x * RR_component.x) - w;
      float max = fmax( fmax( fabs(fr), fabs(fl) ), fmax( fabs(rl), fabs(rr) ) );
      if (max > 1) { // 正規化
          fr /= max;
          fl /= max;
          rl /= max;
          rr /= max; 
      }
      fr *= WHEEL_MAX_RPM;  // 適当な速度を導く
      fl *= WHEEL_MAX_RPM;  // 適当な速度を導く
      rl *= WHEEL_MAX_RPM;  // 適当な速度を導く
      rr *= WHEEL_MAX_RPM;  // 適当な速度を導く
      FR.spin(forward, fr, vex::velocityUnits::rpm);  // モータに速度命令
      FL.spin(forward, fl, vex::velocityUnits::rpm);  // モータに速度命令
      RL.spin(forward, rl, vex::velocityUnits::rpm);  // モータに速度命令
      RR.spin(forward, rr, vex::velocityUnits::rpm);  // モータに速度命令
}
```

最後に、経路を実行するプログラムを次に示します。自己位置推定によって「distanceTraveled」という変数を更新し、軌道の進捗状況を把握し、次の「Waypoint」を受け取ります。この「Waypoint」を用いて適切なコントローラ入力を数値的に再現します。

```c++
    /// @brief 経路を実行
    /// @param trajectory 走る経路
    /// @return 実行の捗り (0から1)
    float follow(HolonomicTrajectory trajectory) {
        localize(); // 自己位置推定手法を更新
        float progress = fitToRange( distanceTraveled / trajectory.length, 0, 1 ); // 実行捗りを求める
        if ( progress < 1 ) { // 実行が終了わってない限り
            Waypoint waypoint = trajectory.get(distanceTraveled); // 走った距離を用い経路から次の経由地を特定
            //　ホロノミック姿勢の場合、PID制御を用いて目的角度を到達するために適切な出力を導く。
            //　概念的には、現在角度と目的角度の最短差を導き、その差が０に近づけるよに出力量を決める
            float w = trajectory.orientation ? omegaPID.get( wrap(pose.w, waypoint.heading.w) , 0) : 0;
            arcadeDrive( Vector {waypoint.heading.x, waypoint.heading.y}, w ); // コントローラ操作の関数に入力
            return progress; //　実行捗りを毎回返す
        }      
        stop();   // モータを全て停止
        return 1; // 経路が無事実行されたことを再び示す
    }
```