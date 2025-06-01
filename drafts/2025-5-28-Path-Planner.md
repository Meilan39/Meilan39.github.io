---
title: "Path Planning Interface for Differential and Holonomic VEX Robots"
date: 2025-05-28
categories: projects
---

# Description

This project details the application of [cubic Hermite spline interpolation](https://en.wikipedia.org/wiki/Cubic_Hermite_spline) in smooth path generation for differential and holonomic [Vex](https://www.vexrobotics.com/) robots. Path generation is simplified through the use of a [Desmos](https://www.desmos.com/) The project also proposes a simple method to generate a velocity profile given a initial and final velocity, initial and final acceleration, and distance. This is a project undertaken during my senior year of high school, and thus contains many potential improvements, some of which I will attempt to highlight in this article

# Links

[Github Repository](https://github.com/Meilan39/Vex-Library-Public)  
[Presentation](https://drive.google.com/file/d/1s2wVjT6lOR31UDFTtDkMZKHKzBM_V8rS/view)

The normal Path Planner version, where a starting and end point and velocity are can be visualized.

[Path Planner](https://www.desmos.com/calculator/zqpztqvvoi)

The Path Planner Plus version, where a starting, mid, and end point and velocity can be visualized. Path Planner Plus is ideal for long or complex paths that require an extra point for optimized object avoidance.

[Path Planner Plus](https://www.desmos.com/calculator/dg7kybaxyb)

# Tools

- C++
- Desmos

# Motivation

The autonomous control persiod is a period within the Vex Robotics Competition, where robots run a pre-programmed autonomous routine and score points. Most teams resort to running a serise of discreate linear paths, using encoder distance or time-based approaches. A down-side to these approaches is that, when an obstacle lies between the current and desiered positions, multiple paths must be run consequtively, introducing inefficiencies and potential for large errors. The motivatin of this project was to create an interface to easily generate a continuous and flexible path between two points.

# Background

### Cubic Hermite Interpolation

The overarching idea is to use an initial and end point and "velocity" pair to generate a flexible path. The initial and end point dictate the start and end point of the path, and the "velocity" dictates the orientation and it's effect on the path. "velocity" is placed in quotations, because it does not represent the robot's velocity at the start and end point, and rather the weight of the end-point orientations on the curvature of the path. This is an interpolation problem, and specifically, this implementation uses Hermite spline interpolation. It can be said that a Hermite spline is mathematically identical to a Bezier curve, another well-known curve interpolation algorithm. A Hermite spline is a better fit for robot path-planning because orientation at the start and end points of the path can be explicitly defined, whereas for Bezier curves, mid-points that are arbiturary in the physical context are used to define curvature.

As stated above, the Hermite Spline is generated using four pieces of information. They will hitherto be reffered to as follows:

$P_0$ ::= The x-y coordinate of the start point  
$P_1$ ::= The x-y coordinate of the end point  
$P_2$ ::= The "velocity" of the start point  
$P_3$ ::= The "velocity" of the end point

We then consider a general cubic polynomial and its derivative in terms of a parameter $t \in \mathbf{R}$ and 2-dimensional vector coeffients $a, b, c, d \in \mathbf{R^2}$.

$$ \begin{align*} 
P(t) &= at^3+bt^2+ct+d \\
P'(t) &= 3at^2+2bt+c
\end{align*} $$

Natrually, parametric function $P(t)$ produces a 2-dimensional vector, or x-y coordinate, and we will define it such that $P(0)$ gives the start point of the path, and $P(1)$ gives the end point of the path. In particular, plugging 0 and 1 into $P(t)$ and $P'(t)$ gives the following.

$$ \begin{align*}
P(0) = P_0 &= d\\
P(1) = P_1 &= a + b + c + d\\
P'(0) = P_2 &= c\\
P'(1) = P_3 &= 3a + 2b + c
\end{align*} $$

Rewritting in matrix form gives:

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

$$ 
\begin{bmatrix}
0 & 0 & 0 & 1 \\
1 & 1 & 1 & 1 \\
0 & 0 & 1 & 0 \\
3 & 2 & 1 & 0
\end{bmatrix} ^ {-1}
\begin{bmatrix} P_0 \\ P_1 \\ P_2 \\ P_3 \end{bmatrix}
=
\begin{bmatrix} a \\ b \\ c \\ d \end{bmatrix}
$$

Performing Gaussian elimination on the inverse matrix gives,

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

To recap, the relationship above gives the coefficients of our cubic interpolation, expressed in terms of the four pieces of input information. Lastly, the parametric equation given the coeffecients is as follows.

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

Therefore, 

$$ P(t) = P_0 h_1 + P_1 h_2 + P_2 h_3 + P_3 h_4 $$

where,

$h_{1}(t) = 2t^3-3t^2+1$  
$h_{2}(t) = -2t^3 + 3t^2$  
$h_{3}(t) = t^3-2t^2+t$  
$h_{4}(t) = t^3-t^2$  

$h_{1}(t)$ through $h_{4}(t)$ is refered to as the 4 Hermite baisis functions. 

### Velocity Profile

In practice, the interpolatin function is sampled discretely during the robot initialization time given before each match. During sampling, it is also possible to calculate a discrete approximation of the first and second order derivatives of the path. This information can be important because fast robots may want to scale their velocity as to not overrun or tip when passing through sharp corners. 

The objective was to define a function that can convert intuitive parameters into a flexible velocity-profile. Specifically, the function will be defined by the following 6 parameteres:

$d$ ::= The distance of the profile  
$m$ ::= The maximum velocity of the profile  
$k_1$ ::= acceleration constant  
$k_2$ ::= deceleration constant  
$s_1$ ::= initial velocity  
$s_2$ ::= final velocity  

These 6 parameters should be sufficient to define any given velocity profile, with a relatively large degree of freedom. 

We begin with a general form of the sigmoid function, where $a_i, b_i$ are constants.

$$ f(x) = \frac{1}{1 - a_ie^{-b_ix}} \left( a_i > 0, b_i > 0 \right)$$

The "steepness" of the curve is defined by the exponential coefficient $b_i$. Thus,

$$ b_i = k_1 $$

Next, given that $f(0)$ should equal the initial velocity parameter $s_1$, $a_i$ is derived as follows.

$$ s_1 = f(0) = \frac{1}{1 + a_i}
\implies a_i = \frac{1}{s_1} - 1 $$

Similarly, the deceleration end of the profile can be defined by a generalization of the inverted sigmoid, where $a_f, b_f$ are constants.

$$ g(x) = \frac{1}{1 - a_fe^{b_fx}} \left( a_f > 0, b_f > 0 \right)$$

Because the $g(0)$ of this graph should allign with $x=d$, the graph is shifted to the right by $d$ as follows:

$$ g(x) = \frac{1}{1 - a_fe^{b_f(x-d)}} \left( a_f > 0, b_f > 0 \right)$$

Finally, multipling the two graphs and scaling it by the maximum velocity $m$ results in the profile's final form.

![Velocity Profile](./2025-5-28-Path-Planner-assets/xdrive.svg "A slide from the presentation linked above")

$$ mf(x)g(x) = 
\frac{m}{
  \{ 1 + ( \frac{m}{s_1} - 1 ) e^{-k_1x} \} 
  \{ 1 + ( \frac{m}{s_2} - 1 ) e^{k_2(x - d)} \} 
}
$$


### Intricacies of Holonomic Control

Holonomic drives are robotic drive bases that are able to theoretically translate in all directions and rotate similtaneously. There are many types of holonomic drive bases, but Vex robotics is dominated by the "X-drive" (pictured below) due to its simplicity. Despite being a holonomic drive base however, the true mobility of these drives are not fully exploited at the High school level, as the control is significantly more advanced than its differential counterpart.

For example, given a controller with left joystick input $(lx, ly)$ and right joystick input $(r_x, r_y)$, we assign the following relationship.

$$
\mathbf{v} = 
\begin{bmatrix} l_x \\ l_y \end{bmatrix}
\ \ \ \ 
\omega = r_x
$$

where $\mathbf{v}$ represents the velocity of the robot **relative to the field**, and $\omega$ represents angular velocity. "relative to the field" means that, regardless of the orientation of the robot, moving the left joystick in a certain direction always moves the robot in the same direction relative to the field. 

This method of control, typically referred to as **field-centric** drive control, can be achived by measuring the robot's heading using a gyroscope, which will be refered to as $\theta$ here. Given a robot's heading $\theta$, we can use a rotation matricy to rotate the input vector $\mathbf{v}$, $-\theta$ degrees about the origin, to convert the field-centric input into a robot-centric one. The relationship is given by,

$$
\mathbf{v_{robot}} =
\begin{bmatrix}
  \cos(-\theta) & - \sin(-\theta) \\
  \sin(-\theta) & \cos(-\theta)
\end{bmatrix}
\begin{bmatrix} l_x \\ r_x \end{bmatrix}
$$

Imagine that the robot has a heading of 90 degrees clockwise. Without processing, pushing the left joystick directly forward would move the robot directly to the right relative to the field. To counter-act this, the controller input is rotated -90 degrees, or 90 degrees counter-clockwise. This means that moving the left joystick directly forward tells the robot to move directly to the left, which would be correct from the robot's persepective. This process can be restated as, **converting the input expressed in the world-baisis into the robot-baisis.**

![X-drive](./2025-5-28-Path-Planner-assets/xdrive.svg "A slide from the presentation linked above")

Redefining $\mathbf{v}$ to $\mathbf{v_{robot}}$, we will now consider how the individual wheel motors must be moved in order to produce movement in the intended direction. The X-drive is an omni-directional drive, meaning that it can translate in any direction. We will achieve this by taking the inner-product of our velocity vector $\mathbf{v}$ and 4 unit vectors that represent the forward direction of each wheel. Specifically, front left, front right, rear left, and rear right motors given as follows.

$$
\mathbf{v_{fl}} = \begin{bmatrix} \cos(45) \\ \sin(45) \end{bmatrix} \ \
\mathbf{v_{fr}} = \begin{bmatrix} \cos(135) \\ \sin(135) \end{bmatrix} \ \
\mathbf{v_{rl}} = \begin{bmatrix} \cos(135) \\ \sin(135) \end{bmatrix} \ \
\mathbf{v_{rr}} = \begin{bmatrix} \cos(45) \\ \sin(45) \end{bmatrix} \ \
$$

Taking the inner-product of vectors can give a measure of how closely they allign in space. This information can be used to determine how much each wheel should be rotated relative to each other, to move in the desiered direction. Lastly, considering the direction of the angular velocity input $\omega$ from ealier, the velocity of each wheel is given as follows:

$$ \begin{align*}
v_{fl} &= l_x * \cos(45)\ \ + l_y * \sin(45)\ \ + \omega \\  
v_{fr} &= l_x * \cos(135) + l_y * \sin(135) - \omega \\    
v_{rl} &= l_x * \cos(135) + l_y * \sin(135) + \omega \\    
v_{rr} &= l_x * \cos(45)\ \  + l_y * \sin(45)\ \  - \omega
\end{align*} $$

These values are then normalized by dividing each value by the largest value, ensuring that the motor output is never greater then 1, and the relative relationship of the motors are not lost. Specifically, this operation is critical in preserving the relationship derived from the inner-products.

In this implementation, odometry encoders are used alongside the gyroscope, to perform real-time pose estimation. Discrete integration of the encoder's velocity is used to find the positional displacement of the robot every iteration. As these displacemment vectors are given in the robot's baisis, they are rotated against the gyroscope heading to derive the robot's estimated pose relative to the field. Similarly, distance traveled can be measured by discretely integrating the norm of the displacement vectors, and this feature is used to run the trajectories.

### Desmos UI

[Path Planner](https://www.desmos.com/calculator/zqpztqvvoi)
[Path Planner Plus](https://www.desmos.com/calculator/dg7kybaxyb)

The two desmos Graphs linked here serve as a UI for the trajectory generation. The dots that represent the robot's initial and final coordinate and "velocity" can be dragged along the screen. In line with the behaivior of Hermite splines, extending the initial and final "velocities" make the increases the coresponding angle's "control" over the path.

The Path Planner Plus graph serves as the UI for the extended trajectory planning scheme, which is simply a piecewise interpolation of two Hermite splines. One extra coordinate and "velocity" are added to the path, allowing greater and more direct control over the behavior of the path around the middle point. 

# Implementation

## Path Generation

The following code is a direct and naive implenmatation of the theory presented above.

```c++
  /// @brief エルミート補間式にある処理位置（x）を問い、そ地点の姿勢を返す
  /// @param path エルミート補間式の定義
  /// @param previous 前回の処理位置の姿勢
  /// @param x 処理位置（0から１）
  /// @return 処理位置（x）のロボット姿勢
  Pose CubicHermiteInterpolation(Path path, Pose previous, float x) {
    // エルミート補間多項式の表現
    float h1 = 2*(x*x*x) - 3*(x*x) + 1; //　１から始まり０に向かって低下する
    float h2 = -2*(x*x*x) + 3*(x*x);    //　０から始まり１に向かって上昇する
    float h3 = (x*x*x) - 2*(x*x) + x;   //　序盤に上に膨らみ終盤に低下する
    float h4 = (x*x*x) - (x*x);         //　終盤に下に膨らみ序盤に低下する
    // エルミート補間定義に従い処理位置（x）の姿勢を導く
    Pose current; //　姿勢オブジェクトを作成
    current.x = path.p0.x * h1 + path.p1.x * h2 + path.t0.x * h3 + path.t1.x * h4;  //　x　値を導く
    current.y = path.p0.y * h1 + path.p1.y * h2 + path.t0.y * h3 + path.t1.y * h4;  //　y　値を導く
    //　今回の位置から前回の位置を引くことでその差を表すベクトルを生成
    //　生成されたベクトルの角度を導き経路の角度を近似することができる
    current.w = Vector {current.x - previous.x, current.y - previous.y}.getAngle(); 
    return current; //　姿勢を返す
  }
```

Because the only thing that changes between iterations of path generation is the parameter $t$, it is faster to group the matrix with the vector of input information as shown below, so as to require only one calculation of the polynomial's coefficients.

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

For holonomic drives, it is possible to define an orientation path distinct from the "velocity" orientation. Setting the start point of the path to 0, and the end point to 1, a vector of so-called "HolonomicPose" can be passed to define the orientation of the robot at different points of the path. For example, a pair such as {0.5, 180} would indicate that the robot should be facing 180 degrees exactly halfway through the path. These distance and angle pairs are then lineraly interpolated produce a smooth transition between each setpoint angle when the path is discreatized. The code for this given below.

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

Code for the actual discreatization of the path is given below. The "clarity" of the path is set to 100 by default, meaning 100 points are sampled through the path, velocity profile, and orientation interpolation, and stored as a vector of "Waypoint" objects.

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

The field-centric Holonomic drive code as described above, is also given

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

And finally, the path-following method is defined as such. As can be observed, pose estimation is used to update a "distanceTraveled" variable, which is then used to query the trajectory for a new waypoint. The returned waypoint has all the data necessary to simulate a controller input.

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