# PyBullet 模擬邏輯預備

既然你要寫 Python 程式來讀取 URDF 並施加 `p.applyExternalForce`，這裡有幾個物理實現的細節建議：

## A. 扭矩模擬 (Torque Simulation)
除了往上的推力（Thrust），你必須根據轉向施加反向扭矩（Yaw Torque），公式如下：
* **Motor 1 (CW)**: 施加一個繞著馬達 Z 軸為 **正** 的扭矩向量。
* **Motor 2 (CCW)**: 施加一個繞著馬達 Z 軸為 **負** 的扭矩向量。
* **Motor 3 (CCW)**: 施加一個繞著馬達 Z 軸為 **負** 的扭矩向量。

### PyBullet 真實坐標系映射 (World Coordinates vs local structure)
從程式自動讀取的數據中，揭示了世界基準坐標系與模型的前後左右關係（右翼為 -X，前頭為 -Y）：
* **馬達 1 (CW, 右前)**位置: `X = -0.25m, Y = -0.08m`,z = 0.085m
* **馬達 2 (CCW, 左前)**位置: `X = +0.25m, Y = -0.08m`,z = 0.085m
* **馬達 3 (CCW, 尾部)**位置: `X =  0.00m, Y = +0.36m`, z = 0.06m


## B. 傾轉舵機 (Tilt Servos)
在 URDF 中，Motor 1 與 Motor 2 的 Joint 應設為 `revolute`。
* **起飛 (Hover)**: `targetPosition` 設為 $0$ ($0^\circ$)。（依據 Issue List 目前已更新測試為 `0` rad）
* **巡航 (Cruise)**: `targetPosition` 設為 $1.57$ ($90^\circ$)。

## 3. 重心 (CoM) 與配重驗證
* **總重設定**: $2.25\text{ kg}$。
* **重心位置**: 機翼前緣 (LE) 往後 $50\text{ mm}$。精確實體座標為 `x, y, z = 0.513, 49.883, 1.455 mm`
* **平衡測試**: 在你的 PyBullet 腳本中，若三軸推力均等且無人機能平穩垂直上升，即代表重心定義與馬達力臂分佈正確。


<br/>
<hr/>

## 🛠️ Issue List (與 takeoff_test.py 對齊進度)

1. **修正起飛角度狀態**
   * `tiltRotor_L`, `tiltRotor_R` = `0 rad` (朝上懸停測試)
2. **全動態姿態與懸停控制** 
   * 依據 `motor_1_CW`, `motor_2_CCW`, `motor_3_CWW` 三個螺旋槳的轉速差與反扭矩。
   * 將三者整合進 PID 控制器中，全面控制飛機的動態姿態 (Roll, Pitch, Yaw)。
   * **垂直推力維持**：提供足以抵銷重力的向下推力以對抗地心引力，讓 $2.25\text{ kg}$ 飛機穩定懸停。
- 給予反向推力 forceObj=[0, 0, -force_n]
