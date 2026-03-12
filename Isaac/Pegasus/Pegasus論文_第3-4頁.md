# Pegasus Simulator 論文解析 — 第 3–4 頁

> 本段對應論文原文章節：**II. System Architecture（系統架構）** 與 **III. Vehicle and Sensor Modeling（飛行器與感測器建模）** 前半段。

---

## II. 系統架構 (System Architecture)

Pegasus Simulator 採用**模組化架構**，以 Python 擴充套件的形式介接 NVIDIA® Isaac Sim，如下方架構示意圖所示。

### 整體架構概念

在模擬世界中，**一架飛行器 (Vehicle)** 由以下元件組成：
* 一個以 **USD 格式**定義的 3D 模型，被放置於模擬世界中
* 遵循**繼承設計模式 (Inheritance Pattern)**，多旋翼 (Multirotor) 是 Vehicle 物件的子類別，包含：
  * **感測器 (Sensors)**：IMU、GPS、磁力計、氣壓計
  * **推進力模型 (Thruster Model)**
  * **阻力模型 (Drag Model)**
  * **一個或多個控制後端 (Control Backends)**：供使用者控制飛行器、取得系統狀態及感測器數據

```
┌────────────────────────────────────────────────────────────┐
│                     NVIDIA Isaac Sim                        │
│   3D WORLDS | RL TOOLS | MULTIMODAL SENSORS                │
│                        ↕  VEHICLE API                       │
│   ┌─────────────────────────────────────────────────────┐  │
│   │           MULTIROTOR API (飛行器核心 API)           │  │
│   │  [3D MODEL][THRUSTERS API][DRAG API][SENSORS API]   │  │
│   │   ↳ BAROMETER  MAGNETOMETER  IMU  GPS               │  │
│   │                ↕  CONTROL BACKEND API                │  │
│   │  Python Script │ ROS2 Nodes │ MAVLink (PX4 SITL)    │  │
│   └─────────────────────────────────────────────────────┘  │
│                  VEHICLE MANAGER (車輛管理器)               │
└────────────────────────────────────────────────────────────┘
```

### A. 感測器、推力與阻力模型 (Sensors, Thrust, and Drag Models)

此框架的第一個版本在 Isaac Sim 原有的感測器套件上**新增了四種額外感測器**：
1. 氣壓計 (Barometer)
2. 磁力計 (Magnetometer)
3. IMU（慣性測量單元）
4. GPS

透過**感測器 API (Sensors API)** 可直接取得飛行器狀態，並自動處理各感測器的更新頻率（Update Rate）。此架構同時支援**自訂推力曲線**與**阻力模型**。目前的實作提供了一個**二次方推力曲線 (Quadratic Thrust Curve)** 與**線性阻力模型 (Linear Drag Model)**。

### B. 控制後端 (Control Backend)

為支援多樣的制導（Guidance）、導航（Navigation）與控制（Control）應用，Pegasus 提供了帶有**回調函式 (Callback Functions)** 的控制後端 API，可便捷地存取系統狀態與各機載感測器資料。每個控制後端都必須實作一個方法，用於設定每顆旋翼的**目標角速度 (Target Angular Velocities)**。

目前的開箱即用控制後端包含：
* **Python 腳本 (Python Script)**：直接控制
* **ROS2 Nodes**：透過 ROS2 Topics/Services 控制
* **MAVLink (PX4-Autopilot)**：完整整合 PX4 SITL，可**自動啟動/停止 PX4 模擬**，無需手動開啟額外的終端機。

### C. Pegasus 介面 (Pegasus Interface)

**Vehicle Manager** 負責追蹤模擬環境中所有已加入的飛行器實例，可透過 Python 腳本或圖形介面（GUI）加入飛行器。

### D. 圖形使用者介面 (GUI)

Pegasus 提供一個直觀的 GUI，讓使用者能夠：
* 從預先提供的環境資產中選擇 3D 世界（如 NVIDIA 倉庫場景）
* 從預先設定好的飛行器配置中選擇機型（如 3DR® IRIS）
* 在背景自動啟動 PX4-Autopilot 模擬
* 無需直接操作 Python API 即可完成上述操作

---

## III. 飛行器與感測器建模 (Vehicle and Sensor Modeling)

### A. 座標系 (Notation and Reference Frames)

Pegasus（同 Gazebo）採用右手定則座標系：
* **慣性座標系 {U}**：Z 軸朝上，Y 軸朝北 → **ENU (East-North-Up) 慣例**
* **機體座標系 {B}**：**FLU (Front-Left-Up) 慣例**

> ⚠️ **PX4 相容性注意**：PX4-Autopilot 使用的是 **NED (North-East-Down)** 與 **FRD (Front-Right-Down)** 慣例。Pegasus 框架內部一律以 ENU-FLU 進行計算，並在輸出給感測器資料前進行座標轉換，以確保對 PX4 的完整相容性。

### B. 系統動力學 (System Dynamics)

論文考慮了一個**受線性阻力影響的多旋翼動力學模型**。定義：
* $\mathbf{p} := [x, y, z]^\top \in \mathbb{R}^3$：飛行器機體座標系 {B} 在慣性座標系 {U} 中的位置
* $\mathbf{v} := [\dot{x}, \dot{y}, \dot{z}]^\top$：速度
* $\mathbf{q} := [q_x, q_y, q_z, q_w]^\top \in S^3$：{B} 相對於 {U} 的姿態四元數

線性平移動力學方程：

$$\dot{\mathbf{p}} = \mathbf{v} \tag{1}$$

$$\dot{\mathbf{v}} = -g\mathbf{e}_3 + \underbrace{q \odot \mathbf{e}_3 \frac{T}{m} - q \odot (D q^{-1} \odot \mathbf{v})}_{\eta_1} \tag{2}$$

* $g \approx 9.81 \text{ m/s}^2$：重力加速度
* $T = \sum_{i=1}^{N} F_i$：N 顆旋翼的總推力（沿各旋翼 Z 軸分量的合力）
* $D = \text{diag}(d_x, d_y, d_z)$：對角線性阻力係數矩陣
* $\odot$：四元數誘導的向量旋轉運算子

角速度動力學方程：

$$\dot{\boldsymbol{\omega}} = J^{-1}(\boldsymbol{\tau} - \boldsymbol{\omega} \times J\boldsymbol{\omega}) \tag{3}$$
$$\dot{\mathbf{q}} = \frac{1}{2} \text{Sk}(\boldsymbol{\omega}) \cdot \mathbf{q} \tag{4}$$

* $J = \text{diag}(j_x, j_y, j_z)$：慣性張量（在機體座標系 {B} 中表示）
* $\boldsymbol{\omega} := [p, q, r]^\top$：角速度
* $\boldsymbol{\tau} := [\tau_x, \tau_y, \tau_z]^\top$：各旋翼施加力所產生的總扭矩
* **完整系統狀態**：$\{p, v, q, \omega\}$

### C. 旋翼建模 (Rotor Modeling)

扭矩由**分配矩陣 (Allocation Matrix)** $A$ 決定：

$$\boldsymbol{\tau} = A\mathbf{F} \tag{5}$$

Z 軸方向的反扭矩（Yaw 控制用）：

$$\tau_z = k \sum_{i=1}^{N} (-1)^{i+1} F_i \tag{6}$$

**推力模型（二次方）**：

$$F_i = c \cdot \omega_i^2, \quad i = 1, \ldots, N \tag{7}$$

* $\omega_i$：控制後端輸入的第 $i$ 顆旋翼目標角速度
* $c \in \mathbb{R}^+$：推力係數
* $k$：反扭力矩係數 (Reaction Torque Coefficient)

> 📌 **SimEng 重點筆記**：此模型**不含時間延遲 (No Time Delays)**。這是一個簡化假設，意謂若需要更精確的模擬，開發者需要透過 Modular Thruster API 自行加入 First-Order 延遲模型（即 Time Constant $\tau$）。

### D. 感測器建模 (Sensor Modeling)

所有感測器預設以 **250 Hz** 運行，GPS 則預設為 **1 Hz**。

**1. 氣壓計 (Barometer)**：採用 ISA 國際標準大氣模型，溫度：

$$T = T_0 - 0.0065h \tag{8}$$

氣壓：

$$p = \frac{p_0}{(T_0/T)^{5.2561}} + w + d \tag{9}$$

其中 $w$ 為高斯白雜訊，$d$ 為緩慢變化的偏差項（Random Walk）。
