# Pegasus Simulator 論文解析 — 第 5–6 頁

> 本段對應論文原文章節：**III-D 感測器建模（續）**、**IV. 範例應用情境**、**V. 結論**、**VI. 未來工作**，以及完整的**參考文獻**。

---

## III-D. 感測器建模（續）(Sensor Modeling continued)

### 2. 磁力計 (Magnetometer)

Pegasus 使用與 **PX4-SITL 相同的磁偏角 (Declination $D$)、磁傾角 (Inclination $I$) 及磁場強度 (Strength $S$)** 預計算查找表，資料來源為 **WMM-2015（世界地磁模型）**。以 ENU 慣例表示在慣性座標系 {U} 中的三軸磁場分量：

$$H = S \cos(I)$$

$$S_X = H \cos(D) + w_X + d_X$$

$$S_Y = H \sin(D) + w_Y + d_Y \tag{11}$$

$$S_Z = H \tan(I) + w_Z + d_Z$$

其中：
* $w \sim \mathcal{N}(0, \Sigma_S)$：帶協方差矩陣 $\Sigma_S$ 的加成性高斯白雜訊
* $d_X, d_Y, d_Z$：三軸上緩慢變化的**隨機遊走偏差 (Random Walk Bias)**

### 3. 慣性測量單元 (IMU)

IMU 由**陀螺儀（角速度）**與**加速度計（線加速度）**組成，測量值（帶雜訊）如下：

$$\begin{cases} \tilde{\omega} = \omega + \eta_g + b_g \\ \tilde{a} = \dot{v} + \eta_a + b_a \end{cases} \tag{12}$$

其中：
* $\omega$ 與 $\dot{v}$：飛行器在機體座標系 {B} 中的真實角速度，以及在慣性座標系 {U} 中的線加速度一階導數
* $\eta_g, \eta_a$：高斯白雜訊過程 (Gaussian White Noise Processes)
* $b_g, b_a$：緩慢變化的隨機遊走擴散過程（參考文獻 [18] Kalibr 校正方法）

### 4. GPS

本地位置（帶雜訊）的估計值 $\tilde{p}$：

$$\tilde{p} = p + \eta_p + b_p \tag{13}$$

其中 $\eta_p$ 為高斯白雜訊，$b_p$ 為隨機遊走過程。

為確保與 PX4 導航系統的完整相容性，必須將本地座標轉換為**全球座標（緯度與經度）**。Pegasus 採用**方位等距投影 (Azimuthal Equidistant Projection)** 配合 **WGS84 世界大地測量系統**，計算方式如下：

計算飛行器到中心點的角距離 $c$：
$$c = \sqrt{x^2 + y^2} \tag{14}$$

計算緯度 $\phi$ 與經度 $\lambda$：
$$\phi = \arcsin\left(\cos(c)\sin(\phi_0) + \frac{y \sin(c)\cos(\phi_0)}{c}\right) \tag{15}$$

$$\lambda = \lambda_0 + \arctan\left(\frac{x\sin(c)}{c\cos(\phi_0)\cos(c) - y\sin(\phi_0)\sin(c)}\right) \tag{16}$$

若 $c = 0$，則 $\phi = \phi_0, \lambda = \lambda_0$。

> 📌 **此模型確保了從模擬座標到 GPS 真實輸出格式（WGS84 緯經度）的完整鏈路，是 SITL 能正常運作的基礎。**

---

## IV. 範例應用情境 (Example Use-Case)

為展示框架的靈活性，論文使用**控制後端 API (Control Backend API)** 實作了 Mellinger & Kumar [21] 提出的**非線性控制器 (Nonlinear Controller)**，並複現了 Pinto et al. [22] 中兩架四旋翼執行**高機動接力飛行（Aggressive Relay Maneuvers）**的情境：

**軌跡方程（參數化形式）**：

$$x(t) = t$$
$$y(t) = 1 \cdot e^{-0.5(t/s)^2} \tag{17}$$
$$z(t) = 1 + 1 \cdot e^{-0.5(t/s)^2}$$

其中 $s = 0.6$ 為控制軌跡激烈程度的常數，$t$ 為參數變量。

**情境意義**：此場景特別具有代表性，因為在**實際硬體飛行之前**若未先在模擬中完成控制調校，對實機進行如此激進的飛行測試將是極度危險的行為。Pegasus 的存在，正是讓這類高風險演算法驗證可以在虛擬環境中被安全地反覆試誤。

**實驗結果**：兩架四旋翼均能成功追蹤各自的高機動軌跡，位置誤差有界，性能表現與 MATLAB 模擬結果相當。
* **測試硬體**：AMD Ryzen 5900X CPU + NVIDIA RTX 3090 GPU

---

## V. 結論 (Conclusion)

Pegasus Simulator 的第一個版本實現了：

1. **照片寫實的即時模擬環境**（基於 NVIDIA® Isaac Sim）
2. **直觀 GUI** 供快速原型開發
3. **開箱即用的 PX4 SITL / HITL 整合** 與 **ROS2 整合**
4. **模組化的 Python API**，便於擴充自訂感測器、飛行器、環境、通訊協議與控制層
5. **多飛行器同時模擬**能力

> 論文定位：Pegasus 是 **Gazebo（功能豐富、易用但不寫實）** 與 **遊戲引擎模擬器（視覺卓越但學習曲線陡峭）** 之間的中間地帶，是目前最適合同時兼顧「開發友善度」與「視覺保真度」的 UAV 開發平台。

---

## VI. 未來工作 (Future Work)

論文明確指出的後續發展方向（**對 SimEng 極具參考價值！**）：

* 🛩️ **擴充至固定翼飛機（Fixed-wing Aircraft）設計** — 目前框架僅支援多旋翼，未來將支援更多機型
* 💨 **新增空速感測器 (Airspeed)** 與 **光流感測器 (Optical Flow)** 模型
* 🎥 **新增虛擬雲台控制器 (Virtual Gimbal Controller)**，以提升與 PX4-Autopilot SITL 的相容性

> ⚠️ **對您（SimEng）的啟示**：Pegasus 官方目前尚未對「定翼機（Fixed-wing）」有完整的支援。如果您的專案目標是「定翼機數位孿生」，需要自行基於框架的模組化 API 擴充定翼機的氣動力模型（升力、阻力曲線），這恰好也是您 `CAD2DigitalTwin.md` 教材中所提到的 SimEng 核心挑戰之一。

---

## 參考文獻 (References)

| 編號 | 文獻 |
| :---: | :--- |
| [1] | NVIDIA, "NVIDIA Isaac Sim," 2022. — <https://developer.nvidia.com/isaac-sim> |
| [2] | N. Koenig & A. Howard, "Design and use paradigms for Gazebo," *IROS 2004* |
| [3] | M. Quigley et al., "ROS: an open-source robot operating system," *ICRA 2009* |
| [4] | F. Furrer et al., "RotorS — A Modular Gazebo MAV Simulator Framework," *Springer 2016* |
| [5] | L. Meier et al., "PX4: A node-based multithreaded open source robotics framework," *ICRA 2015* |
| [6] | Epic Games, "Unreal Engine," 2022. — <https://www.unrealengine.com> |
| [7] | A. Juliani et al., "Unity: A General Platform for Intelligent Agents," *arXiv 2018* |
| [8] | S. Shah et al., "AirSim: High-Fidelity Visual and Physical Simulation," *Field & Service Robotics 2018* |
| [9] | G. Brockman et al., "OpenAI Gym," *arXiv 2016* — <https://arxiv.org/abs/1606.01540> |
| [10] | Y. Song et al., "Flightmare: A flexible quadrotor simulator," *CoRL 2020* |
| [11] | A. Babushkin, "jMAVSim," 2013. — <https://github.com/PX4/jMAVSim> |
| [12] | L. Meier et al., "Pixhawk: A system for autonomous flight using onboard computer vision," *ICRA 2011* |
| [13] | E. Todorov et al., "MuJoCo: A physics engine for model-based control," *IROS 2012* |
| [14] | M. Mittal et al., "Orbit: A unified simulation framework for interactive robot learning," 2023 |
| [15] | NVIDIA, "Nvidia Omni.anim.people," 2022 |
| [16] | M. Cavcar, "The International Standard Atmosphere (ISA)," *Anadolu University, 2000* |
| [17] | Government of Canada, "Magnetic Components," 2020. — <https://geomag.nrcan.gc.ca> |
| [18] | J. Rehder et al., "Extending Kalibr: Calibrating IMUs," *ICRA 2016* |
| [19] | J.P. Snyder, "Map projections: A working manual," *U.S. Government 1987* |
| [20] | Mathworld, "Azimuthal Equidistant Projection," 2023 |
| [21] | D. Mellinger & V. Kumar, "Minimum snap trajectory generation and control for quadrotors," *ICRA 2011* |
| [22] | J. Pinto et al., "Planning Parcel Relay Manoeuvres for Quadrotors," *ICUAS 2021* |
