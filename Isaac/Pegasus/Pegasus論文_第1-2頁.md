# Pegasus Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation

**arXiv**：2307.05263v1 [cs.RO] — 2023 年 7 月 11 日

**作者**：
- Marcelo Jacinto¹, João Pinto¹, Rita Cunha¹, António Pascoal¹ — ISR/IST, 里斯本大學（葡萄牙）
- Jay Patrikar², John Keller², Sebastian Scherer² — 卡內基梅隆大學機器人研究所（美國）

---

## Abstract（摘要）

為空中飛行器開發並驗證新型控制與運動規劃演算法是一項極具挑戰性的任務。機器人學研究界越來越依賴 3D 模擬技術，以便在多種環境與條件下評估新演算法的性能。

本研究介紹了 **Pegasus Simulator**——一個以 **NVIDIA® Isaac Sim** 擴充套件形式實現的模組化框架，能夠在**照片寫實**的環境中**即時模擬多架多旋翼飛行器**，並透過其模組化架構與直覺式圖形使用者介面（GUI），提供與廣泛採用的 **PX4-Autopilot** 及 **ROS2** 的開箱即用整合。

為展示其能力，本文實作了一個**非線性控制器（Nonlinear Controller）**，並呈現了兩架無人機執行高機動飛行動作的模擬結果。

**補充資料**：
- 🔗 GitHub：<https://github.com/PegasusSimulator/PegasusSimulator>
- 🎬 示範影片：<https://youtu.be/caFPdl1rOT4>

---

## I. Introduction（引言）

近年來，無人空中飛行器（UAV）的需求急速成長，尤其是小型多旋翼系統。這類飛行器提供了一種高品質且低成本的從空中俯瞰環境的手段，因此已成為**空中攝影、監控與維護任務**的首選工具。

過去幾年機器學習（ML）的量子躍進徹底改變了機器人學的面貌，以及自主系統的設計方式。如今，同時設計結合「控制」與「感知」的系統已成為常態。控制研究社群長期以來致力於整合進階控制技術，如：

- 非線性控制（Nonlinear Control）
- 模型預測控制（MPC, Model Predictive Control）
- 強化學習（RL, Reinforcement Learning）

然而，⚠️ **獲取資料以訓練和驗證這些演算法可能代價高昂、耗時、不切實際且存在安全風險**——因為收集飛行資料往往必須使用真實的實體飛行器。

### 一個優質模擬框架需要具備的條件

為了提供一個良好的演算法開發環境，一個合格的模擬框架必須保證以下特性：

1. ✅ **物理精確的飛行器及感測器模型**，並能以高速率生成資料。
2. ✅ **照片寫實的模擬環境**，及具真實感的多模態感測器資料。
3. ✅ **支援多飛行器同時並行模擬**。
4. ✅ **提供直覺且簡單的應用程式介面 (API)**，以實現快速原型開發。
5. ✅ **提供與真實飛控韌體的整合**，即同時具備軟體在環（SITL）與硬體在環（HITL）能力。

本論文提出的 **Pegasus Simulator** 正是一個具備上述所有特性的解決方案——以 NVIDIA® Isaac Sim 作為基礎，打造一個開源且模組化的框架，以優雅且強大的方式在照片寫實環境中模擬多旋翼飛行器。

---

## A. Related Work（相關工作）

多年來，許多模擬器與框架相繼被開發出來，各有其優缺點。

### 模擬器特性比較表

| 模擬框架 | 底層引擎/模擬器 | 照片寫實 | 視覺感測器 | 機載感測器 (IMU, GPS 等) | MAVLink 介面 | API 複雜度 |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **jMAVSim** | Java | ✗ | ✓ | ✗ | ✓ | ★☆☆ |
| **RotorS** | Gazebo | ✗ | ✓ | ✓ | ✓ | ★★☆ |
| **PX4-SITL** | Gazebo | ✗ | ✓ | ✓ | ✓ | ★★☆ |
| **AirSim** | Unreal Engine / Unity | ✓ | ✓ | ✓ | ✓ | ★★★ |
| **Flightmare** | Unity | ✓ | ✓ | ✓ | ✗ | ★★★ |
| **MuJoCo** | OpenGL / Unity | ✗ | ✗ | ✗ | ✗ | ★★☆ |
| **Pegasus** ⬅️ | **NVIDIA Isaac Sim** | ✓ | ✓ | ✓ | ✓ | ★★☆ |

### 各平台評析

**▸ Gazebo (含 RotorS & PX4-SITL)**：
機器人學社群最廣泛採用的模擬器之一。支援 ROS 框架，具備剛體物理引擎（可與 OpenGL 渲染引擎解耦運行）。然而，Gazebo 在模擬複雜視覺環境時相當笨重，視覺保真度無法與現代遊戲引擎（如 Unreal Engine、Unity）競爭。

**▸ AirSim (Microsoft)**：
以 Unreal Engine 為渲染後端的四旋翼模擬擴充套件，提供基本的 PX4-Autopilot 整合及 OpenAI Gym 介面（適合 RL）。

**▸ Flightmare (UZH)**：
以 Unity 遊戲引擎為渲染後端，整合 OpenAI Gym，但**不提供**開箱即用的 PX4-Autopilot 整合。

> ⚠️ **共同缺點**：上述兩者雖然視覺效果出色，但其擴充介面相當複雜，對機器人學研究者不夠友善。這些方案是構建在遊戲引擎之上，並非從機器人學社群的需求出發，因此擴充通常既困難又緩慢。

**▸ jMAVSim**：
PX4 社群提供的 Java 模擬器，僅用於測試 PX4-Autopilot 的基本功能（如 Pixhawk 飛控），**沒有**照片寫實的視覺能力。

**▸ MuJoCo**：
高效的通用剛體物理引擎，在控制與 RL 研究中被廣泛採用，但**沒有**任何飛控軟體的整合能力，在空中機器人學社群中尚未普及。

### NVIDIA® Isaac Sim 的崛起

NVIDIA® 近年推出了 Omniverse 模擬工具套件，其中包含 **Isaac Sim**——一個專為機器人應用與多模態資料生成量身打造的現代模擬器。它具備：
* **RTX 高品質渲染引擎**（獨立於 PhysX 物理引擎運行）
* 遵循由 Pixar 開發、被 Blender/Maya/3DS Max 廣泛支持的 **USD (Universal Scene Description)** 標準
* 支援 ROS 社群採用的 **URDF 格式**，以及基本的 ROS1/ROS2 整合
* 整合 NVIDIA® Isaac Gym 的包裝器，以及 **Isaac Orbit** 等擴充套件
* 支援多模態感測器模擬（RGB 相機、深度相機、光達等）

然而，Isaac Sim 原生**欠缺對空中飛行器的完整支援**——大多數工具針對機械臂與輪式機器人，且缺乏氣壓計、GPS 等感測器，也沒有 PX4 或 MAVLink 的整合。

**Pegasus Simulator** 正是為了填補這個空缺而誕生，以 Isaac Sim Python 擴充套件的形式，將 Gazebo 等級的開發者體驗與 PX4 / MAVLink 完整支援，以及類遊戲引擎的照片寫實渲染全部集於一身。

**第一個版本提供的功能：**
* 一個配備 FPV 相機的 **3DR® IRIS 四旋翼**模型
* 透過 **MAVLink** 進行通訊，直接整合 **PX4-Autopilot (SITL/HITL)**，以及 **ROS2** 通訊後端
