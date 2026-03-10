# Closed-loop Validation

「閉環數位分生（Closed-loop Digital Twin）」的架構願景：不只是從設計到模擬，而是從模擬/實際運行產生的數據，反向修正（Inverse Mapping）物理機構與控制演算法。
- **數據驅動**：實現機構物理參數的逆向優化（Co-design）。

> [!IMPORTANT]
> **數位分生 (Digital Twin) 與傳統模擬的本質差異**：
> 傳統模擬通常是「開環」的單一驗證，而數位分生是**「全物理特性整合」**的閉環驗證，強調虛實世界的即時或循環反饋。

---

## 1. 核心開發框架與 GitHub 專案

- **[Isaac Lab](https://github.com/isaac-sim/IsaacLab)** (原名 Orbit / 基於 Isaac Sim)：這才是你解決物理機構問題的戰場。它擁有強大的物理引擎（PhysX），可以進行「系統辨識」與「強化學習（RL）」。
- **關鍵專案推薦**：[Aerial Gym](https://github.com/ntnu-arl/aerial_gym) & [Pegasus](https://github.com/PegasusSimulator/PegasusSimulator)
- **PyBullet 推薦？**：適合輕量級驗證，但精度與平行模擬能力（如需解決機構共振）建議直上 Isaac Lab。
- **NOTE (Isaac ROS)**：主要用於感測器處理與導航（例如 VSLAM、避障）。它不太能解決「機構共振」或「物理參數優化」，因為它處理的是軟體通訊與算法加速。

這些專案能幫助你實現從「數據」回到「物理參數」的逆向過程。

---

## 2. 關鍵論文參考 (理論基石)

如果你想實作「從數據反向修正機構」，請參考這幾篇論文的路徑：

1. **《Aerial Gym: A Framework for Highly Parallelized Simulation of Aerial Robots》 (2025)**:
   - 解釋了如何在 Isaac 環境中建立**可重構（Reconfigurable）**的無人機平台。
2. **《NeuroBAM: Neural Body Architecture Model》**:
   - 探討如何將「身體結構」與「神經控制」一起優化（Co-design）。
3. **《Sim-to-Real Transfer for Mobile Robots with Reinforcement Learning》**:
   - 探討如何將 Isaac Sim 訓練的結果（針對不均勻動力修正後的參數）遷移到真實機器人上。

---

## 3. 如何透過 Isaac 解決「共振」？

共振通常源於高頻振動與結構剛性的耦合。在 Isaac Lab 中，你可以這樣做：

- **高頻採集**：將模擬頻率調高（例如 $1\text{kHz}+$），並在馬達節點加入高頻擾動。
- **結構修正（URDF 逆向）**：
  - 在 Isaac 中運行無人機，收集 IMU 的震動頻譜。
  - 利用 **CMA-ES** (共變異矩陣適應策略) 或 **RL 算法**，自動調整 URDF 中的質量分佈（Mass Distribution）與轉動慣量（Inertia）。
  - **目標**：讓結構頻率避開馬達的工作頻率。

---

## 💡 實踐活動：情境模擬

假設你發現無人機在 $150\text{Hz}$ 時會發生劇烈共振，導致 IMU 數據失效（演算失效）。在 Isaac Lab 中，你會選擇哪種方式來「找到」解決方案？

- **(A) 手動嘗試**：在程式碼中手動嘗試調整機臂長度，每次改完都跑一次模擬。
- **(B) 大規模平行優化**：設定一個範圍（例如機臂長 $20\text{cm} \sim 25\text{cm}$），讓 Isaac 同時跑 1000 種不同的機型，並自動回傳震動最小的那一組。
- **(C) 降低轉速**：把馬達轉速調低，避開共振點。

**選一個答案，我將告訴你如何用 Isaac Lab 的 Manager-based 環境來實作它！**