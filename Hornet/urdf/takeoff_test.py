import pybullet as p
import pybullet_data
import time
import math

class SimplePID:
    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = output_limit
        self.integral = 0
        self.prev_error = 0

    def update(self, error, dt):
        self.integral += error * dt
        # 積分抗飽和 (Anti-windup)
        if self.ki != 0:
            i_limit = self.limit / max(self.ki, 1e-5)
            self.integral = max(-i_limit, min(i_limit, self.integral))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(min(output, self.limit), -self.limit)


class HornetSim:
    def __init__(self, urdf_path):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.planeId = p.loadURDF("plane.urdf")

        startPos = [0, 0, 0.05]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.droneId = p.loadURDF(urdf_path, startPos, startOrientation)

        self.joints = {}
        for i in range(p.getNumJoints(self.droneId)):
            info = p.getJointInfo(self.droneId, i)
            name = info[1].decode('utf-8')
            self.joints[name] = i

        print("\n=== 初始化完成 ===")
        print("找到的關節:", list(self.joints.keys()))

        # 傾轉馬達設定為 0 (朝上起飛)
        if 'tiltRotor_L' in self.joints and 'tiltRotor_R' in self.joints:
            p.setJointMotorControl2(self.droneId, self.joints['tiltRotor_L'],
                                    p.POSITION_CONTROL, targetPosition=0, force=50)
            p.setJointMotorControl2(self.droneId, self.joints['tiltRotor_R'],
                                    p.POSITION_CONTROL, targetPosition=0, force=50)

        self.m1 = self.joints.get('motor_1_CW', -1)
        self.m2 = self.joints.get('motor_2_CCW', -1)
        self.m3 = self.joints.get('motor_3_CWW', -1)

    def apply_thrust_world(self, link_index, thrust_n):
        """施加世界座標 +Z 方向的升力，完全不受飛機姿態影響"""
        if link_index < 0:
            return
        p.applyExternalForce(self.droneId, link_index,
                             forceObj=[0, 0, thrust_n],
                             posObj=[0, 0, 0],
                             flags=p.WORLD_FRAME)

    def apply_yaw_torque_world(self, link_index, torque_z):
        """施加世界座標 Z 軸扭矩（Yaw 反扭矩）"""
        if link_index < 0:
            return
        p.applyExternalTorque(self.droneId, link_index,
                              torqueObj=[0, 0, torque_z],
                              flags=p.WORLD_FRAME)

    def run(self):
        print("\n開始模擬... (路線A: 世界座標系正交推力 + 機身座標阻尼)")
        dt = 1. / 240.
        target_z = 2.0
        Kf = 2.6e-8
        Km = 0.05 * Kf

        # --- 靜態配重計算 (來自 diagnose.py 的實測數據) ---
        # 馬達實測位置: M1(Y=-0.0808), M2(Y=-0.0808), M3(Y=+0.3598)
        # CoM 實測: 出生時 RPY=(0,0,0)，代表 PyBullet 把重心視為幾何中心 (約 Y≈0)
        # ★ 關鍵：CoM 在 Y≈0 而不是 Y=0.044！
        # 前馬達力臂 = |0 - (-0.0808)| = 0.0808m
        # 後馬達力臂 = |0.3598 - 0| = 0.3598m
        #
        # 靜力矩平衡（繞 X 軸）：
        # 2 * T_front * 0.0808 = T_rear * 0.3598
        # T_rear = 2 * T_front * (0.0808 / 0.3598) = 0.4492 * T_front
        #
        # 總推力 = 2 * T_front + T_rear = M * g = 2.25 * 9.81 = 22.07 N
        # 2 * T_front + 0.4492 * T_front = 22.07
        # T_front * 2.4492 = 22.07 → T_front = 9.01 N
        # T_rear  = 0.4492 * 9.01 = 4.05 N
        # ==========================================
        base_thrust_front = 9.01  # N（前馬達，力臂短所以推力要大）
        base_thrust_rear  = 4.05  # N（後馬達，力臂長所以推力要小）
        base_rpm_front = math.sqrt(base_thrust_front / Kf)  # ~18622 RPM
        base_rpm_rear  = math.sqrt(base_thrust_rear  / Kf)  # ~12474 RPM
        print(f"靜態懸停 RPM (修正後): 前方 {int(base_rpm_front)}, 後方 {int(base_rpm_rear)}")

        # --- PID 控制器 ---
        pid_z     = SimplePID(500.0,  10.0, 2000.0, 5000.0)
        pid_roll  = SimplePID(600.0,   0.0,  200.0, 3000.0)
        pid_pitch = SimplePID(600.0,   0.0,  200.0, 3000.0)
        pid_yaw   = SimplePID(800.0,  50.0,  200.0, 2500.0)
        target_yaw = 0.0

        # --- [開放參數] 偏航配平 ---
        yaw_trim = 0.0
        
        # --- 診斷模式：前 2 秒等速推力，印出真實 RPY 方向 ---
        print("診斷中... 前 2 秒施加均等推力，印出機身真實角度")
        for warmup_step in range(480):  # 2 秒
            pos, orn, *_ = p.getBasePositionAndOrientation(self.droneId), None
            pos, orn = p.getBasePositionAndOrientation(self.droneId)
            euler = p.getEulerFromQuaternion(orn)
            z = pos[2]
            # 只給剛好懸停的推力
            f_front = base_thrust_front
            f_rear  = base_thrust_rear
            self.apply_thrust_world(self.m1, f_front)
            self.apply_thrust_world(self.m2, f_front)
            self.apply_thrust_world(self.m3, f_rear)
            p.stepSimulation()
            time.sleep(dt)
            if warmup_step % 120 == 0:
                print(f"  Warmup Z={z:.3f}m RPY=({math.degrees(euler[0]):.1f}°,{math.degrees(euler[1]):.1f}°,{math.degrees(euler[2]):.1f}°)")
        print("診斷完成，切換 PID 控制！")
        
        counter = 0
        try:
            while True:
                if not p.isConnected():
                    break

                pos, orn = p.getBasePositionAndOrientation(self.droneId)
                _, ang_vel = p.getBaseVelocity(self.droneId)
                euler = p.getEulerFromQuaternion(orn)
                roll, pitch, yaw = euler[0], euler[1], euler[2]
                z = pos[2]

                # 把世界座標角速度投影到機身座標
                # 這樣不管 Yaw 旋轉多少度，Roll/Pitch 的阻尼方向都是正確的！
                rm = p.getMatrixFromQuaternion(orn)
                # rm 是 3x3 旋轉矩陣，展平成 9 個元素
                # 機身 X 軸 = (rm[0], rm[3], rm[6])
                # 機身 Y 軸 = (rm[1], rm[4], rm[7])
                # 機身 Z 軸 = (rm[2], rm[5], rm[8])
                wx = rm[0]*ang_vel[0] + rm[3]*ang_vel[1] + rm[6]*ang_vel[2]
                wy = rm[1]*ang_vel[0] + rm[4]*ang_vel[1] + rm[7]*ang_vel[2]
                wz = rm[2]*ang_vel[0] + rm[5]*ang_vel[1] + rm[8]*ang_vel[2]

                # --- 高度 PID ---
                u_z = pid_z.update(target_z - z, dt)

                # --- 姿態 PID (目標 Roll=0, Pitch=0) ---
                u_roll  = pid_roll.update(-roll, dt)  - 200.0 * wx
                u_pitch = pid_pitch.update(-pitch, dt) - 200.0 * wy

                # --- Yaw 絕對鎖定 ---
                yaw_err = target_yaw - yaw
                yaw_err = (yaw_err + math.pi) % (2 * math.pi) - math.pi
                u_yaw = pid_yaw.update(yaw_err, dt) - 300.0 * wz

                # --- 混控器 (Mixer) ---
                # 馬達位置已確認: M1(右前X=-0.25), M2(左前X=+0.25), M3(後Y=+0.36)
                # 右傾(roll>0) 需加大右側M1, 減小左側M2 -> M1: -u_roll, M2: +u_roll
                # 前傾(pitch>0) 需加大前側M1/M2, 減小後M3 -> M1,M2: -u_pitch, M3: +u_pitch
                # 左偏(yaw>0,CCW) 需產生CW扭矩 -> 加大CW馬達M1(yaw+), 減小CCW馬達M2/M3(yaw-)
                rpm1 = max(0.0, min(24000.0, base_rpm_front + u_z - u_roll - u_pitch + u_yaw + yaw_trim))
                rpm2 = max(0.0, min(24000.0, base_rpm_front + u_z + u_roll - u_pitch - u_yaw - yaw_trim))
                rpm3 = max(0.0, min(24000.0, base_rpm_rear  + u_z          + u_pitch * 0.78 - u_yaw - yaw_trim))

                # --- 1. 世界座標推力 (永遠 +Z，不受機身歪斜影響！) ---
                f1 = Kf * rpm1**2
                f2 = Kf * rpm2**2
                f3 = Kf * rpm3**2
                self.apply_thrust_world(self.m1, f1)
                self.apply_thrust_world(self.m2, f2)
                self.apply_thrust_world(self.m3, f3)

                # --- 2. Yaw 反扭矩 (世界座標 Z 軸) ---
                # M1(CW)  在真實世界讓機身往 CCW 轉 -> 要讓機身產生 +Z(CW) 反力須給 -Z 的反作用。
                # M2/M3(CCW) 在真實世界讓機身往 CW 轉  -> 要讓機身產生 -Z(CCW) 反力須給 +Z 的反作用。
                self.apply_yaw_torque_world(self.m1, -Km * rpm1**2)
                self.apply_yaw_torque_world(self.m2,  Km * rpm2**2)
                self.apply_yaw_torque_world(self.m3,  Km * rpm3**2)

                # --- 3. 姿態扶正力矩 (直接對機身重心施加世界座標扭矩) ---
                # 這是最直接、最不受座標系影響的補償方式
                p.applyExternalTorque(self.droneId, -1,
                                      torqueObj=[15.0 * u_roll, 15.0 * u_pitch, 0],
                                      flags=p.WORLD_FRAME)

                p.stepSimulation()
                time.sleep(dt)

                counter += 1
                if counter % 240 == 0:
                    px, py = pos[0], pos[1]
                    print(f"XYZ: ({px:.2f},{py:.2f},{z:.3f})m | "
                          f"RPY: ({math.degrees(roll):.1f}°,{math.degrees(pitch):.1f}°,{math.degrees(yaw):.1f}°) | "
                          f"RPM: {int(rpm1)}/{int(rpm2)}/{int(rpm3)}")

        except Exception as e:
            print("\n模擬結束:", e)
        finally:
            if p.isConnected():
                p.disconnect()


if __name__ == "__main__":
    sim = HornetSim("robot.urdf")
    sim.run()
