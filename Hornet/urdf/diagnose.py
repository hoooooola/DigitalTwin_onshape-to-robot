import pybullet as p
import pybullet_data
import math
import sys

print("啟動診斷腳本...")
sys.stdout.flush()

c = p.connect(p.DIRECT)
print(f"PyBullet connected: client={c}")
sys.stdout.flush()

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF('plane.urdf')
drone = p.loadURDF('robot.urdf', [0, 0, 0.05], p.getQuaternionFromEuler([0, 0, 0]))
print(f"Drone loaded: id={drone}")
sys.stdout.flush()

joints = {}
for i in range(p.getNumJoints(drone)):
    n = p.getJointInfo(drone, i)[1].decode()
    joints[n] = i

# ★ 讀取 PyBullet 真實的重心位置
dyn = p.getDynamicsInfo(drone, -1)
local_com = dyn[3]
print(f"\n[PyBullet 真實 CoM (localInertiaPos)] = {local_com}")
print(f"  → CoM X={local_com[0]:.4f}, Y={local_com[1]:.4f}, Z={local_com[2]:.4f}")

# 出生時姿態
pos, orn = p.getBasePositionAndOrientation(drone)
e = p.getEulerFromQuaternion(orn)
print(f"\n[出生] RPY=({math.degrees(e[0]):.1f}°, {math.degrees(e[1]):.1f}°, {math.degrees(e[2]):.1f}°)  Z={pos[2]:.4f}m")

m1 = joints.get('motor_1_CW', -1)
m2 = joints.get('motor_2_CCW', -1)
m3 = joints.get('motor_3_CWW', -1)

for mn in ['motor_1_CW', 'motor_2_CCW', 'motor_3_CWW']:
    if mn in joints:
        wp = p.getLinkState(drone, joints[mn])[0]
        print(f"  {mn}: X={wp[0]:.4f}  Y={wp[1]:.4f}  Z={wp[2]:.4f}")

sys.stdout.flush()

# ★ Binary Search：找到讓 Pitch≈0 的最佳後馬達推力
print("\n--- 正在搜尋平衡推力 ---")
total_thrust = 2.25 * 9.81  # 22.07N

lo, hi = 0.0, 10.0
for trial in range(15):
    rear_t = (lo + hi) / 2.0
    front_t = (total_thrust - rear_t) / 2.0

    # 重新載入飛機（每次試驗都要從零開始）
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.loadURDF('plane.urdf')
    drone2 = p.loadURDF('robot.urdf', [0, 0, 0.05], p.getQuaternionFromEuler([0, 0, 0]))
    j2 = {}
    for i in range(p.getNumJoints(drone2)):
        n = p.getJointInfo(drone2, i)[1].decode()
        j2[n] = i
    m1b = j2.get('motor_1_CW', -1)
    m2b = j2.get('motor_2_CCW', -1)
    m3b = j2.get('motor_3_CWW', -1)

    for _ in range(240):
        p.applyExternalForce(drone2, m1b, [0, 0, front_t], [0, 0, 0], p.WORLD_FRAME)
        p.applyExternalForce(drone2, m2b, [0, 0, front_t], [0, 0, 0], p.WORLD_FRAME)
        p.applyExternalForce(drone2, m3b, [0, 0, rear_t],  [0, 0, 0], p.WORLD_FRAME)
        p.stepSimulation()

    pos2, orn2 = p.getBasePositionAndOrientation(drone2)
    e2 = p.getEulerFromQuaternion(orn2)
    pitch_deg = math.degrees(e2[0])

    if pitch_deg < 0:   # 機頭往下 → 後馬達還是太強 → 減小 rear
        hi = rear_t
    else:               # 機頭往上 → 後馬達太弱 → 增大 rear
        lo = rear_t

    print(f"  Trial {trial+1:2d}: front={front_t:.3f}N rear={rear_t:.3f}N → Pitch={pitch_deg:.1f}°")
    sys.stdout.flush()

print(f"\n★ 最佳推力找到！")
print(f"  前馬達: {front_t:.3f} N  ← 每顆")
print(f"  後馬達: {rear_t:.3f} N")
import math as _m
print(f"  前方 RPM: {int(_m.sqrt(front_t / 2.6e-8))}")
print(f"  後方 RPM: {int(_m.sqrt(rear_t  / 2.6e-8))}")
print("\n診斷完成！")
p.disconnect()

