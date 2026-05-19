import numpy as np

# ===== Kalibr output: T_cam_imu (imu -> camera) =====
T_cam_imu = np.array([
    [ 0.01120139,  0.99992559, -0.00471856,  0.07438326],
    [ 0.99988601, -0.01120139,  0.01012409, -0.00222439],
    [ 0.01007049, -0.00483191, -0.99993762, -0.00878880],
    [ 0.0,         0.0,         0.0,         1.0]
])

# ===== inverse transform =====
T_imu_cam = np.linalg.inv(T_cam_imu)

np.set_printoptions(precision=12, suppress=True)

print("=== T_cam_imu (imu -> camera) ===")
print(T_cam_imu)

print("\n=== T_imu_cam (camera -> imu) ===")
print(T_imu_cam)

# ===== manual verification =====
R = T_cam_imu[:3, :3]
t = T_cam_imu[:3, 3]

R_inv = R.T
t_inv = -R.T @ t

print("\n=== Manual inverse ===")
print("R_inv =")
print(R_inv)

print("\nt_inv =")
print(t_inv)

# ===== consistency check =====
I = T_cam_imu @ T_imu_cam

print("\n=== T * T_inv ===")
print(I)