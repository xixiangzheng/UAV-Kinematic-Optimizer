import numpy as np
import math
import matplotlib.pyplot as plt
import os

def run_visualization():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.normpath(os.path.join(current_dir, '..', 'data', 'problem5-simulator-strategy.csv'))

    try:
        raw_data = np.genfromtxt(csv_path, delimiter=',', dtype=None, encoding='utf-8')
        final_params = np.array([list(row)[1:] for row in raw_data], dtype=float)
        print(f"Successfully loaded parameters from: {csv_path}") 
    except FileNotFoundError:
        print(f"Error: The file was not found at {csv_path}")
        return
    except Exception as e:
        print(f"Error: An unexpected error occurred while reading the CSV: {e}")
        return
    
    uav_task_counts = np.array([3, 3, 3, 3, 3])

    G = np.array([0.0, 0.0, -9.8])
    MISSILE_VELOCITY_SCALAR = 300.0
    MISSILE_INITIAL_POS = np.array([
        [20000.0, 0.0, 2000.0],
        [19000.0, 600.0, 2100.0],
        [18000.0, -600.0, 1900.0]
    ])
    UAV_INITIAL_POS = np.array([
        [17800.0, 0.0, 1800.0],
        [12000.0, 1400.0, 1400.0],
        [6000.0, -3000.0, 700.0],
        [11000.0, 2000.0, 1800.0],
        [13000.0, -2000.0, 1300.0]
    ])
    FAKE_TARGET_POS = np.array([0.0, 0.0, 0.0])
    SMOKE_CLOUD_SINK_VELOCITY = np.array([0.0, 0.0, -3.0])
    SMOKE_EFFECTIVE_DURATION = 20.0
    final_params = np.array([
        [140.00, 3.1354, 0.00, 3.61, 3.65, 5.38, 1.92, 6.05],
        [139.75, 3.7525, 6.08, 6.67, 2.02, 8.75, 4.21, 8.81],
        [110.12, 2.2689, 27.22, 8.16, 1.26, 8.51, 1.43, 8.19],
        [138.25, 3.7529, 8.05, 13.07, 2.48, 14.01, 3.51, 14.07],
        [139.31, 2.4435, 13.26, 5.15, 2.50, 6.94, 4.49, 5.80]
    ])

    MISSILE_VELOCITY_VEC = np.zeros((3, 3))
    for i in range(3):
        direction = FAKE_TARGET_POS - MISSILE_INITIAL_POS[i]
        MISSILE_VELOCITY_VEC[i] = (direction / np.linalg.norm(direction)) * MISSILE_VELOCITY_SCALAR

    time_step = 0.1
    simulation_time = 60
    time_points = np.arange(0, simulation_time, time_step)

    total_bombs = int(np.sum(uav_task_counts))
    missile_trajectories = np.zeros((3, len(time_points), 3))
    uav_trajectories = np.full((5, len(time_points), 3), np.nan)
    bomb_trajectories = np.full((total_bombs, len(time_points), 3), np.nan)
    smoke_trajectories = np.full((total_bombs, len(time_points), 3), np.nan)
    
    det_times = np.zeros(total_bombs)
    det_pos = np.zeros((total_bombs, 3))
    drop_times_all = np.zeros(total_bombs)
    drop_pos_all = np.zeros((total_bombs, 3))
    uav_vel_all = np.zeros((total_bombs, 3))
    
    bomb_idx = 0
    for i in range(5):
        if uav_task_counts[i] == 0:
            continue
            
        uav_speed, uav_fly_angle_rad, t_drop1, delay1, delta_t_drop2, delay2, delta_t_drop3, delay3 = final_params[i]
        uav_fly_direction = np.array([math.cos(uav_fly_angle_rad), math.sin(uav_fly_angle_rad), 0.0])
        uav_velocity = uav_fly_direction * uav_speed
        drop_times = np.array([t_drop1, t_drop1 + delta_t_drop2, t_drop1 + delta_t_drop2 + delta_t_drop3])
        delays = np.array([delay1, delay2, delay3])
        
        for j in range(uav_task_counts[i]):
            drop_time = drop_times[j]
            drop_pos = UAV_INITIAL_POS[i] + uav_velocity * drop_time
            det_time = drop_time + delays[j]

            drop_times_all[bomb_idx] = drop_time
            drop_pos_all[bomb_idx] = drop_pos
            det_times[bomb_idx] = det_time
            det_pos[bomb_idx] = drop_pos + uav_velocity * delays[j] + G * 0.5 * delays[j] ** 2
            uav_vel_all[bomb_idx] = uav_velocity
            bomb_idx += 1

    for t_idx, t in enumerate(time_points):
        for i in range(3):
            missile_trajectories[i, t_idx, :] = MISSILE_INITIAL_POS[i] + MISSILE_VELOCITY_VEC[i] * t
        
        for i in range(5):
            if uav_task_counts[i] > 0:
                uav_speed, uav_fly_angle_rad, _, _, _, _, _, _ = final_params[i]
                uav_fly_direction = np.array([math.cos(uav_fly_angle_rad), math.sin(uav_fly_angle_rad), 0.0])
                uav_velocity = uav_fly_direction * uav_speed
                uav_trajectories[i, t_idx, :] = UAV_INITIAL_POS[i] + uav_velocity * t
        
        for b in range(total_bombs):
            if drop_times_all[b] <= t < det_times[b]:
                time_since_drop = t - drop_times_all[b]
                bomb_trajectories[b, t_idx, :] = drop_pos_all[b] + uav_vel_all[b] * time_since_drop + 0.5 * G * time_since_drop**2
            if det_times[b] <= t < det_times[b] + SMOKE_EFFECTIVE_DURATION:
                time_since_det = t - det_times[b]
                smoke_trajectories[b, t_idx, :] = det_pos[b] + SMOKE_CLOUD_SINK_VELOCITY * time_since_det

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot([], [], color='blue', linestyle='-', label='Missile Traj.')
    ax.plot([], [], color='orange', linestyle='--', label='UAV Traj.')
    ax.plot([], [], color='green', linestyle='-.', label='Bomb Traj. (pre-detonation)')
    ax.plot([], [], color='gray', linestyle=':', label='Smoke Cloud Traj.')
    
    for i in range(3):
        ax.plot(missile_trajectories[i, :, 0], missile_trajectories[i, :, 1], missile_trajectories[i, :, 2], color='blue', linestyle='-')
    
    for i in range(5):
        if uav_task_counts[i] > 0:
            ax.plot(uav_trajectories[i, :, 0], uav_trajectories[i, :, 1], uav_trajectories[i, :, 2], color='orange', linestyle='--')

    for i in range(total_bombs):
        ax.plot(bomb_trajectories[i, :, 0], bomb_trajectories[i, :, 1], bomb_trajectories[i, :, 2], color='green', linestyle='-.', alpha=0.9)
        ax.plot(smoke_trajectories[i, :, 0], smoke_trajectories[i, :, 1], smoke_trajectories[i, :, 2], color='gray', linestyle=':', alpha=0.8)

    ax.scatter(MISSILE_INITIAL_POS[:, 0], MISSILE_INITIAL_POS[:, 1], MISSILE_INITIAL_POS[:, 2], marker='x', s=100, label='Missile Start')
    
    active_uav_indices = np.where(uav_task_counts > 0)[0]
    if len(active_uav_indices) > 0:
        ax.scatter(UAV_INITIAL_POS[active_uav_indices, 0], UAV_INITIAL_POS[active_uav_indices, 1], UAV_INITIAL_POS[active_uav_indices, 2], marker='o', s=100, label='UAV Start')
    
    if total_bombs > 0:
        ax.scatter(drop_pos_all[:, 0], drop_pos_all[:, 1], drop_pos_all[:, 2], c='cyan', marker='v', s=50, label='Bomb Drop Point')
        ax.scatter(det_pos[:, 0], det_pos[:, 1], det_pos[:, 2], c='purple', marker='^', s=50, label='Smoke Detonation')

    ax.scatter(FAKE_TARGET_POS[0], FAKE_TARGET_POS[1], FAKE_TARGET_POS[2], c='red', marker='*', s=200, label='Target')

    ax.set_xlabel('X coordinate (m)')
    ax.set_ylabel('Y coordinate (m)')
    ax.set_zlabel('Z coordinate (m)')
    ax.set_title('Full Simulation: Missile, UAV, Bomb, and Smoke Trajectories')
    ax.legend(loc='center left', bbox_to_anchor=(1.0, 0.5))
    ax.grid(True)

    fig.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_visualization()