import numpy as np
import matplotlib.pyplot as plt
from math import sin, pi
from kinematics import Kinematics


# Home positions (x, y, z) for each leg: LF, LB, RB, RF
home_positions = {
    0: np.array([ 0.3,  0.2, -0.51]),  # LF
    1: np.array([-0.3,  0.2, -0.51]),  # LB
    2: np.array([-0.3, -0.2, -0.51]),  # RB
    3: np.array([ 0.3, -0.2, -0.51])   # RF
}


def foot_trajectory(phase, step_length=0.06, step_height=0.03):
    beta = 0.5
    if phase < (1 - beta):  # Swing
        swing_phase = phase / (1 - beta)
        x = (swing_phase - 0.5) * step_length
        z = step_height * sin(pi * swing_phase)
    else:  # Stance
        stance_phase = (phase - (1 - beta)) / beta
        x = (0.5 - stance_phase) * step_length
        z = 0.0
    return np.array([x, 0.0, z])


def generate_gait_trajectories(duration=1.0, dt=0.01):
    kin = Kinematics()
    num_steps = int(duration / dt)
    trajectories = {leg: [] for leg in range(4)}
    phase_offsets = {0: 0.0, 1: 0.5, 2: 0.0, 3: 0.5}

    for step in range(num_steps):
        t = step * dt
        for leg in range(4):
            phase = (t + phase_offsets[leg]) % 1.0
            foot_offset = foot_trajectory(phase)
            foot_pos = home_positions[leg] + foot_offset
            joint_angles = kin.leg_IK(foot_pos, legID = leg)[:3]
            trajectories[leg].append((foot_pos, phase , joint_angles))

    return trajectories


def visualize_leg_gaits(trajectories):
    labels = {0: "Left Front (LF)", 1: "Left Back (LB)", 2: "Right Back (RB)", 3: "Right Front (RF)"}
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    axs = axs.flatten()

    for leg in range(4):
        traj_data = trajectories[leg]
        xs = []
        zs = []
        colors = []

        for pos, phase , angles in traj_data:
            xs.append(pos[0])
            zs.append(pos[2])
            if phase < 0.5:
                colors.append('blue')  # Swing
            else:
                colors.append('orange')  # Stance

        axs[leg].scatter(xs, zs, c=colors, s=10)
        axs[leg].invert_yaxis()
        axs[leg].set_title(labels[leg])
        axs[leg].set_xlabel("X (Forward)")
        axs[leg].set_ylabel("Z (Height)")
        axs[leg].grid(True)

    plt.suptitle("Trotting Gait - Foot Trajectories (Swing: Blue, Stance: Orange)", fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()


if __name__ == "__main__":
    trajs = generate_gait_trajectories(duration=1.0, dt=0.01)
    visualize_leg_gaits(trajs)

    for leg in range(4):
        print(f"\nTrajectory for leg {leg} ({['LF','LB','RB','RF'][leg]}):")
        for i, (pos, phase, angles) in enumerate(trajs[leg]):
            x, y, z = pos
            hip, thigh, knee = angles

            if leg == 1 or leg == 2 :
                thigh = np.pi*3/2 - thigh - 0.1
            # angles [1] = angles[1]
        
            if leg == 0 or leg == 3 :
                thigh = np.pi*3/2 - thigh + 0.8
                # angles[1] = angles[1]

            if leg == 1 or leg == 0 : 
                hip = hip - 0.18 
            if leg == 2 or leg == 3 : 
                hip = hip - 2.9

            knee = -(0.5 + knee)
            print(f"Step {i:3d}: x = {x:.3f}, y = {y:.3f}, z = {z:.3f}")
            print(f"Step {i:3d}: Hip = {hip:6.2f}°, Thigh = {thigh:6.2f}°, Knee = {knee:6.2f}°")
    


