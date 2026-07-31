import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


# Shoutouts to my close personal friend Claude for this one
def shade_spans(ax, t, mask, color, label, alpha=0.18):
    """Shade the contiguous time spans where `mask` is True on axis `ax`.

    Only the first span carries the legend label so we get one entry, not one
    per region.
    """
    mask = np.asarray(mask, dtype=int)
    t = np.asarray(t)
    # +1 marks a False->True edge, -1 a True->False edge (padded so runs that
    # touch the start/end are still closed).
    edges = np.diff(np.concatenate(([0], mask, [0])))
    starts = np.flatnonzero(edges == 1)
    ends = np.flatnonzero(edges == -1)  # exclusive index, one past the run
    labeled = False
    for s, e in zip(starts, ends):
        right = t[e] if e < len(t) else t[-1]
        ax.axvspan(t[s], right, color=color, alpha=alpha, lw=0,
                   label=(None if labeled else label))
        labeled = True


df = pd.read_csv('circle_no_proc_noise.csv')

t = df['t']

# GPS was there but the filter refused it (out of tolerance / lockout window)
gps_lockout = (df['gps_avail'] == 1) & (df['gps_used'] == 0)
# GPS reported no data at all (a plain dropout, not a lockout)
gps_dropout = (df['gps_avail'] == 0)

# plots for x, y, vx, vy
fig1, ax1 = plt.subplots(2, 2, figsize=(8,8))

# x
ax1[0, 0].plot(t, df['x_true'], label='x_true', color='black', linestyle='-')
ax1[0, 0].plot(t, df['x_est'], label='x_est', color='green', linestyle='-')
ax1[0, 0].plot(t, df['gps_x'], label='gps_x', color='red', marker='.', linestyle='', alpha = 0.3)
ax1[0, 0].plot(t, df['lidar_x'], label='lidar_x', color='blue', marker='.', linestyle='', alpha = 0.3)
ax1[0, 0].set_xlabel('Time (s)')
ax1[0, 0].set_ylabel('Position (m)')
ax1[0, 0].set_title('x Position')
ax1[0, 0].legend()


# y
ax1[0, 1].plot(t, df['y_true'], label='y_true', color='black', linestyle='-')
ax1[0, 1].plot(t, df['y_est'], label='y_est', color='green', linestyle='-')
ax1[0, 1].plot(t, df['gps_y'], label='gps_y', color='red', marker='.', linestyle='', alpha = 0.3)
ax1[0, 1].plot(t, df['lidar_y'], label='lidar_y', color='blue', marker='.', linestyle='', alpha = 0.3)
ax1[0, 1].set_xlabel('Time (s)')
ax1[0, 1].set_ylabel('Position (m)')
ax1[0, 1].set_title('y Position')
ax1[0, 1].legend()

# vx
ax1[1, 0].plot(t, df['vx_true'], label='vx_true', color='black', linestyle='-')
ax1[1, 0].plot(t, df['vx_est'], label='vx_est', color='green', linestyle='-')
ax1[1, 0].plot(t, df['imu_vx'], label='imu_vx', color='red', marker='.', linestyle='', alpha = 0.3)
ax1[1, 0].set_xlabel('Time (s)')
ax1[1, 0].set_ylabel('Velocity (m/s)')
ax1[1, 0].set_title('x Velocity')
ax1[1, 0].legend()

# vy
ax1[1, 1].plot(t, df['vy_true'], label='vy_true', color='black', linestyle='-')
ax1[1, 1].plot(t, df['vy_est'], label='vy_est', color='green', linestyle='-')
ax1[1, 1].plot(t, df['imu_vy'], label='imu_vy', color='red', marker='.', linestyle='', alpha = 0.3)
ax1[1, 1].set_xlabel('Time (s)')
ax1[1, 1].set_ylabel('Velocity (m/s)')
ax1[1, 1].set_title('y Velocity')
ax1[1, 1].legend()


# Shade GPS lockout (and plain-dropout) regions on every subplot, then rebuild
# each legend so the shaded bands get an entry.
for ax in ax1.flat:
    shade_spans(ax, t, gps_lockout, 'orange', 'GPS locked out')
    shade_spans(ax, t, gps_dropout, 'gray', 'GPS unavailable')
    ax.legend(loc='best', fontsize=8)





# Errors
# plots for x, y, vx, vy
fig2, ax2 = plt.subplots(2, 2, figsize=(8,8))

# x
ax2[0, 0].plot(t, df['x_est'] - df['x_true'], label='x_est - x_true', color='blue', linestyle='-')
ax2[0, 0].set_xlabel('Time (s)')
ax2[0, 0].set_ylabel('Difference (m)')
ax2[0, 0].set_title('x Estimation Error')
ax2[0, 0].axhline(y=0, color='black', linestyle='--')
ax2[0, 0].legend()


# y
ax2[0, 1].plot(t, df['y_est'] - df['y_true'], label='y_est - y_true', color='blue', linestyle='-')
ax2[0, 1].set_xlabel('Time (s)')
ax2[0, 1].set_ylabel('Difference (m)')
ax2[0, 1].set_title('y Estimation Error')
ax2[0, 1].axhline(y=0, color='black', linestyle='--')
ax2[0, 1].legend()


# vx
ax2[1, 0].plot(t, df['vx_est'] - df['vx_true'], label='vx_est - vx_true', color='blue', linestyle='-')
ax2[1, 0].set_xlabel('Time (s)')
ax2[1, 0].set_ylabel('Difference (m/s)')
ax2[1, 0].set_title('vx Estimation Error')
ax2[1, 0].axhline(y=0, color='black', linestyle='--')
ax2[1, 0].legend()

# vy
ax2[1, 1].plot(t, df['vy_est'] - df['vy_true'], label='vy_est - vy_true', color='blue', linestyle='-')
ax2[1, 1].set_xlabel('Time (s)')
ax2[1, 1].set_ylabel('Difference (m/s)')
ax2[1, 1].set_title('vy Estimation Error')
ax2[1, 1].axhline(y=0, color='black', linestyle='--')
ax2[1, 1].legend()


fig3, ax3 = plt.subplots(1, 1, figsize=(8,8))
ax3.scatter(t, df['gps_y'] - df['y_true'], label='gps_y - y_true', color='red', alpha = 0.3, s=6)
ax3.scatter(t, df['lidar_y'] - df['y_true'], label='lidar_y - y_true', color='blue', alpha = 0.3, s=6)
ax3.plot(t, df['y_est'] - df['y_true'], label='y_est - y_true', color='green', linestyle='-')
ax3.axhline(y=0, color='black', linestyle='--', alpha = 0.7)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Difference (m)')
ax3.set_title('y measurement error vs estimation error')
ax3.legend()






plt.tight_layout()
plt.show()