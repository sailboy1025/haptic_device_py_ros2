#!/usr/bin/env python3

import asyncio
import websockets
import orjson
import numpy as np
import csv
import time
import os
import matplotlib.pyplot as plt

# ========== 1. Read Position Data via WebSocket ==========


async def record_positions(filename, duration_sec=20, loop_time=0.001):
    uri = 'ws://localhost:10001'
    record_position = []
    force = {"x": 0, "y": 0, "z": 0}
    print(f"Connecting to WebSocket server {uri}...")
    async with websockets.connect(uri) as ws:
        print(
            f"Connected. Recording for {duration_sec} seconds to {filename}...")
        start_time = time.time()

        while time.time() - start_time < duration_sec:
            try:
                response = await asyncio.wait_for(ws.recv(), timeout=2.0)
            except asyncio.TimeoutError:
                print("WebSocket timeout! No new message within 2 seconds.")
                break
            data = orjson.loads(response)
            inverse3_devices = data.get("inverse3", [])
            inverse3_data = inverse3_devices[0] if inverse3_devices else {}
            inverse3_device_id = inverse3_data.get("device_id")
            if not inverse3_devices:
                print('No device')
                continue

            pos = inverse3_devices[0]["state"]["cursor_position"]
            position = [pos.get("x", 0.0), pos.get(
                "y", 0.0), pos.get("z", 0.0)]
            record_position.append(position)
            print(position)
            # Must send forces to receive state updates (even if forces are 0)
            request_msg = {
                "inverse3": [
                    {
                        "device_id": inverse3_device_id,
                        "commands": {
                            "set_cursor_force": {
                                "values": force
                            }
                        }
                    }
                ]
            }

            # Send the force command message to the server
            await ws.send(orjson.dumps(request_msg))
            await asyncio.sleep(loop_time)

    # Save to CSV
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "z"])
        for p in record_position:
            writer.writerow(p)

    print(f"Data saved to {filename}")
    return np.array(record_position)


# ========== 2. Plane Fitting ==========
def fit_plane(points):
    A = np.c_[points[:, 0], points[:, 1], np.ones(points.shape[0])]
    C, _, _, _ = np.linalg.lstsq(A, points[:, 2], rcond=None)
    normal = np.array([-C[0], -C[1], 1.0])
    normal /= np.linalg.norm(normal)
    return normal

# ========== 3. Calibration Process ==========


def perform_calibration(xy_data, yz_data):
    normal_xy = fit_plane(xy_data)
    normal_yz = fit_plane(yz_data)

    # Normalize the normal vectors
    x_axis = -normal_yz[:3] / np.linalg.norm(normal_yz[:3])
    z_axis = normal_xy[:3] / np.linalg.norm(normal_xy[:3])

    # Compute y_axis via cross product to ensure orthogonality
    y_axis = np.cross(x_axis, z_axis)
    y_axis /= np.linalg.norm(y_axis)

    # Re-orthogonalize x to be perpendicular to y and z (just in case)
    x_axis = np.cross(y_axis, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    R_device_frame = np.column_stack((x_axis, y_axis, z_axis))

    return R_device_frame

# ========== 4. Plotting Helper ==========


def plot_3d_data(xy_data, yz_data):
    fig = plt.figure(figsize=(12, 6))

    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter(xy_data[:, 0], xy_data[:, 1], xy_data[:, 2], s=2)
    ax1.set_title('XY Plane Data')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')

    ax2 = fig.add_subplot(122, projection='3d')
    ax2.scatter(yz_data[:, 0], yz_data[:, 1], yz_data[:, 2], s=2, color='r')
    ax2.set_title('YZ Plane Data')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')

    plt.tight_layout()
    plt.show()


def plot_frame_comparison(R_misaligned):
    R_world = np.eye(3)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    origin = np.array([0, 0, 0])
    colors = ['r', 'g', 'b']
    labels = ['X', 'Y', 'Z']

    # Draw world frame (solid lines)
    for i in range(3):
        ax.quiver(*origin, *R_world[:, i], color=colors[i],
                  length=0.1, normalize=True, label=f'World {labels[i]}')

    # Draw calibrated frame (dashed lines)
    for i in range(3):
        ax.quiver(*origin, *R_misaligned[:, i], color=colors[i], length=0.1,
                  linestyle='dashed', alpha=0.5, label=f'Calibrated {labels[i]}')

    ax.set_xlim([-0.15, 0.15])
    ax.set_ylim([-0.15, 0.15])
    ax.set_zlim([-0.15, 0.15])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("World Frame (solid) vs. Calibrated Frame (dashed)")
    ax.legend()
    plt.tight_layout()
    plt.show()
# ========== 5. Main ==========


async def main():
    os.makedirs('calibration_logs', exist_ok=True)
    dir_path = os.path.dirname(os.path.abspath(__file__))
    pkg_root = os.path.abspath(os.path.join(dir_path, '..'))
    dir_path_device_cal = os.path.join(pkg_root, 'device_cal')
    # make it if it doesn't exist
    os.makedirs(dir_path_device_cal, exist_ok=True)
    log_xy = 'calibration_logs/position_log_xy.csv'
    log_yz = 'calibration_logs/position_log_yz.csv'

    input("Move device in XY plane, then press Enter to start logging...")
    await record_positions(log_xy, duration_sec=10)

    input("Move device in YZ plane, then press Enter to start logging...")
    await record_positions(log_yz, duration_sec=10)

    # Load recorded CSVs
    xy_data = np.loadtxt(log_xy, delimiter=',', skiprows=1)
    yz_data = np.loadtxt(log_yz, delimiter=',', skiprows=1)

    # Plot to visually verify
    plot_3d_data(xy_data, yz_data)

    # Perform calibration
    rotation_matrix = perform_calibration(xy_data, yz_data)

    print("Calibration rotation matrix:")
    print(rotation_matrix)
    plot_frame_comparison(rotation_matrix)

    # Save to JSON using orjson
    with open(os.path.join(dir_path_device_cal, 'inverse3_cali_param.json'), 'wb') as f:
        f.write(orjson.dumps(rotation_matrix.tolist(), option=orjson.OPT_INDENT_2))

    print("Calibration matrix saved to inverse3_cali_param.json")

def run():
    # Run the main function
    asyncio.run(main())

if __name__ == "__main__":
    run()
