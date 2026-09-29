import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath.base import angvec2r
 
# ---- fixed offsets, must match the xacro -----------------------------------
# tool0 -> gopro_optical_frame, taken from the fixed joints in ur3_cell.urdf.xacro
T_TOOL0_GOPRO = SE3(0.055 + 0.0355, 0, 0) * SE3.RPY([-90, 0, -90], unit='deg', order='xyz')
 
 
def load_robot(urdf_path, ee_link="tool0"):
    robot = rtb.ERobot.URDF(urdf_path)
    robot.ee_link = robot.link_dict[ee_link] if ee_link else robot.ee_links[0]
    return robot
 
 
# ---- geometry ----------------------------------------------------------------
 
def look_at(eye, target, up=(0, 0, 1)):
    """SE3 with origin at `eye`, +z axis pointing at `target` (camera-optical convention)."""
    eye = np.asarray(eye, dtype=float)
    target = np.asarray(target, dtype=float)
    up = np.asarray(up, dtype=float)
 
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-6:            # eye is directly above/below target
        up = np.array([1.0, 0, 0])
        x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
 
    R = np.column_stack((x, y, z))
    return SE3.Rt(R, eye)
 
 
def orbit_poses(cone_pos, radius, height, n, start_deg=0, end_deg=360):
    """Camera poses on a horizontal circle around the cone, looking at cone_pos.
    Returns a list of SE3 (optical-frame targets), evenly spaced in angle."""
    cone_pos = np.asarray(cone_pos, dtype=float)
    angles = np.linspace(np.radians(start_deg), np.radians(end_deg), n,
                          endpoint=(abs(end_deg - start_deg) < 360))
    poses = []
    for a in angles:
        eye = cone_pos + np.array([radius * np.cos(a), radius * np.sin(a), height])
        poses.append(look_at(eye, cone_pos))
    return poses
 
 
# ---- IK over a sequence, seeded from the previous solution -------------------
 
def solve_sequence(robot, optical_targets, q_seed, mask=None):
    """Solve IK for each target, using the previous solution as the seed for
    the next one -- keeps the arm from jumping between IK branches.
    optical_targets: SE3 poses for gopro_optical_frame.
    Returns (q_list, ok_list)."""
    q_list, ok_list = [], []
    q = np.asarray(q_seed, dtype=float)
    for T_opt in optical_targets:
        T_tool0 = T_opt * T_TOOL0_GOPRO.inv()          # optical target -> tool0 target
        sol = robot.ikine_LM(T_tool0, q0=q, mask=mask, joint_limits=True, end="tool0")
        ok_list.append(bool(sol.success))
        if sol.success:
            q = sol.q                                   # seed next solve with this one
        q_list.append(q.copy())
    return q_list, ok_list
 
 
# ---- time parameterisation -----------------------------------------------
 
def jtraj_segments(q_waypoints, samples_per_segment=50):
    """Smooth joint-space motion through a list of key configurations."""
    traj = []
    for a, b in zip(q_waypoints[:-1], q_waypoints[1:]):
        seg = rtb.jtraj(a, b, samples_per_segment)
        traj.append(seg.q)
    return np.vstack(traj)
 
 
# ---- example end-to-end plan -----------------------------------------------
 
def build_inspection_plan(robot, q_home, cone_pos,
                           grasp_pose_tool0, lift_height=0.15,
                           orbit_radius=0.25, orbit_height=0.30, orbit_points=12):
    """
    1. home -> grasp pose (gopro pickup)
    2. grasp -> lifted
    3. lifted -> first orbit point -> around the orbit, always looking at the cone
    Returns a single Nx6 joint trajectory ready to publish as /joint_states.
    """
    q = np.asarray(q_home, dtype=float)
 
    sol_grasp = robot.ikine_LM(grasp_pose_tool0, q0=q, joint_limits=True, end="tool0")
    assert sol_grasp.success, "IK failed for grasp pose"
    q_grasp = sol_grasp.q
 
    T_lift = SE3(0, 0, lift_height) * grasp_pose_tool0
    sol_lift = robot.ikine_LM(T_lift, q0=q_grasp, joint_limits=True, end="tool0")
    assert sol_lift.success, "IK failed for lift pose"
    q_lift = sol_lift.q
 
    orbit_targets = orbit_poses(cone_pos, orbit_radius, orbit_height, orbit_points)
    q_orbit, ok = solve_sequence(robot, orbit_targets, q_seed=q_lift)
    n_fail = sum(not o for o in ok)
    if n_fail:
        print(f"WARNING: {n_fail}/{len(ok)} orbit IK solves failed -- "
              f"check radius/height are reachable, or re-seed those points.")
 
    key_confs = [q, q_grasp, q_lift, *q_orbit]
    return jtraj_segments(key_confs)
 
 
if __name__ == "__main__":
    import sys
    urdf_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/ur3_cell.urdf"
    robot = load_robot(urdf_path)
 
    q_home = [0, -1.57, 1.57, -1.57, -1.57, 0]
    cone_pos = [0.5, 0.0, 0.0]                 # cone base, world frame, metres
    grasp = SE3(0.3, -0.2, 0.05) * SE3.RPY([180, 0, 0], unit='deg')
 
    traj = build_inspection_plan(robot, q_home, cone_pos, grasp)
    print(f"Trajectory: {traj.shape[0]} samples x {traj.shape[1]} joints")
 
