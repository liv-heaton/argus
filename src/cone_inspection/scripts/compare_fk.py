"""

Usage:
    python3 compare_fk.py --joints 0 -1.57 1.57 -1.57 -1.57 0
    python3 compare_fk.py --joints 0 -1.57 1.57 -1.57 -1.57 0 --urdf /tmp/ur3_cell.urdf
"""
import argparse
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3


def fk_stock_model(q):
    robot = rtb.models.UR3()
    T = robot.fkine(q)
    return T


def fk_from_urdf(q, urdf_path, ee_link=None):
    robot = rtb.ERobot.URDF(urdf_path)
    if ee_link:
        T = robot.fkine(q, end=ee_link)
    else:
        T = robot.fkine(q)
    return T, robot


def pretty(T: SE3, label: str):
    p = T.t
    rpy = T.rpy(unit='deg')
    print(f"{label}")
    print(f"  xyz (m)   : {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    print(f"  rpy (deg) : {rpy[0]:.2f} {rpy[1]:.2f} {rpy[2]:.2f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--joints", nargs=6, type=float, required=True,
                     help="6 joint angles in radians, base to wrist3")
    ap.add_argument("--urdf", type=str, default=None,
                     help="Path to the EXPANDED urdf (see README note below)")
    ap.add_argument("--ee-link", type=str, default="tool0",
                     help="Link to compute FK to when using --urdf")
    args = ap.parse_args()
    q = np.array(args.joints)

    T_stock = fk_stock_model(q)
    pretty(T_stock, "Toolbox stock UR3 model -> tool frame")

    if args.urdf:
        T_urdf, robot = fk_from_urdf(q, args.urdf, args.ee_link)
        pretty(T_urdf, f"Toolbox URDF model -> {args.ee_link}")
        print("\nLink order the toolbox expects q in:")
        print(" ", [l.name for l in robot.links if l.isjoint])

    print("\nNow compare against RViz/TF for the SAME joint values:")
    print("  ros2 run tf2_ros tf2_echo world tool0")
    print("(Set the sliders in joint_state_publisher_gui to these radian values first.)")
