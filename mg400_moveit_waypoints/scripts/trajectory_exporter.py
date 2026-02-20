#!/usr/bin/env python3
"""
MG400 Trajectory Exporter - JOINT ANGLE VERSION
Listens to MoveIt2 planned paths and exports joint angles to JSON.

Usage:
    Terminal 1: ros2 launch mg400_moveit_waypoints demo.launch.py
    Terminal 2: python3 trajectory_exporter.py
    
    Then plan a path in RViz and press 's' to save.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
import json
import os
import math
import sys
import select
import termios
import tty
from datetime import datetime
from typing import List, Dict, Optional


class TrajectoryExporter(Node):
    """Listens to MoveIt2 trajectories and exports joint angles to JSON"""
    
    def __init__(self, output_dir: str = "waypoints"):
        super().__init__('trajectory_exporter')
        
        self.output_dir = output_dir
        self.latest_trajectory = None
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        
        # Subscribe to MoveIt2's planned path topic
        self.subscription = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.trajectory_callback,
            10
        )
        
        self.print_header()
    
    def print_header(self):
        print("\n" + "=" * 55)
        print("   MG400 TRAJECTORY EXPORTER (Joint Angles)")
        print("=" * 55)
        print(f"  Output folder: {self.output_dir}/")
        print("  Mode: Joint angles (j1, j2, j3, j4)")
        print("=" * 55)
        print("\n  WORKFLOW:")
        print("  1. Plan a path in RViz (drag marker → 'Plan')")
        print("  2. Press 's' here to save")
        print("  3. Run: python3 mg400_executor.py <file> --servo --servo-t 0.2")
        print("\n  KEYS:  s=Save  p=Preview  i=Info  q=Quit")
        print("=" * 55 + "\n")
    
    def trajectory_callback(self, msg):
        self.latest_trajectory = msg
        total = sum(len(t.joint_trajectory.points) for t in msg.trajectory)
        print(f"[RECEIVED] Trajectory with {total} waypoints ✓")
        print("           Press 's' to save\n")
    
    def extract_waypoints(self, decimate: int = 1) -> List[Dict]:
        """
        Extract joint angle waypoints from the latest trajectory.
        
        Args:
            decimate: Keep every Nth point (1=all, 5=every 5th, etc.)
        
        Returns: List of waypoints with j1, j2, j3, j4 in DEGREES
        """
        if not self.latest_trajectory:
            return []
        
        waypoints = []
        
        for traj in self.latest_trajectory.trajectory:
            joint_traj = traj.joint_trajectory
            joint_names = joint_traj.joint_names
            
            # Create mapping from joint name to index
            joint_map = {}
            for i, name in enumerate(joint_names):
                if name == 'j1':
                    joint_map['j1'] = i
                elif name == 'j2':
                    joint_map['j2'] = i
                elif name == 'j3':
                    joint_map['j3'] = i
                elif name == 'j4':
                    joint_map['j4'] = i
            print(f"\n[DEBUG] Joint names from MoveIt2: {joint_names}")
            print(f"[DEBUG] Joint map created: {joint_map}")
            if len(joint_map) < 4:
                print(f"[WARNING] Only mapped {len(joint_map)} joints! Using fallback indices.")
                
            for point in joint_traj.points:
                positions = point.positions
                
                # Extract joint angles and convert to degrees
                wp = {
                    'j1': round(math.degrees(positions[joint_map.get('j1', 0)]), 2),
                    'j2': round(math.degrees(positions[joint_map.get('j2', 1)]), 2),
                    'j3': round(math.degrees(positions[joint_map.get('j3', 2)]), 2),
                    'j4': round(math.degrees(positions[joint_map.get('j4', 3)]), 2),
                    'timestamp': point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                }
                
                waypoints.append(wp)
        
        # Decimate waypoints
        if decimate > 1 and len(waypoints) > 2:
            decimated = [waypoints[0]]  # Always keep first
            for i in range(decimate, len(waypoints) - 1, decimate):
                decimated.append(waypoints[i])
            decimated.append(waypoints[-1])  # Always keep last
            return decimated
        
        return waypoints
    
    def save_trajectory(self, decimate: int = 1) -> bool:
        """Save trajectory to JSON file."""
        if not self.latest_trajectory:
            print("[ERROR] No trajectory! Plan a path first.\n")
            return False
        
        # Get all waypoints first to show raw count
        all_waypoints = self.extract_waypoints(decimate=1)
        waypoints = self.extract_waypoints(decimate=decimate)
        
        if not waypoints:
            print("[ERROR] No waypoints extracted!\n")
            return False
        
        # Generate filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"trajectory_{timestamp}.json"
        
        # Build output data
        output = {
            "metadata": {
                "created": datetime.now().isoformat(),
                "robot": "MG400",
                "mode": "joint_angles",
                "raw_waypoints": len(all_waypoints),
                "saved_waypoints": len(waypoints),
                "decimation": decimate,
                "units": "degrees",
                "usage": "python3 mg400_executor.py <this_file> --cp 50 --speed 30"
            },
            "waypoints": waypoints
        }
        
        # Save to file
        filepath = os.path.join(self.output_dir, filename)
        with open(filepath, 'w') as f:
            json.dump(output, f, indent=2)
        
        print(f"\n[SAVED] {len(all_waypoints)} raw → {len(waypoints)} waypoints")
        print(f"        File: {filepath}")
        print(f"[NEXT]  python3 mg400_executor.py {filepath} --cp 50 --speed 30\n")
        
        return True
    
    def preview_waypoints(self):
        """Print preview of current waypoints"""
        if not self.latest_trajectory:
            print("[ERROR] No trajectory yet!\n")
            return
        
        waypoints = self.extract_waypoints(decimate=5)
        
        print(f"\n{'─' * 60}")
        print(f"  PREVIEW ({len(waypoints)} waypoints) - Joint Angles in Degrees")
        print(f"{'─' * 60}")
        print(f"  {'#':<4} {'J1':>10} {'J2':>10} {'J3':>10} {'J4':>10}")
        print(f"{'─' * 60}")
        
        for i, wp in enumerate(waypoints):
            print(f"  {i+1:<4} {wp['j1']:>10.2f} {wp['j2']:>10.2f} "
                  f"{wp['j3']:>10.2f} {wp['j4']:>10.2f}")
        
        print(f"{'─' * 60}")
        
        # Show joint limits check
        print(f"\n  MG400 Joint Limits:")
        print(f"  J1: ±160°  J2: -25° to +85°  J3: -25° to +105°  J4: ±180°")
        
        # Check for any violations
        violations = []
        for i, wp in enumerate(waypoints):
            if abs(wp['j1']) > 160: violations.append(f"Point {i+1}: J1={wp['j1']:.1f}°")
            if wp['j2'] < -25 or wp['j2'] > 85: violations.append(f"Point {i+1}: J2={wp['j2']:.1f}°")
            if wp['j3'] < -25 or wp['j3'] > 105: violations.append(f"Point {i+1}: J3={wp['j3']:.1f}°")
            if abs(wp['j4']) > 180: violations.append(f"Point {i+1}: J4={wp['j4']:.1f}°")
        
        if violations:
            print(f"\n  ⚠️  WARNINGS - These may exceed limits:")
            for v in violations[:5]:  # Show first 5
                print(f"      {v}")
        else:
            print(f"\n  ✓ All waypoints within joint limits")
        
        print()
    
    def show_info(self):
        """Show info about current trajectory"""
        if not self.latest_trajectory:
            print("[INFO] No trajectory yet.\n")
            return
        
        all_wp = self.extract_waypoints(decimate=1)
        dec_wp = self.extract_waypoints(decimate=5)
        
        print(f"\n  Raw waypoints: {len(all_wp)}")
        print(f"  After decimation (5): {len(dec_wp)}")
        
        if all_wp:
            first = all_wp[0]
            last = all_wp[-1]
            print(f"\n  Start: J1={first['j1']:.1f}° J2={first['j2']:.1f}° "
                  f"J3={first['j3']:.1f}° J4={first['j4']:.1f}°")
            print(f"  End:   J1={last['j1']:.1f}° J2={last['j2']:.1f}° "
                  f"J3={last['j3']:.1f}° J4={last['j4']:.1f}°\n")
    
    def run_interactive(self):
        """Run in interactive mode with keyboard input"""
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1).lower()
                    
                    if key == 's':
                        self.save_trajectory()
                    elif key == 'p':
                        self.preview_waypoints()
                    elif key == 'i':
                        self.show_info()
                    elif key == 'q':
                        print("\n[EXIT] Goodbye!\n")
                        break
        
        except KeyboardInterrupt:
            print("\n[EXIT] Interrupted.\n")
        
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    rclpy.init()
    exporter = TrajectoryExporter(output_dir="waypoints")
    
    try:
        exporter.run_interactive()
    finally:
        exporter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    