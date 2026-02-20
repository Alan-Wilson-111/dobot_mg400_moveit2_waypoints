#!/usr/bin/env python3
"""
MG400 Trajectory Executor - JOINT ANGLE VERSION with CP Smoothing & ServoJ
Executes joint angle waypoints from JSON file on real MG400 robot.

IMPORTANT: MoveIt/URDF uses RELATIVE J3 (relative to link2).
           The real MG400 uses ABSOLUTE J3 (relative to ground).
           This executor converts: real_J3 = moveit_J3 + J2

Usage:
    python3 mg400_executor.py waypoints/trajectory.json --cp 80 --speed 30
    python3 mg400_executor.py waypoints/trajectory.json --servo
    python3 mg400_executor.py waypoints/trajectory.json --servo --servo-t 0.1
    python3 mg400_executor.py waypoints/trajectory.json --dry-run
"""

import socket
import json
import time
import argparse
import sys
from typing import List, Dict, Optional


class MG400:
    """MG400 TCP/IP Control Class"""
    
    JOINT_LIMITS = {
        'j1': (-160, 160),
        'j2': (-25, 85),
        'j3': (-25, 105),
        'j4': (-360, 360)
    }
    
    def __init__(self, ip: str = "192.168.1.6", dashboard_port: int = 29999, move_port: int = 30003):
        self.ip = ip
        self.dashboard_port = dashboard_port
        self.move_port = move_port
        self.dashboard_socket: Optional[socket.socket] = None
        self.move_socket: Optional[socket.socket] = None
    
    def connect(self) -> bool:
        try:
            self.dashboard_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.dashboard_socket.settimeout(5)
            self.dashboard_socket.connect((self.ip, self.dashboard_port))
            print(f"[OK] Dashboard port {self.dashboard_port} connected")
            
            self.move_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.move_socket.settimeout(10)
            self.move_socket.connect((self.ip, self.move_port))
            print(f"[OK] Move port {self.move_port} connected")
            
            time.sleep(0.5)
            self._clear_buffer(self.dashboard_socket)
            self._clear_buffer(self.move_socket)
            return True
        except Exception as e:
            print(f"[ERROR] Connection failed: {e}")
            return False
    
    def disconnect(self):
        if self.dashboard_socket:
            self.dashboard_socket.close()
        if self.move_socket:
            self.move_socket.close()
        print("[OK] Disconnected")
    
    def _clear_buffer(self, sock: socket.socket):
        sock.setblocking(False)
        try:
            while sock.recv(1024):
                pass
        except:
            pass
        sock.setblocking(True)
        sock.settimeout(10)
    
    def _send_dashboard(self, cmd: str) -> str:
        try:
            self.dashboard_socket.send(f"{cmd}\n".encode())
            time.sleep(0.1)
            return self.dashboard_socket.recv(1024).decode().strip()
        except Exception as e:
            return f"Error: {e}"
    
    def _send_move(self, cmd: str) -> str:
        try:
            self.move_socket.send(f"{cmd}\n".encode())
            time.sleep(0.05)
            return "OK"
        except Exception as e:
            return f"Error: {e}"

    def _send_move_no_delay(self, cmd: str) -> str:
        try:
            self.move_socket.setblocking(False)
            try:
                while self.move_socket.recv(1024):
                    pass
            except:
                pass
            self.move_socket.setblocking(True)
            self.move_socket.settimeout(10)
            
            self.move_socket.send(f"{cmd}\n".encode())
            return "OK"
        except Exception as e:
            return f"Error: {e}"
    
    # ========== Dashboard Commands ==========
    
    def enable_robot(self) -> bool:
        response = self._send_dashboard("EnableRobot()")
        print(f"[CMD] EnableRobot() -> {response}")
        time.sleep(1)
        return "0" in response
    
    def disable_robot(self) -> bool:
        response = self._send_dashboard("DisableRobot()")
        print(f"[CMD] DisableRobot() -> {response}")
        return "0" in response
    
    def clear_error(self) -> bool:
        response = self._send_dashboard("ClearError()")
        print(f"[CMD] ClearError() -> {response}")
        return "0" in response
    
    def reset_robot(self) -> bool:
        response = self._send_dashboard("ResetRobot()")
        print(f"[CMD] ResetRobot() -> {response}")
        time.sleep(2)
        return "0" in response
    
    def set_speed(self, speed: int) -> bool:
        speed = max(1, min(100, speed))
        response = self._send_dashboard(f"SpeedFactor({speed})")
        print(f"[CMD] SpeedFactor({speed}) -> {response}")
        return "0" in response

    def set_cp(self, cp: int) -> bool:
        cp = max(0, min(100, cp))
        response = self._send_dashboard(f"CP({cp})")
        print(f"[CMD] CP({cp}) -> {response}")
        return "0" in response
    
    def get_pose(self) -> str:
        return self._send_dashboard("GetPose()")
    
    def get_angle(self) -> str:
        return self._send_dashboard("GetAngle()")
    
    # ========== Motion Commands ==========
    
    def joint_move_j(self, j1: float, j2: float, j3: float, j4: float) -> str:
        cmd = f"JointMovJ({j1},{j2},{j3},{j4})"
        return self._send_move(cmd)

    def servo_j(self, j1: float, j2: float, j3: float, j4: float, t: float = 0.03) -> str:
        cmd = f"ServoJ({j1},{j2},{j3},{j4},t={t})"
        return self._send_move_no_delay(cmd)
    
    def move_j(self, x: float, y: float, z: float, r: float, cp: int = 0) -> str:
        if cp > 0:
            cmd = f"MovJ({x},{y},{z},{r},cp={cp})"
        else:
            cmd = f"MovJ({x},{y},{z},{r})"
        return self._send_move(cmd)
    
    def sync(self):
        self._send_move("Sync()")
        time.sleep(0.1)
    
    # ========== Safety ==========
    
    def check_joint_limits(self, j1: float, j2: float, j3: float, j4: float) -> List[str]:
        violations = []
        if j1 < self.JOINT_LIMITS['j1'][0] or j1 > self.JOINT_LIMITS['j1'][1]:
            violations.append(f"J1={j1:.1f}° (limit: ±160°)")
        if j2 < self.JOINT_LIMITS['j2'][0] or j2 > self.JOINT_LIMITS['j2'][1]:
            violations.append(f"J2={j2:.1f}° (limit: -25° to +85°)")
        if j3 < self.JOINT_LIMITS['j3'][0] or j3 > self.JOINT_LIMITS['j3'][1]:
            violations.append(f"J3={j3:.1f}° (limit: -25° to +105°)")
        if j4 < self.JOINT_LIMITS['j4'][0] or j4 > self.JOINT_LIMITS['j4'][1]:
            violations.append(f"J4={j4:.1f}° (limit: ±360°)")
        return violations


# ========== Conversion Helpers ==========

def convert_moveit_to_real(j2: float, j3_relative: float) -> float:
    return j3_relative + j2

def convert_real_to_moveit(j2: float, j3_absolute: float) -> float:
    return j3_absolute - j2


# ========== File Loading ==========

def load_waypoints(filepath: str) -> List[Dict]:
    with open(filepath, 'r') as f:
        data = json.load(f)
    return data.get('waypoints', [])


# ========== Safety Check ==========

def check_all_waypoints(robot: MG400, waypoints: List[Dict]) -> bool:
    print("\n[CHECK] Verifying joint limits (converting relative J3 → absolute)...")
    all_ok = True
    
    for i, wp in enumerate(waypoints):
        j1 = wp.get('j1', 0)
        j2 = wp.get('j2', 0)
        j3_rel = wp.get('j3', 0)
        j4 = wp.get('j4', 0)
        j3_abs = convert_moveit_to_real(j2, j3_rel)
        
        violations = robot.check_joint_limits(j1, j2, j3_abs, j4)
        if violations:
            all_ok = False
            print(f"  ⚠️  Point {i+1}: {', '.join(violations)}")
            print(f"       (MoveIt J3={j3_rel:.1f}° → Robot J3={j3_abs:.1f}°)")
    
    if all_ok:
        print("  ✓ All waypoints within joint limits\n")
    else:
        print()
    
    return all_ok


# ========== Execution: JointMovJ Mode ==========

def execute_trajectory(robot: MG400, waypoints: List[Dict], cp: int = 50, speed: int = 30):
    print(f"{'=' * 65}")
    print(f"  EXECUTING TRAJECTORY (JointMovJ + CP blending)")
    print(f"{'=' * 65}")
    print(f"  Waypoints: {len(waypoints)}")
    print(f"  CP (smoothing): {cp}%")
    print(f"  Speed: {speed}%")
    print(f"{'=' * 65}\n")
    
    robot.set_speed(speed)
    robot.set_cp(cp)
    time.sleep(0.3)
    
    for i, wp in enumerate(waypoints):
        j1 = wp.get('j1', 0)
        j2 = wp.get('j2', 0)
        j3_rel = wp.get('j3', 0)
        j4 = wp.get('j4', 0)
        j3_abs = convert_moveit_to_real(j2, j3_rel)
        
        current_cp = cp if i < len(waypoints) - 1 else 0
        
        print(f"  [{i+1:2}/{len(waypoints)}] JointMovJ({j1:7.2f}, {j2:7.2f}, {j3_abs:7.2f}, {j4:7.2f}) cp={current_cp}")
        robot.joint_move_j(j1, j2, j3_abs, j4)
        time.sleep(0.05)
    
    print("\n  Waiting for motion to complete...")
    robot.sync()
    time.sleep(10)
    print("\n[DONE] Trajectory complete!\n")


# ========== Execution: ServoJ Mode ==========

def execute_trajectory_servo(robot: MG400, waypoints: List[Dict], interval: float = 0.02, speed: int = 30, servo_t: float = 0.03):
    """
    Execute trajectory using ServoJ for smooth real-time streaming.
    
    Args:
        robot: Connected MG400 instance
        waypoints: List of waypoint dicts (from MoveIt, relative J3)
        interval: Time between sending points in seconds (default: 0.02)
        speed: Speed for initial move to start position (1-100)
        servo_t: ServoJ t parameter - time the robot takes per point (default: 0.03)
                 Increase for slower/smoother motion (e.g. 0.1, 0.2)
    """
    print(f"{'=' * 65}")
    print(f"  EXECUTING TRAJECTORY (ServoJ streaming)")
    print(f"{'=' * 65}")
    print(f"  Waypoints: {len(waypoints)}")
    print(f"  Send interval: {interval*1000:.0f}ms ({1/interval:.0f}Hz)")
    print(f"  ServoJ t: {servo_t*1000:.0f}ms (robot time per point)")
    print(f"  Estimated duration: {len(waypoints) * max(interval, servo_t):.1f}s")
    print(f"{'=' * 65}\n")
    
    # Move to start position using JointMovJ first
    first = waypoints[0]
    j1_s, j2_s = first.get('j1', 0), first.get('j2', 0)
    j3_rel_s, j4_s = first.get('j3', 0), first.get('j4', 0)
    j3_abs_s = convert_moveit_to_real(j2_s, j3_rel_s)
    
    print(f"  Moving to start position: ({j1_s:.2f}, {j2_s:.2f}, {j3_abs_s:.2f}, {j4_s:.2f})")
    robot.set_speed(speed)
    robot.joint_move_j(j1_s, j2_s, j3_abs_s, j4_s)
    robot.sync()
    print("  Waiting for robot to reach start position...")
    time.sleep(1)
    robot._clear_buffer(robot.move_socket)
    print("  At start position. Beginning ServoJ stream...\n")
    
    # Use the larger of interval and servo_t for timing
    actual_interval = max(interval, servo_t)
    
    # Stream waypoints using ServoJ at fixed interval
    dropped = 0
    for i, wp in enumerate(waypoints):
        t_start = time.perf_counter()
        
        j1 = wp.get('j1', 0)
        j2 = wp.get('j2', 0)
        j3_rel = wp.get('j3', 0)
        j4 = wp.get('j4', 0)
        j3_abs = convert_moveit_to_real(j2, j3_rel)
        
        result = robot.servo_j(j1, j2, j3_abs, j4, t=servo_t)
        
        # For the first 3 points, read and print the response to check for errors
        if i < 3:
            time.sleep(0.05)
            try:
                robot.move_socket.setblocking(False)
                resp = robot.move_socket.recv(1024).decode().strip()
                robot.move_socket.setblocking(True)
                robot.move_socket.settimeout(10)
                print(f"  [DEBUG] ServoJ response: {resp}")
            except:
                robot.move_socket.setblocking(True)
                robot.move_socket.settimeout(10)
        
        # Print progress every 10th point
        if i % 10 == 0 or i == len(waypoints) - 1:
            print(f"  [{i+1:4}/{len(waypoints)}] ServoJ({j1:7.2f}, {j2:7.2f}, {j3_abs:7.2f}, {j4:7.2f}) t={servo_t}")
        
        # Precise timing
        elapsed = time.perf_counter() - t_start
        sleep_time = actual_interval - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            dropped += 1
    
    if dropped > 0:
        print(f"\n  ⚠️  {dropped} points exceeded {actual_interval*1000:.0f}ms interval")
    
    time.sleep(0.5)
    print(f"\n[DONE] ServoJ trajectory complete!\n")


# ========== Main ==========

def main():
    parser = argparse.ArgumentParser(description='MG400 Joint Angle Executor with CP Smoothing & ServoJ')
    parser.add_argument('file', help='Waypoints JSON file')
    parser.add_argument('--ip', default='192.168.1.6', help='Robot IP (default: 192.168.1.6)')
    parser.add_argument('--cp', type=int, default=50, help='Continuous path 0-100 (default: 50, JointMovJ mode only)')
    parser.add_argument('--speed', type=int, default=30, help='Speed 1-100 (default: 30)')
    parser.add_argument('--servo', action='store_true', help='Use ServoJ streaming mode (smooth, for dense waypoints)')
    parser.add_argument('--interval', type=float, default=0.03, help='ServoJ send interval in seconds (default: 0.03 = 30ms)')
    parser.add_argument('--servo-t', type=float, default=0.03, help='ServoJ t parameter - time per point (default: 0.03). Increase for slower/smoother (e.g. 0.1, 0.2)')
    parser.add_argument('--dry-run', action='store_true', help='Preview only, no execution')
    parser.add_argument('--skip-check', action='store_true', help='Skip joint limit check')
    
    args = parser.parse_args()
    
    # Load waypoints
    try:
        waypoints = load_waypoints(args.file)
        print(f"[OK] Loaded {len(waypoints)} waypoints from {args.file}")
    except Exception as e:
        print(f"[ERROR] Failed to load waypoints: {e}")
        sys.exit(1)
    
    if not waypoints:
        print("[ERROR] No waypoints found!")
        sys.exit(1)
    
    first_wp = waypoints[0]
    if 'j1' not in first_wp:
        print("[ERROR] Waypoints file doesn't contain joint angles (j1, j2, j3, j4)")
        sys.exit(1)
    
    robot = MG400(ip=args.ip)
    mode = "ServoJ" if args.servo else "JointMovJ"
    
    # Dry run
    if args.dry_run:
        print(f"\n[DRY RUN] Mode: {mode}")
        if args.servo:
            print(f"  Send interval: {args.interval*1000:.0f}ms ({1/args.interval:.0f}Hz)")
            print(f"  ServoJ t: {args.servo_t*1000:.0f}ms")
            print(f"  Estimated duration: {len(waypoints) * max(args.interval, args.servo_t):.1f}s")
        else:
            print(f"  CP: {args.cp}%, Speed: {args.speed}%")
        
        print(f"{'─' * 75}")
        print(f"  {'#':<4} {'J1':>10} {'J2':>10} {'J3(MoveIt)':>12} {'J3(Robot)':>12} {'J4':>10}")
        print(f"{'─' * 75}")
        
        for i, wp in enumerate(waypoints):
            j1, j2, j3_rel, j4 = wp['j1'], wp['j2'], wp['j3'], wp['j4']
            j3_abs = convert_moveit_to_real(j2, j3_rel)
            print(f"  {i+1:<4} {j1:>10.2f} {j2:>10.2f} {j3_rel:>12.2f} {j3_abs:>12.2f} {j4:>10.2f}")
        
        print(f"{'─' * 75}\n")
        check_all_waypoints(robot, waypoints)
        sys.exit(0)
    
    # Connect
    if not robot.connect():
        sys.exit(1)
    
    try:
        # Safety check
        if not args.skip_check:
            limits_ok = check_all_waypoints(robot, waypoints)
            if not limits_ok:
                response = input("⚠️  Some waypoints may exceed limits. Continue? (y/N): ")
                if response.lower() != 'y':
                    print("[CANCELLED] Execution cancelled by user\n")
                    robot.disconnect()
                    sys.exit(0)
        
        # Initialize
        print("[INFO] Initializing robot...")
        robot.clear_error()
        robot.enable_robot()
        time.sleep(1)
        print(f"[INFO] Current angles: {robot.get_angle()}")
        print(f"[INFO] Mode: {mode}")
        
        input("\n>>> Press ENTER to execute (Ctrl+C to cancel) <<<\n")
        
        # Execute
        if args.servo:
            execute_trajectory_servo(robot, waypoints, interval=args.interval, speed=args.speed, servo_t=args.servo_t)
        else:
            execute_trajectory(robot, waypoints, cp=args.cp, speed=args.speed)
        
    except KeyboardInterrupt:
        print("\n[CANCELLED] Execution cancelled by user\n")
    except Exception as e:
        print(f"\n[ERROR] {e}\n")
    finally:
        robot.disable_robot()
        robot.disconnect()


if __name__ == "__main__":
    main()
    