#!/usr/bin/env python3

import os
import subprocess
import signal
import time
import psutil

def cleanup_fastdds_shm():
    """Clean up FastDDS shared memory files"""
    try:
        # Remove FastDDS shared memory files
        subprocess.run(['sudo', 'rm', '-rf', '/dev/shm/fastrtps_*', '/dev/shm/sem.fastrtps_*'], 
                      check=True)
        print("✓ Cleaned up FastDDS shared memory files")
    except subprocess.CalledProcessError as e:
        print(f"✗ Failed to clean up FastDDS shared memory: {e}")

def find_camera_processes():
    """Find processes using the RealSense camera"""
    camera_processes = []
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            # Check if process is using the camera
            if proc.info['cmdline'] and any('realsense' in cmd.lower() for cmd in proc.info['cmdline']):
                camera_processes.append(proc)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return camera_processes

def kill_camera_processes():
    """Terminate processes using the RealSense camera"""
    processes = find_camera_processes()
    if not processes:
        print("✓ No camera processes found")
        return

    for proc in processes:
        try:
            # Try SIGTERM first
            proc.terminate()
            print(f"✓ Sent SIGTERM to process {proc.pid}")
        except psutil.NoSuchProcess:
            continue

    # Wait for processes to terminate
    time.sleep(2)

    # Check if any processes are still running and force kill them
    for proc in processes:
        try:
            if proc.is_running():
                proc.kill()
                print(f"✓ Force killed process {proc.pid}")
        except psutil.NoSuchProcess:
            continue

def main():
    print("Starting cleanup...")
    
    # Clean up FastDDS shared memory
    cleanup_fastdds_shm()
    
    # Kill camera processes
    kill_camera_processes()
    
    print("Cleanup complete!")

if __name__ == '__main__':
    main() 