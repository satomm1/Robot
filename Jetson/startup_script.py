#!/usr/bin/env python3
from http.server import BaseHTTPRequestHandler, HTTPServer
import subprocess

# Global variable to keep track of the launch process
launch_process = None

class LaunchServer(BaseHTTPRequestHandler):
    def do_GET(self):
        global launch_process
        
        # Route to start the launch file
        if self.path == '/start':
            if launch_process is None or launch_process.poll() is not None:
                # IMPORTANT: We source the ROS environment right before running the launch command
                cmd = "source /opt/ros/noetic/setup.bash && source /workspace/catkin_ws/devel/setup.bash && source /workspace/catkin_ws/src/robot_env.sh && roslaunch mattbot_bringup short.launch"
                launch_process = subprocess.Popen(cmd, shell=True, executable='/bin/bash')
                
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b"ROS Launch started successfully!")
            else:
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b"Launch file is already running.")
                
        # Route to stop the launch file gracefully
        elif self.path == '/stop':
            if launch_process and launch_process.poll() is None:
                launch_process.terminate() # Sends a SIGTERM to cleanly shut down ROS nodes
                launch_process = None
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b"ROS Launch stopped cleanly.")
            else:
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b"Nothing is currently running.")
        else:
            self.send_response(404)
            self.end_headers()

if __name__ == '__main__':
    # Listen on port 8080 across the entire local network
    server = HTTPServer(('0.0.0.0', 8080), LaunchServer)
    print("Web launcher listening on port 8080...")
    server.serve_forever()