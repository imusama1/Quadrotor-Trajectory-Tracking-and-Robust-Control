"""
AR.Drone 2.0 Hardware Interface
"""

import time
import math

try:
    from pyardrone import ARDrone
    from pyardrone import at
    ARDRONE_AVAILABLE = True
except ImportError:
    ARDRONE_AVAILABLE = False
    print("Warning: pyardrone not installed. Using simulation mode.")


class DroneInterface:
    """
    Interface to AR.Drone 2.0 hardware.
    
    Communication:
    - WiFi AP: ardrone2_XXXXXX (192.168.1.1)
    - AT Commands: UDP port 5556
    - NavData: UDP port 5554
    """
    
    def __init__(self):
        self.drone = None
        self.connected = False
        self.simulation_mode = not ARDRONE_AVAILABLE
        
        # Simulation state
        self._sim_x = 0.0
        self._sim_y = 0.0
        self._sim_z = 1.0
        self._sim_vx = 0.0
        self._sim_vy = 0.0
        self._sim_yaw = 0.0
    
    def connect(self):
        """Connect to AR.Drone via WiFi."""
        print("="*50)
        print("Connecting to AR.Drone 2.0...")
        print("  AT Commands → UDP 5556")
        print("  NavData ← UDP 5554")
        print("="*50)
        
        if self.simulation_mode:
            print("Running in SIMULATION mode")
            self.connected = True
            return True
        
        try:
            self.drone = ARDrone()
            self.drone.navdata_ready.wait(timeout=10)
            
            if not self.drone.navdata_ready.is_set():
                print("ERROR: NavData not ready (check WiFi)")
                return False
            
            print("Connected! Sensor data streaming.")
            self.connected = True
            return True
            
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from drone."""
        if self.drone:
            self.drone.close()
        self.connected = False
    
    def read_state(self):
        """
        Read state from NavData demo packet.
        
        Returns:
            tuple: (vx_world, vy_world, vz, altitude, yaw_deg)
        """
        if self.simulation_mode:
            return self._sim_vx, self._sim_vy, 0.0, self._sim_z, self._sim_yaw
        
        try:
            demo = self.drone.navdata.demo
            
            # Velocities from optical flow (body frame)
            vx_body = demo.vx / 1000.0  # mm/s → m/s
            vy_body = demo.vy / 1000.0
            vz = demo.vz / 1000.0
            
            # Attitude (for frame transformation)
            yaw_deg = demo.psi / 1000.0  # milli-deg → deg
            yaw = yaw_deg * math.pi / 180.0  # → radians
            
            # Altitude from ultrasonic
            alt = demo.altitude / 100.0  # cm → m
            
            # Transform velocity: body → world frame
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            vx_world = vx_body * cos_yaw - vy_body * sin_yaw
            vy_world = vx_body * sin_yaw + vy_body * cos_yaw
            
            return vx_world, vy_world, vz, alt, yaw_deg
            
        except Exception:
            return 0, 0, 0, 1.0, 0
    
    def send_command(self, roll, pitch, gaz, yaw):
        """
        Send movement command to drone.
        
        Args:
            roll: Roll command [-1, 1]
            pitch: Pitch command [-1, 1]
            gaz: Vertical speed [-1, 1]
            yaw: Yaw rate [-1, 1]
        """
        if self.simulation_mode:
            # Simple simulation response
            self._sim_vx = pitch * 0.5
            self._sim_vy = roll * 0.5
            self._sim_x += self._sim_vx * 0.05
            self._sim_y += self._sim_vy * 0.05
            self._sim_z += gaz * 0.1 * 0.05
            return
        
        cmd = at.PCMD(at.PCMD.flag.progressive, roll, pitch, gaz, yaw)
        self.drone.send(cmd)
    
    def hover(self):
        """Command drone to hover in place."""
        if self.simulation_mode:
            self._sim_vx = 0
            self._sim_vy = 0
            return
        
        self.drone.hover()
    
    def takeoff(self):
        """Execute takeoff sequence."""
        print("\nTaking off...")
        
        if self.simulation_mode:
            self._sim_z = 1.0
            print("Simulated takeoff complete")
            return True
        
        self.drone.takeoff()
        
        timeout = time.time() + 10
        while not self.drone.state.fly_mask and time.time() < timeout:
            time.sleep(0.1)
        
        if not self.drone.state.fly_mask:
            print("Takeoff failed!")
            return False
        
        print("Stabilizing (4s)...")
        time.sleep(4)
        
        _, _, _, alt, _ = self.read_state()
        print(f"Hovering at {alt*100:.0f}cm")
        return True
    
    def land(self):
        """Execute landing sequence."""
        print("\nLanding...")
        
        if self.simulation_mode:
            self._sim_z = 0.0
            print("Simulated landing complete")
            return
        
        self.hover()
        time.sleep(0.5)
        self.drone.land()
        
        timeout = time.time() + 10
        while self.drone.state.fly_mask and time.time() < timeout:
            time.sleep(0.1)
        
        print("Landed!")
    
    @property
    def is_flying(self):
        """Check if drone is currently flying."""
        if self.simulation_mode:
            return self._sim_z > 0.1
        return self.drone.state.fly_mask if self.drone else False
