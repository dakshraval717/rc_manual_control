import tkinter as tk
import rospy
import subprocess
import threading
import os  # NEW for path expansion
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray

class RCCarInterface:
    def __init__(self):
        rospy.init_node('RC-Car-UI', anonymous=True)  # node name cant have spaces

        # Main window setup
        self.root = tk.Tk()
        self.root.title("RC Car Network Controller")
        self.root.geometry("800x600")

        # Car status tracking, ie. are we tapped into the visual and LiDAR streams of this specific car?
        self.carStatus = {f'Car {i+1}': False for i in range(4)}

        self.createInterface()

        # Status display
        self.statusLabel = tk.Label(self.root, text="System Ready", font=("Arial", 12))
        self.statusLabel.grid(row=1, column=0, columnspan=4, pady=10)

    def createInterface(self):
        # Create 4 car control buttons
        for i in range(4):
            car_id = i + 1
            rcCarButton = tk.Button(
                self.root,
                text=f"Launch RC Car {car_id}",
                width=18,
                height=3,
                command=lambda cid=car_id: self.launchCarSystem(cid),
                bg='lightblue'
            )
            rcCarButton.grid(row=0, column=i, padx=10, pady=20)

    def launchCarSystem(self, carID):
        self.statusLabel.config(text=f"Launching Car {carID}...")
        thread = threading.Thread(target=self.executeLaunch, args=(carID,))
        thread.daemon = True
        thread.start()

    def executeLaunch(self, carID):
        scriptPath = os.path.expanduser(
            f"~/catkin_ws/src/real-time-ui/launch-car-{carID}/terminator.sh"
        )
        subprocess.call(['bash', scriptPath])

        self.carStatus[f'Car {carID}'] = True
        self.statusLabel.config(text=f"Car {carID} Active")

if __name__ == '__main__':
    interface = RCCarInterface()
    interface.root.mainloop()
