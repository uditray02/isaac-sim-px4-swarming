#!/usr/bin/env python


# Imports to start Isaac Sim from this script
import carb

from isaacsim import SimulationApp


simulation_app = SimulationApp({
    "headless": False,
    "extra_args": [
        "--ext-folder", "extensions",
        "--enable", "pegasus.simulator"
    ]
})


# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World
from isaacsim.core.utils.extensions import enable_extension

# Enable/disable ROS bridge extensions to keep only ROS2 Bridge
enable_extension("isaacsim.ros2.bridge")

# Update the simulation app with the new extensions
simulation_app.update()

# -------------------------------------------------------------------------------------------------
# These lines are needed to restart the USD stage and make sure that the people extension is loaded
# -------------------------------------------------------------------------------------------------
import omni.usd
omni.usd.get_context().new_stage()

import numpy as np

# Import the Pegasus API for simulating drones
from omni.isaac.core.prims import XFormPrim

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.people.person import Person
from pegasus.simulator.logic.people.person_controller import PersonController
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Example controller class that make a person move in a circle around the origin of the world
# Note: You could create a different controller with a different behaviour. For instance, you could:
# 1. read the keyboard input to move the person around the world.
# 2. read the target position from a ros topic,
# 3. read the target position from a file,
# 4. etc.
class CirclePersonController(PersonController):

    def __init__(self, radius=5.0, speed=0.3, direction=1.0):
        super().__init__()
        self._radius = radius
        self.gamma = 0.0
        self.gamma_dot = speed * direction  # +CW / -CCW

    def update(self, dt: float):
        self.gamma += self.gamma_dot * dt
        self._person.update_target_position([
            self._radius * np.cos(self.gamma),
            self._radius * np.sin(self.gamma),
            0.0
        ])

class CircleCarController:
    def __init__(self, radius=6.0, speed=0.2, z=0.0):
        self.radius = radius
        self.speed = speed
        self.theta = 0.0
        self.z = z

    def update(self, dt, prim):
        self.theta += self.speed * dt
        x = self.radius * np.cos(self.theta)
        y = self.radius * np.sin(self.theta)
        prim.set_world_pose(
            position=[x, y, self.z],
            orientation=Rotation.from_euler(
                "XYZ", [0.0, 0.0, np.degrees(self.theta)], degrees=True
            ).as_quat()
        )

        

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation

# -------------------------------------------------------------------------------------------------
# Define the PegasusApp class where the simulation will be run
# -------------------------------------------------------------------------------------------------
class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics,
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        #self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        print("\nAVAILABLE SIMULATION ENVIRONMENTS:")
        for k in SIMULATION_ENVIRONMENTS.keys():
            print(" -", k)

        self.pg.load_asset(SIMULATION_ENVIRONMENTS["Black Gridroom"], "/World/layout")
        

        # Check the available assets for people
        # ---------------- PEOPLE SETUP (5 persons) ----------------
        # ---------------- PEOPLE SETUP (5 persons) ----------------
        # ---------------- PEOPLE SETUP (5 persons, varied motion) ----------------
        # ---------------- PEOPLE SETUP (3 moving only) ----------------
        person_assets = [
            "original_male_adult_construction_01",
            "original_male_adult_police_04",
            "original_female_adult_business_02",
        ]

        radius_spawn = 8.0
        angle_step = 2 * np.pi / len(person_assets)

        # Motion diversity
        directions = [1.0, -1.0, 1.0]     # CW, CCW, CW
        speeds     = [0.25, 0.40, 0.30]   # different angular speeds
        circle_r   = [5.0, 6.5, 4.5]      # different radii

        for i, asset in enumerate(person_assets):
            angle = i * angle_step
            x = radius_spawn * np.cos(angle)
            y = radius_spawn * np.sin(angle)

            controller = CirclePersonController(
                radius=circle_r[i],
                speed=speeds[i],
                direction=directions[i]
            )

            Person(
                f"person_{i}",
                asset,
                init_pos=[x, y, 0.0],
                init_yaw=angle,
                controller=controller
            )
# -------------------------------------------------------------

        # ------------------------------------------------------------------------

        # ----------------------------------------------------------



        config_multirotor = MultirotorConfig()
        # Create the multirotor configuration
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": "/home/udit-2/PX4-Autopilot",
            "px4_vehicle_model": "iris"
        })

        config_multirotor.backends = [
            PX4MavlinkBackend(mavlink_config),
            ROS2Backend(vehicle_id=1, 
                config={
                    "namespace": 'drone', 
                    "pub_sensors": False,
                    "pub_graphical_sensors": True,
                    "pub_state": True,
                    "pub_tf": False,
                    "sub_control": False,})]
        
        config_multirotor.graphical_sensors = [MonocularCamera("camera", config={"update_rate": 60.0})]
        
        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, -23.0, 1.5],
            Rotation.from_euler("XYZ", [0.0, 0.0, 90.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Set the camera of the viewport to a nice position
        #self.pg.set_viewport_camera([5.0, 9.0, 6.5], [0.0, 0.0, 0.0])
        self.pg.set_viewport_camera([9.0, -26.0, 2.5], [0.0, 0.0, 0.0])
        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:
            # Update the UI of the app and perform the physics step
            self.world.step(render=True)

        # Cleanup and stop
        carb.log_warn("Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():

    # Instantiate the template app
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
