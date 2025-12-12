# This script is a placeholder for developing an Isaac Sim script to generate and capture synthetic RGB-D data.
# Detailed implementation will be provided in the relevant chapter content.

import carb
from omni.isaac.kit import SimulationApp

CONFIG = {"headless": False}
simulation_app = SimulationApp(CONFIG)

import omni.timeline
import omni.isaac.core as ic
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.articulations import Articulation
from omni.isaac.sensor import Camera # Assuming Camera class from Isaac Sim handles RGB-D
from omni.isaac.synthetic_utils import SyntheticDataHelper # For capturing data

class RGBDDataCapturer:
    def __init__(self):
        self.kit = simulation_app
        self.stage = ic.get_current_stage()
        self.timeline = omni.timeline.get_timeline_interface()
        self.assets_root_path = get_assets_root_path()
        if self.assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            simulation_app.close()
            exit()
        
        self.sd_helper = SyntheticDataHelper()
        self.camera_prim_path = "/humanoid/base_link/humanoid_rgbd" # Assumes camera is attached
        self.camera = None

    def initialize_camera(self):
        """
        Initializes and retrieves the camera object from the stage.
        Assumes configure_sensors.py has already attached the camera.
        """
        try:
            self.camera = Camera(prim_path=self.camera_prim_path)
            self.camera.initialize()
            print(f"Initialized camera at {self.camera_prim_path}")
        except Exception as e:
            print(f"Error initializing camera: {e}. Ensure camera is attached at {self.camera_prim_path}")
            # Fallback for standalone testing - try to add a dummy camera
            print("Attempting to add a placeholder camera for testing.")
            world = ic.get_current_scene()
            if world is None:
                 world = ic.SimulationContext()
                 world.set_simulation_context(simulation_app.app.new_stage())
                 world.add_ground_plane()

            from omni.isaac.core.utils.stage import add_reference_to_stage
            humanoid_usd_path = self.assets_root_path + "/Isaac/Robots/Humanoid/humanoid_instanceable.usd"
            add_reference_to_stage(humanoid_usd_path, "/humanoid")
            world.scene.add(Articulation(prim_path="/humanoid", name="humanoid_robot"))
            world.reset()
            world.render()
            
            # Re-attempt camera creation
            self.camera = Camera(
                prim_path=self.camera_prim_path,
                position=ic.utils.buffers.np.array([0.1, 0.0, 0.1]),
                orientation=ic.utils.buffers.np.array([1.0, 0.0, 0.0, 0.0]),
                resolution=(640, 480),
                fov_y=60.0,
                focal_length=2.4,
                clipping_range=(0.1, 100000.0)
            )
            self.camera.initialize()
            print(f"Successfully added and initialized placeholder camera at {self.camera_prim_path}")


    def capture_rgbd_data(self, output_dir="output_rgbd_data"):
        """
        Captures RGB and Depth data from the configured camera.
        """
        if self.camera is None:
            print("Camera not initialized. Cannot capture data.")
            return

        # Ensure directory exists
        import os
        os.makedirs(output_dir, exist_ok=True)

        self.timeline.play()
        self.kit.update() # Update once to ensure sensor data is ready
        self.sd_helper.initialize(sensor_list=[self.camera])
        
        # Capture a single frame (or multiple in a loop)
        self.kit.update()
        self.sd_helper.render()

        # RGB data
        rgb_data = self.sd_helper.get_data(self.camera_prim_path, "rgb")
        # Depth data
        depth_data = self.sd_helper.get_data(self.camera_prim_path, "distance_to_image_plane")

        if rgb_data is not None and depth_data is not None:
            import numpy as np
            from PIL import Image

            # Save RGB image
            rgb_image = Image.fromarray(rgb_data)
            rgb_filename = os.path.join(output_dir, "rgb_image_000.png")
            rgb_image.save(rgb_filename)
            print(f"Saved RGB image to {rgb_filename}")

            # Save Depth image (e.g., as a 16-bit PNG or numpy array)
            # Normalize depth for visualization if saving as PNG, or save raw numpy
            depth_filename_npy = os.path.join(output_dir, "depth_data_000.npy")
            np.save(depth_filename_npy, depth_data)
            print(f"Saved Depth data to {depth_filename_npy}")
            
            # Optionally, save a visualized depth image
            # depth_visualized = (depth_data / np.max(depth_data) * 255).astype(np.uint8)
            # depth_image = Image.fromarray(depth_visualized, mode='L')
            # depth_filename_png = os.path.join(output_dir, "depth_image_000.png")
            # depth_image.save(depth_filename_png)
            # print(f"Saved visualized Depth image to {depth_filename_png}")


        self.timeline.stop()
        print("RGB-D data capture complete.")


    def __del__(self):
        """
        Cleans up the simulation app when the object is destroyed.
        """
        if self.kit:
            self.kit.close()

if __name__ == "__main__":
    world = ic.SimulationContext()
    world.set_simulation_context(simulation_app.app.new_stage())
    world.add_ground_plane()

    # Create a dummy humanoid and attach a camera for standalone script testing
    # In a real scenario, load_humanoid.py and configure_sensors.py would have set this up
    assets_root_path = get_assets_root_path()
    humanoid_usd_path = assets_root_path + "/Isaac/Robots/Humanoid/humanoid_instanceable.usd"
    ic.utils.stage.add_reference_to_stage(humanoid_usd_path, "/humanoid")
    world.scene.add(Articulation(prim_path="/humanoid", name="humanoid_robot"))
    world.reset()
    world.render()

    sensor_config = Camera(
            prim_path="/humanoid/base_link/humanoid_rgbd",
            position=ic.utils.buffers.np.array([0.1, 0.0, 0.1]),
            orientation=ic.utils.buffers.np.array([1.0, 0.0, 0.0, 0.0]),
            resolution=(640, 480),
            fov_y=60.0,
            focal_length=2.4,
            clipping_range=(0.1, 100000.0)
    )
    sensor_config.initialize()
    # Need to enable specific data products
    sensor_config.set_enabled_fov_flag(True)
    sensor_config.set_enabled_horizontal_aperture_flag(True)
    sensor_config.set_enabled_vertical_aperture_flag(True)
    sensor_config.set_enabled_clipping_range_flag(True)
    sensor_config.add_semantic_segmentation_to_stream(semantic_type="class")
    sensor_config.add_depth_to_stream() # Enable depth data


    capturer = RGBDDataCapturer()
    capturer.initialize_camera() # This will ensure camera is ready
    capturer.capture_rgbd_data()
    
    print("RGB-D capture script finished.")
    # Keep simulation app running for manual inspection if not headless
    while simulation_app.is_running():
        simulation_app.update()

    del capturer
