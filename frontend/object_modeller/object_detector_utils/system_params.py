#########################################
# 
# system_params.py
#
# Global system parameters shared across perception and mapping modules.
#
# Authors: Generic template by ChatGPT
# Date: November 2025
#
#########################################

from dataclasses import dataclass
from typing import Optional, Any


@dataclass
class SystemParams:
    """
    Global configuration parameters for the perception + mapping system.
    """

    # ==== System behavior ====
    enable_rerun_viz: bool = False       # Enable/disable Rerun visualization output
    save_debug_images: bool = False      # Whether to save visualization or debug outputs
    log_level: str = "INFO"              # Logging verbosity ("DEBUG", "INFO", "WARNING", "ERROR")

    # ==== Timing and synchronization ====
    fps_limit: Optional[float] = None    # Max processing rate in Hz (None = unlimited)
    time_sync_tolerance: float = 0.05    # Allowed timestamp offset between sensors (seconds)

    # ==== Coordinate system and transforms ====
    world_frame: str = "world"
    camera_frame: str = "camera_link"
    base_frame: str = "base_link"

    # ==== Output directories ====
    output_dir: str = "~/data/output"
    log_dir: str = "~/data/logs"

    # ==== Visualization parameters ====
    show_masks: bool = True              # Whether to render segmentation masks
    show_point_clouds: bool = True       # Whether to render RGB-D point clouds
    show_bounding_boxes: bool = False    # Whether to visualize bounding boxes

    # ==== Computational resources ====
    device: str = "cuda"                 # Default compute device: "cuda" or "cpu"
    num_threads: int = 4                 # For CPU-heavy preprocessing

    # ==== Map and localization ====
    use_global_map: bool = False
    map_resolution: float = 0.05         # m per voxel for voxel maps
    map_update_rate: float = 1.0         # Hz

    # ==== Debug and experimental flags ====
    enable_scene_flow_dynamic_obj_removal: bool = False
    optimized_fastsam_inference: bool = True
    optimized_clip_embedding_calculation: bool = True

    # ==== Miscellaneous ====
    random_seed: int = 42
    extra: Optional[dict[str, Any]] = None


# Optional: global default instance
default_system_params = SystemParams()
