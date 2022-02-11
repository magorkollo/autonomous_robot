import pyrealsense2 as rs
import numpy as np

class DepthCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()

        # pipeline and product line information to get resolution support
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = config.resolve(pipeline_wrapper)
        device = self.pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        # point cloud initialization
        self.pointcloud = rs.pointcloud()
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # configuring the camera stream
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        depth_frame_distance = frames.get_depth_frame().as_depth_frame()

        # get depth information 
        self.pointcloud.map_to(depth_frame)
        points = self.pointcloud.calculate(depth_frame)

        depth_image = np.asanyarray(depth_frame.get_data())
        depth = depth_image.astype(float)
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame or not points:
            return False, None, None, None
        return True, depth, color_image, points, depth_frame_distance

    def release(self):
        self.pipeline.stop()

    def get_profile(self):
        return self.pipeline_profile