const presets : {[key :string] :string[]} = {
    "State Estimation": [
        "/tf",
        "/static_tf",
        "/mavros/local_position/pose",
        "/mavros/local_position/velocity_body",
        "/mavros/local_position/velocity_local",
        "/rovio/odometry",
        "/rovio/transform",
        "/rovio/pcl",
        "/rovio/patch",
        "/rovio/markers",
        "/rovio/trackingimage",
        "/world_evidence",
        "/mavros/imu/data_raw",
        "/mavros/global_position/raw/gps_vel",
        "/mavros/global_position/raw/fix",
        "/mavros/global_position/local",
    ],
    "Planning": [
        "/reference_path_topic",
        "/speed_profile",
        "/world_state",
        "/planning_map_topic2",
        "/planning/laps_completed",
        "/planning/viz/ald_cones",
        "/planning/viz/triangulation",
        "/planning/viz/boundary_markers",
    ],
    "Acceleration": [
        "/planning/accel_model_state",
        "/planning/accel_line_params",
        "/viz/ref_path",
        "/viz/accel/line1_cones",
        "/viz/accel/line2_cones",
        "/world_state",
    ],
    "Controls": [
        "/vehicle_interface/to/TrajectorySetpoints",
        "/vehicle_interface/from/SteeringAngle",
        "/vehicle_interface/from/MotorTorques",
        "/vehicle_interface/from/MotorVelocities",
        "/vehicle_interface/from/AMSStateUpdate",
        "/vehicle_interface/from/ASSStateUpdate",
    ],
    "Perception CV": [
        "/camera/quad1/image_rect_color",
        "/camera/quad2/image_rect_color",
        "/camera/quad3/image_rect_color",
        "/camera/quad4/image_rect_color",
    ],
    "Lidar": [
        "/velodyne_packets",
        "/velodyne_points",
        "/mavros/local_position/velocity_body",
    ]
};

export default presets