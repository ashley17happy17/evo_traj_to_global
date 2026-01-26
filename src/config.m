function cfg = config()
    % Config of Time
    cfg.time.utc = 8;  % Taiwan is UTC+8
    cfg.time.leapSec = 27;  % Leap second in 2025 is 27sec

    % Config for Test Data
    cfg.test.filePath = "../data/20250916_d_pinslam/slam_poses_tum.txt";
    cfg.test.format = 1; % 1:tum; 2:kitti
    cfg.test.timeSource = 1; % 1:fromFile (only for tum file) % 2: from LiDAR data

    cfg.lidar.filePath = "../../output_raw_lidar_pcd/output/20250916_d_raw_10hz/";
    cfg.lidar.format = "pcd";
    cfg.lidar.la = [0.967, -0.037, 0.444];  % gnss to sensor in front/right/down axes(m)
    cfg.lidar.bs = [0.235, -0.496, -79.108];
    cfg.lidar.initAtt = [0; 0; 18.99361]; % unit: deg, [roll,pitch,head], while "head" is 0 at north, 90 at west

    % Config for Reference
    cfg.ref.filePath = "../data/20250916_d_pinslam/20250916163954_d.csv";
    
    % Config for Output
    cfg.output.timeFormat = 2; % 1:unixTime; 2:gpsTime
    cfg.output.filePath = "../output/20250916_d_pinslam_slam_tum.csv";
end