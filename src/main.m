function main()
%% Initialization
    clearvars; clc;

    % Read Config and File
    cfg = config();
    f_ref = readmatrix(cfg.ref.filePath);
    initPos = f_ref(1, 2:4);
    initAtt = cfg.lidar.initAtt;
    f_test = readmatrix(cfg.test.filePath);
    
    % Variables Initialization
    len = size(f_test, 1);
    t_unix = zeros(len, 1);
    t_gps = zeros(len, 1);
    r_g_n_all = size(f_test, 3);
    wgs84PosAtt = zeros(len, 7);
    
    % Extract UnixTime from FileName
    lidarFormat = '*.' + cfg.lidar.format;
    f_lidar = dir(fullfile(cfg.lidar.filePath, lidarFormat));

    switch cfg.test.timeSource
        case 1
            if (cfg.test.format == 1)
                % Save Time Format
                t_unix_start = str2double(extractBefore(f_lidar(1).name, 12)) * 0.1;
                for i = 1:len
                    t_unix = t_unix_start + f_test(i,1);
                    % Convert UnixTime to GPStime
                    t_gps = unix2GpsTime(t_unix, cfg.time.leapSec);
                    
                    switch cfg.output.timeFormat
                        case 1
                            wgs84PosAtt(i,1) = t_unix;
                        case 2
                            wgs84PosAtt(i,1) = t_gps;
                        otherwise
                            printf("[ERROR] Wrong Data Format Chosen.\n");
                            exit;
                    end
                end
                
            else
                fprintf("[INFO] Unconsistent time source input.\n");
                exit;
            end

        case 2
            for i = 1:len
                t_unix(i) = str2double(extractBefore(f_lidar(i).name, 12)) * 0.1;
            end
                
            % Convert UnixTime to GPStime
            t_gps = unix2GpsTime(t_unix, cfg.time.leapSec);
        
            % Save Time Format
            switch cfg.output.timeFormat
                case 1
                    wgs84PosAtt(:,1) = t_unix;
                case 2
                    wgs84PosAtt(:,1) = t_gps;
                otherwise
                    printf("[ERROR] Wrong Data Format Chosen.\n");
                    exit;
            end

        otherwise
            fprintf("[INFO] Wrong time source input.\n");
            exit;
    end

    
    
%% DG Process
    r_g2l_v = [cfg.lidar.la(2); cfg.lidar.la(1); -cfg.lidar.la(3)];
    R_v2l = eul2rotm([-cfg.lidar.bs(3), cfg.lidar.bs(1), cfg.lidar.bs(2)*pi/180]./180.*pi, 'ZYX');
    R_n2v = eul2rotm([-initAtt(3), initAtt(1), initAtt(2)]./180.*pi, 'ZYX');
    R_v2n = R_n2v';
    
    switch cfg.test.format
        % TUM data format [time, x, y, z, qx, qy, qz, qw]
        case 1
            % Check data column
            if size(f_test, 2) ~= 8
                printf("[ERROR] Wrong Test File Chosen.\n");
                exit;
            end

            for i = 1:len
                % Rotation Calculation
                [roll, pitch, head] = quat2euldeg(f_test(i,5:8));
                % "head" is up axis rotate counter-clockwise (right-handed)
                if (head < 0)
                    head = head + 360;
                end
                R_v2l_ = eul2rotm([head, roll, pitch]./180.*pi, 'ZYX');
                R_n2l_ = R_v2l_*R_n2v;
                R_n2l = R_v2l*R_n2v;
                R_l2n = R_n2l';
                eulUFR = rotm2eul(R_n2l_, 'ZYX')./pi.*180;
                eulFRD = [eulUFR(2); eulUFR(3); -eulUFR(1)];
                if eulFRD(3) < 0
                    eulFRD(3) = eulFRD(3) + 360;
                end
                wgs84PosAtt(i,5) = eulFRD(1);  % roll
                wgs84PosAtt(i,6) = eulFRD(2);  % pitch
                wgs84PosAtt(i,7) = eulFRD(3);  % yaw

                % Translation Calculation
                r_l__l = f_test(i,2:4);
                r_g_n = R_l2n*r_l__l' +  R_v2n*r_g2l_v;
                r_g_n_all(i,1:3) = r_g_n';
                [latNew, lonNew, altNew] = local2latlon(r_g_n(1), r_g_n(2), r_g_n(3), initPos);
                wgs84PosAtt(i,2) = latNew;
                wgs84PosAtt(i,3) = lonNew;
                wgs84PosAtt(i,4) = altNew;
            end

        % KITTI data format [rotm11, rotm12, rotm13, rotm14, rotm21, rotm22, rotm23, rotm24, rotm31, rotm32, rotm33, rotm34]
        case 2
            % Check data column
            if size(f_test, 2) ~= 12
                printf("[ERROR] Wrong Test File Chosen.\n");
                exit;
            end

            for i = 1:len
                % Rotation Calculation
                RT_kit = [f_test(i,1),f_test(i,2),f_test(i,3),f_test(i,4);...
                    f_test(i,5),f_test(i,6),f_test(i,7),f_test(i,8); ...
                    f_test(i,9),f_test(i,10),f_test(i,11),f_test(i,12);
                    0,0,0,1];
                R_v2l_ = RT_kit(1:3,1:3);

                R_n2l_ = R_v2l_*R_n2v;
                R_n2l = R_v2l*R_n2v;
                R_l2n = R_n2l';
                eulUFR = rotm2eul(R_n2l_, 'ZYX')./pi.*180;
                eulFRD = [eulUFR(2); eulUFR(3); -eulUFR(1)];
                if eulFRD(3) < 0
                    eulFRD(3) = eulFRD(3) + 360;
                end
                wgs84PosAtt(i,5) = eulFRD(1);  % roll
                wgs84PosAtt(i,6) = eulFRD(2);  % pitch
                wgs84PosAtt(i,7) = eulFRD(3);  % yaw

                % Translation Calculation
                r_l__l = RT_kit(1:3,4);
                r_g_n = R_l2n*r_l__l +  R_v2n*r_g2l_v;
                r_g_n_all(i,1:3) = r_g_n';
                [latNew, lonNew, altNew] = local2latlon(r_g_n(1), r_g_n(2), r_g_n(3), initPos);
                wgs84PosAtt(i,2) = latNew;
                wgs84PosAtt(i,3) = lonNew;
                wgs84PosAtt(i,4) = altNew;
            end
        otherwise
            printf("[ERROR] Wrong Trajectory Format Chosen.\n");
            exit;
    end
    
    % Check result based on navigation frame
    figure;
    x = r_g_n_all(:,1);
    y = r_g_n_all(:,2);
    plot(x,y);
    xlabel("x"); ylabel("y");

%% Coordinate Transform
    % for i = 1:len
    %     [latNew, lonNew, h] = local2latlon(-f_test(i,3), f_test(i,2), f_test(i,4), initPos);
    %     wgs84PosAtt(i, 2) = latNew;
    %     wgs84PosAtt(i, 3) = lonNew;
    %     wgs84PosAtt(i, 4) = h;
    % end
    
%% Output result
    writematrix(wgs84PosAtt, cfg.output.filePath);
end