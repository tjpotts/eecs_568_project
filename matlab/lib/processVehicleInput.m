% t_odometry: Nx1
% odometry: Nx3x3

% t_gps: Nx1
% gps: Nx2

% t_landmarks: Nx1
% landmarks: Nx4

function [processed_data] = processVehicleInput(input, t_start, t_end)
    t_odometry = input.Odometry.YawRateTime;
    odometry = zeros(numel(t_odometry),1);
    odometry_index = 1;
    for i = 1:numel(t_odometry)
        t = t_odometry(i);
        if (t < input.Odometry.VehicleSpeedTime(1))
            odometry(i,1) = input.Odometry.VehicleSpeed(1);
        elseif (t > input.Odometry.VehicleSpeedTime(end))
            odometry(i,1) = input.Odometry.VehicleSpeed(end);
        else
            while (t > input.Odometry.VehicleSpeedTime(odometry_index+1))
                odometry_index = odometry_index+1;
            end
            t1 = input.Odometry.VehicleSpeedTime(odometry_index);
            t2 = input.Odometry.VehicleSpeedTime(odometry_index+1);
            v1 = input.Odometry.VehicleSpeed(odometry_index);
            v2 = input.Odometry.VehicleSpeed(odometry_index+1);
            slope = (v2 - v1) / (t2 - t1);
            odometry(i,1) = v1 + slope * (t - t1);
        end

        odometry(i,1) = odometry(i,1) * 0.27777;

        % Rotational velocity (converted from deg/s to rad/s)
        odometry(i,3) = input.Odometry.YawRate(i);
    end

    t_gps = input.GPS.Time;
    gps = zeros(numel(t_gps),2);
    i_gps = find(t_gps > t_start, 1);
    lat0 = input.GPS.Lat(i_gps);
    lon0 = input.GPS.Long(i_gps);
    gps(:,2) = (input.GPS.Lat - lat0) .* (pi / 180 * 6378100);
    gps(:,1) = -(input.GPS.Long - lon0) .* (pi / 180 * 6378100) .* cos(lat0);
    for i = 1:numel(t_gps)
        [x,y] = gpsTransform(input.GPS.Long(i), input.GPS.Lat(i), lon0, lat0);
        gps(i,1) = x;
        gps(i,2) = y;
    end

    landmark_count = min([
        size(input.Landmark.Time,1),
        size(input.Landmark.ID,1),
        size(input.Landmark.X,1),
        size(input.Landmark.Y,1),
        size(input.Landmark.Type,1)
    ]);

    t_landmarks = input.Landmark.Time(1:landmark_count,:);
    t_landmarks = reshape(t_landmarks,[numel(t_landmarks),1]);

    landmarks = zeros(numel(t_landmarks),4);
    landmarks(:,1) = reshape(input.Landmark.ID(1:landmark_count,:), [numel(t_landmarks),1]);
    landmarks(:,2) = reshape(input.Landmark.X(1:landmark_count,:), [numel(t_landmarks),1]);
    landmarks(:,3) = reshape(input.Landmark.Y(1:landmark_count,:), [numel(t_landmarks),1]);
    landmarks(:,4) = reshape(input.Landmark.Type(1:landmark_count,:), [numel(t_landmarks),1]);

    % Sort by time
    [t_landmarks, landmarks_sorted_index] = sort(t_landmarks);
    landmarks = landmarks(landmarks_sorted_index,:);

    % Filter out empty landmark measurements
    landmarks_filtered_index = landmarks(:,1) > 0;
    t_landmarks = t_landmarks(landmarks_filtered_index);
    landmarks = landmarks(landmarks_filtered_index,:);

    % Create unique ids
    landmark_id_times = zeros(0,3);
    next_landmark_id = max(landmarks(:,1)) + 1;
    for i = 1:numel(t_landmarks)
        id = landmarks(i,1);
        idx = find(landmark_id_times(:,1) == id,1);
        if (numel(idx) == 0)
            % First time seeing landmark
            landmark_id_times = [landmark_id_times; id t_landmarks(i) id];
        elseif (t_landmarks(i) - landmark_id_times(idx,2) < 10)
            % Consider this the same landmark as previously seen
            landmark_id_times(idx,2) = t_landmarks(i);
            landmarks(i,1) = landmark_id_times(idx,3);
        else
            % Consider this a new landmark
            landmark_id_times(idx,2) = t_landmarks(i);
            landmark_id_times(idx,3) = next_landmark_id;
            landmarks(i,1) = next_landmark_id;
            next_landmark_id = next_landmark_id + 1;
        end
    end

    % Create ouput structure
    processed_data = {};
    processed_data.t_odometry = t_odometry;
    processed_data.odometry = odometry;
    processed_data.t_gps = t_gps;
    processed_data.gps = gps;
    processed_data.t_landmarks = t_landmarks;
    processed_data.landmarks = landmarks;
    processed_data.ref_long = lon0;
    processed_data.ref_lat = lat0;
end

