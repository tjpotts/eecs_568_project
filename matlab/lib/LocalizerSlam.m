import gtsam.*

classdef LocalizerSlam
    properties
    end
    methods
        % Initialize the SLAM algorithm
        % initialPose: SE2 pose
        function initialize(obj, initialPose)
            % ISAM2 optimizer
            obj.isam = ISAM2;

            % Objects to hold new factors and initial estimates added to the factor graph
            obj.newFactors = NonlinearFactorGraph;
            obj.initialEstimates = Values;

            % Index number to use for next odometry measurement
            obj.odometryIndex = 0;
        end

        % Adds a new odometry measurement to the pose graph
        % t: timestamp of measurement
        % meas: [3x3] SE2 transformation
        function addOdometry(obj, t, meas)
            % TODO: Create odometry factor
            % TODO: Calculate initial estimate based on previous estimate
            % TODO: Add to newFactors/initialEstimates objects
        end

        % Adds a new GPS measurement to the pose graph
        % t: timestamp of measurement
        % meas: [X,Y] coordinates of GPS measurement
        function addGps(obj, t, meas)
            % TODO: Create gps factor
            % TODO: Add to newFactors object
        end

        % Adds a new landmark observation to the pose graph
        % t: timestamp of measurement
        % meas: [Nx4] data of landmark observations at current timestep
        % meas(n,:) [ID,X,Y,Type] data for landmark observation n
        function addLandmarks(obj, t, meas)
            % TODO: Add landmark factors
            % TODO: Calculate the initial estimate based on previous estimate if exists, otherwised based on prevous pose
            % TODO: Add to newFactors/initialEstimates objects
        end

        % Run the optimizer
        function optimize(obj)
            % Run ISAM2
            isam.update(obj.newFactors,obj.initialEstimates)

            % Reset the newFactors and initialEstimates objects
            obj.newFactors = NonlinearFactorGraph
            obj.initialEstimates = Values;
        end

        % Get the current pose estimate of the vehicle
        % returns: SE2 pose
        function pose = getPoseEstimate(obj)
            result = isam.calculateEstimate()
            gtPose = result.getPose2At(obj.odometryIndex)
            % TODO: Convert from gtsam type to 3x3 matrix
        end

        % Get the current estimated trajectory of the vehicle
        % returns: [Nx3x3] matrics of stacked SE2 poses
        function traj = getTrajectoryEstimate(obj)
            % TODO: Implement
        end

        % Get the current estimate of the landmark map
        % returns: Nx3 matrix of landmark estimates
        % map(n,:) = [X,Y,Type] for landmark n
        function map = getMapEstimate(obj)
            % TODO: Implemeent
        end
    end
end

