close all

HEREmap_xy = zeros(size(signDataOut_03.latLong,1),2);

for i = 1:size(signDataOut_03.latLong,1)
    [tempx, tempy] = gpsTransform(signDataOut_03.latLong(i,2), signDataOut_03.latLong(i,1), vehicle_input.ref_long, vehicle_input.ref_lat);
    HEREmap_xy(i,:) = [tempx,tempy];
end
% n = size(signDataOut_03.latLong, 1);
% heremap.Location = [signDataOut_03.latLong zeros(n,1)];
% heremap.Label = signDataOut_03.TypeArray';

%transform source_data and create source cloud
source_cloud.Location = [sim_output.local_map(:,2:3) zeros(size(sim_output.local_map,1),1)];
source_cloud.Label = sim_output.local_map(:,4);
SLAM_trajectory = [sim_output.trajectory zeros(size(sim_output.trajectory,1),1)];
theta = deg2rad(0);
R = [cos(theta) -sin(theta) 0;
    sin(theta) cos(theta) 0;
    0 0 1];
x_trans = 0;
y_trans = 0;
source_cloud.Location = (R * source_cloud.Location')';
source_cloud.Location(:,1) = source_cloud.Location(:,1) + x_trans;
source_cloud.Location(:,2) = source_cloud.Location(:,2) + y_trans;

SLAM_trajectory = (R * SLAM_trajectory')';
SLAM_trajectory(:,1) = SLAM_trajectory(:,1) + x_trans;
SLAM_trajectory(:,2) = SLAM_trajectory(:,2) + y_trans;

target_cloud.Location = [HEREmap_xy zeros(size(HEREmap_xy,1),1)];
target_cloud.Label = signDataOut_03.TypeArray';

%remove label 46
idx_46 = find(target_cloud.Label == 46);
target_cloud.Label(idx_46,1) = -target_cloud.Label(idx_46,1);

%trim HEREmap
buffer = 1000;
xrange = [min(sim_output.local_map(:,2)) - buffer, max(sim_output.local_map(:,2)) + buffer];
yrange = [min(sim_output.local_map(:,3)) - buffer, max(sim_output.local_map(:,3)) + buffer];

target_cloud = trimMap(target_cloud, xrange, yrange);


%loop through different initial transforms
x = [-200 -100 0 100 200];
y = [-200 -100 0 100 200];
angle = deg2rad([-10 5 0 5 10]);
initT = zeros(4,4);
allowableError = 100;
cur_avg_error = allowableError;
curTransform = [];
for i = 1:length(x)
    for j = 1:length(y)
        for k = 1:length(angle)
            text = ['x: ' num2str(x(i)) ' y: ' num2str(y(j)) ' angle: ' num2str(angle(k))];
            display(text);
            initT = [cos(angle(k)) -sin(angle(k)) 0 x(i);
                     sin(angle(k)) cos(angle(k))  0 y(j);
                     0            0               1     0;
                     0 0 0 1];
            sicp_obj = sicp(target_cloud, source_cloud, initT);
            correlation = sicp_obj.getCorrectedPose();
            source_cloud_new = sicp_obj.transformedSource;
            if sicp_obj.converged
                for q = 1:size(correlation,1)
                    error_location = [error_location; source_cloud_new(correlation(q,1),1:2) target_cloud.Location(correlation(q,2),1:2)];
                    error = [error; norm(error_location(q,1:2) - error_location(q,3:4))];
                end
                avg_error = mean(error);
                if avg_error < cur_avg_error
                    curTransform = sicp_obj.T;
                end
            end
        end
    end
end

%perform ICP
% sicp_obj = sicp(target_cloud, source_cloud, eye(4));
% correlation = sicp_obj.getCorrectedPose();


%3d with labels as z
% hold on
% plot3(source_cloud.Location(correlation(:,1),1), source_cloud.Location(correlation(:,1),2), source_cloud.Label(correlation(:,1)), 'xg'); 
% plot3(target_cloud.Location(correlation(:,2),1), target_cloud.Location(correlation(:,2),2), target_cloud.Label(correlation(:,1)), 'ob'); 
% plot3(sim_output.trajectory(:,1), sim_output.trajectory(:,2), zeros(size(sim_output.trajectory,1),1), 'r');

%2d with labels labelled
figure
hold on
s_labels = {};
t_labels = {};
for i = 1:size(correlation,1)
   s_labels{i,1} = num2str(source_cloud.Label(correlation(i,1)));
   t_labels{i,1} = num2str(target_cloud.Label(correlation(i,2)));
end
%observed landmark
plot(source_cloud.Location(correlation(:,1),1), source_cloud.Location(correlation(:,1),2), 'og'); 
%here map landmark
plot(target_cloud.Location(correlation(:,2),1), target_cloud.Location(correlation(:,2),2), 'ob');
labelpoints(source_cloud.Location(correlation(:,1),1), source_cloud.Location(correlation(:,1),2), s_labels);
labelpoints(target_cloud.Location(correlation(:,2),1), target_cloud.Location(correlation(:,2),2), t_labels);
%correlating landmarks
for i = 1:size(correlation,1)
    plot([source_cloud.Location(correlation(i,1),1) target_cloud.Location(correlation(i,2),1)], [source_cloud.Location(correlation(i,1),2) target_cloud.Location(correlation(i,2),2)])
end
%plot(sim_output.local_map(:,2), sim_output.local_map(:,3), 'o')
plot(sim_output.trajectory(:,1), sim_output.trajectory(:,2), 'r');
plot(SLAM_trajectory(:,1), SLAM_trajectory(:,2), 'b');

% figure
% plot(target_cloud.Location(:,1), target_cloud.Location(:,2), 'o')
% plot(sim_output.local_map(:,2), sim_output.local_map(:,3), 'o')


%find and plot all points in HEREmap with specified label
label_num = 46;
idx = find(target_cloud.Label(:,1) == label_num);
%figure
plot(target_cloud.Location(idx, 1), target_cloud.Location(idx, 2), 'xm');

label_num = 150;
idx = find(target_cloud.Label(:,1) == label_num);
%figure
plot(target_cloud.Location(idx, 1), target_cloud.Location(idx, 2), 'xr');

label_num = 210;
idx = find(target_cloud.Label(:,1) == label_num);
%figure
plot(target_cloud.Location(idx, 1), target_cloud.Location(idx, 2), 'xy');


%plot new trajectory
newTrajectory = (sicp_obj.T(1:3,1:3) * SLAM_trajectory')';
newTrajectory(:,1) = newTrajectory(:,1) + sicp_obj.T(1,4);
newTrajectory(:,2) = newTrajectory(:,2) + sicp_obj.T(2,4);
plot(newTrajectory(:,1), newTrajectory(:,2), 'g');

legend('local SLAM landmarks', 'target HERE landmarks');


%apply transfromation to source landmark map
newSource = (sicp_obj.T(1:3,1:3) * source_cloud.Location')';
newSource(:,1) = newSource(:,1) + sicp_obj.T(1,4);
newSource(:,2) = newSource(:,2) + sicp_obj.T(2,4);

figure
plot(newSource(correlation(:,1),1), newSource(correlation(:,1),2), 'or');
hold on
plot(target_cloud.Location(correlation(:,2),1), target_cloud.Location(correlation(:,2),2), 'og');

sicp_obj.T

error_location=[];
error=[];
for i = 1:size(correlation,1)
    error_location = [error_location; newSource(correlation(i,1),1:2) target_cloud.Location(correlation(i,2),1:2)];
    error = [error; norm(error_location(i,1:2) - error_location(i,3:4))];
end

mean(error)












%EXTRA STUFF THAT I CAN"T REMEMBER WHAT IT DOES

% for i = 1:length(sicp_obj.source.Label)
%     if ~isempty(find(sicp_obj.target.Label == sicp_obj.source.Label(i)))
%         i
%     end
%      
% end
% 
% coord = [41.894 -87.6268];
% latLong = [testListArray.GPS.Lat testListArray.GPS.Long];
% 
% dist = sqrt(sum((latLong - coord).^2,2));

% figure
% plot(sim_output.trajectory(:,1), sim_output.trajectory(:,2))
% title("SLAM output");
% figure
% plot(sicp_obj.newLocation(:,1), sicp_obj.newLocation(:,2));
% title("ICP output");



% startTime = 780;  
% endTime = 780+20*60;
% startInd = find(testListArray.Odometry.VehicleSpeedTime > startTime,1);
% endInd = find(testListArray.Odometry.VehicleSpeedTime > endTime,1);
% len = endInd-startInd+1;  
% x = zeros(len,1);  
% y = zeros(len,1);  
% curPose =squeeze(testListArray.Odometry.Pose(startInd-1,:,:));  
% for i = startInd:endInd  
%     curPose = curPose*squeeze(testListArray.Odometry.Pose(i,:,:));
%     x(i-startInd+1) = curPose(1,3);  
%     y(i-startInd+1) = curPose(2,3);  
% end


