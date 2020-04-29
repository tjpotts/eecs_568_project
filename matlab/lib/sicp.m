classdef sicp < handle
    properties
        %inital guess for transform
        initialT
        
        %target and source point clouds
        target
        source
        trimmedTarget;
        
        %output transform, new pose, and error
        T
        newLocation
        
        %points with no matching semantic class are sent to -1
        err
       
    end
    
    methods
        function obj = sicp(targetCloud, sourceCloud, initialTransform)
            if nargin ~= 3
                error('input arguments do not match');
            end
            obj.initialT = initialTransform;
            obj.target = targetCloud;
            obj.source = sourceCloud;
            obj.err = zeros(size(targetCloud.Location,1));
        end
        
        function getCorrectedPose(obj)
            %TODO: look into how to get a better initial guess
            %Idea: Compute the beginning error after some amount of steps,
            %and trim based on that
            
            [obj.T, idx] = sicp_SE2(obj.target, obj.source, obj.initialT);
            
            obj.newLocation = obj.source.Location * obj.T(1:3,1:3)';
            obj.newLocation(:,1) = obj.newLocation(:,1) + obj.T(1,4);
            obj.newLocation(:,2) = obj.newLocation(:,2) + obj.T(2,4);
            obj.newLocation(:,3) = obj.newLocation(:,3) + obj.T(3,4);
            obj.err = zeros(size(idx,1),3)-1;
            for i = 1:size(idx,1)
                if idx(i) ~= 0
                    obj.err(i,:) = obj.newLocation(i,:) - obj.target.Location(idx(i),1:3);
                end
            end
        end  
        
        %update source cloud
        function updateSource(obj,source_cloud)
            %Todo: add error checking for size
            obj.source = source_cloud;
        end
        
        %update target cloud
        function updateTarget(obj, target_cloud)
            %Todo: add error checking for size
            obj.target = target_cloud;        
        end
        
        %update initial transform
        function updateInitialT(obj, T)
            if size(T,1) ~= 4 | size(T,2)~= 4
                error('initial transfrom must be 4x4');
            end
            obj.initialT = T;
        end
        
        %trim the global landmark map to the location where the local map
        %is
        %takes an x and y range of the current location, adds an offset,
        %and then returns all global landmarks within that range
        %range will need to be computed before being passed in
%         function trimGlobalMap(obj, globalMap, xRange, yRange)
%             n = size(globalMap.Location, 1);
%             idx = zeros(n,1);
% 
%             for i = 1:n
%                 inXRange = globalMap.Location(i,1) >= xRange(1) & globalMap.Location(i,1) <= xRange(2);
%                 inYRange = globalMap.Location(i,2) >= yRange(1) & globalMap.Location(i,2) <= yRange(2);
%                 idx(i) = inXRange & inYRange;
%             end
% 
%             trimmedTarget.Location = globalMap.Location(idx == 1,:);
%             trimmedTarget.Label = globalMap.Label(idx == 1);
%         end
        
    end
end