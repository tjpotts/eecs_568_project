classdef sicp < handle
    properties
        %inital guess for transform
        initialT
        
        %target and source point clouds
        target
        source
        
        %output transform, new pose, and error
        T
        transformedSource
        
        %true if converge, flase if not converged
        converged
        
        %correlation matrix between transformed source cloud and target
        %cloud
        correlation
        
        %error
        errorLocation
        avgError
       
    end
    
    methods
        function obj = sicp(targetCloud, sourceCloud, initialTransform)
            if nargin ~= 3
                error('input arguments do not match');
            end
            obj.initialT = initialTransform;
            obj.target = targetCloud;
            obj.source = sourceCloud;
%             obj.err = zeros(size(targetCloud.Location,1));
        end
        
        function getCorrectedPose(obj)
            %TODO: look into how to get a better initial guess
            %Idea: Compute the beginning error after some amount of steps,
            %and trim based on that
            
            [obj.T, idx, obj.converged] = sicp_SE2(obj.target, obj.source, obj.initialT);
            
            obj.transformedSource = obj.source.Location * obj.T(1:3,1:3)';
            obj.transformedSource(:,1) = obj.transformedSource(:,1) + obj.T(1,4);
            obj.transformedSource(:,2) = obj.transformedSource(:,2) + obj.T(2,4);
            obj.transformedSource(:,3) = obj.transformedSource(:,3) + obj.T(3,4);

            obj.correlation = [];
            for i = 1:size(idx,1)
                if idx(i) ~= 0
%                     obj.err(i,:) = obj.transformedSource(i,:) - obj.target.Location(idx(i),1:3);
                    %correlation from source landmark index to target
                    %landmark index
                    obj.correlation = [obj.correlation; i idx(i)];
                end
            end
            
            if size(obj.correlation,1) < 6
               obj.converged = false; 
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
            if size(T,1) ~= 4 || size(T,2)~= 4
                error('initial transfrom must be 4x4');
            end
            obj.initialT = T;
        end
        
        %calculate error between correlated classes
        function calculateError(obj)
            obj.errorLocation=[];
            error=[];
            if ~isempty(obj.correlation)
                for i = 1:size(obj.correlation,1)
                    obj.errorLocation = [obj.errorLocation; obj.transformedSource(obj.correlation(i,1),1:2) obj.target.Location(obj.correlation(i,2),1:2)];
                    error = [error; norm(obj.errorLocation(i,1:2) - obj.errorLocation(i,3:4))];
                end
                obj.avgError = mean(error);
            else
                obj.avgError = 10000;
            end  
         end
    end
end