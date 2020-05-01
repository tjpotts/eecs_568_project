
%Define JSON Files importeted from HERE the first needts to be a partion
%from the topology geometry layer
%https://platform.here.com/data/hrn:here:data::olp-here:rib-2/topology-geometry/schema}}
%The second file should be from the advanced navigation layer 
%text= fileread('C:\Users\RJONE646\Downloads\19743895-decoded.json');
text = fileread('C:\Users\RJONE646\Documents\AVMasters\Naval568\Project\HereDataJsons\20095973_gps-decoded.json');
roadSegmentData = jsondecode(text);

%text= fileread('C:\Users\RJONE646\Downloads\19743895-decoded (2).json');
text = fileread('C:\Users\RJONE646\Documents\AVMasters\Naval568\Project\HereDataJsons\20095973_signs-decoded.json');
signData = jsondecode(text);

% Define Enum values for signs
enumMap.overtaking_restriction = [];
enumMap.protected_overtaking = [];
enumMap.railway_crossing = -1;
enumMap.speed_limit.value.V8 = -13%5
enumMap.speed_limit.value.V16 = -12%10
enumMap.speed_limit.value.V24 = -2%15
enumMap.speed_limit.value.V32 = -14%20
enumMap.speed_limit.value.V40 = -3%25
enumMap.speed_limit.value.V48 = -4%30
enumMap.speed_limit.value.V56 = -15%35
enumMap.speed_limit.value.V64 = -5%40
enumMap.speed_limit.value.V72 = -6%45
enumMap.speed_limit.value.V80 = -16%50
enumMap.speed_limit.value.V89 = -7%55
enumMap.speed_limit.value.V97 = -17%60
enumMap.speed_limit.value.V105 = -8%65
enumMap.speed_limit.value.V113 = -18%70
enumMap.speed_limit.value.V121 = -19%75
enumMap.speed_limit.value.V129 = -20%80
enumMap.traffic_signal =[];
enumMap.traffic_sign.traffic_sign_type.RAILWAY_CROSSING_UNPROTECTED = -10
enumMap.traffic_sign.traffic_sign_type.ANIMAL_CROSSING = -11
enumMap.traffic_sign.traffic_sign_type.LANE_MERGE_RIGHT = 152;
enumMap.traffic_sign.traffic_sign_type.LANE_MERGE_LEFT = 146;
enumMap.traffic_sign.traffic_sign_type.SHARP_CURVE_LEFT = 136;
enumMap.traffic_sign.traffic_sign_type.SHARP_CURVE_RIGHT = -24;
enumMap.traffic_sign.traffic_sign_type.WINDING_ROAD_STARTING_LEFT = -25;
enumMap.traffic_sign.traffic_sign_type.WINDING_ROAD_STARTING_RIGHT = -26;
enumMap.traffic_sign.traffic_sign_type.STEEP_HILL_DOWNWRADS = -27;
enumMap.traffic_sign.traffic_sign_type.STOP_SIGN = 210;
enumMap.traffic_sign.traffic_sign_type.SLIPPERY_ROAD = 160;
enumMap.traffic_sign.traffic_sign_type.SCHOOL_ZONE = -30;
enumMap.traffic_sign.traffic_sign_type.PEDESTRIAN_CROSSING = 150;
enumMap.traffic_sign.traffic_sign_type.YIELD = 168;
enumMap.traffic_sign.traffic_sign_type.NO_TURN_ON_RED = -46;
enumMap.traffic_sign.traffic_sign_type.RAILWAY_CROSSING_PROTECTED = -34;
enumMap.traffic_sign.traffic_sign_type.ROAD_NARROWS = 156;
enumMap.traffic_sign.traffic_sign_type.ICY_CONDITIONS = -36

signDataOut.TypeArray = [];
signDataOut.SegmentAnchorIndexArray = [];
j=0;
%Build Sign and Anchor Arrays

%signData.overtaking_restriction = [];
%signData.protected_overtaking = [];
try
    [m,~] = size(signData.railway_crossing.segment_anchor_index)
    if m > 0
        signDataOut.TypeArray(j+1:m+j) = enumMap.railway_crossing;
        signDataOut.SegmentAnchorIndexArray(j+1:m+j) = signData.railway_crossing.segment_anchor_index+1;
        j=j+m;
        signDataOut
    end
catch
    disp('Failure1')
end
%%%Posted Speed Limits do not seem to corelate well to the actual signs
% try
%     [m,~] = size(signData.speed_limit)
%     if m > 0
%         for i = 1:m
%             i
%             mm = length(signData.speed_limit(i).segment_anchor_index)
%             signDataOut.TypeArray(j+1:j+mm) = enumMap.speed_limit.value.(sprintf('V%d',signData.speed_limit(i).value));
%             signDataOut.SegmentAnchorIndexArray(j+1:j+mm) = signData.speed_limit(i).segment_anchor_index+1;
%             j=j+mm;
%             signDataOut
%         end
%     end
% catch
%     disp('Failure2')
% end        
    
%signData.traffic_signal =[];
try
    [m,~] = size(signData.traffic_sign)
    if m > 0
        for i = 1:m
            i
            mm = length(signData.traffic_sign(i).segment_anchor_index)
            signDataOut.TypeArray(j+1:j+mm) = enumMap.traffic_sign.traffic_sign_type.(signData.traffic_sign(i).traffic_sign_type);
            signDataOut.SegmentAnchorIndexArray(j+1:j+mm) = signData.traffic_sign(i).segment_anchor_index+1;
            j=j+mm;
            signDataOut
        end
    end
catch
    disp('Failure3')
end        

removeIndex = [];
%preallocate arrays
[~,m] = size(signDataOut.TypeArray);
signDataOut.segmentID = zeros(1,m);
signDataOut.startOffset = zeros(1,m);
signDataOut.endOffset = zeros(1,m);

for i =1:m
    segment = signData.segment_anchor(signDataOut.SegmentAnchorIndexArray(i)).oriented_segment_ref.segment_ref.identifier;
    signDataOut.segmentID(i) = str2num(segment(17:end));
    try
    signDataOut.startOffset(i) = signData.segment_anchor(signDataOut.SegmentAnchorIndexArray(i)).first_segment_start_offset.value;
    catch
        signDataOut.startOffset(i) = nan;
        removeIndex(end+1) = i;
    end
    try
    signDataOut.endOffset(i) = signData.segment_anchor(signDataOut.SegmentAnchorIndexArray(i)).last_segment_end_offset.value;
    catch
        signDataOut.endOffset(i) = nan;
    end
    
end

signDataOut.SegmentAnchorIndexArray(removeIndex) = [];
signDataOut.TypeArray(removeIndex) = [];
signDataOut.segmentID(removeIndex) = [];
signDataOut.startOffset(removeIndex) = [];
signDataOut.endOffset(removeIndex) = [];


%Get GPS Location


sparseIndex = zeros(m,1)
%create a sparse matrix for indexing
[m,~] = size(roadSegmentData.segment)
for i = 1:m
    segment = roadSegmentData.segment(i).identifier;
    sparseIndex(i) = str2num(segment(17:end));
end
sparseIndexingArray = sparse(sparseIndex,ones(m,1),1:m);


[~,m] = size(signDataOut.TypeArray);
signDataOut.latLong = zeros(m,2)
for i=1:m
    j=sparseIndexingArray(signDataOut.segmentID(i));
    geo = roadSegmentData.segment(j).geometry;
    [signDataOut.latLong(i,1),signDataOut.latLong(i,2)] = getLatLong(signDataOut.startOffset(i),geo.point);
    
    
end



function [latPoint,longPoint] = getLatLong(startOffset,geoPoints)

[m,~] = size(geoPoints);
lat = zeros(m,1);
long = zeros(m,1);
segmentedPointIndex= zeros(m-1,1);
for ii =1:m
    lat(ii) = geoPoints(ii).latitude;
    long(ii) = geoPoints(ii).longitude;
end
%lat=lat(end:-1:1);
%long=long(end:-1:1);


segLengths=(diff(lat).^2+diff(long).^2).^.5;
normalizedSegLengths = segLengths./sum(segLengths);

for ii =1:m-1
segmentedPointIndex(ii) =  sum(normalizedSegLengths(1:ii));
end
segmentedPointIndexp0 = [0;segmentedPointIndex];

ind = find(segmentedPointIndex>=startOffset-.00001,1);


latLong = [lat(ind);long(ind)] + ([lat(ind+1);long(ind+1)] - [lat(ind);long(ind)])*(startOffset-segmentedPointIndexp0(ind)); 

latPoint = latLong(1);
longPoint = latLong(2);


end

