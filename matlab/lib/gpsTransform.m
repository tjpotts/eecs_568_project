function [x,y] = gpsTransform(long, lat, ref_long, ref_lat)
    earth_radius = 6378100; % meters
    x = distance(ref_lat, ref_long, ref_lat, long) * (pi / 180 * earth_radius);
    if (long < ref_long)
        x = -x;
    end
    y = distance(ref_lat, ref_long, lat, ref_long) * (pi / 180 * earth_radius);
    if (lat < ref_lat)
        y = -y;
    end
end

