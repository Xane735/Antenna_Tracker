import math

import math

def calculate_azimuth(lat_gcs, lon_gcs, lat_uav, lon_uav):
    # Convert from degrees to radians
    lat_gcs = math.radians(lat_gcs)
    lon_gcs = math.radians(lon_gcs)
    lat_uav = math.radians(lat_uav)
    lon_uav = math.radians(lon_uav)

    d_lon = lon_uav - lon_gcs

    y = math.sin(d_lon) * math.cos(lat_uav)
    x = math.cos(lat_gcs) * math.sin(lat_uav) - math.sin(lat_gcs) * math.cos(lat_uav) * math.cos(d_lon)

    ang_deg = math.degrees(math.atan2(y, x))
    fin_ang_rad = (ang_deg + 360) % 360  # Normalize to 0–360°
    rounded_angle = round(fin_ang_rad, 2)
    
    print(rounded_angle)
    return rounded_angle

def calculate_elevation(lat_gcs, lon_gcs, lat_uav, lon_uav, alt_uav):
    # Convert from degrees to radians
    lat_gcs = math.radians(lat_gcs)
    lon_gcs = math.radians(lon_gcs)
    lat_uav = math.radians(lat_uav)
    lon_uav = math.radians(lon_uav)

    R = 6371e3  # Earth's radius in meters
    alt_gcs = 1  # Altitude of GCS in meters
    d_lon = lon_uav - lon_gcs
    d_lat = lat_uav - lat_gcs
    d_h = alt_uav - alt_gcs

    dist = math.acos(math.sin(lat_gcs) * math.sin(lat_uav) + 
                     math.cos(lat_gcs) * math.cos(lat_uav) * math.cos(d_lon)) * R

    if dist == 0:
        ang_deg = 90.0 if d_h > 0 else 0.0
    else:
        ang_deg = math.degrees(math.atan(d_h / dist))

    fin_ang_rad = (ang_deg + 360) % 360
    rounded_angle = round(fin_ang_rad, 2)

    print(rounded_angle)
    return rounded_angle


def simulator():

    lat_gcs = -35.3620359
    lat_uav = -35.4567829
    lon_gcs =  149.1659043
    lon_uav =  149.5629277

    for i in range(10):
        #print("Calculating azimuth")
        new_azi = calculate_azimuth(lat_gcs, lon_gcs, lat_uav, lon_uav)
        #print("Calculating elevation")
        new_elev = calculate_elevation(lat_gcs, lon_gcs, lat_uav, lon_uav)
        print(new_azi,new_elev)
        lat_gcs += lat_gcs
        lat_uav += lat_uav
        lon_gcs += lon_gcs
        lon_uav += lon_uav
   
