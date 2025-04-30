import math

def calculate_azimuth(lat_gcs, lon_gcs, lat_uav, lon_uav):
    # Convert from degrees to radians
    lat_gcs = math.radians(lat_gcs) # Latitude of GCS
    lon_gcs = math.radians(lon_gcs) # Longitude of GCS
    lat_uav = math.radians(lat_uav) # Latitude of UAV
    lon_uav = math.radians(lon_uav) # Longitude of UAV

    d_lon = lon_uav - lon_gcs

    y = math.sin(d_lon) * math.cos(lat_uav)
    x = (math.cos(lat_gcs) * math.sin(lat_uav))-(math.sin(lat_gcs) * math.cos(lat_uav) * math.cos(d_lon))

    ang_deg = math.atan2(y, x)

    # Convert from radians to degrees and normalize to 0–360°
    ang_rad = math.degrees(ang_deg)
    fin_ang_rad = (ang_rad + 360) % 360 #Normalizing
    #print(fin_ang_rad)

    return fin_ang_rad

def calculate_elevation(lat_gcs, lon_gcs, lat_uav, lon_uav, alt_uav):
    # Convert from degrees to radians
    lat_gcs = math.radians(lat_gcs) # Latitude of GCS
    lon_gcs = math.radians(lon_gcs) # Longitude of GCS
    lat_uav = math.radians(lat_uav) # Latitutde of UAV
    lon_uav = math.radians(lon_uav) # Longitude of UAV
    R = 6.371*math.pow(10,3)        # Approx Radius of Earth
    alt_uav = 100                   # Altitude of UAV
    alt_gcs = 6                     # Altitude of GCS
    
    d_lon = lon_uav - lon_gcs
    d_lat = lat_uav - lat_gcs
    d_h = alt_uav - alt_gcs

    dist = math.acos((math.sin(lat_gcs)*math.sin(lat_uav))+ (math.cos(lat_gcs)*math.cos(lat_uav)*math.cos(d_lon)))* R
    ang_deg = math.atan(d_h/dist)
    ang_rad = math.degrees(ang_deg)
    fin_ang_rad = (ang_rad + 360) % 360
    #print(fin_ang_rad)

    return fin_ang_rad

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

simulator()    
