import csv
import azi_elev_5 as tracker

INPUT_CSV = "Test_bench.csv"
OUTPUT_CSV = "gps_with_servo_angles.csv"
GEAR_RATIO = 2.0  # 2:1 gear ratio

def process_csv(input_path, output_path):
    with open(input_path, "r") as infile, open(output_path, "w", newline="") as outfile:
        reader = csv.DictReader(infile)
        fieldnames = reader.fieldnames + [
            "azimuth", "elevation",
            "adjusted_azimuth", "adjusted_elevation",
            "servo_azimuth_input", "servo_elevation_input",
            "horizontal_distance", "slant_range", "altitude_difference", "bearing"
        ]
        writer = csv.DictWriter(outfile, fieldnames=fieldnames)
        writer.writeheader()

        for i, row in enumerate(reader):
            try:
                base_lat = float(row["base_lat"])
                base_lon = float(row["base_lon"])
                base_alt = float(row["base_alt"])
                drone_lat = float(row["drone_lat"])
                drone_lon = float(row["drone_lon"])
                drone_alt = float(row["drone_alt"])

                info = tracker.get_tracking_info(
                    base_lat, base_lon, base_alt,
                    drone_lat, drone_lon, drone_alt
                )

                if info:
                    # Add servo input angles (adjusted for gear ratio)
                    info["servo_azimuth_input"] = round(info["adjusted_azimuth"] / GEAR_RATIO, 2)
                    info["servo_elevation_input"] = round(info["adjusted_elevation"] / GEAR_RATIO, 2)

                    row.update(info)
                else:
                    # Fill empty if calculation failed
                    row.update({key: None for key in fieldnames if key not in row})

                writer.writerow(row)

            except Exception as e:
                print(f"Row {i+1} skipped due to error: {e}")
                continue

    print(f"\n✅ Process complete. Output saved to: '{output_path}'")

if __name__ == "__main__":
    process_csv(INPUT_CSV, OUTPUT_CSV)
