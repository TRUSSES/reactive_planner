import csv
import math

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

BAG_PATH = "rosbag2_2026_01_23-07_38_36"
ODOM_TOPIC = "/odom"
OUTPUT_CSV = "trajectory_xy.csv"

MIN_DISTANCE = 0.1   # meters

def main():

    reader = SequentialReader()

    storage_options = StorageOptions(
        uri=BAG_PATH,
        storage_id="sqlite3"
    )

    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader.open(storage_options, converter_options)

    prev_x = None
    prev_y = None

    with open(OUTPUT_CSV, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["time_sec", "x", "y"])

        while reader.has_next():
            topic, data, timestamp = reader.read_next()

            if topic != ODOM_TOPIC:
                continue

            msg = deserialize_message(data, Odometry)

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            time_sec = timestamp * 1e-9

            # First point always saved
            if prev_x is None:
                writer.writerow([time_sec, x, y])
                prev_x = x
                prev_y = y
                continue

            # Euclidean distance
            dist = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)

            if dist >= MIN_DISTANCE:
                writer.writerow([time_sec, x, y])
                prev_x = x
                prev_y = y

    print(f"Saved filtered trajectory to {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
