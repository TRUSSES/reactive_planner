import csv
import matplotlib.pyplot as plt

CSV_FILE = "trajectory_xy.csv"

def main():

    x_vals = []
    y_vals = []
    t_vals = []

    with open(CSV_FILE) as f:
        reader = csv.DictReader(f)

        for row in reader:
            t_vals.append(float(row["time_sec"]))
            x_vals.append(float(row["x"]))
            y_vals.append(float(row["y"]))

    plt.figure()
    plt.plot(x_vals, y_vals, marker="o")

    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Robot Trajectory")

    plt.axis("equal")
    plt.grid(True)

    plt.show()

if __name__ == "__main__":
    main()