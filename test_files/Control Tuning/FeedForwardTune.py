import csv

import matplotlib.pyplot as plt
import numpy as np

# TEST DATA ONLY. WE WILL GET CSV AND THEN INPUT TO THIS FILE LATER
velocity  = []
voltage = []

# Read data from CSV file
with open("FeedForwardData.csv", "r") as file:
    reader = csv.DictReader(file)

    for row in reader:

        velocity.append(
            float(row["velocity"])
        )

        voltage.append(
            float(row["voltage"])
        )

# Linear Regression to get u = kS * sign(v) + kV * v
a,b = np.polyfit(velocity,voltage, deg = 1)

plt.scatter(velocity,voltage)
plt.xlabel("Velocity (cm/s)")
plt.ylabel("Voltage (milivolts)")

x_line = np.linspace(0, max(velocity), 100)
y_line = a * x_line + b

plt.plot(x_line, y_line)

plt.scatter(0, b, color='red', label='kS (intercept)')
plt.legend()

a = round(a, 3)
b = round(b, 3)

print(f"The line is y = {a}x + {b}")
print(f"kS = {b}")
print(f"kV = {a}")

# Save constants
with open("Config.txt", "w") as file:
    file.write(f"kS: {b}\n")
    file.write(f"kV: {a}\n")
    
plt.show()

