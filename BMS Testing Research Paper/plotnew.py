import pandas as pd
import matplotlib.pyplot as plt

# Load CSV
df = pd.read_csv("BMS_CLEANED_Final.csv")

time = df["Time(s)"]

# -----------------------------
# 1. Voltage
# -----------------------------
plt.figure()
# plt.plot(time, df["Voltage(V)"], label="Measured Voltage (V)")
plt.plot(time, df["Calculated Voltage (V)"], label="Voltage (V)")
plt.xlabel("Time (s)")
plt.ylabel("Voltage")
plt.title("Pack Voltage")
plt.legend()
plt.grid(True)
plt.show()

# -----------------------------
# 1B. Current
# -----------------------------
plt.figure()
plt.plot(time, df["Current(ma)"], label="Current (mA)")
plt.xlabel("Time (s)")
plt.ylabel("Current")
plt.title("Pack Current")
plt.legend()
plt.grid(True)
plt.show()

# -----------------------------
# 2A. State of Charge
# -----------------------------
plt.figure()
plt.plot(time, df["SoC(%)"], label="SoC (%)")
plt.xlabel("Time (s)")
plt.ylabel("SoC (%)")
plt.title("State of Charge")
plt.grid(True)
plt.show()

# -----------------------------
# 2B. Discharge Rate
# -----------------------------
plt.figure()
plt.plot(time, df["Discharge Rate(W)"], label="Discharge Rate (W)")
plt.xlabel("Time (s)")
plt.ylabel("Power (W)")
plt.title("Discharge Rate")
plt.grid(True)
plt.show()

# -----------------------------
# 2C. Energy
# -----------------------------
plt.figure()
plt.plot(time, df["Energy(mWh)"], label="Energy (mWh)")
plt.xlabel("Time (s)")
plt.ylabel("Energy (mWh)")
plt.title("Energy Consumption")
plt.grid(True)
plt.show()

# -----------------------------
# 2D. Charge
# -----------------------------
plt.figure()
plt.plot(time, df["Charge(mAh)"], label="Charge (mAh)")
plt.xlabel("Time (s)")
plt.ylabel("Charge (mAh)")
plt.title("Accumulated Charge")
plt.grid(True)
plt.show()

# -----------------------------
# 3A. Cell Voltages (Cell 1–6)
# -----------------------------
plt.figure()
for i in range(1, 7):
    plt.plot(time, df[f"Cell{i}(mV)"], label=f"Cell {i}")
plt.xlabel("Time (s)")
plt.ylabel("Voltage (mV)")
plt.title("Cell Voltages (Cell 1–6)")
plt.legend()
plt.grid(True)
plt.show()

# -----------------------------
# 3B. Cell Voltages (Cell 7–12)
# -----------------------------
plt.figure()
for i in range(7, 13):
    plt.plot(time, df[f"Cell{i}(mV)"], label=f"Cell {i}")
plt.xlabel("Time (s)")
plt.ylabel("Voltage (mV)")
plt.title("Cell Voltages (Cell 7–12)")
plt.legend()
plt.grid(True)
plt.show()

# -----------------------------
# 4A. Individual Cell Temperatures
# -----------------------------
plt.figure()
for i in range(1, 13):
    plt.plot(time, df[f"Temp{i}(°C)"], label=f"Temp {i}")
plt.xlabel("Time (s)")
plt.ylabel("Temperature (°C)")
plt.title("Individual Cell Temperatures")
plt.legend(ncol=3)
plt.grid(True)
plt.show()

# -----------------------------
# 4B. Average Temperature
# -----------------------------
temp_cols = [f"Temp{i}(°C)" for i in range(1, 13)]
df["Avg_Temp"] = df[temp_cols].mean(axis=1)

plt.figure()
plt.plot(time, df["Avg_Temp"], label="Average Temperature")
plt.xlabel("Time (s)")
plt.ylabel("Temperature (°C)")
plt.title("Average Battery Temperature")
plt.grid(True)
plt.show()
