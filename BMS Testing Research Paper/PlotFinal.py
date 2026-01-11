import pandas as pd
import matplotlib.pyplot as plt

# -----------------------------
# Load data
# -----------------------------
df = pd.read_csv("BMS_CLEANED_Final.csv")

# -----------------------------
# Pre-compute average temperature
# -----------------------------
temp_cols = [f"Temp{i}(°C)" for i in range(1, 13)]
df["Avg_Temp"] = df[temp_cols].mean(axis=1)

# -----------------------------
# 1. Pack Voltage vs SoC
# -----------------------------
plt.figure()
plt.plot(df["SoC(%)"], df["Calculated Voltage (V)"])
plt.xlabel("State of Charge (%)")
plt.ylabel("Voltage (V)")
plt.title("Pack Voltage vs State of Charge")
plt.grid(True)
plt.show()

# -----------------------------
# 2. Current vs Pack Voltage
# -----------------------------
plt.figure()
plt.plot(df["Calculated Voltage (V)"], df["Current(ma)"])
plt.xlabel("Voltage (V)")
plt.ylabel("Current (mA)")
plt.title("Current vs Voltage")
plt.grid(True)
plt.show()

# -----------------------------
# 3. Discharge Power vs SoC
# -----------------------------
plt.figure()
plt.plot(df["SoC(%)"], df["Discharge Rate(W)"])
plt.xlabel("State of Charge (%)")
plt.ylabel("Power (W)")
plt.title("Discharge Rate vs State of Charge")
plt.grid(True)
plt.show()

# -----------------------------
# 4. Energy Consumed vs SoC
# -----------------------------
plt.figure()
plt.plot(df["SoC(%)"], df["Energy(mWh)"])
plt.xlabel("State of Charge (%)")
plt.ylabel("Energy (mWh)")
plt.title("Energy Consumption vs SoC")
plt.grid(True)
plt.show()

# -----------------------------
# 5. Charge Throughput vs SoC
# -----------------------------
plt.figure()
plt.plot(df["SoC(%)"], df["Charge(mAh)"])
plt.xlabel("State of Charge (%)")
plt.ylabel("Charge (mAh)")
plt.title("Charge Throughput vs SoC")
plt.grid(True)
plt.show()

# -----------------------------
# 6A. Cell Voltages (Cell 1–6) vs SoC
# -----------------------------
plt.figure()
for i in range(1, 7):
    plt.plot(df["SoC(%)"], df[f"Cell{i}(mV)"], label=f"Cell {i}")
plt.xlabel("SoC (%)")
plt.ylabel("Cell Voltage (mV)")
plt.title("Cell Voltages (1–6) vs SoC")
plt.legend()
plt.grid(True)
plt.show()

# -----------------------------
# 6B. Cell Voltages (Cell 7–12) vs SoC
# -----------------------------
plt.figure()
for i in range(7, 13):
    plt.plot(df["SoC(%)"], df[f"Cell{i}(mV)"], label=f"Cell {i}")
plt.xlabel("SoC (%)")
plt.ylabel("Cell Voltage (mV)")
plt.title("Cell Voltages (7–12) vs SoC")
plt.legend()
plt.grid(True)
plt.show()

# -----------------------------
# 7. Average Temperature vs SoC
# -----------------------------
plt.figure()
plt.plot(df["SoC(%)"], df["Avg_Temp"])
plt.xlabel("State of Charge (%)")
plt.ylabel("Average Temperature (°C)")
plt.title("Average Battery Temperature vs SoC")
plt.grid(True)
plt.show()

# -----------------------------
# 8. Temperature vs Pack Voltage
# -----------------------------
plt.figure()
plt.plot(df["Calculated Voltage (V)"], df["Avg_Temp"])
plt.xlabel("Voltage (V)")
plt.ylabel("Average Temperature (°C)")
plt.title("Average Temperature vs Voltage")
plt.grid(True)
plt.show()
