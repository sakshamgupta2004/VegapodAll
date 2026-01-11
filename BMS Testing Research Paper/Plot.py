import pandas as pd
import matplotlib.pyplot as plt

# ------------------ Load data ------------------
df = pd.read_csv("BMS_Testing_2_Cleaned.csv")

# Ensure numeric data
for col in df.columns:
    df[col] = pd.to_numeric(df[col], errors="coerce")

# Global plot settings (paper-friendly)
plt.rcParams.update({
    "font.size": 12,
    "axes.labelsize": 14,
    "axes.titlesize": 16,
    "xtick.labelsize": 12,
    "ytick.labelsize": 12,
    "lines.linewidth": 2,
    "figure.figsize": (8, 5),
    "axes.grid": True
})

# ------------------ Voltage vs Time ------------------
plt.figure()
plt.plot(df["Time"], df["Voltage(V)"])
plt.xlabel("Time (s)")
plt.ylabel("Voltage (V)")
plt.title("Battery Voltage vs Time")
plt.tight_layout()
plt.savefig("Voltage_vs_Time.svg")
plt.show()

# ------------------ Current vs Time ------------------
plt.figure()
plt.plot(df["Time"], df["Current(ma)"])
plt.xlabel("Time (s)")
plt.ylabel("Current (mA)")
plt.title("Battery Current vs Time")
plt.tight_layout()
plt.savefig("Current_vs_Time.svg")
plt.show()

# ------------------ SoC vs Time ------------------
plt.figure()
plt.plot(df["Time"], df["SoC"])
plt.xlabel("Time (s)")
plt.ylabel("State of Charge (%)")
plt.title("State of Charge vs Time")
plt.tight_layout()
plt.savefig("SoC_vs_Time.svg")
plt.show()

# ------------------ Discharge Rate vs Time ------------------
plt.figure()
plt.plot(df["Time"], df["Discharge Rate(W)"])
plt.xlabel("Time (s)")
plt.ylabel("Discharge Power (W)")
plt.title("Discharge Power vs Time")
plt.tight_layout()
plt.savefig("Discharge_Power_vs_Time.svg")
plt.show()

# ------------------ Energy vs Time ------------------
plt.figure()
plt.plot(df["Time"], df["Energy(mWh)"])
plt.xlabel("Time (s)")
plt.ylabel("Energy (mWh)")
plt.title("Energy Consumption vs Time")
plt.tight_layout()
plt.savefig("Energy_vs_Time.svg")
plt.show()

# ------------------ Charge vs Time ------------------
plt.figure()
plt.plot(df["Time"], df["Charge(mAh)"])
plt.xlabel("Time (s)")
plt.ylabel("Charge (mAh)")
plt.title("Charge vs Time")
plt.tight_layout()
plt.savefig("Charge_vs_Time.svg")
plt.show()
