# QDevice x-io Technologies x-IMU3

[![Technosport amU](https://img.shields.io/badge/Technosport-amU%20%7C%20ISM-002D62?style=flat-square)](https://github.com/Technosport)
[![QDevice](https://img.shields.io/badge/QTM-QDevice%200.10-009EE2?style=flat-square)](../../releases)
[![Windows](https://img.shields.io/badge/Windows-x64%20Wi--Fi%20%7C%20USB-26A669?style=flat-square)](#)

> **Qualisys Track Manager integration plugin for x-io Technologies x-IMU3 sensors.**  
> *Developed at **Technosport / ISM** (UMR 7287 CNRS — Aix-Marseille Université) • [**Releases**](../../releases)*

---

## 🚀 Getting Started

1. Close **Qualisys Track Manager (QTM)**.
2. Download and run the latest installer from **[Releases](../../releases)**.
3. Open **QTM > Project Options > Input Devices > Add Device...** and enable **xio Technologies xIMU3**.
4. Connect the x-IMU3 units over Wi-Fi (or USB), then go to **IMUs > xio Technologies xIMU3**, click **Locate Devices**, **Synchronize Settings**, and **OK**.

---

## 🛠️ Channels & Options

| Channel / Option | Description |
| :--- | :--- |
| **`Quat W / X / Y / Z`** (`400 Hz`) | Quaternion orientation |
| **`Gyro X / Y / Z`** (`deg/s`) & **`Acc X / Y / Z`** (`g`) | Inertial measurements |
| **`Sync Edge`** (`0 / 1`) | Hardware BNC sync edges (`serial_mode = 4`, firmware `v2.6.0+`) |
| **`Battery Pct`** & **`Rssi Pct`** (`%`) | Battery level and Wi-Fi signal strength (`Include Diagnostics`, enabled by default) |
| **`Mag X / Y / Z`** (`uT`, optional) | Magnetometer (`Include Magnetometer`) |
| **`Sync Only`** *(per IMU)* | Expose only `Sync Edge` on the base sync IMU |
| **`Require BNC Hardware Sync`** | Enforce hardware BNC sync and raise an error if missing (unchecked = Wi-Fi sync) |
