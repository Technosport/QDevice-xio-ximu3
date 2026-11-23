# QDevice x-io Technologies x-IMU3

[![Technosport amU](https://img.shields.io/badge/Technosport-amU%20%7C%20ISM-002D62?style=flat-square)](https://github.com/Technosport)
[![QDevice](https://img.shields.io/badge/QTM-QDevice%200.10-009EE2?style=flat-square)](../../releases)
[![Windows](https://img.shields.io/badge/Windows-x64%20Wi--Fi%20%7C%20USB-26A669?style=flat-square)](#)

> **Plugin d'intégration multi-IMU des capteurs x-io Technologies x-IMU3 (Quaternions, Gyroscope, Accéléromètre, Magnétomètre, Diagnostic et Synchro BNC) en temps réel dans Qualisys Track Manager.**  
> *Développé au sein du **Technosport / ISM** (UMR 7287 CNRS — Aix-Marseille Université) • [**Releases**](../../releases)*

---

## 🚀 Démarrage

1. Fermer **Qualisys Track Manager (QTM)**.
2. Télécharger et exécuter **`QDevice-xIMU3-Setup-v0.1.0.exe`** depuis les **[Releases](../../releases)**.
3. Ouvrir **QTM > Project Options > Input Devices > Add Device...** et activer **xio Technologies xIMU3**.
4. Connecter les x-IMU3 au réseau Wi-Fi (ou en USB), puis dans **IMUs > xio Technologies xIMU3** cliquer sur **Locate Devices**, cocher **Sync Only** sur l'IMU dédié au câble BNC si besoin, puis **Synchronize Settings** et **OK**.

---

## 🛠️ Fonctionnalités & Contenu

| Canal / Option / Fichier | Rôle / Description |
| :--- | :--- |
| **`Quat W / X / Y / Z`** (`400 Hz`) | Orientation quaternion rééchantillonnée par NLERP plus court chemin |
| **`Gyro X / Y / Z`** (`deg/s`) & **`Acc X / Y / Z`** (`g`) | Mesures inertielles synchronisées sur horloge réseau partagée |
| **`Sync Edge`** (`0 / 1`) | Fronts TTL/BNC issus du *Serial Accessory* (`serial_mode = 4` / Sync, firmware `v2.6.0+`) |
| **`Sync Only`** *(par IMU)* | Case à cocher pour n'enregistrer que `Sync Edge` sur l'IMU Base |
| **`Include Magnetometer`** / **`Include Diagnostics`** | Canaux optionnels `Mag X/Y/Z` (`uT`), `Battery Pct` et `Rssi Pct` |
| **`src/ImuDeviceManager.cpp`** | Découverte Wi-Fi UDP / USB et rééchantillonnage multi-IMU synchronisé |
| **`src/QDevicePlugin.cpp`** | Point d'entrée QDevice API `0.10.6` (`qd_device_category_imu`) |
