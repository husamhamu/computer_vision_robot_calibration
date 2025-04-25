
# 🤖 Fortlaufende Kalibrierung eines 6-DOF-Roboterarms mit monokularer Kamera & AprilTags

Systemübersicht

![image](https://github.com/user-attachments/assets/202765d1-5e48-43dc-9d8a-0948f2261e18)

> 🎓 **Masterarbeit** an der **TU Darmstadt**  
> 👨‍💻 Entwickelt von: **Husam Hamu**  
> 🧩 Thema: **ROS + AprilTag3 + Produkt-von-Exponentiellen (PoE) Kinematik + Optimierung**

---

## 🚀 Projektüberblick

Diese Arbeit präsentiert ein **kosteneffizientes, vision-basiertes Kalibrierungssystem** für Industrieroboter – ohne teure externe Sensorik. Mithilfe einer **monokularen Kamera** und **AprilTags** wird die Pose des **Tool Center Point (TCP)** geschätzt 📐. Anschließend erfolgt die Optimierung eines **PoE-Kinematikmodells** 🔁.

🧰 **Technologiestack:**  
ROS 🐢 | Python 🐍 | AprilTag3 🏷️ | RViz 🖼️ | CAN-Bus 🔌

Kalibrierungssystem: Entwicklung eines ROS-basierten Systems zur fortlaufenden Kalibrierung eines 6-DOF-Roboterarms durch vision-basierte TCP-Pose-Schätzung

Bildverarbeitung: Integration des AprilTag3-Algorithmus zur robusten Markererkennung und präzisen Pose-Bestimmung mittels monokularer Kamera

Kinematikmodellierung: Anwendung der Produkt-von-Exponentiellen-Methode zur Beschreibung der Roboterkinematik und Identifikation fehlerhafter Modellparameter

Robotersteuerung: Umsetzung einer sicheren Bewegungsausführung über CAN-Bus und ROS-Nodes mit geschwindigkeitsbasierter Steuerung und RViz-Visualisierung

---

## 📁 Projektstruktur

```bash
📦 repo/
├── calibration_node/       🧠 Kalibrierlogik & PoE-Optimierung
├── robot_control/          🤖 Mover6-Steuerung via CAN
├── apriltag_pose/          🎯 AprilTag-Erkennung & TF-Veröffentlichung
├── tf_tools/               🔁 Transformationsermittlung & -publikation
├── config/                 ⚙️  YAML-Konfigurationsdateien
├── launch/                 🚀 ROS-Launch-Dateien
├── rviz/                   🖼️  Visualisierungs-Setups
└── data/                   📊 Kalibrier- & Validierungsdaten
```

---

## 🧠 Technische Highlights

### 🧩 1. Vision-basierte Pose-Schätzung

📷 **AprilTag3 mit `apriltag_ros`**  
🔧 Konfigurierbar über `settings.yaml` & `tags.yaml`  
🖼️ Visualisierung in RViz  
📌 Unterstützung für Tag-Bundles

![image](https://github.com/user-attachments/assets/8c634882-e1ee-4fa0-ac0b-da92b7a163ef)

🔁 **Feste Transformationen:**  
- Kamera ↔ Roboterbasis 
- TCP ↔ AprilTag am Endeffektor 
---

### 🦿 2. Robotersteuerung

⚙️ Geschwindigkeitsbasierte Regelung  
🔁 Kommunikation über `/JointJog` und `/joint_states`  
🧪 GUI zur manuellen Steuerung in RViz  
🧱 Klassen: `JointPositionPublisher`, `Mover6CalibrationSystem`

![image](https://github.com/user-attachments/assets/ff30c79d-0de1-4c84-9dbd-ebfb80968542)


---

### 🔧 3. Kinematische Kalibrierung

📌 Modellierung mit PoE (via `modern_robotics`)  
📈 Iterative Optimierung der Modellparameter  
🧮 Nutzung der Identifikations-Jakobimatrix  
📦 Datenerfassung über ROS-Transformationsbaum

![Kalibrierdaten](figures/abbildung_1_6.png)

---

## 🧪 Experimenteller Aufbau

### 🔍 Vorbereitung

🧭 **Kamerakalibrierung:**  
Schachbrettmuster mit 8×6 Feldern, 30 mm Kantenlänge  
![image](https://github.com/user-attachments/assets/ebb77308-d259-4f17-9060-4e94143d6e5e)



🧰 **Hardware-Aufbau:**

- 🎥 Logitech Webcam  
- 🦾 Mover6 Roboter  
- 🖧 CAN-zu-USB Verbindung  
- 📦 AprilTags

![Versuchsaufbau](figures/abbildung_2_7.png)

---

## 📈 Ergebnisse

### 📐 Genauigkeit der AprilTag3-Pose-Schätzung

| 📏 Achse | 📉 Mittelwert (mm) | 🚨 Maximum (mm) |
|---------|--------------------|-----------------|
| X       | 2.60               | 5.87            |
| Y       | 2.76               | 4.42            |
| Z       | 2.09               | 3.94            |

🔍 Referenz: **Tabelle 2.1**

---

## 🧰 Verwendete Technologien

| Tool | Beschreibung |
|------|--------------|
| 🐍 Python | Hauptimplementierung |
| 🐢 ROS | Kommunikation & Nodes |
| 📦 `modern_robotics` | PoE-Formel |
| 📊 `NumPy`, `SciPy` | Mathematische Operationen |
| 🖼️ RViz | Visualisierung |
| 🏷️ AprilTag3 | Marker-basierte Pose-Schätzung |


---

## 📚 Weiterführendes

📄 **Masterarbeit:** [Download hier](link_zur_pdf)  

---
