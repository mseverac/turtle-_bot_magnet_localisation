
# 🐢 Turtle Bot – Magnet Localization

Ce projet implémente une architecture ROS2 permettant la **localisation d’un TurtleBot** à l’aide d’un capteur magnétique, d’un **EKF** et de données rejouées depuis des fichiers.

---

## 🚀 Compilation du workspace

Depuis la racine du workspace :

```bash
~/turtle_bot$ colcon build --symlink-install
```

---

## 🎯 Visualiser la trajectoire après localisation

Lance le nœud d’enregistrement/visualisation :

```bash
~/turtle_bot$ ros2 run test_visu pose_recorder
```

---

## 📌 Lancer la localisation (EKF)

La position initiale du robot est fixée à **(0, 0, 0)**.

```bash
~/turtle_bot$ ros2 run ekf_localization ekf_localization_node
```

---

## 🔁 Rejouer les données (data replay)

> ⚠️ Ce nœud doit être lancé **depuis le dossier contenant les données**.

```bash
~/turtle_bot/data$ ros2 run data_replay data_replay
```

---

## 🔄 Nouvelle expérience : réinitialisation

Avant de recommencer une expérience :

1. Arrêter le nœud de recording.
2. Relancer le nœud de **ekf_localization_node** afin de **réinitialiser la position**.
3. Relancer ensuite le **data replay**

---

## 📝 À faire

### ✔️ Compréhension & fonctionnement

* Faire fonctionner l’architecture actuelle.
* Comprendre précisément le rôle de chaque nœud (replay, measurement, EKF, visualisation…).

### 🔧 Nouveau data_replay

* Implémenter une version de `data_replay` qui **ne publie plus de "measurement"**,
  mais **uniquement les `rawsensor_data`**, soit la valeur brute entre **0 et 255**.

### 🧠 Nouvelle architecture

* Mettre en place une nouvelle architecture utilisant :

  * le **nouveau data_replay**
  * le **measurement_node** 
  * l’**EKF**
* Attention : la fonction `ExtractMeasurement` marche dans `data_replay`,
  mais **pas forcément** dans `measurement_node`.
  ➜ Il faut donc vérifier/adapter l’extraction.

### 🚀 Launch file

* Créer un launch file permettant de lancer **simultanément** :

  * l’EKF
  * le measurement_node

