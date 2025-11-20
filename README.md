# turtle_bot_magnet_localisation

Petit guide pour compiler et lancer les nœuds principaux du package.

## Prérequis
- ROS 2 installé et sourcing du workspace (ex. `source /opt/ros/<distro>/setup.bash`).
- colcon installé.

## Compilation
Depuis la racine du workspace (ex. `~/turtle_bot`) :
```bash
~/turtle_bot$ colcon build --symlink-install
```
Sourcer le workspace après la build :
```bash
~/turtle_bot$ source install/setup.bash
```

## Lancer les nœuds
1. Visualiser la trajectoire (après localisation) :
```bash
~/turtle_bot$ ros2 run test_visu pose_recorder
```

2. Lancer le nœud de localisation (position de départ : (0,0,0)) :
```bash
~/turtle_bot$ ros2 run ekf_localization ekf_localization_node
```

3. Lancer le nœud data_replay  
    Important : lancer ce nœud depuis le dossier `data` :
```bash
~/turtle_bot/data$ ros2 run data_replay data_replay
```

## Procédure d'expérimentation
1. Lancer le nœud `pose_recorder` pour la visualisation.
2. Lancer le nœud de localisation (`ekf_localization_node`).
3. Dans un autre terminal, lancer `data_replay` depuis `~/turtle_bot/data`.
4. Avant de démarrer une nouvelle expérience :  
    - revenir au terminal du nœud de recording (ou du launcher principal) et relancer le nœud de localisation pour réinitialiser la position de départ à (0,0,0).

## Remarques et conseils
- Vérifier que les topics et les formats de message entre `data_replay`, `measurement_node` et l’EKF correspondent (types et fréquences).
- Si vous rencontrez des incohérences, utilisez `ros2 topic echo` et `ros2 topic list` pour déboguer.

## À faire (TODO)
- Faire fonctionner l'architecture actuelle et bien la comprendre.
- Refaire `data_replay` pour qu'il publie uniquement les données brutes du capteur (entier 0–255) au lieu du measurement déjà traité.
- Adapter/implémenter `measurement_node` pour consommer les raw sensor data et produire les measurements attendus par l’EKF.
- Vérifier que la fonction `ExtractMeasurement` fonctionne correctement dans `measurement_node` (elle fonctionne dans `data_replay` aujourd’hui mais pas forcément ailleurs).
- Créer un launchfile pour démarrer ensemble l’EKF et le `measurement_node`.

Si tu veux, je peux te proposer un exemple de launchfile ROS 2 ou un template pour le nouveau `data_replay`.
