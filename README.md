# turtle-_bot_magnet_localisation


build le colcon : 

~/turtle_bot$ colcon build --symlink-install


Pour visualiser la trajcetoire après localisation : 


~/turtle_bot$ ros2 run test_visu pose_recorder 


Lancer le noeud de localisation (la position de départ est (0,0,0)): 


~/turtle_bot$ ros2 run ekf_localization ekf_localization_node 


Lancer le noeud de data_replay : 
/!\ lancer le noeud dans le fichier data 

~/turtle_bot/data$ ros2 run data_replay data_replay 


Ensuite faire entrer dans le terminal du noeud de recording et relancer le noeud de localisation avant de faire une nouvelle expérience pour que la position soit réinitialisée. 


A faire : 
-Faire marcher l'architecture actuelle et comprendre comment elle fonctionne
-Faire une nouvelle version de data_replay qui ne publie pas de measurement mais simplement le rawsensor_data cet à dire l'entier entre 0 et 255
-Faire marcher une nouvelle architecture avec le nouveau data_replay, le measurement node et l'EKF /!\ La fonciton ExtractMeasurment fonctionne dasn data_replay mais pas forcément dans measurement_node 
-Faire un launchfile pour lancer l'EKF et le measurement_node en même temps

