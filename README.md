# ORIGINES — Mission 001 : Terre-Air

Un rover au sol et, plus tard, un drone coopèrent pour naviguer intelligemment dans un bâtiment : le drone repère un passage bloqué, le rover change de chemin tout seul.

Projet développé en public par **RobotiqueStudio Labs (RSL)**.

## État actuel

- [x] Modèle 3D du rover (URDF) affiché dans RViz
- [x] Arène de simulation Gazebo avec obstacle
- [x] Le rover roule en simulation, piloté par ROS 2
- [ ] LiDAR simulé
- [ ] Évitement d'obstacle
- [ ] Navigation autonome (SLAM + Nav2)
- [ ] Construction du rover réel
- [ ] Drone éclaireur

## Contenu

| Paquet | Rôle |
|---|---|
| `origines_description` | Modèle URDF du rover, configuration RViz |
| `origines_sim` | Monde Gazebo (arène 4 x 4 m) et lanceur de simulation |

## Environnement

- Ubuntu 26.04
- ROS 2 Lyrical Luth
- Gazebo Jetty (avec `ros-lyrical-ros-gz`)

## Lancer la simulation

```bash
cd ~/origines_ws && colcon build --symlink-install
source install/setup.bash
ros2 launch origines_sim sim.launch.py
```

Dans un second terminal, pour faire avancer le rover :

```bash
source ~/origines_ws/install/setup.bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

## Suivi du projet

Journal de bord publié sur LinkedIn : série « ORIGINES — Log #NN ».
