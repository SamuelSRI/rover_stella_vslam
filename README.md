# rover_stella_vslam

Package ROS 2 pour intégrer Stella-VSLAM au rover avec un flux monoculaire déjà
rectifié.

## Chaîne

```text
/camera/front/image_raw
        -> stella_vslam_ros
        -> /stella_vslam/camera_pose (map visuelle -> caméra)
        -> stella_pose_adapter
        -> /visual_slam/pose (map visuelle -> base_link)
        -> rover_slam_selector
```

Stella est lancé avec `publish_tf=false`. Le seul `map -> odom` doit rester celui
de `rover_slam_selector`.

## Précondition TF

Il faut une TF statique valide :

```text
base_link -> camera_front_optical_frame
```

Le `header.frame_id` du topic image doit idéalement être
`camera_front_optical_frame`.

Vérification :

```bash
ros2 topic echo /camera/front/image_raw/header --once
ros2 run tf2_ros tf2_echo base_link camera_front_optical_frame
```

## Paramètres caméra

Éditer :

```text
config/stella_camera_corrected.yaml
```

Remplacer impérativement :

```yaml
fx: ...
fy: ...
cx: ...
cy: ...
fps: 25.0
cols: 640
rows: 480
```

Comme le flux est supposé déjà rectifié, les coefficients restent à zéro :

```yaml
k1: 0.0
k2: 0.0
p1: 0.0
p2: 0.0
k3: 0.0
```

Les paramètres `fx`, `fy`, `cx`, `cy` doivent être ceux de l'image rectifiée,
pas ceux de l'image brute avant correction.

## Installation

```bash
cd ~/ros2_ws/src
unzip ~/Downloads/rover_stella_vslam.zip

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select rover_stella_vslam

source install/setup.bash
```

## Vocabulaire

Trouver le fichier :

```bash
find ~/ros2_ws ~/lib -name "orb_vocab.fbow" 2>/dev/null
```

Puis fournir son chemin au lancement.

## Lancement

```bash
ros2 launch rover_stella_vslam stella_vslam.launch.py \
  image_topic:=/camera/front/image_raw \
  vocabulary:=/chemin/vers/orb_vocab.fbow
```

## Vérifications

```bash
ros2 topic hz /stella_vslam/camera_pose
ros2 topic echo /visual_slam/pose --once
ros2 topic echo /localization/active_source
```

Activer la source extérieure :

```bash
ros2 topic pub --once /env/exterior std_msgs/msg/Bool "{data: true}"
```

## Attention monoculaire

La translation d'un SLAM monoculaire n'a pas d'échelle métrique garantie.
L'adaptateur ne corrige pas encore cette échelle. Il convertit seulement la pose
de la caméra vers `base_link`. Une estimation d'échelle par l'odométrie du rover
devra être ajoutée avant une fusion métrique fiable.
