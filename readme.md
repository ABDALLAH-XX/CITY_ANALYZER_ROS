cat > README.md <<EOL
# CITY_ANALYZER_ROS

Un package ROS2 pour analyser et visualiser des données urbaines en 3D.

## Structure du projet

\`\`\`
city_analyzer/
│
├─ data/                  
├─ include/city_analyzer_ros/  
│   └─ CityAnalyzer.hpp
├─ launch/
│   └─ city_visualizer.launch.py
├─ rviz/
│   └─ city_view.rviz
├─ src/
│   ├─ CityAnalyzer.cpp
│   └─ main.cpp
├─ CMakeLists.txt
├─ package.xml
├─ LICENSE
└─ .gitignore
\`\`\`

## Installation

1. Cloner le dépôt dans votre workspace ROS2 :
\`\`\`
cd ~/ros2_ws/src
git clone https://github.com/ABDALLAH-XX/city_analyzer_ros.git
\`\`\`

2. Construire le package avec \`colcon\` :
\`\`\`
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
\`\`\`

## Lancer le visualisateur
\`\`\`
ros2 launch city_analyzer_ros city_visualizer.launch.py
\`\`\`

## Dépendances
- ROS2 Humble/Foxy/Galactic
- Rviz2
- CMake, g++, colcon

## Auteur
- Abdallah
EOL
