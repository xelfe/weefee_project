#!/bin/bash
# Script de vérification minimale du fonctionnement du nœud servo_commander

# Couleurs pour les messages
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}==============================================${NC}"
echo -e "${BLUE}    Vérification de servo_commander           ${NC}"
echo -e "${BLUE}==============================================${NC}"

# Source l'environnement ROS2
echo -e "\n${YELLOW}Sourcing ROS2 environment...${NC}"
source /opt/ros/jazzy/setup.bash
source $HOME/weefee_project/ros2_ws/install/setup.bash

# Vérifie les principaux topics impliqués
echo -e "\n${YELLOW}Vérification des topics:${NC}"
ros2 topic info /servo_angles
ros2 topic info /robot_command
ros2 topic info /command_input

# Test de publication directe de servo_angles
echo -e "\n${YELLOW}Test de publication directe sur /servo_angles (pour valider le topic):${NC}"
ros2 topic pub --once /servo_angles std_msgs/msg/Int32MultiArray "{layout: {dim: [{label: 'servos', size: 12, stride: 12}], data_offset: 0}, data: [90, 45, 135, 90, 45, 45, 90, 45, 135, 90, 45, 45]}"

# Attends un peu pour voir si les autres nœuds réagissent
sleep 2

# Test avec calibrate pour initialisation
echo -e "\n${YELLOW}Envoi de la commande 'calibrate' sur /command_input:${NC}"
ros2 topic pub --once /command_input std_msgs/msg/String "{data: 'calibrate'}"

# Vérifie ce qui se passe dans les logs de la session screen
echo -e "\n${YELLOW}Vérifié la sortie du log servo_commander (doit montrer 'Command sent: Calibration position'):${NC}"
screen -ls | grep servo_commander

echo -e "\n${YELLOW}Attente de traitement de la calibration...${NC}"
sleep 3

# Test de la commande servo directe
echo -e "\n${YELLOW}Test de commande de servos directe sur /command_input:${NC}"
ros2 topic pub --once /command_input std_msgs/msg/String "{data: 'servo:90,45,90,90,45,90,90,45,90,90,45,90'}"

# Attends un peu pour voir si des messages sont publiés
sleep 2

# Test avec la commande stand après calibration
echo -e "\n${YELLOW}Envoi de la commande 'stand' sur /command_input:${NC}"
ros2 topic pub --once /command_input std_msgs/msg/String "{data: 'stand'}"

echo -e "\n${YELLOW}Vérification du flux de messages après commande 'stand'...${NC}"
echo -e "Écoute de /servo_angles pendant 3 secondes après stand:"
timeout 3 ros2 topic echo /servo_angles || echo -e "${RED}Aucun message détecté sur /servo_angles${NC}"

echo -e "\n${BLUE}==============================================${NC}"
echo -e "${BLUE}    Vérification terminée                     ${NC}"
echo -e "${BLUE}==============================================${NC}"

echo -e "\n${YELLOW}Pour voir les logs du nœud servo_commander:${NC}"
echo -e "${GREEN}screen -r weefee_servo_commander${NC}"
