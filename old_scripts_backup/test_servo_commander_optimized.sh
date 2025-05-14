#!/bin/bash
# Script to test the optimized servo_commander node
# This version only tests the /robot_command topic since it's the only one
# now used to communicate with the ESP32

# Colors for messages
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Testing optimized servo_commander node${NC}"

# First check if the servo_commander node is already running in a screen session
SCREEN_NAME="weefee_servo_commander"
SCREEN_RUNNING=false

if screen -list | grep -q $SCREEN_NAME; then
    echo -e "${GREEN}The servo_commander node is already running in a screen session ($SCREEN_NAME)${NC}"
    echo -e "Using existing instance for testing..."
    SCREEN_RUNNING=true
else
    echo -e "No screen session $SCREEN_NAME is running."
    
    # Check if servo_commander process is already running directly
    EXISTING_PID=$(pgrep -f "servo_commander")
    if [ ! -z "$EXISTING_PID" ]; then
        echo -e "${YELLOW}A servo_commander process is already running (PID: $EXISTING_PID)${NC}"
        echo -e "Do you want to use this process for testing? (y/n)"
        read -r answer
        if [[ "$answer" =~ ^[Yy]$ ]]; then
            echo -e "${GREEN}Using existing process for testing...${NC}"
            SERVO_COMMANDER_PID=$EXISTING_PID
        else
            echo -e "${YELLOW}Stopping existing process...${NC}"
            kill $EXISTING_PID
            sleep 2
            # Source ROS2 environment
            echo -e "Sourcing ROS2 environment..."
            source /opt/ros/jazzy/setup.bash
            source $HOME/weefee_project/ros2_ws/install/setup.bash
            
            # Launch servo_commander node
            echo -e "Launching servo_commander node..."
            ros2 run weefee_node servo_commander &
            SERVO_COMMANDER_PID=$!
            
            # Wait a bit for the node to start
            sleep 3
        fi
    else
        # Source l'environnement ROS2
        echo -e "Sourcing ROS2 environment..."
        source /opt/ros/jazzy/setup.bash
        source $HOME/weefee_project/ros2_ws/install/setup.bash
        
        # Lance le nœud servo_commander
        echo -e "Lancement du nœud servo_commander..."
        ros2 run weefee_node servo_commander &
        SERVO_COMMANDER_PID=$!
        
        # Attends un peu que le nœud démarre
        sleep 3
    fi
    
    # Vérifie que le nœud est en cours d'exécution
    if ps -p $SERVO_COMMANDER_PID > /dev/null; then
        echo -e "${GREEN}Le nœud servo_commander est en cours d'exécution (PID: $SERVO_COMMANDER_PID)${NC}"
    else
        echo -e "${RED}ERREUR: Le nœud servo_commander n'est pas en cours d'exécution${NC}"
        exit 1
    fi
fi

# Vérifie la liste des topics
echo -e "\n${YELLOW}Liste des topics ROS2:${NC}"
ros2 topic list

# Vérifie si le topic /robot_command existe
if ros2 topic list | grep -q "/robot_command"; then
    echo -e "${GREEN}Le topic /robot_command existe${NC}"
    
    # Affiche les informations sur le topic
    echo -e "\n${YELLOW}Informations sur le topic /robot_command:${NC}"
    ros2 topic info /robot_command
    
    # Démarrage d'un listener sur /robot_status pour voir les réponses de l'ESP32
    echo -e "\n${YELLOW}Démarrage d'un listener sur /robot_status (en arrière-plan)...${NC}"
    ros2 topic echo /robot_status > /tmp/robot_status_output.txt &
    LISTENER_PID=$!
    
    # Attends un peu pour que le listener démarre
    sleep 1
    
    # Calibrage du robot (nécessaire avant toute commande)
    echo -e "\n${YELLOW}Test de la commande 'calibrate'...${NC}"
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'calibrate'}"
    
    # Attends pour voir la réponse
    echo -e "Attente de 3 secondes pour voir la réponse de calibration..."
    sleep 3
    
    # Teste la commande "stand"
    echo -e "\n${YELLOW}Test de la commande 'stand'...${NC}"
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'stand'}"
    
    # Attends pour voir la réponse
    echo -e "Attente de 3 secondes pour voir la réponse..."
    sleep 3
    
    # Teste la commande "servo:" (commande directe de servo)
    echo -e "\n${YELLOW}Test de la commande directe de servo...${NC}"
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'servo:90,45,135,90,45,45,90,45,135,90,45,45'}"
    
    # Attends pour voir la réponse
    echo -e "Attente de 3 secondes pour voir la réponse..."
    sleep 3
    
    # Teste la commande directe sans préfixe (format compatible)
    echo -e "\n${YELLOW}Test du format compatible d'angles séparés par des virgules...${NC}"
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: '90,45,135,90,45,45,90,45,135,90,45,45'}"
    
    # Attends pour voir la réponse
    echo -e "Attente de 3 secondes pour voir la réponse..."
    sleep 3
    
    # Teste la commande "sit"
    echo -e "\n${YELLOW}Test de la commande 'sit'...${NC}"
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'sit'}"
    
    # Attends pour voir la réponse
    echo -e "Attente de 3 secondes pour voir la réponse..."
    sleep 3
    
    # Arrête le listener
    kill $LISTENER_PID
    
    # Vérifie si des données ont été capturées
    if [ -s /tmp/robot_status_output.txt ]; then
        echo -e "\n${GREEN}Réponses reçues sur /robot_status:${NC}"
        cat /tmp/robot_status_output.txt
    else
        echo -e "\n${YELLOW}Aucune réponse reçue sur /robot_status${NC}"
        echo -e "Vérifiez que l'ESP32 est connecté et que le topic /robot_status fonctionne correctement"
    fi
    
    # Nettoie le fichier temporaire
    rm /tmp/robot_status_output.txt
    
    # Teste également via /command_input (l'interface utilisateur)
    echo -e "\n${YELLOW}Test via l'interface utilisateur (/command_input)...${NC}"
    if ros2 topic list | grep -q "/command_input"; then
        echo -e "${GREEN}Le topic /command_input existe${NC}"
        
        # Démarre un nouveau listener pour /robot_status
        echo -e "Démarrage d'un listener sur /robot_status pour les commandes via command_input..."
        ros2 topic echo /robot_status > /tmp/robot_status_command_input.txt &
        LISTENER_PID=$!
        
        # Attends un peu pour que le listener démarre
        sleep 1
        
        # Teste la commande "stand" via command_input
        echo -e "Publication de la commande 'stand' via command_input..."
        ros2 topic pub --once /command_input std_msgs/msg/String "{data: 'stand'}"
        
        # Attends pour voir la réponse
        echo -e "Attente de 3 secondes pour voir la réponse..."
        sleep 3
        
        # Arrête le listener
        kill $LISTENER_PID
        
        # Vérifie si des données ont été capturées
        if [ -s /tmp/robot_status_command_input.txt ]; then
            echo -e "\n${GREEN}Réponses reçues sur /robot_status pour les commandes via command_input:${NC}"
            cat /tmp/robot_status_command_input.txt
        else
            echo -e "\n${YELLOW}Aucune réponse reçue sur /robot_status pour les commandes via command_input${NC}"
        fi
        
        # Nettoie le fichier temporaire
        rm /tmp/robot_status_command_input.txt
    else
        echo -e "${RED}Le topic /command_input n'existe pas${NC}"
    fi
else
    echo -e "${RED}ERREUR: Le topic /robot_command n'existe pas${NC}"
    echo -e "Vérifiez que le nœud servo_commander est bien démarré"
fi

# N'arrête le nœud servo_commander que si nous l'avons démarré dans ce script
if [ "$SCREEN_RUNNING" = false ] && [ ! -z "$SERVO_COMMANDER_PID" ]; then
    echo -e "\n${YELLOW}Arrêt du nœud servo_commander...${NC}"
    kill $SERVO_COMMANDER_PID
else
    echo -e "\n${GREEN}Laisse le nœud servo_commander en cours d'exécution dans sa session screen${NC}"
fi

echo -e "\n${YELLOW}Test terminé${NC}"
