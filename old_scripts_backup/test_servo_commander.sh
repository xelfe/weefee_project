#!/bin/bash
# Script pour tester le fonctionnement du nœud servo_commander

# Couleurs pour les messages
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${RED}ATTENTION: Ce script est obsolète!${NC}"
echo -e "${YELLOW}Il teste les topics /servo_angles et /servo_angles_str qui ne sont plus utilisés.${NC}"
echo -e "${GREEN}Utilisez plutôt le script 'test_servo_commander_optimized.sh' qui teste uniquement le topic /robot_command.${NC}"
echo ""
echo -e "Souhaitez-vous exécuter le script optimisé à la place? (y/n)"
read -r answer
if [[ "$answer" =~ ^[Yy]$ ]]; then
    echo -e "Lancement du script optimisé..."
    ./test_servo_commander_optimized.sh
    exit 0
fi

echo -e "Voulez-vous continuer avec ce script obsolète? (y/n)"
read -r answer
if [[ ! "$answer" =~ ^[Yy]$ ]]; then
    echo -e "Exécution annulée."
    exit 0
fi

echo -e "${YELLOW}Test du nœud servo_commander (OBSOLÈTE)${NC}"

# Vérifie d'abord si le nœud servo_commander est déjà en cours d'exécution dans une session screen
SCREEN_NAME="weefee_servo_commander"
SCREEN_RUNNING=false

if screen -list | grep -q $SCREEN_NAME; then
    echo -e "${GREEN}Le nœud servo_commander est déjà en cours d'exécution dans une session screen ($SCREEN_NAME)${NC}"
    echo -e "Utilisation de l'instance existante pour le test..."
    SCREEN_RUNNING=true
else
    echo -e "Aucune session screen $SCREEN_NAME en cours d'exécution."
    
    # Vérifie si le processus servo_commander est déjà en cours d'exécution directement
    EXISTING_PID=$(pgrep -f "servo_commander")
    if [ ! -z "$EXISTING_PID" ]; then
        echo -e "${YELLOW}Un processus servo_commander est déjà en cours d'exécution (PID: $EXISTING_PID)${NC}"
        echo -e "Souhaitez-vous utiliser ce processus pour le test? (y/n)"
        read -r answer
        if [[ "$answer" =~ ^[Yy]$ ]]; then
            echo -e "${GREEN}Utilisation du processus existant pour le test...${NC}"
            SERVO_COMMANDER_PID=$EXISTING_PID
        else
            echo -e "${YELLOW}Arrêt du processus existant...${NC}"
            kill $EXISTING_PID
            sleep 2
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
echo -e "Liste des topics ROS2:"
ros2 topic list

# Vérifie spécifiquement le topic /servo_angles
if ros2 topic list | grep -q "/servo_angles"; then
    echo -e "${GREEN}Le topic /servo_angles existe${NC}"
    
    # Affiche les informations sur le topic
    echo -e "Informations sur le topic /servo_angles:"
    ros2 topic info /servo_angles
    
    # Essaie d'écouter le topic (avec timeout)
    echo -e "Tentative d'écoute du topic /servo_angles (5 secondes)..."
    timeout 5 ros2 topic echo /servo_angles || echo -e "${YELLOW}Aucun message reçu en 5 secondes${NC}"
else
    echo -e "${RED}ERREUR: Le topic /servo_angles n'existe pas${NC}"
fi

# Vérifie si le topic /robot_command existe
if ros2 topic list | grep -q "/robot_command"; then
    echo -e "${GREEN}Le topic /robot_command existe${NC}"
    
    # Teste la commande "stand"
    echo -e "\n${YELLOW}Test de la commande 'stand' sur /robot_command...${NC}"
    echo -e "Publication de la commande 'stand'..."
    
    # Lance un listener sur /servo_angles dans un autre terminal
    echo -e "Démarrage d'un listener sur /servo_angles (en arrière-plan)..."
    ros2 topic echo /servo_angles > /tmp/servo_angles_output.txt &
    LISTENER_PID=$!
    
    # Attends un peu pour que le listener démarre
    sleep 1
    
    # Publie la commande "stand"
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'stand'}"
    
    # Attends un peu pour voir si des messages sont publiés
    echo -e "Attente de 3 secondes pour voir si des angles sont publiés..."
    sleep 3
    
    # Arrête le listener
    kill $LISTENER_PID
    
    # Vérifie si des données ont été capturées
    if [ -s /tmp/servo_angles_output.txt ]; then
        echo -e "${GREEN}Des messages ont été publiés sur /servo_angles suite à la commande 'stand'${NC}"
        echo -e "Contenu des messages:"
        cat /tmp/servo_angles_output.txt
    else
        echo -e "${RED}ERREUR: Aucun message n'a été publié sur /servo_angles suite à la commande 'stand'${NC}"
        echo -e "Vérifiez que le nœud servo_commander traite correctement la commande 'stand'"
        echo -e "Il est possible que vous deviez publier sur /command_input au lieu de /robot_command"
    fi
    
    # Nettoie le fichier temporaire
    rm /tmp/servo_angles_output.txt
    
    # Teste également la commande sur /command_input comme alternative
    echo -e "\n${YELLOW}Test de la commande 'stand' sur /command_input...${NC}"
    if ros2 topic list | grep -q "/command_input"; then
        echo -e "${GREEN}Le topic /command_input existe${NC}"
        
        # Lancer le calibrage d'abord
        echo -e "\n${YELLOW}Calibrage du robot (nécessaire avant toute commande)...${NC}"
        echo -e "Publication de la commande 'calibrate'..."
        
        # Lance un listener sur /servo_angles dans un autre terminal
        echo -e "Démarrage d'un listener sur /servo_angles (en arrière-plan)..."
        ros2 topic echo /servo_angles > /tmp/servo_angles_calibrate.txt &
        LISTENER_PID=$!
        
        # Attends un peu pour que le listener démarre
        sleep 1
        
        # Publie la commande "calibrate"
        ros2 topic pub --once /command_input std_msgs/msg/String "{data: 'calibrate'}"
        
        # Attends un peu pour que la calibration prenne effet
        echo -e "Attente de 3 secondes pour la calibration..."
        sleep 3
        
        # Arrête le listener
        kill $LISTENER_PID
        
        # Vérifie si des données ont été capturées
        if [ -s /tmp/servo_angles_calibrate.txt ]; then
            echo -e "${GREEN}Des messages ont été publiés sur /servo_angles suite à la commande 'calibrate'${NC}"
            echo -e "Contenu des messages (résumé):"
            head -10 /tmp/servo_angles_calibrate.txt
        else
            echo -e "${YELLOW}Note: Aucun message n'a été publié sur /servo_angles suite à la commande 'calibrate'${NC}"
        fi
        
        # Nettoie le fichier temporaire
        rm /tmp/servo_angles_calibrate.txt
        
        # Teste la commande "stand" 
        echo -e "\n${YELLOW}Test de la commande 'stand' après calibration...${NC}"
        echo -e "Publication de la commande 'stand'..."
        
        # Lance un listener sur /servo_angles dans un autre terminal
        echo -e "Démarrage d'un listener sur /servo_angles (en arrière-plan)..."
        ros2 topic echo /servo_angles > /tmp/servo_angles_output2.txt &
        LISTENER_PID=$!
        
        # Attends un peu pour que le listener démarre
        sleep 1
        
        # Publie la commande "stand"
        ros2 topic pub --once /command_input std_msgs/msg/String "{data: 'stand'}"
        
        # Attends un peu pour voir si des messages sont publiés
        echo -e "Attente de 3 secondes pour voir si des angles sont publiés..."
        sleep 3
        
        # Arrête le listener
        kill $LISTENER_PID
        
        # Vérifie si des données ont été capturées
        if [ -s /tmp/servo_angles_output2.txt ]; then
            echo -e "${GREEN}Des messages ont été publiés sur /servo_angles suite à la commande 'stand'${NC}"
            echo -e "Contenu des messages (résumé):"
            head -10 /tmp/servo_angles_output2.txt
        else
            echo -e "${RED}ERREUR: Aucun message n'a été publié sur /servo_angles suite à la commande 'stand'${NC}"
            echo -e "Même après calibration, le problème persiste."
        fi
        
        # Nettoie le fichier temporaire
        rm /tmp/servo_angles_output2.txt
    else
        echo -e "${RED}Le topic /command_input n'existe pas${NC}"
    fi
    
    # Test de publication directe des angles de servo
    echo -e "\n${YELLOW}Test de publication directe sur /servo_angles...${NC}"
    echo -e "Publication d'un message de test sur /servo_angles..."

    # Format pour Int32MultiArray
    ros2 topic pub --once /servo_angles std_msgs/msg/Int32MultiArray "{layout: {dim: [{label: 'servos', size: 12, stride: 12}], data_offset: 0}, data: [90, 45, 135, 90, 45, 45, 90, 45, 135, 90, 45, 45]}"

    echo -e "Vérification que d'autres nœuds peuvent recevoir les messages sur /servo_angles..."
else
    echo -e "${RED}ERREUR: Le topic /robot_command n'existe pas${NC}"
fi

# N'arrête le nœud servo_commander que si nous l'avons démarré dans ce script
if [ "$SCREEN_RUNNING" = false ] && [ ! -z "$SERVO_COMMANDER_PID" ]; then
    echo -e "Arrêt du nœud servo_commander..."
    kill $SERVO_COMMANDER_PID
else
    echo -e "${GREEN}Laisse le nœud servo_commander en cours d'exécution dans sa session screen${NC}"
fi

echo -e "${YELLOW}Test terminé${NC}"
