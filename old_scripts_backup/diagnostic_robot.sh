#!/bin/bash
# Script de diagnostic pour le système Weefee ROS2

# Couleurs pour les messages
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}==============================================${NC}"
echo -e "${BLUE}    Diagnostic complet du système Weefee     ${NC}"
echo -e "${BLUE}==============================================${NC}"

# Source l'environnement ROS2
echo -e "\n${YELLOW}Sourcing ROS2 environment...${NC}"
source /opt/ros/jazzy/setup.bash
source $HOME/weefee_project/ros2_ws/install/setup.bash

# Affiche la liste complète des nœuds ROS2 en cours d'exécution
echo -e "\n${YELLOW}Liste des nœuds ROS2 en cours d'exécution:${NC}"
ros2 node list
echo ""

# Affiche la liste complète des topics
echo -e "${YELLOW}Liste des topics ROS2:${NC}"
ros2 topic list
echo ""

# Détails sur les topics importants
TOPICS=("servo_angles" "robot_command" "command_input" "gait_control" "robot_status")

for topic in "${TOPICS[@]}"; do
    if ros2 topic list | grep -q "/$topic"; then
        echo -e "${GREEN}Le topic /$topic existe${NC}"
        echo -e "  Type: $(ros2 topic info /$topic | grep 'Type' | awk '{print $2}')"
        echo -e "  Publishers: $(ros2 topic info /$topic | grep 'Publisher count' | awk '{print $3}')"
        echo -e "  Subscribers: $(ros2 topic info /$topic | grep 'Subscription count' | awk '{print $3}')"
        
        # Essaie d'écouter le topic (très brièvement pour ne pas bloquer)
        echo -e "  Tentative d'écoute de /$topic (1 seconde)..."
        MESSAGES=$(timeout 1 ros2 topic echo /$topic 2>/dev/null)
        if [ ! -z "$MESSAGES" ]; then
            echo -e "  ${GREEN}Messages détectés sur /$topic:${NC}"
            echo "$MESSAGES" | head -5
            echo "  ..."
        else
            echo -e "  ${YELLOW}Aucun message sur /$topic pendant la période d'écoute${NC}"
        fi
        echo ""
    else
        echo -e "${RED}Le topic /$topic n'existe pas${NC}"
        echo ""
    fi
done

# Vérifie l'interconnexion entre les nœuds pertinents
echo -e "\n${YELLOW}Analyse des interconnexions entre nœuds:${NC}"
echo -e "Nœuds publiant sur /servo_angles:"
ros2 topic info /servo_angles -v 2>/dev/null | grep -A 10 "Publishers:" | grep -v "Subscribers:" || echo "  Aucun"

echo -e "\nNœuds abonnés à /servo_angles:"
ros2 topic info /servo_angles -v 2>/dev/null | grep -A 10 "Subscribers:" || echo "  Aucun"

echo -e "\nNœuds publiant sur /robot_command:"
ros2 topic info /robot_command -v 2>/dev/null | grep -A 10 "Publishers:" | grep -v "Subscribers:" || echo "  Aucun"

echo -e "\nNœuds abonnés à /robot_command:"
ros2 topic info /robot_command -v 2>/dev/null | grep -A 10 "Subscribers:" || echo "  Aucun"

echo -e "\nNœuds publiant sur /command_input:"
ros2 topic info /command_input -v 2>/dev/null | grep -A 10 "Publishers:" | grep -v "Subscribers:" || echo "  Aucun"

echo -e "\nNœuds abonnés à /command_input:"
ros2 topic info /command_input -v 2>/dev/null | grep -A 10 "Subscribers:" || echo "  Aucun"

# Test de communication
echo -e "\n${YELLOW}Test de communication:${NC}"
echo -e "Envoi de la commande 'calibrate' sur /command_input..."
ros2 topic pub --once /command_input std_msgs/msg/String "{data: 'calibrate'}"

echo -e "\nAttente de 3 secondes pour voir si des réactions apparaissent..."
sleep 3

echo -e "\nVérification des messages sur les topics importants après calibration..."
for topic in "${TOPICS[@]}"; do
    if ros2 topic list | grep -q "/$topic"; then
        echo -e "\nÉcoute de /$topic pendant 1 seconde:"
        timeout 1 ros2 topic echo /$topic 2>/dev/null || echo "  Aucun message"
    fi
done

# Vérification du robot
echo -e "\n${YELLOW}Interrogation du statut du robot:${NC}"
echo -e "Derniers messages de statut du robot:"
timeout 2 ros2 topic echo /robot_status 2>/dev/null || echo "  Aucun message de statut reçu"

# Test de la commande stand
echo -e "\n${YELLOW}Test de la commande 'stand' après calibration:${NC}"
echo -e "Envoi de la commande 'stand' sur /command_input..."
ros2 topic pub --once /command_input std_msgs/msg/String "{data: 'stand'}"

echo -e "\nAttente de 3 secondes pour voir si des angles sont publiés..."
sleep 3

echo -e "\nVérification des messages sur /servo_angles après commande 'stand':"
timeout 2 ros2 topic echo /servo_angles 2>/dev/null || echo "  Aucun message sur /servo_angles"

# Vérification des logs des session screens
echo -e "\n${YELLOW}Vérification des sessions screen:${NC}"
screen -list

echo -e "\n${BLUE}==============================================${NC}"
echo -e "${BLUE}    Diagnostic terminé                        ${NC}"
echo -e "${BLUE}==============================================${NC}"

echo -e "\nPour voir les logs d'une session screen, utilisez:"
echo -e "${GREEN}screen -r [nom_de_la_session]${NC}"
echo -e "Pour quitter sans fermer: ${GREEN}Ctrl+A puis D${NC}"
