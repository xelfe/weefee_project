#!/bin/bash
# Script de débug pour comprendre le problème de communication ROS2

# Couleurs pour les messages
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}==============================================${NC}"
echo -e "${BLUE}    Débogage profond du système ROS2         ${NC}"
echo -e "${BLUE}==============================================${NC}"

# Source l'environnement ROS2
echo -e "\n${YELLOW}Sourcing ROS2 environment...${NC}"
source /opt/ros/jazzy/setup.bash
source $HOME/weefee_project/ros2_ws/install/setup.bash

# Vérifie la variable RMW_IMPLEMENTATION
echo -e "\n${YELLOW}Configuration du middleware ROS2:${NC}"
echo -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"

# Vérifie les autres variables d'environnement pertinentes
echo -e "\n${YELLOW}Variables d'environnement pertinentes:${NC}"
env | grep -E 'ROS|RMW|CYCLONE|DDS|AMENT|COLCON'

# Vérifie les domaines ROS2
echo -e "\n${YELLOW}Domaine ROS2:${NC}"
echo -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-'(non défini, valeur par défaut: 0)'}"

# Vérifie le statut du démon ROS2
echo -e "\n${YELLOW}Statut du démon ROS2:${NC}"
ros2 daemon status

# Lister tous les nœuds ROS
echo -e "\n${YELLOW}Liste de tous les nœuds ROS2:${NC}"
ros2 node list

# Créer un nœud de test simple pour la publication
echo -e "\n${YELLOW}Test de publication sur /servo_angles avec un nœud Python:${NC}"
python3 - << 'END_PYTHON'
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time

def main():
    rclpy.init()
    node = Node('test_publisher')
    publisher = node.create_publisher(Int32MultiArray, '/servo_angles', 10)
    
    # Création du message
    msg = Int32MultiArray()
    dim = msg.layout.dim
    dim.append(msg._get_types()[1]())  # MultiArrayDimension
    dim[0].label = 'servos'
    dim[0].size = 12
    dim[0].stride = 12
    msg.layout.data_offset = 0
    msg.data = [91, 46, 136, 91, 46, 46, 91, 46, 136, 91, 46, 46]  # légèrement différent pour voir la différence
    
    node.get_logger().info('Publishing test message to /servo_angles')
    publisher.publish(msg)
    time.sleep(0.1)  # Attendre un peu pour que le message soit envoyé
    
    node.get_logger().info('Message sent successfully')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
END_PYTHON

# Test d'écoute pour vérifier la réception
echo -e "\n${YELLOW}Test d'écoute sur /servo_angles avec un nœud Python:${NC}"
timeout 3 python3 - << 'END_PYTHON'
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time

class TestListener(Node):
    def __init__(self):
        super().__init__('test_listener')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/servo_angles',
            self.listener_callback,
            10)
        self.counter = 0
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Listener started on /servo_angles')
    
    def listener_callback(self, msg):
        self.counter += 1
        self.get_logger().info(f'Message #{self.counter} received: {msg.data}')
    
    def timer_callback(self):
        self.get_logger().info(f'Still listening... ({self.counter} messages received so far)')

def main():
    rclpy.init()
    listener = TestListener()
    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
END_PYTHON

# Vérification des informations du topic
echo -e "\n${YELLOW}Détails du topic /servo_angles:${NC}"
ros2 topic info -v /servo_angles

# Vérifier la connectivité directe avec la commande rostopic
echo -e "\n${YELLOW}Publication directe sur /servo_angles avec ros2 topic pub:${NC}"
ros2 topic pub --once /servo_angles std_msgs/msg/Int32MultiArray "{layout: {dim: [{label: 'servos', size: 12, stride: 12}], data_offset: 0}, data: [90, 45, 135, 90, 45, 45, 90, 45, 135, 90, 45, 45]}"

# Message d'information final
echo -e "\n${BLUE}==============================================${NC}"
echo -e "${BLUE}    Fin du débogage                          ${NC}"
echo -e "${BLUE}==============================================${NC}"

echo -e "\nSi aucun message n'est reçu, il pourrait y avoir un problème avec:"
echo -e "1. La configuration DDS (domaine, QoS)"
echo -e "2. Un conflit de noms entre nœuds ou topics"
echo -e "3. Des problèmes réseau (pare-feu, interfaces multiples)"
echo -e "4. Une limitation CPU/mémoire qui bloque la communication"
echo -e "5. Un problème de permissions système"
