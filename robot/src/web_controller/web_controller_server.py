#!/usr/bin/env python3
"""
Serveur Web pour contrôler un robot ROS2 via une interface web
Utilise rclpy et Flask/SocketIO pour la communication
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import json
import threading
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from flask import Flask, render_template_string, request, jsonify
from flask_socketio import SocketIO, emit
import subprocess
import os
import math

class RobotWebController(Node):
    def __init__(self):
        super().__init__('robot_web_controller')
        
        # QoS Profile pour communication fiable
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile)
        
        # Timer pour vérification de sécurité (arrêt automatique)
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        self.last_command_time = time.time()
        self.safety_timeout = 2.0  # secondes
        
        # Flask app
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'robot_control_ros2_secret'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # Variables d'état
        self.current_velocity = Twist()
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.connected_clients = 0
        self.is_moving = False
        
        # Configuration des routes et événements
        self.setup_routes()
        self.setup_socketio_events()
        
        # Démarrer rosbridge pour ROS2
        self.start_rosbridge_ros2()
        
        self.get_logger().info("Contrôleur web ROS2 initialisé")

    def start_rosbridge_ros2(self):
        """Démarre rosbridge_server pour ROS2"""
        try:
            # Vérifier si rosbridge est déjà en cours
            result = subprocess.run(['pgrep', '-f', 'rosbridge'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info("rosbridge_server déjà en cours d'exécution")
                return
            
            # Démarrer rosbridge_server pour ROS2
            self.get_logger().info("Démarrage de rosbridge_server pour ROS2...")
            subprocess.Popen(['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
                           stdout=subprocess.DEVNULL, 
                           stderr=subprocess.DEVNULL)
            time.sleep(3)
            self.get_logger().info("rosbridge_server ROS2 démarré")
            
        except Exception as e:
            self.get_logger().warn(f"Impossible de démarrer rosbridge_server: {e}")
            self.get_logger().warn("Démarrez manuellement: ros2 launch rosbridge_server rosbridge_websocket_launch.xml")

    def safety_check(self):
        """Vérification de sécurité - arrêt automatique si pas de commande"""
        if self.is_moving and (time.time() - self.last_command_time) > self.safety_timeout:
            self.get_logger().warn("Timeout de sécurité - arrêt du robot")
            self.emergency_stop()

    def setup_routes(self):
        """Configuration des routes Flask"""
        
        @self.app.route('/')
        def index():
            """Page d'accueil avec interface de base"""
            return '''
            <!DOCTYPE html>
            <html>
            <head>
                <title>Robot ROS2 Control Interface</title>
                <meta charset="UTF-8">
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
                <style>
                    body { 
                        font-family: 'Segoe UI', sans-serif; 
                        margin: 40px; 
                        background: linear-gradient(135deg, #1e3c72, #2a5298);
                        color: white;
                        min-height: 100vh;
                    }
                    .container {
                        max-width: 800px;
                        margin: 0 auto;
                        background: rgba(255,255,255,0.1);
                        padding: 30px;
                        border-radius: 20px;
                        backdrop-filter: blur(10px);
                    }
                    h1 { 
                        text-align: center; 
                        margin-bottom: 30px;
                        font-size: 2.5rem;
                    }
                    .status-grid {
                        display: grid;
                        grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
                        gap: 20px;
                        margin-bottom: 30px;
                    }
                    .status-card {
                        background: rgba(255,255,255,0.1);
                        padding: 20px;
                        border-radius: 15px;
                        text-align: center;
                    }
                    .status-value {
                        font-size: 1.5rem;
                        font-weight: bold;
                        color: #4fd1c7;
                    }
                    .controls {
                        display: grid;
                        grid-template-columns: repeat(3, 1fr);
                        gap: 15px;
                        max-width: 400px;
                        margin: 20px auto;
                    }
                    button { 
                        padding: 15px 20px; 
                        margin: 5px; 
                        font-size: 16px;
                        border: none;
                        border-radius: 10px;
                        background: linear-gradient(135deg, #667eea, #764ba2);
                        color: white;
                        cursor: pointer;
                        transition: all 0.3s ease;
                    }
                    button:hover {
                        transform: translateY(-2px);
                        box-shadow: 0 5px 15px rgba(0,0,0,0.3);
                    }
                    .stop-btn {
                        background: linear-gradient(135deg, #ff6b6b, #ee5a52);
                    }
                    .speed-control {
                        text-align: center;
                        margin: 20px 0;
                    }
                    .speed-slider {
                        width: 300px;
                        height: 8px;
                        border-radius: 5px;
                        background: rgba(255,255,255,0.3);
                        outline: none;
                    }
                </style>
            </head>
            <body>
                <div class="container">
                    <h1>🤖 Robot ROS2 Control</h1>
                    
                    <div class="status-grid">
                        <div class="status-card">
                            <h3>Position</h3>
                            <div class="status-value" id="position">x: 0, y: 0</div>
                        </div>
                        <div class="status-card">
                            <h3>Vitesse Linéaire</h3>
                            <div class="status-value" id="linear-velocity">0.0 m/s</div>
                        </div>
                        <div class="status-card">
                            <h3>Vitesse Angulaire</h3>
                            <div class="status-value" id="angular-velocity">0.0 rad/s</div>
                        </div>
                    </div>
                    
                    <div class="speed-control">
                        <h3>Vitesse</h3>
                        <input type="range" id="speed-slider" class="speed-slider" 
                               min="0.1" max="2.0" step="0.1" value="0.5">
                        <div>Vitesse: <span id="speed-value">0.5</span> m/s</div>
                    </div>
                    
                    <div class="controls">
                        <div></div>
                        <button onmousedown="sendCommand(getSpeed(), 0)" onmouseup="stopRobot()">⬆️ Avant</button>
                        <div></div>
                        
                        <button onmousedown="sendCommand(0, 0.5)" onmouseup="stopRobot()">⬅️ Gauche</button>
                        <button class="stop-btn" onclick="emergencyStop()">🛑 STOP</button>
                        <button onmousedown="sendCommand(0, -0.5)" onmouseup="stopRobot()">➡️ Droite</button>
                        
                        <div></div>
                        <button onmousedown="sendCommand(-getSpeed(), 0)" onmouseup="stopRobot()">⬇️ Arrière</button>
                        <div></div>
                    </div>
                    
                    <div style="text-align: center; margin-top: 30px;">
                        <p>🎮 Utilisez les boutons ou les touches WASD/flèches</p>
                        <p>🌐 Interface complète: Ouvrez le fichier HTML séparé</p>
                    </div>
                </div>
                
                <script>
                    let currentSpeed = 0.5;
                    
                    // Gestion du slider
                    document.getElementById('speed-slider').addEventListener('input', function() {
                        currentSpeed = parseFloat(this.value);
                        document.getElementById('speed-value').textContent = currentSpeed.toFixed(1);
                    });
                    
                    function getSpeed() { return currentSpeed; }
                    
                    function sendCommand(linear, angular) {
                        fetch('/cmd_vel', {
                            method: 'POST',
                            headers: {'Content-Type': 'application/json'},
                            body: JSON.stringify({linear: linear, angular: angular})
                        });
                    }
                    
                    function stopRobot() {
                        sendCommand(0, 0);
                    }
                    
                    function emergencyStop() {
                        fetch('/emergency_stop', {method: 'POST'});
                    }
                    
                    // Contrôles clavier
                    let keysPressed = {};
                    document.addEventListener('keydown', function(e) {
                        if (keysPressed[e.key]) return;
                        keysPressed[e.key] = true;
                        
                        switch(e.key.toLowerCase()) {
                            case 'w': case 'arrowup': sendCommand(getSpeed(), 0); break;
                            case 's': case 'arrowdown': sendCommand(-getSpeed(), 0); break;
                            case 'a': case 'arrowleft': sendCommand(0, 0.5); break;
                            case 'd': case 'arrowright': sendCommand(0, -0.5); break;
                            case ' ': emergencyStop(); break;
                        }
                    });
                    
                    document.addEventListener('keyup', function(e) {
                        keysPressed[e.key] = false;
                        switch(e.key.toLowerCase()) {
                            case 'w': case 's': case 'a': case 'd':
                            case 'arrowup': case 'arrowdown': case 'arrowleft': case 'arrowright':
                                stopRobot(); break;
                        }
                    });
                    
                    // Mise à jour de l'état
                    setInterval(function() {
                        fetch('/status')
                            .then(response => response.json())
                            .then(data => {
                                document.getElementById('position').textContent = 
                                    `x: ${data.pose.x.toFixed(2)}, y: ${data.pose.y.toFixed(2)}`;
                                document.getElementById('linear-velocity').textContent = 
                                    `${data.velocity.linear.toFixed(2)} m/s`;
                                document.getElementById('angular-velocity').textContent = 
                                    `${data.velocity.angular.toFixed(2)} rad/s`;
                            })
                            .catch(e => console.log('Status update failed'));
                    }, 500);
                </script>
            </body>
            </html>
            '''
        
        @self.app.route('/cmd_vel', methods=['POST'])
        def cmd_vel():
            """Endpoint pour envoyer des commandes de vitesse"""
            try:
                data = request.get_json()
                linear = float(data.get('linear', 0.0))
                angular = float(data.get('angular', 0.0))
                
                # Limiter les vitesses
                linear = max(-2.0, min(2.0, linear))
                angular = max(-1.5, min(1.5, angular))
                
                # Créer et publier le message Twist
                twist = Twist()
                twist.linear.x = linear
                twist.angular.z = angular
                
                self.cmd_vel_pub.publish(twist)
                self.current_velocity = twist
                self.last_command_time = time.time()
                self.is_moving = (abs(linear) > 0.01 or abs(angular) > 0.01)
                
                self.get_logger().info(f"Commande - Linear: {linear:.2f}, Angular: {angular:.2f}")
                
                return jsonify({
                    'status': 'success',
                    'linear': linear,
                    'angular': angular,
                    'timestamp': time.time()
                })
                
            except Exception as e:
                self.get_logger().error(f"Erreur cmd_vel: {e}")
                return jsonify({'status': 'error', 'message': str(e)}), 500
        
        @self.app.route('/status')
        def status():
            """Endpoint pour obtenir l'état du robot"""
            return jsonify({
                'pose': self.robot_pose,
                'velocity': {
                    'linear': self.current_velocity.linear.x,
                    'angular': self.current_velocity.angular.z
                },
                'connected_clients': self.connected_clients,
                'is_moving': self.is_moving,
                'ros2_time': time.time()
            })
        
        @self.app.route('/emergency_stop', methods=['POST'])
        def emergency_stop_route():
            """Arrêt d'urgence via HTTP"""
            self.emergency_stop()
            return jsonify({'status': 'emergency_stop_activated'})

    def setup_socketio_events(self):
        """Configuration des événements SocketIO pour ROS2"""
        
        @self.socketio.on('connect')
        def handle_connect():
            self.connected_clients += 1
            self.get_logger().info(f"Client WebSocket connecté. Total: {self.connected_clients}")
            emit('status', {
                'connected': True,
                'message': 'Connecté au robot ROS2',
                'node_name': self.get_name()
            })
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            self.connected_clients -= 1
            self.get_logger().info(f"Client WebSocket déconnecté. Total: {self.connected_clients}")
            # Arrêter le robot si plus de clients connectés
            if self.connected_clients == 0:
                self.emergency_stop()
        
        @self.socketio.on('cmd_vel')
        def handle_cmd_vel_ws(data):
            """Gestion des commandes via WebSocket"""
            try:
                linear = float(data.get('linear', 0.0))
                angular = float(data.get('angular', 0.0))
                
                # Limites de sécurité
                linear = max(-2.0, min(2.0, linear))
                angular = max(-1.5, min(1.5, angular))
                
                twist = Twist()
                twist.linear.x = linear
                twist.angular.z = angular
                
                self.cmd_vel_pub.publish(twist)
                self.current_velocity = twist
                self.last_command_time = time.time()
                self.is_moving = (abs(linear) > 0.01 or abs(angular) > 0.01)
                
                emit('cmd_vel_ack', {
                    'linear': linear,
                    'angular': angular,
                    'timestamp': time.time()
                })
                
            except Exception as e:
                self.get_logger().error(f"Erreur WebSocket cmd_vel: {e}")
                emit('error', {'message': str(e)})
        
        @self.socketio.on('get_status')
        def handle_get_status():
            """Envoyer l'état du robot via WebSocket"""
            emit('robot_status', {
                'pose': self.robot_pose,
                'velocity': {
                    'linear': self.current_velocity.linear.x,
                    'angular': self.current_velocity.angular.z
                },
                'is_moving': self.is_moving,
                'timestamp': time.time()
            })

    def odom_callback(self, msg):
        """Callback pour les données d'odométrie ROS2"""
        try:
            # Position
            self.robot_pose['x'] = msg.pose.pose.position.x
            self.robot_pose['y'] = msg.pose.pose.position.y
            
            # Orientation (quaternion vers euler)
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            
            # Conversion quaternion -> angle yaw
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            self.robot_pose['theta'] = math.atan2(siny_cosp, cosy_cosp)
            
            # Envoyer aux clients WebSocket
            if self.connected_clients > 0:
                self.socketio.emit('odom_update', {
                    'pose': self.robot_pose,
                    'velocity': {
                        'linear': msg.twist.twist.linear.x,
                        'angular': msg.twist.twist.angular.z
                    },
                    'timestamp': time.time()
                })
                
        except Exception as e:
            self.get_logger().error(f"Erreur odom_callback: {e}")

    def emergency_stop(self):
        """Arrêt d'urgence"""
        twist = Twist()  # Tous les champs à zéro
        self.cmd_vel_pub.publish(twist)
        self.current_velocity = twist
        self.is_moving = False
        self.get_logger().warn("🛑 ARRÊT D'URGENCE ACTIVÉ")
        
        # Notifier les clients WebSocket
        if self.connected_clients > 0:
            self.socketio.emit('emergency_stop', {
                'message': 'Arrêt d\'urgence activé',
                'timestamp': time.time()
            })

    def run_web_server(self, host='0.0.0.0', port=5000):
        """Lancer le serveur web dans un thread séparé"""
        self.get_logger().info(f"🌐 Serveur web démarré sur http://{host}:{port}")
        
        server_thread = threading.Thread(
            target=self.socketio.run,
            args=(self.app,),
            kwargs={
                'host': host, 
                'port': port, 
                'debug': False, 
                'use_reloader': False,
                'log_output': False
            }
        )
        server_thread.daemon = True
        server_thread.start()
        
        return server_thread

    def shutdown(self):
        """Arrêt propre"""
        self.get_logger().info("Arrêt du contrôleur web ROS2...")
        self.emergency_stop()


def main(args=None):
    """Fonction principale ROS2"""
    rclpy.init(args=args)
    
    try:
        # Créer le nœud
        controller = RobotWebController()
        
        # Démarrer le serveur web
        web_thread = controller.run_web_server(host='0.0.0.0', port=5000)
        
        controller.get_logger().info("=== CONTRÔLEUR WEB ROS2 DÉMARRÉ ===")
        controller.get_logger().info("🌐 Interface web: http://localhost:5000")
        controller.get_logger().info("🔌 WebSocket ROS2: ws://localhost:9090")
        controller.get_logger().info("⌨️  Ctrl+C pour arrêter")
        
        # Spin ROS2
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info("Arrêt demandé par l'utilisateur")
    except Exception as e:
        controller.get_logger().error(f"Erreur critique: {e}")
    finally:
        if 'controller' in locals():
            controller.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
