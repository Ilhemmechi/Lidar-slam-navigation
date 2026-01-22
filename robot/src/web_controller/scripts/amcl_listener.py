#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class AMCLListener(Node):
    def __init__(self):
        super().__init__('amcl_listener')
        # Souscrire au topic AMCL
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',  # topic AMCL
            self.listener_callback,
            10
        )
        self.subscription  # éviter l’avertissement "unused variable"

    def listener_callback(self, msg):
        # Récupérer la position et orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Afficher ou sauvegarder
        print(f"{t} {x} {y} {z} {qx} {qy} {qz} {qw}")
        # Tu peux aussi écrire dans un fichier pour EVO
        with open('amcl_live.txt', 'a') as f:
            f.write(f"{t} {x} {y} {z} {qx} {qy} {qz} {qw}\n")

def main(args=None):
    rclpy.init(args=args)
    node = AMCLListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


