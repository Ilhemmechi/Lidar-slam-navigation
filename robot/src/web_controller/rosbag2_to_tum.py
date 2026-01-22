#!/usr/bin/env python3
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from geometry_msgs.msg import PoseWithCovarianceStamped

import sys

bag_path = sys.argv[1]  # chemin vers le dossier du bag
topic_name = sys.argv[2]  # ex: /amcl_pose
output_file = sys.argv[3]  # ex: amcl_tum.txt

# Options pour lire le bag
storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = ConverterOptions('', '')

reader = SequentialReader()
reader.open(storage_options, converter_options)

topics = reader.get_all_topics_and_types()
topic_types = {t.name: t.type for t in topics}

with open(output_file, 'w') as f_out:
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, PoseWithCovarianceStamped)
        timestamp = t / 1e9  # convertir en secondes
        f_out.write(f"{timestamp} {msg.pose.pose.position.x} {msg.pose.pose.position.y} {msg.pose.pose.position.z} "
                    f"{msg.pose.pose.orientation.x} {msg.pose.pose.orientation.y} "
                    f"{msg.pose.pose.orientation.z} {msg.pose.pose.orientation.w}\n")

