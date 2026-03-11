#!/usr/bin/env python3

import math
import os
import xml.etree.ElementTree as ET
from typing import Dict, Iterable, Optional, Tuple

import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Pose, Quaternion
from gazebo_msgs.srv import SpawnEntity


COLOR_MAP: Dict[str, Tuple[float, float, float, float]] = {
    "blue": (0 / 255.0, 0 / 255.0, 168 / 255.0, 1.0),
    "green": (0 / 255.0, 100 / 255.0, 0 / 255.0, 1.0),
    "red": (139 / 255.0, 0 / 255.0, 0 / 255.0, 1.0),
    "purple": (138 / 255.0, 0 / 255.0, 226 / 255.0, 1.0),
    "orange": (255 / 255.0, 140 / 255.0, 0 / 255.0, 1.0),
}

PART_TYPES = {"battery", "pump", "sensor", "regulator"}

DEFAULT_PARTS: Dict[str, Tuple[float, float, float, float, float, float]] = {
    "green_battery": (-0.25, -0.10, 1.025, 0.0, 0.0, 0.0),
    "blue_pump": (-0.25, 0.10, 1.025, 0.0, 0.0, 0.0),
    "red_regulator": (-0.35, -0.10, 1.025, 0.0, 0.0, 0.0),
    "orange_sensor": (-0.35, 0.10, 1.025, 0.0, 0.0, 0.0),
}


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    half_roll = roll / 2.0
    half_pitch = pitch / 2.0
    half_yaw = yaw / 2.0

    sin_r = math.sin(half_roll)
    cos_r = math.cos(half_roll)
    sin_p = math.sin(half_pitch)
    cos_p = math.cos(half_pitch)
    sin_y = math.sin(half_yaw)
    cos_y = math.cos(half_yaw)

    q = Quaternion()
    q.x = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y
    q.y = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y
    q.z = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y
    q.w = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y
    return q


class AssemblyPartSpawner(Node):
    def __init__(self) -> None:
        super().__init__("assembly_part_spawner")

        self.declare_parameter(
            "models_path",
            os.path.join(get_package_share_directory("kortex_bringup"), "models"),
        )
        self.declare_parameter("parts_yaml", "")

        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")

    def run(self) -> None:
        parts_yaml = (
            self.get_parameter("parts_yaml").get_parameter_value().string_value
        )

        if parts_yaml:
            try:
                loaded = yaml.safe_load(parts_yaml) or {}
            except yaml.YAMLError as exc:
                self.get_logger().error(
                    f"解析 parts_yaml 失败，将使用默认配置。错误: {exc}"
                )
                loaded = {}
        else:
            loaded = {}

        if not isinstance(loaded, dict) or not loaded:
            parts_param = DEFAULT_PARTS
        else:
            parts_param = loaded

        if not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/spawn_entity 服务不可用，无法生成零件。")
            return

        models_path = self.get_parameter("models_path").get_parameter_value().string_value

        for key, pose_list in parts_param.items():
            color_name, part_type = self._parse_part_key(key)
            if color_name is None or part_type is None:
                continue

            pose_tuple = self._sanitize_pose(pose_list, key)
            if pose_tuple is None:
                continue

            model_file = os.path.join(models_path, part_type, "model.sdf")

            if not os.path.isfile(model_file):
                self.get_logger().error(f"[{key}] 模型文件不存在: {model_file}")
                continue

            xml_string = self._load_sdf(model_file)
            if not xml_string:
                self.get_logger().error(f"[{key}] 读取模型文件失败: {model_file}")
                continue

            color_rgba = COLOR_MAP[color_name]
            colored_xml = self._apply_color(xml_string, color_rgba)
            pose = self._build_pose(pose_tuple)

            success = self._spawn_entity(
                name=key,
                xml=colored_xml,
                pose=pose,
            )

            if success:
                self.get_logger().info(f"零件 {key} 生成成功（{part_type}, 颜色 {color_name}）。")
            else:
                self.get_logger().warn(f"零件 {key} 生成失败。")

    def _parse_part_key(self, key: str) -> Tuple[Optional[str], Optional[str]]:
        tokens = key.split("_", 1)
        if len(tokens) != 2:
            self.get_logger().error(f"[{key}] 命名格式错误，需为 <color>_<type>。")
            return None, None

        color_name, part_type = tokens[0], tokens[1]

        if color_name not in COLOR_MAP:
            available = ", ".join(sorted(COLOR_MAP.keys()))
            self.get_logger().error(
                f"[{key}] 颜色 {color_name} 不支持，可选项: {available}。"
            )
            return None, None

        if part_type not in PART_TYPES:
            available = ", ".join(sorted(PART_TYPES))
            self.get_logger().error(
                f"[{key}] 零件类型 {part_type} 不支持，可选项: {available}。"
            )
            return None, None

        return color_name, part_type

    def _sanitize_pose(
        self, pose_list: Iterable[float], key: str
    ) -> Optional[Tuple[float, float, float, float, float, float]]:
        try:
            values = [float(v) for v in pose_list]
        except (TypeError, ValueError):
            self.get_logger().error(f"[{key}] pose 需为 6 个数字。当前值: {pose_list}")
            return None

        if len(values) != 6:
            self.get_logger().error(f"[{key}] pose 长度必须为 6 (x, y, z, roll, pitch, yaw)。")
            return None

        return tuple(values)

    def _load_sdf(self, file_path: str) -> str:
        try:
            with open(file_path, "r", encoding="utf-8") as sdf_file:
                return sdf_file.read()
        except OSError as exc:
            self.get_logger().error(f"打开模型文件失败: {file_path} ({exc})")
            return ""

    def _apply_color(self, xml_string: str, color_rgba) -> str:
        xml_root = ET.fromstring(xml_string)
        color_text = " ".join(str(component) for component in color_rgba)

        for visual in xml_root.findall(".//visual"):
            if visual.get("name") != "base":
                continue

            material = visual.find("material")
            if material is None:
                material = ET.SubElement(visual, "material")

            ambient = material.find("ambient")
            if ambient is None:
                ambient = ET.SubElement(material, "ambient")
            ambient.text = color_text

            diffuse = material.find("diffuse")
            if diffuse is None:
                diffuse = ET.SubElement(material, "diffuse")
            diffuse.text = color_text

        return ET.tostring(xml_root, encoding="unicode")

    def _build_pose(self, pose_tuple) -> Pose:
        pose = Pose()
        pose.position.x = pose_tuple[0]
        pose.position.y = pose_tuple[1]
        pose.position.z = pose_tuple[2]

        pose.orientation = quaternion_from_euler(
            pose_tuple[3],
            pose_tuple[4],
            pose_tuple[5],
        )

        return pose

    def _spawn_entity(self, name: str, xml: str, pose: Pose) -> bool:
        request = SpawnEntity.Request()
        request.name = name
        request.xml = xml
        request.robot_namespace = ""
        request.reference_frame = "world"
        request.initial_pose = pose

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(f"/spawn_entity 调用失败: {future.exception()}")
            return False

        return future.result().success


def main(args=None) -> None:
    rclpy.init(args=args)
    spawner = AssemblyPartSpawner()
    spawner.run()
    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

