"""PathMap 管理服务 - 解析和修改机器人 pathMap.xml"""

import xml.etree.ElementTree as ET
import math
from typing import Optional
from app.models.path_map import (
    PathMapData, Node, Segment, PickStation, DropStation,
    ChargeStation, Actuator, PickStationPosition
)
import copy

# pathMap.xml 文件路径
PATHMAP_PATH = "/home/jetson/Desktop/keyi/robot/pathMap.xml"


class PathMapManager:
    """PathMap 管理器 - 单例模式"""

    _instance: Optional["PathMapManager"] = None
    _pathmap_path: str = PATHMAP_PATH
    _cached_data: Optional[PathMapData] = None
    _tree: Optional[ET.ElementTree] = None

    def __new__(cls) -> "PathMapManager":
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def parse_xml(self) -> PathMapData:
        """解析 pathMap.xml 文件"""
        tree = ET.parse(self._pathmap_path)
        root = tree.getroot()
        path_elem = root.find("PATH")
        if path_elem is None:
            raise ValueError("pathMap.xml 中未找到 PATH 节点")

        # 缓存 tree 用于后续保存
        self._tree = copy.deepcopy(tree)

        # 解析节点
        nodes = []
        node_elem = path_elem.find("Node")
        if node_elem is not None:
            for n in node_elem.findall("node"):
                nodes.append(Node(
                    id=int(n.get("id")),
                    x=float(n.get("x")),
                    y=float(n.get("y")),
                ))

        # 解析线段
        segments = []
        seg_elem = path_elem.find("Segment")
        if seg_elem is not None:
            for s in seg_elem.findall("segment"):
                segments.append(Segment(
                    id=int(s.get("id")),
                    type=s.get("type", "line"),
                    begin_node=int(s.get("beginNode")),
                    end_node=int(s.get("endNode")),
                    nav_type=s.get("navType", "laser"),
                    max_speed=float(s.get("maxSpeed", "0.8")),
                    move_direction=s.get("moveDirection", "forward"),
                    ox=float(s.get("ox")) if s.get("ox") else None,
                    oy=float(s.get("oy")) if s.get("oy") else None,
                    direction=s.get("direction") if s.get("direction") else None,
                ))

        # 解析取货站
        pick_stations = []
        pick_elem = path_elem.find("PickStation")
        if pick_elem is not None:
            for p in pick_elem.findall("path"):
                pick_stations.append(PickStation(
                    id=int(p.get("id")),
                    connect_node=int(p.get("connectNode")),
                    start_id=int(p.get("startId")),
                    ox=float(p.get("ox")),
                    oy=float(p.get("oy")),
                    R=float(p.get("R")),
                    station_num=int(p.get("stationNum")),
                    max_speed=float(p.get("maxSpeed", "0.4")),
                ))

        # 解析放货站
        drop_stations = []
        drop_elem = path_elem.find("DropStation")
        if drop_elem is not None:
            for d in drop_elem.findall("station"):
                drop_stations.append(DropStation(
                    id=int(d.get("id")),
                    connect_node=int(d.get("connectNode")),
                    node=int(d.get("node")),
                    boom_pos=float(d.get("boomPos", "0")),
                    bucket_pos=float(d.get("bucketPos", "0")),
                    bucket_out_pos=float(d.get("bucketOutPos", "0")),
                ))

        # 解析充电站
        charge_stations = []
        charge_elem = path_elem.find("ChargeStation")
        if charge_elem is not None:
            for c in charge_elem.findall("charge"):
                charge_stations.append(ChargeStation(
                    id=int(c.get("id")),
                    node=int(c.get("node")),
                    vehicle_th=float(c.get("vehicleTh", "0")),
                    ip=c.get("ip", ""),
                    port=int(c.get("port", "0")),
                ))

        # 解析执行器参数
        actuator = None
        act_elem = path_elem.find("PickStation/actuator")
        if act_elem is not None:
            actuator = Actuator(
                pick_boom_pos=float(act_elem.get("pickBoomPos", "0")),
                pick_bucket_pos=float(act_elem.get("pickBucketPos", "0")),
                walk_run_boom_pos=float(act_elem.get("walkRunBoomPos", "0")),
                walk_run_bucket_pos=float(act_elem.get("walkRunBucketPos", "0")),
            )

        self._cached_data = PathMapData(
            nodes=nodes,
            segments=segments,
            pick_stations=pick_stations,
            drop_stations=drop_stations,
            charge_stations=charge_stations,
            actuator=actuator,
        )

        # 计算站点坐标
        self._resolve_coordinates(self._cached_data)
        return self._cached_data

    def get_stations(self) -> PathMapData:
        """获取站点数据（带缓存）"""
        if self._cached_data is None:
            return self.parse_xml()
        return self._cached_data

    def _resolve_coordinates(self, data: PathMapData) -> None:
        """解析后计算站点的实际 xy 坐标"""
        # 构建 node_id -> (x, y) 查找表
        node_map = {n.id: (n.x, n.y) for n in data.nodes}

        # 取货站：按圆形分布算法生成所有站位坐标
        for ps in data.pick_stations:
            if ps.r > 0 and ps.station_num > 0:
                connect_coords = node_map.get(ps.connect_node)
                if connect_coords:
                    cx, cy = connect_coords
                    dx = cx - ps.ox
                    dy = cy - ps.oy
                    dist = math.sqrt(dx * dx + dy * dy)
                    if dist > 0:
                        temp = ps.r / dist
                        # 直线与圆的两个交点
                        A1x = ps.ox + dx * temp
                        A1y = ps.oy + dy * temp
                        A2x = ps.ox - dx * temp
                        A2y = ps.oy - dy * temp
                        d1 = math.sqrt((A1x - cx) ** 2 + (A1y - cy) ** 2)
                        d2 = math.sqrt((A2x - cx) ** 2 + (A2y - cy) ** 2)
                        # 选择离 connectNode 更近的交点作为 startNode
                        sx, sy = (A1x, A1y) if d1 < d2 else (A2x, A2y)

                        # 以圆心为原点，计算 startNode 的相对坐标
                        start_ox = sx - ps.ox
                        start_oy = sy - ps.oy
                        delta_th = 2 * math.pi / ps.station_num

                        positions = []
                        for i in range(ps.station_num):
                            angle = delta_th * i  # 逆时针旋转
                            nx = start_ox * math.cos(angle) - start_oy * math.sin(angle)
                            ny = start_ox * math.sin(angle) + start_oy * math.cos(angle)
                            positions.append(PickStationPosition(
                                station_id=ps.start_id + i,
                                x=nx + ps.ox,
                                y=ny + ps.oy,
                            ))
                        ps.positions = positions

        # 放货站：通过 connectNode 查找坐标
        for ds in data.drop_stations:
            coords = node_map.get(ds.connect_node)
            if coords:
                ds.x = coords[0]
                ds.y = coords[1]

    def refresh(self) -> PathMapData:
        """强制刷新缓存并重新解析"""
        self._cached_data = None
        self._tree = None
        return self.parse_xml()

    def update_pick_station(self, station_id: int, **kwargs) -> PathMapData:
        """更新取货站参数并保存到 XML"""
        # 重新解析最新数据
        data = self.refresh()

        # 找到对应的取货站
        station = None
        for ps in data.pick_stations:
            if ps.id == station_id:
                station = ps
                break

        if station is None:
            raise ValueError(f"取货站 ID {station_id} 不存在")

        # 更新字段
        update_map = {
            "ox": "ox",
            "oy": "oy",
            "R": "R",
            "station_num": "stationNum",
            "max_speed": "maxSpeed",
            "connect_node": "connectNode",
            "start_id": "startId",
        }
        for field, attr in update_map.items():
            if field in kwargs:
                setattr(station, field, kwargs[field])

        # 保存到 XML
        self._save_xml(data)
        return data

    def update_drop_station(self, station_id: int, **kwargs) -> PathMapData:
        """更新放货站参数并保存到 XML"""
        data = self.refresh()

        station = None
        for ds in data.drop_stations:
            if ds.id == station_id:
                station = ds
                break

        if station is None:
            raise ValueError(f"放货站 ID {station_id} 不存在")

        update_map = {
            "node": "node",
            "connect_node": "connectNode",
            "boom_pos": "boomPos",
            "bucket_pos": "bucketPos",
            "bucket_out_pos": "bucketOutPos",
        }
        for field, attr in update_map.items():
            if field in kwargs:
                setattr(station, field, kwargs[field])

        self._save_xml(data)
        return data

    def _save_xml(self, data: PathMapData) -> None:
        """将数据保存回 pathMap.xml（保持原格式）"""
        tree = ET.parse(self._pathmap_path)
        root = tree.getroot()
        path_elem = root.find("PATH")
        if path_elem is None:
            raise ValueError("pathMap.xml 中未找到 PATH 节点")

        # 更新取货站
        pick_elem = path_elem.find("PickStation")
        if pick_elem is not None:
            for ps_xml in pick_elem.findall("path"):
                ps_id = int(ps_xml.get("id"))
                for ps_model in data.pick_stations:
                    if ps_model.id == ps_id:
                        ps_xml.set("ox", str(int(ps_model.ox)))
                        ps_xml.set("oy", str(int(ps_model.oy)))
                        ps_xml.set("R", str(int(ps_model.R)))
                        ps_xml.set("stationNum", str(ps_model.station_num))
                        ps_xml.set("maxSpeed", str(ps_model.max_speed))
                        ps_xml.set("connectNode", str(ps_model.connect_node))
                        ps_xml.set("startId", str(ps_model.start_id))

        # 更新放货站
        drop_elem = path_elem.find("DropStation")
        if drop_elem is not None:
            for ds_xml in drop_elem.findall("station"):
                ds_id = int(ds_xml.get("id"))
                for ds_model in data.drop_stations:
                    if ds_model.id == ds_id:
                        ds_xml.set("node", str(ds_model.node))
                        ds_xml.set("connectNode", str(ds_model.connect_node))
                        ds_xml.set("boomPos", str(int(ds_model.boom_pos)))
                        ds_xml.set("bucketPos", str(int(ds_model.bucket_pos)))
                        ds_xml.set("bucketOutPos", str(int(ds_model.bucket_out_pos)))

        # 写入文件（保持 XML 声明和缩进）
        ET.indent(tree, space="  ")
        tree.write(self._pathmap_path, encoding="UTF-8", xml_declaration=True)
