import pandas as pd
import xml.etree.ElementTree as ET

import math

def create_xml(file_path, node_name):
    # 读取 XLSX 文件的各个工作表
    map_header = pd.read_excel(file_path, sheet_name='map头相关配置')
    points_data = pd.read_excel(file_path, sheet_name='points相关配置')
    lanes_data = pd.read_excel(file_path, sheet_name='lane相关配置', header=0)
    links_data = pd.read_excel(file_path, sheet_name='Link相关配置', header=0)

    # 清理数据
    links_data['道路宽度（m）'] = pd.to_numeric(links_data['道路宽度（m）'], errors='coerce')
    lanes_data['车道宽度（m）'] = lanes_data['车道宽度（m）'].apply(lambda x: float(x.split('.')[0]) if '.' in str(x) else pd.to_numeric(x, errors='coerce'))
    lanes_data['车道laneID'] = pd.to_numeric(lanes_data['车道laneID'], errors='coerce')
    links_data['道路id'] = pd.to_numeric(links_data['道路id'], errors='coerce')
    
    # Create XML Structure
    message_frame = ET.Element("MessageFrame")
    map_frame = ET.SubElement(message_frame, "mapFrame")
    
    ET.SubElement(map_frame, "msgCnt").text = str(map_header['msgCnt'][0])
    ET.SubElement(map_frame, "timeStamp").text = str(map_header['timeStamp'][0])
    
    nodes = ET.SubElement(map_frame, "nodes")
    node = ET.SubElement(nodes, "Node")
    ET.SubElement(node, "name").text = node_name
    
    id_element = ET.SubElement(node, "id")
    ET.SubElement(id_element, "region").text = str(map_header['region'][0])
    ET.SubElement(id_element, "id").text = str(int(links_data['道路id'].iloc[2]))

    ref_pos = ET.SubElement(node, "refPos")
    ET.SubElement(ref_pos, "lat").text = f"{int(points_data['纬度'][0] * 1e9)}"
    ET.SubElement(ref_pos, "long").text = f"{int(points_data['经度'][0] * 1e9)}"

    # 处理每个 Link
    in_links = ET.SubElement(node, "inLinks")
    for _, link in links_data.iterrows():
        if pd.isna(link['道路id']) or pd.isna(link['道路限速（km/h）']) or pd.isna(link['道路宽度（m）']):
            print(f"Skipping link due to NaN value: {link['name']}")
            continue

        link_element = ET.SubElement(in_links, "Link")
        ET.SubElement(link_element, "name").text = str(link['name'])

        # 生成 link_ids
        link_id_str = str(int(link['道路id']))
        upstream_node_id = ET.SubElement(link_element, "upstreamNodeId")
        ET.SubElement(upstream_node_id, "id").text = link_id_str
        ET.SubElement(upstream_node_id, "region").text = str(map_header['region'][0])

        # 添加 speedLimits
        speed_kmh = float(link['道路限速（km/h）'])
        speed_02ms = (speed_kmh * 1000 / 3600) / 0.02
        regulatory_speed_limit = ET.SubElement(link_element, "speedLimits")
        speed_limit_element = ET.SubElement(regulatory_speed_limit, "RegulatorySpeedLimit")
        type_element = ET.SubElement(speed_limit_element, "type")
        ET.SubElement(type_element, "vehicleMaxSpeed")
        ET.SubElement(speed_limit_element, "speed").text = str(int(speed_02ms))

        # 添加 linkWidth
        link_width = int(link['道路宽度（m）'] * 100)
        ET.SubElement(link_element, "linkWidth").text = str(link_width)
        
# ********************************************************

        # 添加 movements
        link_movements = ET.SubElement(link_element, "movements")  # 注意：这里改为 link_element
        for i in range(7, 15, 2):  # 处理 Movement 信息，每 2 列一组
            movement_id = link.get(link.index[i - 1])  # 获取 Movement ID
            movement_phaseId = link.get(link.index[i])  # 获取 Movement Phase ID

            if pd.notna(movement_id) and pd.notna(movement_phaseId):
                movement_element = ET.SubElement(link_movements, "Movement")
                remote_intersection = ET.SubElement(movement_element, "remoteIntersection")
                # ET.SubElement(remote_intersection, "region").text = "500"  # 固定值
                ET.SubElement(remote_intersection, "region").text = str(map_header['region'][0])
                ET.SubElement(remote_intersection, "id").text = str(int(movement_id))
                ET.SubElement(movement_element, "phaseId").text = str(int(movement_phaseId))
        
# ********************************************************

        # Link 下的 points（保持原样）
        link_points = ET.SubElement(link_element, "points")
        road_ids = points_data['标注'].unique()

        first_node_prefix = None
        for road_id in road_ids:
            if road_id.startswith('in-'):
                first_node_prefix = '-'.join(road_id.split('-')[:2]) + '-'
                break  

        if first_node_prefix:
            related_roads = [road_id for road_id in road_ids if road_id.startswith(first_node_prefix)]
            if related_roads:
                last_related_road_id = related_roads[-1].rsplit('-', 1)[0]
                link_specific_points = points_data[points_data['标注'].str.startswith(last_related_road_id)]

                for _, point in link_specific_points.iterrows():
                    if pd.notna(point['经度']) and pd.notna(point['纬度']):
                        road_point = ET.SubElement(link_points, "RoadPoint")
                        pos_offset = ET.SubElement(road_point, "posOffset")
                        offset_ll = ET.SubElement(pos_offset, "offsetLL")
                        position_latlon = ET.SubElement(offset_ll, "position-LatLon")
                        ET.SubElement(position_latlon, "lon").text = f"{int(point['经度'] * 1e9)}"
                        ET.SubElement(position_latlon, "lat").text = f"{int(point['纬度'] * 1e9)}"
            else:
                print("未找到与第一个节点相关的点")
        else:
            print("未找到道路")

        # 修改 lanes 的逻辑
        lanes = ET.SubElement(link_element, "lanes")
        for _, lane in lanes_data[lanes_data['道路id'] == link['道路id']].iterrows():
            if pd.isna(lane['车道laneID']) or pd.isna(lane['车道宽度（m）']):
                print(f"跳过车道，因NaN值: {lane['车道laneID']}")
                continue

            lane_element = ET.SubElement(lanes, "Lane")
            ET.SubElement(lane_element, "laneId").text = str(int(lane['车道laneID']))
            lane_width_cm = round(float(lane['车道宽度（m）'] * 100))
            ET.SubElement(lane_element, "laneWidth").text = str(lane_width_cm)

            # 添加 maneuvers
            maneuvers_value = str(lane.get('本车道的允许转向行为', ''))
            # if pd.notna(maneuvers_value) and maneuvers_value.strip():
            ET.SubElement(lane_element, "maneuvers").text = maneuvers_value

            # 添加 connectsTo
            connects_to = ET.SubElement(lane_element, "connectsTo")
            for i in range(10, 22, 4):  # 处理 Connection 信息，每 4 列一组
                conn_id = lane.get(lane.index[i - 1])  # 获取 Connection ID
                conn_lane = lane.get(lane.index[i])      # 获取 Connection Lane
                conn_phase_id = lane.get(lane.index[i + 1])  # 获取 Connection Phase ID
                conn_maneuver = lane.get(lane.index[i + 2])  # 获取 Connection maneuver

                print(f"原始 maneuver 值: {conn_maneuver}")

                if pd.notna(conn_id):
                    connection_element = ET.SubElement(connects_to, "Connection")
                    remote_intersection = ET.SubElement(connection_element, "remoteIntersection")
                    ET.SubElement(remote_intersection, "region").text = str(map_header['region'][0])
                    ET.SubElement(remote_intersection, "id").text = str(int(conn_id))

                    connecting_lane = ET.SubElement(connection_element, "connectingLane")
                    # 如果 conn_lane 为空，则默认为 1
                    ET.SubElement(connecting_lane, "lane").text = str(int(conn_lane)) if pd.notna(conn_lane) else "1"
                    
                    # 处理 conn_maneuver 的值
                    if pd.notna(conn_maneuver) and not (isinstance(conn_maneuver, float) and math.isnan(conn_maneuver)):
                        conn_maneuver = str(conn_maneuver).strip()  # 清理空白字符

                        # 保持原始格式为字符串
                        if conn_maneuver.isdigit():
                            ET.SubElement(connecting_lane, "maneuver").text = conn_maneuver
                            print(f"maneuver 值设置为: {conn_maneuver}")
                        else:
                            print(f"无效的 maneuver 值 (不是数字): {conn_maneuver}")
                    else:
                        print(f"跳过 maneuver，由于 NaN 或空值: {conn_maneuver}")

                    if pd.notna(conn_phase_id):
                        ET.SubElement(connection_element, "phaseId").text = str(int(conn_phase_id))


            # 添加 speedLimits
            lane_speed_kmh = pd.to_numeric(lane['车道限速（km/h）'], errors='coerce')
            if pd.notna(lane_speed_kmh):
                lane_speed_02ms = (lane_speed_kmh * 1000 / 3600) / 0.02
                lane_speed_limit = ET.SubElement(lane_element, "speedLimits")
                speed_limit_element = ET.SubElement(lane_speed_limit, "RegulatorySpeedLimit")
                type_element = ET.SubElement(speed_limit_element, "type")
                ET.SubElement(type_element, "vehicleMaxSpeed")
                ET.SubElement(speed_limit_element, "speed").text = str(int(lane_speed_02ms))

            # 添加 lane 的 points
            points = ET.SubElement(lane_element, "points")
            lane_points = points_data[(points_data['标注'].str.contains(f'in-{link_id_str}-{int(lane["车道laneID"])}'))]

            for _, point in lane_points.iterrows():
                if pd.notna(point['经度']) and pd.notna(point['纬度']):
                    road_point = ET.SubElement(points, "RoadPoint")
                    pos_offset = ET.SubElement(road_point, "posOffset")
                    offset_ll = ET.SubElement(pos_offset, "offsetLL")
                    position_latlon = ET.SubElement(offset_ll, "position-LatLon")
                    ET.SubElement(position_latlon, "lon").text = f"{int(point['经度'] * 1e9)}"
                    ET.SubElement(position_latlon, "lat").text = f"{int(point['纬度'] * 1e9)}"


    # 保存 XML 文件
    output_file_path = f"{node_name}_2.4.xml"
    tree = ET.ElementTree(message_frame)
    tree.write(output_file_path, encoding="utf-8", xml_declaration=True)

# 调用函数生成 XML
create_xml('/media/erlang/My Passport/project/simple_v2x/conversion/填充文档/JiGang_center.xlsx', '济钢center1111111')
# create_xml('/home/erlang/文档/node_5/node5.xlsx', 'node_5')