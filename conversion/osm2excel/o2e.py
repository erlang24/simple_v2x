import xml.etree.ElementTree as ET
import pandas as pd

# 解析 OSM 文件
tree = ET.parse('/home/erlang/simple_v2x/conversion/osm2excel/lanelet2_maps.osm')
root = tree.getroot()

# 存储节点信息
nodes = {}
for node in root.findall('node'):
    node_id = node.get('id')
    lat = node.get('lat')
    lon = node.get('lon')
    nodes[node_id] = {'lat': lat, 'lon': lon}

# 分类道路
ways = {'west': [], 'south': [], 'east': [], 'north': []}
way_mapping = {
    '58': 'west',
    '127': 'south',
    '220': 'east',
    '256': 'north'
}

# 提取每条道路的点信息
for way in root.findall('way'):
    way_id = way.get('id')
    if way_id in way_mapping:
        direction = way_mapping[way_id]
        for nd in way.findall('nd'):
            ref = nd.get('ref')
            if ref in nodes:
                ways[direction].append((ref, nodes[ref]['lat'], nodes[ref]['lon']))

# 提取中心点
center_points = []
for node_id, coords in nodes.items():
    found = False
    for direction, points in ways.items():
        if any(node_id == point[0] for point in points):
            found = True
            break
    if not found:
        center_points.append((node_id, coords['lat'], coords['lon']))

# 将数据写入 Excel 文件
with pd.ExcelWriter('roads_data66.xlsx') as writer:
    for direction, points in ways.items():
        df = pd.DataFrame(points, columns=['节点ID', '纬度', '经度'])
        df.to_excel(writer, sheet_name=direction, index=False)

    center_df = pd.DataFrame(center_points, columns=['节点ID', '纬度', '经度'])
    center_df.to_excel(writer, sheet_name='中心点', index=False)
