import json

def convert_to_decimal(coord, coord_type):
    coord_str = str(coord)
    if coord_type == 'lat':
        decimal_coord = float(coord_str[:2] + '.' + coord_str[2:])
    elif coord_type == 'lon':
        decimal_coord = float(coord_str[:3] + '.' + coord_str[3:])
    return decimal_coord

def calculate_difference(center_lat, center_lon, other_lat, other_lon):
    lat_diff = other_lat - center_lat
    lon_diff = other_lon - center_lon
    lat_result = round(lat_diff * 10**7)
    lon_result = round(lon_diff * 10**7)
    return lat_result, lon_result

def transform_points(center_lat, center_lon, points):
    transformed_points = []
    for point in points["RoadPoint"]:
        lon = int(point["posOffset"]["offsetLL"]["position-LatLon"]["lon"])
        lat = int(point["posOffset"]["offsetLL"]["position-LatLon"]["lat"])
        other_lat = convert_to_decimal(lat, 'lat')
        other_lon = convert_to_decimal(lon, 'lon')
        lat_offset, lon_offset = calculate_difference(center_lat, center_lon, other_lat, other_lon)
        transformed_points.append({
            "posOffset": {
                "offsetLL": ("position-LL2", {
                    "lon": lon_offset,
                    "lat": lat_offset
                })
            }
        })
    return transformed_points

def transform_lanes(center_lat, center_lon, lanes):
    transformed_lanes = []
    if isinstance(lanes, dict):
        lanes = [lanes]
        
    for lane in lanes:
        connects_to = []
        for connection in lane["connectsTo"]["Connection"]:
            connects_to.append({
                "remoteIntersection": {
                    "region": int(connection["remoteIntersection"]["region"]),
                    "id": int(connection["remoteIntersection"]["id"])
                },
                "connectingLane": {
                    "lane": int(connection["connectingLane"]["lane"]),
                    "maneuver": (b'100000000000', 96)
                },
                "phaseId": int(connection["phaseId"])
            })

        points = transform_points(center_lat, center_lon, lane["points"])

        transformed_lanes.append({
            "laneID": int(lane["laneId"]),
            "laneWidth": int(lane["laneWidth"]),
            "maneuvers": (b'110000000000', 96),
            "connectsTo": connects_to,
            "speedLimits": [{"type": 5, "speed": 416}],
            "points": points
        })
    return transformed_lanes

def process_node(node):
    center_lat = convert_to_decimal(int(node["refPos"]["lat"]), 'lat')
    center_lon = convert_to_decimal(int(node["refPos"]["long"]), 'lon')

    new_in_links = []
    for link in node["inLinks"]["Link"]:
        points = transform_points(center_lat, center_lon, link["points"])
        lanes = transform_lanes(center_lat, center_lon, link["lanes"]["Lane"])

        new_link = {
            "name": link["name"],
            "upstreamNodeId": {
                "id": int(link["upstreamNodeId"]["id"]),
                "region": int(link["upstreamNodeId"]["region"])
            },
            "speedLimits": [{"type": 5, "speed": 416}],
            "linkWidth": int(link["linkWidth"]),
            "movements": [
                {
                    "remoteIntersection": {
                        "region": 500,
                        "id": movement["remoteIntersection"]["id"]
                    },
                    "phaseId": int(movement["phaseId"])
                }
                for movement in link["movements"]["Movement"]
            ],
            "points": points,
            "lanes": lanes
        }
        new_in_links.append(new_link)
    return new_in_links

def generate_map_content(data):
    map_frame = data["MessageFrame"]["mapFrame"]
    nodes = map_frame["nodes"]["Node"]
    
    result = {
        "msgCnt": int(map_frame["msgCnt"]),
        "timeStamp": int(map_frame["timeStamp"]),
        "nodes": [{
            "name": nodes["name"],
            "id": {
                "region": int(nodes["id"]["region"]),
                "id": int(nodes["id"]["id"])
            },
            "refPos": {
                "lat": int(nodes["refPos"]["lat"]),
                "long": int(nodes["refPos"]["long"])
            },
            "inLinks": process_node(nodes)
        }]
    }
    
    output_content = f"""# Generated Map Data
MAP_DATA = {result}

def get_map_data():
    return MAP_DATA
"""
    return output_content

def main():
    with open('/home/erlang/simple_v2x/rsu/JiGang_Center.json', 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    content = generate_map_content(data)
    
    with open('generated_map.py', 'w', encoding='utf-8') as f:
        f.write(content)

if __name__ == "__main__":
    main()
