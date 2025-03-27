def convert_to_decimal(coord, coord_type):
    """
    根据坐标类型将整数坐标转换为十进制度数。
    如果是纬度lat取前两位加小数点。
    如果是经度lon取后三位加小数点。
    """
    coord_str = str(coord)
    if coord_type == 'lat':
        # 纬度：取前两位加小数点
        decimal_coord = float(coord_str[:2] + '.' + coord_str[2:])
    elif coord_type == 'lon':
        # 经度：取后三位加小数点
        decimal_coord = float(coord_str[:3] + '.' + coord_str[3:])
    else:
        raise ValueError("Invalid coordinate type. Use 'lat' or 'lon'.")
    return decimal_coord

def calculate_difference(center_lat, center_lon, other_lat, other_lon):
    """计算其他坐标与中心坐标的差值并乘以10的7次方"""
    # 计算经度和纬度的差值
    lat_diff = other_lat - center_lat
    lon_diff = other_lon - center_lon
    
    # 乘以10的7次方
    lat_result = lat_diff * 10**7
    lon_result = lon_diff * 10**7
    
    return lat_result, lon_result

# 中心点的经纬度坐标
center_coordinates = {
    "lat": 3675197690,
    "long": 11725553117
}

# 将中心点的坐标转换为十进制
center_lat = convert_to_decimal(center_coordinates["lat"], 'lat')
center_lon = convert_to_decimal(center_coordinates["long"], 'lon')

# 其他经纬度坐标
other_coordinates = {
"lon": "1172554997532",
                                                                "lat": "3675232690838"
}

# 将其他坐标转换为十进制
other_lat = convert_to_decimal(other_coordinates["lat"], 'lat')
other_lon = convert_to_decimal(other_coordinates["lon"], 'lon')

print(f"中心点经度: {center_lon}")
print(f"中心点纬度: {center_lat}")
print(f"其他点经度: {other_lon}")
print(f"其他点纬度: {other_lat}")

# 计算差值
lat_result, lon_result = calculate_difference(center_lat, center_lon, other_lat, other_lon)

lat_result_rounded = round(lat_result)
lon_result_rounded = round(lon_result)
# lat_result_rounded = (lat_result)
# lon_result_rounded = (lon_result)

# 输出结果
print(f"=====================经度差结果: {lon_result_rounded}")
print(f"=====================纬度差结果: {lat_result_rounded}")

# ----------------------------------------------------------------------------------------------------------

# # 定义中心点的经纬度
# center_lat = 36.75197690
# center_long = 117.25553117

# # 定义其他点的经纬度
# other_lat = 36.75232690838
# other_long = 117.2554997532

# # 计算偏差并乘以10的七次方
# lat_deviation = (other_lat - center_lat) * 10**7
# long_deviation = (other_long - center_long) * 10**7


# # print(round(lat_deviation * 10**-7) + center_lat)
# print((round(lat_deviation )+0.0837999632495) * 10**-7 + center_lat)

# print((round(lat_deviation )) * 10**-7 + center_lat)
# print(round(long_deviation))
# print(round(long_deviation )* 10**-7 + center_long) 

# # 输出结果
# print(f"纬度偏差: {lat_deviation}")
# print(f"经度偏差: {long_deviation}")

