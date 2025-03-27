import xmltodict
import json

# 读取 XML 文件
with open('/home/erlang/simple_v2x/济钢center_2.4.xml', 'r', encoding='utf-8') as xml_file:
    xml_content = xml_file.read()

# 将 XML 转换为字典
data_dict = xmltodict.parse(xml_content)

# 将字典转换为 JSON
json_data = json.dumps(data_dict, indent=4, ensure_ascii=False)

# 输出 JSON 数据
# print(json_data)

# 可选：保存为 JSON 文件
with open('JiGang_Center88.json', 'w', encoding='utf-8') as json_file:
    json_file.write(json_data)