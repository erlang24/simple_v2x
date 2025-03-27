#include <cstdio>
#include "map_collector/package_utils.hpp"
#include <boost/asio.hpp>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>

#include <fstream>
#include <nlohmann/json.hpp>

#include <vector>
#include <algorithm>
#include <random>

#include <cstdlib> // 包含 rand() 和 srand() 函数
#include <ctime> // 包含 time() 函数

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

typedef std::chrono::high_resolution_clock Clock;

PackageUtils::PacketHandler packetHandler;


class MapCollector : public rclcpp::Node
{
public:
  nlohmann::json map_data_;
  std::vector<PackageUtils::ByteVector> packets;
  boost::asio::io_context io_context;  // IO 上下文
  boost::asio::ip::udp::socket socket; 
  boost::asio::ip::udp::endpoint receive_endpoint;
  MapCollector()
  : Node("MapCollector"),
    io_context(),
    socket(io_context),
    receive_endpoint(boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string("192.168.62.224"), 30301))
        {
            submsgs_=this->create_subscription<std_msgs::msg::UInt8MultiArray>("/map", rclcpp::QoS{5},
                                std::bind(&MapCollector::packages, this, std::placeholders::_1));
            udp_sub_ = std::make_shared<std::thread>(
                        [this]() {
                            UdpSub();
                        });
        }
    ~MapCollector() {
        if (udp_sub_) {
        udp_sub_->join();
        }
    }
    PackageUtils::ByteVector test_packetHandler(PackageUtils::ByteVector & packet);
    void UdpSub();
    
private:
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr submsgs_{nullptr};
    void packages(const std_msgs::msg::UInt8MultiArray::SharedPtr msgIn);
    std::shared_ptr<std::thread> udp_sub_ = nullptr;
};

PackageUtils::ByteVector MapCollector::test_packetHandler(PackageUtils::ByteVector & packet)
    {
        packetHandler.push_buffer(packet);

        PackageUtils::ByteVector mergedData;
        uint64_t sequenceNumber = packetHandler.pack(mergedData);
        if (mergedData.size() > 0){
            std::cout << "组包序列号: " << sequenceNumber << std::endl;
            std::cout << "组包大小: " << mergedData.size() << std::endl;
        }
        return mergedData;
    }
void MapCollector::packages(const std_msgs::msg::UInt8MultiArray::SharedPtr msgIn)
  {
    PackageUtils::ByteVector betyData (msgIn->data.begin(), msgIn->data.end());
    PackageUtils::ByteVector map = test_packetHandler(betyData);
    size_t size=map.size(); 
    if (size > 0)
    {   nlohmann::json map_json_;
        map_json_ = nlohmann::json::parse(map);
        RCLCPP_INFO(this->get_logger(), "size of packets: %zu", size);
        std::cout << map_json_["MessageFrame"]["mapFrame"]["timeStamp"] << std::endl;
    }
  }
void MapCollector::UdpSub()
    {   
        socket.open(boost::asio::ip::udp::v4());
        socket.bind(receive_endpoint);
        PackageUtils::ByteVector buffer(1024);
        while (rclcpp::ok())
        {
            std::size_t length = socket.receive_from(boost::asio::buffer(buffer), receive_endpoint);
            std::cout << "Received " << length << " bytes from " 
                      << receive_endpoint.address().to_string() << ":" 
                      << receive_endpoint.port() << std::endl;

            // 将接收到的数据转换为 std::vector<uint8_t> 并输出
            PackageUtils::ByteVector received_data(buffer.begin(), buffer.begin() + length);
            PackageUtils::ByteVector map = test_packetHandler(received_data);
            size_t size=map.size(); 
            std::cout << "size" << size << std::endl;
            if (size > 0)
            {   nlohmann::json map_json_;
                map_json_ = nlohmann::json::parse(map);
                // 判断
                // RCLCPP_INFO(this->get_logger(), "size of packets: %zu", size);
                std::cout << map_json_["MessageFrame"]["mapFrame"]["timeStamp"] << std::endl;
            }
        }
        
    }

