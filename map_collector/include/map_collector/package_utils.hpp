// #include "package_utils.hpp"
#include <iostream>
#include <vector>
#include <list>
#include <cstring>
#include <chrono>

using namespace std;

namespace PackageUtils
{

// 定义类型别名
using ByteVector = std::vector<uint8_t>;    

struct Package{
    uint64_t  index; 
    ByteVector data;
};

class PackageBuffer{

 public:
    PackageBuffer( uint64_t  sequence_num, uint64_t  total_packets);

    // callback  
    static bool comparePackage(const Package& p1, const Package& p2) 
    {
        return p1.index < p2.index;
    }

    void sort();

    void append(uint64_t index, const ByteVector &data);

    bool pkg_buffer_is_ready();

    ByteVector get_pkg_buffer_data();

    uint64_t get_sequence_number();
    uint64_t get_total_packets();

    uint64_t get_packets_count();
    
    std::chrono::milliseconds time_ms;

 private:
    uint64_t  sequence_number; 
    uint64_t  total_packets; 

    std::list<Package> package_list;

};


class PacketHandler{
 public:
    PacketHandler();

    bool push_buffer(const ByteVector &bytes);  // 放入缓存

    uint64_t pack(ByteVector &data); // 组包的实现

    // 拆包的实现
    std::vector<ByteVector> unpack(const ByteVector& inputVector, size_t chunkSize);


 private:

    void cleanup_timeout_pkg();
    void cleanup_history_pkg(uint64_t sequence_number);


    // 序列化
    void Serialize(std::vector<uint8_t>& bytes, const uint64_t sequence_number, const uint64_t total_packets,const uint64_t index) const ;

    // 反序列化
    void Deserialize(const ByteVector& bytes, uint64_t &sequence_number,uint64_t &total_packets,uint64_t &index) ;

 private:

    uint64_t send_sequence_number = 0;
    uint64_t sequence_number_max = 255;

    std::vector<PackageBuffer> package_buffer_array;
    const uint64_t sub_package_max_size = 1024*20 /*51200**/;
};

};
