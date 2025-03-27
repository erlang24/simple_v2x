#include "map_collector/package_utils.hpp"
#include <iostream>
#include <vector>
#include <list>
#include <cstring>
#include <chrono>

using namespace std;

namespace PackageUtils
{
   PackageBuffer::PackageBuffer( uint64_t  sequence_num, uint64_t  total_packets):
        sequence_number(sequence_num), total_packets(total_packets)
    {
        auto now = std::chrono::system_clock::now();
        // 转换为毫秒表示
         this->time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    }

    void PackageBuffer::sort()
    {
        package_list.sort(comparePackage);
    }

    void PackageBuffer::append(uint64_t index, const ByteVector &data)
    {
        Package package = {index,data};
        package_list.push_back(package);
    }

    bool PackageBuffer::pkg_buffer_is_ready()
    {
        // std::cout << " total_packets : " << this->total_packets <<std::endl;
        // std::cout << " package_list size : " << package_list.size() <<std::endl;
        if (package_list.size()==this->total_packets) {
            // std::cout << " sequence_number is raday : " << sequence_number <<std::endl;
            return true;
        }
        return false;
    }

    ByteVector PackageBuffer::get_pkg_buffer_data()
    {
        ByteVector merged_vector;
        // 遍历排序后的 package_list
        for (const auto& pkg : package_list) {
            merged_vector.insert(merged_vector.end(), pkg.data.begin(), pkg.data.end());
        }
        return merged_vector;
    }

    uint64_t PackageBuffer::get_sequence_number()
    {
        return this->sequence_number;
    }
    uint64_t PackageBuffer::get_total_packets()
    {
        return this->total_packets;
    }

    uint64_t PackageBuffer::get_packets_count()
    {
        return this->package_list.size();
    }
    


    PacketHandler::PacketHandler(){};

    bool PacketHandler::push_buffer(const ByteVector &bytes)  // 放入缓存
    {
        this->cleanup_timeout_pkg();

        if (bytes.size() <= (sizeof(uint64_t))){
            return false;
        }

        uint64_t sequence_number;
        uint64_t total_packets;
        uint64_t index; 

        Deserialize(bytes, sequence_number,total_packets,index);  // 
        // std::cout << " recive_sequence_number: " <<  sequence_number;
        // std::cout << " total_packets: " <<  total_packets;
        // std::cout << " index: " <<  index << std::endl;

        for (auto& pkg_buffer : this->package_buffer_array) {
            if (pkg_buffer.get_sequence_number() ==  sequence_number){
                 ByteVector  data =  ByteVector(bytes.begin() + sizeof(uint64_t)*3, bytes.end());
                 pkg_buffer.append(index,data);

                // std::cout << " insert sequence_number: " <<  sequence_number <<  
                // " total_packets : " <<total_packets << " index: "<<index<< std::endl;

                 return true;
            }
        }

        PackageBuffer pkg_buffer(sequence_number,total_packets);
        ByteVector  data =  ByteVector(bytes.begin() + sizeof(uint64_t)*3, bytes.end());
        pkg_buffer.append(index,data);

        // std::cout << " insert sequence_number: " <<  sequence_number <<  
        // " total_packets : " <<total_packets << " index: "<<index<< std::endl;
        this->package_buffer_array.push_back(pkg_buffer);
        if ( this->package_buffer_array.size() > 255 ){
            this->package_buffer_array.erase(this->package_buffer_array.begin());
        }

        return true;
    }

    uint64_t PacketHandler::pack(ByteVector &data) // 组包的实现
    {
        bool clean_flag = false;
        uint64_t sequence_number_ready = 0;        
        for (auto& pkg_buffer : this->package_buffer_array) {
            if (pkg_buffer.pkg_buffer_is_ready()){
                pkg_buffer.sort();
                sequence_number_ready =pkg_buffer.get_sequence_number();
                data = pkg_buffer.get_pkg_buffer_data();
                clean_flag = true;
            }
        }
        // std::cout << "clean_flag : " << clean_flag  << " package_buffer_array size : " <<this->package_buffer_array.size()<< std::endl;
        if (clean_flag) this->cleanup_history_pkg(sequence_number_ready);
        return sequence_number_ready;
    }    

    // 拆包的实现
    std::vector<ByteVector> PacketHandler::unpack(const ByteVector& inputVector, size_t chunkSize)
    {

        size_t inputSize = inputVector.size();


        if (inputSize <= (sizeof(uint64_t))){
        
            return std::vector<ByteVector>();
        }

        uint64_t total_packets = inputSize / chunkSize; 
        if (inputSize % chunkSize > 0)total_packets +=1;
        
        /////////
        size_t numChunks = inputSize / chunkSize;
        std::vector<std::vector<uint8_t>> result;

        auto sequence_number = this->send_sequence_number ++;
        if (this->send_sequence_number > sequence_number_max){
            this->send_sequence_number =0;
        }



        uint64_t index; 
        for (size_t i = 0; i < numChunks; ++i)
        {
            index =i; 
            ByteVector pkg_data_head; 
            this->Serialize(pkg_data_head, sequence_number,total_packets,index);

            auto pkg_data_body = ByteVector(inputVector.begin() + i * chunkSize,
                                            inputVector.begin() + (i + 1) * chunkSize);
            
            pkg_data_head.insert(pkg_data_head.end(), pkg_data_body.begin(), pkg_data_body.end());

            result.push_back(pkg_data_head);
        }

        // Handle the remaining elements if the input size is not a multiple of chunk size
        size_t remaining = inputSize % chunkSize;
        if (remaining > 0){
            index = total_packets-1; 
            if (this->send_sequence_number > sequence_number_max)
                this->send_sequence_number =0;
            ByteVector pkg_data_head; 
            this->Serialize(pkg_data_head, sequence_number,total_packets,index);

            auto pkg_data_body = ByteVector(inputVector.end() - remaining, inputVector.end());

            pkg_data_head.insert(pkg_data_head.end(), pkg_data_body.begin(), pkg_data_body.end());
            result.push_back(pkg_data_head);

        }
        // std::cout << " send_sequence_number: " <<  this->send_sequence_number;
        // std::cout << " total_packets: " <<  total_packets;
        // std::cout << " index: " <<  index << std::endl;
        return result;
    }


void PacketHandler::cleanup_timeout_pkg(){
    // 清除package_buffer_array中的成员 , 旧数据
    auto now = std::chrono::system_clock::now();
    auto current_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());

    for (auto it = package_buffer_array.begin(); it != package_buffer_array.end();) {
        auto time_diff_ms = current_time_ms - it->time_ms;

        if (time_diff_ms.count() > 1000) { // 如果时间差超过阈值，删除该成员
            std::cout <<"timeout -> sequence_number:" << it->get_sequence_number() << 
                " total size : " << it->get_total_packets() << 
                " size : " << it->get_packets_count() << std::endl;
            it = package_buffer_array.erase(it);
            continue;;
        }
        it++;
    }
}



void PacketHandler::cleanup_history_pkg(uint64_t sequence_number){

    for (auto it = package_buffer_array.begin(); it != package_buffer_array.end();) {
        if (sequence_number == 0 ){
            if (this->sequence_number_max - it->get_sequence_number() < 10 ){
                it = package_buffer_array.erase(it);
                std::cout <<"cleanup_history_pkg -> " << 
                    ", sequence_number_raday : " << sequence_number << 
                    ", sequence_number : " << it->get_sequence_number() << 
                    ", total size : " << it->get_total_packets() << 
                    ", size : " << it->get_packets_count() << std::endl;

                continue;
            }
        }

        if(it->get_sequence_number() == sequence_number){
            it = package_buffer_array.erase(it);
            continue;
        }
        if(it->get_sequence_number() < sequence_number){
            it = package_buffer_array.erase(it);
            std::cout <<"cleanup_history_pkg -> " << 
                ", sequence_number_raday : " << sequence_number << 
                ", sequence_number : " << it->get_sequence_number() << 
                ", total size : " << it->get_total_packets() << 
                ", size : " << it->get_packets_count() << std::endl;

            continue;
        }
        ++it;
    }
}


// 序列化
void PacketHandler::Serialize(std::vector<uint8_t>& bytes, const uint64_t sequence_number, const uint64_t total_packets,const uint64_t index) const 
{
    // 将每个变量的字节表示依次添加到向量中
    const uint8_t* seqBytes = reinterpret_cast<const uint8_t*>(&sequence_number);
    bytes.insert(bytes.end(), seqBytes, seqBytes + sizeof(sequence_number));

    const uint8_t* totalBytes = reinterpret_cast<const uint8_t*>(&total_packets);
    bytes.insert(bytes.end(), totalBytes, totalBytes + sizeof(total_packets));

    const uint8_t* indexBytes = reinterpret_cast<const uint8_t*>(&index);
    bytes.insert(bytes.end(), indexBytes, indexBytes + sizeof(index));
    
}

// 反序列化
void PacketHandler::Deserialize(const ByteVector& bytes, uint64_t &sequence_number,uint64_t &total_packets,uint64_t &index) 
{
    // 从字节中提取对应的值并赋给相应的变量
    std::memcpy(&sequence_number, &bytes[0], sizeof(sequence_number));
    std::memcpy(&total_packets, &bytes[sizeof(sequence_number)], sizeof(total_packets));
    std::memcpy(&index, &bytes[sizeof(sequence_number) + sizeof(total_packets)], sizeof(index));
}


}
