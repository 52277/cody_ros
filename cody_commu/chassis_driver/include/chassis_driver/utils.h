#pragma once
#include <iostream>
#include <vector>
#include <sstream>
#include <iomanip>
#include <chrono>

// 将十六进制字符串转换为字节数组
char* hexStringToBytes(const std::string& hexString) {
    std::istringstream iss(hexString);
    std::vector<int> values;
    int value;
    
    while (iss >> std::hex >> value) {
        values.push_back(value);
    }
    
    // 将整数值存储在char数组中
    char* charArray = new char[values.size()];
    for (size_t i = 0; i < values.size(); ++i) {
        charArray[i] = static_cast<char>(values[i]);
    }
    return charArray;
}

bool find_all(std::vector<size_t> &positions, const std::string &my_string, const std::string &substr)
{
    if(my_string.find(substr, 0) == std::string::npos)
    {
        return false;
    }
    else
    {
        size_t pos = my_string.find(substr, 0); // 从索引0开始查找第一个出现的子串位置
        while (pos != std::string::npos) { // npos 表示未找到
            positions.push_back(pos); // 将找到的位置索引添加到向量中
            pos = my_string.find(substr, pos + 1); // 从下一个位置开始查找下一个子串位置
        }
        return true;
    }
    
}


//定时器调试can频率
class Timer
{
    public:
        Timer()
        {
            m_StartTimepoint = std::chrono::high_resolution_clock::now();
        }

        ~Timer()
        {
            stop();
        }

        void stop()
        {
            auto endTimepoint = std::chrono::high_resolution_clock::now();
            auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimepoint).time_since_epoch().count();
            auto end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimepoint).time_since_epoch().count();
            auto duration = end - start;
            double ms = duration * 0.001;
            std::cerr << duration  << "us (" << ms << "ms)\n";
     
     
        }

    private:
        std::chrono::time_point< std::chrono::high_resolution_clock> m_StartTimepoint;
};
