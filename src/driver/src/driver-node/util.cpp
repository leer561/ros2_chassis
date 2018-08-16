#include <map>
#include <string>
#include <vector>

namespace driverNodeUtil
{

// 转速转换比例，执行速度调整比例
float GetCoef(float item)
{
    if (item > 128)
    {
        return 256;
    }
    else if (item < -128)
    {
        return 0;
    }
    else
    {
        return item + 128;
    }
};

// 枚举通讯协议的类型
// std::map<std::string, std::vector<std::string> > const protocol{
//     {"setSpeed", {0xea, 0x05, 0x7e, 0x80, 0x80, 0x00, 0x0d}},
//     {"resetEncoder", {0xea, 0x03, 0x35, 0x00, 0x0d}},
//     {"resetBase", {0xea, 0x03, 0x50, 0x00, 0x0d}},
//     {"enableTimeout", {0xea, 0x02, 0x39, 0x00, 0x0d}}};
} // namespace driverNodeUtil