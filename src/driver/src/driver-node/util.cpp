namespace driverNodeUtil
{

// 转速转换比例，执行速度调整比例
float GetCoef(float item)
{
    if (item > 255)
    {
        return 255;
    }
    else if (item < 0)
    {
        return 0;
    }
    else
    {
        return item;
    }
};

} // namespace driverNodeUtil