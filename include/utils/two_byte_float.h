/*************************************************************************************************************************
 * Copyright 2025 Grifcc
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *************************************************************************************************************************/
#include <cstdint>
#include <iomanip>
#include <cmath>

class TwoByteFloat
{
private:
    uint16_t value;

    // 位域分配:
    // - 10位用于整数部分 (0-1023)
    // - 6位用于小数部分，表示千分之几

public:
    // 默认构造函数
    TwoByteFloat(uint16_t v = 0) : value(v) {}

    // 从浮点数构造
    TwoByteFloat(float f)
    {
        if (f < 0 || f >= 1024)
        {
            throw std::out_of_range("Value must be positive and integer part less than 1024");
        }

        // 分离整数和小数部分
        uint16_t intPart = static_cast<uint16_t>(f);
        float fracPart = f - intPart;

        // 将小数部分转为千分位 (确保精度到小数点后3位)
        // 6位最多表示到63，所以我们将小数映射到0-63的范围
        // 1000/64 ≈ 15.625，意味着每一点的增量约为0.015625
        uint16_t fracBits = static_cast<uint16_t>(round(fracPart * 64));
        if (fracBits == 64)
        { // 处理舍入问题
            fracBits = 0;
            intPart++;
            if (intPart >= 1024)
            {
                throw std::out_of_range("Rounding caused overflow in integer part");
            }
        }

        // 组合成16位值
        value = (intPart << 6) | fracBits;
    }

    // 转换回浮点数
    float toFloat() const
    {
        // 提取整数部分（高10位）
        uint16_t intPart = (value >> 6) & 0x3FF;

        // 提取小数部分（低6位）
        uint16_t fracBits = value & 0x3F;

        // 将小数部分转换回浮点数（每个单位代表1/64）
        float fracPart = static_cast<float>(fracBits) / 64.0f;

        return static_cast<float>(intPart) + fracPart;
    }

    // 获取原始16位值
    uint16_t getRawValue() const
    {
        return value;
    }
};
