#include "utils/npy_reader.h"
#include <numeric> // 用于 std::accumulate

std::string NPYReader::ReadMagic(std::ifstream &file)
{
    char magic[6];
    file.read(magic, sizeof(magic));
    return std::string(magic, sizeof(magic));
}

uint16_t NPYReader::ReadHeaderLen(std::ifstream &file)
{
    uint16_t header_len;
    file.read(reinterpret_cast<char *>(&header_len), sizeof(header_len));
    return header_len;
}

bool NPYReader::ReadHeader(std::ifstream &file, bool &fortran_order,
                           std::vector<size_t> &shape, size_t &word_size)
{
    // 验证魔数
    std::string magic = ReadMagic(file);
    if (magic != "\x93NUMPY")
    {
        std::cerr << "Invalid NPY file" << std::endl;
        return false;
    }

    // 读取版本
    uint8_t major, minor;
    file.read(reinterpret_cast<char *>(&major), 1);
    file.read(reinterpret_cast<char *>(&minor), 1);

    // 读取头部长度
    uint16_t header_len = ReadHeaderLen(file);

    // 读取头部字符串
    std::string header(header_len, ' ');
    file.read(&header[0], header_len);

    return ParseHeader(header, fortran_order, shape, word_size);
}

bool NPYReader::ParseHeader(const std::string &header, bool &fortran_order,
                            std::vector<size_t> &shape, size_t &word_size)
{
    try
    {
        fortran_order = (header.find("'fortran_order': True") != std::string::npos);

        // 查找 shape 信息
        size_t shape_start = header.find("'shape': (");
        if (shape_start == std::string::npos)
        {
            std::cerr << "Cannot find shape in header" << std::endl;
            return false;
        }
        shape_start += 9; // 跳过 "'shape': ("

        size_t shape_end = header.find(")", shape_start);
        if (shape_end == std::string::npos)
        {
            std::cerr << "Cannot find end of shape in header" << std::endl;
            return false;
        }

        // 提取 shape 字符串并清理
        std::string shape_str = header.substr(shape_start, shape_end - shape_start);

        // std::cout << "Raw shape string: [" << shape_str << "]" << std::endl;

        // 移除所有空白字符
        shape_str.erase(std::remove_if(shape_str.begin(), shape_str.end(),
                                       [](unsigned char c)
                                       { return std::isspace(c); }),
                        shape_str.end());

        // std::cout << "Cleaned shape string: [" << shape_str << "]" << std::endl;

        // 解析 shape 数组
        shape.clear();
        std::string number;
        std::stringstream ss(shape_str);

        while (std::getline(ss, number, ','))
        {
            if (!number.empty())
            {
                // 移除括号和其他非数字字符
                number.erase(std::remove_if(number.begin(), number.end(),
                                            [](unsigned char c)
                                            { return !std::isdigit(c); }),
                             number.end());

                // std::cout << "Parsing number: [" << number << "]" << std::endl;

                if (!number.empty())
                {
                    try
                    {
                        size_t value = std::stoull(number);
                        shape.push_back(value);
                        // std::cout << "Parsed value: " << value << std::endl;
                    }
                    catch (const std::exception &e)
                    {
                        std::cerr << "Failed to parse number: " << number << " - " << e.what() << std::endl;
                        return false;
                    }
                }
            }
        }

        // 验证 shape 数组
        if (shape.size() != 3)
        {
            std::cerr << "Expected 3 dimensions, got " << shape.size() << std::endl;
            return false;
        }

        // 确定数据类型
        if (header.find("'descr': '<f4'") != std::string::npos)
        {
            word_size = 4; // float32
        }
        else
        {
            std::cerr << "Unsupported data type (only float32 is supported)" << std::endl;
            return false;
        }

        // 打印最终的 shape 信息
        // std::cout << "Final shape: [";
        for (size_t i = 0; i < shape.size(); ++i)
        {
            std::cout << shape[i];
            if (i < shape.size() - 1)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error parsing header: " << e.what() << std::endl;
        return false;
    }
}

bool NPYReader::ReadMesh(const std::string &filename, std::vector<std::vector<std::vector<float>>> &data,
                         size_t &num_triangles, size_t &vertices_per_triangle,
                         size_t &coords_per_vertex)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open())
    {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }

    try
    {
        bool fortran_order;
        std::vector<size_t> shape;
        size_t word_size;

        if (!ReadHeader(file, fortran_order, shape, word_size))
        {
            std::cerr << "无法读取 NPY 头部" << std::endl;
            return false;
        }

        if (word_size != sizeof(float))
        {
            std::cerr << "意外的数据类型大小: " << word_size << std::endl;
            return false;
        }

        if (shape.size() != 3)
        {
            std::cerr << "三角形网格应为 3D 数组" << std::endl;
            return false;
        }

        num_triangles = shape[0];
        vertices_per_triangle = shape[1];
        coords_per_vertex = shape[2];

        size_t total_floats = num_triangles * vertices_per_triangle * coords_per_vertex;
        if (total_floats == 0)
        {
            std::cerr << "无效的数组大小: " << total_floats << std::endl;
            return false;
        }

        // 将 data 向量调整为 (n, 3, 3) 的形状
        data.resize(num_triangles);
        for (size_t i = 0; i < num_triangles; ++i)
        {
            data[i].resize(vertices_per_triangle);
            for (size_t j = 0; j < vertices_per_triangle; ++j)
            {
                data[i][j].resize(coords_per_vertex);
            }
        }

        // 将所有 float 值读入临时缓冲区
        std::vector<float> temp_buffer(total_floats);
        file.read(reinterpret_cast<char *>(temp_buffer.data()), total_floats * sizeof(float));

        if (file.fail())
        {
            std::cerr << "读取数据失败" << std::endl;
            return false;
        }

        // 将数据从临时缓冲区复制到正确形状的向量中。
        size_t index = 0;
        for (size_t i = 0; i < num_triangles; ++i)
        {
            for (size_t j = 0; j < vertices_per_triangle; ++j)
            {
                for (size_t k = 0; k < coords_per_vertex; ++k)
                {
                    data[i][j][k] = temp_buffer[index++];
                }
            }
        }

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "读取网格数据时出错: " << e.what() << std::endl;
        return false;
    }
}