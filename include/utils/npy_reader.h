#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cstring>
#include <cstdint>
#include <algorithm>
#include <sstream>
#include <cctype>

class NPYReader {
public:
    static bool ReadTriangleMesh(const std::string& filename, std::vector<std::vector<std::vector<float>>>& data,
                                size_t& num_triangles, size_t& vertices_per_triangle,
                                size_t& coords_per_vertex);
private:
    static bool ReadHeader(std::ifstream& file, bool& fortran_order, 
                          std::vector<size_t>& shape, size_t& word_size);
    static std::string ReadMagic(std::ifstream& file);
    static uint16_t ReadHeaderLen(std::ifstream& file);
    static bool ParseHeader(const std::string& header, bool& fortran_order, 
                          std::vector<size_t>& shape, size_t& word_size);
};