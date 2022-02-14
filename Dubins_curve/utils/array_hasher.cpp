// #include <iostream>
// #include <unordered_map>
#include <map>
// #include <algorithm>

struct ArrayHasher{
        std::size_t operator()(const std::array<int, 2> &a) const{
            std::size_t h = 0;
            for (auto e : a){
                h ^= std::hash<int>{}(e) + 0x9e3779b9 + (h << 6) + (h >> 2);
            }
            return h;
        }
    };


struct ArrayHasher3{
        std::size_t operator()(const std::array<int, 3> &a) const{
            std::size_t h = 0;
            for (auto e : a){
                h ^= std::hash<int>{}(e) + 0x9e3779b9 + (h << 6) + (h >> 2);
            }
            return h;
        }
    };


