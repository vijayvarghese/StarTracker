#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <unordered_map>


struct StarPair {
    std::string id1;
    std::string id2;
};
std::unordered_map<int, std::vector<StarPair>> lookup;


#include <chrono>


//loadBinary("stars.bin", lookup);








void write_to_bin(const std::string& name, const std::unordered_map<int, std::vector<StarPair>>& lookup){
    
    std::fstream out(name,std::ios::binary | std::ios::out); 
    if(!out)return;

    size_t mapSize = lookup.size();
    std::cout<<"Lookup Map size : "<<sizeof(size_t)<<" , Entry Count : "<<mapSize<<std::endl;
    out.write(reinterpret_cast<const char*>(&mapSize),sizeof(mapSize));
    int idx = 0;
    for (const auto& [key, vec] : lookup) {
        out.write(reinterpret_cast<const char*>(&key), sizeof(key));

        size_t vecSize = vec.size();
        out.write(reinterpret_cast<const char*>(&vecSize), sizeof(vecSize));

        for (const auto& sp : vec){
            size_t s1Len = sp.id1.size();
        out.write(reinterpret_cast<const char*>(&s1Len), sizeof(s1Len));
        out.write(sp.id1.data(),s1Len);

            size_t s2Len = sp.id2.size();
        out.write(reinterpret_cast<const char*>(&s2Len), sizeof(s2Len));
        out.write(sp.id2.data(),s2Len);
        
        }
        idx++;
        std::cout<<"\rMaps Written to bin : "<<idx;
        std::cout.flush();
    }
std::cout<<"Done ! -> "<<idx<<std::endl;

}



void loadBin(const std::string& filename,
                std::unordered_map<int, std::vector<StarPair>>& lookup)
{
    std::ifstream in(filename, std::ios::binary);
    if (!in) return;

    lookup.clear();

    size_t mapSize;
    in.read(reinterpret_cast<char*>(&mapSize), sizeof(mapSize));

    for (size_t i = 0; i < mapSize; ++i) {
        int key;
        in.read(reinterpret_cast<char*>(&key), sizeof(key));

        size_t vecSize;
        in.read(reinterpret_cast<char*>(&vecSize), sizeof(vecSize));

        std::vector<StarPair> vec(vecSize);

        for (size_t j = 0; j < vecSize; ++j) {
            size_t len1, len2;

            in.read(reinterpret_cast<char*>(&len1), sizeof(len1));
            vec[j].id1.resize(len1);
            in.read(&vec[j].id1[0], len1);

            in.read(reinterpret_cast<char*>(&len2), sizeof(len2));
            vec[j].id2.resize(len2);
            in.read(&vec[j].id2[0], len2);
        }

        lookup.emplace(key, std::move(vec));
    }
}




int main() {
    //int idx = 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    std::ifstream file("../data/processed/6.5/hipparcos_mag65_lookup_fov23.csv");
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file." << std::endl;
        return 1;
    }

    std::string line;
    
    // --- SKIP THE HEADER LINE HERE ---
    if (!std::getline(file, line)) {
        return 0; // File is empty
    }

    // Now start processing data lines
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;

        while (std::getline(ss, cell, ',')) {
            row.push_back(cell);
        }
        
        // Safety check: ensure row has at least 3 columns before accessing index 2
        if (row.size() >= 3) {
            try {
                double val = std::stod(row[2]);
                int bin = std::round(val / 0.01);
                lookup[bin].push_back({row[0],row[1]});
                //std::cout<<"Bin Processed lookup populated : "<<bin<<std::endl;
                //idx++;
                //std::cout.flush();
                //std::cout << "lookup[" << bin << "].push_back({\"" 
                //  << row[0] << "\",\"" 
                //  << row[1] << "\"});" << std::endl;

            } catch (const std::invalid_argument& e) {
                // Skips any malformed data lines that aren't the header
                continue; 
            }
        }
    }
auto t2 = std::chrono::high_resolution_clock::now();
std::cout << "CsV load: "
          << std::chrono::duration<double>(t2 - t1).count()
          << " sec\n";
    std::cout<<"Done"<<std::endl;

    write_to_bin("mag65_fov23.bin",lookup);

lookup.clear();


 t1 = std::chrono::high_resolution_clock::now();
loadBin("mag65_fov23.bin",lookup);

 t2 = std::chrono::high_resolution_clock::now();
std::cout << "Binary load: "
          << std::chrono::duration<double>(t2 - t1).count()
          << " sec\n";
    
    
          std::cout<<"Done"<<std::endl;



auto it = lookup.find(134);

if (it == lookup.end()) {
    std::cout << "Key 134 not found\n";
} 
else if (it->second.empty()) {
    std::cout << "Key 134 exists but vector is empty\n";
} 
else {
    std::cout << it->second[0].id1 
              << " , " 
              << it->second[0].id2 
              << std::endl;
}



    return 0;
}
