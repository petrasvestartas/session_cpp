#include "spatial_rtree.h"
#include "exact_boxes.h"
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    int target = -1;
    std::string indices;
    for (int i = 1; i < argc; ++i) {
        const std::string flag(argv[i]);
        if (flag == "--target" && i + 1 < argc) target = std::atoi(argv[++i]);
        else if (flag == "--indices" && i + 1 < argc) indices = argv[++i];
        else return 3;
    }
    if (target >= static_cast<int>(box_bits.size())) return 3;
    std::vector<int> order;
    if (indices.empty()) {
        for (int i = 0; i < static_cast<int>(box_bits.size()); ++i)
            if (target < 0 || i <= target) order.push_back(i);
    } else {
        std::ifstream input(indices);
        if (!input) return 3;
        int index;
        while (input >> index) {
            if (index < 0 || index >= static_cast<int>(box_bits.size())) return 3;
            order.push_back(index);
        }
        if (!input.eof()) return 3;
    }
    std::array<std::array<double,4>,647> boxes;
    for (size_t i = 0; i < box_bits.size(); ++i)
        for (int d = 0; d < 4; ++d) boxes[i][d] = std::bit_cast<double>(box_bits[i][d]);
    const double tolerance = std::bit_cast<double>(tolerance_bits);
    const double inf = std::numeric_limits<double>::infinity();
    session_cpp::SpatialRTree<size_t,double,2> tree;
    std::vector<int> inserted;
    size_t total = 0, mismatch_queries = 0, missing = 0, extra = 0;
    bool target_seen = target < 0;
    std::printf("{\"mismatches\":[");
    for (int id : order) {
        const auto& box = boxes[id];
        const std::array<double,2> lo={box[0],box[1]},hi={box[2],box[3]};
        if (target < 0 || id == target) {
            target_seen = true;
            std::array<double,2> qlo,qhi;
            for (int d=0;d<2;++d) {
                qlo[d]=std::nextafter(lo[d]-tolerance,-inf);
                qhi[d]=std::nextafter(hi[d]+tolerance,inf);
            }
            std::vector<size_t> found,expected;
            tree.search(qlo.data(),qhi.data(),[&](size_t j){found.push_back(j);return true;});
            for (int j : inserted) {
                const auto& previous=boxes[j];
                bool overlap=true;
                for(int d=0;d<2;++d)
                    if(qhi[d]<previous[d] || previous[2+d]<qlo[d]) overlap=false;
                if(overlap) expected.push_back(static_cast<size_t>(j));
            }
            std::sort(found.begin(),found.end());
            std::sort(expected.begin(),expected.end());
            total+=found.size();
            if(found!=expected) {
                if(mismatch_queries++) std::printf(",");
                std::printf("{\"query\":%d,\"found\":[",id);
                for(size_t k=0;k<found.size();++k) std::printf("%s%zu",k?",":"",found[k]);
                std::printf("],\"expected\":[");
                for(size_t k=0;k<expected.size();++k) std::printf("%s%zu",k?",":"",expected[k]);
                std::printf("]}");
                for(size_t j:expected) missing+=std::find(found.begin(),found.end(),j)==found.end();
                for(size_t j:found) extra+=std::find(expected.begin(),expected.end(),j)==expected.end();
            }
        }
        tree.insert(lo.data(),hi.data(),static_cast<size_t>(id));
        inserted.push_back(id);
    }
    std::printf("],\"queries_mismatched\":%zu,\"missing\":%zu,\"extra\":%zu,\"pairs\":%zu,\"insertions\":%zu,\"target_seen\":%s}\n",mismatch_queries,missing,extra,total,inserted.size(),target_seen?"true":"false");
    if(!target_seen) return 3;
    return mismatch_queries?1:0;
}
