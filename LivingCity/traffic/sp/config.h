#include <climits>
#include <limits>

#ifndef _ABM_CONFIG_H_
#define _ABM_CONFIG_H_
#define INIT_EDGE_INDEX_NOT_SET std::numeric_limits<int>::max() // max int value
#define END_OF_PATH std::numeric_limits<int>::max() // max int value
#define FLOAT_COMPARISON_EPSILON 0.0001

namespace abm {
namespace graph {
//! Vertex id type
using vertex_t = long int;
using edge_id_t = int;
//! Weight type, that can be added with +
using weight_t = double;

}  // namespace graph
}  // namespace abm

struct personPath {
    uint person_id;
    std::vector<int> pathInVertexes;
};

struct parameters{
    float a;
    float b;
    float T;
    float s_0;
};

#endif  // _ABM_CONFIG_H_


