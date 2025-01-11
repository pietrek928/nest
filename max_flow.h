#include <vector>
#include <queue>


using Tvertex = int;
using Tweight = int;
using Tflow = int;


typedef struct FlowEdge {
    Tvertex a, b;  // a < b
    Tflow cap_fwd, cap_bwd;
    Tweight w_fwd, w_bwd;
} FlowEdge;

typedef struct FlowGraph {
    // 0=src ; 1=snk
    std::vector<FlowEdge> edges;
    std::vector<std::vector<Tvertex>> adj;
} FlowGraph;
