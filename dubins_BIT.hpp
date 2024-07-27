#ifndef BIT_PLANNER_HEADER
#define BIT_PLANNER_HEADER

#include <array>
#include <tuple>
#include <vector>
#include <iostream>
#include <unordered_set>
#include <utility>
#include <queue>
#include <memory>
#include <unordered_map>
#include <limits>
#include <cmath>

#include "planning/utils/nanoflann.hpp"

constexpr float PI = 3.14159265358979323846f;

class Environment_Map
{
    private:
        const std::pair<float,float> m_x_range {};
        const std::pair<float,float> m_y_range {};
        std::vector<std::array<float,4>> m_obstacles {};

    public:
        Environment_Map(const std::pair<float,float> x_range, const std::pair<float,float> y_range)
            : m_x_range {x_range}, m_y_range {y_range}
        {
        }

        void add_obstacles(const std::array<float,4>& obstacle)
        {
            m_obstacles.push_back(obstacle);
        }

        std::vector<std::array<float,4>> get_obstacles() const
        {
            return m_obstacles;
        }
        
        std::pair<float,float> get_xRange() const
        {
            return m_x_range;
        }

        std::pair<float,float> get_yRange() const
        {
            return m_y_range;
        }

        bool point_collision(std::shared_ptr<BIT_Node>& node_ptr)
        {
            /*
            Check if a node/point is feasible to exist in the given map, ensure it is within map bounds+clearances, not inside obstacles or within obs clearance bounds
            */
        }
        bool path_collision() const
        {
            /*
            Check if a path collides with any obstacles and their safety bounds/other collision conditions
            */
        }
};

class BIT_Node
{
    private:
        float x {};
        float y {};
        float heading {};
        std::shared_ptr<BIT_Node> parent {};
    public:
        
        BIT_Node(float arg_x, float arg_y)
            : x {arg_x}, y {arg_y}
        { // Empty constructor body
        }

        float get_x() const 
        {
            return x;
        }
        float get_y() const
        {
            return y;
        }
        float get_heading() const 
        {
            return heading;
        }

        std::shared_ptr<BIT_Node> get_parent() const 
        {
            return parent;
        }

        void set_heading(float heading)
        {
            // Ensure that heading values are within range of [0, 360)
            /*
            Original Heading: -450 -> Normalized Heading: 270 degrees
            Original Heading: 720 -> Normalized Heading: 0 degrees
            Original Heading: 45 -> Normalized Heading: 45 degrees
            Original Heading: -90 -> Normalized Heading: 270 degrees
            Original Heading: 360 -> Normalized Heading: 0 degrees
            */
            float normalized = fmod(heading, 360.0f);
            if (normalized < 0.0f) {
                normalized += 360.0f;
            }
            this->heading = normalized;
        }

        void set_parent(std::shared_ptr<BIT_Node> parent_pointer)
        {
            this->parent = parent_pointer;
        }
        // Equality operator
        bool operator==(const BIT_Node& other) const
        {
            return x == other.x && y == other.y && heading == other.heading && parent == other.parent;
        }


};

typedef std::shared_ptr<BIT_Node> BIT_node_ptr_t;
typedef std::pair<BIT_node_ptr_t, BIT_node_ptr_t> BIT_node_ptr_pair_t;
typedef std::pair<BIT_node_ptr_pair_t,float> EdgeCost_t;
typedef std::pair<BIT_node_ptr_t,float> VertexCost_t;
constexpr float inf = std::numeric_limits<float>::infinity();



// Custom hash function for BIT_Node pointer objects
struct NPHash {
    std::size_t operator()(const BIT_node_ptr_t& np) const
    {
        // Use std::hash for each member
        size_t h1 = std::hash<float>()(np->get_x());
        size_t h2 = std::hash<float>()(np->get_y());
        size_t h3 = std::hash<float>()(np->get_heading());
        size_t h4 = std::hash<BIT_node_ptr_t>()(np->get_parent());
        
        // Combine the hash values using prime numbers
        return h1 ^ (h2 * 2) ^ (h3 * 3) ^ (h4 * 11); // Prime number multipliers
    }
};

// Custom equality functor for std::shared_ptr<BIT_Node>
struct NPNodeEqual {
    bool operator()(const BIT_node_ptr_t& lhs, const BIT_node_ptr_t& rhs) const {
        // Check if the pointed-to objects are equal
        return *lhs == *rhs;
    }
};

// Custom comparison functor for the priority queue
struct CompareQueuePairCosts {
    template<typename T>
    bool operator()(const std::pair<T, float>& lhs, const std::pair<T, float>& rhs) const {
        // Compare based on the float value
        return lhs.second < rhs.second; // Change to < for min heap
    }
};

// Define a custom hash function for std::pair<node_ptr, node_ptr>
struct PairNPHash {
    std::size_t operator()(const BIT_node_ptr_pair_t& p) const {
        std::size_t h1 = NPHash()(p.first);
        std::size_t h2 = NPHash()(p.second);
        return h1 ^ (h2 << 1); // Combine the two hash values
    }
};

// Define a custom equality function for std::pair<node_ptr, node_ptr>, check pointed to nodes are equal
struct PairNPEqual {
    bool operator()(const BIT_node_ptr_pair_t& p1, const BIT_node_ptr_pair_t& p2) const {
        return (*(p1.first) == *(p2.first)) && (*(p1.second) == *(p2.second));
    }
};

// nanoflann KD-tree adapter for BIT_Node
struct NodeKDTreeAdapter
{
    std::unordered_set<BIT_node_ptr_t, NPHash, NPNodeEqual> nodes; // Vector of shared_ptr<BIT_Node>

    // Nanoflann requires a method to return the number of data points
    inline size_t kdtree_get_point_count() const { return nodes.size(); }

    // Nanoflann requires a method to get the vector data point
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const 
    { 
        if (dim >= 2) {
            throw std::out_of_range("Dimension out of bounds in kdtree_get_pt.");
        }

        auto it = nodes.begin();
        std::advance(it, idx);

        if (it == nodes.end()) {
            throw std::out_of_range("Index out of range in kdtree_get_pt.");
        }

        if (dim == 0) {
            return (*it)->get_x();
        } else { // dim == 1
            return (*it)->get_y();
        }
    }

    // Optional: Nanoflann requires methods for computing the bounding box, but not used in this example
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, NodeKDTreeAdapter>,
        NodeKDTreeAdapter,
        2 /* dimension of space (x, y) */,
        float /* type of the coordinate (float here) */
    > KDTree_t;
    

#endif 