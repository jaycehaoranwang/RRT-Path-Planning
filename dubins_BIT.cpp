#include "dubins_BIT.hpp"
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
#include <Eigen/Dense>

class Node
{
    private:
        Node *parent {};

    public:
        float x {};
        float y {};
        float heading {};

        Node(float arg_x, float arg_y)
            : x {arg_x}, y {arg_y}
        { // Empty constructor body
        }

        std::pair<float, float> get_location() const 
        {
            return std::make_pair(x,y);
        }

        float getHeading() const 
        {
            return heading;
        }

        Node* get_parent() const 
        {
            return parent;
        }

        void set_heading(float heading)
        {
            this->heading = heading;
        }

        void set_parent(Node * parent_pointer)
        {
            this->parent = parent_pointer;
        }
        // Equality operator
        bool operator==(const Node& other) const
        {
            return x == other.x && y == other.y && heading == other.heading && parent == other.parent;
        }


};



class Environment_Map
{
    private:
        const std::pair<int,int> m_x_range {};
        const std::pair<int,int> m_y_range {};
        std::vector<std::array<float,4>> m_obstacles {};

    public:
        Environment_Map(const std::pair<int,int> x_range, const std::pair<int,int> y_range)
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

        bool path_collision() const
        {
        }
};


// Custom hash function for Node objects
struct NodeHash {
    size_t operator()(const std::shared_ptr<Node>& node) const
    {
        // Use std::hash for each member
        size_t h1 = std::hash<float>()(node->get_location().first);
        size_t h2 = std::hash<float>()(node->get_location().second);
        size_t h3 = std::hash<float>()(node->getHeading());
        size_t h4 = std::hash<Node*>()(node->get_parent());
        
        // Combine the hash values using prime numbers
        return h1 ^ (h2 * 31) ^ (h3 * 37) ^ (h4 * 41); // Prime number multipliers
    }
};

// Custom equality functor for std::shared_ptr<Node>
struct SharedPtrNodeEqual {
    bool operator()(const std::shared_ptr<Node>& lhs, const std::shared_ptr<Node>& rhs) const {
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

typedef std::pair<std::pair<Node*, Node*>,float> EdgeCostPairType;
typedef std::pair<Node*,float> VertexCostPairType;

class BIT_Dubins_Planner
{
    private:
        Node m_start;
        Node m_goal;
        const std::vector<float>& m_discrete_headings {};  // Reference member variable
        Environment_Map& m_env_map;
        const float m_min_turning_radius {};
        const int m_max_iters {};
        float m_search_radius {};
        std::unordered_set<std::shared_ptr<Node>> m_map_samples {};
        std::unordered_set<std::shared_ptr<Node>> m_vertex_set {};
        std::unordered_set<std::pair<Node*, Node*>> m_edge_set {};
        std::priority_queue<EdgeCostPairType, std::vector<EdgeCostPairType>, CompareQueuePairCosts> m_edge_queue {};
        std::priority_queue<VertexCostPairType, std::vector<VertexCostPairType>, CompareQueuePairCosts> m_vertex_queue {};
        std::unordered_map<std::shared_ptr<Node>, float, NodeHash, SharedPtrNodeEqual> m_cost_to_node_table {};

        std::pair<double, double> calc_dist_and_angle(const Node& start_node, const Node& end_node)
        {
            float dx {end_node.x - start_node.x};
            float dy {end_node.y - start_node.y};
            return std::make_pair(std::hypot(dx, dy),std::atan2(dy, dx));
        };

        // Function to perform rotation to world frame
        Eigen::Matrix3d RotationToWorldFrame(const Node& x_start, const Node& x_goal, double L) {
            Eigen::Vector3d a1((x_goal.x - x_start.x) / L, (x_goal.y - x_start.y) / L, 0.0);
            Eigen::Vector3d e1(1.0, 0.0, 0.0);
            Eigen::Matrix3d M = a1 * e1.transpose();
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3d U = svd.matrixU();
            Eigen::Matrix3d V_T = svd.matrixV().transpose();
            double det_UV_T = U.determinant() * V_T.determinant();
            Eigen::Matrix3d C = U * Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, det_UV_T) * V_T;

            return C;
        }

    public:
        BIT_Dubins_Planner(Node& start, Node& goal, const std::vector<float>& discrete_headings, Environment_Map& env_map, float search_radius, const int max_iters, const float min_turning_radius)
            : m_start(start)
            , m_goal(goal)
            , m_discrete_headings {discrete_headings}
            , m_env_map(env_map)
            , m_min_turning_radius {min_turning_radius}
            , m_max_iters {max_iters}
            , m_search_radius {search_radius}
        {
            auto m_shared_start_ptr = std::make_shared<Node>(m_start);
            auto m_shared_goal_ptr = std::make_shared<Node>(m_goal);

            m_map_samples.insert(m_shared_goal_ptr);
            m_vertex_set.insert(m_shared_start_ptr);
            m_cost_to_node_table[m_shared_start_ptr] = 0.0f;
            m_cost_to_node_table[m_shared_goal_ptr] = std::numeric_limits<float>::infinity();

            std::pair<double, double> ellipse_params {calc_dist_and_angle(m_start, m_goal)}; //pair is (cMin, theta)
            double m_cMin = ellipse_params.first;
            double m_theta = ellipse_params.second;
            Eigen::Matrix3d m_C {RotationToWorldFrame(m_start, m_goal, cMin)};
        };
};


int main()
{
    return 0;
};