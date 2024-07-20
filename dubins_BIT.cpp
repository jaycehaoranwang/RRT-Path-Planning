#include "dubins_BIT.hpp"
#include "nanoflann.hpp"


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


class BIT_Dubins_Planner
{
    private:
        Node m_start;
        Node m_goal;
        const std::vector<float>& m_discrete_headings {};  // Reference member variable
        Environment_Map& m_env_map;
        NodeKDTreeAdapter m_samples_kdAdapter;
        NodeKDTreeAdapter m_vertex_kdAdapter;
        std::unordered_set<node_ptr_t, NPHash, NPNodeEqual> m_map_samples {};
        std::unordered_set<node_ptr_t, NPHash, NPNodeEqual> m_vertex_set {};
        std::unordered_set<node_ptr_t, NPHash, NPNodeEqual> m_vertex_set_old {};
        std::unordered_set<node_ptr_pair_t, PairNPHash, PairNPEqual> m_edge_set {};
        std::priority_queue<EdgeCost_t, std::vector<EdgeCost_t>, CompareQueuePairCosts> m_edge_queue {};
        std::priority_queue<VertexCost_t, std::vector<VertexCost_t>, CompareQueuePairCosts> m_vertex_queue {};
        std::unordered_map<node_ptr_t, float, NPHash, NPNodeEqual> m_cost_to_node_table {};
        
        const float m_search_radius {};
        constexpr int m_max_kdtree_leafs = 10;
        const float m_cMin {};
        const float m_theta {};
        const float m_min_turning_radius {};
        const int m_max_iters {};
        const Eigen::Matrix3f m_C {};
        const Eigen::Vector3f m_ellipse_center {};

        std::pair<float, float> calc_dist_and_angle(const Node& start_node, const Node& end_node) const
        {
            float dx {end_node.get_x() - start_node.get_x()};
            float dy {end_node.get_y() - start_node.get_y()};
            return std::make_pair(static_cast<float>(std::hypot(dx, dy)), static_cast<float>(std::atan2(dy, dx)));
        }

        // Function to perform rotation to world frame
        Eigen::Matrix3f RotationToWorldFrame(const Node& x_start, const Node& x_goal, float L) const 
        {
            Eigen::Vector3f a1((x_goal.get_x() - x_start.get_x()) / L, (x_goal.get_y() - x_start.get_y()) / L, 0.0f);
            Eigen::Vector3f e1(1.0f, 0.0f, 0.0f);
            Eigen::Matrix3f M = a1 * e1.transpose();
            Eigen::JacobiSVD<Eigen::Matrix3f> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3f U = svd.matrixU();
            Eigen::Matrix3f V_T = svd.matrixV().transpose();
            float det_UV_T = U.determinant() * V_T.determinant();
            Eigen::Matrix3f C = U * Eigen::DiagonalMatrix<float, 3>(1.0f, 1.0f, det_UV_T) * V_T;
            return C;
        }
        
        float g_estimated(const Node& node) const
        {
            return calc_dist_and_angle(m_start, node).first;
        }

        float h_estimated(const Node& node) const
        {
            return calc_dist_and_angle(node, m_goal).first;
        }

        float f_estimated(const Node& node) const 
        {
            return g_estimated(node) + h_estimated(node);
        }

        void prune(const float best_cost)
        {
            // Prune samples in map_samples
            for (auto it = m_map_samples.begin(); it != m_map_samples.end();) {
                if (f_estimated(**it) < best_cost) {
                    ++it; // Move to the next element
                } else {
                    it = m_map_samples.erase(it); // Erase returns the next iterator
                }
            }
            // Prune samples in vertex set
            for (auto it = m_vertex_set.begin(); it != m_vertex_set.end();) {
                if (f_estimated(**it) <= best_cost) {
                    ++it; // Move to the next element
                } else {
                    it = m_vertex_set.erase(it); // Erase returns the next iterator
                }
            }
            // Prune edges
            for (auto it = m_edge_set.begin(); it != m_edge_set.end();) {
                // Edges are stored as a pair of pointers to nodes and f_estimated takes Nodes as input
                if (f_estimated(*(it->first)) <= best_cost && f_estimated(*(it->second)) <= best_cost) {
                    ++it; // Move to the next element
                } else {
                    it = m_edge_set.erase(it); // Erase returns the next iterator
                }
            }

            std::unordered_set<node_ptr_t> temp_vertexs {};
            for (auto it = m_vertex_set.begin(); it != m_vertex_set.end();) {
                if (m_cost_to_node_table(*it) == std::numeric_limits<float>::infinity()) {
                    temp_vertexs.insert(*it)
                } else {
                    ++it; // Move to the next element
                }
            }

            m_map_samples.merge(temp_vertexs);

            // Reprune the vertex set
            for (auto it = m_vertex_set.begin(); it != m_vertex_set.end();) {
                if (m_cost_to_node_table(*it) < std::numeric_limits<float>::infinity()) {
                    ++it; // Move to the next element
                } else {
                    it = m_vertex_set.erase(it); // Erase returns the next iterator
                }
            }

        }

        void expand_vertex(const node_ptr_t& vertex_node, KDTree_t& samples_kdtree, KDTree_t& vertex_kdtree)
        {
            // Remove vertex from vertex priority queue, but if using priority queue vertex should be removed from queue already when retrieved

            //Find nodes near arg vertex_node
            std::vector<nanoflann::ResultItem<float, float>> ret_matches;
            const float query_pt[2] = {vertex_node->get_x(),vertex_node->get_y()}
            const size_t nMatches = samples_kdtree.radiusSearch(&query_pt[0], m_search_radius, ret_matches);
            std::unordered_set<node_ptr_t, NPHash, NPNodeEqual> samples_near;
            
            // Prune samples in map_samples
            
        }

        KDTree_t build_samples_kdtree()
        {
            m_samples_kdAdapter.nodes = m_map_samples;
            KDTree_t samples_kdtree(2 /* dimensions */, m_samples_kdAdapter, {m_max_kdtree_leafs /* Leaf nodes*/});
            samples_kdtree.buildIndex();
            return samples_kdtree
        }

        KDTree_t build_vertex_kdtree()
        {
            m_vertex_kdAdapter.nodes = m_vertex_set;
            KDTree_t vertex_kdtree(2 /* dimensions */, m_vertex_kdAdapter, {m_max_kdtree_leafs /* Leaf nodes*/});
            vertex_kdtree.buildIndex();
            return vertex_kdtree
        }

    public:
        BIT_Dubins_Planner(Node& start, Node& goal, const std::vector<float>& discrete_headings, Environment_Map& env_map, float search_radius, const int max_iters, const float min_turning_radius)
            : m_start {start}
            , m_goal {goal}
            , m_discrete_headings {discrete_headings}
            , m_env_map {env_map}
            , m_min_turning_radius {min_turning_radius}
            , m_max_iters {max_iters}
            , m_search_radius {search_radius}
            , m_cMin {calc_dist_and_angle(m_start, m_goal).first}
            , m_theta {calc_dist_and_angle(m_start, m_goal).second}
            , m_C {RotationToWorldFrame(m_start, m_goal, cMin)}
            , m_ellipse_center {Eigen::Vector3f((m_start.get_x() + m_goal.get_x()) / 2.0f, (m_start.get_y() + m_goal.get_y()) / 2.0f, 0.0f)}
        {
            auto m_shared_start_ptr = std::make_shared<Node>(m_start);
            auto m_shared_goal_ptr = std::make_shared<Node>(m_goal);

            m_map_samples.insert(m_shared_goal_ptr);
            m_vertex_set.insert(m_shared_start_ptr);
            m_cost_to_node_table[m_shared_start_ptr] = 0.0f;
            m_cost_to_node_table[m_shared_goal_ptr] = std::numeric_limits<float>::infinity();

        }

        std::vector<std::pair> plan()
        {

            
        };
};


int main()
{
    return 0;
};