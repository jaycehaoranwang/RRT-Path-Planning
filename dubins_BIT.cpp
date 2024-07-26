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
#include <random>
#include <chrono>

class BIT_Planner
{
    private:
        Node m_start;
        Node m_goal;
        node_ptr_t m_shared_start_ptr;
        node_ptr_t m_shared_goal_ptr;
        const std::vector<float>& m_discrete_headings {};  // Reference member variable
        Environment_Map& m_env_map;
        NodeKDTreeAdapter m_samples_kdAdapter;
        NodeKDTreeAdapter m_vertex_kdAdapter;
        std::unordered_set<node_ptr_t, NPHash, NPNodeEqual> m_map_samples {};
        std::unordered_set<node_ptr_t, NPHash, NPNodeEqual> m_explored_vertices {};
        std::unordered_set<node_ptr_t, NPHash, NPNodeEqual> m_explored_vertices_old {};
        std::unordered_set<node_ptr_pair_t, PairNPHash, PairNPEqual> m_explored_edges {};
        std::priority_queue<EdgeCost_t, std::vector<EdgeCost_t>, CompareQueuePairCosts> m_edge_queue {};
        std::priority_queue<VertexCost_t, std::vector<VertexCost_t>, CompareQueuePairCosts> m_vertex_queue {};
        std::unordered_map<node_ptr_t, float, NPHash, NPNodeEqual> m_cost_to_node_dict {};
        bool m_solution_existence {false};
        const float m_search_radius {};
        constexpr int m_max_kdtree_leafs = 10;
        const float m_cMin {};
        const float m_theta {};
        const float m_min_turning_radius {};
        const int m_max_iters {};
        const int m_batch_sample_count {};
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

        void Prune(const float best_cost)
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
            for (auto it = m_explored_vertices.begin(); it != m_explored_vertices.end();) {
                if (f_estimated(**it) <= best_cost) {
                    ++it; // Move to the next element
                } else {
                    it = m_explored_vertices.erase(it); // Erase returns the next iterator
                }
            }
            // Prune edges
            for (auto it = m_explored_edges.begin(); it != m_explored_edges.end();) {
                // Edges are stored as a pair of pointers to nodes and f_estimated takes Nodes as input
                if (f_estimated(*(it->first)) <= best_cost && f_estimated(*(it->second)) <= best_cost) {
                    ++it; // Move to the next element
                } else {
                    it = m_explored_edges.erase(it); // Erase returns the next iterator
                }
            }

            std::unordered_set<node_ptr_t> temp_vertexs {};
            for (auto it = m_explored_vertices.begin(); it != m_explored_vertices.end();) {
                if (m_cost_to_node_dict[*it] == inf) {
                    temp_vertexs.insert(*it)
                } else {
                    ++it; // Move to the next element
                }
            }

            m_map_samples.merge(temp_vertexs);

            // Reprune the vertex set
            for (auto it = m_explored_vertices.begin(); it != m_explored_vertices.end();) {
                if (m_cost_to_node_dict[*it] < inf) {
                    ++it; // Move to the next element
                } else {
                    it = m_explored_vertices.erase(it); // Erase returns the next iterator
                }
            }

        }

        float true_cost(const node_ptr_t& start_n, const node_ptr_t& end_n)
        {
            // Need to implement
            if (m_env_map.path_collision(start_n, end_n))
            {
                return inf;
            } else 
            {
                return calc_dist_and_angle(*start_n, *end_n).first;
            }
        }

        void ExpandVertex(const node_ptr_t& vertex_node, KDTree_t& samples_kdtree, KDTree_t& vertex_kdtree, const std::vector<node_ptr_t>& samples_vec, const std::vector<node_ptr_t>& vertex_vec)
        {
            // Remove vertex from vertex priority queue, but if using priority queue vertex should be removed from queue already when retrieved
            
            float g_estimated_vertex = g_estimated(*vertex_node);
            //Find nodes near arg vertex_node
            std::vector<nanoflann::ResultItem<float, float>> ret_matches_samples;
            const float query_pt[2] = {vertex_node->get_x(),vertex_node->get_y()}
            const size_t nMatches_samples = samples_kdtree.radiusSearch(&query_pt[0], m_search_radius, ret_matches_samples);
            std::unordered_set<node_ptr_t, NPHash, NPNodeEqual> samples_near;
            for (size_t i = 0; i < nMatches_samples; i++)
            {
                samples_near.insert(samples_vec[ret_matches_samples[i].first]);
            }

            for (auto it = samples_near.begin(); it != samples_near.end();) 
            {
                float edge_distance = calc_dist_and_angle(*vertex_node,**it).first
                float h_estimated_cost = h_estimated(**it);
                if (g_estimated_vertex + edge_distance + h_estimated_cost < m_cost_to_node_dict[m_shared_goal_ptr]) 
                {
                    m_cost_to_node_dict[*it] = inf;
                    // Compute edge cost
                    EdgeCost_t edge(std::make_pair(vertex_node, *it), m_cost_to_node_dict[vertex_node] + edge_distance + h_estimated_cost);
                    m_edge_queue.push(edge);
                }
                ++it; // Move to the next element
            }

            // Use count to check if item is in the unordered_set 1 if exists, 0 else
            if (m_explored_vertices_old.count(vertex_node) == 0)
            {
                // find vertexes near arg vertex within radius
                std::unordered_set<node_ptr_t, NPHash, NPNodeEqual> vertexes_near;
                std::vector<nanoflann::ResultItem<float, float>> ret_matches_v;
                const size_t nMatches_v = vertex_kdtree.radiusSearch(&query_pt[0], m_search_radius, ret_matches_v);
                for (size_t i = 0; i < nMatches_v; i++)
                {
                    vertexes_near.insert(vertex_vec[ret_matches_v[i].first]);
                }

                //Iterate through near vertices
                for (auto it = vertexes_near.begin(); it != vertexes_near.end();) 
                {
                    node_ptr_pair_t edge_pair = std::make_pair(vertex_node,*it);
                    float edge_cost = calc_dist_and_angle(*vertex_node,**it).first;

                    if  ((m_explored_edges.count(edge_pair) == 0) && 
                        (g_estimated_vertex + edge_cost + h_estimated(**it) < m_cost_to_node_dict[m_shared_goal_ptr]) &&
                        (m_cost_to_node_dict[vertex_node] + edge_cost < m_cost_to_node_dict[*it]))
                    {
                        EdgeCost_t edge(std::make_pair(vertex_node, *it), m_cost_to_node_dict[vertex_node] + edge_cost + h_estimated(**it));
                        m_edge_queue.push(edge);
                        if (m_cost_to_node_dict.count(*it) == 0)
                        {
                            m_cost_to_node_dict[*it] = inf;
                        }
                    }
                    ++it;
                }
            }
        }


        KDTree_t BuildSamplesKdtree()
        {
            m_samples_kdAdapter.nodes = m_map_samples;
            KDTree_t samples_kdtree(2 /* dimensions */, m_samples_kdAdapter, {m_max_kdtree_leafs /* Leaf nodes*/});
            samples_kdtree.buildIndex();
            return samples_kdtree
        }

        KDTree_t BuildVertexKdtree()
        {
            m_vertex_kdAdapter.nodes = m_explored_vertices;
            KDTree_t vertex_kdtree(2 /* dimensions */, m_vertex_kdAdapter, {m_max_kdtree_leafs /* Leaf nodes*/});
            vertex_kdtree.buildIndex();
            return vertex_kdtree
        }

        float BestVertexQueueValue() const
        {
            if (m_vertex_queue.empty())
            {
                return inf;
            } else 
            {
                return m_vertex_queue.top().second; //retrieve from the min heap/priority q
            }
        }

        float BestEdgeQueueValue() const
        {
            if (m_edge_queue.empty())
            {
                return inf;
            } else 
            {
                return m_edge_queue.top().second; //retrieve from the min heap/priority q
            }
        }

        node_ptr_t BestInVertexQueue() const
        {

            assert(!m_vertex_queue.empty() && "Error: Vertex Priority Queue should NOT be empty!");
            return m_vertex_queue.top().first; //retrieve from the min heap/priority q
            
        }

        node_ptr_pair_t BestInEdgeQueue() const
        {
            assert(!m_edge_queue.empty() && "Error: Edge Priority Queue should NOT be empty!");
            return m_edge_queue.top().first;
        }

        Eigen::Vector3f SampleUnitNBall() const
        {
            // Create a random number generator
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<float> dis(0.0, 1.0);
            std::uniform_real_distribution<float> angle_dis(0.0, 2 * PI);

            // Generate random angle theta
            float theta = angle_dis(gen);
            
            // Generate random radius r using the square root of a uniformly distributed random number
            float r = std::sqrt(dis(gen));

            // Convert polar coordinates to Cartesian coordinates
            float x = r * std::cos(theta);
            float y = r * std::sin(theta);

            // Return the 3D vector with the z-coordinate set to 0.0
            return Eigen::Vector3f(x, y, 0.0f);
        }

        void SampleEllipsoid(const float cMax)
        {
            Eigen::Vector3f r(cMax / 2.0f, std::sqrt(cMax*cMax - m_cMin*m_cMin)/2.0f, std::sqrt(cMax*cMax - m_cMin*m_cMin)/2.0f);
            Eigen::Matrix3f L = Eigen::Matrix3d::Zero();
            L.diagonal() = r;

            int sample_count = 0;
            while (sample_count < m_batch_sample_count)
            {
                Eigen::Vector3f xBall{SampleUnitNBall()};
                Eigen::Vector3f random_sample = ((m_C * L) * xBall) + m_ellipse_center;
                node_ptr_t new_node = std::make_shared<Node>(random_sample[0], random_sample[1]);
                if (!m_env_map.point_collision(new_node))
                {
                    m_map_samples.insert(new_node);
                    sample_count++;
                }
            }
        }

        void SampleFreeSpace()
        {
            int sample_count = 0;
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<float> map_x_range(m_env_map.get_xRange().first, m_env_map.get_xRange().second);
            std::uniform_real_distribution<float> map_y_range(m_env_map.get_yRange().first, m_env_map.get_yRange().second);
            while (sample_count < m_batch_sample_count)
            {
                node_ptr_t new_node = std::make_shared<Node>(map_x_range(gen), map_y_range(gen));
                if (!m_env_map.point_collision(new_node))
                {
                    m_map_samples.insert(new_node);
                    sample_count++;
                }
            }
        }

        void SamplePoints(const float cMax)
        {
            if (cMax < inf)
            {
                SampleEllipsoid(cMax);
            } else
            {
                SampleFreeSpace();
            }
        }

        void empty_queues()
        {
            while (!m_edge_queue.empty()) 
            {
                m_edge_queue.pop();
            }

            while (!m_vertex_queue.empty()) 
            {
                m_vertex_queue.pop();
            }
        }

    public:
        BIT_Dubins_Planner(Node& start, Node& goal, const std::vector<float>& discrete_headings, Environment_Map& env_map, float search_radius, 
                            const int max_iters, const float min_turning_radius, const int batch_sample_count)
            : m_start {start}
            , m_goal {goal}
            , m_discrete_headings {discrete_headings}
            , m_env_map {env_map}
            , m_min_turning_radius {min_turning_radius}        
            , m_search_radius {search_radius}
            , m_cMin {calc_dist_and_angle(m_start, m_goal).first}
            , m_theta {calc_dist_and_angle(m_start, m_goal).second}
            , m_max_iters {max_iters}
            , m_batch_sample_count {batch_sample_count}
            , m_C {RotationToWorldFrame(m_start, m_goal, cMin)}
            , m_ellipse_center {Eigen::Vector3f((m_start.get_x() + m_goal.get_x()) / 2.0f, (m_start.get_y() + m_goal.get_y()) / 2.0f, 0.0f)}
        {
            m_shared_start_ptr = std::make_shared<Node>(m_start);
            m_shared_goal_ptr = std::make_shared<Node>(m_goal);

            m_map_samples.insert(m_shared_goal_ptr);
            m_explored_vertices.insert(m_shared_start_ptr);
            m_cost_to_node_dict[m_shared_start_ptr] = 0.0f;
            m_cost_to_node_dict[m_shared_goal_ptr] = inf;
        }

        std::pair<std::vector<std::pair<float, float>>,float> extract_best_path()
        {
            float path_cost = 0.0f;
            assert(m_solution_existence && "Error: Solution extraction CANNOT be called if no solution exist");
            std::vector<std::pair<float, float>> solution_path;
            solution_path.push_back(std::make_pair(m_goal.get_x(), m_goal.get_y()));
            node_ptr_t curr_node = m_shared_goal_ptr;
            while (curr_node->get_parent() != nullptr)
            {
                path_cost = path_cost + calc_dist_and_angle(*curr_node,*(curr_node->get_parent())).first;
                curr_node = curr_node->get_parent();
                solution_path.push_back(std::make_pair(curr_node.get_x(), curr_node.get_y()));
            }
            return std::make_pair(solution_path,path_cost);
        }

        std::vector<std::pair> plan()
        {   
            // Main search/planning loop, modify as required during implementation in car
            auto start = std::chrono::high_resolution_clock::now();
            for (int i=0; i<m_max_iters; i++){
                bool no_solution = false;
                if (m_vertex_queue.empty() && m_edge_queue.empty())
                {
                    // If first batch, double the number of sampled points
                    int points_to_sample = m_batch_sample_count;
                    if (i==0)
                    {
                        points_to_sample = points_to_sample*2;
                    }

                    Prune(m_cost_to_node_dict[m_shared_goal_ptr]);
                    SamplePoints(m_cost_to_node_dict[m_shared_goal_ptr]);
                    m_explored_vertices_old = m_explored_vertices;
                    // Iterate through vertex set and add them to priority q
                    for (auto it = m_explored_vertices.begin(); it != m_explored_vertices.end();) 
                    {
                        // Check if the node exist in the map/cost dictionary
                        try 
                        {
                            // Accessing a possibly non-existing key using at()
                            float vertex_value = m_cost_to_node_dict.at(*it);
                            m_vertex_queue.push(std::make_pair(*it,vertex_value+h_estimated(**it)));
                        } catch (const std::out_of_range& e) //.at will throw out of range error if key doesnt exist in map
                        {
                            m_cost_to_node_dict[*it] = inf; 
                            m_vertex_queue.push(std::make_pair(*it,inf));
                        }
                    }
                }

                // Before ExpandVertex loop cbuild the kd trees and make a vector c
                std::vector<node_ptr_t> samples_vec(m_map_samples.begin(), m_map_samples.end());
                std::vector<node_ptr_t> vertex_vec(m_explored_vertices.begin(), m_explored_vertices.end());
                KDTree_t samples_tree = build_samples_kdtree();
                KDTree_t vertex_tree = build_vertex_kdtree();

                while (BestVertexQueueValue() <= BestEdgeQueueValue())
                {
                    if (m_vertex_queue.empty())
                    {
                        no_solution = true;
                        break;
                    }
                    ExpandVertex(BestInVertexQueue(), samples_tree, vertex_tree, samples_vec, vertex_vec);
                }

                if (no_solution)
                {
                    // No solution for this batch, empty queues and move to next batch
                    empty_queues();
                    std::cout << "No solution found, going to next batch" << std::endl;
                    continue;
                }

                node_ptr_pair_t best_edge = BestInEdgeQueue();
                m_edge_queue.pop(); //remove the above edge from the queue
                float estimated_cost_to_goal = h_estimated(*(best_edge.second);
                if (m_cost_to_node_dict[best_edge.first] + calc_dist_and_angle(*(best_edge.first), *(best_edge.second)).first + estimated_cost_to_goal) <  m_cost_to_node_dict[m_shared_goal_ptr])
                {
                    const float actual_edge_cost = true_cost(best_edge.first, best_edge.second); // NEED TO BE IMPLEMENTED WITH FULL COLLISION CHECKING/DUBINS
                    if (g_estimated(*(best_edge.first)) + actual_edge_cost + estimated_cost_to_goal < m_cost_to_node_dict[m_shared_goal_ptr])
                    {
                        if (m_cost_to_node_dict[best_edge.first] + actual_edge_cost < m_cost_to_node_dict[best_edge.second])
                        {
                            if (m_explored_vertices.find(best_edge.second) != m_explored_vertices.end()) // if second vertex in edge is in vertex set
                            {
                                // remove edges that contain this vertex
                                for (auto it = m_explored_edges.begin(); it != m_explored_edges.end();) 
                                {
                                    // Edges are stored as a pair of pointers to nodes
                                    const auto& pair = *it;
                                    if (pair.second == best_edge.second) 
                                    {
                                        it = m_explored_edges.erase(it); // Erase returns the next iterator
                                    } else {
                                        ++it; // Move to the next element
                                    }
                                }

                            } else 
                            {
                                m_map_samples.erase(best_edge.second);
                                m_explored_vertices.insert(best_edge.second);
                                try 
                                {
                                    // Accessing a possibly non-existing key using at()
                                    float vertex_value = m_cost_to_node_dict.at(best_edge.second);
                                    m_vertex_queue.push(std::make_pair(best_edge.second,vertex_value+h_estimated(*(best_edge.second))));
                                } catch (const std::out_of_range& e) //.at will throw out of range error if key doesnt exist in map
                                {
                                    m_cost_to_node_dict[best_edge.second] = inf; 
                                    m_vertex_queue.push(std::make_pair(best_edge.second,inf));
                                }
                            }


                            m_cost_to_node_dict[best_edge.second] = m_cost_to_node_dict[best_edge.first] + actual_edge_cost;
                            m_explored_edges.insert(best_edge);
                            best_edge.second->set_parent(best_edge.first);
                        
                            //Copy over to a vector bc priority queue cannot be iterated through
                            std::vector<EdgeCost_t> edge_Q_elements;
                            // Extract elements from the temporary queue and store them in the vector
                            while (!m_edge_queue.empty()) 
                            {
                                edge_Q_elements.push_back(m_edge_queue.top());
                                m_edge_queue.pop();
                            }

                            for (auto it = edge_Q_elements.begin(); it != edge_Q_elements.end();) 
                            {
                                const EdgeCost_t& edge_pair = *it;
                                if ((edge_pair.first.second == best_edge.second) && 
                                (m_cost_to_node_dict[edge_pair.first.first] + calc_dist_and_angle(*(edge_pair.first.first),*(best_edge.second)).first >= m_cost_to_node_dict[best_edge.second]))
                                {
                                    it = edge_Q_elements.erase(it);
                                } else 
                                {
                                    m_edge_queue.push(edge_pair)
                                    ++it;
                                }
                            }
                        }
                    }

                } else 
                {
                    empty_queues();
                    if (!m_solution_existence)
                    {
                        m_solution_existence = true;
                    }
                    auto end = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double> elapsed = end - start;
                    std::cout << "Batch Complete, Path Found With Cost: " << extract_best_path().second <<std::endl;
                    std::cout << "Time from Start/Last Solution (s): " << elapsed.count() << std::endl;
                    auto start = std::chrono::high_resolution_clock::now();
                }
            }
        }
};


int main()
{   
    /*
    BIT_Dubins_Planner(Node& start, Node& goal, const std::vector<float>& discrete_headings, Environment_Map& env_map, float search_radius, 
                            const int max_iters, const float min_turning_radius, const int batch_sample_count)
                            */
    std::vector<float> dubins_headings = {0.0f};
    std::array<float,4> obs_one = {10.0f, 10.0f, 15.0f, 15.0f};
    std::array<float,4> obs_two = {20.0f, 20.0f, 25.0f, 25.0f};
    std::array<float,4> obs_three = {25.0f, 15.0f, 30.0f, 20.0f};
    Environment_Map map(std::make_pair(0.0f, 50.0f), std::make_pair(0.0f, 30.0f));
    map.add_obstacles(obs_one);
    map.add_obstacles(obs_two);
    map.add_obstacles(obs_three);
    Node start_node(2.0f, 15.0f);
    Node goal_node(45.0f, 15.0f);
    float search_r = 10.0f;
    const int max_iters = 10000;
    const float min_turning_radius = 0.0f;
    const int batch_sample_count = 100;
    BIT_Planner bit_planner(start_node, goal_node, dubins_headings, map, search_r, max_iters, min_turning_radius, batch_sample_count);
    bit_planner.plan();
    return 0;
};