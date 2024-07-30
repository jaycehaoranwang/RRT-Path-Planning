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
#include "Eigen/Dense"
#include <random>
#include <chrono>
#include <set>
#include "nanoflann.hpp"
#include "dubins_BIT.hpp"
//#include "planning/utils/nanoflann.hpp"
//#include "planning/BIT_local_planner.hpp"
//#include "mapping/types/pose2.hpp"

class BIT_Planner
{
    public:
        BIT_Planner(BIT_Node& start, BIT_Node& goal, const std::vector<float>& discrete_headings, Environment_Map& env_map, float search_radius, const int max_iters, const float min_turning_radius, const int batch_sample_count)
            : m_start {start}
            , m_goal {goal}
            , m_shared_start_ptr {&m_start}
            , m_goal_ptr {&m_goal}
            //, m_discrete_headings {discrete_headings}
            , m_env_map {env_map}
            , m_min_turning_radius {min_turning_radius}        
            , m_search_radius {search_radius}
            , m_cMin {calc_dist_and_angle(m_start, m_goal).first}
            , m_theta {calc_dist_and_angle(m_start, m_goal).second}
            , m_max_iters {max_iters}
            , m_batch_sample_count {batch_sample_count}
            , m_C {RotationToWorldFrame(m_start, m_goal, m_cMin)}
            , m_ellipse_center {Eigen::Vector3f((m_start.get_x() + m_goal.get_x()) / 2.0f, (m_start.get_y() + m_goal.get_y()) / 2.0f, 0.0f)}
        {

            m_map_samples.insert(m_goal_ptr);
            m_explored_vertices.insert(m_shared_start_ptr);
            m_cost_to_node_dict[m_shared_start_ptr] = 0.0f;
            m_cost_to_node_dict[m_goal_ptr] = inf;

        };

        std::pair<std::vector<std::pair<float, float>>,float> extract_best_path()
        {
            float path_cost = 0.0f;
            assert(m_solution_existence && "Error: Solution extraction CANNOT be called if no solution exist");
            std::vector<std::pair<float, float>> solution_path;
            solution_path.push_back(std::make_pair(m_goal.get_x(), m_goal.get_y()));
            BIT_Node* curr_node = m_goal_ptr;
            while (curr_node->get_parent() != nullptr)
            {
                path_cost = path_cost + calc_dist_and_angle(*curr_node,*(curr_node->get_parent())).first;
                curr_node = curr_node->get_parent();
                solution_path.push_back(std::make_pair(curr_node->get_x(), curr_node->get_y()));
            }
            return std::make_pair(solution_path,path_cost);
        }

        void plan()
        {
            KDTree_t samples_kdtree(2 /* dimensions */, m_samples_kdAdapter, {m_max_kdtree_leafs /* Leaf nodes*/});
            KDTree_t vertex_kdtree(2 /* dimensions */, m_vertex_kdAdapter, {m_max_kdtree_leafs /* Leaf nodes*/});
            std::vector<BIT_Node*> samples_vec {};
            std::vector<BIT_Node*> vertex_vec {};
            bool no_solution = false;
            // Main search/planning loop, modify as required during implementation in car
            auto start = std::chrono::high_resolution_clock::now();
            for (int i=0; i<m_max_iters; i++)
            {
                no_solution = false;
                if (m_vertex_queue.empty() && m_edge_queue.empty())
                {

                    Prune(m_cost_to_node_dict[m_goal_ptr]);
                    SamplePoints(m_cost_to_node_dict[m_goal_ptr]);
                    m_explored_vertices_old = m_explored_vertices;
                    m_vertex_queue = m_explored_vertices;
                    
                }

                m_samples_kdAdapter.nodes = m_map_samples;
                samples_kdtree.buildIndex();
                samples_vec.clear();
                samples_vec.insert(samples_vec.end(),m_map_samples.begin(), m_map_samples.end());

                m_vertex_kdAdapter.nodes = m_explored_vertices;
                vertex_kdtree.buildIndex();
                vertex_vec.clear();
                vertex_vec.insert(vertex_vec.end(),m_explored_vertices.begin(), m_explored_vertices.end());

                std::pair<BIT_Node*, float> BestVertexQueueResult {BestVertexQueueValue()};
                std::pair<BIT_node_pair_t, float> BestEdgeQueueResult {BestEdgeQueueValue()};
                while (BestVertexQueueResult.second <= BestEdgeQueueResult.second)
                {
                    if (BestVertexQueueResult.first == nullptr)
                    {
                        std::cout<< "in plan: vertex queue is empty, breaking" << std::endl;
                        no_solution = true;
                        break;
                    } else 
                    {
                        ExpandVertex(BestVertexQueueResult.first, samples_kdtree, vertex_kdtree, samples_vec, vertex_vec);
                    }
                    BestVertexQueueResult = BestVertexQueueValue();
                    BestEdgeQueueResult = BestEdgeQueueValue();
                }

                if (no_solution)
                {
                    // No solution for this batch, empty queues and move to next batch
                    empty_queues();
                    std::cout << "No solution found, going to next batch" << std::endl;
                    continue;
                }
                BIT_node_pair_t best_edge = BestEdgeQueueResult.first;
                m_edge_queue.erase(BestEdgeQueueResult.first);

                // Remove the edge from queue
                float estimated_cost_to_goal {h_estimated(*(best_edge.second))};


                if (m_cost_to_node_dict[best_edge.first] + calc_dist_and_angle(*(best_edge.first), *(best_edge.second)).first + estimated_cost_to_goal < m_cost_to_node_dict[m_goal_ptr])
                {
                    const float actual_edge_cost = true_cost(best_edge.first, best_edge.second); // NEED TO BE IMPLEMENTED WITH FULL COLLISION CHECKING/DUBINS
                    if (g_estimated(*(best_edge.first)) + actual_edge_cost + estimated_cost_to_goal < m_cost_to_node_dict[m_goal_ptr])
                    {
                        if (m_cost_to_node_dict[best_edge.first] + actual_edge_cost < m_cost_to_node_dict[best_edge.second])
                        {
                            if (m_explored_vertices.find(best_edge.second) != m_explored_vertices.end()) // if second vertex in edge is in vertex set
                            {
                                // remove edges that contain this vertex
                                for (auto it = m_explored_edges.begin(); it != m_explored_edges.end();) 
                                {
                                    // Edges are stored as a pair of pointers to nodes
                                    if (it->second == best_edge.second) 
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
                                m_vertex_queue.insert(best_edge.second);
                            }

                            m_cost_to_node_dict[best_edge.second] = m_cost_to_node_dict[best_edge.first] + actual_edge_cost;
                            m_explored_edges.insert(best_edge);
                            best_edge.second->set_parent(best_edge.first);

                            for (auto it = m_edge_queue.begin(); it != m_edge_queue.end();) 
                            {
                                if (it->second == best_edge.second && m_cost_to_node_dict[it->first] + calc_dist_and_angle(*(it->first), *(best_edge.second)).first >= m_cost_to_node_dict[best_edge.second]) 
                                {
                                    it = m_edge_queue.erase(it);
                                } else 
                                {
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
                    std::pair<std::vector<std::pair<float, float>>,float> solution_result {extract_best_path()};
                    std::cout << "Batch Complete, Path Found With Cost: " << solution_result.second <<std::endl;
                    std::cout << "Batch Done Time from Start/Last Solution (s): " << elapsed.count() << std::endl;
                    auto start = std::chrono::high_resolution_clock::now();
                }
            }
            std::cout << "Planning DONE" << std::endl;
        }

    private:
        BIT_Node m_start;
        BIT_Node m_goal;
        BIT_Node* const m_shared_start_ptr;
        BIT_Node* const m_goal_ptr;
        //const std::vector<float>& m_discrete_headings {};  // only for dubins
        Environment_Map& m_env_map;
        NodeKDTreeAdapter m_samples_kdAdapter;
        NodeKDTreeAdapter m_vertex_kdAdapter;
        std::unordered_set<BIT_Node*, NPHash> m_map_samples {};
        std::unordered_set<BIT_Node*, NPHash> m_explored_vertices {};
        std::unordered_set<BIT_Node*, NPHash> m_explored_vertices_old {};
        std::unordered_set<std::pair<BIT_Node*,BIT_Node*>, PairNPHash, PairNPEqual> m_explored_edges {};
        //std::priority_queue<EdgeCost_t, std::vector<EdgeCost_t>, CompareQueuePairCosts> m_edge_queue {};
        //std::priority_queue<VertexCost_t, std::vector<VertexCost_t>, CompareQueuePairCosts> m_vertex_queue {};
        std::unordered_set<std::pair<BIT_Node*,BIT_Node*>, PairNPHash, PairNPEqual> m_edge_queue {};
        std::unordered_set<BIT_Node*, NPHash> m_vertex_queue {};
        std::unordered_map<BIT_Node*, float, NPHash> m_cost_to_node_dict {};
        bool m_solution_existence {false};
        const float m_search_radius {};
        const size_t m_max_kdtree_leafs = 10;
        const float m_cMin {};
        const float m_theta {};
        const float m_min_turning_radius {};
        const int m_max_iters {};
        const int m_batch_sample_count {};
        const Eigen::Matrix3f m_C {};
        const Eigen::Vector3f m_ellipse_center {};

        std::pair<float, float> calc_dist_and_angle(const BIT_Node& start_node, const BIT_Node& end_node) const
        {
            float dx {end_node.get_x() - start_node.get_x()};
            float dy {end_node.get_y() - start_node.get_y()};
            return std::make_pair(static_cast<float>(std::hypot(dx, dy)), static_cast<float>(std::atan2(dy, dx)));
        }

        // Function to perform rotation to world frame
        Eigen::Matrix3f RotationToWorldFrame(const BIT_Node& x_start, const BIT_Node& x_goal, float L) const 
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
        
        float g_estimated(const BIT_Node& node) const
        {
            return calc_dist_and_angle(m_start, node).first;
        }

        float h_estimated(const BIT_Node& node) const
        {
            return calc_dist_and_angle(node, m_goal).first;
        }

        float f_estimated(const BIT_Node& node) const 
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
                    delete *it;
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
            for (auto it = m_explored_edges.begin(); it != m_explored_edges.end();) 
            {
                // Edges are stored as a pair of pointers to nodes and f_estimated takes Nodes as input
                if (f_estimated(*(it->first)) <= best_cost && f_estimated(*(it->second)) <= best_cost) 
                {
                    ++it; // Move to the next element
                } else {
                    it = m_explored_edges.erase(it); // Erase returns the next iterator
                }
            }

            for (auto it = m_explored_vertices.begin(); it != m_explored_vertices.end(); ++it) 
            {
                if (m_cost_to_node_dict[*it] == inf) 
                {
                    m_map_samples.insert(*it);
                } 
            }

            // Reprune the vertex set
            for (auto it = m_explored_vertices.begin(); it != m_explored_vertices.end();) 
            {
                if (m_cost_to_node_dict[*it] < inf) {
                    ++it; // Move to the next element
                } else {
                    it = m_explored_vertices.erase(it); // Erase returns the next iterator
                }
            }

        }

        float true_cost(const BIT_Node* start_n, const BIT_Node* end_n)
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


        void ExpandVertex(BIT_Node* vertex_node, KDTree_t& samples_kdtree, KDTree_t& vertex_kdtree, const std::vector<BIT_Node*>& samples_vec, const std::vector<BIT_Node*>& vertex_vec)
        {
            int erased = m_vertex_queue.erase(vertex_node);
            assert(erased==1);
            
            // Remove vertex from vertex priority queue, the top item popped SHOULD be the vertex node
            float g_estimated_vertex = g_estimated(*vertex_node);
            //Find nodes near arg vertex_node
            std::vector<nanoflann::ResultItem<float, float>> ret_matches_samples;
            const float query_pt[2] = {vertex_node->get_x(),vertex_node->get_y()};
            const size_t nMatches_samples = samples_kdtree.radiusSearch(&query_pt[0], m_search_radius, ret_matches_samples);
            std::vector<BIT_Node*> samples_near;
            for (size_t i = 0; i < nMatches_samples; i++)
            {
                samples_near.push_back(samples_vec[ret_matches_samples[i].first]);
            }

            for (const auto & sample_ptr : samples_near) 
            {
                float edge_distance = calc_dist_and_angle(*vertex_node,*sample_ptr).first;
                float h_estimated_cost = h_estimated(*sample_ptr);
                if (g_estimated_vertex + edge_distance + h_estimated_cost < m_cost_to_node_dict[m_goal_ptr]) 
                {
                    m_cost_to_node_dict[sample_ptr] = inf;
                    // Compute edge cost
                    m_edge_queue.insert(std::make_pair(vertex_node, sample_ptr));
                }
            }

            // Use count to check if item is in the unordered_set 1 if exists, 0 else
            if (m_explored_vertices_old.count(vertex_node) == 0)
            {
                // find vertexes near arg vertex within radius
                std::vector<BIT_Node*> vertexes_near;
                std::vector<nanoflann::ResultItem<float, float>> ret_matches_v;
                const size_t nMatches_v = vertex_kdtree.radiusSearch(&query_pt[0], m_search_radius, ret_matches_v);
                for (size_t i = 0; i < nMatches_v; i++)
                {
                    vertexes_near.push_back(vertex_vec[ret_matches_v[i].first]);
                }

                //Iterate through near vertices
                for (const auto & w : vertexes_near) 
                {
                    BIT_node_pair_t edge_pair = std::make_pair(vertex_node,w);
                    float edge_cost = calc_dist_and_angle(*vertex_node,*w).first;
                    if  ((m_explored_edges.count(edge_pair) == 0) && 
                        (g_estimated_vertex + edge_cost + h_estimated(*w) < m_cost_to_node_dict[m_goal_ptr]) &&
                        (m_cost_to_node_dict[vertex_node] + edge_cost < m_cost_to_node_dict[w]))
                    {

                        m_edge_queue.insert(std::make_pair(vertex_node, w));
                        if (m_cost_to_node_dict.count(w) == 0)
                        {
                            m_cost_to_node_dict[w] = inf;
                        }
                    }
                }
            }
        }

        std::pair<BIT_Node*,float> BestVertexQueueValue()
        {
            if (m_vertex_queue.empty())
            {
                return std::make_pair(nullptr,inf);
            } else 
            {
                float min_v_value = inf;
                // Iterate through the set and return the min value
                float vertex_value {inf};
                BIT_Node * best_vertex_ptr {};
                for (const auto & vertex : m_vertex_queue) 
                {
                    
                    if (m_cost_to_node_dict.count(vertex) == 1)
                    {
                        vertex_value = m_cost_to_node_dict[vertex] + h_estimated(*vertex);
                        if (vertex_value < min_v_value)
                        {
                            min_v_value = vertex_value;
                            best_vertex_ptr = vertex;
                        }
                    } else
                    {
                        assert(m_cost_to_node_dict.count(vertex) == 0 && "ERROR IN BESTVERTEXQUEUE: vertex should NOT be a key to more than 1 element");
                    }
                }
                return std::make_pair(best_vertex_ptr,min_v_value); //retrieve from the min heap/priority q
            }
        }

        std::pair<BIT_node_pair_t,float> BestEdgeQueueValue()
        {
            if (m_edge_queue.empty())
            {
                return std::make_pair(std::make_pair(nullptr,nullptr),inf);
            } else 
            {
                float min_edge_value = inf;
                float edge_value {inf};
                BIT_node_pair_t best_edge_ptr;
                for (const auto & edge : m_edge_queue) 
                {
                    if (m_cost_to_node_dict.count(edge.first) == 1)
                    {
                        edge_value =  m_cost_to_node_dict[edge.first] + calc_dist_and_angle(*(edge.first), *(edge.second)).first + h_estimated(*(edge.second));

                        if (edge_value < min_edge_value)
                        {
                            min_edge_value = edge_value;
                            best_edge_ptr = std::make_pair(edge.first,edge.second);
                        }
                    } else
                    {
                        assert(m_cost_to_node_dict.count(edge.first) == 0 && "ERROR IN BESTEDGEQUEUE: edge vertex initial should NOT be a key to more than 1 element");
                    }
                    
                }
                return std::make_pair(best_edge_ptr,min_edge_value); //retrieve from the min heap/priority q
            }
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
            Eigen::Matrix3f L = Eigen::Matrix3d::Zero().cast<float>();
            L.diagonal() = r;

            int sample_count = 0;
            while (sample_count < m_batch_sample_count)
            {
                Eigen::Vector3f xBall{SampleUnitNBall()};
                Eigen::Vector3f random_sample = ((m_C * L) * xBall) + m_ellipse_center;
                BIT_Node* const new_node = new BIT_Node(random_sample[0], random_sample[1]);
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
                BIT_Node* new_node = new BIT_Node(map_x_range(gen), map_y_range(gen));
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
            m_edge_queue.clear();
            m_vertex_queue.clear();
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
    BIT_Node start_node(2.0f, 15.0f);
    BIT_Node goal_node(45.0f, 15.0f);
    float search_r = 100.0f;
    const int max_iters = 10000;
    const float min_turning_radius = 0.0f;
    const int batch_sample_count = 100;
    BIT_Planner bit_planner(start_node, goal_node, dubins_headings, map, search_r, max_iters, min_turning_radius, batch_sample_count);
    bit_planner.plan();
    return 0;
};
