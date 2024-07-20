#include "nanoflann.hpp"
#include "test_nanoflann.hpp"
#include <iostream>
#include <memory>
#include <random>
#include <format>

typedef std::shared_ptr<Node> node_ptr_t;

const float calc_squared_dist(const Node& start_node, const Node& end_node)
{
            float dx {end_node.get_x() - start_node.get_x()};
            float dy {end_node.get_y() - start_node.get_y()};
            return dx*dx + dy*dy;
}

int main()
{
    std::cout << "Testing Node object and Shared Node Pointer set manipulation/properties" << std::endl;
    std::unordered_set<node_ptr_t, NPHash, NPNodeEqual> m_map_samples {};
    int num_nodes {10};

    std::random_device rd;
    std::mt19937 gen(rd());
    // Define the range for the random numbers
    std::uniform_real_distribution<float> dis(0.0f, 50.0f);
    std::uniform_real_distribution<float> dis_heading(0.0f, 360.0f);

    std::shared_ptr<Node> node_ptr = std::make_shared<Node>(0.0f, 0.0f);
    node_ptr->set_heading(0.0f);
    m_map_samples.insert(node_ptr);
    std::string formattedString = std::format("X: {} , Y: {} , Heading: {}", 0.0f, 0.0f, 0.0f);
    std::cout << formattedString << std::endl;

    for (int i = 0; i < num_nodes; i++){
        float x {dis(gen)};
        float y {dis(gen)};
        float heading {dis_heading(gen)};
        std::shared_ptr<Node> node_ptr = std::make_shared<Node>(x, y);
        node_ptr->set_heading(heading);
        m_map_samples.insert(node_ptr);
        std::string formattedString = std::format("X: {} , Y: {} , Heading: {}", x, y, heading);
        std::cout << formattedString << std::endl;
    }

    std::shared_ptr<Node> node_ptr2 = std::make_shared<Node>(0.0f, 0.0f);
    node_ptr2->set_heading(0.0f);
    m_map_samples.insert(node_ptr2);

    std::string string2 = std::format("Size of set: {}, expected value should be {}",m_map_samples.size(),num_nodes+1);
    std::cout <<  string2 << std::endl;

    std::shared_ptr<Node> query_node = std::make_shared<Node>(30.0f, 30.0f);   
    std::cout << "Iterating through set and retrieving properties" << std::endl;
    for (auto it = m_map_samples.begin(); it != m_map_samples.end();) {
                std::cout <<  std::format("Sample X: {}, Y: {}, Heading: {}, Distance to Query: {}", 
                (*it)->get_x(), (*it)->get_y(), (*it)->get_heading(),calc_squared_dist(**it,*query_node)) << std::endl;
                ++it;
    }
    
    std::cout << "Testing K-D Trees using above points" << std::endl;

    NodeKDTreeAdapter adapter;

    adapter.nodes = m_map_samples;
    KDTree_t kdtree(2 /* dimensions */, adapter, {10 /* Leaf nodes*/});
    kdtree.buildIndex();
    const float query_pt[2] = {30.0f,30.0f};
    const float search_radius = 100.0f;
    std::cout <<  std::format("Query Point X: {}, Y: {}", query_pt[0], query_pt[1]) << std::endl;
    std::vector<nanoflann::ResultItem<float, float>> ret_matches;

    const size_t nMatches = kdtree.radiusSearch(&query_pt[0], search_radius, ret_matches);

        std::cout << "radiusSearch(): radius=" << search_radius << " -> " << nMatches
             << " matches\n";
        for (size_t i = 0; i < nMatches; i++)
             std::cout << "idx[" << i << "]=" << ret_matches[i].first << " dist[" << i
                 << "]=" << ret_matches[i].second <<  std::endl;
         std::cout << "\n";

    return 0;
}