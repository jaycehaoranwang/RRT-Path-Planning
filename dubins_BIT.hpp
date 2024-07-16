#ifndef DUBINS_BIT_PLANNER_HEADER
#define DUBINS_BIT_PLANNER_HEADER
#include <array>
#include <tuple>
#include <vector>
#include <iostream>
#include <unordered_set>

class Node
{
    private:
        float m_x {};
        float m_y {};
        float heading {};
        Node *parent {};

    public:
        Node(float x, float y)
            : m_x {x}, m_y {y}
        { // Empty constructor body
        }

        std::tuple<float, float> get_location() const 
        {
            return std::make_tuple(m_x,m_y);
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

};

class Environment_Map
{
    private:
        std::tuple<int,int> m_x_range {};
        std::tuple<int,int> m_y_range {};
        std::vector<std::array<float,4>> m_obstacles {};

    public:
        Environment_Map(std::tuple<int,int> x_range, std::tuple<int,int> y_range)
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
};

class BIT_Dubins_Planner
{
    private:
        const Node* m_start {};
        Node* m_goal {};
        const std::vector<float>& m_discrete_headings {};  // Reference member variable
        const Environment_Map& m_env_map {};
        const float m_min_turning_radius {};
        const int m_max_iters {};
        float m_search_radius {};

    public:
        BIT_Dubins_Planner(const Node* start, Node* goal, const std::vector<float>& discrete_headings, const Environment_Map& env_map, float search_radius, const int max_iters, const float min_turning_radius)
            : m_start {start}
            , m_goal {goal}
            , m_discrete_headings {discrete_headings}
            , m_env_map {env_map}
            , m_min_turning_radius {min_turning_radius}
            , m_max_iters {max_iters}
            , m_search_radius {search_radius}
        {
        }
};


#endif 