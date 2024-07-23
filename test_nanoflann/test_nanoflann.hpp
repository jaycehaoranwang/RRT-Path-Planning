#include "nanoflann.hpp"
#include <iostream>
#include <memory>
#include <random>
#include <format>
#include <unordered_set>
#include <cmath> // Include for fmod

class Node
{
    private:
        float x {};
        float y {};
        float heading {};
        std::shared_ptr<Node> parent {};
    public:
        
        Node(float arg_x, float arg_y)
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

        std::shared_ptr<Node> get_parent() const 
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

        void set_parent(std::shared_ptr<Node> parent_pointer)
        {
            this->parent = parent_pointer;
        }
        // Equality operator
        bool operator==(const Node& other) const
        {
            return x == other.x && y == other.y && heading == other.heading && parent == other.parent;
        }


};

// Custom hash function for Node pointer objects
struct NPHash {
    std::size_t operator()(const std::shared_ptr<Node>& np) const
    {
        // Use std::hash for each member
        size_t h1 = std::hash<float>()(np->get_x());
        size_t h2 = std::hash<float>()(np->get_y());
        size_t h3 = std::hash<float>()(np->get_heading());
        size_t h4 = std::hash<std::shared_ptr<Node>>()(np->get_parent());
        
        // Combine the hash values using prime numbers
        return h1 ^ (h2 * 2) ^ (h3 * 3) ^ (h4 * 11); // Prime number multipliers
    }
};

// Custom equality functor for std::shared_ptr<Node>
struct NPNodeEqual {
    bool operator()(const std::shared_ptr<Node>& lhs, const std::shared_ptr<Node>& rhs) const {
        // Check if the pointed-to objects are equal
        return *lhs == *rhs;
    }
};

struct NodeKDTreeAdapter
{
    std::unordered_set<std::shared_ptr<Node>, NPHash, NPNodeEqual> nodes; // set of shared_ptr<Node>

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


template <typename T>
class IndexableSet {
public:
    bool insert(const T& value) {
        if (set.find(value) != set.end()) {
            return false; // Value already exists
        }
        set.insert(value);
        vec.push_back(value);
        return true;
    }

    bool erase(const T& value) {
        auto it = set.find(value);
        if (it == set.end()) {
            return false; // Value not found
        }
        set.erase(it);
        vec.erase(std::remove(vec.begin(), vec.end(), value), vec.end());
        return true;
    }

    const T& at(std::size_t index) const {
        if (index >= vec.size()) {
            throw std::out_of_range("Index out of range");
        }
        return vec.at(index);
    }

    std::size_t size() const {
        return vec.size();
    }

private:
    std::vector<T> vec;
    std::unordered_set<T> set;
};

