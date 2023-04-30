//
// Created by itamar on 4/16/23.
//

#ifndef MANIPULATION_PLANNING_KDTREE_HPP
#define MANIPULATION_PLANNING_KDTREE_HPP

#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>


/// \brief A class that represents a node in the KDTree
/// \tparam T The type of the data that the node holds
template <typename T>
class KDNode{
public:
    /// \brief The constructor of the class
    /// \param data The data that the node holds
    /// \param parent The parent of the node
    /// \param left The left child of the node
    /// \param right The right child of the node
    /// \param depth The depth of the node
    KDNode(const T& data, KDNode<T>* parent, KDNode<T>* left, KDNode<T>* right, int depth) :
            data(data), parent(parent), left(left), right(right), depth(depth) {}

    /// \brief The data that the node holds
    T data;

    /// \brief The parent of the node
    KDNode<T>* parent;

    /// \brief The left child of the node
    KDNode<T>* left;

    /// \brief The right child of the node
    KDNode<T>* right;

    /// \brief The depth of the node
    int depth;
};

/// \brief A class that represents a KDTree
/// \tparam T The type of the data that the tree holds
template <typename T>
class KDTree{
public:
    /// \brief The constructor of the class
    /// \param data The data that the tree holds
    /// \param k The number of dimensions of the data
    KDTree(const std::vector<T>& data, int k) : data(data), k(k) {
        root = buildTree(data, 0);
    }

    /// \brief The destructor of the class
    ~KDTree(){
        deleteTree(root);
    }

    /// \brief A function that returns the root of the tree
    /// \return The root of the tree
    KDNode<T>* getRoot(){
        return root;
    }

    /// \brief A function that returns the data that the tree holds
    /// \return The data that the tree holds
    std::vector<T> getData(){
        return data;
    }

    /// \brief A function that returns the number of dimensions of the data
    /// \return The number of dimensions of the data
    int getK(){
        return k;
    }

    /// \brief A function that returns the nearest neighbor of a given point
    /// \param point The given point
    /// \return The nearest neighbor of the given point
    T nearestNeighbor(const T& point){
        return nearestNeighbor(root, point, root->data);
    }

    /// \brief A function that returns the nearest neighbor of a given point
    /// \param node The current node
    /// \param point The given point
    /// \param best The current best neighbor
    /// \return The nearest neighbor of the given point
    T nearestNeighbor(KDNode<T>* node, const T& point, const T& best){
        if (node == nullptr){
            return best;
        }
        if (node->data == point){
            return node->data;
        }
        auto bestDistance = distance(best, point);
        auto nodeDistance = distance(node->data, point);
        std::cout << "best: " << bestDistance << " node: " << nodeDistance << std::endl;
        if (nodeDistance < bestDistance){
            return nearestNeighbor(node->left, point, node->data);
        }
        else{
            return nearestNeighbor(node->right, point, best);
        }
    }

    /// \brief A function that returns the distance between two points
    /// \param point1 The first point
    /// \param point2 The second point
    /// \return The distance between the two points
    double distance(const T& point1, const T& point2){
        return acos(cos(point1[0])*cos(point2[0]) + sin(point1[0])*sin(point2[0])*cos(point1[1] - point2[1] + point1[2] - point2[2]));
    }

private:

    /// \brief The data that the tree holds
    std::vector<T> data;

    /// \brief The number of dimensions of the data
    int k;

    /// \brief The root of the tree
    KDNode<T>* root;

    /// \brief A function that builds the tree
    /// \param data_ The data that the tree holds
    /// \param depth The depth of the current node
    /// \return The root of the tree
    KDNode<T>* buildTree(const std::vector<T>& data_, int depth){
        if (data_.empty()){
            return nullptr;
        }
        int axis = depth % k;
        std::vector<T> sortedData = data_;
        std::sort(sortedData.begin(), sortedData.end(), [axis](const T& point1, const T& point2){
            return point1[axis] < point2[axis];
        });
        int median = sortedData.size() / 2;
        auto* node = new KDNode<T>(sortedData[median], nullptr, nullptr, nullptr, depth);
        std::vector<T> leftData(sortedData.begin(), sortedData.begin() + median);
        std::vector<T> rightData(sortedData.begin() + median + 1, sortedData.end());
        node->left = buildTree(leftData, depth + 1);
        node->right = buildTree(rightData, depth + 1);
        if (node->left != nullptr){
            node->left->parent = node;
        }
        if (node->right != nullptr){
            node->right->parent = node;
        }
        return node;
    }

    /// \brief A function that deletes the tree
    /// \param node The current node
    void deleteTree(KDNode<T>* node){
        if (node == nullptr){
            return;
        }
        deleteTree(node->left);
        deleteTree(node->right);
        delete node;
    }
};



#endif //MANIPULATION_PLANNING_KDTREE_HPP
