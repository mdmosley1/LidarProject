/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KDTREE_H
#define KDTREE_H

#include "render/render.h"
#include <stdio.h>

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

Node(std::vector<float> arr, int setId)
:    point(arr), id(setId), left(NULL), right(NULL)
        {}
};

class KdTree
{
public:
    Node* root;
    int depth = 0;
    KdTree();

    void insert(std::vector<float> point, int id);
    std::vector<int> search(const std::vector<float> target,
                            const float distanceTol) const ;
private:

    void insertNode(Node* &node, std::vector<float> point, int id);

    bool WithinBox(const Node* node,
                   const std::vector<float> target,
                   const float distanceTol) const;

    double norm(std::vector<float> a, std::vector<float> b) const;
    bool WithinTolerance(const Node* node,
                         const std::vector<float> target,
                         const float distanceTol) const;
    // return a list of point ids in the tree that are within distance of target
    void SearchRecursive(const Node* node,
                         const std::vector<float> target,
                         const float distanceTol,
                         const int depth,
                         std::vector<int>& ids) const;

};

#endif /* KDTREE_H */
