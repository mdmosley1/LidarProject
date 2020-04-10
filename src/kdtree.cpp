#include "kdtree.h"

KdTree::KdTree() : root(NULL)
{

}

void KdTree::insertNode(Node* &node, std::vector<float> point, int id)
{
    int splitIdx = depth++ % 3; // 3 because this is 3d tree
    // base case
    if (node == NULL)
        node = new Node(point, id);
    else if (point[splitIdx] < node->point[splitIdx])
    {
        // insert node onto left child if x/y is < node.x/y
        insertNode(node->left, point, id);
    }
    else
    {
        // insert node onto right child if x/y is >=  node.x/y
        insertNode(node->right, point, id);
    }
}

void KdTree::insert(std::vector<float> point, int id)
{
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the root
    depth = 0;
    insertNode(root, point, id);
}


bool KdTree::WithinBox(const Node* node,
               const std::vector<float> target,
               const float distanceTol) const
{
    const std::vector<float>& p = node->point;
    return p[0] > target[0] - distanceTol &&
        p[0] < target[0] + distanceTol &&
               p[1] > target[1] - distanceTol &&
        p[1] < target[1] + distanceTol;
}

double KdTree::norm(std::vector<float> a, std::vector<float> b) const
{
    return sqrt( (a[0] - b[0])*(a[0] - b[0]) +  (a[1] - b[1])*(a[1] - b[1]) );
}

bool KdTree::WithinTolerance(const Node* node,
                     const std::vector<float> target,
                     const float distanceTol) const
{
    const std::vector<float>& p = node->point;
    return norm(p, target) < distanceTol;
}

// return a list of point ids in the tree that are within distance of target
void KdTree::SearchRecursive(const Node* node,
                     const std::vector<float> target,
                     const float distanceTol,
                     const int depth,
                     std::vector<int>& ids) const
{
    // check if this node is null
    if (node == NULL)
        return;
    // check if this node is within the box
    if (WithinBox(node, target, distanceTol))
    {
        // if it is, then check if distance is within the
        // tolerance
        if (WithinTolerance(node, target, distanceTol))
        {
            // if node withing tolerance then add this node
            // the the list of points
            ids.push_back(node->id);
        }
        SearchRecursive(node->left, target, distanceTol, depth + 1, ids);
        SearchRecursive(node->right, target, distanceTol, depth + 1, ids);
    }
    else
    {
        // Determine which side of the branch to search. Check
        // which side(s) contain the search box
        int splitIdx = depth % 3; // 3 because this is 3d tree
        if (target[splitIdx] - distanceTol < node->point[splitIdx])
            SearchRecursive(node->left, target, distanceTol, depth + 1, ids);
        if (target[splitIdx] + distanceTol > node->point[splitIdx])
            SearchRecursive(node->right, target, distanceTol, depth + 1, ids);                
    }
}

std::vector<int> KdTree::search(const std::vector<float> target,
                        const float distanceTol) const
{
    std::vector<int> ids;
    SearchRecursive(root, target, distanceTol, 0, ids);// recursive function to search kdtree
    return ids;
}
