#include "cluster.h"

static std::set<int> processedPoints_g;

void MarkPointAsProcessed(const int i)
{
    processedPoints_g.emplace(i);
}

bool PointHasBeenProcessed(const int i)
{
    if (processedPoints_g.find(i) == processedPoints_g.end())
        return false;
    else
        return true;
}


// add all points within desired proximity of point to cluster
void Proximity(const std::vector<std::vector<float>>& points,
               std::vector<int>& cluster,
               const int ptIdx,
               const KdTree* tree,
               const float distanceTol)
{
    if (PointHasBeenProcessed(ptIdx))
        return;
    // mark point as processed
    MarkPointAsProcessed(ptIdx);
    // add point to cluster
    cluster.push_back(ptIdx);
    // find nearby points from tree
    std::vector<int> nearbyPointIndices = tree->search(points[ptIdx], distanceTol);

    // recurse for each nearby point
    for (auto i : nearbyPointIndices)
        if (!PointHasBeenProcessed(i))
            Proximity(points, cluster, i, tree, distanceTol);
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points,
                                               const KdTree* tree,
                                               float distanceTol)
{
    processedPoints_g.clear();
    std::vector<std::vector<int>> clusters;

    for (int i = 0; i < points.size(); ++i)
    {
        if (!PointHasBeenProcessed(i))
        {
            std::vector<int> cluster;
            Proximity(points, cluster, i, tree, distanceTol);
            clusters.push_back(cluster);
        }
    }

    return clusters;
}
