#ifndef CLUSTER_H
#define CLUSTER_H

#include "render/render.h"
#include "render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"

void MarkPointAsProcessed(const int i);

bool PointHasBeenProcessed(const int i);

// add all points within desired proximity of point to cluster
void Proximity(const std::vector<std::vector<float>>& points,
               std::vector<int>& cluster,
               const int ptIdx,
               const KdTree* tree,
               const float distanceTol);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points,
                                               const KdTree* tree,
                                               float distanceTol);

#endif /* CLUSTER_H */
