#include "KDTree.h"

#define DIMENSIONS 3

KDTree::KDTree(Vecpoint& pts)
    : pts(pts)
{
    const float t = vvr::getSeconds();
    m_root = new KDNode();
    m_depth = makeNode(m_root, pts, 0);
    const float KDTree_construction_time = vvr::getSeconds() - t;
    echo(KDTree_construction_time);
    echo(m_depth);
}

KDTree::~KDTree()
{
    const float t = vvr::getSeconds();
    delete m_root;
    const float KDTree_destruction_time = vvr::getSeconds() - t;
    echo(KDTree_destruction_time);
}

int KDTree::makeNode(KDNode* node, Vecpoint& pts, const int level)
{
    //! Sort along the appropriate axis, find median point and split.
    const int axis = level % DIMENSIONS;
    std::sort(pts.begin(), pts.end(), VecComparator(axis));
    const int i_median = round(pts.size() / 2);

    //! Set node members
    node->level = level;
    node->axis = axis;
    node->split_point = pts[i_median].first;
    node->split_point_index = pts[i_median].second;

    //! Continue recursively or stop.
    if (pts.size() <= 1)
    {
        return level;
    }
    else
    {
        int level_left = 0;
        int level_right = 0;
        Vecpoint pts_left(pts.begin(), pts.begin() + i_median);
        Vecpoint pts_right(pts.begin() + i_median + 1, pts.end());

        if (!pts_left.empty())
        {
            node->child_left = new KDNode();
            level_left = makeNode(node->child_left, pts_left, level + 1);

        }
        if (!pts_right.empty())
        {
            node->child_right = new KDNode();
            level_right = makeNode(node->child_right, pts_right, level + 1);
        }

        int max_level = std::max(level_left, level_right);
        return max_level;
    }
}

void findNearest(const vec& test_pt, const KDNode* root, const KDNode** nn, float* best_dist)
{
    if (!root) return;

    //! Distances
    const double d = test_pt.Distance(root->split_point);
    const double d_split = root->split_point.ptr()[root->axis] - test_pt.ptr()[root->axis];
    const bool right_of_split = d_split <= 0;

    if (*nn == NULL || d < *best_dist) {
        *best_dist = d;
        *nn = root;
    }

    if (right_of_split) findNearest(test_pt, root->child_right, nn, best_dist);
    else findNearest(test_pt, root->child_left, nn, best_dist);
    
    if (d_split  >= *best_dist) return;

    if (right_of_split) findNearest(test_pt, root->child_left, nn, best_dist);
    else findNearest(test_pt, root->child_right, nn, best_dist);

    return;
}