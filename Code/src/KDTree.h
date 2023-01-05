//#pragma once
#include <VVRScene/scene.h>

/**
 * A node of a KD-Tree
 */

typedef std::vector<std::pair<vec, int>> Vecpoint;

struct KDNode
{
    vec split_point;
    int split_point_index;
    int axis;
    int level;
    KDNode* child_left;
    KDNode* child_right;
    KDNode() : child_left(NULL), child_right(NULL) {}
    ~KDNode() { delete child_left; delete child_right; }
};

/**
 * KD-Tree wrapper. Holds a ptr to tree root.
 */
class KDTree
{
public:
    KDTree(Vecpoint& pts);
    ~KDTree();
    int depth() const { return m_depth; }
    const KDNode* root() const { return m_root; }
    
    const Vecpoint& pts;

private:
    static int makeNode(KDNode* node, Vecpoint& pts, const int level);

private:
    KDNode* m_root;
    int m_depth;
};


void findNearest(const vec& test_pt, const KDNode* root, const KDNode** nn, float* best_dist);


/**
 * Function object to compare 2 3D-vecs in the specified axis.
 */
struct VecComparator {
    unsigned axis;
    VecComparator(unsigned axis) : axis(axis % 3) {}
    virtual inline bool operator() (const std::pair<vec, int>& v1, const std::pair<vec, int>& v2) {
        return (v1.first.ptr()[axis] < v2.first.ptr()[axis]);
    }
};