#include "Octree.h"
#include <algorithm>
#include <array>
#include <queue>
#include <vector>

struct Octree::OctreeNode {
  AABB boundingBox;
  std::shared_ptr<OctreeNode> subNodes[8];
  int primCount = -1;
  int primIdxBuffer[ocLeafMaxSize];
  std::vector<int> primIdxExtra;
};

namespace {
inline bool rayBoxIntersect(const AABB &box, const Ray &ray) {
  float t0 = ray.tNear;
  float t1 = ray.tFar;
  for (int axis = 0; axis < 3; ++axis) {
    float dir = ray.direction[axis];
    float org = ray.origin[axis];
    if (std::abs(dir) < EPSILON) {
      if (org < box.pMin[axis] || org > box.pMax[axis]) {
        return false;
      }
      continue;
    }

    float invD = 1.f / dir;
    float tNear = (box.pMin[axis] - org) * invD;
    float tFar = (box.pMax[axis] - org) * invD;
    if (tNear > tFar) {
      std::swap(tNear, tFar);
    }
    t0 = std::max(t0, tNear);
    t1 = std::min(t1, tFar);
    if (t0 > t1) {
      return false;
    }
  }
  return true;
}
} // namespace

Octree::OctreeNode *Octree::recursiveBuild(const AABB &aabb,
                                           const std::vector<int> &primIdxBuffer) {
  //* todo 完成递归构建八叉树
  //* 构建方法请看实验手册
  //* 要注意的一种特殊是当节点的某个子包围盒和当前节点所有物体都相交，我们就不用细分了，当前节点作为叶子节点即可。
  if (primIdxBuffer.empty()) {
    return nullptr;
  }

  auto *node = new OctreeNode();
  node->boundingBox = aabb;

  auto makeLeaf = [&](const std::vector<int> &buffer) {
    node->primCount = (int)buffer.size();
    if (node->primCount <= ocLeafMaxSize) {
      for (int i = 0; i < node->primCount; ++i) {
        node->primIdxBuffer[i] = buffer[i];
      }
    } else {
      node->primIdxExtra = buffer;
    }
  };

  if ((int)primIdxBuffer.size() <= ocLeafMaxSize) {
    makeLeaf(primIdxBuffer);
    return node;
  }

  Point3f c = aabb.Center();
  std::array<AABB, 8> subBoxes;
  for (int i = 0; i < 8; ++i) {
    Point3f mn{
        (i & 0b100) ? c[0] : aabb.pMin[0],
        (i & 0b010) ? c[1] : aabb.pMin[1],
        (i & 0b001) ? c[2] : aabb.pMin[2],
    };
    Point3f mx{
        (i & 0b100) ? aabb.pMax[0] : c[0],
        (i & 0b010) ? aabb.pMax[1] : c[1],
        (i & 0b001) ? aabb.pMax[2] : c[2],
    };
    subBoxes[i] = AABB(mn, mx);
  }

  std::array<std::vector<int>, 8> subBuffers;
  bool stopSplit = false;
  for (int i = 0; i < 8; ++i) {
    for (int idx : primIdxBuffer) {
      if (shapes[idx]->getAABB().Overlap(subBoxes[i])) {
        subBuffers[i].emplace_back(idx);
      }
    }
    if ((int)subBuffers[i].size() == (int)primIdxBuffer.size()) {
      stopSplit = true;
    }
  }

  if (stopSplit) {
    makeLeaf(primIdxBuffer);
    return node;
  }

  bool hasChild = false;
  for (int i = 0; i < 8; ++i) {
    if (subBuffers[i].empty()) {
      continue;
    }
    hasChild = true;
    node->subNodes[i] =
        std::shared_ptr<OctreeNode>(recursiveBuild(subBoxes[i], subBuffers[i]));
  }

  if (!hasChild) {
    makeLeaf(primIdxBuffer);
  }
  return node;
}
void Octree::build() {
  //* 首先计算整个场景的范围
  for (const auto & shape : shapes) {
    //* 自行实现的加速结构请务必对每个shape调用该方法，以保证TriangleMesh构建内部加速结构
    //* 由于使用embree时，TriangleMesh::getAABB不会被调用，因此出于性能考虑我们不在TriangleMesh
    //* 的构造阶段计算其AABB，因此当我们将TriangleMesh的AABB计算放在TriangleMesh::initInternalAcceleration中
    //* 所以请确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
    shape->initInternalAcceleration();

    boundingBox.Expand(shape->getAABB());
  }

  //* 构建八叉树
  std::vector<int> primIdxBuffer(shapes.size());
  std::iota(primIdxBuffer.begin(), primIdxBuffer.end(), 0);
  root = recursiveBuild(boundingBox, primIdxBuffer);
}

bool Octree::rayIntersect(Ray &ray, int *geomID, int *primID,
                          float *u, float *v) const {
  //*todo 完成八叉树求交
  if (root == nullptr) {
    return false;
  }

  bool hit = false;
  std::vector<const OctreeNode *> stack;
  stack.reserve(256);
  stack.emplace_back(root);
  std::vector<char> tested(shapes.size(), 0);

  while (!stack.empty()) {
    const OctreeNode *node = stack.back();
    stack.pop_back();

    if (node == nullptr || !rayBoxIntersect(node->boundingBox, ray)) {
      continue;
    }

    if (node->primCount >= 0) {
      if (node->primCount <= ocLeafMaxSize) {
        for (int i = 0; i < node->primCount; ++i) {
          int idx = node->primIdxBuffer[i];
          if (!tested[idx] && shapes[idx]->rayIntersectShape(ray, primID, u, v)) {
            *geomID = idx;
            hit = true;
          }
          tested[idx] = 1;
        }
      } else {
        for (int idx : node->primIdxExtra) {
          if (!tested[idx] && shapes[idx]->rayIntersectShape(ray, primID, u, v)) {
            *geomID = idx;
            hit = true;
          }
          tested[idx] = 1;
        }
      }
      continue;
    }

    for (int i = 0; i < 8; ++i) {
      if (node->subNodes[i] != nullptr) {
        stack.emplace_back(node->subNodes[i].get());
      }
    }
  }

  return hit;
}