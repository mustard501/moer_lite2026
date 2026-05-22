#include "BVH.h"
#include <algorithm>
#include <limits>
#include <vector>

struct BVH::BVHNode {
  BVHNode *left = nullptr;
  BVHNode *right = nullptr;
  AABB box;
  int firstShapeOffset = 0;
  int nShape = 0;
  int splitAxis = 0;
};

namespace {
struct BVHShapeInfo {
  int id;
  AABB box;
  Point3f center;
};

inline bool rayBoxIntersect(const AABB &box, const Ray &ray, float *hitNear) {
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
  if (hitNear != nullptr) {
    *hitNear = t0;
  }
  return true;
}
} // namespace

void BVH::build() {
  AABB sceneBox;
  for (const auto &shape : shapes) {
    //* 自行实现的加速结构请务必对每个shape调用该方法，以保证TriangleMesh构建内部加速结构
    //* 由于使用embree时，TriangleMesh::getAABB不会被调用，因此出于性能考虑我们不在TriangleMesh
    //* 的构造阶段计算其AABB，因此当我们将TriangleMesh的AABB计算放在TriangleMesh::initInternalAcceleration中
    //* 所以请确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
    shape->initInternalAcceleration();
    sceneBox.Expand(shape->getAABB());
  }
  boundingBox = sceneBox;

  if (shapes.empty()) {
    root = nullptr;
    return;
  }

  std::vector<BVHShapeInfo> infos(shapes.size());
  for (int i = 0; i < (int)shapes.size(); ++i) {
    infos[i] = {i, shapes[i]->getAABB(), shapes[i]->getAABB().Center()};
  }

  std::vector<std::shared_ptr<Shape>> orderedShapes;
  orderedShapes.reserve(shapes.size());

  auto recursiveBuild = [&](auto &&self, int l, int r) -> BVHNode * {
    auto *node = new BVHNode();

    AABB nodeBox;
    AABB centroidBox;
    for (int i = l; i < r; ++i) {
      nodeBox.Expand(infos[i].box);
      centroidBox.Expand(infos[i].center);
    }
    node->box = nodeBox;

    const int nShapes = r - l;
    if (nShapes <= bvhLeafMaxSize) {
      node->firstShapeOffset = (int)orderedShapes.size();
      node->nShape = nShapes;
      for (int i = l; i < r; ++i) {
        orderedShapes.emplace_back(shapes[infos[i].id]);
      }
      return node;
    }

    Vector3f diag = centroidBox.pMax - centroidBox.pMin;
    int axis = 0;
    if (diag[1] > diag[axis]) {
      axis = 1;
    }
    if (diag[2] > diag[axis]) {
      axis = 2;
    }
    node->splitAxis = axis;

    if (centroidBox.pMin[axis] == centroidBox.pMax[axis]) {
      node->firstShapeOffset = (int)orderedShapes.size();
      node->nShape = nShapes;
      for (int i = l; i < r; ++i) {
        orderedShapes.emplace_back(shapes[infos[i].id]);
      }
      return node;
    }

    std::sort(infos.begin() + l, infos.begin() + r,
              [axis](const BVHShapeInfo &a, const BVHShapeInfo &b) {
                return a.center[axis] < b.center[axis];
              });
    int mid = (l + r) / 2;
    if (mid <= l || mid >= r) {
      node->firstShapeOffset = (int)orderedShapes.size();
      node->nShape = nShapes;
      for (int i = l; i < r; ++i) {
        orderedShapes.emplace_back(shapes[infos[i].id]);
      }
      return node;
    }

    node->left = self(self, l, mid);
    node->right = self(self, mid, r);
    return node;
  };

  root = recursiveBuild(recursiveBuild, 0, (int)infos.size());
  shapes.swap(orderedShapes);
}
bool BVH::rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const {
  if (root == nullptr) {
    return false;
  }

  bool hit = false;
  std::vector<const BVHNode *> stack;
  stack.reserve(256);
  stack.emplace_back(root);

  while (!stack.empty()) {
    const BVHNode *node = stack.back();
    stack.pop_back();
    if (node == nullptr) {
      continue;
    }

    if (!rayBoxIntersect(node->box, ray, nullptr)) {
      continue;
    }

    if (node->nShape > 0) {
      for (int i = 0; i < node->nShape; ++i) {
        int idx = node->firstShapeOffset + i;
        if (shapes[idx]->rayIntersectShape(ray, primID, u, v)) {
          *geomID = idx;
          hit = true;
        }
      }
      continue;
    }

    float tLeft = std::numeric_limits<float>::infinity();
    float tRight = std::numeric_limits<float>::infinity();
    bool hitLeft = node->left != nullptr && rayBoxIntersect(node->left->box, ray, &tLeft);
    bool hitRight = node->right != nullptr && rayBoxIntersect(node->right->box, ray, &tRight);

    if (hitLeft && hitRight) {
      if (tLeft < tRight) {
        stack.emplace_back(node->right);
        stack.emplace_back(node->left);
      } else {
        stack.emplace_back(node->left);
        stack.emplace_back(node->right);
      }
    } else if (hitLeft) {
      stack.emplace_back(node->left);
    } else if (hitRight) {
      stack.emplace_back(node->right);
    }
  }

  return hit;
}


