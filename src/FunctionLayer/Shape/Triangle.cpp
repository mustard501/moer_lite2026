#include "Triangle.h"
#include <FunctionLayer/Acceleration/Linear.h>
#include <cmath>
//--- Triangle ---
Triangle::Triangle(int _primID, int _vtx0Idx, int _vtx1Idx, int _vtx2Idx,
                   const TriangleMesh *_mesh)
    : primID(_primID), vtx0Idx(_vtx0Idx), vtx1Idx(_vtx1Idx), vtx2Idx(_vtx2Idx),
      mesh(_mesh) {
  Point3f vtx0 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx0Idx]),
          vtx1 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx1Idx]),
          vtx2 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx2Idx]);
  boundingBox.Expand(vtx0);
  boundingBox.Expand(vtx1);
  boundingBox.Expand(vtx2);
  this->geometryID = mesh->geometryID;
}

bool Triangle::rayIntersectShape(Ray &ray, int *primID, float *u,
                                 float *v) const {
  Point3f p0 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx0Idx]);
  Point3f p1 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx1Idx]);
  Point3f p2 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx2Idx]);

  Vector3f e1 = p1 - p0;
  Vector3f e2 = p2 - p0;

  Vector3f pvec = cross(ray.direction, e2);
  float det = dot(e1, pvec);
  if (std::abs(det) < EPSILON) {
    return false;
  }
  float invDet = 1.f / det;

  Vector3f tvec = ray.origin - p0;
  float b1 = dot(tvec, pvec) * invDet;
  if (b1 < .0f || b1 > 1.f) {
    return false;
  }

  Vector3f qvec = cross(tvec, e1);
  float b2 = dot(ray.direction, qvec) * invDet;
  if (b2 < .0f || b1 + b2 > 1.f) {
    return false;
  }

  float t = dot(e2, qvec) * invDet;
  if (t < ray.tNear || t > ray.tFar) {
    return false;
  }

  ray.tFar = t;
  *primID = this->primID;
  *u = b1;
  *v = b2;
  return true;
}

void Triangle::fillIntersection(float distance, int primID, float u, float v,
                                Intersection *intersection) const {
  // 该函数实际上不会被调用
  return;
}

//--- TriangleMesh ---
TriangleMesh::TriangleMesh(const Json &json) : Shape(json) {
  const auto &filepath = fetchRequired<std::string>(json, "file");
  meshData = MeshData::loadFromFile(filepath);
}

RTCGeometry TriangleMesh::getEmbreeGeometry(RTCDevice device) const {
  RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

  float *vertexBuffer = (float *)rtcSetNewGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float),
      meshData->vertexCount);
  for (int i = 0; i < meshData->vertexCount; ++i) {
    Point3f vertex = transform.toWorld(meshData->vertexBuffer[i]);
    vertexBuffer[3 * i] = vertex[0];
    vertexBuffer[3 * i + 1] = vertex[1];
    vertexBuffer[3 * i + 2] = vertex[2];
  }

  unsigned *indexBuffer = (unsigned *)rtcSetNewGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
      3 * sizeof(unsigned), meshData->faceCount);
  for (int i = 0; i < meshData->faceCount; ++i) {
    indexBuffer[i * 3] = meshData->faceBuffer[i][0].vertexIndex;
    indexBuffer[i * 3 + 1] = meshData->faceBuffer[i][1].vertexIndex;
    indexBuffer[i * 3 + 2] = meshData->faceBuffer[i][2].vertexIndex;
  }
  rtcCommitGeometry(geometry);
  return geometry;
}

bool TriangleMesh::rayIntersectShape(Ray &ray, int *primID, float *u,
                                     float *v) const {
  //* 当使用embree加速时，该方法不会被调用
  int geomID = -1;
  return acceleration->rayIntersect(ray, &geomID, primID, u, v);
}

void TriangleMesh::fillIntersection(float distance, int primID, float u,
                                    float v, Intersection *intersection) const {
  intersection->distance = distance;
  intersection->shape = this;
  const auto &face = meshData->faceBuffer[primID];
  const auto &d0 = face[0];
  const auto &d1 = face[1];
  const auto &d2 = face[2];

  Point3f p0 = transform.toWorld(meshData->vertexBuffer[d0.vertexIndex]);
  Point3f p1 = transform.toWorld(meshData->vertexBuffer[d1.vertexIndex]);
  Point3f p2 = transform.toWorld(meshData->vertexBuffer[d2.vertexIndex]);

  float w = 1.f - u - v;
  Point3f position{w * p0[0] + u * p1[0] + v * p2[0],
                   w * p0[1] + u * p1[1] + v * p2[1],
                   w * p0[2] + u * p1[2] + v * p2[2]};
  intersection->position = position;

  Vector3f geometricNormal = normalize(cross(p1 - p0, p2 - p0));
  Vector3f shadingNormal = geometricNormal;
  if (!meshData->normalBuffer.empty() && d0.normalIndex >= 0 &&
      d1.normalIndex >= 0 && d2.normalIndex >= 0) {
    Vector3f n0 = transform.toWorld(meshData->normalBuffer[d0.normalIndex]);
    Vector3f n1 = transform.toWorld(meshData->normalBuffer[d1.normalIndex]);
    Vector3f n2 = transform.toWorld(meshData->normalBuffer[d2.normalIndex]);
    shadingNormal = normalize(n0 * w + n1 * u + n2 * v);
  }
  intersection->normal = shadingNormal;

  Vector2f uv0{0.f, 0.f}, uv1{1.f, 0.f}, uv2{0.f, 1.f};
  bool hasUV = (!meshData->texcodBuffer.empty() && d0.texcodIndex >= 0 &&
                d1.texcodIndex >= 0 && d2.texcodIndex >= 0);
  if (hasUV) {
    uv0 = meshData->texcodBuffer[d0.texcodIndex];
    uv1 = meshData->texcodBuffer[d1.texcodIndex];
    uv2 = meshData->texcodBuffer[d2.texcodIndex];
  }
  intersection->texCoord = uv0 * w + uv1 * u + uv2 * v;

  Vector3f dp02 = p0 - p2;
  Vector3f dp12 = p1 - p2;
  Vector2f duv02 = uv0 - uv2;
  Vector2f duv12 = uv1 - uv2;
  float det = duv02[0] * duv12[1] - duv02[1] * duv12[0];

  Vector3f dpdu, dpdv;
  if (hasUV && std::abs(det) > EPSILON) {
    float invDet = 1.f / det;
    dpdu = (dp02 * duv12[1] - dp12 * duv02[1]) * invDet;
    dpdv = (-dp02 * duv12[0] + dp12 * duv02[0]) * invDet;
  } else {
    dpdu = p1 - p0;
    dpdv = cross(geometricNormal, dpdu);
    if (dpdv.isZero()) {
      dpdu = Vector3f{1.f, 0.f, 0.f};
      if (std::abs(dot(dpdu, geometricNormal)) > .9f) {
        dpdu = Vector3f{0.f, 1.f, 0.f};
      }
      dpdv = cross(geometricNormal, dpdu);
    }
  }

  intersection->dpdu = dpdu;
  intersection->dpdv = dpdv;

  Vector3f tangent = normalize(dpdu);
  Vector3f bitangent = normalize(cross(intersection->normal, tangent));
  if (bitangent.isZero()) {
    tangent = Vector3f{1.f, 0.f, 0.f};
    if (std::abs(dot(tangent, intersection->normal)) > .9f) {
      tangent = Vector3f{0.f, 1.f, 0.f};
    }
    bitangent = normalize(cross(intersection->normal, tangent));
    tangent = normalize(cross(bitangent, intersection->normal));
  }
  intersection->tangent = tangent;
  intersection->bitangent = bitangent;
}

void TriangleMesh::initInternalAcceleration() {
  acceleration = Acceleration::createAcceleration();
  int primCount = meshData->faceCount;
  for (int primID = 0; primID < primCount; ++primID) {
    int vtx0Idx = meshData->faceBuffer[primID][0].vertexIndex,
        vtx1Idx = meshData->faceBuffer[primID][1].vertexIndex,
        vtx2Idx = meshData->faceBuffer[primID][2].vertexIndex;
    std::shared_ptr<Triangle> triangle =
        std::make_shared<Triangle>(primID, vtx0Idx, vtx1Idx, vtx2Idx, this);
    acceleration->attachShape(triangle);
  }
  acceleration->build();
  // TriangleMesh的包围盒就是其内部加速结构的包围盒
  boundingBox = acceleration->boundingBox;
}
REGISTER_CLASS(TriangleMesh, "triangle")