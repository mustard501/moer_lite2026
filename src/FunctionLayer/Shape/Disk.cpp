#include "Disk.h"
#include "ResourceLayer/Factory.h"
bool Disk::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* todo 完成光线与圆环的相交 填充primId,u,v.如果相交，更新光线的tFar
    //* 1.光线变换到局部空间
    //* 2.判断局部光线的方向在z轴分量是否为0
    //* 3.计算光线和平面交点
    //* 4.检验交点是否在圆环内
    //* 5.更新ray的tFar,减少光线和其他物体的相交计算次数
    //* Write your code here.
    Ray localRay = transform.inverseRay(ray);
    Point3f origin = localRay.origin;
    Vector3f direction = localRay.direction;
    if(direction[2] == 0){
        return false;
    }
    float t = -origin[2] / direction[2];
    if(t < ray.tNear || t > ray.tFar){
        return false;
    }
    Point3f hit = origin + t*direction;
    float distance = (hit - Point3f(0, 0, 0)).length();
    if(distance > radius || distance < innerRadius){
        return false;
    }
    float phi = atan2(hit[1], hit[0]);
    if(phi < 0){
        phi += 2*PI;
    }
    if(phi > phiMax){
        return false;
    }
    *primID = 0;
    *u = phi / phiMax;
    *v = (distance -innerRadius) / (radius - innerRadius);
    ray.tFar = t;
    return true;
}


void Disk::fillIntersection(float distance, int primID, float u, float v, Intersection *intersection) const {
    /// ----------------------------------------------------
    //* todo 填充圆环相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    //* 2.位置信息可以根据uv计算出，同样需要变换
    //* Write your code here.
    /// ----------------------------------------------------

    float phi = u * phiMax;
    float theta = v * (radius - innerRadius) + innerRadius;
    Point3f position = Point3f(0, 0, 0) + theta * Vector3f(cos(phi), sin(phi), 0);
    intersection->position = transform.toWorld(position);
    intersection->normal = transform.toWorld(Vector3f(0, 0, 1));

    intersection->shape = this;
    intersection->distance = distance;
    intersection->texCoord = Vector2f{u, v};
    Vector3f tangent{1.f, 0.f, .0f};
    Vector3f bitangent;
    if (std::abs(dot(tangent, intersection->normal)) > .9f) {
        tangent = Vector3f(.0f, 1.f, .0f);
    }
    bitangent = normalize(cross(tangent, intersection->normal));
    tangent = normalize(cross(intersection->normal, bitangent));
    intersection->tangent = tangent;
    intersection->bitangent = bitangent;
}

Disk::Disk(const Json &json) : Shape(json) {
//    normal = transform.toWorld(Vector3f(0,0,1));
//    origin = transform.toWorld(Point3f(0,0,0));
//    auto
//    //radius认为是三个方向的上的scale平均
//    vecmat::vec4f v(1,1,1,0);
//    auto radiusVec = transform.scale * v;
//    radiusVec/=radiusVec[3];
//    radius = (radiusVec[0]+radiusVec[1]+radiusVec[2])/3;
     radius = fetchOptional(json,"radius",1.f);
     innerRadius = fetchOptional(json,"inner_radius",0.f);
     phiMax = fetchOptional(json,"phi_max",2 * PI);
     AABB local(Point3f(-radius,-radius,0),Point3f(radius,radius,0));
     boundingBox = transform.toWorld(local);
}

void Disk::uniformSampleOnSurface(Vector2f sample, Intersection *result, float *pdf) const {
        //采样光源 暂时不用实现
}
REGISTER_CLASS(Disk, "disk")

