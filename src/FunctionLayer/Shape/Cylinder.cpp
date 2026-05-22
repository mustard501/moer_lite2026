#include "Cylinder.h"
#include "ResourceLayer/Factory.h"
bool Cylinder::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* todo 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
    //* 1.光线变换到局部空间
    //* 2.联立方程求解
    //* 3.检验交点是否在圆柱范围内
    //* 4.更新ray的tFar,减少光线和其他物体的相交计算次数
    //* Write your code here.
    Ray localRay = transform.inverseRay(ray);
    Point3f origin = localRay.origin;
    Vector3f direction = localRay.direction;
    float A = direction[0]*direction[0] + direction[1]*direction[1];
    float B = 2*(origin[0]*direction[0] + origin[1]*direction[1]);
    float C = origin[0]*origin[0] + origin[1]*origin[1] - radius*radius;
    float t0, t1;
    bool flag[2] = {false, false};
    if(!Quadratic(A, B, C, &t0, &t1)){
        return false;
    }

    Point3f hit_t0 = origin + t0*direction;
    Point3f hit_t1 = origin + t1*direction;

    float phi_t0 = atan2(hit_t0[1], hit_t0[0]);
    float phi_t1 = atan2(hit_t1[1], hit_t1[0]);


    if(t0 >= ray.tNear && t0 <= ray.tFar){
        if(hit_t0[2] >= 0 && hit_t0[2] <= height){
            if(phi_t0 < 0){
                phi_t0 += 2*PI;
            }
            if(phi_t0 <= phiMax){
                flag[0] = true;
            }
        }
    }
    if(t1 >= ray.tNear && t1 <= ray.tFar){
        if(hit_t1[2] >= 0 && hit_t1[2] <= height){
            if(phi_t1 < 0){
                phi_t1 += 2*PI;
            }
            if(phi_t1 <= phiMax){
                flag[1] = true;
            }
        }
    }
    
    // t0小于t1,优先判断t0
    if(flag[0]){
        *primID = 0;
        ray.tFar = t0;
        *u = phi_t0 / phiMax;
        *v = (hit_t0[2] - 0) / height;
        return true;
    }
    else if(flag[1]){
        *primID = 0;
        ray.tFar = t1;
        *u = phi_t1 / phiMax;
        *v = (hit_t1[2] - 0) / height;
        return true;
    }
    else{
        return false;
    }
    return false;
}

void Cylinder::fillIntersection(float distance, int primID, float u, float v, Intersection *intersection) const {
    /// ----------------------------------------------------
    //* todo 填充圆柱相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    //* 2.位置信息可以根据uv计算出，同样需要变换
    //* Write your code here.
    /// ----------------------------------------------------
    float phi = u * phiMax;
    float theta = v * height;
    Point3f position = Point3f(0, 0, theta) + radius * Vector3f(cos(phi), sin(phi), 0);
    intersection->position = transform.toWorld(position);
    intersection->normal =transform.toWorld(Vector3f(cos(phi), sin(phi), 0));
    
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

void Cylinder::uniformSampleOnSurface(Vector2f sample, Intersection *result, float *pdf) const {

}

Cylinder::Cylinder(const Json &json) : Shape(json) {
    radius = fetchOptional(json,"radius",1.f);
    height = fetchOptional(json,"height",1.f);
    phiMax = fetchOptional(json,"phi_max",2 * PI);
    AABB localAABB = AABB(Point3f(-radius,-radius,0),Point3f(radius,radius,height));
    boundingBox = transform.toWorld(localAABB);
}

REGISTER_CLASS(Cylinder,"cylinder")
