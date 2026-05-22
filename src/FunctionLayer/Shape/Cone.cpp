#include "Cone.h"
#include "ResourceLayer/Factory.h"

bool Cone::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* todo 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
    //* 1.光线变换到局部空间
    //* 2.联立方程求解
    //* 3.检验交点是否在圆锥范围内
    //* 4.更新ray的tFar,减少光线和其他物体的相交计算次数
    //* Write your code here.
    Ray localRay = transform.inverseRay(ray);
    Point3f origin = localRay.origin;
    Vector3f direction = localRay.direction;
    Point3f upper_point = Point3f(0, 0, height);
    Vector3f axis_dir = Vector3f(0, 0, -1);
    Vector3f p2o = origin - upper_point;
    
    float A = dot(direction, axis_dir) * dot(direction, axis_dir) - cosTheta * cosTheta;
    float B = 2*(dot(direction, axis_dir)*dot(p2o, axis_dir) - cosTheta * cosTheta * dot(p2o, direction));
    float C = dot(p2o, axis_dir) * dot(p2o, axis_dir) - cosTheta * cosTheta * dot(p2o, p2o);
    float t0, t1;
    if(!Quadratic(A, B, C, &t0, &t1)){
        return false;
    }

    Point3f hit_t0 = origin + t0*direction;
    Point3f hit_t1 = origin + t1*direction;

    float phi_t0 = atan2(hit_t0[1], hit_t0[0]);
    float phi_t1 = atan2(hit_t1[1], hit_t1[0]);
    if(phi_t0 < 0){
        phi_t0 += 2*PI;
    }
    if(phi_t1 < 0){
        phi_t1 += 2*PI;
    }
    bool flag[2] = {false, false};
    if(t0 >= ray.tNear && t0 <= ray.tFar){
        if(hit_t0[2] >= 0 && hit_t0[2] <= height){
            if(phi_t0 <= phiMax){
                flag[0] = true;
            }
        }
    }
    if(t1 >= ray.tNear && t1 <= ray.tFar){
        if(hit_t1[2] >= 0 && hit_t1[2] <= height){
            if(phi_t1 <= phiMax){
                flag[1] = true;
            }
        }
    }

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

void Cone::fillIntersection(float distance, int primID, float u, float v, Intersection *intersection) const {
    /// ----------------------------------------------------
    //* todo 填充圆锥相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    //* 2.位置信息可以根据uv计算出，同样需要变换
    //* Write your code here.
    /// ----------------------------------------------------

    float phi = u * phiMax;
    float theta = v * height;
    Point3f position = Point3f(0, 0, theta) + (radius - (theta/height)*radius) * Vector3f(cos(phi), sin(phi), 0);
    intersection->position = transform.toWorld(position);
    Point3f K = Point3f(0, 0, height - (height - theta)/(cosTheta * cosTheta));
    Vector3f n = normalize(position - K);
    intersection->normal = transform.toWorld(n);
    
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

void Cone::uniformSampleOnSurface(Vector2f sample, Intersection *result, float *pdf) const {

}

Cone::Cone(const Json &json) : Shape(json) {
    radius = fetchOptional(json, "radius", 1.f);
    height = fetchOptional(json, "height", 1.f);
    phiMax = fetchOptional(json, "phi_max", 2 * PI);
    float tanTheta = radius / height;
    cosTheta = sqrt(1/(1+tanTheta * tanTheta));
    //theta = fetchOptional(json,)
    AABB localAABB = AABB(Point3f(-radius,-radius,0),Point3f(radius,radius,height));
    boundingBox = transform.toWorld(localAABB);
    boundingBox = AABB(Point3f(-100,-100,-100),Point3f(100,100,100));
}

REGISTER_CLASS(Cone, "cone")
