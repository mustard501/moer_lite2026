#include "ThinLens.h"
#include <cmath>

ThinLensCamera::ThinLensCamera(const Json& json) : PerspectiveCamera(json) {
    lensRadius    = json["lensRadius"].get<float>();
    focalDistance = json["focalDistance"].get<float>();
}

static Point3f cameraSpaceFocusPoint(float filmX, float filmY, float verticalFov,
                                     int filmHeight, float focalDistance) {
    float tanHalfFov = fm::tan(verticalFov * 0.5f);
    float z = -filmHeight * 0.5f / tanHalfFov;
    Vector3f w = normalize(Vector3f{filmX, filmY, z});
    float t = -focalDistance / w[2];
    return Point3f{w[0] * t, w[1] * t, w[2] * t};
}

Ray ThinLensCamera::sampleRay(const CameraSample& sample, Vector2f NDC) const {

    float x = (NDC[0] - 0.5f) * film->size[0] + sample.xy[0],
          y = (0.5f - NDC[1]) * film->size[1] + sample.xy[1];

    Point3f pFocus =
        cameraSpaceFocusPoint(x, y, verticalFov, film->size[1], focalDistance);

    float u1 = sample.lens[0], u2 = sample.lens[1];
    float r = lensRadius * std::sqrt(u1);
    float theta = 2.f * PI * u2;
    float lx = r * fm::cos(theta);
    float ly = r * fm::sin(theta);
    Point3f lensO{lx, ly, 0.f};

    Vector3f dirCam = normalize(pFocus - lensO);
    Point3f originW = transform.toWorld(lensO);
    Vector3f dirW = normalize(transform.toWorld(dirCam));

    return Ray(originW, dirW, tNear, tFar, timeStart);
}

Ray ThinLensCamera::sampleRayDifferentials(const CameraSample& sample, Vector2f NDC) const {
    float x = (NDC[0] - 0.5f) * film->size[0] + sample.xy[0],
          y = (0.5f - NDC[1]) * film->size[1] + sample.xy[1];

    Point3f pFocus =
        cameraSpaceFocusPoint(x, y, verticalFov, film->size[1], focalDistance);
    Point3f pFocusX =
        cameraSpaceFocusPoint(x + 1.f, y, verticalFov, film->size[1], focalDistance);
    Point3f pFocusY =
        cameraSpaceFocusPoint(x, y + 1.f, verticalFov, film->size[1], focalDistance);

    float u1 = sample.lens[0], u2 = sample.lens[1];
    float r = lensRadius * std::sqrt(u1);
    float theta = 2.f * PI * u2;
    float lx = r * fm::cos(theta);
    float ly = r * fm::sin(theta);
    Point3f lensO{lx, ly, 0.f};

    Vector3f dirCam = normalize(pFocus - lensO);
    Vector3f dirCamX = normalize(pFocusX - lensO);
    Vector3f dirCamY = normalize(pFocusY - lensO);

    Point3f originW = transform.toWorld(lensO);
    Vector3f dirW = normalize(transform.toWorld(dirCam));
    Vector3f dirWX = normalize(transform.toWorld(dirCamX));
    Vector3f dirWY = normalize(transform.toWorld(dirCamY));

    Ray ret(originW, dirW, tNear, tFar, timeStart);
    ret.hasDifferentials = true;
    ret.directionX = dirWX;
    ret.directionY = dirWY;
    ret.originX = ret.originY = originW;
    return ret;
}

REGISTER_CLASS(ThinLensCamera, "thinlens")