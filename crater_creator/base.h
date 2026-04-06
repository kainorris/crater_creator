#ifndef INCLUDE_CRATER_CREATOR_BASE_H_
#define INCLUDE_CRATER_CREATOR_BASE_H_
struct Point2D {
  float x;
  float y;
};
struct Point3D {
  float x, y, z;
  Point3D operator-(Point3D other) {
    return Point3D{
        .x = x - other.x,
        .y = y - other.y,
        .z = z - other.z,
    };
  }
};

#endif // INCLUDE_CRATER_CREATOR_BASE_H_
