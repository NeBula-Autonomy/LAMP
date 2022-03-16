#ifndef __VEC3_H__
#define __VEC3_H__

struct Vec3 {
  Vec3() : x(0), y(0), z(0) {}
  
  Vec3(double x, double y, double z)
    : x(x), y(y), z(z) {}
  union {
    struct {
      double x;
      double y;
      double z;
    };
    float v[3];
  };
  
  Vec3& operator+=(Vec3 const &v2) {
    x += v2.x;
    y += v2.y;
    z += v2.z;
    return *this;
  }

  Vec3 operator-(Vec3 const &v2) {
    return Vec3(x - v2.x,
                y - v2.y,
                z - v2.z);
  }
  
  Vec3 operator+(Vec3 const &v2) {
    return Vec3(x + v2.x,
                y + v2.y,
                z + v2.z);
  }
  
  Vec3 operator/(double const &d) {
    return Vec3(x/d, y/d, z/d);
  }
  Vec3 operator*(double const &d) {
    return Vec3(x*d, y*d, z*d);
  }

  static Vec3 min(Vec3 v1, Vec3 v2) {
    return Vec3(std::min(v1.x, v2.x),
                std::min(v1.y, v2.y),
                std::min(v1.z, v2.z));
  }

  static Vec3 max(Vec3 v1, Vec3 v2) {
    return Vec3(std::max(v1.x, v2.x),
                std::max(v1.y, v2.y),
                std::max(v1.z, v2.z));
  }

  static Vec3 abs(Vec3 v1) {
    return Vec3(fabs(v1.x),
                fabs(v1.y),
                fabs(v1.z));
  }
};

#endif // __VEC3_H__
