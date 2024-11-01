#ifndef RAY__H
#define RAY__H

#include <vector>
#include <cmath>
#include <iostream>
#include <limits.h>
#include <float.h>

struct Vec2{
    float x, y;
    Vec2() : x(0), y(0) {}
    Vec2(float x, float y): x(x), y(y) {}

    Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }

    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y - other.y);
    }

    Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }

    float dot(const Vec2& other) const {
        return x * other.x + y * other.y;
    }

    float length() const {
        return std::sqrt(x * x + y* y);
    }
    
    float cross(const Vec2& other) const {
        return x * other.y - y * other.x;
    }

    Vec2 rotate_vector(float theta) {
    float cos_theta = std::cos(theta);
    float sin_theta = std::sin(theta);
    float new_x = x * cos_theta - y * sin_theta;
    float new_y = x * sin_theta + y * cos_theta;
    return Vec2(new_x, new_y);
}


};

struct Vec3 {
    float x, y, z;

    // Default constructor
    Vec3() : x(0), y(0), z(0) {}

    // Parameterized constructor
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    // Subtract another vector
    Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    // Add another vector
    Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    // Scalar multiplication
    Vec3 operator*(float scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    Vec3 operator/(float scalar) const {
        return Vec3(x / scalar, y / scalar, z / scalar);
    }

    // Dot product
    float dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // Length (magnitude) of the vector
    float length() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    // Cross product with another vector
    Vec3 cross(const Vec3& other) const {
        return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    // Rotate around the X-axis
    Vec3 rotateX(float theta) const {
        float cos_theta = std::cos(theta);
        float sin_theta = std::sin(theta);
        return Vec3(
            x,
            y * cos_theta - z * sin_theta,
            y * sin_theta + z * cos_theta
        );
    }

    // Rotate around the Y-axis
    Vec3 rotateY(float theta) const {
        float cos_theta = std::cos(theta);
        float sin_theta = std::sin(theta);
        return Vec3(
            x * cos_theta + z * sin_theta,
            y,
            -x * sin_theta + z * cos_theta
        );
    }

    // Rotate around the Z-axis
    Vec3 rotateZ(float theta) const {
        float cos_theta = std::cos(theta);
        float sin_theta = std::sin(theta);
        return Vec3(
            x * cos_theta - y * sin_theta,
            x * sin_theta + y * cos_theta,
            z
        );
    }

    // Print the vector for debugging
    void print() const {
        std::cout << "Vec3(" << x << ", " << y << ", " << z << ")" << std::endl;
    }

    Vec3 normalize() const {
    float len = length();
    if (len > 0) {
        return *this / len;
    }
    return Vec3(0, 0, 0); // Return zero vector if length is zero
    }
};


struct Ray {
    constexpr Ray (const Vec2& origin, const Vec2& direction) : origin_(origin), direction_(direction) {}
        
    Vec2 point_at_parameter(const float t) const { return this->origin_ + (this->direction_*t);}
    Vec2 origin() const {return this->origin_;}
    Vec2 direction() const {return this->direction_;}

    private:
        const Vec2 origin_;
        const Vec2 direction_;
    
};

struct Ray3D {
    constexpr Ray3D (const Vec3& origin, const Vec3& direction) : origin_(origin), direction_(direction) {}
        
    Vec3 point_at_parameter(const float t) const { return this->origin_ + (this->direction_*t);}
    Vec3 origin() const {return this->origin_;}
    Vec3 direction() const {return this->direction_;}

    private:
        const Vec3 origin_;
        const Vec3 direction_;
    
};

// Define a plane
struct Plane {
    Vec3 normal;
    float d;

    Plane() : normal(Vec3()), d(0) {}
    Plane(const Vec3& n, float d) : normal(n), d(d) {}
};

struct Line {
    Vec3 point;    // A point on the line
    Vec3 direction; // Direction vector of the line

    // Constructor with two points
    Line(const Vec3& p1, const Vec3& p2) : point(p1), direction((p2 - p1).normalize()) {}


    Vec3 intersection(const Line& other) const {
        Vec3 u = direction;
        Vec3 v = other.direction;
        Vec3 w = point - other.point;
        
        float a = u.dot(u);
        float b = u.dot(v);
        float c = v.dot(v);
        float d = u.dot(w);
        float e = v.dot(w);
        float denominator = a * c - b * b;

        if (std::fabs(denominator) < 1e-6f) { // Use an epsilon value
            return Vec3(FLT_MAX, FLT_MAX, FLT_MAX); // Lines are parallel
        }

        float s = (b * e - c * d) / denominator;
        return point + u * s;
    }

    // Calculate the angle of incidence between this line and another line in degrees
    float angleOfIncidence(const Line& other) const {
        float dot_product = direction.dot(other.direction);
        float magnitude_product = direction.length() * other.direction.length();
        float cos_theta = dot_product / magnitude_product;
        
        // Clamp the value to avoid possible numerical issues with acos
        cos_theta = std::max(-1.0f, std::min(1.0f, cos_theta));
        
        float angle_radians = std::acos(cos_theta);
        float angle_degrees = angle_radians * (180.0f / M_PI); // Convert radians to degrees
        
        return angle_degrees;
    }
};

#endif