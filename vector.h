#pragma once 

#define DEG2RAD 0.01745329251
#define RAD2DEG 57.2957795131


struct Vec {

    float x = 0;
    float y = 0;
    float z = 0;
    Vec(float a, float b, float c) : x(a), y(b), z(c) {}
    Vec(float a, float b) : x(a), y(b), z(0) {}
    ~Vec() = default;
    Vec() = default;

    // Arduino Override
    operator String() const {
        return String(x) + "," + String(y) + "," + String(z);
    }

    // OS stream overload
    // friend std::ostream& operator<<(std::ostream& o, const Vec<T> &v){
    //	return o << v.x << ", " << v.y << ", " << v.z << std::endl;
    //  }
    float operator[](int i) {
        if (i == 0) {
            return x;
        } else if (i == 1) {
            return y;
        } else if (i == 2) {
            return z;
        }
    }

    float get(int i) {
        if (i == 0) {
            return x;
        } else if (i == 1) {
            return y;
        } else if (i == 2) {
            return z;
        }
    }
    // Basic Types
    template <typename S>
    Vec operator*(S f) {
        return Vec(x * f, y * f, z * f);
    }
    template <typename S>
    Vec operator/(S f) {
        return Vec(x / f, y / f, z / f);
    }
    template <typename S>
    Vec operator+(S f) {
        return Vec(x + f, y + f, z + f);
    }
    template <typename S>
    Vec operator-(S f) {
        return Vec(x - f, y - f, z - f);
    }

    template <typename S>
    void operator*=(S f) {
        *this = *(this) * f;
    }
    template <typename S>
    void operator/=(S f) {
        *this = *(this) / f;
    }
    // Vector to Vector operations
    Vec operator-(const Vec &v) { return Vec(x - v.x, y - v.y, z - v.z); }
    Vec operator+=(const Vec &v) { return Vec(x += v.x, y += v.y, z += v.z); }
    // element wise division
    Vec operator/(const Vec &v) {
        Vec r;
        r.x = x / v.x;
        r.y = y / v.y;
        r.z = z / v.z;
        return r;
    }
    // Cross product
    Vec operator*(const Vec &v) {
        Vec r;
        r.x = (y * v.z - z * v.y);
        r.y = (z * v.x - x * v.z);
        r.z = (x * v.y - y * v.x);
        return r;
    }
    // Vector to Vector Dot Product
    float dot(const Vec &v) { return x * v.x + y * v.y + z * v.z; }

    float magnitude() { return sqrt(x * x + y * y + z * z); }

    void normalize() { *this = *this / magnitude(); }

    void toDegrees() { *this = (*this) * (RAD2DEG); }

    void toRadians() { *this = (*this) * (DEG2RAD); }
    // bool functions
    template <typename S>
    bool operator<(const S f) {
        return (x < f || y < f || z < f);
    }
    template <typename S>
    bool operator>(const S f) {
        return (x > f || y > f || z > f);
    }
    // BLA Matrix conversions
    template <typename S>
    Vec fromMat(const S &mat) {
        return Vec(mat(0), mat(1), mat(2));
    }
    template <typename S>
    S toMat(const S &mat) {
        mat(0) = x;
        mat(1) = y;
        mat(2) = z;
        return mat;
    }
};