#ifndef BASEMENTS_MATRIX4_H
#define BASEMENTS_MATRIX4_H

#include <cmath>
#include "vec3.h"
#include "matrix3.h"
#include "quaternion.h"

namespace basements {
namespace math {

/**
 * @brief 4x4 matrix for 3D transformations
 * 
 * Memory layout: Row-major
 * [m00 m01 m02 m03]
 * [m10 m11 m12 m13]
 * [m20 m21 m22 m23]
 * [m30 m31 m32 m33]
 * 
 * Used for:
 * - Affine transformations (rotation + translation + scale)
 * - Projection matrices
 * - View matrices
 */
struct alignas(64) Matrix4 {
    float m[4][4];
    
    // ============================================================
    // Constructors
    // ============================================================
    
    Matrix4() {
        // Identity
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    }
    
    Matrix4(float m00, float m01, float m02, float m03,
            float m10, float m11, float m12, float m13,
            float m20, float m21, float m22, float m23,
            float m30, float m31, float m32, float m33) {
        m[0][0]=m00; m[0][1]=m01; m[0][2]=m02; m[0][3]=m03;
        m[1][0]=m10; m[1][1]=m11; m[1][2]=m12; m[1][3]=m13;
        m[2][0]=m20; m[2][1]=m21; m[2][2]=m22; m[2][3]=m23;
        m[3][0]=m30; m[3][1]=m31; m[3][2]=m32; m[3][3]=m33;
    }
    
    /// Construct from Matrix3 (rotation) and Vec3 (translation)
    static Matrix4 from_rotation_translation(const Matrix3& rot, const Vec3& trans) {
        return Matrix4(
            rot[0][0], rot[0][1], rot[0][2], trans.x,
            rot[1][0], rot[1][1], rot[1][2], trans.y,
            rot[2][0], rot[2][1], rot[2][2], trans.z,
            0.0f, 0.0f, 0.0f, 1.0f
        );
    }
    
    /// Construct from Quaternion and translation
    static Matrix4 from_quaternion_translation(const Quaternion& q, const Vec3& trans) {
        Matrix3 rot = Matrix3::from_quaternion(q);
        return from_rotation_translation(rot, trans);
    }
    
    /// Translation matrix
    static Matrix4 translation(const Vec3& t) {
        return Matrix4(
            1, 0, 0, t.x,
            0, 1, 0, t.y,
            0, 0, 1, t.z,
            0, 0, 0, 1
        );
    }
    
    /// Scale matrix
    static Matrix4 scale(float sx, float sy, float sz) {
        return Matrix4(
            sx, 0, 0, 0,
            0, sy, 0, 0,
            0, 0, sz, 0,
            0, 0, 0, 1
        );
    }
    
    static Matrix4 scale(const Vec3& s) {
        return scale(s.x, s.y, s.z);
    }
    
    /// Perspective projection
    static Matrix4 perspective(float fov_y, float aspect, float near, float far) {
        float tan_half_fov = std::tan(fov_y / 2.0f);
        
        return Matrix4(
            1.0f/(aspect*tan_half_fov), 0, 0, 0,
            0, 1.0f/tan_half_fov, 0, 0,
            0, 0, -(far+near)/(far-near), -2.0f*far*near/(far-near),
            0, 0, -1, 0
        );
    }
    
    /// Orthographic projection
    static Matrix4 ortho(float left, float right, float bottom, float top, float near, float far) {
        return Matrix4(
            2.0f/(right-left), 0, 0, -(right+left)/(right-left),
            0, 2.0f/(top-bottom), 0, -(top+bottom)/(top-bottom),
            0, 0, -2.0f/(far-near), -(far+near)/(far-near),
            0, 0, 0, 1
        );
    }
    
    /// Look-at view matrix
    static Matrix4 look_at(const Vec3& eye, const Vec3& center, const Vec3& up) {
        Vec3 f = (center - eye).normalized();
        Vec3 r = f.cross(up).normalized();
        Vec3 u = r.cross(f);
        
        return Matrix4(
            r.x, r.y, r.z, -r.dot(eye),
            u.x, u.y, u.z, -u.dot(eye),
            -f.x, -f.y, -f.z, f.dot(eye),
            0, 0, 0, 1
        );
    }
    
    // ============================================================
    // Element Access
    // ============================================================
    
    float* operator[](int row) { return m[row]; }
    const float* operator[](int row) const { return m[row]; }
    
    // ============================================================
    // Matrix Operations
    // ============================================================
    
    Matrix4 operator*(const Matrix4& other) const {
        Matrix4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    result.m[i][j] += m[i][k] * other.m[k][j];
                }
            }
        }
        return result;
    }
    
    /// Transform point (w=1)
    Vec3 transform_point(const Vec3& p) const {
        float x = m[0][0]*p.x + m[0][1]*p.y + m[0][2]*p.z + m[0][3];
        float y = m[1][0]*p.x + m[1][1]*p.y + m[1][2]*p.z + m[1][3];
        float z = m[2][0]*p.x + m[2][1]*p.y + m[2][2]*p.z + m[2][3];
        float w = m[3][0]*p.x + m[3][1]*p.y + m[3][2]*p.z + m[3][3];
        
        if (std::abs(w) > EPSILON) {
            return Vec3(x/w, y/w, z/w);
        }
        return Vec3(x, y, z);
    }
    
    /// Transform direction (w=0)
    Vec3 transform_direction(const Vec3& d) const {
        return Vec3(
            m[0][0]*d.x + m[0][1]*d.y + m[0][2]*d.z,
            m[1][0]*d.x + m[1][1]*d.y + m[1][2]*d.z,
            m[2][0]*d.x + m[2][1]*d.y + m[2][2]*d.z
        );
    }
    
    /// Transpose
    Matrix4 transposed() const {
        Matrix4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = m[j][i];
            }
        }
        return result;
    }
    
    /// Determinant
    float determinant() const {
        // Cofactor expansion along first row
        float det = 0.0f;
        for (int j = 0; j < 4; ++j) {
            det += m[0][j] * cofactor(0, j);
        }
        return det;
    }
    
    /// Inverse (using adjugate method)
    Matrix4 inversed() const {
        float det = determinant();
        if (std::abs(det) < EPSILON) {
            return Matrix4();  // Return identity
        }
        
        Matrix4 result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[j][i] = cofactor(i, j) / det;  // Transposed
            }
        }
        return result;
    }
    
    /// Extract rotation matrix
    Matrix3 rotation() const {
        return Matrix3(
            m[0][0], m[0][1], m[0][2],
            m[1][0], m[1][1], m[1][2],
            m[2][0], m[2][1], m[2][2]
        );
    }
    
    /// Extract translation
    Vec3 translation() const {
        return Vec3(m[0][3], m[1][3], m[2][3]);
    }
    
    // ============================================================
    // Static Factory Methods
    // ============================================================
    
    static Matrix4 identity() { return Matrix4(); }
    
private:
    float cofactor(int row, int col) const {
        // Compute 3x3 minor determinant
        float minor[3][3];
        int mi = 0;
        for (int i = 0; i < 4; ++i) {
            if (i == row) continue;
            int mj = 0;
            for (int j = 0; j < 4; ++j) {
                if (j == col) continue;
                minor[mi][mj] = m[i][j];
                ++mj;
            }
            ++mi;
        }
        
        float det = minor[0][0] * (minor[1][1]*minor[2][2] - minor[1][2]*minor[2][1])
                  - minor[0][1] * (minor[1][0]*minor[2][2] - minor[1][2]*minor[2][0])
                  + minor[0][2] * (minor[1][0]*minor[2][1] - minor[1][1]*minor[2][0]);
        
        return ((row + col) % 2 == 0) ? det : -det;
    }
};

} // namespace math
} // namespace basements

#endif // BASEMENTS_MATRIX4_H
