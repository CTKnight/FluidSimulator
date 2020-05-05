
#ifndef CGL_Vector3R_H
#define CGL_Vector3R_H

#include "real.h"
#include <cmath>
#include <vector>
#include "cuda/cudaUtils.h"

using namespace std;

/**
 * Defines 3D vectors of REAL version from CGL Vector3D
 */
class Vector3R {
 public:

  // components
  REAL x, y, z;

  /**
   * Constructor.
   * Initializes tp vector (0,0,0).
   */
  CUDA_CALLABLE Vector3R() : x( 0.0 ), y( 0.0 ), z( 0.0 ) { }

  /**
   * Constructor.
   * Initializes to vector (x,y,z).
   */
  CUDA_CALLABLE Vector3R( REAL x, REAL y, REAL z) : x( x ), y( y ), z( z ) { }

  /**
   * Constructor.
   * Initializes to vector (c,c,c)
   */
  CUDA_CALLABLE Vector3R( REAL c ) : x( c ), y( c ), z( c ) { }

  /**
   * Constructor.
   * Initializes from existing vector
   */
  CUDA_CALLABLE Vector3R( const Vector3R& v ) : x( v.x ), y( v.y ), z( v.z ) { }

  // returns reference to the specified component (0-based indexing: x, y, z)
  CUDA_CALLABLE inline REAL& operator[] ( const int& index ) {
    return ( &x )[ index ];
  }

  // returns const reference to the specified component (0-based indexing: x, y, z)
  CUDA_CALLABLE inline const REAL& operator[] ( const int& index ) const {
    return ( &x )[ index ];
  }

  CUDA_CALLABLE inline bool operator==( const Vector3R& v) const {
    return v.x == x && v.y == y && v.z == z;
  }

  // negation
  CUDA_CALLABLE inline Vector3R operator-( void ) const {
    return Vector3R( -x, -y, -z );
  }

  // addition
  CUDA_CALLABLE inline Vector3R operator+( const Vector3R& v ) const {
    return Vector3R( x + v.x, y + v.y, z + v.z );
  }

  // subtraction
  CUDA_CALLABLE inline Vector3R operator-( const Vector3R& v ) const {
    return Vector3R( x - v.x, y - v.y, z - v.z );
  }

  // right scalar multiplication
  CUDA_CALLABLE inline Vector3R operator*( const REAL& c ) const {
    return Vector3R( x * c, y * c, z * c );
  }

  // scalar division
  CUDA_CALLABLE inline Vector3R operator/( const REAL& c ) const {
    const REAL rc = 1.0/c;
    return Vector3R( rc * x, rc * y, rc * z );
  }

  // addition / assignment
  CUDA_CALLABLE inline void operator+=( const Vector3R& v ) {
    x += v.x; y += v.y; z += v.z;
  }

  // subtraction / assignment
  CUDA_CALLABLE inline void operator-=( const Vector3R& v ) {
    x -= v.x; y -= v.y; z -= v.z;
  }

  // scalar multiplication / assignment
  CUDA_CALLABLE inline void operator*=( const REAL& c ) {
    x *= c; y *= c; z *= c;
  }

  // scalar division / assignment
  CUDA_CALLABLE inline void operator/=( const REAL& c ) {
    (*this) *= ( 1./c );
  }

  /**
   * Returns Euclidean length.
   */
  CUDA_CALLABLE inline REAL norm( void ) const {
    return sqrt( x*x + y*y + z*z );
  }

  /**
   * Returns Euclidean length squared.
   */
  CUDA_CALLABLE inline REAL norm2( void ) const {
    return x*x + y*y + z*z;
  }

  /**
   * Returns unit vector.
   */
  CUDA_CALLABLE inline Vector3R unit( void ) const {
    REAL rNorm = 1. / sqrt( x*x + y*y + z*z );
    return Vector3R( rNorm*x, rNorm*y, rNorm*z );
  }

  /**
   * Divides by Euclidean length.
   */
  CUDA_CALLABLE inline void normalize( void ) {
    (*this) /= norm();
  }

}; // class Vector3R

// left scalar multiplication
CUDA_CALLABLE inline Vector3R operator* ( const REAL& c, const Vector3R& v ) {
  return Vector3R( c * v.x, c * v.y, c * v.z );
}

// dot product (a.k.a. inner or scalar product)
CUDA_CALLABLE inline REAL dot( const Vector3R& u, const Vector3R& v ) {
  return u.x*v.x + u.y*v.y + u.z*v.z ;
}

// cross product
CUDA_CALLABLE inline Vector3R cross( const Vector3R& u, const Vector3R& v ) {
  return Vector3R( u.y*v.z - u.z*v.y,
                   u.z*v.x - u.x*v.z,
                   u.x*v.y - u.y*v.x );
}

// prints components
std::ostream& operator<<( std::ostream& os, const Vector3R& v );


inline Vector3R &REAL3AsVector3R(REAL3 &REAL3) {
  return reinterpret_cast<Vector3R &>(REAL3);
}

inline const Vector3R &REAL3AsVector3R(const REAL3 &REAL3) {
  return reinterpret_cast<const Vector3R &>(REAL3);
}

inline vector<Vector3R> &REAL3AsVector3R(vector<REAL3> &REAL3s) {
  return reinterpret_cast<vector<Vector3R> &>(REAL3s);
}

inline const vector<Vector3R> &REAL3AsVector3R(const vector<REAL3> &REAL3s) {
  return reinterpret_cast<const vector<Vector3R> &>(REAL3s);
}

inline Vector3R *REAL3AsVector3R(REAL3 *REAL3s) {
  return reinterpret_cast<Vector3R *>(REAL3s);
}

inline const Vector3R *REAL3AsVector3R(const REAL3 *REAL3s) {
  return reinterpret_cast<const Vector3R *>(REAL3s);
}

#endif // CGL_Vector3R_H