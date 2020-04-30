
#ifndef CGL_Vector3R_H
#define CGL_Vector3R_H

#include "real.h"
#include <cmath>

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
  Vector3R() : x( 0.0 ), y( 0.0 ), z( 0.0 ) { }

  /**
   * Constructor.
   * Initializes to vector (x,y,z).
   */
  Vector3R( REAL x, REAL y, REAL z) : x( x ), y( y ), z( z ) { }

  /**
   * Constructor.
   * Initializes to vector (c,c,c)
   */
  Vector3R( REAL c ) : x( c ), y( c ), z( c ) { }

  /**
   * Constructor.
   * Initializes from existing vector
   */
  Vector3R( const Vector3R& v ) : x( v.x ), y( v.y ), z( v.z ) { }

  // returns reference to the specified component (0-based indexing: x, y, z)
  inline REAL& operator[] ( const int& index ) {
    return ( &x )[ index ];
  }

  // returns const reference to the specified component (0-based indexing: x, y, z)
  inline const REAL& operator[] ( const int& index ) const {
    return ( &x )[ index ];
  }

  inline bool operator==( const Vector3R& v) const {
    return v.x == x && v.y == y && v.z == z;
  }

  // negation
  inline Vector3R operator-( void ) const {
    return Vector3R( -x, -y, -z );
  }

  // addition
  inline Vector3R operator+( const Vector3R& v ) const {
    return Vector3R( x + v.x, y + v.y, z + v.z );
  }

  // subtraction
  inline Vector3R operator-( const Vector3R& v ) const {
    return Vector3R( x - v.x, y - v.y, z - v.z );
  }

  // right scalar multiplication
  inline Vector3R operator*( const REAL& c ) const {
    return Vector3R( x * c, y * c, z * c );
  }

  // scalar division
  inline Vector3R operator/( const REAL& c ) const {
    const REAL rc = 1.0/c;
    return Vector3R( rc * x, rc * y, rc * z );
  }

  // addition / assignment
  inline void operator+=( const Vector3R& v ) {
    x += v.x; y += v.y; z += v.z;
  }

  // subtraction / assignment
  inline void operator-=( const Vector3R& v ) {
    x -= v.x; y -= v.y; z -= v.z;
  }

  // scalar multiplication / assignment
  inline void operator*=( const REAL& c ) {
    x *= c; y *= c; z *= c;
  }

  // scalar division / assignment
  inline void operator/=( const REAL& c ) {
    (*this) *= ( 1./c );
  }

  /**
   * Returns Euclidean length.
   */
  inline REAL norm( void ) const {
    return sqrt( x*x + y*y + z*z );
  }

  /**
   * Returns Euclidean length squared.
   */
  inline REAL norm2( void ) const {
    return x*x + y*y + z*z;
  }

  /**
   * Returns unit vector.
   */
  inline Vector3R unit( void ) const {
    REAL rNorm = 1. / sqrt( x*x + y*y + z*z );
    return Vector3R( rNorm*x, rNorm*y, rNorm*z );
  }

  /**
   * Divides by Euclidean length.
   */
  inline void normalize( void ) {
    (*this) /= norm();
  }

}; // class Vector3R

// left scalar multiplication
inline Vector3R operator* ( const REAL& c, const Vector3R& v ) {
  return Vector3R( c * v.x, c * v.y, c * v.z );
}

// dot product (a.k.a. inner or scalar product)
inline REAL dot( const Vector3R& u, const Vector3R& v ) {
  return u.x*v.x + u.y*v.y + u.z*v.z ;
}

// cross product
inline Vector3R cross( const Vector3R& u, const Vector3R& v ) {
  return Vector3R( u.y*v.z - u.z*v.y,
                   u.z*v.x - u.x*v.z,
                   u.x*v.y - u.y*v.x );
}

// prints components
std::ostream& operator<<( std::ostream& os, const Vector3R& v );

#endif // CGL_Vector3R_H