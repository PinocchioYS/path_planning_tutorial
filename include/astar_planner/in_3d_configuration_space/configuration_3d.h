#ifndef PATH_PLANNING_TUTORIAL_CONFIGURATION_H
#define PATH_PLANNING_TUTORIAL_CONFIGURATION_H

#include <quadmap/quadmap.h>

#define TWO_PI (2.0 * M_PI)

/*
 * Configuration consisting of ( X , Y , R )
 */
struct Configuration3D {
    Configuration3D () { data[0] = data[1] = data[2] = 0.0; }

    Configuration3D (const Configuration3D& _other) {
        data[0] = _other(0);
        data[1] = _other(1);
        data[2] = _other(2);
    }

    Configuration3D (float _x, float _y, float _r) {
        data[0] = _x;
        data[1] = _y;
        data[2] = _r;
    }

    inline Configuration3D& operator= (const Configuration3D& _other)  {
        data[0] = _other(0);
        data[1] = _other(1);
        data[2] = _other(2);
        return *this;
    }

    inline const float& operator() (unsigned int i) const
    {
        return data[i];
    }
    inline float& operator() (unsigned int i)
    {
        return data[i];
    }

    inline float& x()
    {
        return operator()(0);
    }

    inline float& y()
    {
        return operator()(1);
    }

    inline float& r()
    {
        return operator()(2);
    }

    inline const float& x() const
    {
        return operator()(0);
    }

    inline const float& y() const
    {
        return operator()(1);
    }

    inline const float& r() const
    {
        return operator()(2);
    }

    float data[3];
};

struct Configuration3DKey {
    Configuration3DKey () {}

    Configuration3DKey (quadmap::key_type a, quadmap::key_type b, quadmap::key_type c){
        k[0] = a;
        k[1] = b;
        k[2] = c;
    }

    Configuration3DKey(const Configuration3DKey& _other){
        k[0] = _other.k[0];
        k[1] = _other.k[1];
        k[2] = _other.k[2];
    }

    bool operator== (const Configuration3DKey& _other) const {
        return ((k[0] == _other[0]) && (k[1] == _other[1]) && (k[2] == _other[2]));
    }

    bool operator!= (const Configuration3DKey& _other) const {
        return( (k[0] != _other[0]) || (k[1] != _other[1]) || (k[2] != _other[2]) );
    }

    Configuration3DKey& operator=(const Configuration3DKey& _other){
        k[0] = _other.k[0]; k[1] = _other.k[1]; k[2] = _other.k[2];
        return *this;
    }

    const quadmap::key_type& operator[] (unsigned int i) const {
        return k[i];
    }

    quadmap::key_type& operator[] (unsigned int i) {
        return k[i];
    }

    quadmap::key_type k[3];

    /// Provides a hash function on Keys
    struct KeyHash{
        size_t operator()(const Configuration3DKey& _key) const{
            // a simple hashing function
            // explicit casts to size_t to operate on the complete range
            // constants will be promoted according to C++ standard
            return static_cast<size_t>(_key.k[0])
                   + 1447*static_cast<size_t>(_key.k[1])
                   + 345637*static_cast<size_t>(_key.k[2]);
        }
    };
};

struct Configuration3DConverter {
    // Constructor
    explicit Configuration3DConverter(double _xy_resolution, double _r_resolution)
            : XY_RESOLUTION(_xy_resolution), XY_RESOLUTION_FACTOR(1.0/_xy_resolution),
              R_RESOLUTION(_r_resolution), R_RESOLUTION_FACTOR(1.0/_r_resolution) { }

    // Conversion: key <--> configuration
    inline quadmap::key_type coordToKey(double _coordinate) const { return (quadmap::key_type)(((int)floor(XY_RESOLUTION_FACTOR * _coordinate)) + GRID_MAX_VAL); }
    inline quadmap::key_type rotationToKey(double _rotation) const {
        while(_rotation > M_PI)     _rotation -= TWO_PI;
        while(_rotation <= -M_PI)   _rotation += TWO_PI;
        return (quadmap::key_type)(((int)floor(R_RESOLUTION_FACTOR * _rotation + 0.5)) + GRID_MAX_VAL);
    }
    inline Configuration3DKey confToKey(const Configuration3D& _conf) const { return Configuration3DKey(coordToKey(_conf.x()), coordToKey(_conf.y()), rotationToKey(_conf.r())); }
    inline Configuration3DKey confToKey(const double _x, const double _y, const double _r) const { return Configuration3DKey(coordToKey(_x), coordToKey(_y), rotationToKey(_r)); }

    inline double keyToCoord(quadmap::key_type _key) const { return (double(_key - GRID_MAX_VAL) + 0.5) * XY_RESOLUTION; }
    inline double keyToRotation(quadmap::key_type _key) const {
        double rotation = (double(_key - GRID_MAX_VAL)) * R_RESOLUTION;
        while(rotation > M_PI)       rotation -= TWO_PI;
        while(rotation <= -M_PI)     rotation += TWO_PI;
        return rotation;
    }
    inline Configuration3D keyToConf(const Configuration3DKey& _key) const { return Configuration3D((float)keyToCoord(_key[0]), (float)keyToCoord(_key[1]), (float)keyToRotation(_key[2])); }

    const double XY_RESOLUTION;
    const double XY_RESOLUTION_FACTOR;
    const double R_RESOLUTION;
    const double R_RESOLUTION_FACTOR;
    const int    GRID_MAX_VAL = 32768;
};

#endif //PATH_PLANNING_TUTORIAL_CONFIGURATION_H
