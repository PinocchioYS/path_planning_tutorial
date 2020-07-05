#ifndef PATH_PLANNING_TUTORIAL_CONFIGURATION_2D_H
#define PATH_PLANNING_TUTORIAL_CONFIGURATION_2D_H

#include <quadmap/quadmap.h>

/*
 * Configuration consisting of ( X , Y )
 */
struct Configuration2D {
    Configuration2D () { data[0] = data[1]; }

    Configuration2D (const Configuration2D& _other) {
        data[0] = _other(0);
        data[1] = _other(1);
    }

    Configuration2D (float _x, float _y) {
        data[0] = _x;
        data[1] = _y;
    }

    inline Configuration2D& operator= (const Configuration2D& _other)  {
        data[0] = _other(0);
        data[1] = _other(1);
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

    inline const float& x() const
    {
        return operator()(0);
    }

    inline const float& y() const
    {
        return operator()(1);
    }

    double distanceTo(const Configuration2D& _other) const {
        return std::sqrt(std::pow(this->x() - _other.x(), 2) + std::pow(this->y() - _other.y(), 2));
    }

    float data[2];
};

struct Configuration2DKey {
    Configuration2DKey () {}

    Configuration2DKey (quadmap::key_type a, quadmap::key_type b){
        k[0] = a;
        k[1] = b;
    }

    Configuration2DKey(const Configuration2DKey& _other){
        k[0] = _other.k[0];
        k[1] = _other.k[1];
    }

    bool operator== (const Configuration2DKey& _other) const {
        return ((k[0] == _other[0]) && (k[1] == _other[1]));
    }

    bool operator!= (const Configuration2DKey& _other) const {
        return( (k[0] != _other[0]) || (k[1] != _other[1]) );
    }

    Configuration2DKey& operator=(const Configuration2DKey& _other){
        k[0] = _other.k[0]; k[1] = _other.k[1];
        return *this;
    }

    const quadmap::key_type& operator[] (unsigned int i) const {
        return k[i];
    }

    quadmap::key_type& operator[] (unsigned int i) {
        return k[i];
    }

    quadmap::key_type k[2];

    /// Provides a hash function on Keys
    struct KeyHash{
        size_t operator()(const Configuration2DKey& _key) const{
            // a simple hashing function
            // explicit casts to size_t to operate on the complete range
            // constants will be promoted according to C++ standard
            return static_cast<size_t>(_key.k[0])
                   + 1447*static_cast<size_t>(_key.k[1]);
        }
    };
};

struct Configuration2DConverter {
    // Constructor
    explicit Configuration2DConverter(double _xy_resolution) : XY_RESOLUTION(_xy_resolution), XY_RESOLUTION_FACTOR(1.0/_xy_resolution) { }

    // Conversion: key <--> configuration
    inline quadmap::key_type coordToKey(double _coordinate) const { return (quadmap::key_type)(((int)floor(XY_RESOLUTION_FACTOR * _coordinate)) + GRID_MAX_VAL); }
    inline Configuration2DKey confToKey(const Configuration2D& _conf) const { return Configuration2DKey(confToKey(_conf.x(), _conf.y())); }
    inline Configuration2DKey confToKey(double _x, double _y) const { return Configuration2DKey(coordToKey(_x), coordToKey(_y)); }

    inline double keyToCoord(quadmap::key_type _key) const { return (double((int)_key - GRID_MAX_VAL) + 0.5) * XY_RESOLUTION; }
    inline Configuration2D keyToConf(const Configuration2DKey& _key) const { return Configuration2D((float)keyToCoord(_key[0]), (float)keyToCoord(_key[1])); }

    const double    XY_RESOLUTION;
    const double    XY_RESOLUTION_FACTOR;
    const int       GRID_MAX_VAL = 32768;
};

#endif //PATH_PLANNING_TUTORIAL_CONFIGURATION_2D_H
