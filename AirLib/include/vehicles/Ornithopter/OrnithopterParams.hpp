// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_OrnithopterParameters_hpp
#define msr_airlib_OrnithopterParameters_hpp

#include "common/Common.hpp"

namespace msr { namespace airlib {

class OrnithopterParams
{
    struct Params
    {
        real_T mass;
        Matrix3x3r inertia;
        real_T restitution = 0.55f; // value of 1 would result in perfectly elastic collisions, 0 would be completely inelastic.
        real_T friction = 0.5f;
    };

//protected: //must override by derived class
//    virtual void setupParams() = 0;

public:
    virtual ~OrnithopterParams() = default;

    virtual void initialize(const AirSimSettings::VehicleSetting* vehicle_setting)
    {
        //setupParams();
    }

    const Params& getParams() const
    {
        return params_;
    }
    Params& getParams()
    {
        return params_;
    }

private:
    Params params_;
};

}} //namespace
#endif