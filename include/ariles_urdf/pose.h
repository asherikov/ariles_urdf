#pragma once

#include "vector3.h"
#include "rotation.h"


namespace ariles_urdf
{
    class Pose : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, position, Vector3)                                                                         \
    ARILES2_TYPED_ENTRY_(v, rotation, Rotation)
#include ARILES2_INITIALIZE

    public:
        Pose()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }
    };
}  // namespace ariles_urdf
