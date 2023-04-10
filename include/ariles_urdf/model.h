#pragma once

#include <ariles2/ariles.h>
#include <ariles2/extra.h>
#include <ariles2/adapters/basic.h>
#include <ariles2/adapters/std_vector.h>
#include <ariles2/adapters/eigen.h>


#include "color.h"
#include "pose.h"
#include "joint.h"
#include "link.h"


namespace ariles_urdf
{
    class Model : public ariles2::RelaxedSloppyBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, name, std::string)                                                                         \
    ARILES2_TYPED_ENTRY_(v, link, std::vector<Link>)                                                                   \
    ARILES2_TYPED_ENTRY_(v, joint, std::vector<Joint>)                                                                 \
    ARILES2_TYPED_ENTRY_(v, material, std::vector<Material>)
#define ARILES2_DEFAULT_ID "robot"
#include ARILES2_INITIALIZE


    public:
        std::vector<ariles_urdf::Link::Ptr> links_;
        std::vector<ariles_urdf::Joint::Ptr> joints_;


    public:
        Model()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }
    };
}  // namespace ariles_urdf
