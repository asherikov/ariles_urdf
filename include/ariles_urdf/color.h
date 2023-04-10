#pragma once

#include "utils.h"


namespace ariles_urdf
{
    class Color : public ariles2::DefaultBase, public VectorValue<Eigen::Matrix<float, 4, 1>>
    {
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, rgba, std::string)
#include ARILES2_INITIALIZE


    public:
        Color()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        void arilesVisit(const ariles2::Finalize & /*visitor*/, const ariles2::Finalize::Parameters & /*param*/)
        {
            fromString(rgba_);
            CPPUT_ASSERT(is0to1(), "Color is outside the valid range [0, 1]");
        }


        void arilesVisit(const ariles2::PreWrite & /*visitor*/, const ariles2::PreWrite::Parameters & /*param*/)
        {
            toString(rgba_);
        }
    };
}  // namespace ariles_urdf
