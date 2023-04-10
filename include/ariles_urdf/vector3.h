#pragma once

#include <string>

#include "utils.h"


namespace ariles_urdf
{
    class Vector3 : public ariles2::DefaultBase, public VectorValue<Eigen::Vector3d>
    {
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, xyz, std::string)
#include ARILES2_INITIALIZE


    public:
        Vector3()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            setZero();
        }


        void arilesVisit(const ariles2::Finalize & /*visitor*/, const ariles2::Finalize::Parameters & /*param*/)
        {
            fromString(xyz_);
        }


        void arilesVisit(const ariles2::PreWrite & /*visitor*/, const ariles2::PreWrite::Parameters & /*param*/)
        {
            toString(xyz_);
        }
    };
}  // namespace ariles_urdf
