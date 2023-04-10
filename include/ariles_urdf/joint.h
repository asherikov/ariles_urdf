#pragma once

#include <string>
#include <algorithm>
#include <cctype>


namespace ariles_urdf
{
    class LinkName : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, link, std::string)
#include ARILES2_INITIALIZE

    public:
        void arilesVisit(const ariles2::Finalize & /*visitor*/, const ariles2::Finalize::Parameters & /*param*/)
        {
            CPPUT_ASSERT(not link_.empty(), "Link name is empty.");
        }
    };


    class Dynamics : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, damping, double)                                                                           \
    ARILES2_TYPED_ENTRY_(v, friction, double)
#include ARILES2_INITIALIZE

    public:
        Dynamics()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            damping_ = 0.0;
            friction_ = 0.0;
        }

        void arilesVisit(const ariles2::Finalize & /*visitor*/, const ariles2::Finalize::Parameters & /*param*/)
        {
            CPPUT_ASSERT(
                    damping_ >= 0 and damping_ < std::numeric_limits<double>::infinity(),
                    "Damping must not be negative and finite.");
            CPPUT_ASSERT(
                    friction_ >= 0 and friction_ < std::numeric_limits<double>::infinity(),
                    "Friction must not be negative and finite.");
        }
    };


    class Limits : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, lower, double)                                                                             \
    ARILES2_TYPED_ENTRY_(v, upper, double)                                                                             \
    ARILES2_TYPED_ENTRY_(v, effort, double)                                                                            \
    ARILES2_TYPED_ENTRY_(v, velocity, double)
#include ARILES2_INITIALIZE


    public:
        Limits()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            lower_ = 0.0;
            upper_ = 0.0;

            effort_ = std::numeric_limits<double>::quiet_NaN();
            velocity_ = std::numeric_limits<double>::quiet_NaN();
        }

        void arilesVisit(const ariles2::Finalize & /*visitor*/, const ariles2::Finalize::Parameters & /*param*/)
        {
            CPPUT_ASSERT(
                    effort_ >= 0 and effort_ < std::numeric_limits<double>::infinity(),
                    "effort must not be negative and finite.");
            CPPUT_ASSERT(
                    velocity_ >= 0 and velocity_ < std::numeric_limits<double>::infinity(),
                    "velocity must not be negative and finite.");
            CPPUT_ASSERT(lower_ <= upper_, "Inconsistent position limits.");
        }
    };


    class Mimic : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, offset, double)                                                                            \
    ARILES2_TYPED_ENTRY_(v, multiplier, double)                                                                        \
    ARILES2_TYPED_ENTRY_(v, joint, std::string)
#include ARILES2_INITIALIZE


    public:
        Mimic()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            multiplier_ = 1.0;
            offset_ = 0.0;
        }

        void arilesVisit(const ariles2::Finalize & /*visitor*/, const ariles2::Finalize::Parameters & /*param*/)
        {
            CPPUT_ASSERT(not joint_.empty(), "Mimicked joint name must not be empty.");
        }
    };


    class Axis : public Vector3
    {
#define ARILES2_ENTRIES(v) ARILES2_PARENT(v, Vector3)
#include ARILES2_INITIALIZE


    public:
        Axis()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            vector_ = { 1.0, 0.0, 0.0 };
        }

        void arilesVisit(const ariles2::Finalize & /*visitor*/, const ariles2::Finalize::Parameters & /*param*/)
        {
            CPPUT_ASSERT(std::abs(1.0 - vector_.norm()) < 1e-8, "Axis must be a unit vector");
        }
    };


    class Calibration : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, rising, double)                                                                            \
    ARILES2_TYPED_ENTRY_(v, falling, double)
#include ARILES2_INITIALIZE


    public:
        Calibration()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }
    };


    class SafetyController : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, soft_lower_limit, double)                                                                  \
    ARILES2_TYPED_ENTRY_(v, soft_upper_limit, double)                                                                  \
    ARILES2_TYPED_ENTRY_(v, k_position, double)                                                                        \
    ARILES2_TYPED_ENTRY_(v, k_velocity, double)
#include ARILES2_INITIALIZE


    public:
        SafetyController()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            soft_lower_limit_ = 0.0;
            soft_upper_limit_ = 0.0;
            k_position_ = 0.0;
            k_velocity_ = std::numeric_limits<double>::quiet_NaN();
        }

        void arilesVisit(const ariles2::Finalize & /*visitor*/, const ariles2::Finalize::Parameters & /*param*/)
        {
            CPPUT_ASSERT(soft_lower_limit_ <= soft_upper_limit_, "Inconsistent soft limits.");
        }
    };


    /*
     * http://wiki.ros.org/urdf/XML/joint
     *
    <joint name="my_joint" type="floating">
      <origin xyz="0 0 1" rpy="0 0 3.1416"/>
      <parent link="link1"/>
      <child link="link2"/>

      <calibration rising="0.0"/>
      <dynamics damping="0.0" friction="0.0"/>
      <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
      <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
    </joint>
    */
    class Joint : public ariles2::DefaultBase, public cpput::PtrAddon<Joint>
    {
    public:
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, name, std::string)                                                                         \
    ARILES2_TYPED_ENTRY_(v, type, std::string)                                                                         \
    ARILES2_TYPED_ENTRY_(v, origin, Pose)                                                                              \
    ARILES2_TYPED_ENTRY_(v, parent, LinkName)                                                                          \
    ARILES2_TYPED_ENTRY_(v, child, LinkName)                                                                           \
    ARILES2_TYPED_ENTRY_(v, dynamics, ariles2::OptionalPointer<std::shared_ptr<Dynamics>>)                             \
    ARILES2_TYPED_ENTRY_(v, limit, ariles2::OptionalPointer<std::shared_ptr<Limits>>)                                  \
    ARILES2_TYPED_ENTRY_(v, axis, Axis)                                                                                \
    ARILES2_TYPED_ENTRY_(v, mimic, ariles2::OptionalPointer<std::shared_ptr<Mimic>>)                                   \
    ARILES2_TYPED_ENTRY_(v, calibration_, ariles2::OptionalPointer<std::shared_ptr<Calibration>>)                      \
    ARILES2_TYPED_ENTRY_(v, safety_controller, ariles2::OptionalPointer<std::shared_ptr<SafetyController>>)
#include ARILES2_INITIALIZE


    public:
        enum class Type
        {
            UNKNOWN,
            REVOLUTE,
            CONTINUOUS,
            PRISMATIC,
            FLOATING,
            PLANAR,
            FIXED
        } type_id_;


    public:
        Joint()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        void arilesVisit(const ariles2::Finalize &visitor, const ariles2::Finalize::Parameters &param)
        {
            CPPUT_ASSERT(not name_.empty(), "Name must not be empty.");

            // normalize type
            std::transform(type_.begin(), type_.end(), type_.begin(), [](unsigned char c) { return std::tolower(c); });

            if (type_ == "planar")
            {
                type_id_ = Type::PLANAR;
            }
            else if (type_ == "floating")
            {
                type_id_ = Type::FLOATING;
            }
            else if (type_ == "revolute")
            {
                type_id_ = Type::REVOLUTE;
            }
            else if (type_ == "continuous")
            {
                type_id_ = Type::CONTINUOUS;
            }
            else if (type_ == "prismatic")
            {
                type_id_ = Type::PRISMATIC;
            }
            else if (type_ == "fixed")
            {
                type_id_ = Type::FIXED;
            }
            else
            {
                CPPUT_THROW("Unable to parse joint type: " + type_);
            }

            switch (type_id_)
            {
                case Type::REVOLUTE:
                case Type::PRISMATIC:
                    CPPUT_ASSERT(not limit_.isNull(), "Revolute and prismatic joints require limits");
                    if (not safety_controller_.isNull())
                    {
                        CPPUT_ASSERT(
                                limit_->lower_ <= safety_controller_->soft_lower_limit_,
                                "Inconsistent safety/normal limits");
                        CPPUT_ASSERT(
                                limit_->upper_ >= safety_controller_->soft_upper_limit_,
                                "Inconsistent safety/normal limits");
                    }
                    break;

                default:
                    CPPUT_ASSERT(limit_.isNull(), "Meaninless joint limits");
                    CPPUT_ASSERT(safety_controller_.isNull(), "Meaningless safety controller limits");
                    break;
            }

            arilesVisit<ariles2::Finalize>(visitor, param);
        }
    };
}  // namespace ariles_urdf
