#pragma once

namespace ariles_urdf
{
    class Inertia : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, ixx, double)                                                                               \
    ARILES2_TYPED_ENTRY_(v, ixy, double)                                                                               \
    ARILES2_TYPED_ENTRY_(v, ixz, double)                                                                               \
    ARILES2_TYPED_ENTRY_(v, iyy, double)                                                                               \
    ARILES2_TYPED_ENTRY_(v, iyz, double)                                                                               \
    ARILES2_TYPED_ENTRY_(v, izz, double)
#include ARILES2_INITIALIZE


    public:
        Inertia()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            ixx_ = 0.0;
            ixy_ = 0.0;
            ixz_ = 0.0;
            iyy_ = 0.0;
            iyz_ = 0.0;
            izz_ = 0.0;
        }
    };


    class Mass : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, value, double)
#include ARILES2_INITIALIZE


    public:
        Mass()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            value_ = 0.0;
        }
    };


    class Inertial : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, origin, Pose)                                                                              \
    ARILES2_TYPED_ENTRY_(v, mass, Mass)                                                                                \
    ARILES2_TYPED_ENTRY_(v, inertia, Inertia)
#include ARILES2_INITIALIZE


    public:
        Inertial()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }
    };


    class Material : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, name, std::string)                                                                         \
    ARILES2_TYPED_ENTRY_(v, texture, std::string)                                                                      \
    ARILES2_TYPED_ENTRY_(v, color, ariles2::OptionalPointer<std::shared_ptr<Color>>)
#include ARILES2_INITIALIZE


    public:
        Material()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }
    };


    class Sphere : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, radius, double)
#include ARILES2_INITIALIZE

    public:
        Sphere()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        void arilesVisit(const ariles2::Finalize & /*visitor*/, const ariles2::Finalize::Parameters & /*param*/)
        {
            CPPUT_ASSERT(radius_ >= 0, "Radius cannot be negative.");
        }
    };


    class Box : public ariles2::DefaultBase, public VectorValue<Eigen::Vector3d>
    {
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, size, std::string)
#include ARILES2_INITIALIZE

    public:
        Box()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            setNaN();
        }

        void arilesVisit(const ariles2::Finalize & /*visitor*/, const ariles2::Finalize::Parameters & /*param*/)
        {
            fromString(size_);
            CPPUT_ASSERT(isNonNegative(), "Box size must be nonnegative.");
        }

        void arilesVisit(const ariles2::PreWrite & /*visitor*/, const ariles2::PreWrite::Parameters & /*param*/)
        {
            toString(size_);
        }
    };


    class Cylinder : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, length, double)                                                                            \
    ARILES2_TYPED_ENTRY_(v, radius, double)
#include ARILES2_INITIALIZE


    public:
        Cylinder()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        void arilesVisit(const ariles2::Finalize & /*visitor*/, const ariles2::Finalize::Parameters & /*param*/)
        {
            CPPUT_ASSERT(length_ >= 0.0, "Length must be nonnegative.");
            CPPUT_ASSERT(radius_ >= 0.0, "Length must be nonnegative.");
        }
    };


    class Mesh : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, filename, std::string)                                                                     \
    ARILES2_TYPED_ENTRY_(v, scale, double)
#include ARILES2_INITIALIZE

        // Vector3 scale;

    public:
        Mesh()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }
    };


    class Geometry : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, sphere, ariles2::OptionalPointer<std::shared_ptr<Sphere>>)                                 \
    ARILES2_TYPED_ENTRY_(v, box, ariles2::OptionalPointer<std::shared_ptr<Box>>)                                       \
    ARILES2_TYPED_ENTRY_(v, cylinder, ariles2::OptionalPointer<std::shared_ptr<Cylinder>>)                             \
    ARILES2_TYPED_ENTRY_(v, mesh, ariles2::OptionalPointer<std::shared_ptr<Mesh>>)
#include ARILES2_INITIALIZE

    public:
        Geometry()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }

        void arilesVisit(const ariles2::Finalize & /*visitor*/, const ariles2::Finalize::Parameters & /*param*/)
        {
            const std::size_t counter =
                    static_cast<std::size_t>(sphere_.isNull()) + static_cast<std::size_t>(box_.isNull())
                    + static_cast<std::size_t>(cylinder_.isNull()) + static_cast<std::size_t>(mesh_.isNull());
            CPPUT_ASSERT(1 == counter, "Geometry must contain exactly one object.");
        }
    };


    class Visual : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, name, std::string)                                                                         \
    ARILES2_TYPED_ENTRY_(v, origin, Pose)                                                                              \
    ARILES2_TYPED_ENTRY_(v, material, ariles2::OptionalPointer<std::shared_ptr<Material>>)                             \
    ARILES2_TYPED_ENTRY_(v, geometry, Geometry)
#include ARILES2_INITIALIZE

    public:
        Visual()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }
    };


    class Collision : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, name, std::string)                                                                         \
    ARILES2_TYPED_ENTRY_(v, origin, Pose)                                                                              \
    ARILES2_TYPED_ENTRY_(v, geometry, Geometry)
#include ARILES2_INITIALIZE

    public:
        Collision()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }
    };



    /*
     * http://wiki.ros.org/urdf/XML/link
     *
     <link name="my_link">
       <inertial>
         <origin xyz="0 0 0.5" rpy="0 0 0"/>
         <mass value="1"/>
         <inertia ixx="100"  ixy="0"  ixz="0" iyy="100" iyz="0" izz="100" />
       </inertial>

       <visual>
         <origin xyz="0 0 0" rpy="0 0 0" />
         <geometry>
           <box size="1 1 1" />
         </geometry>
         <material name="Cyan">
           <color rgba="0 1.0 1.0 1.0"/>
         </material>
       </visual>

       <collision>
         <origin xyz="0 0 0" rpy="0 0 0"/>
         <geometry>
           <cylinder radius="1" length="0.5"/>
         </geometry>
       </collision>
     </link>
    */
    class Link : public ariles2::DefaultBase, public cpput::PtrAddon<Link>
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, name, std::string)                                                                         \
    ARILES2_TYPED_ENTRY_(v, inertial, ariles2::OptionalPointer<std::shared_ptr<Inertial>>)                             \
    ARILES2_TYPED_ENTRY_(v, visual, std::vector<Visual>)                                                               \
    ARILES2_TYPED_ENTRY_(v, collision, std::vector<Collision>)
#include ARILES2_INITIALIZE

    public:
        Link()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }
    };
}  // namespace ariles_urdf
