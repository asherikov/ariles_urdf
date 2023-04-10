/**
    @file
    @author  Alexander Sherikov

    @brief
*/


#define ARILES2_DEFAULT_VISITORS                                                                                       \
    ARILES2_VISITOR(count)                                                                                             \
    ARILES2_VISITOR(count_missing)                                                                                     \
    ARILES2_VISITOR(finalize)                                                                                          \
    ARILES2_VISITOR(prewrite)                                                                                          \
    ARILES2_VISITOR(defaults)                                                                                          \
    ARILES2_VISITOR(read)                                                                                              \
    ARILES2_VISITOR(write)                                                                                             \
    ARILES2_VISITOR(compare)

#include <ariles2/visitors/yaml_cpp.h>
#include <ariles2/visitors/pugixml.h>
#include <ariles2/visitors/compare.h>

#include <iostream>
#include <sstream>
#include <ariles_urdf/model.h>

#include "boost_utf_common.h"


namespace
{
    class TestParameters : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, filename, std::string)                                                                     \
    ARILES2_TYPED_ENTRY_(v, num_joints, std::size_t)                                                                   \
    ARILES2_TYPED_ENTRY_(v, num_links, std::size_t)                                                                    \
    ARILES2_TYPED_ENTRY_(v, num_materials, std::size_t)
#define ARILES2_DEFAULT_ID "TestParameters"
#include ARILES2_INITIALIZE
    };


    class URDFFixture
    {
    public:
        ariles_urdf::Model model_;
        ariles_urdf::Model model_deserialized_;
        TestParameters test_param_;

        ariles2::Compare compare_visitor_;
        ariles2::Compare::Parameters compare_parameters_;

    public:
        URDFFixture()
        {
            compare_parameters_.double_tolerance_ = 1e-12;
            compare_parameters_.compare_number_of_entries_ = true;
        }


        ~URDFFixture()
        {
        }


        void initialize(const std::string &filename)
        {
            ariles2::apply<ariles2::yaml_cpp::Reader>(filename, test_param_);

            ariles2::apply<ariles2::pugixml::Reader>(test_param_.filename_, model_);
            ariles2::apply<ariles2::yaml_cpp::Writer>(std::cout, model_);

            std::stringstream ios;
            ariles2::apply<ariles2::yaml_cpp::Writer>(ios, model_);
            ariles2::apply<ariles2::yaml_cpp::Reader>(ios, model_deserialized_);
        }
    };
}  // namespace


BOOST_FIXTURE_TEST_CASE(example_01, URDFFixture)
{
    initialize("urdf/example_01.yaml");

    BOOST_CHECK(ariles2::apply(compare_visitor_, model_deserialized_, model_, compare_parameters_));

    BOOST_CHECK_EQUAL(model_.link_.size(), test_param_.num_links_);
    BOOST_CHECK_EQUAL(model_.joint_.size(), test_param_.num_joints_);
    BOOST_CHECK_EQUAL(model_.material_.size(), test_param_.num_materials_);
}
