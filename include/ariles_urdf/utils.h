#pragma once

#include <locale>
#include <stdexcept>
#include <string>
#include <sstream>

#include "common.h"

namespace ariles_urdf
{
    template <class t_Vector>
    class VectorValue
    {
    public:
        t_Vector vector_;


    public:
        void fromString(const std::string &value)
        {
            const std::locale locale_backup = std::locale::global(std::locale::classic());

            Eigen::Index index = 0;
            std::size_t word_start = 0;
            std::size_t word_end = 0;
            while ((word_end = value.find(' ', word_start)) != std::string::npos)
            {
                const std::size_t len = word_end - word_start;
                if (len > 0)
                {
                    try
                    {
                        CPPUT_ASSERT(
                                vector_.rows() > index,
                                "Found more elements than expected while parsing value [" + value + "]");
                        vector_[index] = std::stod(value.substr(word_start, len));
                        ++index;
                    }
                    catch (const std::invalid_argument &e)
                    {
                        CPPUT_THROW("Unable to parse value [" + value + "]: " + e.what());
                    }
                }
                ++word_start;
            }

            std::locale::global(locale_backup);
        }


        void toString(std::string &out) const
        {
            std::stringstream stream;
            stream << vector_.transpose().format(Eigen::IOFormat(Eigen::FullPrecision));
            out = stream.str();
        }


        void setNaN()
        {
            vector_.setConstant(std::numeric_limits<typename t_Vector::Scalar>::quiet_NaN());
        }


        void setZero()
        {
            vector_.setZero();
        }


        bool isNaN() const
        {
            return (vector_.array().isnan().any());
        }


        bool isNonNegative() const
        {
            return ((vector_.array() >= 0.0).all());
        }


        bool is0to1() const
        {
            return ((vector_.array() >= 0.0).all() and (vector_.array() <= 1.0).all());
        }
    };
}  // namespace ariles_urdf
