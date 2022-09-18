//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#ifndef ZEN_COMPONENTS_SENSORPARSING_UTIL_H_
#define ZEN_COMPONENTS_SENSORPARSING_UTIL_H_

#include "ZenTypes.h"
#include "ZenTypesHelpers.h"
#include "ISensorProperties.h"

#include <gsl/span>
#include <cstring>
#include <cstddef>
#include <nonstd/expected.hpp>
#include <cmath>

#include <spdlog/spdlog.h>

namespace zen {
    namespace sensor_parsing_util {
        inline void safe_subspan(gsl::span<const std::byte>& data, size_t shortenBy) {
            if (int(shortenBy) >= data.size()) {
                // subspan with zero cannot be made, so nullify the span here;
                data = {};
            } else {
                data = data.subspan(shortenBy);
            }
        }

        /**
        Parse a float16 from a byte stream, advance the stream and return the float
        */
        inline float parseFloat16(gsl::span<const std::byte>& data, float denominator) noexcept
        {
            const int16_t temp = int16_t(data[0]) + int16_t(data[1]) * 256;
            safe_subspan(data, 2);
            return static_cast<float>(temp) / denominator;
        }

        /**
        Parse a float32 from a byte stream, advance the stream and return the float
        */
        inline float parseFloat32(gsl::span<const std::byte>& data) noexcept
        {
            const int32_t temp = ((int32_t(data[3]) * 256 + int32_t(data[2])) * 256 + int32_t(data[1])) * 256 + int32_t(data[0]);
            safe_subspan(data, 4);
            float result;
            std::memcpy(&result, &temp, 4);
            return result;
        }

        /**
        Convert an integer value to a float using a scale exponent according to this
        formula:

            float_out = int * 10^scaleExponent
        */
        template<class TIntegerType>
        inline double integerToScaledDouble(TIntegerType it, int32_t scaleExponent) {
            return double(it) * std::pow(double(10.0), double(scaleExponent));
        }

        /*
        Convert from rad to degrees in case the sensor is configured to output radians.
        This is needed for IG1-style firmware which has the option to switch
        between degrees and radian output.
        */
        inline ZenError radToDegreesIfNeededVector3(std::unique_ptr<ISensorProperties> const& properties,
            float * targetArray) {
            auto isRadOutput = properties->getBool(ZenImuProperty_DegRadOutput);
            if (!isRadOutput)
                return isRadOutput.error();

            if (isRadOutput.value()) {
                radToDeg3(targetArray);
            }

            return ZenError_None;
        }

        template<int N>
        inline nonstd::expected<bool, ZenError> readVectorNIfAvailable(ZenProperty_t checkProperty,
            std::unique_ptr<ISensorProperties> const& properties, bool lowPrecision, float denominator_16bit,
            gsl::span<const std::byte>& data, float * targetArray) {
            auto enabled = properties->getBool(checkProperty);
            if (!enabled)
                return enabled;

            if (*enabled) {
                const size_t floatSize = lowPrecision ? sizeof(uint16_t) : sizeof(float);

                if (data.size() < int(N * floatSize)) {
                    spdlog::error("Cannot parse Vector{0} because data buffer too small", N);
                    return ZenError_Io_MsgCorrupt;
                }

                for (unsigned idx = 0; idx < N; ++idx)
                    targetArray[idx] = lowPrecision ? parseFloat16(data, denominator_16bit) : parseFloat32(data);
            }

            return enabled;
        }

        inline nonstd::expected<bool, ZenError> readVector3IfAvailable(ZenProperty_t checkProperty,
            std::unique_ptr<ISensorProperties> const& properties, bool lowPrecision, float denominator_16bit,
            gsl::span<const std::byte>& data, float * targetArray) {

            return readVectorNIfAvailable<3>(checkProperty, properties, lowPrecision, denominator_16bit,
                data, targetArray);
        }

        inline nonstd::expected<bool, ZenError> readVector4IfAvailable(ZenProperty_t checkProperty,
            std::unique_ptr<ISensorProperties> const& properties, bool lowPrecision, float denominator_16bit,
            gsl::span<const std::byte>& data, float * targetArray) {

            return readVectorNIfAvailable<4>(checkProperty, properties, lowPrecision, denominator_16bit,
                data, targetArray);
        }

        /**
        Templated function to read a scalar data type from a byte stream.
        */
        template <class TTypeToRead>
        inline void parseAndStoreScalar(gsl::span<const std::byte>& data, TTypeToRead *target, bool useLowPrecision = false,
            float denominator_16bit = 1.0f);

        /**
        Specialization for uint32_t
        */
        template <>
        inline void parseAndStoreScalar(gsl::span<const std::byte>& data, uint32_t *target, bool, float) {
            (*target) = *reinterpret_cast<const uint32_t*>(data.data());
            safe_subspan(data, sizeof(uint32_t));
        }

        /**
        Specialization for uint16_t
        */
        template <>
        inline void parseAndStoreScalar(gsl::span<const std::byte>& data, uint16_t *target, bool, float) {
            (*target) = *reinterpret_cast<const uint16_t*>(data.data());
            safe_subspan(data, sizeof(uint16_t));
        }

        /**
        Specialization for uint8_t
        */
        template <>
        inline void parseAndStoreScalar(gsl::span<const std::byte>& data, uint8_t *target, bool, float) {
            (*target) = *reinterpret_cast<const uint8_t*>(data.data());
            safe_subspan(data, sizeof(uint8_t));
        }

        /**
        Specialization for int32_t
        */
        template <>
        inline void parseAndStoreScalar(gsl::span<const std::byte>& data, int32_t *target, bool, float) {
            (*target) = *reinterpret_cast<const int32_t*>(data.data());
            safe_subspan(data, sizeof(int32_t));
        }

        /**
        Specialization for float
        */
        template <>
        inline void parseAndStoreScalar(
            gsl::span<const std::byte>& data, float *target, bool useLowPrecision, float denominator_16bit) {
            const size_t floatSize = useLowPrecision ? sizeof(uint16_t) : sizeof(float);

            (*target) = useLowPrecision ? parseFloat16(data, denominator_16bit) : parseFloat32(data);
            safe_subspan(data, floatSize);
        }

        /**
        Templated function to check if the output property is enable and if so, read the
        data type from the span byte buffer.
        */
        template <class TTypeToRead>
        inline nonstd::expected<bool, ZenError> readScalarIfAvailable(ZenProperty_t checkProperty,
            std::unique_ptr<ISensorProperties> const& properties,
            gsl::span<const std::byte>& data, TTypeToRead * target, bool lowPrecision = false, float denominator_16bit = 1.0f) {
            auto enabled = properties->getBool(checkProperty);
            if (!enabled)
                return enabled;

            if (*enabled) {
                int checkSize = int(sizeof(TTypeToRead));

                // in case of float and low-precision mode we only need to check for 16-bits
                if (std::is_same<TTypeToRead, float>::value) {
                    checkSize = lowPrecision ? int(sizeof(int16_t)) : int(sizeof(float));
                }

                if (data.size() < checkSize) {
                    spdlog::error("Cannot parse scaler value because data buffer too small");
                    return ZenError_Io_MsgCorrupt;
                }

                parseAndStoreScalar<TTypeToRead>(data, target, lowPrecision, denominator_16bit);
                return true;
            }

            return false;
        }
    }
}

#endif
