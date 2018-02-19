/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/valaitis/workspace/kmti/gimbal/modules/libuavcan/dsdl/uavcan/equipment/camera_gimbal/1044.Status.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_CAMERA_GIMBAL_STATUS_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_CAMERA_GIMBAL_STATUS_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan/equipment/camera_gimbal/Mode.hpp>

/******************************* Source text **********************************
#
# Generic gimbal status.
#

uint8 gimbal_id

Mode mode

#
# Camera axis orientation in body frame (not in fixed frame).
# Please refer to the UAVCAN coordinate frame conventions.
#
float16[4] camera_orientation_in_body_frame_xyzw
float16[<=9] camera_orientation_in_body_frame_covariance   # +inf for non-existent axes
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.camera_gimbal.Status
saturated uint8 gimbal_id
uavcan.equipment.camera_gimbal.Mode mode
saturated float16[4] camera_orientation_in_body_frame_xyzw
saturated float16[<=9] camera_orientation_in_body_frame_covariance
******************************************************************************/

#undef gimbal_id
#undef mode
#undef camera_orientation_in_body_frame_xyzw
#undef camera_orientation_in_body_frame_covariance

namespace uavcan
{
namespace equipment
{
namespace camera_gimbal
{

template <int _tmpl>
struct UAVCAN_EXPORT Status_
{
    typedef const Status_<_tmpl>& ParameterType;
    typedef Status_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > gimbal_id;
        typedef ::uavcan::equipment::camera_gimbal::Mode mode;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 4 > camera_orientation_in_body_frame_xyzw;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 9 > camera_orientation_in_body_frame_covariance;
    };

    enum
    {
        MinBitLen
            = FieldTypes::gimbal_id::MinBitLen
            + FieldTypes::mode::MinBitLen
            + FieldTypes::camera_orientation_in_body_frame_xyzw::MinBitLen
            + FieldTypes::camera_orientation_in_body_frame_covariance::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::gimbal_id::MaxBitLen
            + FieldTypes::mode::MaxBitLen
            + FieldTypes::camera_orientation_in_body_frame_xyzw::MaxBitLen
            + FieldTypes::camera_orientation_in_body_frame_covariance::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::gimbal_id >::Type gimbal_id;
    typename ::uavcan::StorageType< typename FieldTypes::mode >::Type mode;
    typename ::uavcan::StorageType< typename FieldTypes::camera_orientation_in_body_frame_xyzw >::Type camera_orientation_in_body_frame_xyzw;
    typename ::uavcan::StorageType< typename FieldTypes::camera_orientation_in_body_frame_covariance >::Type camera_orientation_in_body_frame_covariance;

    Status_()
        : gimbal_id()
        , mode()
        , camera_orientation_in_body_frame_xyzw()
        , camera_orientation_in_body_frame_covariance()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<228 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 1044 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.camera_gimbal.Status";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool Status_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        gimbal_id == rhs.gimbal_id &&
        mode == rhs.mode &&
        camera_orientation_in_body_frame_xyzw == rhs.camera_orientation_in_body_frame_xyzw &&
        camera_orientation_in_body_frame_covariance == rhs.camera_orientation_in_body_frame_covariance;
}

template <int _tmpl>
bool Status_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(gimbal_id, rhs.gimbal_id) &&
        ::uavcan::areClose(mode, rhs.mode) &&
        ::uavcan::areClose(camera_orientation_in_body_frame_xyzw, rhs.camera_orientation_in_body_frame_xyzw) &&
        ::uavcan::areClose(camera_orientation_in_body_frame_covariance, rhs.camera_orientation_in_body_frame_covariance);
}

template <int _tmpl>
int Status_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::gimbal_id::encode(self.gimbal_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::mode::encode(self.mode, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::camera_orientation_in_body_frame_xyzw::encode(self.camera_orientation_in_body_frame_xyzw, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::camera_orientation_in_body_frame_covariance::encode(self.camera_orientation_in_body_frame_covariance, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Status_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::gimbal_id::decode(self.gimbal_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::mode::decode(self.mode, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::camera_orientation_in_body_frame_xyzw::decode(self.camera_orientation_in_body_frame_xyzw, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::camera_orientation_in_body_frame_covariance::decode(self.camera_orientation_in_body_frame_covariance, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Status_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x47D86D0CE87E7542ULL);

    FieldTypes::gimbal_id::extendDataTypeSignature(signature);
    FieldTypes::mode::extendDataTypeSignature(signature);
    FieldTypes::camera_orientation_in_body_frame_xyzw::extendDataTypeSignature(signature);
    FieldTypes::camera_orientation_in_body_frame_covariance::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef Status_<0> Status;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::camera_gimbal::Status > _uavcan_gdtr_registrator_Status;

}

} // Namespace camera_gimbal
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::camera_gimbal::Status >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::camera_gimbal::Status::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::camera_gimbal::Status >::stream(Stream& s, ::uavcan::equipment::camera_gimbal::Status::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "gimbal_id: ";
    YamlStreamer< ::uavcan::equipment::camera_gimbal::Status::FieldTypes::gimbal_id >::stream(s, obj.gimbal_id, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "mode: ";
    YamlStreamer< ::uavcan::equipment::camera_gimbal::Status::FieldTypes::mode >::stream(s, obj.mode, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "camera_orientation_in_body_frame_xyzw: ";
    YamlStreamer< ::uavcan::equipment::camera_gimbal::Status::FieldTypes::camera_orientation_in_body_frame_xyzw >::stream(s, obj.camera_orientation_in_body_frame_xyzw, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "camera_orientation_in_body_frame_covariance: ";
    YamlStreamer< ::uavcan::equipment::camera_gimbal::Status::FieldTypes::camera_orientation_in_body_frame_covariance >::stream(s, obj.camera_orientation_in_body_frame_covariance, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace camera_gimbal
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::camera_gimbal::Status::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::camera_gimbal::Status >::stream(s, obj, 0);
    return s;
}

} // Namespace camera_gimbal
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_CAMERA_GIMBAL_STATUS_HPP_INCLUDED