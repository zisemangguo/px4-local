/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/zisemangguo/workspace/Firmware/src/modules/uavcan/libuavcan/dsdl/uavcan/protocol/HardwareVersion.uavcan
 */

#ifndef UAVCAN_PROTOCOL_HARDWAREVERSION_HPP_INCLUDED
#define UAVCAN_PROTOCOL_HARDWAREVERSION_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Nested type.
# Generic hardware version information.
# These values should remain unchanged for the device's lifetime.
#

#
# Hardware version code.
#
uint8 major
uint8 minor

#
# All zeros is not a valid UID.
# If filled with zeros, assume that the value is undefined.
#
uint8[16] unique_id

#
# Certificate of authenticity (COA) of the hardware, 255 bytes max.
#
uint8[<=255] certificate_of_authenticity
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.HardwareVersion
saturated uint8 major
saturated uint8 minor
saturated uint8[16] unique_id
saturated uint8[<=255] certificate_of_authenticity
******************************************************************************/

#undef major
#undef minor
#undef unique_id
#undef certificate_of_authenticity

namespace uavcan
{
namespace protocol
{

template <int _tmpl>
struct UAVCAN_EXPORT HardwareVersion_
{
    typedef const HardwareVersion_<_tmpl>& ParameterType;
    typedef HardwareVersion_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > major;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > minor;
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 16 > unique_id;
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 255 > certificate_of_authenticity;
    };

    enum
    {
        MinBitLen
            = FieldTypes::major::MinBitLen
            + FieldTypes::minor::MinBitLen
            + FieldTypes::unique_id::MinBitLen
            + FieldTypes::certificate_of_authenticity::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::major::MaxBitLen
            + FieldTypes::minor::MaxBitLen
            + FieldTypes::unique_id::MaxBitLen
            + FieldTypes::certificate_of_authenticity::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::major >::Type major;
    typename ::uavcan::StorageType< typename FieldTypes::minor >::Type minor;
    typename ::uavcan::StorageType< typename FieldTypes::unique_id >::Type unique_id;
    typename ::uavcan::StorageType< typename FieldTypes::certificate_of_authenticity >::Type certificate_of_authenticity;

    HardwareVersion_()
        : major()
        , minor()
        , unique_id()
        , certificate_of_authenticity()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<2192 == MaxBitLen>::check();
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
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.HardwareVersion";
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
bool HardwareVersion_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        major == rhs.major &&
        minor == rhs.minor &&
        unique_id == rhs.unique_id &&
        certificate_of_authenticity == rhs.certificate_of_authenticity;
}

template <int _tmpl>
bool HardwareVersion_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(major, rhs.major) &&
        ::uavcan::areClose(minor, rhs.minor) &&
        ::uavcan::areClose(unique_id, rhs.unique_id) &&
        ::uavcan::areClose(certificate_of_authenticity, rhs.certificate_of_authenticity);
}

template <int _tmpl>
int HardwareVersion_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::major::encode(self.major, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::minor::encode(self.minor, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::unique_id::encode(self.unique_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::certificate_of_authenticity::encode(self.certificate_of_authenticity, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int HardwareVersion_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::major::decode(self.major, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::minor::decode(self.minor, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::unique_id::decode(self.unique_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::certificate_of_authenticity::decode(self.certificate_of_authenticity, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature HardwareVersion_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xAD5C4C933F4A0C4ULL);

    FieldTypes::major::extendDataTypeSignature(signature);
    FieldTypes::minor::extendDataTypeSignature(signature);
    FieldTypes::unique_id::extendDataTypeSignature(signature);
    FieldTypes::certificate_of_authenticity::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef HardwareVersion_<0> HardwareVersion;

// No default registration

} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::HardwareVersion >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::HardwareVersion::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::HardwareVersion >::stream(Stream& s, ::uavcan::protocol::HardwareVersion::ParameterType obj, const int level)
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
    s << "major: ";
    YamlStreamer< ::uavcan::protocol::HardwareVersion::FieldTypes::major >::stream(s, obj.major, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "minor: ";
    YamlStreamer< ::uavcan::protocol::HardwareVersion::FieldTypes::minor >::stream(s, obj.minor, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "unique_id: ";
    YamlStreamer< ::uavcan::protocol::HardwareVersion::FieldTypes::unique_id >::stream(s, obj.unique_id, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "certificate_of_authenticity: ";
    YamlStreamer< ::uavcan::protocol::HardwareVersion::FieldTypes::certificate_of_authenticity >::stream(s, obj.certificate_of_authenticity, level + 1);
}

}

namespace uavcan
{
namespace protocol
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::HardwareVersion::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::HardwareVersion >::stream(s, obj, 0);
    return s;
}

} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_HARDWAREVERSION_HPP_INCLUDED