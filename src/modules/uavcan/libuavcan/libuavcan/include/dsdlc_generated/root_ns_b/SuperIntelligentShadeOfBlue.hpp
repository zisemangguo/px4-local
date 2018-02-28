/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/zisemangguo/workspace/Firmware/src/modules/uavcan/libuavcan/libuavcan/test/dsdl_test/root_ns_b/SuperIntelligentShadeOfBlue.uavcan
 */

#ifndef ROOT_NS_B_SUPERINTELLIGENTSHADEOFBLUE_HPP_INCLUDED
#define ROOT_NS_B_SUPERINTELLIGENTSHADEOFBLUE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <root_ns_a/NestedMessage.hpp>

/******************************* Source text **********************************
float16[<32] array_f16
root_ns_a.NestedMessage[3] nested_message
******************************************************************************/

/********************* DSDL signature source definition ***********************
root_ns_b.SuperIntelligentShadeOfBlue
saturated float16[<=31] array_f16
root_ns_a.NestedMessage[3] nested_message
******************************************************************************/

#undef array_f16
#undef nested_message

namespace root_ns_b
{

template <int _tmpl>
struct UAVCAN_EXPORT SuperIntelligentShadeOfBlue_
{
    typedef const SuperIntelligentShadeOfBlue_<_tmpl>& ParameterType;
    typedef SuperIntelligentShadeOfBlue_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 31 > array_f16;
        typedef ::uavcan::Array< ::root_ns_a::NestedMessage, ::uavcan::ArrayModeStatic, 3 > nested_message;
    };

    enum
    {
        MinBitLen
            = FieldTypes::array_f16::MinBitLen
            + FieldTypes::nested_message::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::array_f16::MaxBitLen
            + FieldTypes::nested_message::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::array_f16 >::Type array_f16;
    typename ::uavcan::StorageType< typename FieldTypes::nested_message >::Type nested_message;

    SuperIntelligentShadeOfBlue_()
        : array_f16()
        , nested_message()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<507 == MaxBitLen>::check();
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
        return "root_ns_b.SuperIntelligentShadeOfBlue";
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
bool SuperIntelligentShadeOfBlue_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        array_f16 == rhs.array_f16 &&
        nested_message == rhs.nested_message;
}

template <int _tmpl>
bool SuperIntelligentShadeOfBlue_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(array_f16, rhs.array_f16) &&
        ::uavcan::areClose(nested_message, rhs.nested_message);
}

template <int _tmpl>
int SuperIntelligentShadeOfBlue_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::array_f16::encode(self.array_f16, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::nested_message::encode(self.nested_message, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int SuperIntelligentShadeOfBlue_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::array_f16::decode(self.array_f16, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::nested_message::decode(self.nested_message, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature SuperIntelligentShadeOfBlue_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x50D1F32A0A89CBE7ULL);

    FieldTypes::array_f16::extendDataTypeSignature(signature);
    FieldTypes::nested_message::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef SuperIntelligentShadeOfBlue_<0> SuperIntelligentShadeOfBlue;

// No default registration

} // Namespace root_ns_b

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::root_ns_b::SuperIntelligentShadeOfBlue >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::root_ns_b::SuperIntelligentShadeOfBlue::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::root_ns_b::SuperIntelligentShadeOfBlue >::stream(Stream& s, ::root_ns_b::SuperIntelligentShadeOfBlue::ParameterType obj, const int level)
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
    s << "array_f16: ";
    YamlStreamer< ::root_ns_b::SuperIntelligentShadeOfBlue::FieldTypes::array_f16 >::stream(s, obj.array_f16, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "nested_message: ";
    YamlStreamer< ::root_ns_b::SuperIntelligentShadeOfBlue::FieldTypes::nested_message >::stream(s, obj.nested_message, level + 1);
}

}

namespace root_ns_b
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::root_ns_b::SuperIntelligentShadeOfBlue::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::root_ns_b::SuperIntelligentShadeOfBlue >::stream(s, obj, 0);
    return s;
}

} // Namespace root_ns_b

#endif // ROOT_NS_B_SUPERINTELLIGENTSHADEOFBLUE_HPP_INCLUDED