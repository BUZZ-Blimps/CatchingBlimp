

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from ResizedImage_.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef ResizedImage__737261621_h
#define ResizedImage__737261621_h

#ifndef NDDS_STANDALONE_TYPE
#ifndef ndds_cpp_h
#include "ndds/ndds_cpp.h"
#endif
#else
#include "ndds_standalone_type.h"
#endif

#include "sensor_msgs/msg/dds_connext/Image_.h"
namespace opencv_telemetry {
    namespace msg {
        namespace dds_ {

            extern const char *ResizedImage_TYPENAME;

            struct ResizedImage_Seq;
            #ifndef NDDS_STANDALONE_TYPE
            class ResizedImage_TypeSupport;
            class ResizedImage_DataWriter;
            class ResizedImage_DataReader;
            #endif

            class ResizedImage_ 
            {
              public:
                typedef struct ResizedImage_Seq Seq;
                #ifndef NDDS_STANDALONE_TYPE
                typedef ResizedImage_TypeSupport TypeSupport;
                typedef ResizedImage_DataWriter DataWriter;
                typedef ResizedImage_DataReader DataReader;
                #endif

                DDS_UnsignedLong   original_height_ ;
                DDS_UnsignedLong   original_width_ ;
                sensor_msgs::msg::dds_::Image_   image_ ;

            };
            #if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
            /* If the code is building on Windows, start exporting symbols.
            */
            #undef NDDSUSERDllExport
            #define NDDSUSERDllExport __declspec(dllexport)
            #endif

            NDDSUSERDllExport DDS_TypeCode* ResizedImage__get_typecode(void); /* Type code */

            DDS_SEQUENCE(ResizedImage_Seq, ResizedImage_);

            NDDSUSERDllExport
            RTIBool ResizedImage__initialize(
                ResizedImage_* self);

            NDDSUSERDllExport
            RTIBool ResizedImage__initialize_ex(
                ResizedImage_* self,RTIBool allocatePointers,RTIBool allocateMemory);

            NDDSUSERDllExport
            RTIBool ResizedImage__initialize_w_params(
                ResizedImage_* self,
                const struct DDS_TypeAllocationParams_t * allocParams);  

            NDDSUSERDllExport
            void ResizedImage__finalize(
                ResizedImage_* self);

            NDDSUSERDllExport
            void ResizedImage__finalize_ex(
                ResizedImage_* self,RTIBool deletePointers);

            NDDSUSERDllExport
            void ResizedImage__finalize_w_params(
                ResizedImage_* self,
                const struct DDS_TypeDeallocationParams_t * deallocParams);

            NDDSUSERDllExport
            void ResizedImage__finalize_optional_members(
                ResizedImage_* self, RTIBool deletePointers);  

            NDDSUSERDllExport
            RTIBool ResizedImage__copy(
                ResizedImage_* dst,
                const ResizedImage_* src);

            #if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
            /* If the code is building on Windows, stop exporting symbols.
            */
            #undef NDDSUSERDllExport
            #define NDDSUSERDllExport
            #endif
        } /* namespace dds_  */
    } /* namespace msg  */
} /* namespace opencv_telemetry  */

#endif /* ResizedImage_ */

