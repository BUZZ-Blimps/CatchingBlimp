

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from ObjectCount_.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef ObjectCount__1272724443_h
#define ObjectCount__1272724443_h

#ifndef NDDS_STANDALONE_TYPE
#ifndef ndds_cpp_h
#include "ndds/ndds_cpp.h"
#endif
#else
#include "ndds_standalone_type.h"
#endif

#include "std_msgs/msg/dds_connext/Header_.h"
namespace yolo_msgs {
    namespace msg {
        namespace dds_ {

            extern const char *ObjectCount_TYPENAME;

            struct ObjectCount_Seq;
            #ifndef NDDS_STANDALONE_TYPE
            class ObjectCount_TypeSupport;
            class ObjectCount_DataWriter;
            class ObjectCount_DataReader;
            #endif

            class ObjectCount_ 
            {
              public:
                typedef struct ObjectCount_Seq Seq;
                #ifndef NDDS_STANDALONE_TYPE
                typedef ObjectCount_TypeSupport TypeSupport;
                typedef ObjectCount_DataWriter DataWriter;
                typedef ObjectCount_DataReader DataReader;
                #endif

                std_msgs::msg::dds_::Header_   header_ ;
                DDS_Octet   count_ ;

            };
            #if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
            /* If the code is building on Windows, start exporting symbols.
            */
            #undef NDDSUSERDllExport
            #define NDDSUSERDllExport __declspec(dllexport)
            #endif

            NDDSUSERDllExport DDS_TypeCode* ObjectCount__get_typecode(void); /* Type code */

            DDS_SEQUENCE(ObjectCount_Seq, ObjectCount_);

            NDDSUSERDllExport
            RTIBool ObjectCount__initialize(
                ObjectCount_* self);

            NDDSUSERDllExport
            RTIBool ObjectCount__initialize_ex(
                ObjectCount_* self,RTIBool allocatePointers,RTIBool allocateMemory);

            NDDSUSERDllExport
            RTIBool ObjectCount__initialize_w_params(
                ObjectCount_* self,
                const struct DDS_TypeAllocationParams_t * allocParams);  

            NDDSUSERDllExport
            void ObjectCount__finalize(
                ObjectCount_* self);

            NDDSUSERDllExport
            void ObjectCount__finalize_ex(
                ObjectCount_* self,RTIBool deletePointers);

            NDDSUSERDllExport
            void ObjectCount__finalize_w_params(
                ObjectCount_* self,
                const struct DDS_TypeDeallocationParams_t * deallocParams);

            NDDSUSERDllExport
            void ObjectCount__finalize_optional_members(
                ObjectCount_* self, RTIBool deletePointers);  

            NDDSUSERDllExport
            RTIBool ObjectCount__copy(
                ObjectCount_* dst,
                const ObjectCount_* src);

            #if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
            /* If the code is building on Windows, stop exporting symbols.
            */
            #undef NDDSUSERDllExport
            #define NDDSUSERDllExport
            #endif
        } /* namespace dds_  */
    } /* namespace msg  */
} /* namespace yolo_msgs  */

#endif /* ObjectCount_ */

