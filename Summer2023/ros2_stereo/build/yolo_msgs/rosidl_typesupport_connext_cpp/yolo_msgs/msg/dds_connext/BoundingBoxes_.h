

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from BoundingBoxes_.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef BoundingBoxes__1048265649_h
#define BoundingBoxes__1048265649_h

#ifndef NDDS_STANDALONE_TYPE
#ifndef ndds_cpp_h
#include "ndds/ndds_cpp.h"
#endif
#else
#include "ndds_standalone_type.h"
#endif

#include "yolo_msgs/msg/dds_connext/BoundingBox_.h"
namespace yolo_msgs {
    namespace msg {
        namespace dds_ {

            extern const char *BoundingBoxes_TYPENAME;

            struct BoundingBoxes_Seq;
            #ifndef NDDS_STANDALONE_TYPE
            class BoundingBoxes_TypeSupport;
            class BoundingBoxes_DataWriter;
            class BoundingBoxes_DataReader;
            #endif

            class BoundingBoxes_ 
            {
              public:
                typedef struct BoundingBoxes_Seq Seq;
                #ifndef NDDS_STANDALONE_TYPE
                typedef BoundingBoxes_TypeSupport TypeSupport;
                typedef BoundingBoxes_DataWriter DataWriter;
                typedef BoundingBoxes_DataReader DataReader;
                #endif

                yolo_msgs::msg::dds_::BoundingBox_Seq  bounding_boxes_ ;

            };
            #if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
            /* If the code is building on Windows, start exporting symbols.
            */
            #undef NDDSUSERDllExport
            #define NDDSUSERDllExport __declspec(dllexport)
            #endif

            NDDSUSERDllExport DDS_TypeCode* BoundingBoxes__get_typecode(void); /* Type code */

            DDS_SEQUENCE(BoundingBoxes_Seq, BoundingBoxes_);

            NDDSUSERDllExport
            RTIBool BoundingBoxes__initialize(
                BoundingBoxes_* self);

            NDDSUSERDllExport
            RTIBool BoundingBoxes__initialize_ex(
                BoundingBoxes_* self,RTIBool allocatePointers,RTIBool allocateMemory);

            NDDSUSERDllExport
            RTIBool BoundingBoxes__initialize_w_params(
                BoundingBoxes_* self,
                const struct DDS_TypeAllocationParams_t * allocParams);  

            NDDSUSERDllExport
            void BoundingBoxes__finalize(
                BoundingBoxes_* self);

            NDDSUSERDllExport
            void BoundingBoxes__finalize_ex(
                BoundingBoxes_* self,RTIBool deletePointers);

            NDDSUSERDllExport
            void BoundingBoxes__finalize_w_params(
                BoundingBoxes_* self,
                const struct DDS_TypeDeallocationParams_t * deallocParams);

            NDDSUSERDllExport
            void BoundingBoxes__finalize_optional_members(
                BoundingBoxes_* self, RTIBool deletePointers);  

            NDDSUSERDllExport
            RTIBool BoundingBoxes__copy(
                BoundingBoxes_* dst,
                const BoundingBoxes_* src);

            #if (defined(RTI_WIN32) || defined (RTI_WINCE)) && defined(NDDS_USER_DLL_EXPORT)
            /* If the code is building on Windows, stop exporting symbols.
            */
            #undef NDDSUSERDllExport
            #define NDDSUSERDllExport
            #endif
        } /* namespace dds_  */
    } /* namespace msg  */
} /* namespace yolo_msgs  */

#endif /* BoundingBoxes_ */

