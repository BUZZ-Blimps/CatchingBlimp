

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from BoundingBoxes_.idl using "rtiddsgen".
The rtiddsgen tool is part of the RTI Connext distribution.
For more information, type 'rtiddsgen -help' at a command shell
or consult the RTI Connext manual.
*/

#ifndef NDDS_STANDALONE_TYPE
#ifndef ndds_cpp_h
#include "ndds/ndds_cpp.h"
#endif
#ifndef dds_c_log_impl_h              
#include "dds_c/dds_c_log_impl.h"                                
#endif        

#ifndef cdr_type_h
#include "cdr/cdr_type.h"
#endif    

#ifndef osapi_heap_h
#include "osapi/osapi_heap.h" 
#endif
#else
#include "ndds_standalone_type.h"
#endif

#include "BoundingBoxes_.h"

#include <new>

namespace yolo_msgs {
    namespace msg {
        namespace dds_ {

            /* ========================================================================= */
            const char *BoundingBoxes_TYPENAME = "yolo_msgs::msg::dds_::BoundingBoxes_";

            DDS_TypeCode* BoundingBoxes__get_typecode()
            {
                static RTIBool is_initialized = RTI_FALSE;

                static DDS_TypeCode BoundingBoxes__g_tc_bounding_boxes__sequence = DDS_INITIALIZE_SEQUENCE_TYPECODE(RTI_INT32_MAX,NULL);
                static DDS_TypeCode_Member BoundingBoxes__g_tc_members[1]=
                {

                    {
                        (char *)"bounding_boxes_",/* Member name */
                        {
                            0,/* Representation ID */          
                            DDS_BOOLEAN_FALSE,/* Is a pointer? */
                            -1, /* Bitfield bits */
                            NULL/* Member type code is assigned later */
                        },
                        0, /* Ignored */
                        0, /* Ignored */
                        0, /* Ignored */
                        NULL, /* Ignored */
                        RTI_CDR_REQUIRED_MEMBER, /* Is a key? */
                        DDS_PUBLIC_MEMBER,/* Member visibility */
                        1,
                        NULL/* Ignored */
                    }
                };

                static DDS_TypeCode BoundingBoxes__g_tc =
                {{
                        DDS_TK_STRUCT,/* Kind */
                        DDS_BOOLEAN_FALSE, /* Ignored */
                        -1, /*Ignored*/
                        (char *)"yolo_msgs::msg::dds_::BoundingBoxes_", /* Name */
                        NULL, /* Ignored */      
                        0, /* Ignored */
                        0, /* Ignored */
                        NULL, /* Ignored */
                        1, /* Number of members */
                        BoundingBoxes__g_tc_members, /* Members */
                        DDS_VM_NONE  /* Ignored */         
                    }}; /* Type code for BoundingBoxes_*/

                if (is_initialized) {
                    return &BoundingBoxes__g_tc;
                }

                BoundingBoxes__g_tc_bounding_boxes__sequence._data._typeCode = (RTICdrTypeCode *)yolo_msgs::msg::dds_::BoundingBox__get_typecode();

                BoundingBoxes__g_tc_members[0]._representation._typeCode = (RTICdrTypeCode *)& BoundingBoxes__g_tc_bounding_boxes__sequence;

                is_initialized = RTI_TRUE;

                return &BoundingBoxes__g_tc;
            }

            RTIBool BoundingBoxes__initialize(
                BoundingBoxes_* sample) {
                return yolo_msgs::msg::dds_::BoundingBoxes__initialize_ex(sample,RTI_TRUE,RTI_TRUE);
            }

            RTIBool BoundingBoxes__initialize_ex(
                BoundingBoxes_* sample,RTIBool allocatePointers, RTIBool allocateMemory)
            {

                struct DDS_TypeAllocationParams_t allocParams =
                DDS_TYPE_ALLOCATION_PARAMS_DEFAULT;

                allocParams.allocate_pointers =  (DDS_Boolean)allocatePointers;
                allocParams.allocate_memory = (DDS_Boolean)allocateMemory;

                return yolo_msgs::msg::dds_::BoundingBoxes__initialize_w_params(
                    sample,&allocParams);

            }

            RTIBool BoundingBoxes__initialize_w_params(
                BoundingBoxes_* sample, const struct DDS_TypeAllocationParams_t * allocParams)
            {

                void* buffer = NULL;
                if (buffer) {} /* To avoid warnings */

                if (sample == NULL) {
                    return RTI_FALSE;
                }
                if (allocParams == NULL) {
                    return RTI_FALSE;
                }

                if (allocParams->allocate_memory) {
                    yolo_msgs::msg::dds_::BoundingBox_Seq_initialize(&sample->bounding_boxes_ );
                    yolo_msgs::msg::dds_::BoundingBox_Seq_set_element_allocation_params(&sample->bounding_boxes_ ,allocParams);
                    yolo_msgs::msg::dds_::BoundingBox_Seq_set_absolute_maximum(&sample->bounding_boxes_ , RTI_INT32_MAX);
                    if (!yolo_msgs::msg::dds_::BoundingBox_Seq_set_maximum(&sample->bounding_boxes_, (0))) {
                        return RTI_FALSE;
                    }
                } else { 
                    yolo_msgs::msg::dds_::BoundingBox_Seq_set_length(&sample->bounding_boxes_, 0);
                }
                return RTI_TRUE;
            }

            void BoundingBoxes__finalize(
                BoundingBoxes_* sample)
            {

                yolo_msgs::msg::dds_::BoundingBoxes__finalize_ex(sample,RTI_TRUE);
            }

            void BoundingBoxes__finalize_ex(
                BoundingBoxes_* sample,RTIBool deletePointers)
            {
                struct DDS_TypeDeallocationParams_t deallocParams =
                DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;

                if (sample==NULL) {
                    return;
                } 

                deallocParams.delete_pointers = (DDS_Boolean)deletePointers;

                yolo_msgs::msg::dds_::BoundingBoxes__finalize_w_params(
                    sample,&deallocParams);
            }

            void BoundingBoxes__finalize_w_params(
                BoundingBoxes_* sample,const struct DDS_TypeDeallocationParams_t * deallocParams)
            {

                if (sample==NULL) {
                    return;
                }

                if (deallocParams == NULL) {
                    return;
                }

                yolo_msgs::msg::dds_::BoundingBox_Seq_set_element_deallocation_params(
                    &sample->bounding_boxes_,deallocParams);
                yolo_msgs::msg::dds_::BoundingBox_Seq_finalize(&sample->bounding_boxes_);

            }

            void BoundingBoxes__finalize_optional_members(
                BoundingBoxes_* sample, RTIBool deletePointers)
            {
                struct DDS_TypeDeallocationParams_t deallocParamsTmp =
                DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;
                struct DDS_TypeDeallocationParams_t * deallocParams =
                &deallocParamsTmp;

                if (sample==NULL) {
                    return;
                } 
                if (deallocParams) {} /* To avoid warnings */

                deallocParamsTmp.delete_pointers = (DDS_Boolean)deletePointers;
                deallocParamsTmp.delete_optional_members = DDS_BOOLEAN_TRUE;

                {
                    DDS_UnsignedLong i, length;
                    length = yolo_msgs::msg::dds_::BoundingBox_Seq_get_length(
                        &sample->bounding_boxes_);

                    for (i = 0; i < length; i++) {
                        yolo_msgs::msg::dds_::BoundingBox__finalize_optional_members(
                            yolo_msgs::msg::dds_::BoundingBox_Seq_get_reference(
                                &sample->bounding_boxes_, i), deallocParams->delete_pointers);
                    }
                }  

            }

            RTIBool BoundingBoxes__copy(
                BoundingBoxes_* dst,
                const BoundingBoxes_* src)
            {
                try {

                    if (dst == NULL || src == NULL) {
                        return RTI_FALSE;
                    }

                    if (!yolo_msgs::msg::dds_::BoundingBox_Seq_copy(&dst->bounding_boxes_ ,
                    &src->bounding_boxes_ )) {
                        return RTI_FALSE;
                    }

                    return RTI_TRUE;

                } catch (std::bad_alloc&) {
                    return RTI_FALSE;
                }
            }

            /**
            * <<IMPLEMENTATION>>
            *
            * Defines:  TSeq, T
            *
            * Configure and implement 'BoundingBoxes_' sequence class.
            */
            #define T BoundingBoxes_
            #define TSeq BoundingBoxes_Seq

            #define T_initialize_w_params yolo_msgs::msg::dds_::BoundingBoxes__initialize_w_params

            #define T_finalize_w_params   yolo_msgs::msg::dds_::BoundingBoxes__finalize_w_params
            #define T_copy       yolo_msgs::msg::dds_::BoundingBoxes__copy

            #ifndef NDDS_STANDALONE_TYPE
            #include "dds_c/generic/dds_c_sequence_TSeq.gen"
            #include "dds_cpp/generic/dds_cpp_sequence_TSeq.gen"
            #else
            #include "dds_c_sequence_TSeq.gen"
            #include "dds_cpp_sequence_TSeq.gen"
            #endif

            #undef T_copy
            #undef T_finalize_w_params

            #undef T_initialize_w_params

            #undef TSeq
            #undef T
        } /* namespace dds_  */
    } /* namespace msg  */
} /* namespace yolo_msgs  */

