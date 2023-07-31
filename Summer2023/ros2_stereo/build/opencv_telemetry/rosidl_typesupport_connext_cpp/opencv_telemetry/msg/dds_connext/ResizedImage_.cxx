

/*
WARNING: THIS FILE IS AUTO-GENERATED. DO NOT MODIFY.

This file was generated from ResizedImage_.idl using "rtiddsgen".
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

#include "ResizedImage_.h"

#include <new>

namespace opencv_telemetry {
    namespace msg {
        namespace dds_ {

            /* ========================================================================= */
            const char *ResizedImage_TYPENAME = "opencv_telemetry::msg::dds_::ResizedImage_";

            DDS_TypeCode* ResizedImage__get_typecode()
            {
                static RTIBool is_initialized = RTI_FALSE;

                static DDS_TypeCode_Member ResizedImage__g_tc_members[3]=
                {

                    {
                        (char *)"original_height_",/* Member name */
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
                    }, 
                    {
                        (char *)"original_width_",/* Member name */
                        {
                            1,/* Representation ID */          
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
                    }, 
                    {
                        (char *)"image_",/* Member name */
                        {
                            2,/* Representation ID */          
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

                static DDS_TypeCode ResizedImage__g_tc =
                {{
                        DDS_TK_STRUCT,/* Kind */
                        DDS_BOOLEAN_FALSE, /* Ignored */
                        -1, /*Ignored*/
                        (char *)"opencv_telemetry::msg::dds_::ResizedImage_", /* Name */
                        NULL, /* Ignored */      
                        0, /* Ignored */
                        0, /* Ignored */
                        NULL, /* Ignored */
                        3, /* Number of members */
                        ResizedImage__g_tc_members, /* Members */
                        DDS_VM_NONE  /* Ignored */         
                    }}; /* Type code for ResizedImage_*/

                if (is_initialized) {
                    return &ResizedImage__g_tc;
                }

                ResizedImage__g_tc_members[0]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_ulong;

                ResizedImage__g_tc_members[1]._representation._typeCode = (RTICdrTypeCode *)&DDS_g_tc_ulong;

                ResizedImage__g_tc_members[2]._representation._typeCode = (RTICdrTypeCode *)sensor_msgs::msg::dds_::Image__get_typecode();

                is_initialized = RTI_TRUE;

                return &ResizedImage__g_tc;
            }

            RTIBool ResizedImage__initialize(
                ResizedImage_* sample) {
                return opencv_telemetry::msg::dds_::ResizedImage__initialize_ex(sample,RTI_TRUE,RTI_TRUE);
            }

            RTIBool ResizedImage__initialize_ex(
                ResizedImage_* sample,RTIBool allocatePointers, RTIBool allocateMemory)
            {

                struct DDS_TypeAllocationParams_t allocParams =
                DDS_TYPE_ALLOCATION_PARAMS_DEFAULT;

                allocParams.allocate_pointers =  (DDS_Boolean)allocatePointers;
                allocParams.allocate_memory = (DDS_Boolean)allocateMemory;

                return opencv_telemetry::msg::dds_::ResizedImage__initialize_w_params(
                    sample,&allocParams);

            }

            RTIBool ResizedImage__initialize_w_params(
                ResizedImage_* sample, const struct DDS_TypeAllocationParams_t * allocParams)
            {

                if (sample == NULL) {
                    return RTI_FALSE;
                }
                if (allocParams == NULL) {
                    return RTI_FALSE;
                }

                if (!RTICdrType_initUnsignedLong(&sample->original_height_)) {
                    return RTI_FALSE;
                }

                if (!RTICdrType_initUnsignedLong(&sample->original_width_)) {
                    return RTI_FALSE;
                }

                if (!sensor_msgs::msg::dds_::Image__initialize_w_params(&sample->image_,
                allocParams)) {
                    return RTI_FALSE;
                }
                return RTI_TRUE;
            }

            void ResizedImage__finalize(
                ResizedImage_* sample)
            {

                opencv_telemetry::msg::dds_::ResizedImage__finalize_ex(sample,RTI_TRUE);
            }

            void ResizedImage__finalize_ex(
                ResizedImage_* sample,RTIBool deletePointers)
            {
                struct DDS_TypeDeallocationParams_t deallocParams =
                DDS_TYPE_DEALLOCATION_PARAMS_DEFAULT;

                if (sample==NULL) {
                    return;
                } 

                deallocParams.delete_pointers = (DDS_Boolean)deletePointers;

                opencv_telemetry::msg::dds_::ResizedImage__finalize_w_params(
                    sample,&deallocParams);
            }

            void ResizedImage__finalize_w_params(
                ResizedImage_* sample,const struct DDS_TypeDeallocationParams_t * deallocParams)
            {

                if (sample==NULL) {
                    return;
                }

                if (deallocParams == NULL) {
                    return;
                }

                sensor_msgs::msg::dds_::Image__finalize_w_params(&sample->image_,deallocParams);

            }

            void ResizedImage__finalize_optional_members(
                ResizedImage_* sample, RTIBool deletePointers)
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

                sensor_msgs::msg::dds_::Image__finalize_optional_members(&sample->image_, deallocParams->delete_pointers);
            }

            RTIBool ResizedImage__copy(
                ResizedImage_* dst,
                const ResizedImage_* src)
            {
                try {

                    if (dst == NULL || src == NULL) {
                        return RTI_FALSE;
                    }

                    if (!RTICdrType_copyUnsignedLong (
                        &dst->original_height_, &src->original_height_)) { 
                        return RTI_FALSE;
                    }
                    if (!RTICdrType_copyUnsignedLong (
                        &dst->original_width_, &src->original_width_)) { 
                        return RTI_FALSE;
                    }
                    if (!sensor_msgs::msg::dds_::Image__copy(
                        &dst->image_,(const sensor_msgs::msg::dds_::Image_*)&src->image_)) {
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
            * Configure and implement 'ResizedImage_' sequence class.
            */
            #define T ResizedImage_
            #define TSeq ResizedImage_Seq

            #define T_initialize_w_params opencv_telemetry::msg::dds_::ResizedImage__initialize_w_params

            #define T_finalize_w_params   opencv_telemetry::msg::dds_::ResizedImage__finalize_w_params
            #define T_copy       opencv_telemetry::msg::dds_::ResizedImage__copy

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
} /* namespace opencv_telemetry  */

