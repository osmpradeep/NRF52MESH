
/* This file was automatically generated by nrfutil on 2018-03-22 (YY-MM-DD) at 12:39:00 */

#include "stdint.h"
#include "compiler_abstraction.h"

/* This file was generated with a throwaway private key, that is only inteded for a debug version of the DFU project.
  Please see https://github.com/NordicSemiconductor/pc-nrfutil/blob/master/README.md to generate a valid public key. */

#ifdef NRF_DFU_DEBUG_VERSION 

///** @brief Public key used to verify DFU images */
//__ALIGN(4) const uint8_t pk[64] =
//{
//    0x65, 0xfc, 0x2b, 0xf8, 0x16, 0xda, 0x01, 0x9a, 0xde, 0x75, 0xd4, 0xa8, 0x47, 0x71, 0x96, 0x21, 0x1c, 0x87, 0x55, 0x69, 0x10, 0x5a, 0xbe, 0x04, 0x57, 0x8f, 0xd2, 0xb0, 0x29, 0x94, 0x56, 0xb8, 
//    0xca, 0x03, 0x63, 0xd6, 0x35, 0xc6, 0x4d, 0xa7, 0x81, 0x9f, 0xef, 0xa7, 0xd2, 0xec, 0xb7, 0xf3, 0x07, 0xff, 0x80, 0xaf, 0x7a, 0x3b, 0x4c, 0x19, 0xb8, 0x91, 0xc5, 0x14, 0x3c, 0xe8, 0x76, 0x6d
//};

/** @brief Public key used to verify DFU images */
__ALIGN(4) const uint8_t pk[64] =
{
    0x34, 0xd1, 0xfb, 0x38, 0x22, 0x63, 0xc6, 0x42, 0x11, 0x94, 0xaf, 0x8e, 0xd3, 0x4a, 0x59, 0x8d, 0xc4, 0x78, 0xfb, 0x34, 0x88, 0x69, 0xab, 0x3c, 0x30, 0x80, 0xd5, 0x42, 0x6f, 0x28, 0x7c, 0xc6, 
    0x07, 0x4a, 0xdf, 0x8f, 0x09, 0x02, 0x38, 0xc2, 0x0f, 0xba, 0x86, 0x0a, 0x96, 0xf7, 0x17, 0x9c, 0x10, 0x72, 0x9c, 0xa9, 0xd3, 0x92, 0x52, 0xda, 0x13, 0xca, 0xd7, 0x51, 0xa4, 0xeb, 0x34, 0x72
};



#else

/* This file was automatically generated by nrfutil on 2018-05-16 (YY-MM-DD) at 12:24:07 */

//#include "stdint.h"
//#include "compiler_abstraction.h"

/** @brief Public key used to verify DFU images */
__ALIGN(4) const uint8_t pk[64] =
{
    0x34, 0xd1, 0xfb, 0x38, 0x22, 0x63, 0xc6, 0x42, 0x11, 0x94, 0xaf, 0x8e, 0xd3, 0x4a, 0x59, 0x8d, 0xc4, 0x78, 0xfb, 0x34, 0x88, 0x69, 0xab, 0x3c, 0x30, 0x80, 0xd5, 0x42, 0x6f, 0x28, 0x7c, 0xc6, 
    0x07, 0x4a, 0xdf, 0x8f, 0x09, 0x02, 0x38, 0xc2, 0x0f, 0xba, 0x86, 0x0a, 0x96, 0xf7, 0x17, 0x9c, 0x10, 0x72, 0x9c, 0xa9, 0xd3, 0x92, 0x52, 0xda, 0x13, 0xca, 0xd7, 0x51, 0xa4, 0xeb, 0x34, 0x72
};

//#error "Debug public key not valid for production. Please see https://github.com/NordicSemiconductor/pc-nrfutil/blob/master/README.md to generate it"
#endif
