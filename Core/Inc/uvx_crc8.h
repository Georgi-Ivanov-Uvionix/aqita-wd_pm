
/* Define to prevent recursive inclusion */
#ifndef _UVX_CRC8
#define _UVX_CRC8

#define CRC8_INIT_VALUE			0xFF
#define CRC8_FINAL_XOR_VALUE	0xFF

#define CRC8_TABLE_SIZE			256
#define CRC8_POLY_AUTOSAR		0x2F
#define CRC8_POLY_SAE_J1850		0x1D

#define CRC16_INIT_VALUE		0xFFFF
#define CRC16_FINAL_XOR_VALUE	0x0000
#define CRC16_POLY_CCITT		0x1021

#include "uvx_universal_types.h"

void calculate_table_CRC8(U8 crc8_poly, U8* crc8_table);
U8 compute_CRC8(U8* data, U32 num_bytes, U8* crc8_table, U8 crc8_final_xor_value);
U8 uvx_crc_8(U8* data, U32 num_bytes);
U16 uvx_crc_16(U8* data, U32 num_bytes);
U8 compute_CRC8_no_LUT(U8* data, U32 num_bytes, U8 crc8_poly, U8 crc8_final_xor_value);

#endif /* _UVX_CRC8 */
