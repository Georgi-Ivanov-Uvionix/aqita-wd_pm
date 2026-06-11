
#include "uvx_crc8.h"

void calculate_table_CRC8(U8 crc8_poly, U8* crc8_table)
{
	// iterate over all byte values in the range 0 ~ 255
	for ( U32 divident = 0; divident < 256; divident++ )
	{
		U8 currByte = (U8)divident;

		// calculate the CRC8 value for the current byte
		for ( U8 bit = 0; bit < 8; bit++ )
		{
			if ( (currByte & 0x80) != 0 )
			{
				currByte <<= 1;
				currByte ^= crc8_poly;
			}
			else
			{
				currByte <<= 1;
			}
		}

		// store the CRC8 value in the lookup table
		crc8_table[divident] = currByte;
	}
}

U8 compute_CRC8(U8* data, U32 num_bytes, U8* crc8_table, U8 crc8_final_xor_value)
{
	U8 crc = CRC8_INIT_VALUE;

	for ( U32 i = 0; i < num_bytes; i++ )
	{
		// XOR-in the next byte
		U8 index = (U8)(data[i] ^ crc);
		
		// get the CRC8 value from the table
		crc = (U8)(crc8_table[index]);
	}

	return (crc ^ crc8_final_xor_value);
}

U8 uvx_crc_8(U8* data, U32 num_bytes)
{
	U8 crc = 0;
	U16 i = 0;

    for (i = 0; i < num_bytes; i++) 
	{
        crc ^= data[i];
    }
	
	crc ^= CRC8_FINAL_XOR_VALUE; // Apply final XOR value
	
    return crc;
}

U16 uvx_crc_16(U8* data, U32 num_bytes)
{
	U16 crc = CRC16_INIT_VALUE;

	for (U32 i = 0; i < num_bytes; i++)
	{
		crc ^= (U16)data[i] << 8;

		for (U8 bit = 0; bit < 8; bit++)
		{
			if ((crc & 0x8000) != 0)
			{
				crc = (U16)((crc << 1) ^ CRC16_POLY_CCITT);
			}
			else
			{
				crc <<= 1;
			}
		}
	}

	return (crc ^ CRC16_FINAL_XOR_VALUE);
}

U8 compute_CRC8_no_LUT(U8* data, U32 num_bytes, U8 crc8_poly, U8 crc8_final_xor_value)
{
	U8 crc = CRC8_INIT_VALUE;

	for ( U32 i = 0; i < num_bytes; i++ )
	{
		crc ^= data[i];

		for ( U8 bit = 0; bit < 8; bit++ )
		{
			if ( (crc & 0x80) != 0 )
			{
				crc = (U8)((crc << 1) ^ crc8_poly);
			}
			else
			{
				crc <<= 1;
			}
		}
	}

	return (crc ^ crc8_final_xor_value);
}
