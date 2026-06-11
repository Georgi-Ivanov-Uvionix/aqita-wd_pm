/**
  ******************************************************************************
  * @file    uvx_universal_types.h
  * @brief   Platform DEPENDENT - ARMCC / STM32
  * @brief   This file contains Universal data types.	
	******************************************************************************
  *
  * COPYRIGHT(c) 2018 Uvionix Aerospace Corporation
  *
  ******************************************************************************
  */

// Define to prevent recursive inclusion 
#ifndef __UVX_UNIVERSAL_TYPES_H
#define __UVX_UNIVERSAL_TYPES_H



// UNIVERSAL DATA TYPES

// exact-width signed integer types
typedef   signed           char S8;
typedef   signed short     int S16;
typedef   signed           int S32;
typedef   signed     long long S64;
        
typedef   float                F32;
typedef   double               F64;

// exact-width unsigned integer types
typedef unsigned           char U8;
typedef unsigned short     int U16;
typedef unsigned           int U32;
typedef unsigned     long long U64;

typedef   S32				S15_16;
typedef   S32				 S0_31;
typedef   S64				 S0_63;

#define    __IO volatile
    
//#ifdef __ARMCC_VERSION
//#define packed_struct __packed struct
//#define packed_union __packed union
//#else
#define packed_struct struct __attribute__ ((__packed__))
#define packed_union union __attribute__ ((__packed__))
//#endif

typedef packed_struct Q32
{
	F32 q[ 4 ];
	
} Q32;

typedef packed_struct Q64
{
	F64 q[ 4 ];
	
} Q64;

typedef packed_struct E32
{
	F32 Yaw;
	F32 Pitch;
	F32 Roll;
	U16 Sequence;     // Rotation sequence 1-Roll, 2-Pitch, 3-Yaw. Can be 123, 132, 312, etc... - 12 different possibilities
	
} E32;

typedef packed_struct E64
{
	F64 Yaw;
	F64 Pitch;
	F64 Roll;
	U16 Sequence;     // Rotation sequence 1-Roll, 2-Pitch, 3-Yaw. Can be 123, 132, 312, etc... - 12 different possibilities
	
} E64;

typedef packed_struct VECTOR_2D_F32
{
	F32           x;
	F32           y;
	
} VECTOR_2D_F32;

typedef packed_struct VECTOR_3D_U8
{
	U8           x;
	U8           y;
	U8           z;
	
} VECTOR_3D_U8;

typedef packed_struct VECTOR_3D_F32
{
	F32           x;
	F32           y;
	F32           z;
	
} VECTOR_3D_F32;

typedef packed_struct VECTOR_2D_F64
{
	F64           x;
	F64           y;
	
} VECTOR_2D_F64;

typedef packed_struct VECTOR_3D_F64
{
	F64           x;
	F64           y;
	F64           z;
	
} VECTOR_3D_F64;

typedef packed_struct MATRIX_3X3_F32
{
	VECTOR_3D_F32		row1;
	VECTOR_3D_F32		row2;
	VECTOR_3D_F32		row3;
	
} MATRIX_3X3_F32;

typedef packed_struct MATRIX_3X3_F64
{
	VECTOR_3D_F64		row1;
	VECTOR_3D_F64		row2;
	VECTOR_3D_F64		row3;

} MATRIX_3X3_F64;

typedef packed_struct MATRIX_2X2_F32
{
	VECTOR_2D_F32		row1;
	VECTOR_2D_F32		row2;
    
} MATRIX_2X2_F32;

typedef packed_struct MATRIX_2X2_F64
{
	VECTOR_2D_F64		row1;	
	VECTOR_2D_F64		row2;

} MATRIX_2X2_F64;


#endif
