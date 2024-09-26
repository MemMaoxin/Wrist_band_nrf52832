#ifndef SI114X_TYPES_H__

#define	SI114X_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif



typedef signed char       s8;
typedef signed short      s16;
typedef signed int        s32;
typedef unsigned char     u8;
typedef unsigned short    u16;
typedef unsigned int      u32;

typedef signed char       S8;
typedef signed short      S16;
typedef signed int        S32;
typedef unsigned char     U8;
typedef unsigned short    U16;
typedef unsigned int      U32;

typedef void *            HANDLE;
typedef char *            STRING;
typedef s16               PT_RESULT;
typedef s8                PT_BOOL;



#define code
#define xdata


#define LSB 0
#define MSB 1
#define b0  0
//#define b1  1		//ToDo:  these were commented out becasue they conflicted with code in arm_math.h...I would like to remove all dependencied on si114x_types.h
//#define b2  2
//#define b3  3

typedef union uu16
{
    u16 u16;
    s16 s16;
    u8  u8[2];
    s8  s8[2];
} uu16;


typedef union uu32
{
    u32  u32;
    s32  s32;
    uu16 uu16[2];
    u16  u16[2];
    s16  s16[2];
    u8   u8[4];
    s8   s8[4];

} uu32;


typedef char BIT;

#ifndef TRUE
#define TRUE   0xff
#endif

#ifndef FALSE
#define FALSE  0
#endif

#ifndef NULL
#define NULL 0
#endif

typedef enum {
    i2c_return_error,
    i2c_return_ok,
}I2C_TransferReturn_TypeDef;

#ifdef __cplusplus
}
#endif

#endif		//SI114X_TYPES_H__
