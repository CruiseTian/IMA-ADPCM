#ifndef BYTEARRAY_H_INCLUDED
#define BYTEARRAY_H_INCLUDED

#include <stdint.h>

/* 1 字节读取 */
#define ByteArray_ReadUint8(p_array)                    \
  (uint8_t)((p_array)[0])

/* 2 字节读取（大端） */
#define ByteArray_ReadUint16BE(p_array)                 \
  (uint16_t)(                                           \
   (((uint16_t)((p_array)[0])) << 8) |                  \
   (((uint16_t)((p_array)[1])) << 0)                    \
  )

/* 4 字节读取（大端） */
#define ByteArray_ReadUint32BE(p_array)                 \
  (uint32_t)(                                           \
   (((uint32_t)((p_array)[0])) << 24) |                 \
   (((uint32_t)((p_array)[1])) << 16) |                 \
   (((uint32_t)((p_array)[2])) <<  8) |                 \
   (((uint32_t)((p_array)[3])) <<  0)                   \
  )

/* 2 字节读取（小端） */
#define ByteArray_ReadUint16LE(p_array)                 \
  (uint16_t)(                                           \
   (((uint16_t)((p_array)[0])) << 0) |                  \
   (((uint16_t)((p_array)[1])) << 8)                    \
  )

/* 4 字节读取（小端） */
#define ByteArray_ReadUint32LE(p_array)                 \
  (uint32_t)(                                           \
   (((uint32_t)((p_array)[0])) <<  0) |                 \
   (((uint32_t)((p_array)[1])) <<  8) |                 \
   (((uint32_t)((p_array)[2])) << 16) |                 \
   (((uint32_t)((p_array)[3])) << 24)                   \
  )

/* 获取 1 个字节 */
#define ByteArray_GetUint8(p_array, p_u8val) {          \
  (*(p_u8val)) = ByteArray_ReadUint8(p_array);          \
  (p_array) += 1;                                       \
}

/* 获取 2 个字节（大端） */
#define ByteArray_GetUint16BE(p_array, p_u16val) {      \
  (*(p_u16val)) = ByteArray_ReadUint16BE(p_array);      \
  (p_array) += 2;                                       \
}

/* 获取 4 个字节（大端） */
#define ByteArray_GetUint32BE(p_array, p_u32val) {      \
  (*(p_u32val)) = ByteArray_ReadUint32BE(p_array);      \
  (p_array) += 4;                                       \
}

/* 获取 2 个字节（小端） */
#define ByteArray_GetUint16LE(p_array, p_u16val) {      \
  (*(p_u16val)) = ByteArray_ReadUint16LE(p_array);      \
  (p_array) += 2;                                       \
}

/* 获取 4 个字节（小端） */
#define ByteArray_GetUint32LE(p_array, p_u32val) {      \
  (*(p_u32val)) = ByteArray_ReadUint32LE(p_array);      \
  (p_array) += 4;                                       \
}

/* 写入 1 个字节 */
#define ByteArray_WriteUint8(p_array, u8val)   {        \
  ((p_array)[0]) = (uint8_t)(u8val);                    \
}

/* 2 字节导出（大端） */
#define ByteArray_WriteUint16BE(p_array, u16val) {      \
  ((p_array)[0]) = (uint8_t)(((u16val) >> 8) & 0xFF);   \
  ((p_array)[1]) = (uint8_t)(((u16val) >> 0) & 0xFF);   \
}

/* 4 字节导出（大端） */
#define ByteArray_WriteUint32BE(p_array, u32val) {      \
  ((p_array)[0]) = (uint8_t)(((u32val) >> 24) & 0xFF);  \
  ((p_array)[1]) = (uint8_t)(((u32val) >> 16) & 0xFF);  \
  ((p_array)[2]) = (uint8_t)(((u32val) >>  8) & 0xFF);  \
  ((p_array)[3]) = (uint8_t)(((u32val) >>  0) & 0xFF);  \
}

/* 2 字节写入（小端） */
#define ByteArray_WriteUint16LE(p_array, u16val) {      \
  ((p_array)[0]) = (uint8_t)(((u16val) >> 0) & 0xFF);   \
  ((p_array)[1]) = (uint8_t)(((u16val) >> 8) & 0xFF);   \
}

/* 4 字节导出（小端） */
#define ByteArray_WriteUint32LE(p_array, u32val) {      \
  ((p_array)[0]) = (uint8_t)(((u32val) >>  0) & 0xFF);  \
  ((p_array)[1]) = (uint8_t)(((u32val) >>  8) & 0xFF);  \
  ((p_array)[2]) = (uint8_t)(((u32val) >> 16) & 0xFF);  \
  ((p_array)[3]) = (uint8_t)(((u32val) >> 24) & 0xFF);  \
}

/* 1 字节输出 */
#define ByteArray_PutUint8(p_array, u8val) {            \
  ByteArray_WriteUint8(p_array, u8val);                 \
  (p_array) += 1;                                       \
}

/* 2 字节输出（大端） */
#define ByteArray_PutUint16BE(p_array, u16val) {        \
  ByteArray_WriteUint16BE(p_array, u16val);             \
  (p_array) += 2;                                       \
}

/* 4 字节输出（大端） */
#define ByteArray_PutUint32BE(p_array, u32val) {        \
  ByteArray_WriteUint32BE(p_array, u32val);             \
  (p_array) += 4;                                       \
}

/* 2 字节输出（小端） */
#define ByteArray_PutUint16LE(p_array, u16val) {        \
  ByteArray_WriteUint16LE(p_array, u16val);             \
  (p_array) += 2;                                       \
}

/* 4 字节输出（小端） */
#define ByteArray_PutUint32LE(p_array, u32val) {        \
  ByteArray_WriteUint32LE(p_array, u32val);             \
  (p_array) += 4;                                       \
}

#endif /* BYTEARRAY_H_INCLUDED */
