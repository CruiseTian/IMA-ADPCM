#ifndef IMAADPCM_H_INCLDED
#define IMAADPCM_H_INCLDED

#include <stdint.h>

/* 可处理的最大通道数 */
#define IMAADPCM_MAX_NUM_CHANNELS       2

/* 每个样本的位数固定为 4 */
#define IMAADPCM_BITS_PER_SAMPLE        4

/* 结果类型 */
typedef enum IMAADPCMApiResultTag {
  IMAADPCM_APIRESULT_OK = 0,              /* 成功 */
  IMAADPCM_APIRESULT_INVALID_ARGUMENT,    /* 无效参数 */
  IMAADPCM_APIRESULT_INVALID_FORMAT,      /* 格式错误 */
  IMAADPCM_APIRESULT_INSUFFICIENT_BUFFER, /* 缓冲区大小不足 */
  IMAADPCM_APIRESULT_INSUFFICIENT_DATA,   /* 数据不足 */
  IMAADPCM_APIRESULT_PARAMETER_NOT_SET,   /* 未设置参数 */
  IMAADPCM_APIRESULT_NG                   /* 无法分类错误 */
} IMAADPCMApiResult; 

/* IMA-ADPCM格式的wav文件头信息 */
struct IMAADPCMWAVHeaderInfo {
  uint16_t num_channels;          /* 通道数 */
  uint32_t sampling_rate;         /* 采样率 */
  uint32_t bytes_per_sec;         /* 数据速度[byte/sec] */
  uint16_t block_size;            /* 块大小 */
  uint16_t bits_per_sample;       /* 每个样本位数 */
  uint16_t num_samples_per_block; /* 每块样本数 */
  uint32_t num_samples;           /* 每通道总样本数 */
  uint32_t header_size;           /* 从文件开头到数据区域开头的偏移量 */
};

/* 编码参数 */
struct IMAADPCMWAVEncodeParameter {
  uint16_t num_channels;          /* 通道数 */
  uint32_t sampling_rate;         /* 采样率 */
  uint16_t bits_per_sample;       /* 每个样本的位数（目前固定为 4） */
  uint16_t block_size;            /* 块大小[byte] */
};

/* 解码器句柄 */
struct IMAADPCMWAVDecoder;

/* 编码器句柄 */
struct IMAADPCMWAVEncoder;

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* 标头解码 */
IMAADPCMApiResult IMAADPCMWAVDecoder_DecodeHeader(
    const uint8_t *data, uint32_t data_size, struct IMAADPCMWAVHeaderInfo *header_info);

/* 标头编码 */
IMAADPCMApiResult IMAADPCMWAVEncoder_EncodeHeader(
    const struct IMAADPCMWAVHeaderInfo *header_info, uint8_t *data, uint32_t data_size);

/* 解码器工作大小计算 */
int32_t IMAADPCMWAVDecoder_CalculateWorkSize(void);

/* 解码器句柄创建 */
struct IMAADPCMWAVDecoder *IMAADPCMWAVDecoder_Create(void *work, int32_t work_size);

/* 丢弃解码器句柄 */
void IMAADPCMWAVDecoder_Destroy(struct IMAADPCMWAVDecoder *decoder);

/* 解码包括标题在内的整个文件 */
IMAADPCMApiResult IMAADPCMWAVDecoder_DecodeWhole(
    struct IMAADPCMWAVDecoder *decoder,
    const uint8_t *data, uint32_t data_size,
    int16_t **buffer, uint32_t buffer_num_channels, uint32_t buffer_num_samples);

int32_t IMAADPCMWAVEncoder_CalculateWorkSize(void);

/* 编码器句柄创建 */
struct IMAADPCMWAVEncoder *IMAADPCMWAVEncoder_Create(void *work, int32_t work_size);

/* 丢弃编码器句柄 */
void IMAADPCMWAVEncoder_Destroy(struct IMAADPCMWAVEncoder *encoder);

/* 编码参数设置 */
IMAADPCMApiResult IMAADPCMWAVEncoder_SetEncodeParameter(
    struct IMAADPCMWAVEncoder *encoder, const struct IMAADPCMWAVEncodeParameter *parameter);

/* 对包括标题在内的整个文件进行编码 */
IMAADPCMApiResult IMAADPCMWAVEncoder_EncodeWhole(
    struct IMAADPCMWAVEncoder *encoder,
    const int16_t *const *input, uint32_t num_samples,
    uint8_t *data, uint32_t data_size, uint32_t *output_size);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* IMAADPCM_H_INCLDED */
