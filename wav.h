#ifndef WAV_INCLUDED
#define WAV_INCLUDED

#include <stdint.h>

/* PCM类型——不管文件的位数如何，都在内存中以有符号的32位处理 */
typedef int32_t WAVPcmData;

/* WAV数据格式 */
typedef enum WAVDataFormatTag {
  WAV_DATA_FORMAT_PCM             /* 仅支持 PCM */
} WAVDataFormat;

/* 结果类型 */
typedef enum WAVApiResultTag {
  WAV_APIRESULT_OK = 0,
  WAV_APIRESULT_NG,
  WAV_APIRESULT_INVALID_FORMAT,     /* 非法格式 */
  WAV_APIRESULT_IOERROR,            /* 文件 I/O 错误 */
  WAV_APIRESULT_INVALID_PARAMETER   /* 参数错误 */
} WAVApiResult;

/* WAV文件格式 */
struct WAVFileFormat {
  WAVDataFormat data_format;      /* 数据格式 */
  uint32_t      num_channels;     /* 通道数 */
  uint32_t      sampling_rate;    /* 采样率 */
  uint32_t      bits_per_sample;  /* 量化位数 */
  uint32_t      num_samples;      /* 样本数 */
};

/* WAV文件句柄 */
struct WAVFile {
  struct WAVFileFormat  format;   /* 格式 */
  WAVPcmData**          data;     /* 实际数据 */
};

/* 存取器 */
#define WAVFile_PCM(wavfile, samp, ch)  (wavfile->data[(ch)][(samp)])

#ifdef __cplusplus
extern "C" {
#endif

/* 从文件创建WAV文件句柄 */
struct WAVFile* WAV_CreateFromFile(const char* filename);

/* 指定格式新建WAV文件句柄 */
struct WAVFile* WAV_Create(const struct WAVFileFormat* format);

/* 丢弃 WAV 文件句柄 */
void WAV_Destroy(struct WAVFile* wavfile);

/* 文件导出 */
WAVApiResult WAV_WriteToFile(
    const char* filename, const struct WAVFile* wavfile);

/* 从文件中只读 WAV 文件格式 */
WAVApiResult WAV_GetWAVFormatFromFile(
    const char* filename, struct WAVFileFormat* format);

#ifdef __cplusplus
}
#endif

#endif /* WAV_INCLUDED */
