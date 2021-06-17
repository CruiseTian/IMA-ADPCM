#include "wav.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

/* 解析器读取缓冲区大小 */
#define WAVBITBUFFER_BUFFER_SIZE         (10 * 1024)

/* 获得更低的 n_bits */
/* 补充）((1 << n_bits) - 1)是只取出较低数值的掩码 */
#define WAV_GetLowerBits(n_bits, val) ((val) & (uint32_t)((1 << (n_bits)) - 1))

/* 内部错误类型 */
typedef enum WAVErrorTag {
  WAV_ERROR_OK = 0,             /* OK */
  WAV_ERROR_NG,                 /* 分类错误 */
  WAV_ERROR_IO,                 /* IO错误 */
  WAV_ERROR_INVALID_PARAMETER,  /* 错误参数 */
  WAV_ERROR_INVALID_FORMAT      /* 错误格式 */
} WAVError;

/* 位缓冲 */
struct WAVBitBuffer {
  uint8_t   bytes[WAVBITBUFFER_BUFFER_SIZE];   /* 位缓冲 */
  uint32_t  bit_count;                        /* 位输入计数 */
  int32_t   byte_pos;                         /* 字符串读取位置 */
};

/* 解析器 */
struct WAVParser {
  FILE*               fp;       /* 读取文件指针 */
  struct WAVBitBuffer buffer;   /* 位缓冲 */
};

/* 写入 */
struct WAVWriter {
  FILE*     fp;                 /* 写入文件指针 */
  uint32_t  bit_buffer;         /* 输出中间的位 */
  uint32_t  bit_count;          /* 输出计数 */
  struct WAVBitBuffer buffer;   /* 位缓冲 */
};

/* 解析器初始化 */
static void WAVParser_Initialize(struct WAVParser* parser, FILE* fp);
/* 解析器使用结束 */
static void WAVParser_Finalize(struct WAVParser* parser);
/* 获取 n_bit 并右对齐结果 */
static WAVError WAVParser_GetBits(struct WAVParser* parser, uint32_t n_bits, uint64_t* bitsbuf);
/* 搜索（fseek） */
static WAVError WAVParser_Seek(struct WAVParser* parser, int32_t offset, int32_t wherefrom);
/* 写入初始化 */
static void WAVWriter_Initialize(struct WAVWriter* writer, FILE* fp);
/* 写入结束 */
static void WAVWriter_Finalize(struct WAVWriter* writer);
/* 写入 val 的低 n_bit */
static WAVError WAVWriter_PutBits(struct WAVWriter* writer, uint64_t val, uint32_t n_bits);
/* 清除缓冲区中累积的位 */
static WAVError WAVWriter_Flush(struct WAVWriter* writer);
/* 小端输出位模式 */
static WAVError WAVWriter_PutLittleEndianBytes(
    struct WAVWriter* writer, uint32_t nbytes, uint64_t data);

/* 根据文件格式输出头部分导入 */
static WAVError WAVWriter_PutWAVHeader(
    struct WAVWriter* writer, const struct WAVFileFormat* format);
/* 使用捕获的 PCM 数据输出 */
static WAVError WAVWriter_PutWAVPcmData(
    struct WAVWriter* writer, const struct WAVFile* wavfile);

/* 使用小端获取位模式 */
static WAVError WAVParser_GetLittleEndianBytes(
    struct WAVParser* parser, uint32_t nbytes, uint64_t* bitsbuf);
/* 使用解析器获取字符串 */
static WAVError WAVParser_GetString(
  struct WAVParser* parser, char* string_buffer, uint32_t string_length);
/* 使用解析器获取字符串/匹配检查 */
static WAVError WAVParser_CheckSignatureString(
  struct WAVParser* parser, const char* signature, uint32_t signature_length);
/* 使用解析器读取文件格式 */
static WAVError WAVParser_GetWAVFormat(
    struct WAVParser* parser, struct WAVFileFormat* format);
/* 使用解析器读取 PCM 数据 */
static WAVError WAVParser_GetWAVPcmData(
    struct WAVParser* parser, struct WAVFile* wavfile);

/* 将 8bit PCM 格式转换为 32bit 格式 */
static int32_t WAV_Convert8bitPCMto32bitPCM(int32_t in_8bitpcm);
/* 将 16bit PCM 格式转换为 32bit 格式 */
static int32_t WAV_Convert16bitPCMto32bitPCM(int32_t in_16bitpcm);
/* 将 24bit PCM 格式转换为 32bit 格式 */
static int32_t WAV_Convert24bitPCMto32bitPCM(int32_t in_24bitpcm);
/* 将 32bit PCM 格式转换为 32bit 格式 */
static int32_t WAV_Convert32bitPCMto32bitPCM(int32_t in_32bitpcm);

/* 将 32bit PCM 格式转换为 8bit 格式（注：返回值为32位整数，但在8位范围内裁剪） */
static int32_t WAV_Convert32bitPCMto8bitPCM(int32_t in_32bitpcm);
/* 将 32bit PCM 格式转换为 16bit 格式（注：返回值为32位整数，但在16位范围内裁剪） */
static int32_t WAV_Convert32bitPCMto8bitPCM(int32_t in_32bitpcm);
/* 将 32bit PCM 格式转换为 24bit 格式（注：返回值为32位整数，但在24位范围内裁剪） */
static int32_t WAV_Convert32bitPCMto16bitPCM(int32_t in_32bitpcm);
/* 将 32bit PCM 格式转换为 32bit 格式 */
static int32_t WAV_Convert32bitPCMto32bitPCM(int32_t in_32bitpcm);

/* 使用解析器读取文件格式 */
static WAVError WAVParser_GetWAVFormat(
    struct WAVParser* parser, struct WAVFileFormat* format)
{
  uint64_t  bitsbuf;
  int32_t   fmt_chunk_size;
  struct WAVFileFormat tmp_format;

  /* 参数检查 */
  if (parser == NULL || format == NULL) {
    return WAV_ERROR_INVALID_PARAMETER;
  }
  
  /* 检查标题'R'，'I'，'F'，'F' */
  if (WAVParser_CheckSignatureString(parser, "RIFF", 4) != WAV_ERROR_OK) {
    return WAV_ERROR_INVALID_FORMAT;
  }

  /* 文件大小-8（跳过） */
  if (WAVParser_GetLittleEndianBytes(parser, 4, &bitsbuf) != WAV_ERROR_OK) { return WAV_ERROR_IO; }

  /* 检查标题'W'、'A'、'V'、'E' */
  if (WAVParser_CheckSignatureString(parser, "WAVE", 4) != WAV_ERROR_OK) {
    return WAV_ERROR_INVALID_FORMAT;
  }

  /* 检查 fmt 块头'f','m','t','' */
  if (WAVParser_CheckSignatureString(parser, "fmt ", 4) != WAV_ERROR_OK) {
    return WAV_ERROR_INVALID_FORMAT;
  }

  /* 获取 fmt 块中的字节数
   * 补充/注意）跳过大于 16 的 fmt 块的内容（扩展） */
  if (WAVParser_GetLittleEndianBytes(parser, 4, &bitsbuf) != WAV_ERROR_OK) { return WAV_ERROR_IO; }
  fmt_chunk_size = (int32_t)bitsbuf;

  /* 检查格式 ID
   * 补充）不兼容 1（线性 PCM）以外的格式 */
  if (WAVParser_GetLittleEndianBytes(parser, 2, &bitsbuf) != WAV_ERROR_OK) { return WAV_ERROR_IO; }
  if (bitsbuf != 1) {
    /* fprintf(stderr, "Unsupported format: fmt chunk format ID \n"); */
    return WAV_ERROR_INVALID_FORMAT;
  }
  tmp_format.data_format = WAV_DATA_FORMAT_PCM;

  /* 通道数 */
  if (WAVParser_GetLittleEndianBytes(parser, 2, &bitsbuf) != WAV_ERROR_OK) { return WAV_ERROR_IO; }
  tmp_format.num_channels = (uint32_t)bitsbuf;

  /* 采样率 */
  if (WAVParser_GetLittleEndianBytes(parser, 4, &bitsbuf) != WAV_ERROR_OK) { return WAV_ERROR_IO; }
  tmp_format.sampling_rate =(uint32_t) bitsbuf;

  /* 数据速度（byte/sec）读取并跳过 */
  if (WAVParser_GetLittleEndianBytes(parser, 4, &bitsbuf) != WAV_ERROR_OK) { return WAV_ERROR_IO; }

  /* 每块读取大小 */
  if (WAVParser_GetLittleEndianBytes(parser, 2, &bitsbuf) != WAV_ERROR_OK) { return WAV_ERROR_IO; }

  /* 量化比特数（每样本的比特数） */
  if (WAVParser_GetLittleEndianBytes(parser, 2, &bitsbuf) != WAV_ERROR_OK) { return WAV_ERROR_IO; }
  tmp_format.bits_per_sample = (uint32_t)bitsbuf;

  /* 不支持扩展部分读取：跳过 */
  if (fmt_chunk_size > 16) {
    fprintf(stderr, "Warning: skip fmt chunk extention (unsupported). \n");
    if (WAVParser_Seek(parser, fmt_chunk_size - 16, SEEK_CUR) != WAV_ERROR_OK) { return WAV_ERROR_IO; }
  }
  
  /* 信息块读取 */
  while (1) {
    char string_buf[4];
    /* 信息块字符串获取 */
    if (WAVParser_GetString(parser, string_buf, 4) != WAV_ERROR_OK) {
      return WAV_ERROR_IO;
    }
    if (strncmp(string_buf, "data", 4) == 0) {
      /* 找到数据块后结束 */
      break;
    } else {
      /* 获取其他信息块的大小并通过检索跳过 */
      if (WAVParser_GetLittleEndianBytes(parser, 4, &bitsbuf) != WAV_ERROR_OK) {
        return WAV_ERROR_IO;
      }
      /* printf("chunk:%s size:%d \n", string_buf, (int32_t)bitsbuf); */
      WAVParser_Seek(parser, (int32_t)bitsbuf, SEEK_CUR);
    }
  }

  /* 样本数：根据波形数据字节数计算 */
  if (WAVParser_GetLittleEndianBytes(parser, 4, &bitsbuf) != WAV_ERROR_OK) { return WAV_ERROR_IO; }
  tmp_format.num_samples = (uint32_t)bitsbuf;
  assert(tmp_format.num_samples % ((tmp_format.bits_per_sample / 8) * tmp_format.num_channels) == 0);
  tmp_format.num_samples /= ((tmp_format.bits_per_sample / 8) * tmp_format.num_channels);

  /* 结构副本 */
  *format = tmp_format;

  return WAV_ERROR_OK;
}

/* 使用解析器读取 PCM 数据 */
static WAVError WAVParser_GetWAVPcmData(
    struct WAVParser* parser, struct WAVFile* wavfile)
{
  uint32_t  ch, sample, bytes_per_sample;
  uint64_t  bitsbuf;
  int32_t   (*convert_to_sint32_func)(int32_t);

  /* 参数检查 */
  if (parser == NULL || wavfile == NULL) {
    return WAV_ERROR_INVALID_PARAMETER;
  }

  /* 根据位深确定PCM数据转换函数 */
  switch (wavfile->format.bits_per_sample) {
    case 8:
      convert_to_sint32_func = WAV_Convert8bitPCMto32bitPCM;
      break;
    case 16:
      convert_to_sint32_func = WAV_Convert16bitPCMto32bitPCM;
      break;
    case 24:
      convert_to_sint32_func = WAV_Convert24bitPCMto32bitPCM;
      break;
    case 32:
      convert_to_sint32_func = WAV_Convert32bitPCMto32bitPCM;
      break;
    default:
      /* fprintf(stderr, "Unsupported bits per sample format(=%d). \n", wavfile->format.bits_per_sample); */
      return WAV_ERROR_INVALID_FORMAT;
  }

  /* 数据读取 */
  bytes_per_sample = wavfile->format.bits_per_sample / 8;
  for (sample = 0; sample < wavfile->format.num_samples; sample++) {
    for (ch = 0; ch < wavfile->format.num_channels; ch++) {
      if (WAVParser_GetLittleEndianBytes(parser, bytes_per_sample, &bitsbuf) != WAV_ERROR_OK) {
        return WAV_ERROR_IO;
      }
      /* 转换成32位整数格式并设置在数据中 */
      wavfile->data[ch][sample] = convert_to_sint32_func((int32_t)(bitsbuf));
    }
  }

  return WAV_ERROR_OK;
}

/* 从文件中只读 WAV 文件格式 */
WAVApiResult WAV_GetWAVFormatFromFile(
    const char* filename, struct WAVFileFormat* format)
{
  struct WAVParser parser;
  FILE*            fp;

  /* 参数检查 */
  if (filename == NULL || format == NULL) {
    return WAV_APIRESULT_NG;
  }
  
  /* 打开.wav文件 */
  fp = fopen(filename, "rb");
  if (fp == NULL) {
    /* fprintf(stderr, "Failed to open %s. \n", filename); */
    return WAV_APIRESULT_NG;
  }

  /* 解析器初始化 */
  WAVParser_Initialize(&parser, fp);

  /* 读取信头 */
  if (WAVParser_GetWAVFormat(&parser, format) != WAV_ERROR_OK) {
    return WAV_APIRESULT_NG;
  }

  /* 解析器使用结束 */
  WAVParser_Finalize(&parser);

  /* 关闭文件 */
  fclose(fp);

  return WAV_APIRESULT_OK;
}

/* 从文件创建 WAV 文件句柄 */
struct WAVFile* WAV_CreateFromFile(const char* filename)
{
  struct WAVParser      parser;
  FILE*                 fp;
  struct WAVFile*       wavfile;
  struct WAVFileFormat  format;

  /* 参数检查 */
  if (filename == NULL) {
    return NULL;
  }
  
  /* 打开.wav文件 */
  fp = fopen(filename, "rb");
  if (fp == NULL) {
    /* fprintf(stderr, "Failed to open %s. \n", filename); */
    return NULL;
  }

  /* 解析器初始化 */
  WAVParser_Initialize(&parser, fp);

  /* 读取信头 */
  if (WAVParser_GetWAVFormat(&parser, &format) != WAV_ERROR_OK) {
    return NULL;
  }

  /* 处理创建 */
  wavfile = WAV_Create(&format);
  if (wavfile == NULL) {
    return NULL;
  }

  /* PCM数据读取 */
  if (WAVParser_GetWAVPcmData(&parser, wavfile) != WAV_ERROR_OK) {
    goto EXIT_FAILURE_WITH_DATA_RELEASE;
  }

  /* 解析器終了 */
  WAVParser_Finalize(&parser);

  /* 关闭文件 */
  fclose(fp);

  /* 正常结束 */
  return wavfile;

  /* 释放由句柄保护的所有数据并完成 */
EXIT_FAILURE_WITH_DATA_RELEASE:
  WAV_Destroy(wavfile);
  WAVParser_Finalize(&parser);
  fclose(fp);
  return NULL;
}

/* 通过指定格式创建新的 WAV 文件句柄 */
struct WAVFile* WAV_Create(const struct WAVFileFormat* format)
{
  uint32_t ch;
  struct WAVFile* wavfile;

  /* 参数检查 */
  if (format == NULL) {
    return NULL;
  }

  /* 目前只支持PCM格式 */
  if (format->data_format != WAV_DATA_FORMAT_PCM) {
    /* fprintf(stderr, "Unsupported wav data format. \n"); */
    return NULL;
  }

  /* 处理创建 */
  wavfile = (struct WAVFile *)malloc(sizeof(struct WAVFile));
  if (wavfile == NULL) {
    goto EXIT_FAILURE_WITH_DATA_RELEASE;
  }

  /* 获取结构副本格式信息 */
  wavfile->format = (*format);

  /* 指定数据区域 */
  wavfile->data = (WAVPcmData **)malloc(sizeof(WAVPcmData *) * format->num_channels);
  if (wavfile->data == NULL) {
    goto EXIT_FAILURE_WITH_DATA_RELEASE;
  }
  for (ch = 0; ch < format->num_channels; ch++) {
    wavfile->data[ch] = (WAVPcmData *)calloc(format->num_samples, sizeof(WAVPcmData));
    if (wavfile->data[ch] == NULL) {
      goto EXIT_FAILURE_WITH_DATA_RELEASE;
    }
  }

  return wavfile;

EXIT_FAILURE_WITH_DATA_RELEASE:
  WAV_Destroy(wavfile);
  return NULL;
}

/* 将 8bit PCM 格式转换为 32bit 格式 */
static int32_t WAV_Convert8bitPCMto32bitPCM(int32_t in_8bitpcm)
{
  /* 减去 128，然后向上取整为 32 位整数。 */
  return (in_8bitpcm - 128) << 24;
}

/* 将 16bit PCM 格式转换为 32bit 格式 */
static int32_t WAV_Convert16bitPCMto32bitPCM(int32_t in_16bitpcm)
{
  /* 原样左移 16 位 */
  return in_16bitpcm << 16;
}

/* 将 24bit PCM 格式转换为 32bit 格式 */
static int32_t WAV_Convert24bitPCMto32bitPCM(int32_t in_24bitpcm)
{
  /* 按原样左移 8 位 */
  return in_24bitpcm << 8;
}

/* 将 32bit PCM 格式转换为 32bit 格式 */
static int32_t WAV_Convert32bitPCMto32bitPCM(int32_t in_32bitpcm)
{
  return in_32bitpcm;
}

/* 将 32bit PCM 格式转换为 8bit 格式（注意：返回值是一个 32 位整数，但它被裁剪在 8 位范围内。） */
static int32_t WAV_Convert32bitPCMto8bitPCM(int32_t in_32bitpcm)
{
  /* 添加 128 的偏移量 */
  return ((in_32bitpcm >> 24) + 128);
}

/* 将 32bit PCM 格式转换为 16bit 格式（注意：返回值是一个 32 位整数，但被裁剪在 16 位范围内） */
static int32_t WAV_Convert32bitPCMto16bitPCM(int32_t in_32bitpcm)
{
  return (in_32bitpcm >> 16);
}

/* 将 32bit PCM 格式转换为 24bit 格式（注意：返回值是一个 32 位整数，但被裁剪在 24 位范围内） */
static int32_t WAV_Convert32bitPCMto24bitPCM(int32_t in_32bitpcm)
{
  return (in_32bitpcm >> 8);
}

/* 解析器初始化 */
static void WAVParser_Initialize(struct WAVParser* parser, FILE* fp)
{
  parser->fp                = fp;
  memset(&parser->buffer, 0, sizeof(struct WAVBitBuffer));
  parser->buffer.byte_pos   = -1;
}

/* 解析器使用结束 */
static void WAVParser_Finalize(struct WAVParser* parser)
{
  parser->fp                = NULL;
  memset(&parser->buffer, 0, sizeof(struct WAVBitBuffer));
  parser->buffer.byte_pos   = -1;
}

/* 获取 n_bit 并右对齐结果 */
static WAVError WAVParser_GetBits(struct WAVParser* parser, uint32_t n_bits, uint64_t* bitsbuf)
{
  uint64_t tmp;
  struct WAVBitBuffer *buf = &(parser->buffer);

  /* 参数检查 */
  if (parser == NULL || bitsbuf == NULL || n_bits > 64) {
    return WAV_ERROR_INVALID_PARAMETER;
  }

  /* 第一次加载 */
  if (buf->byte_pos == -1) {
      if (fread(buf->bytes, sizeof(uint8_t), WAVBITBUFFER_BUFFER_SIZE, parser->fp) == 0) {
        return WAV_ERROR_IO;
      }
      buf->byte_pos   = 0;
      buf->bit_count  = 8;
  }

  /* 从最高位开始填充数据
   * 在第一个循环中设置为 tmp 的高位
   * 第2次以后以8 bit单位输入并设置为tmp */
  tmp = 0;
  while (n_bits > buf->bit_count) {
    /* 从高位填充 */
    n_bits  -= buf->bit_count;
    tmp     |= (uint64_t)WAV_GetLowerBits(buf->bit_count, buf->bytes[buf->byte_pos]) << n_bits;

    /* 读取 1 个字节 */
    buf->byte_pos++;
    buf->bit_count   = 8;

    /* 如果缓冲区已满，则再次读取 */
    if (buf->byte_pos == WAVBITBUFFER_BUFFER_SIZE) {
      if (fread(buf->bytes, sizeof(uint8_t), WAVBITBUFFER_BUFFER_SIZE, parser->fp) == 0) {
        return WAV_ERROR_IO;
      }
      buf->byte_pos = 0;
    }
  }

  /* 小数位处理 
   * 将剩余位设置为 tmp 的最高有效位 */
  buf->bit_count -= n_bits;
  tmp            |= (uint64_t)WAV_GetLowerBits(n_bits, (uint32_t)(buf->bytes[buf->byte_pos] >> buf->bit_count));

  *bitsbuf = tmp;
  return WAV_ERROR_OK;
}

/* 搜索（fseek） */
static WAVError WAVParser_Seek(struct WAVParser* parser, int32_t offset, int32_t wherefrom)
{
  if (parser->buffer.byte_pos != -1) {
    /* 因为预先读取了缓存的部分，所以返回 */
    offset -= (WAVBITBUFFER_BUFFER_SIZE - (parser->buffer.byte_pos + 1));
  }
  /* 偏移 */
  fseek(parser->fp, offset, wherefrom);
  /* 清除缓冲区 */
  parser->buffer.byte_pos = -1;

  return WAV_ERROR_OK;
}

/* 丢弃 WAV 文件句柄 */
void WAV_Destroy(struct WAVFile* wavfile)
{
  uint32_t ch;

  /* NULL检查和释放 */
#define NULLCHECK_AND_FREE(ptr) { \
  if ((ptr) != NULL) {            \
    free(ptr);                    \
    ptr = NULL;                   \
  }                               \
}

  if (wavfile != NULL) {
    for (ch = 0; ch < wavfile->format.num_channels; ch++) {
      NULLCHECK_AND_FREE(wavfile->data[ch]);
    }
    NULLCHECK_AND_FREE(wavfile->data);
    free(wavfile);
  }

#undef NULLCHECK_AND_FREE
}

/* 根据文件格式输出头部分导入 */
static WAVError WAVWriter_PutWAVHeader(
    struct WAVWriter* writer, const struct WAVFileFormat* format)
{
  uint32_t filesize, pcm_data_size;

  /* 参数检查 */
  if (writer == NULL || format == NULL) {
    return WAV_ERROR_INVALID_PARAMETER;
  }

  /* 格式检查 */
  /* 仅支持 PCM */
  if (format->data_format != WAV_DATA_FORMAT_PCM) {
    return WAV_ERROR_INVALID_FORMAT;
  }

  /* PCM 数据大小 */
  pcm_data_size 
    = format->num_samples * (format->bits_per_sample / 8) * format->num_channels;

  /* 文件大小 */
  filesize 
    = pcm_data_size
    + 44; /* 字段中从“RIFF”到（“data”大小）的字节数
             不包括任何扩展 */
  
  /* 输出标头'R'、'I'、'F'、'F' */
  if (WAVWriter_PutBits(writer, 'R', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };
  if (WAVWriter_PutBits(writer, 'I', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };
  if (WAVWriter_PutBits(writer, 'F', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };
  if (WAVWriter_PutBits(writer, 'F', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };

  /* 句子大小-8（此元素后的大小） */
  if (WAVWriter_PutLittleEndianBytes(writer, 4, filesize - 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; }

  /* 输出头’W’、‘A’、‘V’、‘E’ */
  if (WAVWriter_PutBits(writer, 'W', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };
  if (WAVWriter_PutBits(writer, 'A', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };
  if (WAVWriter_PutBits(writer, 'V', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };
  if (WAVWriter_PutBits(writer, 'E', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };

  /* 输出fmt信息块的头’f’、’m’、‘t’、’’ */
  if (WAVWriter_PutBits(writer, 'f', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };
  if (WAVWriter_PutBits(writer, 'm', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };
  if (WAVWriter_PutBits(writer, 't', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };
  if (WAVWriter_PutBits(writer, ' ', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };

  /* 输出 fmt chunk 的字节数（补充） 目前 16 字节固定 */
  if (WAVWriter_PutLittleEndianBytes(writer, 4, 16) != WAV_ERROR_OK) { return WAV_ERROR_IO; };

  /* 输出格式 ID（补充） 目前 是1（线性 PCM） */
  if (WAVWriter_PutLittleEndianBytes(writer, 2,  1) != WAV_ERROR_OK) { return WAV_ERROR_IO; };

  /* 通道数 */
  if (WAVWriter_PutLittleEndianBytes(writer, 2, format->num_channels) != WAV_ERROR_OK) { return WAV_ERROR_IO; };

  /* 采样率 */
  if (WAVWriter_PutLittleEndianBytes(writer, 4, format->sampling_rate) != WAV_ERROR_OK) { return WAV_ERROR_IO; };

  /* 数据速度（byte/sec） */
  if (WAVWriter_PutLittleEndianBytes(writer, 4,
        format->sampling_rate * (format->bits_per_sample / 8) * format->num_channels)
      != WAV_ERROR_OK) { return WAV_ERROR_IO; }

  /* 每块大小 */
  if (WAVWriter_PutLittleEndianBytes(writer, 2,
        (format->bits_per_sample / 8) * format->num_channels)
      != WAV_ERROR_OK) { return WAV_ERROR_IO; }

  /* 量化比特数（每样本的比特数） */
  if (WAVWriter_PutLittleEndianBytes(writer, 2, format->bits_per_sample) != WAV_ERROR_OK) { return WAV_ERROR_IO; };

  /* “data”信息块的头部输出 */
  if (WAVWriter_PutBits(writer, 'd', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };
  if (WAVWriter_PutBits(writer, 'a', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };
  if (WAVWriter_PutBits(writer, 't', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };
  if (WAVWriter_PutBits(writer, 'a', 8) != WAV_ERROR_OK) { return WAV_ERROR_IO; };

  /* 波形数据字节数 */
  if (WAVWriter_PutLittleEndianBytes(writer, 4, pcm_data_size) != WAV_ERROR_OK) { return WAV_ERROR_IO; }

  return WAV_ERROR_OK;
}

/* 使用捕获的 PCM 数据输出 */
static WAVError WAVWriter_PutWAVPcmData(
    struct WAVWriter* writer, const struct WAVFile* wavfile)
{
  uint32_t  ch, sample, bytes_per_sample;
  int32_t   (*convert_sint32_to_pcmdata_func)(int32_t);

  /* 根据位深确定PCM数据转换函数 */
  switch (wavfile->format.bits_per_sample) {
    case 8:
      convert_sint32_to_pcmdata_func = WAV_Convert32bitPCMto8bitPCM;
      break;
    case 16:
      convert_sint32_to_pcmdata_func = WAV_Convert32bitPCMto16bitPCM;
      break;
    case 24:
      convert_sint32_to_pcmdata_func = WAV_Convert32bitPCMto24bitPCM;
      break;
    case 32:
      convert_sint32_to_pcmdata_func = WAV_Convert32bitPCMto32bitPCM;
      break;
    default:
      /* fprintf(stderr, "Unsupported bits per sample format(=%d). \n", wavfile->format.bits_per_sample); */
      return WAV_ERROR_INVALID_FORMAT;
  }

  /* 通道交错输出 */
  bytes_per_sample = wavfile->format.bits_per_sample / 8;
  for (sample = 0; sample < wavfile->format.num_samples; sample++) {
    for (ch = 0; ch < wavfile->format.num_channels; ch++) {
      if (WAVWriter_PutLittleEndianBytes(writer,
            bytes_per_sample,
            (uint64_t)convert_sint32_to_pcmdata_func(WAVFile_PCM(wavfile, sample, ch))) != WAV_ERROR_OK) {
        return WAV_ERROR_IO;
      }
    }
  }

  return WAV_ERROR_OK;
}

/* 文件写入 */
WAVApiResult WAV_WriteToFile(
    const char* filename, const struct WAVFile* wavfile)
{
  struct WAVWriter  writer;
  FILE*             fp;
  
  /* 参数检查 */
  if (filename == NULL || wavfile == NULL) {
    return WAV_APIRESULT_INVALID_PARAMETER;
  }
  
  /* 打开.wav文件 */
  fp = fopen(filename, "wb");
  if (fp == NULL) {
    /* fprintf(stderr, "Failed to open %s. \n", filename); */
    return WAV_APIRESULT_NG;
  }

  /* 写入初始化 */
  WAVWriter_Initialize(&writer, fp);

  /* 信头导出 */
  if (WAVWriter_PutWAVHeader(&writer, &wavfile->format) != WAV_ERROR_OK) {
    return WAV_APIRESULT_NG;
  }

  /* 数据导出 */
  if (WAVWriter_PutWAVPcmData(&writer, wavfile) != WAV_ERROR_OK) {
    return WAV_APIRESULT_NG;
  }

  /* 写入结束 */
  WAVWriter_Finalize(&writer);

  /* 关闭文件 */
  fclose(fp);

  /* 正常结束 */
  return WAV_APIRESULT_OK;
}

/* 写入初始化 */
static void WAVWriter_Initialize(struct WAVWriter* writer, FILE* fp)
{
  writer->fp                = fp;
  writer->bit_count         = 8;
  writer->bit_buffer        = 0;
  memset(&writer->buffer, 0, sizeof(struct WAVBitBuffer));
  writer->buffer.byte_pos   = 0;
}

/* 写入结束 */
static void WAVWriter_Finalize(struct WAVWriter* writer)
{
  /* 写出缓冲区中多余的数据 */
  WAVWriter_Flush(writer);

  /* 清除成员 */
  writer->fp              = NULL;
  writer->bit_count       = 8;
  writer->bit_buffer      = 0;
  memset(&writer->buffer, 0, sizeof(struct WAVBitBuffer));
  writer->buffer.byte_pos = 0;
}

/* 写入 val 的低 n_bit（大端） */
static WAVError WAVWriter_PutBits(struct WAVWriter* writer, uint64_t val, uint32_t n_bits)
{
  /* 无效参数 */
  if (writer == NULL) {
    return WAV_ERROR_INVALID_PARAMETER;
  }

  /* 从val的高位开始依次输出
   * 在初次循环中填入尾数（输出所需的位数）输出
   * 第2次以后以8 bit单位输出 */
  while (n_bits >= writer->bit_count) {
    n_bits -= writer->bit_count;
    writer->bit_buffer |= (uint8_t)WAV_GetLowerBits(writer->bit_count, val >> n_bits);

    /* 添加到缓存 */
    writer->buffer.bytes[writer->buffer.byte_pos++] = (uint8_t)(writer->bit_buffer & 0xFF);

    /* 缓冲区满时导出 */
    if (writer->buffer.byte_pos == WAVBITBUFFER_BUFFER_SIZE) {
      if (fwrite(writer->buffer.bytes, 
            sizeof(uint8_t), WAVBITBUFFER_BUFFER_SIZE,
            writer->fp) < WAVBITBUFFER_BUFFER_SIZE) {
        return WAV_ERROR_IO;
      }
      /* 重置写入位置 */
      writer->buffer.byte_pos = 0;
    }

    writer->bit_buffer  = 0;
    writer->bit_count   = 8;
  }

  /* 小数位处理:
   * 将剩余量设置为缓冲区的高位 */
  writer->bit_count -= n_bits;
  writer->bit_buffer |= (uint8_t)(WAV_GetLowerBits(n_bits, (uint32_t)val) << writer->bit_count);

  return WAV_ERROR_OK;
}

/* 小端输出位模式 */
static WAVError WAVWriter_PutLittleEndianBytes(
    struct WAVWriter* writer, uint32_t nbytes, uint64_t data)
{
  uint64_t out;
  uint32_t i_byte;

  /* 按小端排序 */
  out = 0;
  for (i_byte = 0; i_byte < nbytes; i_byte++) {
    out |= ((data >> (8 * (nbytes - i_byte - 1))) & 0xFFUL) << (8 * i_byte);
  }

  /* 输出 */
  if (WAVWriter_PutBits(writer, out, (uint8_t)(nbytes * 8)) != WAV_ERROR_OK) {
    return WAV_ERROR_IO;
  }

  return WAV_ERROR_OK;
}

/* 清除缓冲区中累积的位 */
static WAVError WAVWriter_Flush(struct WAVWriter* writer)
{
  /* 参数检查 */
  if (writer == NULL) {
    return WAV_ERROR_INVALID_PARAMETER;
  }

  /* 强制输出剩余位 */
  if (writer->bit_count != 8) {
    if (WAVWriter_PutBits(writer, 0, (uint8_t)writer->bit_count) != WAV_ERROR_OK) {
      return WAV_ERROR_IO;
    }
    writer->bit_buffer = 0;
    writer->bit_count  = 8;
  }

  /* 刷新缓冲区中剩余的数据 */
  if (fwrite(writer->buffer.bytes, 
        sizeof(uint8_t), (uint32_t)writer->buffer.byte_pos,
        writer->fp) < (size_t)writer->buffer.byte_pos) {
    return WAV_ERROR_IO;
  }
  /* 缓冲区余量为0 */
  writer->buffer.byte_pos = 0;

  return WAV_ERROR_OK;
}

/* 使用小端获取位模式 */
static WAVError WAVParser_GetLittleEndianBytes(
    struct WAVParser* parser, uint32_t nbytes, uint64_t* bitsbuf)
{
  uint64_t tmp, ret;
  uint32_t i_byte;

  /* 取得大端 */
  if (WAVParser_GetBits(parser, nbytes * 8, &tmp) != WAV_ERROR_OK) {
    return WAV_ERROR_IO; 
  }

  /* 按小端排序 */
  ret = 0;
  for (i_byte = 0; i_byte < nbytes; i_byte++) {
    ret |= ((tmp >> (8 * (nbytes - i_byte - 1))) & 0xFFUL) << (8 * i_byte);
  }
  *bitsbuf = ret;

  return WAV_ERROR_OK;
}

/* 使用解析器获取字符串 */
static WAVError WAVParser_GetString(
  struct WAVParser* parser, char* string_buffer, uint32_t string_length) 
{
  uint32_t i_byte;
  uint64_t bitsbuf;

  assert(parser != NULL && string_buffer != NULL);

  /* 获取字符串 */
  for (i_byte = 0; i_byte < string_length; i_byte++) { 
    /* 获得 1 个字符 */
    if (WAVParser_GetBits(parser, 8, &bitsbuf) != WAV_ERROR_OK) {
      return WAV_ERROR_IO;                     
    }
    string_buffer[i_byte] = (char)bitsbuf;
  }

  return WAV_ERROR_OK;
}

/* 使用解析器获取字符串/匹配检查 */
static WAVError WAVParser_CheckSignatureString(
  struct WAVParser* parser, const char* signature, uint32_t signature_length) 
{
  uint32_t i_byte;
  uint64_t bitsbuf;

  assert(parser != NULL && signature != NULL);

  /* 获取字符串/检查 */
  for (i_byte = 0; i_byte < signature_length; i_byte++) { 
    /* 获得 1 个字符 */
    if (WAVParser_GetBits(parser, 8, &bitsbuf) != WAV_ERROR_OK) {
      return WAV_ERROR_IO;                     
    }
    /* 签名检查 */
    if (signature[i_byte] != (char)bitsbuf) {
      /* fprintf(stderr, "Failed to check %s header signature. \n", signature); */
      return WAV_ERROR_INVALID_FORMAT;
    }
  }

  return WAV_ERROR_OK;
}
