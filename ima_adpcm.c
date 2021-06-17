#include "ima_adpcm.h"
#include "byte_array.h"

#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#define IMAADPCM_ALIGNMENT              16

/* 编码时要导出的头大小（数据块之前的文件大小） */
#define IMAADPCMWAVENCODER_HEADER_SIZE  60

/* 向上舍入n的倍数 */
#define IMAADPCM_ROUND_UP(val, n) ((((val) + ((n) - 1)) / (n)) * (n))

/* 选择最大值 */
#define IMAADPCM_MAX_VAL(a, b) (((a) > (b)) ? (a) : (b))

/* 选择最小值 */
#define IMAADPCM_MIN_VAL(a, b) (((a) < (b)) ? (a) : (b))

/* 限制在min以上max以下 */
#define IMAADPCM_INNER_VAL(val, min, max) \
  IMAADPCM_MAX_VAL(min, IMAADPCM_MIN_VAL(max, val))

/* 计算指定样本数量所占的数据大小[byte] */
#define IMAADPCM_CALCULATE_DATASIZE_BYTE(num_samples, bits_per_sample) \
  (IMAADPCM_ROUND_UP((num_samples) * (bits_per_sample), 8) / 8)

/* 确认FourCC的一致性 */
#define IMAADPCM_CHECK_FOURCC(u32lebuf, c1, c2, c3, c4) \
  ((u32lebuf) == ((c1 << 0) | (c2 << 8) | (c3 << 16) | (c4 << 24)))

/* 内部错误类型 */
typedef enum IMAADPCMErrorTag {
  IMAADPCM_ERROR_OK = 0,              /* OK */
  IMAADPCM_ERROR_NG,                  /* 无法分类错误 */
  IMAADPCM_ERROR_INVALID_ARGUMENT,    /* 参数错误 */
  IMAADPCM_ERROR_INVALID_FORMAT,      /* 格式错误 */
  IMAADPCM_ERROR_INSUFFICIENT_BUFFER, /* 缓冲区大小不足 */
  IMAADPCM_ERROR_INSUFFICIENT_DATA    /* 数据大小不足   */
} IMAADPCMError;

/* 核心处理解码器 */
struct IMAADPCMCoreDecoder {
  int16_t sample_val;             /* 样本值 */
  int8_t  stepsize_index;         /* 步长表参考索引 */
};

/* 解码器 */
struct IMAADPCMWAVDecoder {
  struct IMAADPCMWAVHeaderInfo  header;
  struct IMAADPCMCoreDecoder    core_decoder[IMAADPCM_MAX_NUM_CHANNELS];
  void                          *work;
};

/* 核心处理编码器 */
struct IMAADPCMCoreEncoder {
  int16_t prev_sample;            /* 样本值 */
  int8_t  stepsize_index;         /* 步长表参考索引 */
};

/* 编码器 */
struct IMAADPCMWAVEncoder {
  struct IMAADPCMWAVEncodeParameter encode_paramemter;
  uint8_t                           set_parameter;
  struct IMAADPCMCoreEncoder        core_encoder[IMAADPCM_MAX_NUM_CHANNELS];
  void                              *work;
};

/* 单样本解码 */
static int16_t IMAADPCMCoreDecoder_DecodeSample(
    struct IMAADPCMCoreDecoder *decoder, uint8_t nibble);

/* 单声道块的解码 */
static IMAADPCMError IMAADPCMWAVDecoder_DecodeBlockMono(
    struct IMAADPCMCoreDecoder *core_decoder,
    const uint8_t *read_pos, uint32_t data_size, 
    int16_t **buffer, uint32_t buffer_num_samples,
    uint32_t *num_decode_samples);

/* 立体声块的解码 */
static IMAADPCMError IMAADPCMWAVDecoder_DecodeBlockStereo(
    struct IMAADPCMCoreDecoder *core_decoder,
    const uint8_t *read_pos, uint32_t data_size, 
    int16_t **buffer, uint32_t buffer_num_samples, 
    uint32_t *num_decode_samples);

/* 单个数据块编码 */
/* 与解码不同，束缚在static上: 因为编码器有内部状态，需要不断调用EncodeBlock */
static IMAADPCMApiResult IMAADPCMWAVEncoder_EncodeBlock(
    struct IMAADPCMWAVEncoder *encoder,
    const int16_t *const *input, uint32_t num_samples, 
    uint8_t *data, uint32_t data_size, uint32_t *output_size);

/* 单声道块编码 */
static IMAADPCMError IMAADPCMWAVEncoder_EncodeBlockMono(
    struct IMAADPCMCoreEncoder *core_encoder,
    const int16_t *const *input, uint32_t num_samples,
    uint8_t *data, uint32_t data_size, uint32_t *output_size);

/* 立体声块编码 */
static IMAADPCMError IMAADPCMWAVEncoder_EncodeBlockStereo(
    struct IMAADPCMCoreEncoder *core_encoder,
    const int16_t *const *input, uint32_t num_samples,
    uint8_t *data, uint32_t data_size, uint32_t *output_size);

/* 索引变动表 */
static const int8_t IMAADPCM_index_table[16] = {
  -1, -1, -1, -1, 2, 4, 6, 8, 
  -1, -1, -1, -1, 2, 4, 6, 8 
};

/* 步长量化表 */
static const uint16_t IMAADPCM_stepsize_table[89] = {
      7,     8,     9,    10,    11,    12,    13,    14, 
     16,    17,    19,    21,    23,    25,    28,    31, 
     34,    37,    41,    45,    50,    55,    60,    66,
     73,    80,    88,    97,   107,   118,   130,   143, 
    157,   173,   190,   209,   230,   253,   279,   307,
    337,   371,   408,   449,   494,   544,   598,   658,
    724,   796,   876,   963,  1060,  1166,  1282,  1411, 
   1552,  1707,  1878,  2066,  2272,  2499,  2749,  3024,
   3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,
   7132,  7845,  8630,  9493, 10442, 11487, 12635, 13899,
  15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
  32767
};

/* 工作量计算 */
int32_t IMAADPCMWAVDecoder_CalculateWorkSize(void)
{
  return IMAADPCM_ALIGNMENT + sizeof(struct IMAADPCMWAVDecoder);
}

/* 创建解码句柄 */
struct IMAADPCMWAVDecoder *IMAADPCMWAVDecoder_Create(void *work, int32_t work_size)
{
  struct IMAADPCMWAVDecoder *decoder;
  uint8_t *work_ptr;
  uint32_t alloced_by_malloc = 0;

  if ((work == NULL) && (work_size == 0)) {
    work_size = IMAADPCMWAVDecoder_CalculateWorkSize();
    work = malloc((uint32_t)work_size);
    alloced_by_malloc = 1;
  }

  /* 参数检查 */
  if ((work == NULL) || (work_size < IMAADPCMWAVDecoder_CalculateWorkSize())) {
    return NULL;
  }

  work_ptr = (uint8_t *)work;

  /* 对齐并配置 */
  work_ptr = (uint8_t *)IMAADPCM_ROUND_UP((uintptr_t)work_ptr, IMAADPCM_ALIGNMENT);
  decoder = (struct IMAADPCMWAVDecoder *)work_ptr;

  /* 将句柄的内容初始化为 0 */
  memset(decoder, 0, sizeof(struct IMAADPCMWAVDecoder));

  /* 在确保自身安全的情况下预先存储内存 */
  decoder->work = alloced_by_malloc ? work : NULL;

  return decoder;
}

/* 丢弃解码句柄 */
void IMAADPCMWAVDecoder_Destroy(struct IMAADPCMWAVDecoder *decoder)
{
  if (decoder != NULL) {
    if (decoder->work != NULL) {
      free(decoder->work);
    }
  }
}

/* 标头解码 */
IMAADPCMApiResult IMAADPCMWAVDecoder_DecodeHeader(
    const uint8_t *data, uint32_t data_size, struct IMAADPCMWAVHeaderInfo *header_info)
{
  const uint8_t *data_pos;
  uint32_t u32buf;
  uint16_t u16buf;
  uint32_t find_fact_chunk;
  struct IMAADPCMWAVHeaderInfo tmp_header_info;

  /* 参数检查 */
  if ((data == NULL) || (header_info == NULL)) {
    return IMAADPCM_APIRESULT_INVALID_ARGUMENT;
  }

  /* 读取指针设置 */
  data_pos = data;

  /* RIFF块ID */
  ByteArray_GetUint32LE(data_pos, &u32buf);
  if (!IMAADPCM_CHECK_FOURCC(u32buf, 'R', 'I', 'F', 'F')) {
    fprintf(stderr, "Invalid RIFF chunk id. \n");
    return IMAADPCM_APIRESULT_INVALID_FORMAT;
  }
  /* RIFF块大小（跳过） */
  ByteArray_GetUint32LE(data_pos, &u32buf);

  /* WAVE块ID */
  ByteArray_GetUint32LE(data_pos, &u32buf);
  if (!IMAADPCM_CHECK_FOURCC(u32buf, 'W', 'A', 'V', 'E')) {
    fprintf(stderr, "Invalid WAVE chunk id. \n");
    return IMAADPCM_APIRESULT_INVALID_FORMAT;
  }

  /* FMT块ID */
  ByteArray_GetUint32LE(data_pos, &u32buf);
  if (!IMAADPCM_CHECK_FOURCC(u32buf, 'f', 'm', 't', ' ')) {
    fprintf(stderr, "Invalid fmt  chunk id. \n");
    return IMAADPCM_APIRESULT_INVALID_FORMAT;
  }
  /* fmt块大小 */
  ByteArray_GetUint32LE(data_pos, &u32buf);
  if (data_size <= u32buf) {
    fprintf(stderr, "Data size too small. fmt chunk size:%d data size:%d \n", u32buf, data_size);
    return IMAADPCM_APIRESULT_INSUFFICIENT_DATA;
  }
  /* WAVE格式类型：仅接受IMA-ADPCM */
  ByteArray_GetUint16LE(data_pos, &u16buf);
  if (u16buf != 17) {
    fprintf(stderr, "Unsupported format: %d \n", u16buf);
    return IMAADPCM_APIRESULT_INVALID_FORMAT;
  }
  /* 通道数 */
  ByteArray_GetUint16LE(data_pos, &u16buf);
  if (u16buf > IMAADPCM_MAX_NUM_CHANNELS) {
    fprintf(stderr, "Unsupported channels: %d \n", u16buf);
    return IMAADPCM_APIRESULT_INVALID_FORMAT;
  }
  tmp_header_info.num_channels = u16buf;
  /* 采样率 */
  ByteArray_GetUint32LE(data_pos, &u32buf);
  tmp_header_info.sampling_rate = u32buf;
  /* 数据速度[byte/sec] */
  ByteArray_GetUint32LE(data_pos, &u32buf);
  tmp_header_info.bytes_per_sec = u32buf;
  /* 块大小 */
  ByteArray_GetUint16LE(data_pos, &u16buf);
  tmp_header_info.block_size = u16buf;
  /* 每个样本的位数 */
  ByteArray_GetUint16LE(data_pos, &u16buf);
  tmp_header_info.bits_per_sample = u16buf;
  /* fmt chunk额外大小 */
  ByteArray_GetUint16LE(data_pos, &u16buf);
  if (u16buf != 2) {
    fprintf(stderr, "Unsupported fmt chunk extra size: %d \n", u16buf);
    return IMAADPCM_APIRESULT_INVALID_FORMAT;
  }
  /* 每块样本数 */
  ByteArray_GetUint16LE(data_pos, &u16buf);
  tmp_header_info.num_samples_per_block = u16buf;

  /* 跳到数据块 */
  find_fact_chunk = 0;
  while (1) {
    uint32_t chunkid;
	
    if (data_size < (uint32_t)(data_pos - data)) {
      return IMAADPCM_APIRESULT_INSUFFICIENT_DATA;
    }
    /* 块ID获取 */
    ByteArray_GetUint32LE(data_pos, &chunkid);
    if (IMAADPCM_CHECK_FOURCC(chunkid, 'd', 'a', 't', 'a')) {
      /* 找到数据块后，结束 */
      break;
    } else if (IMAADPCM_CHECK_FOURCC(chunkid, 'f', 'a', 'c', 't')) {
      /* FACT块（可选） */
      ByteArray_GetUint32LE(data_pos, &u32buf);
      /* FACT块大小 */
      if (u32buf != 4) {
        fprintf(stderr, "Unsupported fact chunk size: %d \n", u16buf);
        return IMAADPCM_APIRESULT_INVALID_FORMAT;
      }
	  
      ByteArray_GetUint32LE(data_pos, &u32buf);
      tmp_header_info.num_samples = u32buf;
      /* 标记发现了一个fact块 */
      assert(find_fact_chunk == 0);
      find_fact_chunk = 1;
    } else {
      uint32_t size;
      /* 其他块仅获取大小并跳过查找 */
      ByteArray_GetUint32LE(data_pos, &size);
      /* printf("chunk:%8X size:%d \n", chunkid, (int32_t)size); */
      data_pos += size;
    }
  }

  /* 数据块大小（跳过） */
  ByteArray_GetUint32LE(data_pos, &u32buf);

  /* 如果没有fact块，则根据块大小计算样本数 */
  if (find_fact_chunk == 0) {
    uint32_t data_chunk_size = u32buf;
    /* +1以包含最后一个块 */
    uint32_t num_blocks = data_chunk_size / tmp_header_info.block_size + 1;
    tmp_header_info.num_samples = tmp_header_info.num_samples_per_block * num_blocks;
  }

  /* 偏移到数据区的开头 */
  tmp_header_info.header_size = (uint32_t)(data_pos - data);

  /* 成功 */
  (*header_info) = tmp_header_info;
  return IMAADPCM_APIRESULT_OK;
}

/* 单样本解码 */
static int16_t IMAADPCMCoreDecoder_DecodeSample(
    struct IMAADPCMCoreDecoder *decoder, uint8_t nibble)
{
  int8_t  idx;
  int32_t predict, qdiff, delta, stepsize;

  assert(decoder != NULL);

  /* 接收自动变量中经常引用的变量 */
  predict = decoder->sample_val;
  idx = decoder->stepsize_index;

  /* 获取步长 */
  stepsize = IMAADPCM_stepsize_table[idx];

  /* 索引更新 */
  idx = (int8_t)(idx + IMAADPCM_index_table[nibble]);
  idx = IMAADPCM_INNER_VAL(idx, 0, 88);

  /* 差分计算 */
  /* diff = stepsize * (delta * 2 + 1) / 8 */
  delta = nibble & 7;
  qdiff = (stepsize * ((delta << 1) + 1)) >> 3;

  /* 用符号位加差开关加/减 */
  if (nibble & 8) {
    predict -= qdiff;
  } else {
    predict += qdiff;
  }

  /* 裁剪到16bit宽度 */
  predict = IMAADPCM_INNER_VAL(predict, -32768, 32767);

  /* 反映计算结果 */
  decoder->sample_val = (int16_t)predict;
  decoder->stepsize_index = idx;

  return decoder->sample_val;
}

/* 单声道块的解码 */
static IMAADPCMError IMAADPCMWAVDecoder_DecodeBlockMono(
    struct IMAADPCMCoreDecoder *core_decoder,
    const uint8_t *read_pos, uint32_t data_size, 
    int16_t **buffer, uint32_t buffer_num_samples,
    uint32_t *num_decode_samples)
{
  uint8_t u8buf;
  uint8_t nibble[2];
  uint32_t smpl, tmp_num_decode_samples;
  const uint8_t *read_head = read_pos;

  /* 参数检查 */
  if ((core_decoder == NULL) || (read_pos == NULL)
      || (buffer == NULL) || (buffer[0] == NULL)) {
    return IMAADPCM_ERROR_INVALID_ARGUMENT;
  }

  /* 计算可解码的样本数* 2是每个字节2个样本，+ 1表示标题 */
  tmp_num_decode_samples = (data_size - 4) * 2;
  tmp_num_decode_samples += 1;
  /* 按缓冲区大小截断 */
  tmp_num_decode_samples = IMAADPCM_MIN_VAL(tmp_num_decode_samples, buffer_num_samples);

  /* 块头解码 */
  ByteArray_GetUint16LE(read_pos, (uint16_t *)&(core_decoder->sample_val));
  ByteArray_GetUint8(read_pos, (uint8_t *)&(core_decoder->stepsize_index));
  ByteArray_GetUint8(read_pos, &u8buf); /* reserved */
  if (u8buf != 0) {
    return IMAADPCM_ERROR_INVALID_FORMAT;
  }

  /* 第一个样本在信头中 */
  buffer[0][0] = core_decoder->sample_val;

  /* 块数据解码 */
  for (smpl = 1; smpl < tmp_num_decode_samples; smpl += 2) {
    assert((uint32_t)(read_pos - read_head) < data_size);
    ByteArray_GetUint8(read_pos, &u8buf);
    nibble[0] = (u8buf >> 0) & 0xF;
    nibble[1] = (u8buf >> 4) & 0xF;
    buffer[0][smpl + 0] = IMAADPCMCoreDecoder_DecodeSample(core_decoder, nibble[0]);
    buffer[0][smpl + 1] = IMAADPCMCoreDecoder_DecodeSample(core_decoder, nibble[1]);
  }

  /* 设置解码样本数 */
  (*num_decode_samples) = tmp_num_decode_samples;
  return IMAADPCM_ERROR_OK;
}

/* 立体声块解码 */
static IMAADPCMError IMAADPCMWAVDecoder_DecodeBlockStereo(
    struct IMAADPCMCoreDecoder *core_decoder,
    const uint8_t *read_pos, uint32_t data_size, 
    int16_t **buffer, uint32_t buffer_num_samples, 
    uint32_t *num_decode_samples)
{
  uint32_t u32buf;
  uint8_t nibble[8];
  uint32_t ch, smpl, tmp_num_decode_samples;
  const uint8_t *read_head = read_pos;

  /* 参数检查 */
  if ((core_decoder == NULL) || (read_pos == NULL)
      || (buffer == NULL) || (buffer[0] == NULL) || (buffer[1] == NULL)) {
    return IMAADPCM_ERROR_INVALID_ARGUMENT;
  }

  /* 计算可以解码的样本数量+1是针对标头的 */
  tmp_num_decode_samples = data_size - 8;
  tmp_num_decode_samples += 1;
  /* 按缓冲区大小截断 */
  tmp_num_decode_samples = IMAADPCM_MIN_VAL(tmp_num_decode_samples, buffer_num_samples);

  /* 块头解码 */
  for (ch = 0; ch < 2; ch++) {
    uint8_t reserved;
    ByteArray_GetUint16LE(read_pos, (uint16_t *)&(core_decoder[ch].sample_val));
    ByteArray_GetUint8(read_pos, (uint8_t *)&(core_decoder[ch].stepsize_index));
    ByteArray_GetUint8(read_pos, &reserved);
    if (reserved != 0) {
      return IMAADPCM_ERROR_INVALID_FORMAT;
    }
  }

  /* 获取第一个样品 */
  for (ch = 0; ch < 2; ch++) {
    buffer[ch][0] = core_decoder[ch].sample_val;
  }

  /* 块数据解码 */
  for (smpl = 1; smpl < tmp_num_decode_samples; smpl += 8) {
    uint32_t smp;
    int16_t  buf[8];
    for (ch = 0; ch < 2; ch++) {
      assert((uint32_t)(read_pos - read_head) < data_size);
      ByteArray_GetUint32LE(read_pos, &u32buf);
      nibble[0] = (uint8_t)((u32buf >>  0) & 0xF);
      nibble[1] = (uint8_t)((u32buf >>  4) & 0xF);
      nibble[2] = (uint8_t)((u32buf >>  8) & 0xF);
      nibble[3] = (uint8_t)((u32buf >> 12) & 0xF);
      nibble[4] = (uint8_t)((u32buf >> 16) & 0xF);
      nibble[5] = (uint8_t)((u32buf >> 20) & 0xF);
      nibble[6] = (uint8_t)((u32buf >> 24) & 0xF);
      nibble[7] = (uint8_t)((u32buf >> 28) & 0xF);

      /* 由于采样数可能不是1 +（8的倍数），因此会暂时将其接收到缓冲区中。 */
      buf[0] = IMAADPCMCoreDecoder_DecodeSample(&(core_decoder[ch]), nibble[0]);
      buf[1] = IMAADPCMCoreDecoder_DecodeSample(&(core_decoder[ch]), nibble[1]);
      buf[2] = IMAADPCMCoreDecoder_DecodeSample(&(core_decoder[ch]), nibble[2]);
      buf[3] = IMAADPCMCoreDecoder_DecodeSample(&(core_decoder[ch]), nibble[3]);
      buf[4] = IMAADPCMCoreDecoder_DecodeSample(&(core_decoder[ch]), nibble[4]);
      buf[5] = IMAADPCMCoreDecoder_DecodeSample(&(core_decoder[ch]), nibble[5]);
      buf[6] = IMAADPCMCoreDecoder_DecodeSample(&(core_decoder[ch]), nibble[6]);
      buf[7] = IMAADPCMCoreDecoder_DecodeSample(&(core_decoder[ch]), nibble[7]);
      for (smp = 0; (smp < 8) && ((smpl + smp) < tmp_num_decode_samples); smp++) {
        buffer[ch][smpl + smp] = buf[smp];
      }
    }
  }

  /* 设置解码样本数 */
  (*num_decode_samples) = tmp_num_decode_samples;
  return IMAADPCM_ERROR_OK;
}

/* 单数据块解码 */
static IMAADPCMApiResult IMAADPCMWAVDecoder_DecodeBlock(
    struct IMAADPCMWAVDecoder *decoder,
    const uint8_t *data, uint32_t data_size, 
    int16_t **buffer, uint32_t buffer_num_channels, uint32_t buffer_num_samples, 
    uint32_t *num_decode_samples)
{
  IMAADPCMError err;
  const struct IMAADPCMWAVHeaderInfo *header;

  /* 参数检查 */
  if ((decoder == NULL) || (data == NULL)
      || (buffer == NULL) || (num_decode_samples == NULL)) {
    return IMAADPCM_APIRESULT_INVALID_ARGUMENT;
  }

  header = &(decoder->header);

  /* 缓冲区大小检查 */
  if (buffer_num_channels < header->num_channels) {
    return IMAADPCM_APIRESULT_INSUFFICIENT_BUFFER;
  }

  /* 块解码 */
  switch (header->num_channels) {
    case 1:
      err = IMAADPCMWAVDecoder_DecodeBlockMono(decoder->core_decoder, 
          data, data_size, buffer, buffer_num_samples, num_decode_samples);
      break;
    case 2:
      err = IMAADPCMWAVDecoder_DecodeBlockStereo(decoder->core_decoder, 
          data, data_size, buffer, buffer_num_samples, num_decode_samples);
      break;
    default:
      return IMAADPCM_APIRESULT_INVALID_FORMAT;
  }

  /* 解码时的错误句柄 */
  if (err != IMAADPCM_ERROR_OK) {
    switch (err) {
      case IMAADPCM_ERROR_INVALID_ARGUMENT:
        return IMAADPCM_APIRESULT_INVALID_ARGUMENT;
      case IMAADPCM_ERROR_INVALID_FORMAT:
        return IMAADPCM_APIRESULT_INVALID_FORMAT;
      case IMAADPCM_ERROR_INSUFFICIENT_BUFFER:
        return IMAADPCM_APIRESULT_INSUFFICIENT_BUFFER;
      default:
        return IMAADPCM_APIRESULT_NG;
    }
  }

  return IMAADPCM_APIRESULT_OK;
}

/* 解码整个文件，包括标题 */
IMAADPCMApiResult IMAADPCMWAVDecoder_DecodeWhole(
    struct IMAADPCMWAVDecoder *decoder, const uint8_t *data, uint32_t data_size,
    int16_t **buffer, uint32_t buffer_num_channels, uint32_t buffer_num_samples)
{
  IMAADPCMApiResult ret;
  uint32_t progress, ch, read_offset, read_block_size, num_decode_samples;
  const uint8_t *read_pos;
  int16_t *buffer_ptr[IMAADPCM_MAX_NUM_CHANNELS];
  const struct IMAADPCMWAVHeaderInfo *header;

  /* 参数检查 */
  if ((decoder == NULL) || (data == NULL) || (buffer == NULL)) {
    return IMAADPCM_APIRESULT_INVALID_ARGUMENT;
  }

  /* 标头解码 */
  if ((ret = IMAADPCMWAVDecoder_DecodeHeader(data, data_size, &(decoder->header)))
      != IMAADPCM_APIRESULT_OK) {
    return ret;
  }
  header = &(decoder->header);

  /* 缓冲区大小检查 */
  if ((buffer_num_channels < header->num_channels)
      || (buffer_num_samples < header->num_samples)) {
    return IMAADPCM_APIRESULT_INSUFFICIENT_BUFFER;
  }

  progress = 0;
  read_offset = header->header_size;
  read_pos = data + header->header_size;
  while ((progress < header->num_samples) && (read_offset < data_size)) {
    /* 确认读取大小 */
    read_block_size = IMAADPCM_MIN_VAL(data_size - read_offset, header->block_size);
    /* 样本出口位置集 */
    for (ch = 0; ch < header->num_channels; ch++) {
      buffer_ptr[ch] = &buffer[ch][progress];
    }

    /* 块解码 */
    if ((ret = IMAADPCMWAVDecoder_DecodeBlock(decoder,
          read_pos, read_block_size,
          buffer_ptr, buffer_num_channels, buffer_num_samples - progress, 
          &num_decode_samples)) != IMAADPCM_APIRESULT_OK) {
      return ret;
    }

    /* 进度更新 */
    read_pos    += read_block_size;
    read_offset += read_block_size;
    progress    += num_decode_samples;
  }

  /* 成功 */
  return IMAADPCM_APIRESULT_OK;
}

/* 标头编码 */
IMAADPCMApiResult IMAADPCMWAVEncoder_EncodeHeader(
    const struct IMAADPCMWAVHeaderInfo *header_info, uint8_t *data, uint32_t data_size)
{
  uint8_t *data_pos;
  uint32_t num_blocks, data_chunk_size;
  uint32_t tail_block_num_samples, tail_block_size;

  /* 参数检查 */
  if ((header_info == NULL) || (data == NULL)) {
    return IMAADPCM_APIRESULT_INVALID_ARGUMENT;
  }

  /* 标头大小和输入数据大小的比较 */
  if (data_size < IMAADPCMWAVENCODER_HEADER_SIZE) {
    return IMAADPCM_APIRESULT_INSUFFICIENT_DATA;
  }

  /* 简单的标头检查：块大小应能够包含所有样本数据 */
  if (IMAADPCM_CALCULATE_DATASIZE_BYTE(header_info->num_samples_per_block, header_info->bits_per_sample) > header_info->block_size) {
    return IMAADPCM_APIRESULT_INVALID_FORMAT;
  }
  
  /* 数据大小计算 */
  assert(header_info->num_samples_per_block != 0);
  num_blocks = (header_info->num_samples / header_info->num_samples_per_block) + 1;
  data_chunk_size = header_info->block_size * num_blocks;
  /* 减少最后一块的剩余样本量 */
  tail_block_num_samples = header_info->num_samples % header_info->num_samples_per_block;
  tail_block_size = IMAADPCM_CALCULATE_DATASIZE_BYTE(header_info->num_samples_per_block - tail_block_num_samples, header_info->bits_per_sample);
  data_chunk_size -= tail_block_size;

  /* 导出指针设置 */
  data_pos = data;

  /* RIFF块ID */
  ByteArray_PutUint8(data_pos, 'R');
  ByteArray_PutUint8(data_pos, 'I');
  ByteArray_PutUint8(data_pos, 'F');
  ByteArray_PutUint8(data_pos, 'F');
  /* RIFF块大小 */
  ByteArray_PutUint32LE(data_pos, IMAADPCMWAVENCODER_HEADER_SIZE + data_chunk_size - 8);
  /* WAVE块ID */
  ByteArray_PutUint8(data_pos, 'W');
  ByteArray_PutUint8(data_pos, 'A');
  ByteArray_PutUint8(data_pos, 'V');
  ByteArray_PutUint8(data_pos, 'E');
  /* FMT块ID */
  ByteArray_PutUint8(data_pos, 'f');
  ByteArray_PutUint8(data_pos, 'm');
  ByteArray_PutUint8(data_pos, 't');
  ByteArray_PutUint8(data_pos, ' ');
  /* FMT 块大小固定为 20 */
  ByteArray_PutUint32LE(data_pos, 20);
  /* WAVE格式类型：由IMA-ADPCM（17）决定 */
  ByteArray_PutUint16LE(data_pos, 17);
  /* 通道数 */
  if (header_info->num_channels > IMAADPCM_MAX_NUM_CHANNELS) {
    return IMAADPCM_APIRESULT_INVALID_FORMAT;
  }
  ByteArray_PutUint16LE(data_pos, header_info->num_channels);
  /* 采样率 */
  ByteArray_PutUint32LE(data_pos, header_info->sampling_rate);
  /* 数据速度[byte/sec] */
  ByteArray_PutUint32LE(data_pos, header_info->bytes_per_sec);
  /* 块大小 */
  ByteArray_PutUint16LE(data_pos, header_info->block_size);
  /* 每个样本的位数: 4 */
  if (header_info->bits_per_sample != IMAADPCM_BITS_PER_SAMPLE) {
    return IMAADPCM_APIRESULT_INVALID_FORMAT;
  }
  ByteArray_PutUint16LE(data_pos, header_info->bits_per_sample);
  /* fmt 块额外大小：2 */
  ByteArray_PutUint16LE(data_pos, 2);
  /* 每个块的样本数 */
  ByteArray_PutUint16LE(data_pos, header_info->num_samples_per_block);

  /* FACT块 ID */
  ByteArray_PutUint8(data_pos, 'f');
  ByteArray_PutUint8(data_pos, 'a');
  ByteArray_PutUint8(data_pos, 'c');
  ByteArray_PutUint8(data_pos, 't');
  /* FACT 块额外大小：4 */
  ByteArray_PutUint32LE(data_pos, 4);
  /* 样本数 */
  ByteArray_PutUint32LE(data_pos, header_info->num_samples);

  /* 不导出其他chunk，立即去数据chunk */

  /* 数据块 ID */
  ByteArray_PutUint8(data_pos, 'd');
  ByteArray_PutUint8(data_pos, 'a');
  ByteArray_PutUint8(data_pos, 't');
  ByteArray_PutUint8(data_pos, 'a');
  /* 数据块大小 */
  ByteArray_PutUint32LE(data_pos, data_chunk_size);

  /* 成功 */
  return IMAADPCM_APIRESULT_OK;
}

/* 编码工作大小计算 */
int32_t IMAADPCMWAVEncoder_CalculateWorkSize(void)
{
  return IMAADPCM_ALIGNMENT + sizeof(struct IMAADPCMWAVEncoder);
}

/* 编码器句柄创建 */
struct IMAADPCMWAVEncoder *IMAADPCMWAVEncoder_Create(void *work, int32_t work_size)
{
  struct IMAADPCMWAVEncoder *encoder;
  uint8_t *work_ptr;
  uint32_t alloced_by_malloc = 0;

  if ((work == NULL) && (work_size == 0)) {
    work_size = IMAADPCMWAVEncoder_CalculateWorkSize();
    work = malloc((uint32_t)work_size);
    alloced_by_malloc = 1;
  }

  /* 参数检查 */
  if ((work == NULL) || (work_size < IMAADPCMWAVEncoder_CalculateWorkSize())) {
    return NULL;
  }

  work_ptr = (uint8_t *)work;

  /* 对齐后配置 */
  work_ptr = (uint8_t *)IMAADPCM_ROUND_UP((uintptr_t)work_ptr, IMAADPCM_ALIGNMENT);
  encoder = (struct IMAADPCMWAVEncoder *)work_ptr;

  /* 将句柄的内容初始化为 0 */
  memset(encoder, 0, sizeof(struct IMAADPCMWAVEncoder));

  /* 参数未设置 */
  encoder->set_parameter = 0;

  /* 在确保自身安全的情况下预先存储内存 */
  encoder->work = alloced_by_malloc ? work : NULL;

  return encoder;
}

/* 丢弃编码器句柄 */
void IMAADPCMWAVEncoder_Destroy(struct IMAADPCMWAVEncoder *encoder)
{
  if (encoder != NULL) {
    if (encoder->work != NULL) {
      free(encoder->work);
    }
  }
}

/* 单样本编码 */
static uint8_t IMAADPCMCoreEncoder_EncodeSample(
    struct IMAADPCMCoreEncoder *encoder, int16_t sample)
{
  uint8_t nibble;
  int8_t idx;
  int32_t prev, diff, qdiff, delta, stepsize, diffabs, sign;

  assert(encoder != NULL);
  
  /* 接收自动变量中经常引用的变量 */
  prev = encoder->prev_sample;
  idx = encoder->stepsize_index;

  /* 获取步长 */
  stepsize = IMAADPCM_stepsize_table[idx];

  /* 差分 */
  diff = sample - prev;
  sign = diff < 0;
  diffabs = sign ? -diff : diff;

  /* 将差分转换为有符号表示 */
  /* nibble = sign(diff) * round(|diff| * 4 / stepsize) */
  nibble = (uint8_t)IMAADPCM_MIN_VAL((diffabs << 2) / stepsize, 7);
  /* nibble的最高有效位是符号位 */
  if (sign) {
    nibble |= 0x8;
  }

  /* 计算量化差异 */
  delta = nibble & 7;
  qdiff = (stepsize * ((delta << 1) + 1)) >> 3;

  /* 这里出现量化误差 */
  /* printf("%d \n", sign ? (-qdiff - diff) : (qdiff - diff)); */

  /* 加上量化误差 */
  if (sign) {
    prev -= qdiff;
  } else {
    prev += qdiff;
  }
  prev = IMAADPCM_INNER_VAL(prev, -32768, 32767);

  /* 索引更新 */
  idx = (int8_t)(idx + IMAADPCM_index_table[nibble]);
  idx = IMAADPCM_INNER_VAL(idx, 0, 88);

  /* 计算结果的反映 */
  encoder->prev_sample = (int16_t)prev;
  encoder->stepsize_index = idx;

  return nibble;
}

/* 单声道编码 */
static IMAADPCMError IMAADPCMWAVEncoder_EncodeBlockMono(
    struct IMAADPCMCoreEncoder *core_encoder,
    const int16_t *const *input, uint32_t num_samples,
    uint8_t *data, uint32_t data_size, uint32_t *output_size)
{
  uint8_t u8buf;
  uint8_t nibble[2];
  uint32_t smpl;
  uint8_t *data_pos = data;

  /* 参数检查 */
  if ((core_encoder == NULL) || (input == NULL)
      || (data == NULL) || (output_size == NULL)) {
    return IMAADPCM_ERROR_INVALID_ARGUMENT;
  }

  /* 确认是否有足够的数据大小 */
  if (data_size < (num_samples / 2 + 4)) {
    return IMAADPCM_ERROR_INSUFFICIENT_DATA;
  }

  /* 在编码器中设置第一个样本 */
  core_encoder->prev_sample = input[0][0];

  /* 块头编码 */
  ByteArray_PutUint16LE(data_pos, core_encoder->prev_sample);
  ByteArray_PutUint8(data_pos, core_encoder->stepsize_index);
  ByteArray_PutUint8(data_pos, 0); /* reserved */

  /* 块数据编码 */
  for (smpl = 1; smpl < num_samples; smpl += 2) {
    assert((uint32_t)(data_pos - data) < data_size);
    nibble[0] = IMAADPCMCoreEncoder_EncodeSample(core_encoder, input[0][smpl + 0]);
    nibble[1] = IMAADPCMCoreEncoder_EncodeSample(core_encoder, input[0][smpl + 1]);
    assert((nibble[0] <= 0xF) && (nibble[1] <= 0xF));
    u8buf = (uint8_t)((nibble[0] << 0) | (nibble[1] << 4));
    ByteArray_PutUint8(data_pos, u8buf);
  }

  /* 设置导出大小 */
  (*output_size) = (uint32_t)(data_pos - data);
  return IMAADPCM_ERROR_OK;
}

/* 立体声块编码 */
static IMAADPCMError IMAADPCMWAVEncoder_EncodeBlockStereo(
    struct IMAADPCMCoreEncoder *core_encoder,
    const int16_t *const *input, uint32_t num_samples,
    uint8_t *data, uint32_t data_size, uint32_t *output_size)
{
  uint32_t u32buf;
  uint8_t nibble[8];
  uint32_t ch, smpl;
  uint8_t *data_pos = data;

  /* 参数检查 */
  if ((core_encoder == NULL) || (input == NULL)
      || (data == NULL) || (output_size == NULL)) {
    return IMAADPCM_ERROR_INVALID_ARGUMENT;
  }

  /* 检查数据大小是否足够 */
  if (data_size < (num_samples + 4)) {
    return IMAADPCM_ERROR_INSUFFICIENT_DATA;
  }

  /* 在编码器中设置第一个样本 */
  for (ch = 0; ch < 2; ch++) {
    core_encoder[ch].prev_sample = input[ch][0];
  }

  /* 块头编码 */
  for (ch = 0; ch < 2; ch++) {
    ByteArray_PutUint16LE(data_pos, core_encoder[ch].prev_sample);
    ByteArray_PutUint8(data_pos, core_encoder[ch].stepsize_index);
    ByteArray_PutUint8(data_pos, 0); /* reserved */
  }

  /* 块数据编码 */
  for (smpl = 1; smpl < num_samples; smpl += 8) {
    for (ch = 0; ch < 2; ch++) {
      assert((uint32_t)(data_pos - data) < data_size);
      nibble[0] = IMAADPCMCoreEncoder_EncodeSample(&(core_encoder[ch]), input[ch][smpl + 0]);
      nibble[1] = IMAADPCMCoreEncoder_EncodeSample(&(core_encoder[ch]), input[ch][smpl + 1]);
      nibble[2] = IMAADPCMCoreEncoder_EncodeSample(&(core_encoder[ch]), input[ch][smpl + 2]);
      nibble[3] = IMAADPCMCoreEncoder_EncodeSample(&(core_encoder[ch]), input[ch][smpl + 3]);
      nibble[4] = IMAADPCMCoreEncoder_EncodeSample(&(core_encoder[ch]), input[ch][smpl + 4]);
      nibble[5] = IMAADPCMCoreEncoder_EncodeSample(&(core_encoder[ch]), input[ch][smpl + 5]);
      nibble[6] = IMAADPCMCoreEncoder_EncodeSample(&(core_encoder[ch]), input[ch][smpl + 6]);
      nibble[7] = IMAADPCMCoreEncoder_EncodeSample(&(core_encoder[ch]), input[ch][smpl + 7]);
      assert((nibble[0] <= 0xF) && (nibble[1] <= 0xF) && (nibble[2] <= 0xF) && (nibble[3] <= 0xF)
          && (nibble[4] <= 0xF) && (nibble[5] <= 0xF) && (nibble[6] <= 0xF) && (nibble[7] <= 0xF));
      u32buf  = (uint32_t)(nibble[0] <<  0);
      u32buf |= (uint32_t)(nibble[1] <<  4);
      u32buf |= (uint32_t)(nibble[2] <<  8);
      u32buf |= (uint32_t)(nibble[3] << 12);
      u32buf |= (uint32_t)(nibble[4] << 16);
      u32buf |= (uint32_t)(nibble[5] << 20);
      u32buf |= (uint32_t)(nibble[6] << 24);
      u32buf |= (uint32_t)(nibble[7] << 28);
      ByteArray_PutUint32LE(data_pos, u32buf);
    }
  }

  /* 设置导出大小 */
  (*output_size) = (uint32_t)(data_pos - data);
  return IMAADPCM_ERROR_OK;
}

/* 单数据块编码 */
static IMAADPCMApiResult IMAADPCMWAVEncoder_EncodeBlock(
    struct IMAADPCMWAVEncoder *encoder,
    const int16_t *const *input, uint32_t num_samples, 
    uint8_t *data, uint32_t data_size, uint32_t *output_size)
{
  IMAADPCMError err;
  const struct IMAADPCMWAVEncodeParameter *enc_param;

  /* 参数检查 */
  if ((encoder == NULL) || (data == NULL)
      || (input == NULL) || (output_size == NULL)) {
    return IMAADPCM_APIRESULT_INVALID_ARGUMENT;
  }
  enc_param = &(encoder->encode_paramemter);

  /* 块解码 */
  switch (enc_param->num_channels) {
    case 1:
      err = IMAADPCMWAVEncoder_EncodeBlockMono(encoder->core_encoder, 
          input, num_samples, data, data_size, output_size);
      break;
    case 2:
      err = IMAADPCMWAVEncoder_EncodeBlockStereo(encoder->core_encoder, 
          input, num_samples, data, data_size, output_size);
      break;
    default:
      return IMAADPCM_APIRESULT_INVALID_FORMAT;
  }

  /* 解码时的错误句柄 */
  if (err != IMAADPCM_ERROR_OK) {
    switch (err) {
      case IMAADPCM_ERROR_INVALID_ARGUMENT:
        return IMAADPCM_APIRESULT_INVALID_ARGUMENT;
      case IMAADPCM_ERROR_INVALID_FORMAT:
        return IMAADPCM_APIRESULT_INVALID_FORMAT;
      case IMAADPCM_ERROR_INSUFFICIENT_BUFFER:
        return IMAADPCM_APIRESULT_INSUFFICIENT_BUFFER;
      default:
        return IMAADPCM_APIRESULT_NG;
    }
  }

  return IMAADPCM_APIRESULT_OK;
}

/* 将编码参数转换成信头 */
static IMAADPCMError IMAADPCMWAVEncoder_ConvertParameterToHeader(
    const struct IMAADPCMWAVEncodeParameter *enc_param, uint32_t num_samples,
    struct IMAADPCMWAVHeaderInfo *header_info)
{
  uint32_t block_data_size;
  struct IMAADPCMWAVHeaderInfo tmp_header = {0, };

  /* 参数检查 */
  if ((enc_param == NULL) || (header_info == NULL)) {
    return IMAADPCM_ERROR_INVALID_ARGUMENT;
  }

  /* 每样本位数固定4 */
  if (enc_param->bits_per_sample != IMAADPCM_BITS_PER_SAMPLE) {
    return IMAADPCM_ERROR_INVALID_FORMAT;
  }

  /* 确定信头大小 */
  tmp_header.header_size = IMAADPCMWAVENCODER_HEADER_SIZE;
  /* 总样本数 */
  tmp_header.num_samples = num_samples;

  /* 可以按原样放入信头的成员 */
  tmp_header.num_channels = enc_param->num_channels;
  tmp_header.sampling_rate = enc_param->sampling_rate;
  tmp_header.bits_per_sample = enc_param->bits_per_sample;
  tmp_header.block_size = enc_param->block_size;

  /* 需要计算的成员 */
  if (enc_param->block_size <= enc_param->num_channels * 4) {
    /* 没有存储数据的区域 */
    return IMAADPCM_ERROR_INVALID_FORMAT;
  }
  /* 4 是每个通道的头部区域大小 */
  assert(enc_param->block_size >= (enc_param->num_channels * 4));
  block_data_size = (uint32_t)(enc_param->block_size - (enc_param->num_channels * 4));
  assert((block_data_size * 8) % (uint32_t)(enc_param->bits_per_sample * enc_param->num_channels) == 0);
  assert((enc_param->bits_per_sample * enc_param->num_channels) != 0);
  tmp_header.num_samples_per_block = (uint16_t)((block_data_size * 8) / (uint32_t)(enc_param->bits_per_sample * enc_param->num_channels));
  /* 信头中的部分+1 */
  tmp_header.num_samples_per_block++;
  assert(tmp_header.num_samples_per_block != 0);
  tmp_header.bytes_per_sec = (enc_param->block_size * enc_param->sampling_rate) / tmp_header.num_samples_per_block;

  /* 成功 */
  (*header_info) = tmp_header;

  return IMAADPCM_ERROR_OK;
}

/* 编码参数设置 */
IMAADPCMApiResult IMAADPCMWAVEncoder_SetEncodeParameter(
    struct IMAADPCMWAVEncoder *encoder, const struct IMAADPCMWAVEncodeParameter *parameter)
{
  struct IMAADPCMWAVHeaderInfo tmp_header = {0, };

  /* 参数检查 */
  if ((encoder == NULL) || (parameter == NULL)) {
    return IMAADPCM_APIRESULT_INVALID_ARGUMENT;
  }

  /* 通过转header检查参数设置是否正确 */
  if (IMAADPCMWAVEncoder_ConvertParameterToHeader(parameter, 0, &tmp_header) != IMAADPCM_ERROR_OK) {
    return IMAADPCM_APIRESULT_INVALID_FORMAT;
  }

  /* 参数设置 */
  encoder->encode_paramemter = (*parameter);

  /* 设置参数设置标志 */
  encoder->set_parameter = 1;

  return IMAADPCM_APIRESULT_OK;
}

/* 对包括标题在内的整个文件进行编码 */
IMAADPCMApiResult IMAADPCMWAVEncoder_EncodeWhole(
    struct IMAADPCMWAVEncoder *encoder,
    const int16_t *const *input, uint32_t num_samples,
    uint8_t *data, uint32_t data_size, uint32_t *output_size)
{
  IMAADPCMApiResult ret;
  uint32_t progress, ch, write_size, write_offset, num_encode_samples;
  uint8_t *data_pos;
  const int16_t *input_ptr[IMAADPCM_MAX_NUM_CHANNELS];
  struct IMAADPCMWAVHeaderInfo header = { 0, };

  /* 参数检查 */
  if ((encoder == NULL) || (input == NULL)
      || (data == NULL) || (output_size == NULL)) {
    return IMAADPCM_APIRESULT_INVALID_ARGUMENT;
  }

  /* 如果未设置参数，则无法编码 */
  if (encoder->set_parameter == 0) {
    return IMAADPCM_APIRESULT_PARAMETER_NOT_SET;
  }

  /* 获取开始位置 */
  data_pos = data;

  /* 将编码参数转换成信头 */
  if (IMAADPCMWAVEncoder_ConvertParameterToHeader(&(encoder->encode_paramemter), num_samples, &header) != IMAADPCM_ERROR_OK) {
    return IMAADPCM_APIRESULT_INVALID_FORMAT;
  }

  /* 头编码 */
  if ((ret = IMAADPCMWAVEncoder_EncodeHeader(&header, data_pos, data_size)) != IMAADPCM_APIRESULT_OK) {
    return ret;
  }

  progress = 0;
  write_offset = IMAADPCMWAVENCODER_HEADER_SIZE;
  data_pos = data + IMAADPCMWAVENCODER_HEADER_SIZE;
  while (progress < num_samples) {
    /* 编码样本数的确定 */
    num_encode_samples 
      = IMAADPCM_MIN_VAL(header.num_samples_per_block, num_samples - progress);
    /* 一组样本参考位置 */
    for (ch = 0; ch < header.num_channels; ch++) {
      input_ptr[ch] = &input[ch][progress];
    }

    /* 块编码 */
    if ((ret = IMAADPCMWAVEncoder_EncodeBlock(encoder,
            input_ptr, num_encode_samples,
            data_pos, data_size - write_offset, &write_size)) != IMAADPCM_APIRESULT_OK) {
      return ret;
    }

    /* 进度更新 */
    data_pos      += write_size;
    write_offset  += write_size;
    progress      += num_encode_samples;
    assert(write_size <= header.block_size);
    assert(write_offset <= data_size);
  }

  /* 成功 */
  (*output_size) = write_offset;
  return IMAADPCM_APIRESULT_OK;
}

