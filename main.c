#include "ima_adpcm.h"
#include "wav.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

/* 块大小目前固定为 1024 */
#define IMAADPCMCUI_BLOCK_SIZE      1024

/* 解码过程 */
static int do_decode(const char *adpcm_filename, const char *decoded_filename)
{
  FILE                          *fp;
  struct stat                   fstat;
  uint8_t                       *buffer;
  uint32_t                      buffer_size;
  struct IMAADPCMWAVDecoder     *decoder;
  struct IMAADPCMWAVHeaderInfo  header;
  struct WAVFile                *wav;
  struct WAVFileFormat          wavformat;
  int16_t                       *output[IMAADPCM_MAX_NUM_CHANNELS];
  uint32_t                      ch, smpl;
  IMAADPCMApiResult             ret;

  /* 文件打开 */
  fp = fopen(adpcm_filename, "rb");
  if (fp == NULL) {
    fprintf(stderr, "Failed to open %s. \n", adpcm_filename);
    return 1;
  }

  /* 输入文件大小获取/缓冲区分配 */
  stat(adpcm_filename, &fstat);
  buffer_size = (uint32_t)fstat.st_size;
  buffer = (uint8_t *)malloc(buffer_size);
  /* 加载数据到缓冲区 */
  fread(buffer, sizeof(uint8_t), buffer_size, fp);
  fclose(fp);

  /* 解码器创建 */
  decoder = IMAADPCMWAVDecoder_Create(NULL, 0);

  /* 读取信头 */
  if ((ret = IMAADPCMWAVDecoder_DecodeHeader(buffer, buffer_size, &header))
      != IMAADPCM_APIRESULT_OK) {
    fprintf(stderr, "Failed to read header. API result: %d \n", ret);
    return 1;
  }

  /* 确保输出缓冲区 */
  for (ch = 0; ch < header.num_channels; ch++) {
    output[ch] = malloc(sizeof(int16_t) * header.num_samples);
  }

  /* 解码所有数据 */
  if ((ret = IMAADPCMWAVDecoder_DecodeWhole(decoder, 
        buffer, buffer_size, output, 
        header.num_channels, header.num_samples)) != IMAADPCM_APIRESULT_OK) {
    fprintf(stderr, "Failed to decode. API result: %d \n", ret);
    return 1;
  }

  /* 创建输出文件 */
  wavformat.data_format = WAV_DATA_FORMAT_PCM;
  wavformat.num_channels = header.num_channels;
  wavformat.sampling_rate = header.sampling_rate;
  wavformat.bits_per_sample = 16;
  wavformat.num_samples = header.num_samples;
  wav = WAV_Create(&wavformat);

  /* PCM导出 */
  for (ch = 0; ch < header.num_channels; ch++) {
    for (smpl = 0; smpl < header.num_samples; smpl++) {
      WAVFile_PCM(wav, smpl, ch) = (output[ch][smpl] << 16);
    }
  }

  WAV_WriteToFile(decoded_filename, wav);

  IMAADPCMWAVDecoder_Destroy(decoder);
  for (ch = 0; ch < header.num_channels; ch++) {
    free(output[ch]);
  }
  WAV_Destroy(wav);
  free(buffer);

  return 0;
}

/* 编码处理 */
static int do_encode(const char *wav_file, const char *encoded_filename)
{
  FILE                              *fp;
  struct WAVFile                    *wavfile;
  struct stat                       fstat;
  int16_t                           *input[IMAADPCM_MAX_NUM_CHANNELS];
  uint32_t                          ch, smpl, buffer_size, output_size;
  uint32_t                          num_channels, num_samples;
  uint8_t                           *buffer;
  struct IMAADPCMWAVEncodeParameter enc_param;
  struct IMAADPCMWAVEncoder         *encoder;
  IMAADPCMApiResult                 api_result;

  /* 获取输入 wav */
  wavfile = WAV_CreateFromFile(wav_file);
  if (wavfile == NULL) {
    fprintf(stderr, "Failed to open %s. \n", wav_file);
    return 1;
  }

  num_channels = wavfile->format.num_channels;
  num_samples = wavfile->format.num_samples;

  /* 输出数据区分配 */
  for (ch = 0; ch < num_channels; ch++) {
    input[ch] = malloc(sizeof(int16_t) * num_samples);
  }
  /* 确保一个与输入wav大小相同的输出区域（预计不会增加） */
  stat(wav_file, &fstat);
  buffer_size = (uint32_t)fstat.st_size;
  buffer = malloc(buffer_size);

  /* 16 位宽的数据采集 */
  for (ch = 0; ch < num_channels; ch++) {
    for (smpl = 0; smpl < num_samples; smpl++) {
      input[ch][smpl] = (int16_t)(WAVFile_PCM(wavfile, smpl, ch) >> 16);
    }
  }

  /* 处理创建 */
  encoder = IMAADPCMWAVEncoder_Create(NULL, 0);

  /* 设置编码参数 */
  enc_param.num_channels    = (uint16_t)num_channels;
  enc_param.sampling_rate   = wavfile->format.sampling_rate;
  enc_param.bits_per_sample = 4;
  enc_param.block_size      = IMAADPCMCUI_BLOCK_SIZE;
  if ((api_result = IMAADPCMWAVEncoder_SetEncodeParameter(encoder, &enc_param))
      != IMAADPCM_APIRESULT_OK) {
    fprintf(stderr, "Failed to set encode parameter. API result:%d \n", api_result);
    return 1;
  }

  /* 编码 */
  if ((api_result = IMAADPCMWAVEncoder_EncodeWhole(
        encoder, (const int16_t *const *)input, num_samples,
        buffer, buffer_size, &output_size)) != IMAADPCM_APIRESULT_OK) {
    fprintf(stderr, "Failed to encode. API result:%d \n", api_result);
    return 1;
  }

  /* 文件导出 */
  fp = fopen(encoded_filename, "wb");
  if (fp == NULL) {
    fprintf(stderr, "Failed to open output file %s \n", encoded_filename);
    return 1;
  }
  if (fwrite(buffer, sizeof(uint8_t), output_size, fp) < output_size) {
    fprintf(stderr, "Warning: failed to write encoded data \n");
    return 1;
  }
  fclose(fp);

  /* 区域开放 */
  IMAADPCMWAVEncoder_Destroy(encoder);
  free(buffer);
  for (ch = 0; ch < num_channels; ch++) {
    free(input[ch]);
  }
  WAV_Destroy(wavfile);

  return 0;
}

/* 残差输出处理 */
static int do_residual_output(const char *wav_file, const char *residual_filename)
{
  struct WAVFile                    *wavfile;
  struct stat                       fstat;
  int16_t                           *pcmdata[IMAADPCM_MAX_NUM_CHANNELS];
  uint32_t                          ch, smpl, buffer_size, output_size;
  uint32_t                          num_channels, num_samples;
  uint8_t                           *buffer;
  struct IMAADPCMWAVEncodeParameter enc_param;
  struct IMAADPCMWAVEncoder         *encoder;
  struct IMAADPCMWAVDecoder         *decoder;
  IMAADPCMApiResult                 api_result;

  /* 获取输入 wav */
  wavfile = WAV_CreateFromFile(wav_file);
  if (wavfile == NULL) {
    fprintf(stderr, "Failed to open %s. \n", wav_file);
    return 1;
  }

  num_channels = wavfile->format.num_channels;
  num_samples = wavfile->format.num_samples;

  /* 输出数据区分配 */
  for (ch = 0; ch < num_channels; ch++) {
    pcmdata[ch] = malloc(sizeof(int16_t) * num_samples);
  }
  /* 确保一个与输入wav大小相同的输出区域（预计不会增加） */
  stat(wav_file, &fstat);
  buffer_size = (uint32_t)fstat.st_size;
  buffer = malloc(buffer_size);

  /* 16 位宽的数据采集 */
  for (ch = 0; ch < num_channels; ch++) {
    for (smpl = 0; smpl < num_samples; smpl++) {
      pcmdata[ch][smpl] = (int16_t)(WAVFile_PCM(wavfile, smpl, ch) >> 16);
    }
  }

  /* 处理创建 */
  encoder = IMAADPCMWAVEncoder_Create(NULL, 0);
  decoder = IMAADPCMWAVDecoder_Create(NULL, 0);

  /* 设置编码参数 */
  enc_param.num_channels    = (uint16_t)num_channels;
  enc_param.sampling_rate   = wavfile->format.sampling_rate;
  enc_param.bits_per_sample = 4;
  enc_param.block_size      = IMAADPCMCUI_BLOCK_SIZE;
  if ((api_result = IMAADPCMWAVEncoder_SetEncodeParameter(encoder, &enc_param))
      != IMAADPCM_APIRESULT_OK) {
    fprintf(stderr, "Failed to set encode parameter. API result:%d \n", api_result);
    return 1;
  }

  /* 编码 */
  if ((api_result = IMAADPCMWAVEncoder_EncodeWhole(
        encoder, (const int16_t *const *)pcmdata, num_samples,
        buffer, buffer_size, &output_size)) != IMAADPCM_APIRESULT_OK) {
    fprintf(stderr, "Failed to encode. API result:%d \n", api_result);
    return 1;
  }

  /* 按原样解码 */
  if ((api_result = IMAADPCMWAVDecoder_DecodeWhole(decoder, 
        buffer, output_size, pcmdata, num_channels, num_samples)) != IMAADPCM_APIRESULT_OK) {
    fprintf(stderr, "Failed to decode. API result: %d \n", api_result);
    return 1;
  }

  /* 残差（量化误差）计算 */
  for (ch = 0; ch < num_channels; ch++) {
    for (smpl = 0; smpl < num_samples; smpl++) {
      int32_t decoded = pcmdata[ch][smpl] << 16;
      WAVFile_PCM(wavfile, smpl, ch) -= decoded;
    }
  }

  /* 将残差写入文件 */
  WAV_WriteToFile(residual_filename, wavfile);

  /* 区域开放 */
  IMAADPCMWAVEncoder_Destroy(encoder);
  IMAADPCMWAVDecoder_Destroy(decoder);
  free(buffer);
  for (ch = 0; ch < num_channels; ch++) {
    free(pcmdata[ch]);
  }
  WAV_Destroy(wavfile);

  return 0;
}

/* 使用方法 */
static void print_usage(const char* program_name)
{
  printf(
      "Usage: %s -[edr] INPUT.wav OUTPUT.wav \n" \
      "-e: encode mode (PCM wav -> IMA-ADPCM wav)\n" \
      "-d: decode mode (IMA-ADPCM wav -> PCM wav)\n" \
      "-r: output residual (PCM wav -> Residual PCM wav)\n", 
      program_name);
}

/* 主程序 */
int main(int argc, char **argv)
{
  clock_t start_time=clock();
  int ret;
  const char *option;
  const char *in_filename, *out_filename;

  /* 参数的数量超出预料 */
  if (argc != 4) {
    print_usage(argv[0]);
    return 1;
  }

  /* 获取选项字符串 */
  option        = argv[1];
  in_filename   = argv[2];
  out_filename  = argv[3];

  /* 无效参数 */
  if ((option == NULL) || (in_filename == NULL) || (out_filename == NULL)) {
    print_usage(argv[0]);
    return 1;
  }
  
  /* 编码/解码调用 */
  if (strncmp(option, "-e", 2) == 0) {
    ret = do_encode(in_filename, out_filename);
  } else if (strncmp(option, "-d", 2) == 0) {
    ret = do_decode(in_filename, out_filename);
  } else if (strncmp(option, "-r", 2) == 0) {
    ret = do_residual_output(in_filename, out_filename);
  } else {
    print_usage(argv[0]);
    return 1;
  }
  clock_t end_time=clock();
  printf("Running time is: %ld ms",(end_time-start_time)/CLOCKS_PER_SEC*1000);

  return ret;
}
