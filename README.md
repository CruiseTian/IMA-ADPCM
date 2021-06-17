# IMA-ADPCM

一个IMA-ADPCM算法的简单实现。

## How to run

首先clone该项目：

```bash
git clone https://github.com/CruiseTian/IMA-ADPCM.git
```

然后使用 `make` 命令编译，得到输出文件 `ima_adpcm` 。

用法如下：

```bash
ima_adpcm -[edr] INPUT.wav OUTPUT.wav
# 参数对应含义
 -e: encode mode (PCM wav -> IMA-ADPCM wav)  
 -d: decode mode (IMA-ADPCM wav -> PCM wav)  
 -r: output residual (PCM wav -> Residual PCM wav)   
```
