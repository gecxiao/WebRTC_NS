#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <windows.h>

#define DR_MP3_IMPLEMENTATION

#include "dr_mp3.h"

#define DR_WAV_IMPLEMENTATION

#include "dr_wav.h"
#include "timing.h"

#include "noise_suppression.h"

#ifndef nullptr
#define nullptr 0
#endif

#ifndef MIN
#define MIN(A, B)        ((A) < (B) ? (A) : (B))
#endif

void wavWrite_int16(char *filename, int16_t *buffer, int sampleRate, uint64_t totalSampleCount, uint32_t channels) {
    drwav_data_format format;
    format.container = drwav_container_riff;
    format.format = DR_WAVE_FORMAT_PCM;
    format.channels = channels;
    format.sampleRate = (drwav_uint32) sampleRate;
    format.bitsPerSample = 16;

    drwav *pWav = drwav_open_file_write(filename, &format);
    if (pWav) {
        drwav_uint64 samplesWritten = drwav_write(pWav, totalSampleCount, buffer);
        drwav_uninit(pWav);
        if (samplesWritten != totalSampleCount) {
            fprintf(stderr, "write file [%s] error.\n", filename);
            exit(1);
        }
    }
}

int16_t *wavRead_int16(const char *filename, uint32_t *sampleRate, uint64_t *sampleCount, uint32_t *channels) {
    drwav_uint64 totalSampleCount = 0;
    int16_t *input = drwav_open_file_and_read_pcm_frames_s16(filename, channels, sampleRate, &totalSampleCount);
    if (input == NULL) {
        drmp3_config pConfig;
        float *mp3_buf = drmp3_open_file_and_read_f32(filename, &pConfig, &totalSampleCount);
        if (mp3_buf != NULL) {
            *channels = pConfig.outputChannels;
            *sampleRate = pConfig.outputSampleRate;
        }
        input = (int16_t *) mp3_buf;
        for (int32_t i = 0; i < *sampleCount; ++i) {
            input[i] = (int16_t)drwav_clamp((mp3_buf[i] * 32768.0f), -32768, 32767);
        }
    }
    if (input == NULL) {
        fprintf(stderr, "read file [%s] error.\n", filename);
        exit(1);
    }
    *sampleCount = totalSampleCount * (*channels);

    return input;
}


//分割路径函数
void splitpath(const char *path, char *drv, char *dir, char *name, char *ext) {
    const char *end;
    const char *p;
    const char *s;
    if (path[0] && path[1] == ':') {
        if (drv) {
            *drv++ = *path++;
            *drv++ = *path++;
            *drv = '\0';
        }
    } else if (drv)
        *drv = '\0';
    for (end = path; *end && *end != ':';)
        end++;
    for (p = end; p > path && *--p != '\\' && *p != '/';)
        if (*p == '.') {
            end = p;
            break;
        }
    if (ext)
        for (s = end; (*ext = *s++);)
            ext++;
    for (p = end; p > path;)
        if (*--p == '\\' || *p == '/') {
            p++;
            break;
        }
    if (name) {
        for (s = p; s < end;)
            *name++ = *s++;
        *name = '\0';
    }
    if (dir) {
        for (s = path; s < p;)
            *dir++ = *s++;
        *dir = '\0';
    }
}

enum nsLevel {
    kLow,
    kModerate,
    kHigh,
    kVeryHigh
};


int nsProcess(int16_t *buffer, uint32_t sampleRate, uint64_t samplesCount, uint32_t channels, enum nsLevel level) {
    if (buffer == nullptr) return -1;
    if (samplesCount == 0) return -1;
    size_t samples = MIN(160, sampleRate / 100);
    if (samples == 0) return -1;
    uint32_t num_bands = 1;
    int16_t *input = buffer;
    size_t frames = (samplesCount / (samples * channels));
    int16_t *frameBuffer = (int16_t *) malloc(sizeof(*frameBuffer) * channels * samples);
    NsHandle **NsHandles = (NsHandle **) malloc(channels * sizeof(NsHandle *));
    if (NsHandles == NULL || frameBuffer == NULL) {
        if (NsHandles)
            free(NsHandles);
        if (frameBuffer)
            free(frameBuffer);
        fprintf(stderr, "malloc error.\n");
        return -1;
    }
    for (int i = 0; i < channels; i++) {
        NsHandles[i] = WebRtcNs_Create();
        if (NsHandles[i] != NULL) {
            int status = WebRtcNs_Init(NsHandles[i], sampleRate);
            if (status != 0) {
                fprintf(stderr, "WebRtcNs_Init fail\n");
                WebRtcNs_Free(NsHandles[i]);
                NsHandles[i] = NULL;
            } else {
                status = WebRtcNs_set_policy(NsHandles[i], level);
                if (status != 0) {
                    fprintf(stderr, "WebRtcNs_set_policy fail\n");
                    WebRtcNs_Free(NsHandles[i]);
                    NsHandles[i] = NULL;
                }
            }
        }
        if (NsHandles[i] == NULL) {
            for (int x = 0; x < i; x++) {
                if (NsHandles[x]) {
                    WebRtcNs_Free(NsHandles[x]);
                }
            }
            free(NsHandles);
            free(frameBuffer);
            return -1;
        }
    }
    for (int i = 0; i < frames; i++) {
        for (int c = 0; c < channels; c++) {
            for (int k = 0; k < samples; k++)
                frameBuffer[k] = input[k * channels + c];

            int16_t *nsIn[1] = {frameBuffer};   //ns input[band][data]
            int16_t *nsOut[1] = {frameBuffer};  //ns output[band][data]
            WebRtcNs_Analyze(NsHandles[c], nsIn[0]);
            WebRtcNs_Process(NsHandles[c], (const int16_t *const *) nsIn, num_bands, nsOut);
            for (int k = 0; k < samples; k++)
                input[k * channels + c] = frameBuffer[k];
        }
        input += samples * channels;
    }

    for (int i = 0; i < channels; i++) {
        if (NsHandles[i]) {
            WebRtcNs_Free(NsHandles[i]);
        }
    }
    free(NsHandles);
    free(frameBuffer);
    return 1;
}

void noise_suppression(char *in_file, char *out_file) {
    //音频采样率
    uint32_t sampleRate = 0;
    uint32_t channels = 0;
    //总音频采样数
    uint64_t inSampleCount = 0;
    int16_t *inBuffer = wavRead_int16(in_file, &sampleRate, &inSampleCount, &channels);

    //如果加载成功
    if (inBuffer != nullptr) {
        double startTime = now();
        nsProcess(inBuffer, sampleRate, inSampleCount, channels, kModerate);
        double time_interval = calcElapsed(startTime, now());
        printf("time interval: %d ms\n ", (int) (time_interval * 1000));


        wavWrite_int16(out_file, inBuffer, sampleRate, inSampleCount, channels);
        free(inBuffer);
    }
}

//int main(int argc, char *argv[]) {
//    //if (argc < 2)
//    //    return -1;
//    //char currentPath[200],subPath[200];
//    //subPath[200];
//    WIN32_FIND_DATA findFileData;
//    HANDLE hFind;
//    char *currentPath = "D:\\test\\";
//    char base[1000];
//    char *in_file;
//    hFind=FindFirstFile(currentPath,&findFileData);
//    while(TRUE)
//    {
//        char *in_file = strcat(currentPath,findFileData.cFileName);
//        printf("%s",in_file);
//        char drive[3];
//        char dir[256];
//        char fname[256];
//        char ext[256];
//        char out_file[1024];
//        splitpath(in_file, drive, dir, fname, ext);
//        sprintf(out_file, "%s%s%s_out%s", drive, dir, fname, ext);
//        noise_suppression(in_file, out_file);
//        if(!FindNextFile(hFind,&findFileData))
//        {//find next file or directory
//            break;
//        }
//    }
//    FindClose(hFind);
//    // *in_file = "D:\\seg_audio\\148359_seg_01.wav";
////    char drive[3];
////    char dir[256];
////    char fname[256];
////    char ext[256];
////    char out_file[1024];
////    splitpath(in_file, drive, dir, fname, ext);
////    sprintf(out_file, "%s%s%s_out%s", drive, dir, fname, ext);
////    noise_suppression(in_file, out_file);
//
//   // printf("press any key to exit. \n");
//    //getchar();
//
//    return 0;
//}

//void loopThrough(char* path,char* findType)
//{
//    char currentPath[200],subPath[200];
//    WIN32_FIND_DATA findFileData;
//    HANDLE hFind;
//
//    sprintf(currentPath,"%s\\%s",path,findType);
//    //find first file or directory under path
//    hFind=FindFirstFile(currentPath,&findFileData);
//
//    if(hFind==INVALID_HANDLE_VALUE)
//    {//first file or directory non-exists
//        printf("INVALID HANDLE!\n");
//        return;
//    }
//    else
//    {//first file or directory exists
//        while(TRUE)
//        {
//            if(findFileData.dwFileAttributes
//               & FILE_ATTRIBUTE_DIRECTORY)
//            {//find directory
//                if(findFileData.cFileName[0]!='.')
//                {
//                    sprintf(subPath,"%s\\%s",path,
//                            findFileData.cFileName);
//                    loopThrough(subPath,findType);
//                }
//            }
//            else
//            {//find file
//                char in_file[100];
//                char *dir_n = "D:\\seg_audio_origin\\seg_audio\\";
//                char *file_n = (char*)(findFileData.cFileName);
//                strcpy(in_file,dir_n);
//                strcat(in_file,file_n);
//                char drive[3];
//                char dir[256];
//                char fname[256];
//                char ext[256];
//                char out_file[1024];
//                char *out_dir = "D:\\denoised_audio\\";
//                strcpy(out_file,out_dir);
//                splitpath(in_file, drive, dir, fname, ext);
//                sprintf(out_file, "%s%s%s_out%s", drive, dir, fname, ext);
//                noise_suppression(in_file, out_file);
//                memset(in_file,0,100);
////                printf("filename:%s\\%s\n",path,
////                       findFileData.cFileName); //print
//            }
//
//            if(!FindNextFile(hFind,&findFileData))
//            {//find next file or directory
//                break;
//            }
//        }
//
//        FindClose(hFind); // close HANDLE
//    }
//}

//int main(int argc,char* argv[])
//{
//    loopThrough("D:\\seg_audio_origin\\seg_audio","*.*");
//    getchar();
//    return 1;
//}
int main() {
//    char missed_audio[] = {"148809_seg_10", "148809_seg_11", "148809_seg_12",
//                              "148811_seg_01", "148811_seg_02", '148811_seg_03', '148811_seg_04',
//                              '148811_seg_05', '148811_seg_06', '148811_seg_07', '148811_seg_08',
//                              '148811_seg_09', '148811_seg_10', '148811_seg_11', '148811_seg_12',
//                              '148811_seg_13', '148811_seg_14', '148811_seg_15', '148811_seg_16',
//                              '148811_seg_17', '148811_seg_18', '148811_seg_19', '148811_seg_20',
//                              '148812_seg_01', '148812_seg_02', '148812_seg_03', '148812_seg_04',
//                              '148812_seg_05', '148812_seg_06', '148812_seg_07', '148812_seg_08',
//                              '148812_seg_09', '148812_seg_10', '148812_seg_11', '148812_seg_12',
//                              '148812_seg_13', '148812_seg_14', '148812_seg_15', '148813_seg_01',
//                              '148813_seg_02', '148813_seg_03', '148813_seg_04', '148813_seg_05',
//                              '148813_seg_06', '148813_seg_07', '148813_seg_08', '148813_seg_09',
//                              '148813_seg_10', '148813_seg_11', '148814_seg_01', '148814_seg_02',
//                              '148814_seg_03', '148814_seg_04', '148814_seg_05', '148814_seg_06',
//                              '148814_seg_07', '148814_seg_08', '148814_seg_09', '148814_seg_10',
//                              '148814_seg_11', '148814_seg_12', '148815_seg_01', '148815_seg_02',
//                              '148815_seg_03', '148815_seg_04', '148815_seg_05', '148815_seg_06',
//                              '148815_seg_07', '148815_seg_08', '148815_seg_09', '148815_seg_10',
//                              '148815_seg_11', '148815_seg_12', '148815_seg_13', '148815_seg_14',
//                              '148815_seg_15', '148817_seg_01', '148817_seg_02', '148817_seg_03',
//                              '148817_seg_04', '148817_seg_05', '148817_seg_06', '148817_seg_07',
//                              '148817_seg_08', '148817_seg_09', '148817_seg_10', '148817_seg_11',
//                              '148817_seg_12', '148817_seg_13', '148817_seg_14', '148817_seg_15',
//                              '148817_seg_16', '148817_seg_17', '148817_seg_18', '148819_seg_01',
//                              '148819_seg_02', '148819_seg_03', '148819_seg_04', '148819_seg_05',
//                              '148819_seg_06', '148819_seg_07', '148819_seg_08', '148819_seg_09',
//                              '148819_seg_10', '148819_seg_11', '148819_seg_12', '148820_seg_01',
//                              '148820_seg_02', '148820_seg_03', '148820_seg_04', '148820_seg_05',
//                              '148820_seg_06', '148820_seg_07', '148820_seg_08', '148820_seg_09',
//                              '148820_seg_10', '148820_seg_11', '148820_seg_12', '148820_seg_13',
//                              '148820_seg_14', '148822_seg_01', '148822_seg_02', '148822_seg_03',
//                              '148822_seg_04', '148822_seg_05', '148822_seg_06', '148822_seg_07',
//                              '148822_seg_08', '148822_seg_09', '148822_seg_10', '148822_seg_11',
//                              '148822_seg_12', '148822_seg_13', '148823_seg_01', '148823_seg_02',
//                              '148823_seg_03', '148823_seg_04', '148823_seg_05', '148823_seg_06',
//                              '148823_seg_07', '148823_seg_08', '148823_seg_09', '148823_seg_10',
//                              '148823_seg_11', '148823_seg_12', '148827_seg_01', '148827_seg_02',
//                              '148827_seg_03', '148827_seg_04', '148827_seg_05', '148827_seg_06',
//                              '148827_seg_07', '148827_seg_08', '148827_seg_09', '148827_seg_10',
//                              '148827_seg_11', '148827_seg_12', '148827_seg_13', '148827_seg_14',
//                              '148828_seg_01', '148828_seg_02', '148828_seg_03', '148828_seg_04',
//                              '148828_seg_05', '148828_seg_06', '148828_seg_07', '148828_seg_08',
//                              '148828_seg_09', '148828_seg_10', '148828_seg_11', '148828_seg_12',
//                              '148828_seg_13', '148828_seg_14', '148828_seg_15', '148828_seg_16',
//                              '148828_seg_17', '148829_seg_01', '148829_seg_02', '148829_seg_03',
//                              '148829_seg_04', '148829_seg_05', '148829_seg_06', '148829_seg_07',
//                              '148829_seg_08', '148829_seg_09', '148829_seg_10', '148829_seg_11',
//                              '148829_seg_12', '148830_seg_01', '148830_seg_02', '148830_seg_03',
//                              '148830_seg_04', '148830_seg_05', '148830_seg_06', '148830_seg_07',
//                              '148830_seg_08'}
    // char in_file[100] = "D:\\seg_audio_origin\\seg_audio\\148809_seg_10.wav";
    char in_file[100]="D:\\denoise_2\\seg_audio\\148830_seg_08.wav";
//char *dir_n = "D:\\seg_audio_origin\\seg_audio\\";
//char *file_n = (char*)(findFileData.cFileName);
//strcpy(in_file,dir_n);
//strcat(in_file,file_n);
    char drive[3];
    char dir[256];
    char fname[256];
    char ext[256];
    char out_file[1024];
    char *out_dir = "D:\\denoised_2\\";
    strcpy(out_file, out_dir);
    splitpath(in_file, drive, dir, fname, ext);
    sprintf(out_file, "%s%s%s_out%s", drive, dir, fname, ext);
    noise_suppression(in_file, out_file);
    getchar();
    return 1;
}
