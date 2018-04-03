
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>

#include <stdio.h>
#define RECORD_16_CHANEL 1
#define SHARE_DATA_LEN_MAX     4096
#ifndef RECORD_16_CHANEL
#define RECORD_CHANEL_NUM      1
#else
#define RECORD_CHANEL_NUM      16
#endif

#define SHARE_BUFFER_COUNT     2

typedef struct {
    unsigned char  *ptr;
    unsigned int WIndex;
    Bool ReadEnable;
}_databuf;

typedef struct {
    _databuf buf[SHARE_BUFFER_COUNT];
    unsigned char RIndex;     //
    unsigned char WIndex;
    Semaphore_Handle SemShareDate;
    unsigned char Tflag;       //flag to write data to buf;
    unsigned int cnt;
}_SharebufStr;


Void share_data_init(Void);
int  alloc_buffer(void);
void Write_buffer(unsigned char  * p,unsigned int len);
int Read_buffer(unsigned char  * p,unsigned int len);
void Start_record(void);
void Stop_record(void);
