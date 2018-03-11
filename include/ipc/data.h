
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>

#include <stdio.h>
#define SHARE_DATA_LEN_MAX     4096
#define SHARE_BUFFER_COUNT     2

typedef struct {
    unsigned char  * databuf;
    unsigned int Writeoffset;
    Bool ReadEnable;
}_databuf;

typedef struct {
    _databuf buf[SHARE_BUFFER_COUNT];
    unsigned char ReadIndex;     //
    unsigned char WriteIndex;
    Semaphore_Handle SemShareDate;
    unsigned char Tflag;       //flag to write data to buf;
}_SharebufStr;


Void share_data_init(Void);
int  alloc_buffer(void);
void Write_buffer(unsigned char  * p,unsigned int len);
int Read_buffer(unsigned char  * p,unsigned int len);
void Start_record(void);
void Stop_record(void);
