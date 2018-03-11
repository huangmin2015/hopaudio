
/* xdctools header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>

#include <stdio.h>

/* package header files */
#include <ti/ipc/remoteproc/Resource.h>
#include <ti/sysbios/hal/Cache.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include "data.h"
_SharebufStr  SharebufStr;

Semaphore_Params params;
Error_Block eb1;
extern HeapMem_Handle myHeap;

Void share_data_init(Void)
{

    memset(&SharebufStr,0,sizeof(SharebufStr));
    Error_init(&eb1);
    /* SEM Initializations */
    Semaphore_Params_init(&params);
    Log_print0(Diags_ENTRY, " share_data_init ...:");
    /* Create semaphores to wait for buffer reclaiming */
    SharebufStr.SemShareDate = Semaphore_create(0, &params, &eb1);
    /* register with xdc.runtime to get a diags mask */
    alloc_buffer();
}

int alloc_buffer(void)
{
    int res=0;
    int count;
    IHeap_Handle iheap;
    iheap = HeapMem_Handle_to_xdc_runtime_IHeap(myHeap);
    Error_init(&eb1);

    for(count = 0; count < (SHARE_BUFFER_COUNT); count ++){
        SharebufStr.buf[count].databuf = Memory_calloc(iheap, SHARE_DATA_LEN_MAX,128, &eb1);
        Log_print2(Diags_ENTRY, "-- databuf:%d  addr:%2x",  count ,  SharebufStr.buf[count].databuf);
        if(NULL == SharebufStr.buf[count].databuf){
            Log_print0(Diags_ENTRY, " MEM_calloc failed...:");
        }
    }
    return res;
}

void Write_buffer(unsigned char  * p,unsigned int len)
{
    unsigned char *pIndex;
  //  if( SharebufStr.Tflag==1){

        pIndex=SharebufStr.buf[SharebufStr.WriteIndex].databuf+SharebufStr.buf[SharebufStr.WriteIndex].Writeoffset;
        memcpy(pIndex,p,len);
        SharebufStr.buf[SharebufStr.WriteIndex].Writeoffset += len;
        if(SharebufStr.buf[SharebufStr.WriteIndex].Writeoffset>=SHARE_DATA_LEN_MAX){
            SharebufStr.buf[SharebufStr.WriteIndex].Writeoffset=0;
            SharebufStr.buf[SharebufStr.WriteIndex].ReadEnable=1;
            SharebufStr.WriteIndex=(SharebufStr.WriteIndex+1) % SHARE_BUFFER_COUNT;
        }
  //  }

}

int Read_buffer(unsigned char  * p,unsigned int len)
{
    int res=0;
    if( SharebufStr.Tflag==1){
       if(SharebufStr.buf[SharebufStr.ReadIndex].ReadEnable==1){
           memcpy(p,SharebufStr.buf[SharebufStr.ReadIndex].databuf,SHARE_DATA_LEN_MAX);
           SharebufStr.buf[SharebufStr.ReadIndex].ReadEnable=0;
           SharebufStr.ReadIndex = (SharebufStr.ReadIndex+1) % SHARE_BUFFER_COUNT;
           res=SHARE_DATA_LEN_MAX;
       }
    }
    return res;

}

void Start_record(void)
{
    int i;
    SharebufStr.Tflag = 1;
    SharebufStr.ReadIndex=0;
    SharebufStr.WriteIndex=0;
    for(i=0;i<SHARE_BUFFER_COUNT;i++){
        SharebufStr.buf[i].Writeoffset=0;
        SharebufStr.buf[i].ReadEnable=0;
    }
}

void Stop_record(void)
{
    int i;
    SharebufStr.Tflag = 0;

}
