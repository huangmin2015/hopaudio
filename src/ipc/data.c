
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
        SharebufStr.buf[count].ptr = Memory_calloc(iheap, SHARE_DATA_LEN_MAX,128, &eb1);
        //Log_print2(Diags_ENTRY, "-- ptr:%d  addr:%2x",  count ,  SharebufStr.buf[count].ptr);
        if(NULL == SharebufStr.buf[count].ptr){
            Log_print0(Diags_ENTRY, " MEM_calloc failed...:");
        }
    }
    return res;
}

void Write_buffer(unsigned char  * p,unsigned int len)
{
    unsigned char *pIndex;
    if( SharebufStr.Tflag==1){
        pIndex=SharebufStr.buf[SharebufStr.WIndex].ptr+SharebufStr.buf[SharebufStr.WIndex].WIndex;
        memcpy(pIndex,p,len);
        SharebufStr.buf[SharebufStr.WIndex].WIndex += len;
        if(SharebufStr.buf[SharebufStr.WIndex].WIndex>=SHARE_DATA_LEN_MAX){
            Log_print2(Diags_ENTRY, "-- buf:%d ready!! cnt:%d", SharebufStr.WIndex,SharebufStr.cnt);
            SharebufStr.cnt++;
            SharebufStr.buf[SharebufStr.WIndex].WIndex=0;
            SharebufStr.buf[SharebufStr.WIndex].ReadEnable=1;
            SharebufStr.WIndex=(SharebufStr.WIndex+1) % SHARE_BUFFER_COUNT;
            // post sem
            Semaphore_post(SharebufStr.SemShareDate);
        }
    }
}

int Read_buffer(unsigned char  *p,unsigned int len)
{
    int res=0;

    if( SharebufStr.Tflag==1){

       if(SharebufStr.buf[SharebufStr.RIndex].ReadEnable==1){
           memcpy(p,SharebufStr.buf[SharebufStr.RIndex].ptr,SHARE_DATA_LEN_MAX);
          // memset(p,SharebufStr.cnt,SHARE_DATA_LEN_MAX);
           SharebufStr.buf[SharebufStr.RIndex].ReadEnable=0;
           Log_print2(Diags_ENTRY, "-- read from buf: %d fb:%2x", SharebufStr.RIndex,*(SharebufStr.buf[SharebufStr.RIndex].ptr));
           SharebufStr.RIndex = (SharebufStr.RIndex+1) % SHARE_BUFFER_COUNT;
           res=SHARE_DATA_LEN_MAX;
       }

    }
    return res;
}

void Start_record(void)
{
    int i;
    SharebufStr.Tflag = 1;
    SharebufStr.RIndex=0;
    SharebufStr.WIndex=0;
    for(i=0;i<SHARE_BUFFER_COUNT;i++){
        SharebufStr.buf[i].WIndex=0;
        SharebufStr.buf[i].ReadEnable=0;
    }
    Semaphore_reset(SharebufStr.SemShareDate,0);
}

void Stop_record(void)
{
    int i;
    SharebufStr.Tflag = 0;
    Semaphore_reset(SharebufStr.SemShareDate,0);
    //
}
