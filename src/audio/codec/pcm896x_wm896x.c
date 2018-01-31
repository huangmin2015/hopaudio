
/* ========================================================================== */
/*                            INCLUDE FILES                                   */
/* ========================================================================== */
#include <codec_if.h>

#include <ICodec.h>

struct Reg_table
{
    unsigned char regAddr;
    unsigned char value;
};
static const struct Reg_table pcm186x_reg_slave_mode[] = {

};


 void wm8960_init(unsigned int baseAddr,unsigned char slaveAddr)
{
    // 重置
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0xf, 0x0);
    // 设置电源   /*鐢垫簮*/
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x19, 1<<8 | 1<<7 | 1<<6 | 1<<5 | 1<<4 | 1<<3 | 1<<2 | 1<<1);
   // CodecRegWrite1(baseAddr, 0x19, 1<<8 | 1<<7 | 1<<6 | 1<<5 | 1<<4 | 1<<1);
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x1a, 1<<8 | 1<<7 | 1<<6 | 1<<5 | 1<<4 | 1<<3 | 1<<2 | 1<<1);
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x2F, 1<<5 | 1<<4 | 1<<3 | 1<<2 );

    //ADCLRC/GPIO Pin For GPIO; ADCCLK From  DACClK
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x9, 0x40);
    // 设置时钟   clk
    //Mclk--div1-->SYSCLK---DIV256--->DAC/ADC sample Freq=11.289(MCLK)/256=44.1KHZ
    //CodecRegWrite(baseAddr, 0x4, 0x0);
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x4, 0x04);  //sysclk=22.5792(MCLK)/(2)=11.289MHz Freq=11.289(MCLK)/256=44.1KHZ

    // 设置ADC-DAC
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x5, 0x0);

     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x15, 0x1ff);
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x16, 0x1ff);


    // 设置audio interface
    /* data format: I2S
       word length: 32 bits  */
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x7, 0x0e);

    // 设置OUTPUTS   //volume
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x2, 0xFF | 0x100);
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x3, 0xFF | 0x100);
    //set by pass
    //Letf DAC to Left Output Mixer
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x22,  1 <<8);
    //Right DAC to Right Output Mixer
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x25,  1 <<8 );
    //Reserved
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x26, 1 <<7 );
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x27, 1 <<7 );
    //Letf Speaker Volume
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x28, 0xf9 | 0x100);
    //Right Speaker Volume
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x29, 0xf9 | 0x100);
    //by pass setup
    //CodecRegWrite1(baseAddr, 0x2d, 1 <<7 );
    //CodecRegWrite1(baseAddr, 0x2e, 1 <<7 );
    //class D Control  L/R Speakers enabled
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x31,  1 <<7 | 1<<6 );

    //setup input  INPUT1 INPUT2 INPUT3 CONNECTED TO ADC
    // CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x20, 1<<8 | 1 <<7 | 1 <<6  | 1 <<3);
    // CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x21, 1<<8 | 1 <<7 | 1 <<6  | 1 <<3);

     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x00, 0x117);
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x01, 0x117);

    // 设置DAC VOLUME
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0xa, 0xFF | 0x100);
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0xb, 0xFF | 0x100);
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x2c, 0X24);
     CodecRegWrite_wm8960(baseAddr,slaveAddr, 0x2b, 0X24);
    return;
}

 void PCM1864_init(unsigned int baseAddr,unsigned char slaveAddr)
 {
     //AUDIO ADC INIT;
     //CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x00,0xff);
     CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x00,0x00);
     CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x06,0x41);
     CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x07,0x41);
     CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x08,0x44);
     CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x09,0x44);

     CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x0B,0x44);
     CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x10,0x00);
     CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x11,0x50);
     CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x12,0x00);
     CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x13,0x40);

     CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x20,0x01);

     CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x01,0x40);
     CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x02,0x40);
     CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x03,0x40);
     CodecRegWrite_PCM1864(baseAddr,slaveAddr,0x04,0x40);
 }


/***************************** End Of File ***********************************/
