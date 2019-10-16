/*
 * spi.c
 *
 *  Created on: Aug 19, 2019
 *      Author: Muhammad.Elzeiny
 */
/*================================================*
 * DEFINES
 * ==============================================*/
#define SPI_PRIVATE_CONFIG
#define Spi_NUMBER_OF_HW_UINTS                    4
/*================================================*
 * INCLUDES
 * ==============================================*/
#include "../../utils/Bit_Math.h"
#include "../../utils/STD_Types.h"
#include "../mcu_hw.h"

#include "../../config/spi_cfg.h"
#include "spi_types.h"
#include "spi.h"

/*================================================*
 * LOCAL FUNCTIONS
 * ==============================================*/
static inline void Spi_SetBitRate(Spi_ChannelType SpiNum ,uint32 BitRate);
static inline void Spi_IntRoutine(Spi_ChannelType channel);
static inline void Spi_writeTxFifo(Spi_ChannelType Channel);
/*================================================*
 * EXTERNS
 * ==============================================*/
extern const Spi_ConfigType Spi_CfgArr[];

/*================================================*
 * LOCAL VARIABLES
 * ==============================================*/
static const uint32 Spi_BaseAddrArr[] = {SPI0_BASE_ADDR, SPI1_BASE_ADDR, SPI2_BASE_ADDR, SPI3_BASE_ADDR};
static Spi_ChannelParamType Spi_ChannelParam[Spi_NUMBER_OF_HW_UINTS];
/*================================================*
 * FUNCTIONS DEFINTIONS
 * ==============================================*/

/*=====================================================================================
 * NAME:        Spi_init
 * DESCRIBTION: Initial Configured Spi Channel with corresponding configuration Table
 * ===================================================================================*/
void Spi_init(void)
{
    uint8 i;
    Spi_ChannelType Channel;
    for (i = 0; i < Spi_NUM_OF_ACTIVATED_UNITS; ++i)
    {
        Channel = Spi_CfgArr[i].Spi_Channel;
        /*(1) Disable Spi to init configuration */
        (SSICR1(Spi_BaseAddrArr[Channel])).B.SSE = 0;

        /*(2) Select whether the SSI is a master or slave  */
        (SSICR1(Spi_BaseAddrArr[Channel])).B.MS = Spi_CfgArr[i].CFG_OprMode;

        /*configure loop back test */
        (SSICR1(Spi_BaseAddrArr[Channel])).B.LBM = Spi_CfgArr[i].CFG_LoopBack;

        /*configure Tx Interrupt Mode  */
        (SSICR1(Spi_BaseAddrArr[Channel])).B.EOT = Spi_CfgArr[i].CFG_TxIntMode;

        /*(3) Configure the SSI clock source by writing to the SSICC register. */
        SSICC(Spi_BaseAddrArr[Channel])= Spi_CfgArr[i].CFG_ClockSource;


        /*(4) Set Bit Rate with clock rate and clock prescaler */
        Spi_SetBitRate((Spi_ChannelType)Channel, Spi_CfgArr[i].CFG_BitRate);


        /*(5) Configure Clock phase */
        SSICC(Spi_BaseAddrArr[Channel])= Spi_CfgArr[i].CFG_DataCaptureClockEdge;

        /*Configure Clock  polarity */
        SSICR0(Spi_BaseAddrArr[Channel]).B.SPO= Spi_CfgArr[i].CFG_IdleCLockState;


        /*Configure Protocol mode  */
        SSICR0(Spi_BaseAddrArr[Channel]).B.FRF= Spi_CfgArr[i].CFG_FrameFormat;


        /*Configure Data Size */
        SSICR0(Spi_BaseAddrArr[Channel]).B.DSS= Spi_CfgArr[i].CFG_DataSize;

        /*(7) Enable Spi to init configuration */
        (SSICR1(Spi_BaseAddrArr[Channel])).B.SSE = 1;

        /*change SPi Unit status to idle*/
        Spi_ChannelParam[Channel].Status = SPI_IDLE;
    }
}
/*=================================================================================
 * NAME:        Spi_enInterrupt
 * DESCRIBTION: Enable all configured interrupts for given channel
 * =================================================================================*/
void Spi_enInterrupt(Spi_ChannelType Channel)
{
    /*Enable all configured interrupts for given channel */
    uint8 i;
    for (i = 0; i < Spi_NUM_OF_ACTIVATED_UNITS; ++i)
    {
        Channel = Spi_CfgArr[i].Spi_Channel;
        SSIIM(Spi_BaseAddrArr[Channel]).B.RORIM =Spi_CfgArr[i].CFG_Interrupt_RxOverRun;
        SSIIM(Spi_BaseAddrArr[Channel]).B.RTIM = Spi_CfgArr[i].CFG_Interrupt_RxTimeout;
        SSIIM(Spi_BaseAddrArr[Channel]).B.RXIM = Spi_CfgArr[i].CFG_Interrupt_Rxc;
        SSIIM(Spi_BaseAddrArr[Channel]).B.TXIM = Spi_CfgArr[i].CFG_Interrupt_Txc;
    }

}
/*=================================================================================
 * NAME:        Spi_diInterrupt
 * DESCRIBTION: Disable all interrupts in corresponding channel
 * =================================================================================*/
void Spi_diInterrupt(Spi_ChannelType Channel)
{
    /*disable all interrupt for this Channel*/
    uint8 i;
    for (i = 0; i < Spi_NUM_OF_ACTIVATED_UNITS; ++i)
    {
        Channel = Spi_CfgArr[i].Spi_Channel;
        SSIIM(Spi_BaseAddrArr[Channel]).B.RORIM =DISABLE;
        SSIIM(Spi_BaseAddrArr[Channel]).B.RTIM = DISABLE;
        SSIIM(Spi_BaseAddrArr[Channel]).B.RXIM = DISABLE;
        SSIIM(Spi_BaseAddrArr[Channel]).B.TXIM = DISABLE;
    }
}

/*=================================================================================
 * NAME:        Spi_GetStatus
 * DESCRIBTION: get status of given channel (Idle/Busy)
 * =================================================================================*/
Spi_StatusType Spi_GetStatus(Spi_ChannelType Channel)
{
    return Spi_ChannelParam[Channel].Status;
}

/*=================================================================================
 * NAME:        Spi_WriteIB
 * DESCRIBTION: The Function shall save the pointed data to the internal buffer
 * =================================================================================*/
Std_ReturnType Spi_WriteIB( Spi_ChannelType Channel, const uint16* DataBufferPtr,uint8 DataBufferSize)
{
    Std_ReturnType ret = E_OK;
    /*save the given data to the internal buffer*/
    if(DataBufferSize>Spi_TX_BUFFER_SIZE)
    {
        DataBufferSize = Spi_TX_BUFFER_SIZE;
    }
    else
    {
        /*Do Nothing*/
    }
    uint8 i;
    if(Spi_ChannelParam[Channel].Status == SPI_IDLE)
    {
        Spi_ChannelParam[Channel].Status = SPI_BUSY;
        for(i = 0; i < DataBufferSize; i++)
        {
            Spi_ChannelParam[Channel].TxBuffer[i] = DataBufferPtr[i];
        }
        Spi_ChannelParam[Channel].Status = SPI_IDLE;
    }
    else
    {
        ret = E_NOT_OK;
    }


    return ret;
}

/* =================================================================================
 * NAME:          Spi_ReadIB
 * DESCRIBTION:   The function Spi_ReadIB provides the service
 * for reading synchronously one or more data from an IB(Rx_Buffer)
 * =================================================================================*/
Std_ReturnType Spi_ReadIB( Spi_ChannelType Channel, uint16* DataBufferPtr, uint8* DataBufferSizePtr )
{
    uint8 i;
    Std_ReturnType ret = E_OK;
    /* Check if there is any Rx Data*/
    if(Spi_ChannelParam[Channel].RxBufferIndex != 0)
    {
        /*Write into DataBufferSizePtr location the last RxBufferIndex*/
        *DataBufferSizePtr = Spi_ChannelParam[Channel].RxBufferIndex;


        /*Copy Data From RXBuffer to DataBufferPtr Location */
        for(i = 0; i < Spi_ChannelParam[Channel].RxBufferIndex ; i++)
        {
            DataBufferPtr[i] = Spi_ChannelParam[Channel].RxBuffer[i];
        }


        /*Reset RxBufferIndex */
        Spi_ChannelParam[Channel].RxBufferIndex = 0;
    }
    else
    {
        /*Return Not Ok if There is no Received Data */
        ret = E_NOT_OK;

    }
    return ret;
}

/*=================================================================================
 * NAME:        Spi_AsyncTransmit
 * DESCRIBTION: start transmission of Tx buffer asynchronously (Interrupt based)
 * =================================================================================*/
Std_ReturnType Spi_AsyncTransmit(Spi_ChannelType Channel)
{
    Std_ReturnType ret = E_OK;
    if(Spi_ChannelParam[Channel].Status == SPI_IDLE)
    {
        Spi_enInterrupt(Channel);
    }
    else
    {
        ret = E_NOT_OK;
    }
    return ret;
}
/*=================================================================================
 * NAME:        Spi_SyncTransmit
 * DESCRIBTION: start transmission of Tx buffer synchronously (Polling based)
 * =================================================================================*/
Std_ReturnType Spi_SyncTransmit(Spi_ChannelType Channel)
{
    Std_ReturnType ret = E_OK;
    uint8 i;
    if(Spi_ChannelParam[Channel].Status == SPI_IDLE)
    {
        Spi_ChannelParam[Channel].Status = SPI_BUSY;

        for (i = 0; i < Spi_ChannelParam[Channel].TxMsgSize; i++)
        {
            SSIDR(Spi_BaseAddrArr[Channel]) = Spi_ChannelParam[Channel].TxBuffer[i];

            while(SSISR(Spi_BaseAddrArr[Channel]).B.BSY == 1);

        }
        Spi_ChannelParam[Channel].Status = SPI_IDLE;
    }
    else
    {
        ret = E_NOT_OK;
    }
    return ret;
}
/*=================================================================================
 * NAME:        Spi_writeTxFifo
 * DESCRIBTION:
 * =================================================================================*/
static inline void Spi_writeTxFifo(Spi_ChannelType Channel)
{




}
/*=================================================================================
 * NAME:        Spi_SetBitRate
 * DESCRIBTION: configure bit rate
 * =================================================================================*/
static inline void Spi_SetBitRate(Spi_ChannelType Channel ,uint32 u32SSInClk)
{
    /*Calculate CPSDVSR and SCR in SSICPSR and SSICR0 Register  */

    uint32 TotalVar = u32SYS_CLOCK_Hz / u32SSInClk;

    uint8 u8CPSDVSR=2;
    uint32 u32SCR=1;
    u8CPSDVSR = TotalVar / ( u32SCR + 1 );
    while(u8CPSDVSR > 254)
    {
        u32SCR++;
        u8CPSDVSR = TotalVar / ( u32SCR + 1 );
    }

    SSICPSR(Spi_BaseAddrArr[Channel]) = u8CPSDVSR;
    SSICR0(Spi_BaseAddrArr[Channel]).B.SCR = u32SCR;


}
/*=================================================================================
 * NAME:        Spi_IntRoutine
 * DESCRIBTION: ISR for all SPI Modules Interrupts
 * =================================================================================*/
static inline void Spi_IntRoutine(Spi_ChannelType Channel)
{
    if(SSIMIS(Spi_BaseAddrArr[Channel]).B.TXMIS == STD_high)
    {


    }

    if(SSIMIS(Spi_BaseAddrArr[Channel]).B.RXMIS == STD_high)
    {


    }

    if(SSIMIS(Spi_BaseAddrArr[Channel]).B.RTMIS == STD_high)
    {


    }

    if(SSIMIS(Spi_BaseAddrArr[Channel]).B.RORMIS == STD_high)
    {


    }

}

/*=================================================================================
 * NAME:        SPI0_IntHandler
 * DESCRIBTION: SPI_0 ISR
 * =================================================================================*/
void SPI0_IntHandler(void)
{
    Spi_IntRoutine(Spi_Channel0);
}
/*=================================================================================
 * NAME:        SPI1_IntHandler
 * DESCRIBTION: SPI_1 ISR
 * =================================================================================*/
void SPI1_IntHandler(void)
{
    Spi_IntRoutine(Spi_Channel1);
}
/*=================================================================================
 * NAME:        SPI2_IntHandler
 * DESCRIBTION: SPI_2 ISR
 * =================================================================================*/
void SPI2_IntHandler(void)
{
    Spi_IntRoutine(Spi_Channel2);
}
/*=================================================================================
 * NAME:        SPI3_IntHandler
 * DESCRIBTION: SPI_3 ISR
 * =================================================================================*/
void SPI3_IntHandler(void)
{
    Spi_IntRoutine(Spi_Channel3);
}
