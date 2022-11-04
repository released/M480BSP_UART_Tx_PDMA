/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "project_config.h"


/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/
volatile uint32_t BitFlag = 0;
volatile uint32_t counter_tick = 0;
volatile uint32_t counter_systick = 0;

#if defined (ENABLE_TICK_EVENT)
typedef void (*sys_pvTimeFunPtr)(void);   /* function pointer */
typedef struct timeEvent_t
{
    uint8_t             active;
    unsigned long       initTick;
    unsigned long       curTick;
    sys_pvTimeFunPtr    funPtr;
} TimeEvent_T;

#define TICKEVENTCOUNT                                 (8)                   
volatile  TimeEvent_T tTimerEvent[TICKEVENTCOUNT];
volatile uint8_t _sys_uTimerEventCount = 0;             /* Speed up interrupt reponse time if no callback function */
#endif


#define UART_PORT  						(UART1)
#define UART_TX_DMA_CH 				    (5)
#define UART_TX_PDMA_OPENED_CH   		((1 << UART_TX_DMA_CH))

#define TXBUFSIZE                       (4096)
uint8_t tBuffer[TXBUFSIZE] = {0};

uint8_t u8TxIdx = 0;
uint32_t g_u32DMAConfig = 0;

DSCT_T DMA_DESC[2];


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

uint32_t get_systick(void)
{
	return (counter_systick);
}

void set_systick(uint32_t t)
{
	counter_systick = t;
}

void systick_counter(void)
{
	counter_systick++;
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    #if 1
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			set_flag(flag_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		set_flag(flag_error , DISABLE);
	}
    #else
    if (memcmp(src, des, nBytes))
    {
        printf("\nMismatch!! - %d\n", nBytes);
        for (i = 0; i < nBytes; i++)
            printf("0x%02x    0x%02x\n", src[i], des[i]);
        return -1;
    }
    #endif

}

void reset_buffer(void *dest, unsigned int val, unsigned int size)
{
    uint8_t *pu8Dest;
//    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;

	#if 1
	while (size-- > 0)
		*pu8Dest++ = val;
	#else
	memset(pu8Dest, val, size * (sizeof(pu8Dest[0]) ));
	#endif
	
}

void copy_buffer(void *dest, void *src, unsigned int size)
{
    uint8_t *pu8Src, *pu8Dest;
    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;


	#if 0
	  while (size--)
	    *pu8Dest++ = *pu8Src++;
	#else
    for (i = 0; i < size; i++)
        pu8Dest[i] = pu8Src[i];
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}



#if defined (ENABLE_TICK_EVENT)
void TickCallback_processB(void)
{
    printf("%s test \r\n" , __FUNCTION__);
}

void TickCallback_processA(void)
{
    printf("%s test \r\n" , __FUNCTION__);
}

void TickClearTickEvent(uint8_t u8TimeEventID)
{
    if (u8TimeEventID > TICKEVENTCOUNT)
        return;

    if (tTimerEvent[u8TimeEventID].active == TRUE)
    {
        tTimerEvent[u8TimeEventID].active = FALSE;
        _sys_uTimerEventCount--;
    }
}

signed char TickSetTickEvent(unsigned long uTimeTick, void *pvFun)
{
    int  i;
    int u8TimeEventID = 0;

    for (i = 0; i < TICKEVENTCOUNT; i++)
    {
        if (tTimerEvent[i].active == FALSE)
        {
            tTimerEvent[i].active = TRUE;
            tTimerEvent[i].initTick = uTimeTick;
            tTimerEvent[i].curTick = uTimeTick;
            tTimerEvent[i].funPtr = (sys_pvTimeFunPtr)pvFun;
            u8TimeEventID = i;
            _sys_uTimerEventCount += 1;
            break;
        }
    }

    if (i == TICKEVENTCOUNT)
    {
        return -1;    /* -1 means invalid channel */
    }
    else
    {
        return u8TimeEventID;    /* Event ID start from 0*/
    }
}

void TickCheckTickEvent(void)
{
    uint8_t i = 0;

    if (_sys_uTimerEventCount)
    {
        for (i = 0; i < TICKEVENTCOUNT; i++)
        {
            if (tTimerEvent[i].active)
            {
                tTimerEvent[i].curTick--;

                if (tTimerEvent[i].curTick == 0)
                {
                    (*tTimerEvent[i].funPtr)();
                    tTimerEvent[i].curTick = tTimerEvent[i].initTick;
                }
            }
        }
    }
}

void TickInitTickEvent(void)
{
    uint8_t i = 0;

    _sys_uTimerEventCount = 0;

    /* Remove all callback function */
    for (i = 0; i < TICKEVENTCOUNT; i++)
        TickClearTickEvent(i);

    _sys_uTimerEventCount = 0;
}
#endif 

void SysTick_Handler(void)
{

    systick_counter();

    if (get_systick() >= 0xFFFFFFFF)
    {
        set_systick(0);      
    }

    // if ((get_systick() % 1000) == 0)
    // {
       
    // }

    #if defined (ENABLE_TICK_EVENT)
    TickCheckTickEvent();
    #endif    
}

void SysTick_delay(unsigned long delay)
{  
    
    uint32_t tickstart = get_systick(); 
    uint32_t wait = delay; 

    while((get_systick() - tickstart) < wait) 
    { 
    } 

}

void SysTick_enable(int ticks_per_second)
{
    set_systick(0);
    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while (1);
    }

    #if defined (ENABLE_TICK_EVENT)
    TickInitTickEvent();
    #endif
}

void TMR1_IRQHandler(void)
{	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            set_flag(flag_timer_period_1000ms ,ENABLE);
		}

		if ((get_tick() % 500) == 0)
		{
            set_flag(flag_timer_period_500ms ,ENABLE);
		}

		if ((get_tick() % 50) == 0)
		{

		}	
    }
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void PDMA_SetTxSGTable(uint8_t id)
{

    g_u32DMAConfig = \
                            ((TXBUFSIZE / 2 - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is TXBUFSIZE/2 */ \
                            PDMA_WIDTH_8 |    /* Transfer width is 8 bits */ \
                            PDMA_SAR_INC |    /* Source increment size is  fixed(no increment) */ \
                            PDMA_DAR_FIX |    /* Destination increment size is 8 bits */ \
                            PDMA_REQ_SINGLE | /* Transfer type is single transfer type */ \
                            PDMA_BURST_1 ;    /* Burst size is 1. No effect in single transfer type */ \
    
    if (id == 0)
    {
        DMA_DESC[0].CTL = g_u32DMAConfig | PDMA_OP_SCATTER;
        /* Configure source address */
        DMA_DESC[0].SA = (uint32_t)&tBuffer[0];/*buffer 1 */
        /* Configure destination address */
        DMA_DESC[0].DA = (uint32_t)UART1_BASE;
        /* Configure next descriptor table address */
        DMA_DESC[0].NEXT = (uint32_t)&DMA_DESC[1] - (PDMA->SCATBA); /* next operation table is table 2 */
    }
    else
    {
        DMA_DESC[1].CTL = g_u32DMAConfig | PDMA_OP_BASIC; 
        // DMA_DESC[1].CTL = g_u32DMAConfig | PDMA_OP_SCATTER;               
        /* Configure source address */
        DMA_DESC[1].SA = (uint32_t)&tBuffer[TXBUFSIZE / 2] ; /* buffer 2 */
        /* Configure destination address */
        DMA_DESC[1].DA = (uint32_t)UART1_BASE;
        /* Configure next descriptor table address */
        // DMA_DESC[1].NEXT = (uint32_t)&DMA_DESC[0] - (PDMA->SCATBA); /* next operation table is table 1 */
        DMA_DESC[1].NEXT = 0; /* next operation table is table 1 */
    }
        
}

void PDMA_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA);
    #if defined (ENALBE_PDMA_SCATTER)    
    static uint8_t cnt = 0;
    #endif
	
    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
		#if 1
        PDMA_CLR_ABORT_FLAG(PDMA, PDMA_GET_ABORT_STS(PDMA));
		#else
        if (PDMA_GET_ABORT_STS(PDMA) & UART_TX_PDMA_OPENED_CH)
        {
        }
        PDMA_CLR_ABORT_FLAG(PDMA, UART_TX_PDMA_OPENED_CH);
        if (PDMA_GET_ABORT_STS(PDMA) & (1 << QSPI_MASTER_RX_DMA_CH))
        {
        }
        PDMA_CLR_ABORT_FLAG(PDMA, (1 << QSPI_MASTER_RX_DMA_CH));
		#endif
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if((PDMA_GET_TD_STS(PDMA) & UART_TX_PDMA_OPENED_CH ) == UART_TX_PDMA_OPENED_CH  )
        {
            /* Clear PDMA transfer done interrupt flag */
            PDMA_CLR_TD_FLAG(PDMA, UART_TX_PDMA_OPENED_CH );
            
			//insert process
            #if defined (ENALBE_PDMA_SCATTER)
            if (cnt++ == 1)
            { 
                cnt = 0;
                UART_PDMA_DISABLE(UART_PORT,UART_INTEN_TXPDMAEN_Msk);                
                set_flag(flag_uart_tx_finish , ENABLE);
            }
            
            #else
            UART_PDMA_DISABLE(UART_PORT,UART_INTEN_TXPDMAEN_Msk);
            set_flag(flag_uart_tx_finish , ENABLE);
            #endif
        }  
    }
    else if (status & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk))     /* Check the DMA time-out interrupt flag */
    {
        // PDMA_CLR_TMOUT_FLAG(PDMA,UART_TX_DMA_CH);
    }
    else
    {
        // printf("status : 0x%4X\r\n" ,status);
    }

}

void UART_TX_PDMA(uint8_t* Datain , uint16_t len)
{
    #if defined (ENALBE_PDMA_POLLING)      
	uint32_t u32RegValue = 0;
	uint32_t u32Abort = 0;	
    static uint8_t cnt = 0;    
    #endif

    set_flag(flag_uart_tx_finish , DISABLE);
	
    #if !defined (ENALBE_PDMA_SCATTER)
	//TX
    PDMA_SetTransferCnt(PDMA,UART_TX_DMA_CH, PDMA_WIDTH_8, len);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA,UART_TX_DMA_CH, (uint32_t) (&Datain[0]), PDMA_SAR_INC, (uint32_t) &UART_PORT->DAT , PDMA_DAR_FIX);//UART5_BASE
    /* Set request source; set basic mode. */

    PDMA_SetTransferMode(PDMA,UART_TX_DMA_CH, PDMA_UART1_TX, FALSE, 0);
    #else
           
    PDMA_SetTxSGTable(0);
    PDMA_SetTxSGTable(1);

    PDMA_SetTransferMode(PDMA,UART_TX_DMA_CH, PDMA_UART1_TX, TRUE, (uint32_t)&DMA_DESC[0]);  
    #endif

	UART_PDMA_ENABLE(UART_PORT,UART_INTEN_TXPDMAEN_Msk);

    #if defined (ENALBE_PDMA_POLLING)   
    while(1)
    {
        /* Get interrupt status */
        u32RegValue = PDMA_GET_INT_STATUS(PDMA);
        /* Check the DMA transfer done interrupt flag */
        if(u32RegValue & PDMA_INTSTS_TDIF_Msk)
        {
            /* Check the PDMA transfer done interrupt flags */
            if((PDMA_GET_TD_STS(PDMA) & UART_TX_PDMA_OPENED_CH) == UART_TX_PDMA_OPENED_CH)
            {
                /* Clear the DMA transfer done flags */
                PDMA_CLR_TD_FLAG(PDMA , UART_TX_PDMA_OPENED_CH);

                #if defined (ENALBE_PDMA_SCATTER)
                if (cnt++ == 1)
                {      
                    cnt = 0;
                    UART_PDMA_DISABLE(UART_PORT,UART_INTEN_TXPDMAEN_Msk);                
                    set_flag(flag_uart_tx_finish , ENABLE);
                    break;
                }
                
                #else
                UART_PDMA_DISABLE(UART_PORT,UART_INTEN_TXPDMAEN_Msk);
                set_flag(flag_uart_tx_finish , ENABLE);
                break;
                #endif                
            }
        }
        /* Check the DMA transfer abort interrupt flag */
        if(u32RegValue & PDMA_INTSTS_ABTIF_Msk)
        {
            /* Get the target abort flag */
            u32Abort = PDMA_GET_ABORT_STS(PDMA);
            /* Clear the target abort flag */
            PDMA_CLR_ABORT_FLAG(PDMA,u32Abort);
            break;
        }		
    }
    #endif

    while(!is_flag_set(flag_uart_tx_finish));
    while(!UART_IS_TX_EMPTY(UART_PORT));

}

void UART_PDMA_Init(void)
{
    UART_Open(UART_PORT, 115200);

    PDMA_Open(PDMA, UART_TX_PDMA_OPENED_CH);

    #if !defined (ENALBE_PDMA_SCATTER)
    PDMA_SetBurstType(PDMA,UART_TX_DMA_CH, PDMA_REQ_SINGLE, PDMA_BURST_128);
    /* Disable table interrupt */
    PDMA->DSCT[UART_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
    #endif

	UART_PDMA_DISABLE(UART_PORT,UART_INTEN_TXPDMAEN_Msk);        

    #if defined (ENALBE_PDMA_IRQ)
    PDMA_EnableInt(PDMA, UART_TX_DMA_CH, PDMA_INT_TRANS_DONE);

    NVIC_SetPriority(PDMA_IRQn,0);
    NVIC_ClearPendingIRQ(PDMA_IRQn);
    NVIC_EnableIRQ(PDMA_IRQn);
    #endif

}

void create_buffer(void)
{
    uint16_t i = 0;  
    for (i = 0 ; i < (TXBUFSIZE/2) ; i++)
    {
        tBuffer[i*2+0] = HIBYTE(i);
        tBuffer[i*2+1] = LOBYTE(i);
    }    
}

void loop(void)
{
	static uint32_t LOG1 = 0;
	// static uint32_t LOG2 = 0;
	static uint32_t cnt = 0;  


    if ((get_systick() % 1000) == 0)
    {
        // printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    }

    if (is_flag_set(flag_timer_period_500ms))
    {
        set_flag(flag_timer_period_500ms ,DISABLE);

        tBuffer[0] = 0x5A;
        tBuffer[1] = 0x5A;      
        tBuffer[TXBUFSIZE-3] = cnt;         
        tBuffer[TXBUFSIZE-2] = 0xA5;        
        tBuffer[TXBUFSIZE-1] = 0xA5;
        
		UART_TX_PDMA(tBuffer , TXBUFSIZE);
        cnt++;
    }

    if (is_flag_set(flag_timer_period_1000ms))
    {
        set_flag(flag_timer_period_1000ms ,DISABLE);

        printf("%s(timer) : %4d\r\n",__FUNCTION__,LOG1++);
        PH0 ^= 1;
    }
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case '1':
                // set_flag(flag_trigger_uart_tx ,ENABLE);
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
				NVIC_SystemReset();		
				break;
		}
	}
}

void UART0_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART0_IRQn);

	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	

//    printf("Product ID 0x%8X\n", SYS->PDID);
	
	#endif
}

void Custom_Init(void)
{
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH0MFP_Msk)) | (SYS_GPH_MFPL_PH0MFP_GPIO);
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH1MFP_Msk)) | (SYS_GPH_MFPL_PH1MFP_GPIO);
	SYS->GPH_MFPL = (SYS->GPH_MFPL & ~(SYS_GPH_MFPL_PH2MFP_Msk)) | (SYS_GPH_MFPL_PH2MFP_GPIO);

	//EVM LED
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    CLK_EnableModuleClock(UART1_MODULE);
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD);

    CLK_EnableModuleClock(PDMA_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	Custom_Init();
	UART0_Init();
	TIMER1_Init();

    SysTick_enable(1000);
    #if defined (ENABLE_TICK_EVENT)
    TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    #endif

    UART_PDMA_Init();
    create_buffer();

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
