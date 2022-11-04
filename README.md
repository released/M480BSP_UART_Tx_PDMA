# M480BSP_UART_Tx_PDMA
 M480BSP_UART_Tx_PDMA


update @ 2022/11/04

1. initial UART1 TX (PB.3) w/ PDMA function 

- with 2 define ENALBE_PDMA_IRQ ,  ENALBE_PDMA_POLLING , for PDMA transfer function

- enable define : ENALBE_PDMA_SCATTER for transfer , without enable define will use PDMA basic mode

2. TX Buffer length set as 4096 , send TX buffer per 500 ms , 

- set buffer index 0 , 1 as 0x5A , for indicator

- set buffer last index n-1 , n-2 , as 0x A5 , for indicator

- set buffer last index n-3 , as a counter (increase per 500ms) , for indicator

3. below is LA capture  , 

below is buffer transmit timing , each packet spend 354 ms , and send packet per 500 ms

![image](https://github.com/released/M480BSP_UART_Tx_PDMA/blob/main/buffer_timing.jpg)	


below is packet head , check the indicator (STEP #2)

![image](https://github.com/released/M480BSP_UART_Tx_PDMA/blob/main/head.jpg)	


below is packet tail , check the indicator (STEP #2)

![image](https://github.com/released/M480BSP_UART_Tx_PDMA/blob/main/tail.jpg)	


