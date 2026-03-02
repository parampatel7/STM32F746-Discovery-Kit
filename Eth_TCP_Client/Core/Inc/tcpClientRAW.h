/**
  ******************************************************************************
  * @file           : tcpClientRAW.h
  * @brief          : Header for tcpClientRAW.c file (LwIP RAW API TCP Client)
  ******************************************************************************
  * This file contains the TCP Client implementation using LwIP RAW API
  * Based on the same structure as tcpServerRAW.c for learning purposes
  ******************************************************************************
  */

#ifndef __TCP_CLIENT_RAW_H
#define __TCP_CLIENT_RAW_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lwip/tcp.h"
#include "lwip/err.h"

/* Server Configuration - MODIFY THESE VALUES --------------------------------*/
#define TCP_SERVER_IP_ADDR0   10
#define TCP_SERVER_IP_ADDR1   4
#define TCP_SERVER_IP_ADDR2   90
#define TCP_SERVER_IP_ADDR3   58      /* Server IP: 10.4.90.100 */
#define TCP_SERVER_PORT       8080       /* Server Port */

/* Client Configuration ------------------------------------------------------*/
#define MAX_BUFFER_SIZE       512
#define RECONNECT_DELAY_MS    5000    /* Reconnection delay in milliseconds */

/* Exported functions --------------------------------------------------------*/
void tcp_client_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TCP_CLIENT_RAW_H */
