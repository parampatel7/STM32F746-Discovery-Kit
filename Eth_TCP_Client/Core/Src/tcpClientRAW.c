/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * TCP Client Implementation for STM32F746NG
 * Based on working tcpServerRAW.c structure
 *
 * Key fix: Bind to local IP/port BEFORE connecting to remote server
 * This prevents ERR_RTE (routing error -4)
 */

#include "tcpClientRAW.h"
#include "lwip/tcp.h"
#include <stdio.h>
#include <string.h>

/*  protocol states */
enum tcp_client_states
{
  ES_NONE = 0,
  ES_CONNECTED,
  ES_RECEIVED,
  ES_CLOSING
};

/* structure for maintaining connection infos */
struct tcp_client_struct
{
  u8_t state;             /* current connection state */
  u8_t retries;
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
};

/* Function prototypes */
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);
static err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_client_error(void *arg, err_t err);
static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb);
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void tcp_client_send(struct tcp_pcb *tpcb, struct tcp_client_struct *es);
static void tcp_client_connection_close(struct tcp_pcb *tpcb, struct tcp_client_struct *es);
static void tcp_client_handle(struct tcp_pcb *tpcb, struct tcp_client_struct *es);

/* Global variables for timer-based sending */
struct tcp_client_struct *esTx = NULL;
struct tcp_pcb *pcbTx = NULL;
int counter = 0;

/* External timer handle - make sure this matches your timer */
extern TIM_HandleTypeDef htim6;  // Changed from htim1 to htim6 based on your main.c

/* Helper function to get state name string */
static const char* get_state_name(u8_t state)
{
  switch(state)
  {
    case ES_NONE:      return "ES_NONE";
    case ES_CONNECTED: return "ES_CONNECTED";
    case ES_RECEIVED:  return "ES_RECEIVED";
    case ES_CLOSING:   return "ES_CLOSING";
    default:           return "UNKNOWN";
  }
}

/**
 * Timer callback - called periodically to send messages
 * Make sure htim6 interrupt is enabled in your code
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)  // Match your timer
    {
        char buf[100];

        /* Prepare the message to send */
        int len = sprintf(buf, "Sending TCPclient Message %d\r\n", counter);

        if (counter != 0 && esTx != NULL && pcbTx != NULL)
        {
            printf("\r\n[TIMER] Triggered (Message #%d)\r\n", counter);

            /* Allocate pbuf */
            esTx->p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);

            if (esTx->p != NULL)
            {
                printf("[TIMER] Buffer allocated (%d bytes)\r\n", len);

                /* Copy data to pbuf */
                pbuf_take(esTx->p, (char*)buf, len);
                printf("[TIMER] Data copied to buffer\r\n");

                printf("\r\n--- DATA TO SEND ---\r\n");
                printf("%s", buf);
                printf("--- END OF DATA ---\r\n\r\n");

                /* Send the data */
                tcp_client_send(pcbTx, esTx);

                /* Free the pbuf */
                pbuf_free(esTx->p);
                esTx->p = NULL;
                printf("[TIMER] Buffer freed\r\n");
            }
            else
            {
                printf("[ERROR] Failed to allocate buffer in timer\r\n");
            }
        }
    }
}

/**
 * Initialize TCP Client
 *
 * KEY FIX: Bind to local address BEFORE connecting
 * This is what your original client was missing!
 */
void tcp_client_init(void)
{
    printf("\r\n========================================\r\n");
    printf("TCP CLIENT INITIALIZATION STARTED\r\n");
    printf("========================================\r\n");

    /* 1. Create new TCP PCB */
    struct tcp_pcb *tpcb;
    tpcb = tcp_new();

    if (tpcb == NULL)
    {
        printf("[ERROR] Failed to create TCP PCB\r\n");
        printf("========================================\r\n\r\n");
        return;
    }
    printf("[INFO] TCP PCB created successfully\r\n");

    err_t err;

    /* 2. CRITICAL: Bind to local IP/port FIRST (like server does) */
    ip_addr_t myIPADDR;
    IP_ADDR4(&myIPADDR, 10, 4, 90, 100);  // Your STM32 IP

    printf("[INFO] Binding to local IP: 10.4.90.100, Port: 0 (random)\r\n");

    // Bind to local address with port 0 (let lwIP choose random port)
    err = tcp_bind(tpcb, &myIPADDR, 0);

    if (err != ERR_OK)
    {
        printf("[ERROR] Failed to bind TCP PCB. Error code: %d\r\n", err);
        memp_free(MEMP_TCP_PCB, tpcb);
        printf("[INFO] TCP PCB memory freed\r\n");
        printf("========================================\r\n\r\n");
        return;
    }
    printf("[SUCCESS] TCP PCB bound to local address!\r\n");

    /* 3. Connect to remote server */
    ip_addr_t destIPADDR;
    IP_ADDR4(&destIPADDR, 10, 4, 90, 58);  // Server IP

    printf("[INFO] Connecting to server IP: 10.4.90.58, Port: 31\r\n");
    printf("[INFO] Initiating TCP connection...\r\n");

    err = tcp_connect(tpcb, &destIPADDR, 31, tcp_client_connected);

    if (err == ERR_OK)
    {
        printf("[SUCCESS] Connection initiated successfully!\r\n");
        printf("[INFO] Waiting for server response...\r\n");
        printf("========================================\r\n\r\n");
    }
    else
    {
        printf("[ERROR] Connection initiation failed. Error code: %d\r\n", err);
        memp_free(MEMP_TCP_PCB, tpcb);
        printf("========================================\r\n\r\n");
    }
}

/**
 * Callback when connection is established with server
 */
static err_t tcp_client_connected(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    printf("\r\n========================================\r\n");
    printf("TCP CLIENT CONNECTED TO SERVER\r\n");
    printf("========================================\r\n");

    err_t ret_err;
    struct tcp_client_struct *es;

    LWIP_UNUSED_ARG(arg);

    if (err != ERR_OK)
    {
        printf("[ERROR] Connection callback with error: %d\r\n", err);
        printf("========================================\r\n\r\n");
        return err;
    }

    /* Get connection information */
    ip4_addr_t server_ip = newpcb->remote_ip;
    uint16_t server_port = newpcb->remote_port;
    char *server_ip_str = ipaddr_ntoa(&server_ip);

    ip4_addr_t local_ip = newpcb->local_ip;
    uint16_t local_port = newpcb->local_port;
    char *local_ip_str = ipaddr_ntoa(&local_ip);

    printf("[INFO] Connected to server: %s:%u\r\n", server_ip_str, server_port);
    printf("[INFO] Local address: %s:%u\r\n", local_ip_str, local_port);

    /* Allocate connection structure */
    es = (struct tcp_client_struct *)mem_malloc(sizeof(struct tcp_client_struct));
    if (es != NULL)
    {
        printf("[INFO] Connection structure allocated\r\n");

        es->state = ES_CONNECTED;
        es->pcb = newpcb;
        es->retries = 0;
        es->p = NULL;

        printf("[INFO] Connection state: %s\r\n", get_state_name(es->state));

        /* Register callbacks */
        tcp_arg(newpcb, es);
        tcp_recv(newpcb, tcp_client_recv);
        tcp_err(newpcb, tcp_client_error);
        tcp_poll(newpcb, tcp_client_poll, 0);
        tcp_sent(newpcb, tcp_client_sent);

        printf("[INFO] All callbacks registered\r\n");
        printf("========================================\r\n");
        printf("TCP CLIENT READY - Starting communication\r\n");
        printf("========================================\r\n\r\n");

        /* Handle initial connection */
        tcp_client_handle(newpcb, es);

        ret_err = ERR_OK;
    }
    else
    {
        printf("[ERROR] Failed to allocate connection structure\r\n");
        tcp_client_connection_close(newpcb, es);
        ret_err = ERR_MEM;
        printf("========================================\r\n\r\n");
    }

    return ret_err;
}

/**
 * Callback when data is received from server
 */
static err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    struct tcp_client_struct *es;
    err_t ret_err;

    LWIP_ASSERT("arg != NULL", arg != NULL);

    es = (struct tcp_client_struct *)arg;

    /* Get server info for logging */
    ip4_addr_t server_ip = tpcb->remote_ip;
    uint16_t server_port = tpcb->remote_port;
    char *server_ip_str = ipaddr_ntoa(&server_ip);

    /* Empty frame = server closed connection */
    if (p == NULL)
    {
        printf("\r\n[RECV] Empty frame from server [%s:%u]\r\n", server_ip_str, server_port);
        printf("[INFO] Server closed connection\r\n");
        printf("[INFO] State: %s -> ES_CLOSING\r\n", get_state_name(es->state));

        es->state = ES_CLOSING;
        if (es->p == NULL)
        {
            printf("[INFO] No pending data, closing connection\r\n");
            tcp_client_connection_close(tpcb, es);
        }
        ret_err = ERR_OK;
    }
    /* Error in received data */
    else if (err != ERR_OK)
    {
        printf("\r\n[ERROR] Error receiving data from [%s:%u]\r\n", server_ip_str, server_port);
        printf("[ERROR] Error code: %d\r\n", err);

        if (p != NULL)
        {
            es->p = NULL;
            pbuf_free(p);
        }
        ret_err = err;
    }
    /* Data received successfully */
    else if (es->state == ES_CONNECTED)
    {
        printf("\r\n========================================\r\n");
        printf("DATA RECEIVED FROM SERVER\r\n");
        printf("========================================\r\n");
        printf("[INFO] From server [%s:%u]\r\n", server_ip_str, server_port);
        printf("[INFO] State: %s\r\n", get_state_name(es->state));
        printf("[INFO] Data length: %u bytes\r\n", p->tot_len);

        /* Display received data */
        printf("\r\n--- DATA RECEIVED FROM SERVER ---\r\n");
        char recv_buffer[p->tot_len + 1];
        memset(recv_buffer, 0, sizeof(recv_buffer));
        pbuf_copy_partial(p, recv_buffer, p->tot_len, 0);
        recv_buffer[p->tot_len] = '\0';
        printf("%s\r\n", recv_buffer);
        printf("--- END OF RECEIVED DATA ---\r\n");

        /* Store reference to pbuf */
        es->p = p;

        /* Acknowledge received data */
        tcp_recved(tpcb, p->tot_len);
        printf("[INFO] Data acknowledged (window: %u bytes)\r\n", p->tot_len);

        /* Handle the data */
        printf("[INFO] Processing received data...\r\n");
        tcp_client_handle(tpcb, es);

        /* Free pbuf */
        pbuf_free(p);
        es->p = NULL;
        printf("[INFO] Receive buffer freed\r\n");
        printf("========================================\r\n\r\n");

        ret_err = ERR_OK;
    }
    /* Data received while closing */
    else if (es->state == ES_CLOSING)
    {
        printf("\r\n[WARN] Data received while closing from [%s:%u]\r\n",
               server_ip_str, server_port);
        tcp_recved(tpcb, p->tot_len);
        es->p = NULL;
        pbuf_free(p);
        ret_err = ERR_OK;
    }
    /* Unknown state */
    else
    {
        printf("\r\n[WARN] Data in unknown state %d from [%s:%u]\r\n",
               es->state, server_ip_str, server_port);
        tcp_recved(tpcb, p->tot_len);
        es->p = NULL;
        pbuf_free(p);
        ret_err = ERR_OK;
    }

    return ret_err;
}

/**
 * Error callback
 */
static void tcp_client_error(void *arg, err_t err)
{
    struct tcp_client_struct *es;

    printf("\r\n========================================\r\n");
    printf("TCP CONNECTION ERROR\r\n");
    printf("========================================\r\n");
    printf("[ERROR] Fatal error occurred\r\n");
    printf("[ERROR] Error code: %d\r\n", err);

    es = (struct tcp_client_struct *)arg;
    if (es != NULL)
    {
        printf("[INFO] State: %s\r\n", get_state_name(es->state));
        mem_free(es);
        printf("[INFO] Memory freed\r\n");
    }
    printf("========================================\r\n\r\n");

    /* Reset global pointers */
    esTx = NULL;
    pcbTx = NULL;
    counter = 0;
}

/**
 * Poll callback
 */
static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb)
{
    err_t ret_err;
    struct tcp_client_struct *es;

    es = (struct tcp_client_struct *)arg;

    if (es != NULL)
    {
        if (es->p != NULL)
        {
            printf("[POLL] Pending data\r\n");
            tcp_client_send(tpcb, es);
        }
        else
        {
            if (es->state == ES_CLOSING)
            {
                printf("[POLL] Closing connection\r\n");
                tcp_client_connection_close(tpcb, es);
            }
        }
        ret_err = ERR_OK;
    }
    else
    {
        printf("[POLL] NULL structure, aborting\r\n");
        tcp_abort(tpcb);
        ret_err = ERR_ABRT;
    }

    return ret_err;
}

/**
 * Sent callback - called when data is acknowledged
 */
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    struct tcp_client_struct *es;

    LWIP_UNUSED_ARG(len);

    es = (struct tcp_client_struct *)arg;
    es->retries = 0;

    printf("[ACK] Server acknowledged %u bytes\r\n", len);
    printf("[INFO] State: %s\r\n", get_state_name(es->state));

    if (es->p != NULL)
    {
        printf("[INFO] More data pending\r\n");
        tcp_client_send(tpcb, es);
    }
    else
    {
        printf("[INFO] No pending data\r\n");
        if (es->state == ES_CLOSING)
        {
            tcp_client_connection_close(tpcb, es);
        }
    }

    return ERR_OK;
}

/**
 * Send data to server
 */
static void tcp_client_send(struct tcp_pcb *tpcb, struct tcp_client_struct *es)
{
    printf("\r\n[SEND] Sending data to server\r\n");
    printf("[INFO] Available buffer: %u bytes\r\n", tcp_sndbuf(tpcb));

    struct pbuf *ptr;
    err_t wr_err = ERR_OK;

    while ((wr_err == ERR_OK) &&
           (es->p != NULL) &&
           (es->p->len <= tcp_sndbuf(tpcb)))
    {
        ptr = es->p;

        printf("[SEND] Chunk size: %u bytes\r\n", ptr->len);

        /* Enqueue data */
        wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);

        if (wr_err == ERR_OK)
        {
            u16_t plen;
            u8_t freed;

            plen = ptr->len;
            printf("[SEND] Data enqueued successfully\r\n");

            /* Move to next pbuf */
            es->p = ptr->next;

            if (es->p != NULL)
            {
                pbuf_ref(es->p);
                printf("[INFO] More data in chain\r\n");
            }
            else
            {
                printf("[INFO] Last chunk\r\n");
            }

            /* Free current pbuf */
            do
            {
                freed = pbuf_free(ptr);
            }
            while (freed == 0);

            printf("[INFO] Pbuf freed\r\n");
        }
        else if (wr_err == ERR_MEM)
        {
            printf("[WARN] Low memory, deferring\r\n");
            es->p = ptr;
        }
        else
        {
            printf("[ERROR] tcp_write failed: %d\r\n", wr_err);
        }
    }

    if (es->p == NULL)
    {
        printf("[SEND] Transmission complete\r\n\r\n");
    }
}

/**
 * Close connection
 */
static void tcp_client_connection_close(struct tcp_pcb *tpcb, struct tcp_client_struct *es)
{
    printf("\r\n========================================\r\n");
    printf("CLOSING TCP CLIENT CONNECTION\r\n");
    printf("========================================\r\n");

    /* Remove callbacks */
    printf("[INFO] Removing callbacks\r\n");
    tcp_arg(tpcb, NULL);
    tcp_sent(tpcb, NULL);
    tcp_recv(tpcb, NULL);
    tcp_err(tpcb, NULL);
    tcp_poll(tpcb, NULL, 0);

    /* Free structure */
    if (es != NULL)
    {
        printf("[INFO] Freeing structure\r\n");
        mem_free(es);
    }

    /* Close connection */
    printf("[INFO] Closing TCP PCB\r\n");
    tcp_close(tpcb);

    printf("[SUCCESS] Connection closed\r\n");
    printf("========================================\r\n\r\n");

    /* Reset globals */
    esTx = NULL;
    pcbTx = NULL;
    counter = 0;
}

/**
 * Handle data processing
 */
static void tcp_client_handle(struct tcp_pcb *tpcb, struct tcp_client_struct *es)
{
    /* Get server info */
    ip4_addr_t server_ip = tpcb->remote_ip;
    uint16_t server_port = tpcb->remote_port;
    char *server_ip_str = ipaddr_ntoa(&server_ip);

    printf("\r\n[HANDLE] Processing from %s:%u\r\n", server_ip_str, server_port);
    printf("[INFO] State: %s\r\n", get_state_name(es->state));

    /* Store globally for timer use */
    esTx = es;
    pcbTx = tpcb;

    counter++;
    printf("[INFO] Message counter: %d\r\n", counter);
    printf("[INFO] Timer will send periodic messages\r\n\r\n");
}
