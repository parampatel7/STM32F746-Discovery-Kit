/*
 * udp_server.c
 *
 *  Created on: Feb 5, 2026
 *      Author: invadl0640
 *  Modified: Added comprehensive logging and improved data handling
 */


#include "lwip/pbuf.h"  // Provides structures and functions for packet buffers.
#include "lwip/udp.h"   // Includes UDP-specific functionality.
#include "lwip/tcp.h"   // Used for TCP/IP stack functionality (though not required for UDP operations).

#include "stdio.h"      // Standard input/output library.
#include "udp_server.h"  // Custom header file for this server (likely contains function prototypes).
#include <string.h>
#include <stdio.h>

// Callback function declaration.
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

void udpServer_init(void)
{
    printf("\r\n========================================\r\n");
    printf("UDP SERVER INITIALIZATION STARTED\r\n");
    printf("========================================\r\n");

    // UDP Control Block structure
    struct udp_pcb *upcb;
    err_t err;

    /* 1. Create a new UDP control block */
    upcb = udp_new();

    if (upcb == NULL)
    {
        printf("[ERROR] Failed to create UDP control block\r\n");
        printf("========================================\r\n\r\n");
        return;
    }
    printf("[INFO] UDP control block created successfully\r\n");

    /* 2. Bind the upcb to the local port */
    ip_addr_t myIPADDR; // Define the IP address for the server.
    IP_ADDR4(&myIPADDR, 10, 4, 90, 100); // Set IP to 10.4.90.100

    printf("[INFO] Binding to IP: 10.4.90.100, Port: 1100\r\n");

    // Bind the UDP control block to the IP and port 1100.
    err = udp_bind(upcb, &myIPADDR, 1100); //Make sure to enter same port in packetsender

    /* 3. Set a receive callback for the upcb */
    if (err == ERR_OK)
    {
        printf("[SUCCESS] UDP server bound successfully!\r\n");

        // Register the callback function for receiving data.
        udp_recv(upcb, udp_receive_callback, NULL);

        printf("[INFO] UDP receive callback registered\r\n");
        printf("========================================\r\n");
        printf("UDP SERVER READY - Waiting for packets...\r\n");
        printf("========================================\r\n\r\n");
    }
    else
    {
        // If binding fails, remove the UDP control block.
        printf("[ERROR] Binding failed! Error code: %d\r\n", err);
        udp_remove(upcb);
        printf("[INFO] UDP control block removed\r\n");
        printf("========================================\r\n\r\n");
    }
}

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    printf("\r\n========================================\r\n");
    printf("UDP PACKET RECEIVED\r\n");
    printf("========================================\r\n");

    struct pbuf *txBuf; // Packet buffer for transmitting data.

    /* Get the IP of the Client */
    char *remoteIP = ipaddr_ntoa(addr); // Convert client IP address to a readable string.

    printf("[INFO] From Client IP: %s\r\n", remoteIP);
    printf("[INFO] From Client Port: %u\r\n", port);
    printf("[INFO] Packet Length: %d bytes\r\n", p->len);
    printf("[INFO] Total Length: %d bytes\r\n", p->tot_len);

    /* Display received data */
    printf("\r\n--- DATA RECEIVED FROM CLIENT ---\r\n");

    // Create a null-terminated string for display
    char recv_buffer[p->tot_len + 1];
    memset(recv_buffer, 0, sizeof(recv_buffer));

    // Copy data from pbuf to buffer
    pbuf_copy_partial(p, recv_buffer, p->tot_len, 0);
    recv_buffer[p->tot_len] = '\0';

    printf("%s\r\n", recv_buffer);
    printf("--- END OF RECEIVED DATA ---\r\n\r\n");

    /* Prepare response */
    char response[200];
    int len;

    // Determine the response based on the received payload
    if (strncmp(recv_buffer, "UDP00", 5) == 0)
    {
        printf("[INFO] Command recognized: UDP00\r\n");
        printf("[INFO] Response: Hello World\r\n");

        // Echo back the received data plus response
        snprintf(response, sizeof(response), "%s + Hello World", recv_buffer);
        len = strlen(response);
    }
    else if (strncmp(recv_buffer, "UDP01", 5) == 0)
    {
        printf("[INFO] Command recognized: UDP01\r\n");
        printf("[INFO] Response: Param Patel\r\n");

        // Echo back the received data plus response
        snprintf(response, sizeof(response), "%s + Param Patel", recv_buffer);
        len = strlen(response);
    }
    else
    {
        printf("[WARN] Unknown command received\r\n");
        printf("[INFO] Response: ERROR\r\n");

        // Echo back the received data plus error
        snprintf(response, sizeof(response), "%s + ERROR", recv_buffer);
        len = strlen(response);
    }

    printf("\r\n--- DATA TO SEND TO CLIENT ---\r\n");
    printf("%s\r\n", response);
    printf("--- END OF DATA TO SEND ---\r\n");
    printf("[INFO] Response length: %d bytes\r\n\r\n", len);

    /* Allocate a pbuf for the outgoing message from RAM */
    txBuf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);

    if (txBuf == NULL)
    {
        printf("[ERROR] Failed to allocate transmission buffer\r\n");
        pbuf_free(p);
        printf("========================================\r\n\r\n");
        return;
    }
    printf("[INFO] Transmission buffer allocated (%d bytes)\r\n", txBuf->len);

    /* Copy the response message into the buffer */
    pbuf_take(txBuf, response, len);
    printf("[INFO] Response data copied to buffer\r\n");

    /* Connect to the remote client */
    err_t err = udp_connect(upcb, addr, port);
    if (err == ERR_OK)
    {
        printf("[INFO] Connected to client %s:%u\r\n", remoteIP, port);
    }
    else
    {
        printf("[ERROR] Failed to connect to client. Error: %d\r\n", err);
    }

    /* Send a reply to the client */
    printf("[INFO] Sending response to client...\r\n");
    err = udp_send(upcb, txBuf);

    if (err == ERR_OK)
    {
        printf("[SUCCESS] Response sent successfully!\r\n");
    }
    else
    {
        printf("[ERROR] Failed to send response. Error code: %d\r\n", err);
    }

    /* Disconnect the UDP connection to allow new clients */
    udp_disconnect(upcb);
    printf("[INFO] Disconnected from client\r\n");

    /* Free the transmit buffer */
    pbuf_free(txBuf);
    printf("[INFO] Transmission buffer freed\r\n");

    /* Free the receive buffer */
    pbuf_free(p);
    printf("[INFO] Receive buffer freed\r\n");

    printf("========================================\r\n");
    printf("PACKET PROCESSING COMPLETE\r\n");
    printf("========================================\r\n\r\n");
}
