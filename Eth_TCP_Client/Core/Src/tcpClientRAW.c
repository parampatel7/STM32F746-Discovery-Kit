/**
 ******************************************************************************
 * @file    tcpClientRAW.c
 * @brief   TCP Client implementation using lwIP RAW API (No RTOS)
 * @board   STM32F746G-DISCO (STM32F746NG)
 * @stack   lwIP 2.x, Ethernet via LAN8742 PHY (RMII)
 ******************************************************************************
 *
 * OVERVIEW
 * --------
 * This file implements a TCP client that connects to a remote server and
 * exchanges data bidirectionally. It uses the lwIP RAW (callback-based)
 * API — the most efficient approach for a no-RTOS embedded system.
 *
 * CONNECTION FLOW
 * ---------------
 *  [Power ON]
 *      │
 *      ▼
 *  tcp_client_init()          ← Called from main.c after MX_LWIP_Init()
 *      │
 *      ├── Link UP? ──NO──► Schedule 2s retry via sys_timeout()
 *      │                      (tcp_client_retry_timer → tcp_client_init)
 *      │
 *      └── Link UP? ──YES─► tcp_new() → tcp_connect()
 *                                │
 *                                ▼ (SYN-ACK received from server)
 *                       tcp_client_connected()
 *                                │
 *                                ├── Register recv / sent / poll / err callbacks
 *                                └── Send "Hello from STM32" welcome message
 *
 * DATA FLOW (Bidirectional, No Loop)
 * -----------------------------------
 *  Server types a message (Eth_TCP_Server.py terminal)
 *      │
 *      ▼
 *  tcp_client_recv()          ← lwIP calls this when data arrives
 *      │
 *      └── tcp_client_handle()
 *              ├── Prints the server message on UART (Minicom)
 *              └── Echoes back: "[STM32 ECHO] <msg>[STM32] Received & processed OK"
 *                      │
 *                      ▼
 *              tcp_client_send()  ← enqueues data via tcp_write() + tcp_output()
 *
 *  Server receives the echo and ONLY prints it — it NEVER auto-replies.
 *  This prevents the infinite ping-pong loop.
 *
 * KEY DESIGN DECISIONS
 * ---------------------
 * 1. Link-Up guard: tcp_connect() must NOT be called when the Ethernet
 *    PHY link is down. Unlike tcp_bind()/tcp_listen() (which are internal
 *    only), tcp_connect() immediately transmits a SYN packet. If the link
 *    is not up, lwIP returns ERR_RTE (-4) — "no route to host".
 *    Solution: check netif_is_link_up() and schedule a retry via sys_timeout().
 *
 * 2. tcp_recved() placement: Must be called in the RECEIVE path (recv callback)
 *    to reopen the TCP receive window. Calling it in the SEND path is wrong
 *    and can cause receive buffer anomalies.
 *
 * 3. No lwIP calls from ISR: pbuf_alloc(), tcp_write() etc. are NOT
 *    interrupt-safe. Never call them from HAL_TIM_PeriodElapsedCallback()
 *    or any other ISR. Use a flag and handle in the main loop instead.
 *
 * 4. tcp_poll interval must be >= 1: Passing 0 disables the poll callback.
 *    The interval is in units of 500 ms (1 = 500 ms, 2 = 1000 ms, etc.).
 *
 * CONFIGURATION
 * -------------
 *  Edit tcpClientRAW.h to change:
 *    TCP_SERVER_IP_ADDR0..3  — Server IP address
 *    TCP_SERVER_PORT         — Server TCP port
 *    MAX_BUFFER_SIZE         — Maximum message buffer size
 *
 ******************************************************************************
 */

#include "tcpClientRAW.h"
#include "lwip/tcp.h"      /* Core lwIP TCP API: tcp_new, tcp_connect, tcp_write, etc. */
#include "lwip/err.h"      /* lwIP error codes: ERR_OK, ERR_RTE, ERR_MEM, etc. */
#include "lwip/sys.h"      /* lwIP system functions: sys_timeout() */
#include "lwip/netif.h"    /* Network interface API: netif_is_link_up() */
#include "lwip/timeouts.h" /* lwIP timer infrastructure: sys_check_timeouts() */
#include "string.h"
#include <stdio.h>

/* ---------------------------------------------------------------------------
 * External reference to the global network interface.
 * 'gnetif' is created and managed by MX_LWIP_Init() in LWIP/App/lwip.c.
 * We use it here to check the Ethernet link state before connecting.
 * --------------------------------------------------------------------------- */
extern struct netif gnetif;

/* ---------------------------------------------------------------------------
 * TCP Client connection states
 * These track what the client is currently doing, allowing each callback
 * to behave correctly for the current phase of the connection.
 * --------------------------------------------------------------------------- */
enum tcp_client_states
{
  ES_NONE = 0,    /* Initial state — no connection has been attempted yet */
  ES_CONNECTING,  /* tcp_connect() called, waiting for server's SYN-ACK */
  ES_CONNECTED,   /* Three-way handshake complete, ready to send/receive */
  ES_RECEIVED,    /* Data was received from the server and is being processed */
  ES_CLOSING      /* Connection teardown in progress */
};

/* ---------------------------------------------------------------------------
 * Connection context structure
 * One instance of this structure exists for the single active connection.
 * It is passed as 'arg' to every lwIP callback via tcp_arg().
 * --------------------------------------------------------------------------- */
struct tcp_client_struct
{
  u8_t state;          /* Current connection state (see tcp_client_states above) */
  u8_t retries;        /* Retry counter — incremented in sent callback, reset on ACK */
  struct tcp_pcb *pcb; /* Pointer to the lwIP Protocol Control Block for this connection */
  struct pbuf *p;      /* Pointer to the pbuf currently queued for transmission */
};

/* ---------------------------------------------------------------------------
 * Global pointer to the single active client context.
 * NULL when not connected.
 * --------------------------------------------------------------------------- */
static struct tcp_client_struct *es_client = NULL;

/* ---------------------------------------------------------------------------
 * Private function prototypes
 * All callbacks and internal helpers are declared static so they are not
 * visible outside this translation unit.
 * --------------------------------------------------------------------------- */
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);
static err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void  tcp_client_error(void *arg, err_t err);
static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb);
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void  tcp_client_send(struct tcp_pcb *tpcb, struct tcp_client_struct *es);
static void  tcp_client_connection_close(struct tcp_pcb *tpcb, struct tcp_client_struct *es);
static void  tcp_client_handle(struct tcp_pcb *tpcb, struct tcp_client_struct *es);

/* ---------------------------------------------------------------------------
 * Helper: returns a human-readable string for a given state value.
 * Used only in printf() calls for debugging.
 * --------------------------------------------------------------------------- */
static const char* get_client_state_name(u8_t state)
{
  switch (state)
  {
    case ES_NONE:       return "ES_NONE";
    case ES_CONNECTING: return "ES_CONNECTING";
    case ES_CONNECTED:  return "ES_CONNECTED";
    case ES_RECEIVED:   return "ES_RECEIVED";
    case ES_CLOSING:    return "ES_CLOSING";
    default:            return "UNKNOWN";
  }
}

/* ===========================================================================
 * RETRY TIMER CALLBACK
 * ===========================================================================
 *
 * @brief  lwIP one-shot timer callback that retries tcp_client_init().
 *
 * This is scheduled by tcp_client_init() via sys_timeout() when the
 * Ethernet link is not yet UP. sys_check_timeouts() (called in the main
 * while loop via MX_LWIP_Process()) fires this callback after the delay
 * expires. It simply calls tcp_client_init() again, which will either
 * connect (if the link is now UP) or schedule another retry.
 *
 * No blocking delays are used — this is entirely cooperative/non-blocking.
 * =========================================================================== */
static void tcp_client_retry_timer(void *arg)
{
  LWIP_UNUSED_ARG(arg);
  printf("[INFO] Retrying TCP connection (checking link state)...\r\n");
  tcp_client_init();
}

/* ===========================================================================
 * INITIALIZATION — PUBLIC ENTRY POINT
 * ===========================================================================
 *
 * @brief  Initialize and start the TCP client connection to the server.
 *
 * Call this function once from main.c after MX_LWIP_Init() and
 * HAL_TIM_Base_Start_IT(). It is safe to call from the main loop.
 *
 * CRITICAL — Link-Up Guard:
 *   Unlike tcp_bind()/tcp_listen() (server-side, which only set internal
 *   lwIP state and do not transmit), tcp_connect() IMMEDIATELY transmits
 *   a TCP SYN packet. If the Ethernet PHY (LAN8742) has not yet finished
 *   auto-negotiating the link speed/duplex, lwIP has no active route and
 *   returns ERR_RTE (-4). This is the most common failure in TCP client
 *   implementations on STM32 with lwIP no-RTOS.
 *
 *   Solution: check netif_is_link_up() first. If the link is down, schedule
 *   a 2-second non-blocking retry using sys_timeout().
 * =========================================================================== */
void tcp_client_init(void)
{
  printf("\n========================================\r\n");
  printf("TCP CLIENT INITIALIZATION STARTED\r\n");
  printf("========================================\r\n");

  /* ---- Step 0: Ethernet link-up guard ------------------------------------ */
  if (!netif_is_link_up(&gnetif))
  {
    /*
     * The LAN8742 PHY is still negotiating the link speed (10/100 Mbps,
     * half/full duplex). tcp_connect() would fail with ERR_RTE (-4) here.
     * Schedule a retry after 2 seconds — no blocking delay needed.
     */
    printf("[WARN] Ethernet link not UP yet. Retry in 2s...\r\n");
    printf("========================================\r\n\n");
    sys_timeout(2000, tcp_client_retry_timer, NULL);
    return;
  }

  printf("[INFO] Ethernet link is UP. Proceeding.\r\n");

  /* ---- Step 1: Create a new TCP Protocol Control Block (PCB) ------------- */
  /*
   * tcp_new() allocates a PCB from the lwIP memory pool. The PCB holds all
   * TCP state: sequence numbers, timers, window sizes, etc.
   */
  struct tcp_pcb *tpcb = tcp_new();
  if (tpcb == NULL)
  {
    printf("[ERROR] Failed to allocate TCP PCB (out of lwIP memory)\r\n");
    return;
  }
  printf("[INFO] TCP PCB created\r\n");

  /* ---- Step 2: Allocate and initialise the application context ----------- */
  /*
   * es_client holds our application-level state for this connection.
   * It is passed to every lwIP callback via tcp_arg() so all callbacks
   * share the same context.
   */
  es_client = (struct tcp_client_struct *)mem_malloc(sizeof(struct tcp_client_struct));
  if (es_client == NULL)
  {
    printf("[ERROR] Failed to allocate client context (out of lwIP heap)\r\n");
    memp_free(MEMP_TCP_PCB, tpcb); /* Return the PCB to the pool */
    return;
  }

  es_client->state   = ES_NONE;
  es_client->pcb     = tpcb;
  es_client->retries = 0;
  es_client->p       = NULL;

  printf("[INFO] Client context allocated\r\n");

  /* ---- Step 3: Build the server IP address ------------------------------- */
  ip_addr_t server_ip;
  IP_ADDR4(&server_ip,
           TCP_SERVER_IP_ADDR0, TCP_SERVER_IP_ADDR1,
           TCP_SERVER_IP_ADDR2, TCP_SERVER_IP_ADDR3);

  printf("[INFO] Connecting to %d.%d.%d.%d:%d\r\n",
         TCP_SERVER_IP_ADDR0, TCP_SERVER_IP_ADDR1,
         TCP_SERVER_IP_ADDR2, TCP_SERVER_IP_ADDR3,
         TCP_SERVER_PORT);

  /* ---- Step 4: Register the context with the PCB ------------------------- */
  /*
   * tcp_arg() binds our es_client pointer to the PCB. lwIP will pass it
   * as the first 'arg' parameter to every callback we register.
   * Must be called BEFORE tcp_connect() so the error callback (registered
   * just below) already has the correct arg if the SYN fails.
   */
  tcp_arg(tpcb, es_client);

  /* Register the fatal-error callback now, before connecting.
   * If the server rejects the SYN (RST), tcp_client_error() is called.
   * Without this, a connection failure during the handshake leaks the PCB. */
  tcp_err(tpcb, tcp_client_error);

  /* ---- Step 5: Initiate the TCP connection (sends SYN) ------------------- */
  /*
   * tcp_connect() transmits a SYN to the server and returns ERR_OK if the
   * packet was successfully queued for transmission (NOT if the connection
   * is established yet). The actual three-way handshake completes
   * asynchronously; tcp_client_connected() is called when it succeeds.
   */
  err_t err = tcp_connect(tpcb, &server_ip, TCP_SERVER_PORT, tcp_client_connected);

  if (err == ERR_OK)
  {
    es_client->state = ES_CONNECTING;
    printf("[INFO] SYN sent — waiting for server ACK...\r\n");
    printf("[INFO] State: %s\r\n", get_client_state_name(es_client->state));
    printf("========================================\r\n\n");
  }
  else
  {
    /* tcp_connect() failed synchronously (e.g. routing error, no memory). */
    printf("[ERROR] tcp_connect() failed, error code: %d\r\n", err);
    mem_free(es_client);
    es_client = NULL;
    memp_free(MEMP_TCP_PCB, tpcb);
  }
}

/* ===========================================================================
 * CONNECTED CALLBACK
 * ===========================================================================
 *
 * @brief  Called by lwIP when the three-way TCP handshake completes.
 * @param  arg   Pointer to our tcp_client_struct (set via tcp_arg()).
 * @param  tpcb  Pointer to the connected TCP PCB.
 * @param  err   ERR_OK on success, non-zero if the connection was refused.
 * @retval err_t ERR_OK to keep connection alive, ERR_ABRT to abort.
 *
 * This is the CLIENT equivalent of the server's tcp_accept() callback.
 * In a server: a new PCB is created when a client connects (accept).
 * In a client: this same PCB transitions from SYN_SENT → ESTABLISHED.
 *
 * Here we:
 *   1. Update state to ES_CONNECTED
 *   2. Register recv/sent/poll callbacks
 *   3. Send an initial "Hello" message so the server knows we are ready
 * =========================================================================== */
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
  struct tcp_client_struct *es = (struct tcp_client_struct *)arg;

  if (err != ERR_OK)
  {
    /* Server refused the connection (sent RST) or a timeout occurred. */
    printf("\n[ERROR] Connection failed, error: %d\r\n", err);
    tcp_client_connection_close(tpcb, es);
    return err;
  }

  printf("\n========================================\r\n");
  printf("CONNECTION ESTABLISHED!\r\n");
  printf("========================================\r\n");
  printf("[INFO] Connected to %s:%u\r\n",
         ipaddr_ntoa(&tpcb->remote_ip), tpcb->remote_port);

  /* Update state */
  es->state = ES_CONNECTED;
  printf("[INFO] State: %s\r\n", get_client_state_name(es->state));

  /* Register the remaining callbacks now that the connection is live.
   * Note: tcp_err() was already registered in tcp_client_init() before
   * connecting, so we do not register it again here. */
  tcp_recv(tpcb, tcp_client_recv);    /* Called when server sends us data */
  tcp_sent(tpcb, tcp_client_sent);    /* Called when server ACKs our data */
  tcp_poll(tpcb, tcp_client_poll, 1); /* Called every 1×500ms = 500ms for housekeeping */

  printf("[INFO] Callbacks registered\r\n");

  /* Send a welcome/hello message so the server terminal shows we connected. */
  const char *welcome = "Hello from STM32 TCP Client!\r\n";
  struct pbuf *p_tx = pbuf_alloc(PBUF_TRANSPORT, strlen(welcome), PBUF_RAM);
  if (p_tx != NULL)
  {
    memcpy(p_tx->payload, welcome, strlen(welcome));
    es->p = p_tx;
    tcp_client_send(tpcb, es);
    printf("[INFO] Welcome message sent\r\n");
  }

  printf("========================================\r\n\n");
  return ERR_OK;
}

/* ===========================================================================
 * RECEIVE CALLBACK
 * ===========================================================================
 *
 * @brief  Called by lwIP whenever data arrives from the server.
 * @param  arg   Pointer to our tcp_client_struct.
 * @param  tpcb  Pointer to the TCP PCB.
 * @param  p     Pointer to the received pbuf chain. NULL = server closed connection.
 * @param  err   ERR_OK if data is valid, non-zero on error.
 * @retval err_t ERR_OK to acknowledge, ERR_ABRT if we aborted the connection.
 *
 * Three scenarios are handled:
 *   1. p == NULL  : Server sent a FIN — connection is closing gracefully.
 *   2. err != OK  : Reception error — free pbuf, return error code.
 *   3. Normal data: Acknowledge the window, then hand off to tcp_client_handle().
 *
 * IMPORTANT — tcp_recved() must be called here (receive path), NOT in the
 * send path. It tells lwIP that our application has consumed the data and
 * the TCP receive window can be reopened by that many bytes.
 * =========================================================================== */
static err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  struct tcp_client_struct *es = (struct tcp_client_struct *)arg;
  err_t ret_err;

  LWIP_ASSERT("arg != NULL", arg != NULL);

  /* Scenario 1: p == NULL means the server sent a TCP FIN (graceful close). */
  if (p == NULL)
  {
    printf("\n[RECV] Server closed connection (FIN received)\r\n");
    es->state = ES_CLOSING;

    if (es->p == NULL)
    {
      /* No outgoing data queued — close immediately. */
      tcp_client_connection_close(tpcb, es);
    }
    else
    {
      /* Flush any queued outgoing data before closing. */
      tcp_sent(tpcb, tcp_client_sent);
      tcp_client_send(tpcb, es);
    }
    return ERR_OK;
  }

  /* Scenario 2: lwIP signalled a reception error. Free the pbuf and return. */
  if (err != ERR_OK)
  {
    printf("\n[RECV] Reception error: %d\r\n", err);
    pbuf_free(p);
    return err;
  }

  /* Scenario 3: Data arrived successfully while connected or receiving. */
  if (es->state == ES_CONNECTED || es->state == ES_RECEIVED)
  {
    printf("\n[RECV] %u bytes received from server\r\n", p->tot_len);

    /*
     * Reopen the receive window by the number of bytes we just consumed.
     * This MUST be called in the receive path. Calling it in the send
     * path is incorrect and can corrupt flow control.
     */
    tcp_recved(tpcb, p->tot_len);

    if (es->p == NULL)
    {
      /* No outgoing TX is in progress — handle the received data. */
      es->p = p;
      tcp_client_handle(tpcb, es);
    }
    else
    {
      /*
       * We are still sending a previous reply. Discard this packet to avoid
       * building up a queue of pending replies that could loop back to the server.
       */
      printf("[RECV] Still sending previous reply — incoming data discarded\r\n");
      pbuf_free(p);
    }
    ret_err = ERR_OK;
  }
  else
  {
    /* Data arrived in an unexpected state. Acknowledge and drop silently. */
    printf("[RECV] Data in unexpected state (%s) — discarded\r\n",
           get_client_state_name(es->state));
    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    ret_err = ERR_OK;
  }

  return ret_err;
}

/* ===========================================================================
 * ERROR CALLBACK
 * ===========================================================================
 *
 * @brief  Called by lwIP when a fatal, unrecoverable error occurs.
 * @param  arg  Pointer to our tcp_client_struct.
 * @param  err  The error code (e.g. ERR_RST = -14 for connection reset).
 *
 * CRITICAL: By the time this callback fires, lwIP has ALREADY freed the PCB.
 * Do NOT call tcp_close() or tcp_abort() here — the PCB is gone.
 * Only clean up our own application memory (es_client struct, pending pbufs).
 *
 * Common error codes:
 *   ERR_RST (-14): Server sent a TCP RST — port not open, firewall, or server restarted.
 *   ERR_ABRT (-8): Connection was aborted (by us or by timeout).
 * =========================================================================== */
static void tcp_client_error(void *arg, err_t err)
{
  struct tcp_client_struct *es = (struct tcp_client_struct *)arg;

  printf("\n========================================\r\n");
  printf("TCP CONNECTION ERROR\r\n");
  printf("========================================\r\n");
  printf("[ERROR] Error code: %d\r\n", err);

  if (es != NULL)
  {
    printf("[INFO] Last known state: %s\r\n", get_client_state_name(es->state));

    /* Free any pbuf that was waiting to be sent. */
    if (es->p != NULL)
    {
      pbuf_free(es->p);
      es->p = NULL;
    }

    /* Free our application context structure. */
    mem_free(es);
    es_client = NULL;
  }

  printf("[INFO] Resources freed. Call tcp_client_init() to reconnect.\r\n");
  printf("========================================\r\n\n");
}

/* ===========================================================================
 * POLL CALLBACK
 * ===========================================================================
 *
 * @brief  Called periodically by lwIP for housekeeping (interval: 1×500ms).
 * @param  arg   Pointer to our tcp_client_struct.
 * @param  tpcb  Pointer to the TCP PCB.
 * @retval err_t ERR_OK normally, ERR_ABRT if we abort.
 *
 * The poll callback interval is set in tcp_poll() inside tcp_client_connected().
 * Currently set to 1 (= 500ms). Value 0 would disable the poll callback entirely.
 *
 * This callback is used for two purposes:
 *   1. Retry sending data if a previous tcp_write() was deferred (ERR_MEM).
 *   2. Finalise the connection close if we are in the ES_CLOSING state.
 *
 * It does NOT send unsolicited data — the STM32 only replies when the server
 * sends something first (via Eth_TCP_Server.py interactive terminal).
 * =========================================================================== */
static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb)
{
  struct tcp_client_struct *es = (struct tcp_client_struct *)arg;

  /* Safety check: if the context is NULL something went wrong — abort. */
  if (es == NULL)
  {
    tcp_abort(tpcb);
    return ERR_ABRT;
  }

  /* If we are closing and the poll fires, finalise the close. */
  if (es->state == ES_CLOSING)
  {
    tcp_client_connection_close(tpcb, es);
    return ERR_OK;
  }

  /* If a previous send was deferred due to a full send buffer, retry it now. */
  if (es->p != NULL)
  {
    tcp_client_send(tpcb, es);
    return ERR_OK;
  }

  /* Nothing to do. The STM32 stays silent until the server sends something. */
  return ERR_OK;
}

/* ===========================================================================
 * SENT CALLBACK
 * ===========================================================================
 *
 * @brief  Called by lwIP when the server acknowledges (ACKs) data we sent.
 * @param  arg   Pointer to our tcp_client_struct.
 * @param  tpcb  Pointer to the TCP PCB.
 * @param  len   Number of bytes the server acknowledged in this ACK.
 * @retval err_t ERR_OK.
 *
 * When a TCP ACK arrives from the server, lwIP calls this callback.
 * If more data is queued in es->p (pbuf chain), we continue sending.
 * If the connection is closing and all data is sent, we finalise the close.
 * =========================================================================== */
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  struct tcp_client_struct *es = (struct tcp_client_struct *)arg;

  LWIP_UNUSED_ARG(len); /* len is informational; we check es->p directly */

  /* Reset retry counter — the server is alive and responding. */
  es->retries = 0;

  printf("\n[ACK] Server acknowledged %u bytes\r\n", len);

  if (es->p != NULL)
  {
    /* More pbuf chunks are still queued — send the next chunk. */
    printf("[INFO] More data pending, continuing...\r\n");
    tcp_sent(tpcb, tcp_client_sent);
    tcp_client_send(tpcb, es);
  }
  else
  {
    /* All data has been acknowledged. */
    printf("[INFO] All data delivered\r\n");

    if (es->state == ES_CLOSING)
    {
      /* We were waiting for the last data to be ACKed before closing. */
      tcp_client_connection_close(tpcb, es);
    }
  }

  return ERR_OK;
}

/* ===========================================================================
 * SEND HELPER
 * ===========================================================================
 *
 * @brief  Enqueues data from es->p into the lwIP send buffer using tcp_write(),
 *         then flushes the segment immediately with tcp_output().
 * @param  tpcb  Pointer to the TCP PCB.
 * @param  es    Pointer to our client context (es->p holds the data to send).
 *
 * tcp_write() copies data into the lwIP internal send buffer. It does NOT
 * transmit immediately — tcp_output() forces the segment to be sent now
 * without waiting for Nagle's algorithm.
 *
 * If the send buffer is full (ERR_MEM), the pbuf is kept in es->p and the
 * poll callback will retry on the next 500ms tick.
 *
 * After a successful tcp_write(), the pbuf chunk is freed (lwIP has its own
 * copy thanks to TCP_WRITE_FLAG_COPY). If the pbuf chain has more chunks,
 * the loop continues until all data is enqueued or the buffer is full.
 *
 * NOTE: tcp_recved() must NOT be called here. It belongs in tcp_client_recv()
 * (the receive path) to reopen the receive window after consuming incoming data.
 * =========================================================================== */
static void tcp_client_send(struct tcp_pcb *tpcb, struct tcp_client_struct *es)
{
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;

  /*
   * Iterate through the pbuf chain. Stop if:
   *   - A write error occurred
   *   - No more data to send (es->p == NULL)
   *   - The current chunk doesn't fit in the available send buffer
   */
  while ((wr_err == ERR_OK) &&
         (es->p != NULL) &&
         (es->p->len <= tcp_sndbuf(tpcb)))
  {
    ptr = es->p;

    /*
     * tcp_write() with TCP_WRITE_FLAG_COPY:
     *   lwIP copies the data internally. We can free the pbuf immediately
     *   after this call succeeds — we do not need to keep it alive.
     */
    wr_err = tcp_write(tpcb, ptr->payload, ptr->len, TCP_WRITE_FLAG_COPY);

    if (wr_err == ERR_OK)
    {
      u8_t freed;

      /* Advance to the next pbuf in the chain. */
      es->p = ptr->next;

      /*
       * If there is a next chunk, increment its reference count so it is
       * not freed by pbuf_free(ptr) below (which only frees 'ptr',
       * not the entire chain, if the ref count of the next is > 0).
       */
      if (es->p != NULL) { pbuf_ref(es->p); }

      /* Free the chunk we just wrote. Loop because pbuf_free can return 0
       * if the buffer is still referenced by something else. */
      do { freed = pbuf_free(ptr); } while (freed == 0);
    }
    else if (wr_err == ERR_MEM)
    {
      /*
       * Send buffer is full. Keep es->p pointing at the unfinished chunk.
       * The poll callback (tcp_client_poll) will retry after 500ms.
       */
      es->p = ptr;
    }
    else
    {
      printf("[SEND] tcp_write error: %d\r\n", wr_err);
    }
  }

  /*
   * tcp_output() pushes the enqueued data as a TCP segment immediately,
   * bypassing Nagle's algorithm. Without this call, the data might sit in
   * the lwIP buffer until the next poll or until enough data accumulates.
   */
  tcp_output(tpcb);
}

/* ===========================================================================
 * CONNECTION CLOSE HELPER
 * ===========================================================================
 *
 * @brief  Cleanly tears down the TCP connection and frees all resources.
 * @param  tpcb  Pointer to the TCP PCB.
 * @param  es    Pointer to our client context (may be NULL).
 *
 * Proper close sequence:
 *   1. Deregister all lwIP callbacks (prevents them from firing on a dead PCB).
 *   2. Free our application context struct.
 *   3. Call tcp_close() to send the TCP FIN and release the PCB.
 *
 * After this function returns, 'tpcb' must not be used — lwIP owns it.
 * =========================================================================== */
static void tcp_client_connection_close(struct tcp_pcb *tpcb, struct tcp_client_struct *es)
{
  printf("\n========================================\r\n");
  printf("CLOSING TCP CONNECTION\r\n");
  printf("========================================\r\n");

  /* Deregister all callbacks so they don't fire after the PCB is closed. */
  tcp_arg(tpcb,  NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb,  NULL);
  tcp_poll(tpcb, NULL, 0);

  /* Free our application context. */
  if (es != NULL)
  {
    mem_free(es);
    es_client = NULL;
  }

  /* Send FIN and release the PCB back to the lwIP memory pool. */
  tcp_close(tpcb);

  printf("[INFO] Connection closed\r\n");
  printf("========================================\r\n\n");
}

/* ===========================================================================
 * APPLICATION DATA HANDLER
 * ===========================================================================
 *
 * @brief  Called by tcp_client_recv() to process data received from the server
 *         and send a reply.
 * @param  tpcb  Pointer to the TCP PCB (used to send the reply).
 * @param  es    Pointer to our client context (es->p holds the received pbuf).
 *
 * COMMUNICATION MODEL
 * -------------------
 *  This function implements the STM32 side of the conversation:
 *
 *    Server sends text (user types in Eth_TCP_Server.py terminal)
 *        → STM32 prints it on UART (visible in Minicom)
 *        → STM32 replies: "[STM32 ECHO] <received text>[STM32] Received OK"
 *
 *  The Python server ONLY prints replies — it never auto-replies.
 *  This design ensures there is NO infinite loop.
 *
 * STEPS
 * -----
 *  1. Copy the received pbuf data into a flat C string (safe for printf).
 *  2. Free the pbuf BEFORE allocating the TX pbuf (avoids fragmentation).
 *  3. Build the echo response string.
 *  4. Allocate a TX pbuf, copy the response, and send it.
 * =========================================================================== */
static void tcp_client_handle(struct tcp_pcb *tpcb, struct tcp_client_struct *es)
{
  /* Step 1: Copy received pbuf data into a flat null-terminated buffer.
   *
   * pbuf_copy_partial() works correctly even if the received data spans
   * multiple chained pbufs (as opposed to just reading es->p->payload which
   * only gives the first chunk).
   */
  char recv_data[MAX_BUFFER_SIZE];
  memset(recv_data, '\0', sizeof(recv_data));

  uint16_t copy_len = (es->p->tot_len < (MAX_BUFFER_SIZE - 1))
                      ? es->p->tot_len
                      : (MAX_BUFFER_SIZE - 1);

  pbuf_copy_partial(es->p, recv_data, copy_len, 0);
  recv_data[copy_len] = '\0';

  /* Step 2: Free the received pbuf NOW — before allocating the TX pbuf.
   *
   * Freeing here is essential because lwIP's memory pool is limited.
   * Holding a RX pbuf while allocating a TX pbuf can exhaust the pool
   * and cause pbuf_alloc() to fail (returns NULL).
   */
  pbuf_free(es->p);
  es->p = NULL;

  /* Step 3: Print received data on UART (visible in Minicom). */
  printf("\n[STM32 RX] From server:\r\n  %s\r\n", recv_data);

  /* Step 4: Build the echo reply and send it.
   *
   * Format: "[STM32 ECHO] <original message>[STM32] Received & processed OK\r\n"
   */
  char response[MAX_BUFFER_SIZE];
  int resp_len = snprintf(response, sizeof(response),
                          "[STM32 ECHO] %s[STM32] Received & processed OK\r\n",
                          recv_data);

  printf("[STM32 TX] Echoing back:\r\n  %s\r\n", response);

  struct pbuf *p_tx = pbuf_alloc(PBUF_TRANSPORT, (u16_t)resp_len, PBUF_RAM);
  if (p_tx != NULL)
  {
    memcpy(p_tx->payload, response, resp_len);
    es->p = p_tx;
    tcp_client_send(tpcb, es);
  }
  else
  {
    printf("[HANDLE] pbuf_alloc failed — reply not sent\r\n");
  }
}
