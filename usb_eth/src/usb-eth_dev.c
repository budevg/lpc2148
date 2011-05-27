
#define usb_SOF_handler(frame_number) FALSE
#define usb_device_status_handler(status) FALSE
#define usb_control_class_handler() FALSE
#define usb_control_standard_custom_handler() FALSE
#define usb_control_vendor_handler() FALSE
#define usb_control_reserved_handler() FALSE



#define INT_EP  1
#define BULK_EP 2

#define MAX_PACKET_SIZE (64)

/* #define ETH_DEV_DEBUG  */
#define ETH_DEV_DBG_ERROR(fmt, args...) DBG("USB DEV ERROR:" fmt, ##args)

#ifdef ETH_DEV_DEBUG
#define ETH_DEV_DBG_INFO(fmt, args...) DBG("USB DEV INFO:" fmt, ##args)
#else
#define ETH_DEV_DBG_INFO(fmt, args...)
#endif

static const uint8_t usb_descriptors[] = {
  0x12,                /* length                      */
  DESC_DEVICE,
  LE_WORD(0x0110),     /* USB version (BCD)           */
  0x02,                /* Class = CDC (Communication) */
  0x00,                /* Device Sub Class            */
  0x00,                /* Device Protocol             */
  MAX_PACKET_SIZE0,    /* max packet size for EP 0    */
  LE_WORD(0x050d),     /* Vendor: Belkin              */
  LE_WORD(0x0004),     /* Product: Belkin             */
  LE_WORD(0x0201),     /* Device release number (BCD) */
  1,                   /* Manufacturer String Index   */
  2,                   /* Product String Index        */
  3,                   /* SerialNumber String Index   */
  1,                   /* Number of Configurations    */

  0x09,
  DESC_CONFIGURATION,
  LE_WORD(32),         /* total length                */
  0x01,                /* number of interfaces        */
  0x01,                /* This Configuration          */
  0,                   /* Configuration String Index  */ 
  USB_ATTR_BUS_POWERED,/* Attributes                  */
  50,                  /* Max Power (unit is 2mA)     */

  0x09,
  DESC_INTERFACE,     /* This is the Data Interface   */
  0,                  /* Interface Number             */
  0,                  /* Alternate Setting            */
  2,                  /* NumEndPoints                 */
  0x00,               /* Interface Class              */
  0x00,               /* Interface Subclass           */
  0x00,               /* Interface Protocol           */
  0x00,               /* Interface String Index       */

  0x07,
  DESC_ENDPOINT,
  USB_EP_OUT | BULK_EP, /* Endpoint Address           */
  0x02,               /* Attributes = bulk            */
  LE_WORD(MAX_PACKET_SIZE),
  0x00,               /* Interval (not used for bulk) */

  0x07,
  DESC_ENDPOINT,
  USB_EP_IN | BULK_EP,/* Endpoint Address             */
  0x02,               /* Attributes = bulk            */
  LE_WORD(MAX_PACKET_SIZE),
  0x00,               /* Interval (not used for bulk) */

  0x04,
  DESC_STRING,
  LE_WORD(0x0409),    /* Language = US English        */

  0x0E,
  DESC_STRING,
  'L', 0, 'P', 0, 'C', 0, 'U', 0, 'S', 0, 'B', 0,

  0x0E,
  DESC_STRING,
  'U', 0, 'S', 0, 'B', 0, 'e', 0, 't', 0, 'h', 0,

  0x12,
  DESC_STRING,
  'D', 0, 'E', 0, 'A', 0, 'D', 0, 'B', 0, 'E', 0, 'E', 0, 'F', 0,

  0 /* terminator */
};


void usb_eth_rx(uint8_t ep, uint8_t stat);
void usb_eth_tx(uint8_t ep, uint8_t stat);
void usb_eth_handle_recv(uint8_t *buffer, uint16_t size);

static usb_ep_handler_t* const usb_ep_handlers[32] = {
   usbEP0OutHandler, usbEP0InHandler, /* EP  0 Out, In */
   0, 0,                              /* EP  1 Out, In */
   &usb_eth_rx, &usb_eth_tx,          /* EP  2 Out, In */
   0,
};

#define PACKET_SIZE (512)
#define RECEIVE_RING_SIZE (8)
typedef struct packet_s {
  uint16_t transferred;
  uint16_t size;
  uint8_t data[PACKET_SIZE];
}packet_t;

static void packet_init(packet_t* packet)
{
  int i;
  packet->size = 0;
  packet->transferred = 0;
  for (i=0 ;i<PACKET_SIZE; i++ ) {
    packet->data[i] = 0;
  }
}

#define RING_IS_FULL(head, tail, size) ((((tail) + 1) % (size)) == head)
#define RING_IS_EMPTY(head, tail, size) ((tail) == (head))
#define RING_INC(index, size)                   \
do {                                            \
  index = (index + 1) % size;                   \
}while (0);

static packet_t receive_ring[RECEIVE_RING_SIZE];
static packet_t *receive_packet;
static uint16_t receive_ring_head;
static uint16_t receive_ring_tail;

static uint16_t bytes_to_send;
static uint16_t bytes_sent;
static uint8_t send_buffer[PACKET_SIZE];
volatile static uint8_t packet_pending;



void usb_eth_rx(uint8_t ep, uint8_t stat)
{
  int len;
  int ip_length;
  uint16_t eth_hdr_type;

  if (receive_packet == 0) {
    if (RING_IS_FULL(receive_ring_head, receive_ring_tail, RECEIVE_RING_SIZE)) {
      ETH_DEV_DBG_ERROR("usb_eth_rx: receive ring is full, head=%d, tail=%d",
                        receive_ring_head, receive_ring_tail);
      return;
    }
    receive_packet = &receive_ring[receive_ring_tail];
    packet_init(receive_packet);
  }

  if (receive_packet->transferred < PACKET_SIZE) {
    len = usbRead(ep,
                  &receive_packet->data[0] + receive_packet->transferred,
                  PACKET_SIZE - receive_packet->transferred);
  }
  else {
    /* discard the bytes */
    len = usbRead(ep, 0, 0);
  }
  receive_packet->transferred += len;
  
  ETH_DEV_DBG_INFO("usb_eth_rx: read %d bytes, head=%d, tail=%d",
                   len,
                   receive_ring_head,
                   receive_ring_tail);

  if (receive_packet->transferred < UIP_LLH_LEN) {
    /* ethernet header was not received yet got some garbage which we need
     to ignore */
    ETH_DEV_DBG_ERROR("got part of packet with %d bytes", receive_packet->transferred);
    receive_packet->transferred = 0;
    return;
  }

  if (receive_packet->size == 0) {
    eth_hdr_type = (receive_packet->data[0xc] << 8) | receive_packet->data[0xd];
  
    if (eth_hdr_type == UIP_ETHTYPE_IP) {
      ip_length = receive_packet->data[0x10]*0xff + receive_packet->data[0x11];
      receive_packet->size = ip_length + UIP_LLH_LEN;
    }
    else if (eth_hdr_type == UIP_ETHTYPE_ARP) {
      receive_packet->size = receive_packet->transferred;
    }
    else {
      ETH_DEV_DBG_ERROR("Unknown ethernet frame 0x%x", eth_hdr_type);
      receive_packet->size = receive_packet->transferred;
    }
  }

  if (receive_packet->transferred >= receive_packet->size) {
    if (receive_packet->transferred >= receive_packet->size) {
      if (receive_packet->transferred >= PACKET_SIZE) {
        ETH_DEV_DBG_ERROR("Discarding packet of size %d", receive_packet->transferred);
        packet_init(receive_packet);
        receive_packet = 0;
      }
      else {
        ETH_DEV_DBG_INFO("Received packet of size %d", receive_packet->transferred);
        receive_packet = 0;
        RING_INC(receive_ring_tail, RECEIVE_RING_SIZE);
      }
    }
  }
}

void usb_eth_tx(uint8_t ep, uint8_t stat)
{
  int len;
  int i;
  
  if (!packet_pending || bytes_to_send == 0) {
    ETH_DEV_DBG_INFO("usb_eth_tx, send buffer is empty, packet_pending=%d", packet_pending);
    usbEnableNAKInterrupts(0);
    return;
  }

  len = MIN(bytes_to_send - bytes_sent, MAX_PACKET_SIZE);
  
  usbWrite(ep, ((uint8_t*) &send_buffer)+bytes_sent, len);
  bytes_sent += len;

  ETH_DEV_DBG_INFO("usb_eth_tx: send %d bytes, frame data sent %d/%d",
                   len, bytes_sent, bytes_to_send);


  if (bytes_sent >= bytes_to_send) {
    /* finished sending data */
    for (i=0; i< bytes_to_send;i++) {
      send_buffer[i] = 0;
    }
    bytes_to_send = 0;
    bytes_sent = 0;
    packet_pending = 0;
  }
}


uint16_t network_device_read()
{
  uint16_t ret = 0;
  int i;
  packet_t *packet;
  if (!RING_IS_EMPTY(receive_ring_head, receive_ring_tail, RECEIVE_RING_SIZE)) {
    /* we got ip packet, copy it and notify uip framework */
    packet = &receive_ring[receive_ring_head];
    for (i=0; i<packet->transferred; i++) {
      uip_buf[i] = packet->data[i];
      packet->data[i] = 0;
    }
    ret = packet->transferred;
    packet->transferred = 0;
    RING_INC(receive_ring_head, RECEIVE_RING_SIZE);
  }
  return ret;
}

void network_device_send()
{
  int i;
  uint16_t len;

  if (packet_pending) {
    ETH_DEV_DBG_ERROR("network_device_send: already pending packet, discarding request");
    return;
  }

  if (uip_len == 0) {
    ETH_DEV_DBG_ERROR("network_device_send: uip_len == 0");
    return;
  }

  len = MIN(sizeof(send_buffer), uip_len);
  
  vicDisable(INT_CHANNEL_USB);
  
  for (i=0; i<len; i++) {
    send_buffer[i] = uip_buf[i];
  }
  bytes_sent=0;
  bytes_to_send = len;
  packet_pending = 1;

  usbEnableNAKInterrupts(INACK_BI);
  vicEnable(INT_CHANNEL_USB);

}

void network_device_init()
{
  int i,j;
  
  for (i=0; i<sizeof(send_buffer); i++) {
    send_buffer[i]=0;
  }
  bytes_to_send=0;
  bytes_sent = 0;
  packet_pending = 0;

  receive_ring_head = 0;
  receive_ring_tail = 0;
  receive_packet = 0;
  for (i=0; i<RECEIVE_RING_SIZE; i++) {
    for (j=0; j<PACKET_SIZE; j++) {
      receive_ring[i].data[j] = 0;
    }
    receive_ring[i].transferred = 0;
  }
}
