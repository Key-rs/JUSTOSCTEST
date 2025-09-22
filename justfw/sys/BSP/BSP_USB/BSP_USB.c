
#include "BSP_USB_cfg.h"
#include "intf_sys.h"

static SharedStreamHandle_t usb_rx_sharer;         // USB RX 数据共享器
static StreamListenerHandle_t usb_tx_listener;     // USB TX 数据监听器
static StreamBufferHandle_t usb_tx_buffer;         // USB TX 数据缓冲区
static StaticStreamBuffer_t usb_tx_buffer_struct;  // USB TX 静态缓冲区结构

#define USB_USE_BUS
#ifdef USB_USE_BUS
static BusTopicHandle_t usb_tx_subscriber;  // USB TX 总线订阅者
static BusTopicHandle_t usb_rx_topic;       // USB RX 总线主题

BusSubscriberHandle_t usb_tx_subscriber_;

static StreamBufferHandle_t usb_rx_output_buffer;  // USB RX 输出缓冲区
static StreamListenerHandle_t usb_rx_output_listener;

static void usb_rx_to_bus_handler(StreamListenerHandle_t listener) {
    (void)listener;
    uint8_t buffer[USB_RX_BUFFER_SIZE];
    uint16_t length = xStreamBufferReceive(usb_rx_output_buffer, buffer, USB_TX_BUFFER_SIZE, 0);
    INTF_Serial_MessageTypeDef msg = {
        .data = buffer,
        .len = length};
    vBusPublish(usb_rx_topic, &msg);
}

static void usb_tx_bus_callback(void *message, BusSubscriberHandle_t subscriber) {
    INTF_Serial_MessageTypeDef *msg = (INTF_Serial_MessageTypeDef *)message;
    xStreamWrite(usb_tx_buffer, msg->data, msg->len, portMAX_DELAY);
}
#endif

void usb_cdc_rx_handler(uint8_t *data, uint32_t length) {
    xSharedStreamWrite(usb_rx_sharer, data, length, portMAX_DELAY);
}

static void usb_tx_data_handler(StreamListener_t *listener) {
    (void)listener;

    uint8_t buffer[USB_TX_BUFFER_SIZE] = {0};
    uint16_t length = xStreamBufferReceive(usb_tx_buffer, buffer, USB_TX_BUFFER_SIZE, 0);
    extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
    CDC_Transmit_FS(buffer, length);
}

void BSP_USB_Init() {
    usb_tx_buffer = xStreamCreate(USB_TX_BUFFER_SIZE);
    usb_rx_sharer = xSharedStreamCreate(USB_RX_BUFFER_SIZE);
    usb_tx_listener = xStreamListenerCreate(usb_tx_buffer);
    usb_tx_listener->pvOnDataReceived = usb_tx_data_handler;

    vSharePtrStatic(USB_TX_BUFFER_NAME, usb_tx_buffer);
    vSharePtrStatic(USB_RX_SHARER_NAME, usb_rx_sharer);

#ifdef USB_USE_BUS
    // usb_rx_output_buffer = xStreamBufferCreate(USB_RX_BUFFER_SIZE, 1);
    // usb_rx_sharer->register_output(usb_rx_sharer, usb_rx_output_buffer);
    usb_rx_output_buffer = xSharedStreamOutputCreate(usb_rx_sharer, USB_RX_BUFFER_SIZE);
    usb_rx_output_listener = xStreamListenerCreate(usb_rx_output_buffer);
    usb_rx_output_listener->pvOnDataReceived = usb_rx_to_bus_handler;

    usb_rx_topic = xBusTopicRegister(BSP_USB_RX_TOPIC_NAME);
     usb_tx_subscriber_ = xBusSubscribeFromName(BSP_USB_TX_TOPIC_NAME, usb_tx_bus_callback);
    usb_tx_subscriber=usb_tx_subscriber_->pxTopic;
#endif
}