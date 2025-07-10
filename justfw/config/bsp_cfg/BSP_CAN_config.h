//
// Created by Ukua on 2023/8/16.
//

#ifndef JUSTFW_CAN_CONFIG_H
#define JUSTFW_CAN_CONFIG_H

//[[[

//@item:发送FIFO缓存区大小
//@type:int
//@default:16
//@range:1~1024
#define BSP_CAN_TX_FIFO_SIZE 16

//@item:接收FIFO缓存区大小
//@type:int
//@default:16
//@range:1~1024
#define BSP_CAN_RX_FIFO_SIZE 16

//@item:CAN接收话题名称
//@type:string
//@default:"CAN_RX"
#define BSP_CAN_RX_TOPIC_NAME "CAN_RX"

//@item:CAN发送话题名称
//@type:string
//@default:"CAN_TX"
#define BSP_CAN_TX_TOPIC_NAME "CAN_TX"

//@item:CAN端口1
//@type:any
//@default:hcan1
#define _BSP_CAN_PORT1 hcan1

//@item:CAN端口2
//@type:any
//@default:hcan2
#define _BSP_CAN_PORT2 hcan2

//]]]

#endif //JUSTFW_CAN_CONFIG_H
