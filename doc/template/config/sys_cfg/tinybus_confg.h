
#ifndef JUSTFW_TINYBUS_CONFIG_H
#define JUSTFW_TINYBUS_CONFIG_H

//[[[

//@item:启用tinybus
//@type:bool
//@default:True
#define TINYBUS_ENABLE

#ifdef TINYBUS_ENABLE

//{{{ 数值参数

//@item:整数
//@type:int
//@default:1
//@range:1~10
#define TINYBUS_TESTINT (1)

//@item:16进制整数
//@type:int16
//@default:0x1
//@range:-0x01~0x10
//@width:2
#define TINYBUS_TESTINT16 (-0x01)

//@item:浮点
//@type:float
//@default:1.0
//@range:1.0~10.0
#define TINYBUS_TESTFP (1.0f)

//}}} 数值参数

//@item:字符串
//@type:string
//@default:"helloworld"
#define TINYBUS_TESTSTRING "helloworld"

//@item:任意值
//@type:any
//@default:TEST
#define TINYBUS_TEST TEST

//@item:选项
//@type:selection
//@default:first
//>first:(1)
//>second:(2)
//>第三:(3)
#define TINYBUS_TESTSELECT (2)

#endif  // TINYBUS_ENABLE

//]]]

#endif  // JUSTFW_TINYBUS_CONFIG_H
