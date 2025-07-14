#include "tinybus.h"
#include "list.h"

// 全局话题列表（存储所有注册的话题）
static List_t topic_list;

/**
 * @brief 初始化总线系统
 * @return pdTRUE 初始化成功
 */
BaseType_t xBusInit(void) {
    // 初始化话题列表（链表结构）
    vListInitialise(&topic_list);
    return pdTRUE;
}

/**
 * @brief 在总线上搜索指定名称的话题
 * @param pcTopicName 话题名称（字符串）
 * @return 找到的话题句柄（未找到返回NULL）
 */
BusTopicHandle_t xBusTopicSearch(const char *pcTopicName) {
    // 遍历话题列表头节点开始查找
    ListItem_t *item = listGET_HEAD_ENTRY(&topic_list);
    while (item != listGET_END_MARKER(&topic_list)) {
        // 从链表项中获取话题实体
        BusTopicHandle_t topic = listGET_LIST_ITEM_OWNER(item);
        // 字符串匹配判断话题名
        if (strcmp(topic->pcName, pcTopicName) == 0) {
            return topic;  // 找到目标话题
        }
        item = listGET_NEXT(item);  // 移动到下一个链表项
    }
    return NULL;  // 未找到话题
}

/**
 * @brief 注册新话题（若已存在则直接返回）
 * @param pcTopicName 话题名称（字符串）
 * @return 话题句柄（注册成功/已存在时返回）
 */
BusTopicHandle_t xBusTopicRegister(const char *pcTopicName) {
    // 先检查话题是否已存在
    BusTopicHandle_t topic = xBusTopicSearch(pcTopicName);
    if (topic != NULL) return topic;  // 话题已存在，直接返回

    // 新建话题结构体并初始化
    topic = (BusTopicHandle_t)JUST_MALLOC(sizeof(BusTopicDef_t));
    topic->pcName = pcTopicName;          // 设置话题名称
    vListInitialise(&topic->xSubscriberList);  // 初始化订阅者列表

    // 将新话题添加到全局话题列表
    ListItem_t *item = (ListItem_t *)JUST_MALLOC(sizeof(ListItem_t));
    listSET_LIST_ITEM_OWNER(item, topic);  // 链表项绑定话题实体
    vListInsertEnd(&topic_list, item);     // 插入到列表尾部

    return topic;
}

/**
 * @brief 为指定话题添加订阅者
 * @param xTopic 目标话题句柄
 * @param pvCallback 订阅者的回调函数（消息到达时触发）
 * @return 订阅者句柄
 */
BusSubscriberHandle_t xBusSubscribe(BusTopicHandle_t xTopic, void (*pvCallback)(void *message, BusSubscriberHandle_t subscriber)) {
    // 新建订阅者结构体并初始化
    BusSubscriberHandle_t subscriber = (BusSubscriberHandle_t)JUST_MALLOC(sizeof(BusTopicDef_t));
    subscriber->pxTopic = xTopic;       // 绑定目标话题
    subscriber->pvCallback = pvCallback; // 设置回调函数
    subscriber->xEnable = pdTRUE;       // 默认启用订阅

    // 将订阅者添加到话题的订阅者列表
    ListItem_t *item = (ListItem_t *)JUST_MALLOC(sizeof(ListItem_t));
    listSET_LIST_ITEM_OWNER(item, subscriber);  // 链表项绑定订阅者实体
    vListInsertEnd(&(xTopic->xSubscriberList), item);  // 插入到订阅者列表尾部

    return subscriber;
}

/**
 * @brief 通过话题名称订阅（自动注册不存在的话题）
 * @param cTopicName 话题名称
 * @param pvCallback 订阅者回调函数
 * @return 订阅者句柄
 */
BusSubscriberHandle_t xBusSubscribeFromName(char *cTopicName, void (*pvCallback)(void *message, BusSubscriberHandle_t subscriber)) {
    // 搜索或注册话题
    BusTopicHandle_t xTopic = xBusTopicSearch((const char *)cTopicName);
    if (xTopic == NULL) xTopic = xBusTopicRegister(cTopicName);

    // 调用基础订阅函数
    return xBusSubscribe(xTopic, pvCallback);
}

/**
 * @brief 向指定话题发布消息（触发所有订阅者回调）
 * @param pxTopic 目标话题句柄
 * @param pvMessage 要发布的消息内容
 */
void vBusPublish(BusTopicHandle_t pxTopic, void *pvMessage) {
    // 遍历话题的订阅者列表
    ListItem_t *item = listGET_HEAD_ENTRY(&pxTopic->xSubscriberList);
    while (item != listGET_END_MARKER(&pxTopic->xSubscriberList)) {
        // 获取订阅者实体并触发回调
        BusSubscriberHandle_t subscriber = listGET_LIST_ITEM_OWNER(item);
        subscriber->pvCallback(pvMessage, subscriber);  // 传递消息和订阅者自身句柄

        item = listGET_NEXT(item);  // 移动到下一个订阅者
    }
}

/**
 * @brief 通过话题名称发布消息（话题不存在则忽略）
 * @param pcTopicName 话题名称
 * @param pvMessage 要发布的消息内容
 */
void vBusPublishFromName(char *pcTopicName, void *pvMessage) {
    // 搜索话题并发布
    BusTopicHandle_t xTopic = xBusTopicSearch((const char *)pcTopicName);
    if (xTopic == NULL) return;  // 话题不存在，直接返回

    vBusPublish(xTopic, pvMessage);
}