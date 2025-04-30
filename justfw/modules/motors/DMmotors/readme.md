# 注意事项
1. 达妙电机有多种驱动模式，本框架内只包含mit模式的驱动。
2. 使用mit的速度模式不会限制力矩输出（来自淘宝商家保修人员的解答）。（初步解决方案是使用一拖四模式，或者自行添加pid控制）
3. 达妙3519刹车会产生电流回馈到母线上，如果搭配了booster升压模块会特别容易烧升压或者电调。
4. 达妙电机还有一拖四驱动模式，协议与大疆c620几乎一样，使用记得使用达妙的调试工具更换一拖四固件。
    特别说明：使用一拖四能够控力矩
# 官方资料
达妙科技导航资料
1. 客户帮助中心：https://gl1po2nscb.feishu.cn/wiki/MZ32w0qnnizTpOkNvAZcJ9SlnXb
2. 达妙产品资料Gitee总链接：https://gitee.com/kit-miao/damiao
3. 达妙产品资料Github总链接:https://github.com/dmBots/DAMIAO-Motor
4. 飞书云盘，免登录直接下载：https://gl1po2nscb.feishu.cn/drive/folder/RJL7fFT4ll9PDSdvM6Pc5vntnPw
