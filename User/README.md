# 用户逻辑控制代码
为了方便模块的更新和管理，降低用户代码与电控库代码的耦合性，请将用户模块放在此目录
## 如何使用
### 1.编写用户自己的模块
example.c
```c
    void example_loop(){
        while(1){
            //TODO 你的控制代码
            osDelay(1);
        }
    }   
    void example_init(){
        osThreadDef(ExampleTask, example_loop, osPriorityNormal, 0, 1024);
        osThreadCreate(osThread(ExampleTask), NULL);
    }
    //启用用户模块，加上这一行程序自动从初始化函数加载用户模块，无需额外的代码
    //第一个参数为模块的名字
    //第二个参数为模块的初始化函数
    USER_EXPORT(example,example_init);
```