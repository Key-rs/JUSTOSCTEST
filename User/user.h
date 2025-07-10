//
// Created by Konodoki on 2024/12/27.
//

#ifndef JUSTFW_USER_H
#define JUSTFW_USER_H
#define SECTION(x)  __attribute__((section(x)))
/**
 * @brief 用户模块
 *
 */
#define USER_EXPORT(_name,_init_func) \
        const CUser_Module SECTION("cUserModule") CUser_Module_##_name##_initFunc = \
        {                             \
            .name=#_name,             \
            .init_func=_init_func     \
        }
typedef struct
{
  const char *name;
  void (*init_func)(void);
} CUser_Module;
void user_init();
#endif // JUSTFW_USER_H
