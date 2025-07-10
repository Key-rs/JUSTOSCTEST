//
// Created by Konodoki on 2024/12/27.
//

#include "user.h"
#include <stddef.h>
extern const unsigned char _user_modules_start;
extern const unsigned char _user_modules_end;
void user_init(){
  CUser_Module *base=(CUser_Module*)(&_user_modules_start);
  size_t count=((size_t)(&_user_modules_end) - (size_t)(&_user_modules_start)) / sizeof(CUser_Module);
  for(size_t i =0;i<count;i++){
    base[i].init_func();
  }
}