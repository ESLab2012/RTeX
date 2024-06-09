#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <thread>
#include "rclcpp/lockFree_readyList/lockFree_readyList.hpp"

// readyList for this RTeX executor
LF_readyList readyList;

map<uintptr_t,uintptr_t> sub_correspond_waitable;

pthread_cond_t MDList_not_empty = PTHREAD_COND_INITIALIZER;
pthread_mutex_t MDList_mutex = PTHREAD_MUTEX_INITIALIZER;
std::condition_variable cv;

bool
lockFree_init_MDList()
{
    bool success = false;
    LF_readyList *temp = NULL;
    temp = &readyList;
    if (temp!=NULL) {
        success = true;
    }
    return success;
}

uintptr_t
lockFree_deleteMin(uint32_t thread_id, uint8_t* type)
{
    uintptr_t temp = readyList.DeleteMin(thread_id);
    if(temp==EMPTY) {
        return NIL;//return case 0
    }
    int k = (*readyList.ValToNode[temp]).cb_type;
    if (k == 1) (*type)=TYPE_TIMER;
    else if(k == 2) (*type)=TYPE_SUB;
    else if(k == 3) (*type)=TYPE_SERVICE;
    else if(k == 4) (*type)=TYPE_CLIENT;
    else if(k == 5) (*type)=TYPE_WAITABLE;
    return temp;
}