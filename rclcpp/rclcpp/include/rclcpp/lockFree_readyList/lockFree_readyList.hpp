#ifndef rclcpp_lockFree
#define rclcpp_lockFree

#include <stddef.h>
#include <vector>
#include <math.h>
#include <unordered_map>
#include <atomic>
#include <iostream>
#include <condition_variable>
#include <map>
#include <cmath>
#include <vector>
#include "rclcpp/executors/exp_config.hpp"

using namespace std;


#define NIL                     0x0
#define EMPTY                   0x1

#define DUMMY_HEAD              0
#define TYPE_TIMER              1
#define TYPE_SUB                2
#define TYPE_SERVICE            3
#define TYPE_CLIENT             4
#define TYPE_WAITABLE           5

#define ALIGN_VALUE             (1<<POSTERFLAG_SIZE)
#define IN                      0x01
#define RM                      0x02
#define SETBIT(n)               (0x1 << (n))
#define CLEARBIT(p, n)          (p & ~SETBIT(n))
#define SETMARK(p, m)           (p | m)
#define CLEARMARK(p, m)         (p & ~m)
#define ISMARKED(p, m)          (p & m)
#define CLEARALL(p)             (p & ~(ALIGN_VALUE-1))
#define ADPMARK(p)              (p & (ALIGN_VALUE-4))
//#define CLEARALL(p)             (p & ~0x3FF)
//#define ADPMARK(p)              (p & 0x3FC)

extern std::condition_variable  cv;
extern pthread_cond_t           MDList_not_empty;
extern pthread_mutex_t          MDList_mutex;

//Data Structures for Node in the readyList
struct lock_free_Node{
    uintptr_t       val;
    uint8_t         cb_type;
    lock_free_Node* child;
};

#define HeadNode lock_free_Node

// If POSTERFLAG_SIZE = 10, Aligned to 0b100 0000 0000, so that we have 10 bits to mark.
struct cStack{
    uint32_t        mark;   //used to mark any nodes on the task chain are ready
    int             counter[CHAIN_NODE_MAX];
} __attribute__ ((aligned (ALIGN_VALUE)));


class LF_readyList {
public:
    LF_readyList() {
        // Number of registered chains in this `readyList`
        chainNum = 0;               
        for(int i=0; i<CHAIN_NUM_MAX; i++){
            // Number of registered nodes in i-th chain
            nodeNum[i] = 0;         
            dummyHead[i] = NULL;
            // global `cStack` pointer of the i-th chain
            gStackPtr[i] = 0;       
        }
        // total ready callbacks in `readyList`
        nodes_Num.store(0);         
        for(int i=0; i<THREAD_MAX_NUM; i++){
            // Init. local `cStack` for each registered thread.
            global_cStack[i] = &local_cStack[i]; 
        }
    }

    // Only used to init. nodes in the `readyList`
    bool first_insert(uint32_t chain_idx, uint8_t cb_type, uintptr_t val) {
        if(dummyHead[chain_idx] == NULL){
            dummyHead[chain_idx]            = (lock_free_Node*)malloc(sizeof(lock_free_Node));
            dummyHead[chain_idx]->val       = NIL;
            dummyHead[chain_idx]->child     = NULL;
            dummyHead[chain_idx]->cb_type   = DUMMY_HEAD;
            chainNum++;
            cStack* temp_Stack = &local_cStack[chain_idx+10];
            temp_Stack->mark   = 0;
            for(int i=0; i<CHAIN_NODE_MAX; ++i){
                temp_Stack->counter[i]=0;
            }
            gStackPtr[chain_idx].store((uintptr_t)temp_Stack);
        }
        pair<int,int>* temp_p = (pair<int,int>*)malloc(sizeof(pair<int,int>));
        lock_free_Node* temp  = (lock_free_Node*)malloc(sizeof(lock_free_Node));
        temp->val             = val;
        temp->cb_type         = cb_type;
        (*temp_p).first       = chain_idx;      // chain index, start from 1
        (*temp_p).second      = 2;              // node index, start from 2
        ValToCoord[val]       = temp_p;
        lock_free_Node* p     = dummyHead[chain_idx]->child;
        while(p != NULL){
            (*ValToCoord[p->val]).second++;
            p = p->child;
        }
        nodeNum[chain_idx]++;
        ValToNode[val]  = temp;
        temp->child     = dummyHead[chain_idx]->child;
        dummyHead[chain_idx]->child = temp;
        return true;
    }

    bool insert(uintptr_t val,uint32_t thread_id){
        uintptr_t S_old, S_desired;
        // Get local `cStack`
        cStack* S_new = TID_to_cStack[(thread_id)];
        // Callback To Coord
        int Ic = ValToCoord[val]->first;    // chain index
        int In = ValToCoord[val]->second;   // node index
        while(true){
            int mask = SETBIT(In);
            uint32_t adpMark = 0;

            // Spin to ensure there is no on-going removal operations.
            while(ISMARKED(gStackPtr[Ic].load(),RM));
            S_old = gStackPtr[Ic].fetch_or(IN | mask);
            S_desired = SETMARK(S_old, IN | mask);

            // Try to adopt other insertions.
            if(ISMARKED(S_old, IN)){
                adpMark = ADPMARK(S_old);
            }
            S_old = CLEARALL(S_old);
            // Copy global `cStack` to local, and update local `cStack` copy
            (*S_new) = *((cStack*)S_old);
            S_new->counter[In]++;
            S_new->mark = SETMARK(S_new->mark, mask);
            if(adpMark){
                for(int i=2; i<=nodeNum[Ic]+2; i++){
                    if(ISMARKED(adpMark, SETBIT(i))){
                        S_new->counter[i]++;
                        S_new->mark = SETMARK(S_new->mark, SETBIT(i));
                    }
                }
            }

            // Commit changes
            if(gStackPtr[Ic].compare_exchange_strong(S_desired,(uintptr_t)S_new)){
                TID_to_cStack[(*(uint32_t*)&thread_id)] = (cStack *)S_old;
                if(nodes_Num.load()==0){
                    nodes_Num.store(nodes_Num+1);
                    pthread_mutex_lock(&MDList_mutex);
                    pthread_cond_signal(&MDList_not_empty);
                    pthread_mutex_unlock(&MDList_mutex);
                    return true;
                }
                nodes_Num.store(nodes_Num+1);
                return true;
            }else if (!ISMARKED(gStackPtr[Ic].load(), IN)){
                if(nodes_Num.load()==0){
                    nodes_Num.store(nodes_Num+1);
                    pthread_mutex_lock(&MDList_mutex);
                    pthread_cond_signal(&MDList_not_empty);
                    pthread_mutex_unlock(&MDList_mutex);
                    return true;
                }
                nodes_Num.store(nodes_Num+1);
                return true;
            }
        }
        return false;
    }

    uintptr_t DeleteMin(uint32_t thread_id){
        uintptr_t callback = EMPTY;
        uintptr_t S_old, S_desired;

        cStack* S_new = TID_to_cStack[(thread_id)];
        int Ic = 1, In = 0;
        uint64_t temp_mark;
        uint32_t adpMark = 0;
        while(Ic <= chainNum){
            temp_mark = ((cStack*)gStackPtr[Ic].load())->mark;
            if(temp_mark == 0){
                Ic++;
            }else{
                while(ISMARKED(gStackPtr[Ic].load(),RM));
                // Ensure there are ready callbacks in this chain.
                temp_mark = ((cStack*)gStackPtr[Ic].load())->mark;
                if(temp_mark == 0){
                    continue;
                }
                // Set `posterFlag`
                S_old     = gStackPtr[Ic].fetch_or(RM);
                S_desired = SETMARK(S_old, RM);

                // Try to adopt other insertions.
                if(ISMARKED(S_old, IN)){
                    adpMark = ADPMARK(S_old);
                }

                // Copy global `cStack` to local, and update local `cStack` copy
                S_old = CLEARALL(S_old);
                (*S_new) = *((cStack*)S_old);
                if(adpMark){
                    for(int i=2; i<=nodeNum[Ic]+2; i++){
                        if(ISMARKED(adpMark, SETBIT(i))){
                            S_new->counter[i]++;
                            S_new->mark = SETMARK(S_new->mark, SETBIT(i));
                        }
                    }
                }

                // Find the ready callback with the highest priority
                for(In=2; In<=nodeNum[Ic]+2; In++){
                    if(ISMARKED(S_new->mark, SETBIT(In))){
                        break;
                    }
                }
                if(S_new->counter[In] == 1){
                    S_new->mark = CLEARBIT(S_new->mark, In);
                }
                S_new->counter[In]--;

                if(gStackPtr[Ic].compare_exchange_strong(S_desired,(uintptr_t)S_new)){
                    TID_to_cStack[(thread_id)] = (cStack *)S_old;
                    pair<int,int> temp_p(Ic, In);
                    callback = CoordToVal(temp_p);
                    nodes_Num.store(nodes_Num.load() - 1);
                    break;
                }
            }
        }
        return callback;
    }

    uintptr_t CoordToVal(pair<int,int> Coord) {
        lock_free_Node *temp = dummyHead[Coord.first]->child;
        for(int i=2; temp!=NULL && i!=Coord.second; i++){
            temp = temp->child;
        }
        return temp->val;
    }

    int chainNum;
    // the number of nodes for each chain
    int nodeNum[CHAIN_NUM_MAX];  
    // the total number of ready nodes       
    atomic<int> nodes_Num;              
    lock_free_Node* dummyHead[CHAIN_NUM_MAX];
    atomic<uintptr_t> gStackPtr[CHAIN_NUM_MAX];
    // callback to coordinate
    unordered_map<uintptr_t, pair<int,int>* > ValToCoord;
    // callback to node   
    unordered_map<uintptr_t, lock_free_Node* > ValToNode;   
    // Thread ID to `cStack`, each thread has its own local `cStack`
    unordered_map<uint32_t,  cStack *> TID_to_cStack;       
    cStack * global_cStack[THREAD_MAX_NUM];
    cStack local_cStack[CSTACK_MAX_NUM];
};

extern LF_readyList readyList;
extern map<uintptr_t,uintptr_t> sub_correspond_waitable;

bool lockFree_init_MDList();
uintptr_t lockFree_deleteMin(uint32_t thread_id, uint8_t* type);

#endif