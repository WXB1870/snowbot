#include "position_receipt.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <thread>
#include <new>
#include <cstdlib>
thread_local bool count_memory=false;thread_local size_t allocations=0,deletions=0;
void* operator new(std::size_t n){if(count_memory)++allocations;if(auto p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void operator delete(void* p)noexcept{if(count_memory&&p)++deletions;std::free(p);}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete[](void* p)noexcept{::operator delete(p);}
void operator delete(void* p,std::size_t)noexcept{::operator delete(p);}
void operator delete[](void* p,std::size_t)noexcept{::operator delete(p);}
using namespace position_session;
int main(){
 ReceiptQueue queue;AdmissionReceipt receipt;
 assert(!queue.pop(receipt));
 count_memory=true;
 for(unsigned k=1;k<=7;++k)assert(queue.push(9,k,3,100+k,k*1000,k%2,"reason"));
 assert(!queue.push(9,8,3,108,8000,true,"overflow"));assert(queue.dropped()==1);
 for(unsigned k=1;k<=7;++k){assert(queue.pop(receipt));assert(receipt.session==9&&receipt.sequence==k&&receipt.sample==100+k&&receipt.decided_ros==k*1000&&receipt.accepted==bool(k%2));assert(std::strcmp(receipt.reason.data(),"reason")==0);}
 const char long_reason[200]="xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
 assert(queue.push(1,1,1,1,1,false,long_reason));assert(queue.pop(receipt));assert(std::strlen(receipt.reason.data())==159);
 assert(queue.push(1,2,1,1,2,true,nullptr));assert(queue.pop(receipt)&&receipt.reason[0]=='\0');
 count_memory=false;assert(allocations==0&&deletions==0);
 std::thread producer([&]{for(uint64_t k=1;k<=50000;++k)while(!queue.push(7,k,11,k,static_cast<int64_t>(k),true,"accepted"))std::this_thread::yield();});
 for(uint64_t k=1;k<=50000;++k){while(!queue.pop(receipt))std::this_thread::yield();assert(receipt.sequence==k&&receipt.sample==k&&receipt.decided_ros==static_cast<int64_t>(k)&&receipt.boot==11&&receipt.accepted);}
 producer.join();assert(!queue.pop(receipt));
 std::cout<<"PASS bounded receipt queue, overflow fallback, order, fixed reason, 50000 threaded handoffs, zero allocation/deletion\n";
}
