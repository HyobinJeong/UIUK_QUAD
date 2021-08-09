#ifndef RBCOMMON_H
#define RBCOMMON_H

#define RB_UNUSED(x) (void)(x)

#ifndef pthread_attr_t
#include <pthread.h>
#endif

inline bool setThreadAffinity(pthread_attr_t &attr, int cpu_id){

    cpu_set_t cpuset;

    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    return pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
}


inline int pthread_create_with_affinity(void* (*t_handler)(void *), int t_cpu_no, const char* t_name, pthread_t &thread_nrt, void* arg){
    pthread_attr_t attr;
    cpu_set_t cpuset;
    int err;
    int ret;

    // initialized with default attributes
    err = pthread_attr_init(&attr);
    if (err != 0) {
       return err;
    }
    // set cpu ID
    if (t_cpu_no >= 0) {
        CPU_ZERO(&cpuset);
        CPU_SET(t_cpu_no, &cpuset);
        err = pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset);
        if (err != 0){
          return err;
        }
    }
    // create a RT thread
    ret = pthread_create(&thread_nrt, &attr, t_handler, arg);
    if (ret != 0) {
        err = pthread_attr_destroy(&attr);
        return err;
    }


    err = pthread_setname_np(thread_nrt, t_name);
    if (err != 0) {
        return err;
    }


    return ret;
}



#endif // RBCOMMON_H
