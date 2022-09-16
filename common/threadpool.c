/*****************************************************************************
 * threadpool.c: thread pooling
 *****************************************************************************
 * Copyright (C) 2010-2022 x264 project
 *
 * Authors: Steven Walters <kemuri9@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *
 * This program is also available under a commercial proprietary license.
 * For more information, contact us at licensing@x264.com.
 *****************************************************************************/

#include "common.h"

/*
* 线程池类似于内存池，在一开始创建时就创建一定数量的线程，当有任务需要处理时，就将该任务丢进线程池中让某个线程来处理，而不是来一个任务就创建一个线程并启动线程工作。
* 使用线程池的好处在于不需要频繁地创建线程，因为创建线程（和销毁线程）是一个会耗费资源的操作，相对而言初始化就创建好线程要比动态创建内存对性能影响更小。
* 其次，线程池可以更好地将线程管理起来，对外提供简单的接口，内部完成对线程的调度。
*/

typedef struct
{
    void *(*func)(void *);
    void *arg;
    void *ret;
} x264_threadpool_job_t;

struct x264_threadpool_t
{
    volatile int   exit;
    int            threads;
    x264_pthread_t *thread_handle;

    /* requires a synchronized list structure and associated methods,
       so use what is already implemented for frames */
    // job理解为任务队列，3个任务队列分别为：未开始运行的（未占用线程）、正在运行的（正在占用线程）、运行完毕的（释放线程）
    x264_sync_frame_list_t uninit; /* list of jobs that are awaiting use */
    x264_sync_frame_list_t run;    /* list of jobs that are queued for processing by the pool */
    x264_sync_frame_list_t done;   /* list of jobs that have finished processing */
};

REALIGN_STACK static void *threadpool_thread( x264_threadpool_t *pool )
{
    while( !pool->exit )
    {
        x264_threadpool_job_t *job = NULL;
        x264_pthread_mutex_lock( &pool->run.mutex );
        while( !pool->exit && !pool->run.i_size )
            x264_pthread_cond_wait( &pool->run.cv_fill, &pool->run.mutex );
        if( pool->run.i_size )
        {
            job = (void*)x264_frame_shift( pool->run.list );
            pool->run.i_size--;
        }
        x264_pthread_mutex_unlock( &pool->run.mutex );
        if( !job )
            continue;
        job->ret = job->func( job->arg );
        x264_sync_frame_list_push( &pool->done, (void*)job );
    }
    return NULL;
}

// 初始化线程池
int x264_threadpool_init( x264_threadpool_t **p_pool, int threads )
{
    if( threads <= 0 )
        return -1;

    if( x264_threading_init() < 0 )
        return -1;

    x264_threadpool_t *pool;
    CHECKED_MALLOCZERO( pool, sizeof(x264_threadpool_t) );
    *p_pool = pool;

    pool->threads   = threads;

    // 创建指定线程数threads的线程池
    CHECKED_MALLOC( pool->thread_handle, pool->threads * sizeof(x264_pthread_t) );

    if( x264_sync_frame_list_init( &pool->uninit, pool->threads ) ||
        x264_sync_frame_list_init( &pool->run, pool->threads ) ||
        x264_sync_frame_list_init( &pool->done, pool->threads ) )
        goto fail;

    for( int i = 0; i < pool->threads; i++ )
    {
       x264_threadpool_job_t *job;
       CHECKED_MALLOC( job, sizeof(x264_threadpool_job_t) );
       // 每一个job对应一个线程，初始化时job为unint，此时尚未占用线程
       x264_sync_frame_list_push( &pool->uninit, (void*)job );
    }
    for( int i = 0; i < pool->threads; i++ )
        if( x264_pthread_create( pool->thread_handle+i, NULL, (void*)threadpool_thread, pool ) )
            goto fail;

    return 0;
fail:
    return -1;
}

// 线程池中有空余线程时，从任务队列中拿出一个任务来执行
void x264_threadpool_run( x264_threadpool_t *pool, void *(*func)(void *), void *arg )
{
    // 从尚未被执行的任务队列（unint）中拿出一个任务
    x264_threadpool_job_t *job = (void*)x264_sync_frame_list_pop( &pool->uninit );
    job->func = func;
    job->arg  = arg;
    // 添加一个任务到正在执行的队列（run）中
    x264_sync_frame_list_push( &pool->run, (void*)job );
}

// 线程池中没有线程可以使用了，此时需要等到正在执行的任务执行完毕后释放线程
void *x264_threadpool_wait( x264_threadpool_t *pool, void *arg )
{
    // 等待正在运行的任务执行完后，释放对应的线程
    x264_pthread_mutex_lock( &pool->done.mutex );
    while( 1 )
    {
        for( int i = 0; i < pool->done.i_size; i++ )
            if( ((x264_threadpool_job_t*)pool->done.list[i])->arg == arg )
            {
                x264_threadpool_job_t *job = (void*)x264_frame_shift( pool->done.list+i );
                pool->done.i_size--;
                x264_pthread_mutex_unlock( &pool->done.mutex );

                void *ret = job->ret;
                // 为什么又要把这个job放到unint队列中呢？
                // 因为任务与线程绑定在一起，该任务执行完后，其线程可以供其他任务使用了，所以在unint队列中就多了一个线程
                x264_sync_frame_list_push( &pool->uninit, (void*)job );
                return ret;
            }

        x264_pthread_cond_wait( &pool->done.cv_fill, &pool->done.mutex );
    }
}

// 删除任务队列
static void threadpool_list_delete( x264_sync_frame_list_t *slist )
{
    for( int i = 0; slist->list[i]; i++ )
    {
        x264_free( slist->list[i] );
        slist->list[i] = NULL;
    }
    x264_sync_frame_list_delete( slist );
}

// 删除线程池
void x264_threadpool_delete( x264_threadpool_t *pool )
{
    x264_pthread_mutex_lock( &pool->run.mutex );
    pool->exit = 1;
    x264_pthread_cond_broadcast( &pool->run.cv_fill );
    x264_pthread_mutex_unlock( &pool->run.mutex );
    for( int i = 0; i < pool->threads; i++ )
        x264_pthread_join( pool->thread_handle[i], NULL );

    threadpool_list_delete( &pool->uninit );
    threadpool_list_delete( &pool->run );
    threadpool_list_delete( &pool->done );
    x264_free( pool->thread_handle );
    x264_free( pool );
}
