#ifndef LOCK_FREE_FIFO_H
#define LOCK_FREE_FIFO_H
#include <linux/kernel.h>
#include <linux/module.h>

struct lf_fifo_env {
    u32 buf;
    u16 rd_idx;
    u16 wr_idx;
    u16 length;
    u16 item_size;
};

static inline bool lf_fifo_full(struct lf_fifo_env *env)
{
    return abs(env->wr_idx-env->rd_idx) == env->length;
}

static inline bool lf_fifo_empty(struct lf_fifo_env *env)
{
    return env->wr_idx == env->rd_idx;
}

u16 lf_fifo_data_amount(struct lf_fifo_env *ptr);

bool lf_fifo_data_put(struct lf_fifo_env *ptr,void *buf_va, void *data);

bool lf_fifo_data_peek(struct lf_fifo_env *ptr,void *buf_va,void **data);

void lf_fifo_data_drop(struct lf_fifo_env *ptr);

bool lf_fifo_data_get(struct lf_fifo_env *ptr,void *buf_va,void *data);


#endif
