#include <linux/lock_free_fifo.h>


#define GET_WR_IDX(fifo) ((fifo)->wr_idx>=(fifo)->length?(fifo)->wr_idx - (fifo)->length:(fifo)->wr_idx)
#define GET_RD_IDX(fifo) ((fifo)->rd_idx>=(fifo)->length?(fifo)->rd_idx - (fifo)->length:(fifo)->rd_idx)

bool lf_fifo_data_put(struct lf_fifo_env *ptr,void *buf_va, void *data)
{
    if(lf_fifo_full(ptr)) {
        return false;
    } else {
        memcpy(buf_va + ptr->item_size * GET_WR_IDX(ptr), data, ptr->item_size);
        if(ptr->wr_idx + 1 == 2*ptr->length) {
            ptr->wr_idx = 0;
        } else {
            ptr->wr_idx = ptr->wr_idx + 1;
        }
        return true;
    }
}
EXPORT_SYMBOL(lf_fifo_data_put);

bool lf_fifo_data_peek(struct lf_fifo_env *ptr,void *buf_va,void **data)
{
    if(lf_fifo_empty(ptr)) {
        return false;
    } else {
        *data = buf_va + ptr->item_size * GET_RD_IDX(ptr);
        return true;
    }
}
EXPORT_SYMBOL(lf_fifo_data_peek);

void lf_fifo_data_drop(struct lf_fifo_env *ptr)
{
    if (ptr->rd_idx + 1 == 2 * ptr->length) {
        ptr->rd_idx = 0;
    } else {
        ptr->rd_idx = ptr->rd_idx + 1;
    }
}
EXPORT_SYMBOL(lf_fifo_data_drop);

bool lf_fifo_data_get(struct lf_fifo_env *ptr,void *buf_va,void *data)
{
    void *fifo_data_ptr;
    if(lf_fifo_data_peek(ptr,buf_va,&fifo_data_ptr)) {
        memcpy(data,fifo_data_ptr,ptr->item_size);
        lf_fifo_data_drop(ptr);
        return true;
    }else {
        return false;
    }
}
EXPORT_SYMBOL(lf_fifo_data_get);


u16 lf_fifo_data_amount(struct lf_fifo_env *ptr)
{
    if(ptr->wr_idx>=ptr->rd_idx)
    {
        return ptr->wr_idx-ptr->rd_idx;
    }else
    {
        return 2*ptr->length+ptr->wr_idx-ptr->rd_idx;
    }
}
EXPORT_SYMBOL(lf_fifo_data_amount);
MODULE_DESCRIPTION("lock free fifo for linkedsemi ipi");
MODULE_LICENSE("GPL");