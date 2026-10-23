#include <linux/kernel.h>
#include "vhub.h"

static void ls_vhub_ep_trb_init(struct ls_vhub_ep_pool *ep_pool, struct ls_vhub_ep *ep)
{
    ep->trb = &ep_pool->trb_pool[ep->number];
    ep->trb_dma = ep_pool->trb_dma + ep->number * sizeof (struct ls_vhub_ep_trb);
}

int ls_vhub_ep_pool_init(struct ls_vhub *vhub, u8 ep_num)
{
    struct ls_vhub_ep_pool *ep_pool = &vhub->ep_pool;
    struct ls_vhub_ep *ep;
    u16 map_idx;
    u8 mirror = VHUB_EP_MAP_REG_COUNT >> 1;
    u8 dir = 0;

    ep_pool->map_table = vhub->regs + VHUB_EP_MAP_REGS_START - DWC3_GLOBALS_REGS_START;
    ep_pool->ep_num = ep_num;

    ep_pool->eps[0].number = 0;
    ep_pool->eps[0].dev = NULL;
    ep_pool->eps[0].map = NULL;
    ep_pool->eps[0].used = 1;
    ep_pool->eps[0].ctrl = 1;
    ep_pool->eps[0].common = 1;
    ls_vhub_ep_trb_init(ep_pool, &ep_pool->eps[0]);

    ep_pool->eps[1].number = 1;
    ep_pool->eps[1].dev = NULL;
    ep_pool->eps[1].map = NULL;
    ep_pool->eps[1].used = 1;
    ep_pool->eps[1].ctrl = 1;
    ep_pool->eps[1].common = 1;
    ls_vhub_ep_trb_init(ep_pool, &ep_pool->eps[1]);

    for (int i = 2; i < ep_pool->ep_num; i++) {
        dir = i & 1;
        ep = &ep_pool->eps[i];
        ep->used = 0;
        ep->number = i;
        ep->dev = NULL;
        ep->ctrl = 0;
        ep->common = 0;
        ep->resource = 0;
        map_idx = dir ? (((i - 2) >> 1) + mirror) : (i - 2) >> 1;
        /* init map reg */
        ep_pool->map_table[map_idx].value = 0;
        /* ep bind map reg */
        ep->map = &ep_pool->map_table[map_idx];
        ep->vhub = vhub;
        ls_vhub_ep_trb_init(ep_pool, ep);
    }

    return 0;
}

struct ls_vhub_ep *ls_vhub_ep_alloc(struct ls_vhub_ep_pool *ep_pool, u8 dir)
{
    for (int i = 2; i < ep_pool->ep_num; i++) {
        struct ls_vhub_ep *_ep = &ep_pool->eps[i];

        if (dir == (_ep->number & 1) && _ep->used == 0) {
            _ep->used = 1;
            return _ep;
        }
    }

    return NULL;
}

void ls_vhub_ep_free(struct ls_vhub_ep_pool *ep_pool, struct ls_vhub_ep *ep)
{
    (void)ep_pool;
    ep->used = 0;
    ls_vhub_ep_unmap(ep);
}