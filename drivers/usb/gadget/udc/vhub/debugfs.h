#ifndef __VHUB_DEBUG_H__
#define __VHUB_DEBUG_H__

struct ls_vhub;
struct ls_vhub_dev;

#ifdef CONFIG_DEBUG_FS
int  ls_vhub_debugfs_init(struct ls_vhub *vhub);
void ls_vhub_debugfs_exit(struct ls_vhub *vhub);
int  ls_vhub_dev_debugfs_init(struct ls_vhub_dev *dev);
void ls_vhub_dev_debugfs_exit(struct ls_vhub_dev *dev);
#else
static inline int  ls_vhub_debugfs_init(struct ls_vhub *vhub) {return 0;}
static inline void  ls_vhub_debugfs_exit(struct ls_vhub *vhub) {return 0;}
static inline int  ls_vhub_dev_debugfs_init(struct ls_vhub *vhub) {return 0;}
static inline void  ls_vhub_dev_debugfs_exit(struct ls_vhub *vhub) {return 0;}
#endif

#endif //__VHUB_DEBUG_H__
