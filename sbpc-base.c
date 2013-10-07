/* sbpc-base.c */

#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/version.h>

#include <asm/io.h>

#include "simbank/simcard-def.h"
#include "simbank/version.h"

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@gmail.com>");
MODULE_DESCRIPTION("Polygator Linux module for SBPC device");
MODULE_LICENSE("GPL");

static int sbpc_major = 0;
module_param(sbpc_major, int, 0);
MODULE_PARM_DESC(sbpc_major, "Major number for Polygator SBPC device");

static int sim_start = 0;
module_param(sim_start, int, 0);
MODULE_PARM_DESC(sim_start, "Start SIM Number");

static int sim_end = 199;
module_param(sim_end, int, 0);
MODULE_PARM_DESC(sim_end, "End SIM Number");

#define SIM_MONITOR
#ifdef SIM_MONITOR
static int sim_mon = -1;
module_param(sim_mon, int, 0);
MODULE_PARM_DESC(sim_mon, "Monitor SIM Number");
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
	#define CLASS_DEV_CREATE(_class, _devt, _device, _name) device_create(_class, _device, _devt, NULL, "%s", _name)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	#define CLASS_DEV_CREATE(_class, _devt, _device, _name) device_create(_class, _device, _devt, _name)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
	#define CLASS_DEV_CREATE(_class, _devt, _device, _name) class_device_create(_class, NULL, _devt, _device, _name)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)
	#define CLASS_DEV_CREATE(_class, _devt, _device, _name) class_device_create(_class, _devt, _device, _name)
#else
	#define CLASS_DEV_CREATE(_class, _devt, _device, _name) class_simple_device_add(_class, _devt, _device, _name)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	#define CLASS_DEV_DESTROY(_class, _devt) device_destroy(_class, _devt)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)
	#define CLASS_DEV_DESTROY(_class, _devt) class_device_destroy(_class, _devt)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,9)
	#define CLASS_DEV_DESTROY(_class, _devt) class_simple_device_remove(_devt)
#else
	#define CLASS_DEV_DESTROY(_class, _devt) class_simple_device_remove(_class, _devt)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)
	static struct class *sbpc_class = NULL;
#else
	static struct class_simple *sbpc_class = NULL;
	#define class_create(_a, _b) class_simple_create(_a, _b)
	#define class_destroy(_a) class_simple_destroy(_a)
#endif

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "sbpc-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "sbpc-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

#define SBPC_DEVICE_MAXCOUNT 256
#define SBPC_BOARD_SIM_MAXCOUNT 200

#define SBPC_SIM_DATA_BUFF_SIZE 1024

#define SBPC_SIM_DATA_DEV_SIZE 32

#define SBPC_BOARD_CTRL_BASE		0x00000
#define SBPC_BOARD_CTRL_SIM_MONITOR	0x04
#define SBPC_BOARD_CTRL_REG_0		0x0c
#define SBPC_BOARD_CTRL_REG_1		0x10
#define SBPC_BOARD_CTRL_ROM_DATA	0x14
#define SBPC_BOARD_CTRL_ID_DATA		0x18
#define SBPC_BOARD_CTRL_RESET		0x1c

#define SBPC_SIM_CTRL_BASE			0x20000
#define SBPC_SIM_CTRL_GENERAL		0x00
#define SBPC_SIM_CTRL_RX_BUF_WP		0x24	// 0x44
#define SBPC_SIM_CTRL_RX_BUF_FULL	0x28	// 0x48
#define SBPC_SIM_CTRL_RX_BUF_OF		0x2c	// 0x4c
#define SBPC_SIM_CTRL_RX_BUF_RP		0x20	// 0x40
#define SBPC_SIM_CTRL_TX_BUF_WP		0x44	// 0x84
#define SBPC_SIM_CTRL_TX_BUF_FULL	0x48	// 0x88
#define SBPC_SIM_CTRL_TX_BUF_RP		0x4c	// 0x8c

#define SBPC_SIM_DATA_RX_BASE		0x80000
#define SBPC_SIM_DATA_TX_BASE		0xc0000

struct sbpc_simcard_device {
	spinlock_t lock;
	size_t usage;

	struct sbpc_board *board;
	size_t position;

	union {
		struct {
			u_int8_t reset:1;
			u_int8_t speed:2;
			u_int8_t led:1;
			u_int8_t xmit:1;
			u_int8_t res:3;
		} __attribute__((packed)) bits;
		u_int8_t full;
	} __attribute__((packed)) control;

	wait_queue_head_t poll_waitq;
	wait_queue_head_t read_waitq;
	wait_queue_head_t write_waitq;

	u_int8_t read_buf[SBPC_SIM_DATA_BUFF_SIZE];
	size_t read_head;
	size_t read_tail;
	size_t read_count;
	size_t read_overflow;

	u_int8_t write_buf[SBPC_SIM_DATA_BUFF_SIZE];
	size_t write_head;
	size_t write_tail;
	size_t write_count;
	size_t write_overflow;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	struct device *device;
#else
	struct class_device *device;
#endif
	int devno;
	struct cdev cdev;
	int cdev_req;
};

struct sbpc_board {
	u_int16_t id;
	char rom[256];

	unsigned long ioport_base_addr;
	int ioport_req;
	void __iomem *iomem_base_ptr;
	int iomem_req;

	struct sbpc_simcard_device *simcards[SBPC_BOARD_SIM_MAXCOUNT];

	struct timer_list poll_timer;
};

static void sbpc_poll_proc(unsigned long data)
{
	size_t i, j;

	struct rx_buf_regs {
		u_int32_t rp;
		u_int32_t wp;
		u_int32_t fl;
		u_int32_t of;
	} __attribute__((packed)) rx_buf_regs;

	struct tx_buf_regs {
		u_int32_t wp;
		u_int32_t fl;
		u_int32_t rp;
	} __attribute__((packed)) tx_buf_regs;

	u_int32_t sim_offset;

	size_t chunk;

	struct sbpc_simcard_device *sim;
	struct sbpc_board *board = (struct sbpc_board *)data;

	for (i = sim_start; i <= sim_end; i++) {
		if ((sim = board->simcards[i])) {
			// set SIM offset
			sim_offset = i % 100;
			if (i >= 100) {
				sim_offset |= 0x80;
			}
			spin_lock(&sim->lock);
			if (sim->control.bits.reset) {
				// rx data
				memcpy_fromio(&rx_buf_regs, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_RX_BUF_RP, sizeof(struct rx_buf_regs));
				while (1) {
					if (rx_buf_regs.of) {
#ifdef SIM_MONITOR
						if ((sim_mon > -1) && (sim_mon == i)) {
							verbose("%03lu: rx_buf OVERFLOW wr=%02x rp=%02x\n", (unsigned long int)i, rx_buf_regs.wp, rx_buf_regs.rp);
						}
#endif
						sim->read_overflow = 1;
						break;
					} else if (sim->read_count == SBPC_SIM_DATA_BUFF_SIZE) {
						break;
					} else if (rx_buf_regs.wp > rx_buf_regs.rp) {
#ifdef SIM_MONITOR
						if ((sim_mon > -1) && (sim_mon == i)) {
							verbose("%03lu: rx_buf: wr=%02x rp=%02x\n", (unsigned long int)i, rx_buf_regs.wp, rx_buf_regs.rp);
						}
#endif
						while (rx_buf_regs.wp > rx_buf_regs.rp) {
							if (sim->read_head < sim->read_tail) {
								chunk = min((size_t)(sim->read_tail - sim->read_head), (size_t)(rx_buf_regs.wp - rx_buf_regs.rp));
								for (j = 0; j < chunk; j++) {
									sim->read_buf[sim->read_head + j] = ioread8(board->iomem_base_ptr + SBPC_SIM_DATA_RX_BASE + (sim_offset << 7) + (rx_buf_regs.rp + j) * 4);
#ifdef SIM_MONITOR
									if ((sim_mon > -1) && (sim_mon == i)) {
										verbose("%03lu: addr=%08x rx %02x\n", (unsigned long int)i, (unsigned int)(SBPC_SIM_DATA_RX_BASE + (sim_offset << 7) + (rx_buf_regs.rp + j) * 4), sim->read_buf[sim->read_head + j]);
									}
#endif
								}
								// update device pointers
								rx_buf_regs.rp += chunk;
								if (rx_buf_regs.rp == SBPC_SIM_DATA_DEV_SIZE) {
									rx_buf_regs.rp = 0;
								}
								iowrite8(rx_buf_regs.rp, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_RX_BUF_RP);
								rx_buf_regs.fl = 0;
								// update buffer pointers
								sim->read_head += chunk;
								if (sim->read_head == SBPC_SIM_DATA_BUFF_SIZE) {
									sim->read_head = 0;
								}
								sim->read_count += chunk;
							} else if ((sim->read_head > sim->read_tail) || (sim->read_count == 0)) {
								chunk = min((size_t)(SBPC_SIM_DATA_BUFF_SIZE - sim->read_head), (size_t)(rx_buf_regs.wp - rx_buf_regs.rp));
								for (j = 0; j < chunk; j++) {
									sim->read_buf[sim->read_head + j] = ioread8(board->iomem_base_ptr + SBPC_SIM_DATA_RX_BASE + (sim_offset << 7) + (rx_buf_regs.rp + j) * 4);
#ifdef SIM_MONITOR
									if ((sim_mon > -1) && (sim_mon == i)) {
										verbose("%03lu: addr=%08x rx %02x\n", (unsigned long int)i, (unsigned int)(SBPC_SIM_DATA_RX_BASE + (sim_offset << 7) + (rx_buf_regs.rp + j) * 4), sim->read_buf[sim->read_head + j]);
									}
#endif
								}
								// update device pointers
								rx_buf_regs.rp += chunk;
								if (rx_buf_regs.rp == SBPC_SIM_DATA_DEV_SIZE) {
									rx_buf_regs.rp = 0;
								}
								iowrite8(rx_buf_regs.rp, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_RX_BUF_RP);
								rx_buf_regs.fl = 0;
								// update buffer pointers
								sim->read_head += chunk;
								if (sim->read_head == SBPC_SIM_DATA_BUFF_SIZE) {
									sim->read_head = 0;
								}
								sim->read_count += chunk;
							} else {
								break;
							}
						}
					} else if ((rx_buf_regs.wp < rx_buf_regs.rp) || (rx_buf_regs.fl)) {
#ifdef SIM_MONITOR
						if ((sim_mon > -1) && (sim_mon == i)) {
							verbose("%03lu: rx_buf: wr=%02x rp=%02x\n", (unsigned long int)i, rx_buf_regs.wp, rx_buf_regs.rp);
						}
#endif
						while ((rx_buf_regs.wp < rx_buf_regs.rp) || (rx_buf_regs.fl)) {
							if (sim->read_head < sim->read_tail) {
								chunk = min((size_t)(sim->read_tail - sim->read_head), (size_t)(SBPC_SIM_DATA_DEV_SIZE - rx_buf_regs.rp));
								for (j = 0; j < chunk; j++) {
									sim->read_buf[sim->read_head + j] = ioread8(board->iomem_base_ptr + SBPC_SIM_DATA_RX_BASE + (sim_offset << 7) + (rx_buf_regs.rp + j) * 4);
#ifdef SIM_MONITOR
									if ((sim_mon > -1) && (sim_mon == i)) {
										verbose("%03lu: addr=%08x rx %02x\n", (unsigned long int)i, (unsigned int)(SBPC_SIM_DATA_RX_BASE + (sim_offset << 7) + (rx_buf_regs.rp + j) * 4), sim->read_buf[sim->read_head + j]);
									}
#endif
								}
								// update device pointers
								rx_buf_regs.rp += chunk;
								if (rx_buf_regs.rp == SBPC_SIM_DATA_DEV_SIZE) {
									rx_buf_regs.rp = 0;
								}
								iowrite8(rx_buf_regs.rp, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_RX_BUF_RP);
								rx_buf_regs.fl = 0;
								// update buffer pointers
								sim->read_head += chunk;
								if (sim->read_head == SBPC_SIM_DATA_BUFF_SIZE) {
									sim->read_head = 0;
								}
								sim->read_count += chunk;
							} else if ((sim->read_head > sim->read_tail) || (sim->read_count == 0)) {
								chunk = min((size_t)(SBPC_SIM_DATA_BUFF_SIZE - sim->read_head), (size_t)(SBPC_SIM_DATA_DEV_SIZE - rx_buf_regs.rp));
								for (j = 0; j < chunk; j++) {
									sim->read_buf[sim->read_head + j] = ioread8(board->iomem_base_ptr + SBPC_SIM_DATA_RX_BASE + (sim_offset << 7) + (rx_buf_regs.rp + j) * 4);
#ifdef SIM_MONITOR
									if ((sim_mon > -1) && (sim_mon == i)) {
										verbose("%03lu: addr=%08x rx %02x\n", (unsigned long int)i, (unsigned int)(SBPC_SIM_DATA_RX_BASE + (sim_offset << 7) + (rx_buf_regs.rp + j) * 4), sim->read_buf[sim->read_head + j]);
									}
#endif
								}
								// update device pointers
								rx_buf_regs.rp += chunk;
								if (rx_buf_regs.rp == SBPC_SIM_DATA_DEV_SIZE) {
									rx_buf_regs.rp = 0;
								}
								iowrite8(rx_buf_regs.rp, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_RX_BUF_RP);
								rx_buf_regs.fl = 0;
								// update buffer pointers
								sim->read_head += chunk;
								if (sim->read_head == SBPC_SIM_DATA_BUFF_SIZE) {
									sim->read_head = 0;
								}
								sim->read_count += chunk;
							} else {
								break;
							}
						}
					} else {
						break;
					}
				}
				if (sim->read_count) {
					wake_up_interruptible(&sim->read_waitq);
					wake_up_interruptible(&sim->poll_waitq);
				}
#if 0
				if (sim->read_overflow) {
					wake_up_interruptible(&sim->poll_waitq);
				}
#endif
				// tx data
				if (sim->write_count) {
					memcpy_fromio(&tx_buf_regs, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_TX_BUF_WP, sizeof(struct tx_buf_regs));
					while (1) {
						if (tx_buf_regs.fl) {
							break;
						} else if (sim->write_head > sim->write_tail) {
							while (sim->write_head > sim->write_tail) {
								if (tx_buf_regs.wp < tx_buf_regs.rp) {
#ifdef SIM_MONITOR
									if ((sim_mon > -1) && (sim_mon == i)) {
										verbose("%03lu: tx_buf: wr=%02x rp=%02x fl=%u\n", (unsigned long int)i, tx_buf_regs.wp, tx_buf_regs.rp, tx_buf_regs.fl);
									}
#endif
									// get chunk length
									chunk = min((size_t)(tx_buf_regs.rp - tx_buf_regs.wp), (size_t)(sim->write_head - sim->write_tail));
									// enable xmit buffer
									sim->control.bits.xmit = 1;
									iowrite8(sim->control.full, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_GENERAL);
									sim->control.bits.xmit = 0;
									// copy data to xmit buffer
									for (j = 0; j < chunk; j++) {
										iowrite8(sim->write_buf[sim->write_tail + j], board->iomem_base_ptr + SBPC_SIM_DATA_TX_BASE + (sim_offset << 7) + (tx_buf_regs.wp + j) * 4);
#ifdef SIM_MONITOR
										if ((sim_mon > -1) && (sim_mon == i)) {
											verbose("%03lu: tx %02x\n", (unsigned long int)i, sim->write_buf[sim->write_tail + j]);
										}
#endif
									}
									// update device pointers
									tx_buf_regs.wp += chunk;
									if (tx_buf_regs.wp == SBPC_SIM_DATA_DEV_SIZE) {
										tx_buf_regs.wp = 0;
									}
									if (tx_buf_regs.wp == tx_buf_regs.rp) {
										tx_buf_regs.fl = 1;
										iowrite8(tx_buf_regs.fl, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_TX_BUF_FULL);
									}
									iowrite8(tx_buf_regs.wp, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_TX_BUF_WP);
									// update buffer pointers
									sim->write_tail += chunk;
									if (sim->write_tail == SBPC_SIM_DATA_BUFF_SIZE) {
										sim->write_tail = 0;
									}
									sim->write_count -= chunk;
								} else if ((tx_buf_regs.wp > tx_buf_regs.rp) || (tx_buf_regs.fl == 0)) {
#ifdef SIM_MONITOR
									if ((sim_mon > -1) && (sim_mon == i)) {
										verbose("%03lu: tx_buf: wr=%02x rp=%02x fl=%u\n", (unsigned long int)i, tx_buf_regs.wp, tx_buf_regs.rp, tx_buf_regs.fl);
									}
#endif
									// get chunk length
									chunk = min((size_t)(SBPC_SIM_DATA_DEV_SIZE - tx_buf_regs.wp), (size_t)(sim->write_head - sim->write_tail));
									// enable xmit buffer
									sim->control.bits.xmit = 1;
									iowrite8(sim->control.full, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_GENERAL);
									sim->control.bits.xmit = 0;
									// copy data to xmit buffer
									for (j = 0; j < chunk; j++) {
										iowrite8(sim->write_buf[sim->write_tail + j], board->iomem_base_ptr + SBPC_SIM_DATA_TX_BASE + (sim_offset << 7) + (tx_buf_regs.wp + j) * 4);
#ifdef SIM_MONITOR
										if ((sim_mon > -1) && (sim_mon == i)) {
											verbose("%03lu: tx %02x\n", (unsigned long int)i, sim->write_buf[sim->write_tail + j]);
										}
#endif
									}
									// update device pointers
									tx_buf_regs.wp += chunk;
									if (tx_buf_regs.wp == SBPC_SIM_DATA_DEV_SIZE) {
										tx_buf_regs.wp = 0;
									}
									if (tx_buf_regs.wp == tx_buf_regs.rp) {
										tx_buf_regs.fl = 1;
										iowrite8(tx_buf_regs.fl, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_TX_BUF_FULL);
									}
									iowrite8(tx_buf_regs.wp, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_TX_BUF_WP);
									// update buffer pointers
									sim->write_tail += chunk;
									if (sim->write_tail == SBPC_SIM_DATA_BUFF_SIZE) {
										sim->write_tail = 0;
									}
									sim->write_count -= chunk;
								} else {
									break;
								}
							}
						} else if ((sim->write_head < sim->write_tail) || (sim->write_count == SBPC_SIM_DATA_BUFF_SIZE)) {
							while ((sim->write_head < sim->write_tail) || (sim->write_count == SBPC_SIM_DATA_BUFF_SIZE)) {
								if (tx_buf_regs.wp < tx_buf_regs.rp) {
#ifdef SIM_MONITOR
									if ((sim_mon > -1) && (sim_mon == i)) {
										verbose("%03lu: tx_buf: wr=%02x rp=%02x fl=%u\n", (unsigned long int)i, tx_buf_regs.wp, tx_buf_regs.rp, tx_buf_regs.fl);
									}
#endif
									// get chunk length
									chunk = min((size_t)(tx_buf_regs.rp - tx_buf_regs.wp), (size_t)(SBPC_SIM_DATA_BUFF_SIZE - sim->write_tail));
									// enable xmit buffer
									sim->control.bits.xmit = 1;
									iowrite8(sim->control.full, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_GENERAL);
									sim->control.bits.xmit = 0;
									// copy data to xmit buffer
									for (j = 0; j < chunk; j++) {
#ifdef SIM_MONITOR
										if ((sim_mon > -1) && (sim_mon == i)) {
											verbose("%03lu: tx %02x\n", (unsigned long int)i, sim->write_buf[sim->write_tail + j]);
										}
#endif
										iowrite8(sim->write_buf[sim->write_tail + j], board->iomem_base_ptr + SBPC_SIM_DATA_TX_BASE + (sim_offset << 7) + (tx_buf_regs.wp + j) * 4);
									}
									// update device pointers
									tx_buf_regs.wp += chunk;
									if (tx_buf_regs.wp == SBPC_SIM_DATA_DEV_SIZE) {
										tx_buf_regs.wp = 0;
								}
									if (tx_buf_regs.wp == tx_buf_regs.rp) {
										tx_buf_regs.fl = 1;
										iowrite8(tx_buf_regs.fl, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_TX_BUF_FULL);
									}
									iowrite8(tx_buf_regs.wp, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_TX_BUF_WP);
									// update buffer pointers
									sim->write_tail += chunk;
									if (sim->write_tail == SBPC_SIM_DATA_BUFF_SIZE) {
										sim->write_tail = 0;
									}
									sim->write_count -= chunk;
								} else if ((tx_buf_regs.wp > tx_buf_regs.rp) || (tx_buf_regs.fl == 0)) {
#ifdef SIM_MONITOR
									if ((sim_mon > -1) && (sim_mon == i)) {
										verbose("%03lu: tx_buf: wr=%02x rp=%02x fl=%u\n", (unsigned long int)i, tx_buf_regs.wp, tx_buf_regs.rp, tx_buf_regs.fl);
									}
#endif
									// get chunk length
									chunk = min((size_t)(SBPC_SIM_DATA_DEV_SIZE - tx_buf_regs.wp), (size_t)(SBPC_SIM_DATA_BUFF_SIZE - sim->write_tail));
									// enable xmit buffer
									sim->control.bits.xmit = 1;
									iowrite8(sim->control.full, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_GENERAL);
									sim->control.bits.xmit = 0;
									// copy data to xmit buffer
									for (j = 0; j < chunk; j++) {
#ifdef SIM_MONITOR
										if ((sim_mon > -1) && (sim_mon == i)) {
											verbose("%03lu: tx %02x\n", (unsigned long int)i, sim->write_buf[sim->write_tail + j]);
										}
#endif
										iowrite8(sim->write_buf[sim->write_tail + j], board->iomem_base_ptr + SBPC_SIM_DATA_TX_BASE + (sim_offset << 7) + (tx_buf_regs.wp + j) * 4);
									}
									// update device pointers
									tx_buf_regs.wp += chunk;
									if (tx_buf_regs.wp == SBPC_SIM_DATA_DEV_SIZE) {
										tx_buf_regs.wp = 0;
									}
									if (tx_buf_regs.wp == tx_buf_regs.rp) {
										tx_buf_regs.fl = 1;
										iowrite8(tx_buf_regs.fl, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_TX_BUF_FULL);
									}
									iowrite8(tx_buf_regs.wp, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_TX_BUF_WP);
									// update buffer pointers
									sim->write_tail += chunk;
									if (sim->write_tail == SBPC_SIM_DATA_BUFF_SIZE) {
										sim->write_tail = 0;
									}
									sim->write_count -= chunk;
								} else {
									break;
								}
							}
						} else {
							break;
						}
					}
				}
			}
			spin_unlock(&sim->lock);
		}
	}

	mod_timer(&board->poll_timer, jiffies + 1);
}

static int sbpc_simcard_open(struct inode *inode, struct file *filp)
{
	size_t usage;
	struct sbpc_simcard_device *sim;

	sim = container_of(inode->i_cdev, struct sbpc_simcard_device, cdev);
	filp->private_data = sim;

	spin_lock_bh(&sim->lock);
	usage = sim->usage++;
	sim->read_count = sim->read_head = sim->read_tail = 0;
	sim->read_overflow = 0;
	sim->write_count = sim->write_head = sim->write_tail = 0;
	sim->write_overflow = 0;
	spin_unlock_bh(&sim->lock);

	if (usage) {
		return -EBUSY;
	} else {
		return 0;
	}
}

static int sbpc_simcard_release(struct inode *inode, struct file *filp)
{
	struct sbpc_simcard_device *sim = filp->private_data;

	spin_lock_bh(&sim->lock);
	sim->usage--;
	spin_unlock_bh(&sim->lock);

	return 0;
}

static ssize_t sbpc_simcard_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	size_t length;
	size_t rest, pos, chunk;
	struct simcard_data data;
	struct sbpc_simcard_device *sim = filp->private_data;

	res = 0;

	spin_lock_bh(&sim->lock);

	for (;;) {
		// check for data present
		if (sim->read_count) {
			break;
		}
		// check for non-block operation
		if (filp->f_flags & O_NONBLOCK) {
			spin_unlock_bh(&sim->lock);
			res = -EAGAIN;
			goto sbpc_simcard_read_end;
		}
		// sleeping
		spin_unlock_bh(&sim->lock);
		if ((res = wait_event_interruptible(sim->read_waitq, sim->read_count))) {
			goto sbpc_simcard_read_end;
		}
		spin_lock_bh(&sim->lock);
	}

	length = sizeof(data.header) + sim->read_count;
	length = min(length, count);
	data.header.type = SIMCARD_CONTAINER_TYPE_DATA;
	data.header.length = length - sizeof(data.header);

	pos = 0;
	rest = data.header.length;
	while (rest) {
		if (sim->read_head > sim->read_tail) {
			chunk = min((size_t)(sim->read_head - sim->read_tail), rest);
			memcpy(data.container.data + pos, sim->read_buf + sim->read_tail, chunk);
			pos += chunk;
			rest -= chunk;
			sim->read_tail += chunk;
			if (sim->read_tail == SBPC_SIM_DATA_BUFF_SIZE) {
				sim->read_tail = 0;
			}
			sim->read_count -= chunk;
		} else if ((sim->read_head < sim->read_tail) || (sim->read_count == SBPC_SIM_DATA_BUFF_SIZE)) {
			chunk = min((size_t)(SBPC_SIM_DATA_BUFF_SIZE - sim->read_tail), rest);
			memcpy(data.container.data + pos, sim->read_buf + sim->read_tail, chunk);
			pos += chunk;
			rest -= chunk;
			sim->read_tail += chunk;
			if (sim->read_tail == SBPC_SIM_DATA_BUFF_SIZE) {
				sim->read_tail = 0;
			}
			sim->read_count -= chunk;
		} else {
			break;
		}
	}

	spin_unlock_bh(&sim->lock);

	if (copy_to_user(buff, &data, length)) {
		res = -EFAULT;
		goto sbpc_simcard_read_end;
	}

	res = length;

sbpc_simcard_read_end:
	return res;
}

static ssize_t sbpc_simcard_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	size_t length;
	size_t rest, pos, chunk;
	void __iomem *mem_ptr;
	struct simcard_data data;

	u_int32_t sim_offset;

	struct sbpc_simcard_device *sim = filp->private_data;

	res = 0;

	// set SIM offset
	sim_offset = sim->position % 100;
	if (sim->position >= 100) {
		sim_offset |= 0x80;
	}

	length = sizeof(struct simcard_data);
	length = min(length, count);
	if (copy_from_user(&data, buff, length)) {
		res = -EFAULT;
		goto sbpc_simcard_write_end;
	}

	spin_lock_bh(&sim->lock);

	switch (data.header.type) {
		case SIMCARD_CONTAINER_TYPE_DATA:
			rest = data.header.length;
			pos = 0;
			while (rest) {
				if (sim->write_head < sim->write_tail) {
					chunk = min((size_t)(sim->write_tail - sim->write_head), rest);
					memcpy(sim->write_buf + sim->write_head, data.container.data + pos, chunk);
					// update source pointers
					rest -= chunk;
					pos += chunk;
					// update buffer pointers
					sim->write_head += chunk;
					if (sim->write_head == SBPC_SIM_DATA_BUFF_SIZE) {
						sim->write_head = 0;
					}
					sim->write_count += chunk;
				} else if ((sim->write_head > sim->write_tail) || (sim->write_count == 0)) {
					chunk = min((size_t)(SBPC_SIM_DATA_BUFF_SIZE - sim->write_head), rest);
					memcpy(sim->write_buf + sim->write_head, data.container.data + pos, chunk);
					// update source pointers
					rest -= chunk;
					pos += chunk;
					// update buffer pointers
					sim->write_head += chunk;
					if (sim->write_head == SBPC_SIM_DATA_BUFF_SIZE) {
						sim->write_head = 0;
					}
					sim->write_count += chunk;
				} else {
					break;
				}
			}
			length -= rest;
			break;
		case SIMCARD_CONTAINER_TYPE_RESET:
			mem_ptr = sim->board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_GENERAL;
			sim->control.bits.reset = data.container.reset & 1;
			iowrite8(sim->control.full, mem_ptr);
			// init buffer pointers
			if (!sim->control.bits.reset) {
				sim->read_count = sim->read_head = sim->read_tail = 0;
				sim->read_overflow = 0;
				sim->write_count = sim->write_head = sim->write_tail = 0;
				sim->write_overflow = 0;
			}
			break;
		case SIMCARD_CONTAINER_TYPE_SPEED:
			mem_ptr = sim->board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_GENERAL;
			switch (data.container.speed & 0xff) {
				case 0x94:
					sim->control.bits.speed = 1;
					break;
				case 0x95:
					sim->control.bits.speed = 2;
					break;
				case 0x96:
					sim->control.bits.speed = 3;
					break;
				default:
					sim->control.bits.speed = 0;
					break;
			}
			iowrite8(sim->control.full, mem_ptr);
			break;
		case SIMCARD_CONTAINER_TYPE_MONITOR:
			mem_ptr = sim->board->iomem_base_ptr + SBPC_BOARD_CTRL_BASE + SBPC_BOARD_CTRL_SIM_MONITOR;
			iowrite8(data.container.monitor, mem_ptr);
			break;
		case SIMCARD_CONTAINER_TYPE_LED:
			mem_ptr = sim->board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_GENERAL;
			sim->control.bits.led = data.container.led & 1;
			iowrite8(sim->control.full, mem_ptr);
			break;
		default:
			spin_unlock_bh(&sim->lock);
			res = -EINVAL;
			goto sbpc_simcard_write_end;
	}

	spin_unlock_bh(&sim->lock);

	res = length;

sbpc_simcard_write_end:
	return res;
}

static unsigned int sbpc_simcard_poll(struct file *filp, struct poll_table_struct *wait_table)
{
	unsigned int res;
	struct sbpc_simcard_device *sim = filp->private_data;

	res = 0;

	poll_wait(filp, &sim->poll_waitq, wait_table);

	spin_lock_bh(&sim->lock);

	if (sim->read_count) {
		res |= POLLIN | POLLRDNORM;
	}

	if (sim->write_count < SBPC_SIM_DATA_BUFF_SIZE) {
		res |= POLLOUT | POLLWRNORM;
	}

	spin_unlock_bh(&sim->lock);

	return res;
}

static struct file_operations sbpc_simcard_fops = {
	.owner		= THIS_MODULE,
	.open		= sbpc_simcard_open,
	.release	= sbpc_simcard_release,
	.read		= sbpc_simcard_read,
	.write		= sbpc_simcard_write,
	.poll		= sbpc_simcard_poll,
};

static struct pci_device_id sbpc_board_id_table[] = {
	{ PCI_DEVICE(0xDEAD, 0xBEEF), .driver_data = 1, },
	{ 0, },
};
MODULE_DEVICE_TABLE(pci, sbpc_board_id_table);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static int sbpc_board_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
#else
static int __devinit sbpc_board_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
#endif
{
	char devname[64];
	u_int16_t tw16_0, tw16_1, tr16_0, tr16_1;
	size_t i;
	int rc;
	u_int32_t sim_offset;
	struct sbpc_board *board;

	// get memory for sbpc board
	if (!(board = kmalloc(sizeof(struct sbpc_board), GFP_KERNEL))) {
		dev_err(&pdev->dev, "can't get memory for struct sbpc_board\n");
		rc = -ENOMEM;
		goto sbpc_board_probe_error;
	}
	memset(board, 0, sizeof(struct sbpc_board));

	rc = pci_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev, "can't enable pci device\n");
		goto sbpc_board_probe_error;
	}

	rc = pci_request_region(pdev, 0, "sbpc");
	if (rc) {
		dev_err(&pdev->dev, "can't request I/O region\n");
		goto sbpc_board_probe_error;
	}
	board->ioport_req = 1;
	board->ioport_base_addr = pci_resource_start(pdev, 0);
	rc = pci_request_region(pdev, 1, "sbpc");
	if (rc) {
		dev_err(&pdev->dev, "can't request I/O region\n");
		goto sbpc_board_probe_error;
	}
	board->iomem_req = 1;
	if (!(board->iomem_base_ptr = pci_iomap(pdev, 1, 0x100000))) {
		dev_err(&pdev->dev, "can't map pci device memory\n");
		rc = -ENOMEM;
		goto sbpc_board_probe_error;
	}
	// Reset board
	iowrite8(0, board->iomem_base_ptr + SBPC_BOARD_CTRL_BASE + SBPC_BOARD_CTRL_RESET);
	mdelay(1);
	iowrite8(1, board->iomem_base_ptr + SBPC_BOARD_CTRL_BASE + SBPC_BOARD_CTRL_RESET);
	// Read subboard ID
	board->id = ioread16(board->iomem_base_ptr + SBPC_BOARD_CTRL_BASE + SBPC_BOARD_CTRL_ID_DATA);
	verbose("board: ID=0x%04x\n", board->id);
	// Read subboard ROM
	for (i = 0; i < sizeof(board->rom); i++) {
		board->rom[i] = ioread8(board->iomem_base_ptr + SBPC_BOARD_CTRL_BASE + SBPC_BOARD_CTRL_ROM_DATA);
	}
	verbose("board: ROM=[%s]\n", &board->rom[2]);
	// test for access to device
	for (i = 0; i < 6; i++) {
		if (i == 0) {
			tw16_0 = 0x0000;
			tw16_1 = 0x0000;
		} else if (i == 1) {
			tw16_0 = 0x5555;
			tw16_1 = 0xaaaa;
		} else if (i == 2) {
			tw16_0 = 0x0000;
			tw16_1 = 0xffff;
		} else if (i == 3) {
			tw16_0 = 0xffff;
			tw16_1 = 0x0000;
		} else if (i == 4) {
			tw16_0 = 0xaaaa;
			tw16_1 = 0x5555;
		} else {
			tw16_0 = 0xffff;
			tw16_1 = 0xffff;
		}
		iowrite16(tw16_0, board->iomem_base_ptr + SBPC_BOARD_CTRL_BASE + SBPC_BOARD_CTRL_REG_0);
		iowrite16(tw16_1, board->iomem_base_ptr + SBPC_BOARD_CTRL_BASE + SBPC_BOARD_CTRL_REG_1);
		tr16_0 = ioread16(board->iomem_base_ptr + SBPC_BOARD_CTRL_BASE + SBPC_BOARD_CTRL_REG_0);
		tr16_1 = ioread16(board->iomem_base_ptr + SBPC_BOARD_CTRL_BASE + SBPC_BOARD_CTRL_REG_1);
		if ((tw16_0 != tr16_0) || (tw16_1 != tr16_1)) {
			verbose("0x%08x: write=0x%04x, read=0x%04x\n", SBPC_BOARD_CTRL_BASE + SBPC_BOARD_CTRL_REG_0, tw16_0, tr16_0);
			verbose("0x%08x: write=0x%04x, read=0x%04x\n", SBPC_BOARD_CTRL_BASE + SBPC_BOARD_CTRL_REG_1, tw16_1, tr16_1);
			dev_err(&pdev->dev, "test for access to device failed\n");
			rc = -ENOMEM;
			goto sbpc_board_probe_error;
		}
	}
	// Create SIM-card device
	for (i = sim_start; i <= sim_end; i++) {
		// get memory for simcard
		if (!(board->simcards[i] = kmalloc(sizeof(struct sbpc_simcard_device), GFP_KERNEL))) {
			log(KERN_ERR, "can't get memory for struct sbpc_simcard_device\n");
			goto sbpc_board_probe_error;
		}
		memset(board->simcards[i], 0, sizeof(struct sbpc_simcard_device));
		// Set SIM owner board end position
		board->simcards[i]->board = board;
		board->simcards[i]->position = i;
		// init simcard data
		board->simcards[i]->devno = MKDEV(sbpc_major, i);
		spin_lock_init(&board->simcards[i]->lock);
		init_waitqueue_head(&board->simcards[i]->poll_waitq);
		init_waitqueue_head(&board->simcards[i]->read_waitq);
		init_waitqueue_head(&board->simcards[i]->write_waitq);
		// Add char device to system
		cdev_init(&board->simcards[i]->cdev, &sbpc_simcard_fops);
		board->simcards[i]->cdev.owner = THIS_MODULE;
		board->simcards[i]->cdev.ops = &sbpc_simcard_fops;
		if ((rc = cdev_add(&board->simcards[i]->cdev, board->simcards[i]->devno, 1)) < 0) {
			log(KERN_ERR, "cdev_add() error=%d\n", rc);
			goto sbpc_board_probe_error;
		}
		board->simcards[i]->cdev_req = 1;
		snprintf(devname, sizeof(devname), "simbank!sim%d", MINOR(board->simcards[i]->devno));
		if (!(board->simcards[i]->device = CLASS_DEV_CREATE(sbpc_class, board->simcards[i]->devno, NULL, devname))) {
			log(KERN_ERR, "class_dev_create() error\n");
			goto sbpc_board_probe_error;
		}
		// init control register
		board->simcards[i]->control.bits.reset = 1;
		board->simcards[i]->control.bits.speed = 0;
		board->simcards[i]->control.bits.led = 0;
		// set SIM offset
		sim_offset = i % 100;
		if (i >= 100) {
			sim_offset |= 0x80;
		}
		iowrite8(board->simcards[i]->control.full, board->iomem_base_ptr + SBPC_SIM_CTRL_BASE + (sim_offset << 7) + SBPC_SIM_CTRL_GENERAL);
	}
	init_timer(&board->poll_timer);
	board->poll_timer.function = sbpc_poll_proc;
	board->poll_timer.data = (unsigned long)board;
	board->poll_timer.expires = jiffies + 1;
	add_timer(&board->poll_timer);

	verbose("board inserted\n");

	pci_set_drvdata(pdev, board);
	return 0;

sbpc_board_probe_error:
	if (board) {
		// release simcard data
		for (i = 0; i < SBPC_BOARD_SIM_MAXCOUNT; i++) {
			if (board->simcards[i]) {
				if (board->simcards[i]->device) {
					CLASS_DEV_DESTROY(sbpc_class, board->simcards[i]->devno);
				}
				if (board->simcards[i]->cdev_req) {
					cdev_del(&board->simcards[i]->cdev);
				}
				kfree(board->simcards[i]);
			}
		}
		if (board->ioport_req) {
			pci_release_region(pdev, 0);
		}
		if (board->iomem_req) {
			pci_release_region(pdev, 1);
		}
		if (board->iomem_base_ptr) {
			pci_iounmap(pdev, board->iomem_base_ptr);
		}
		kfree(board);
	}
	return rc;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static void sbpc_board_remove(struct pci_dev *pdev)
#else
static void __devexit sbpc_board_remove(struct pci_dev *pdev)
#endif
{
	size_t i;
	struct sbpc_board *board = pci_get_drvdata(pdev);

	if (board) {
		del_timer_sync(&board->poll_timer);
		// release simcard data
		for (i = 0; i < SBPC_BOARD_SIM_MAXCOUNT; i++) {
			if (board->simcards[i]) {
				if (board->simcards[i]->device) {
					CLASS_DEV_DESTROY(sbpc_class, board->simcards[i]->devno);
				}
				if (board->simcards[i]->cdev_req) {
					cdev_del(&board->simcards[i]->cdev);
				}
				kfree(board->simcards[i]);
			}
		}
		if (board->ioport_req) {
			pci_release_region(pdev, 0);
		}
		if (board->iomem_req) {
			pci_release_region(pdev, 1);
		}
		if (board->iomem_base_ptr) {
			pci_iounmap(pdev, board->iomem_base_ptr);
		}
		kfree(board);
	}

	verbose("board removed\n");
}

static struct pci_driver sbpc_driver = {
	.name = "sbpc",
	.id_table = sbpc_board_id_table,
	.probe = sbpc_board_probe,
	.remove = sbpc_board_remove,
};

static int __init sbpc_init(void)
{
	dev_t devno;
	int sbpc_major_reg = 0;
	int rc = 0;

	verbose("loading version \"%s\"...\n", SIMBANK_LINUX_VERSION);

	if ((sim_start < 0) || (sim_start >= SBPC_BOARD_SIM_MAXCOUNT)) {
		sim_start = 0;
	}
	if ((sim_end < sim_start) || (sim_end >= SBPC_BOARD_SIM_MAXCOUNT)) {
		sim_end = SBPC_BOARD_SIM_MAXCOUNT - 1;
	}
	if ((sim_start != 0) || (sim_end != (SBPC_BOARD_SIM_MAXCOUNT - 1))) {
		verbose("SIM start=%d, end=%d\n", sim_start, sim_end);
	}

	// Registering sbpc device class
	if (!(sbpc_class = class_create(THIS_MODULE, "sbpc"))) {
		log(KERN_ERR, "class_create() error\n");
		goto sbpc_init_error;
	}
	// Register char device region
	if (sbpc_major) {
		devno = MKDEV(sbpc_major, 0);
		rc = register_chrdev_region(devno, SBPC_DEVICE_MAXCOUNT, "sbpc");
	} else {
		rc = alloc_chrdev_region(&devno, 0, SBPC_DEVICE_MAXCOUNT, "sbpc");
		if (rc >= 0) {
			sbpc_major = MAJOR(devno);
		}
	}
	if (rc < 0) {
		log(KERN_ERR, "register chrdev region error=%d\n", rc);
		goto sbpc_init_error;
	}
	debug("sbpc major=%d\n", sbpc_major);
	sbpc_major_reg = 1;

	if ((rc = pci_register_driver(&sbpc_driver)) < 0) {
		log(KERN_ERR, "can't register pci driver\n");
		goto sbpc_init_error;
	}

	verbose("loaded successfull\n");
	return 0;

sbpc_init_error:
	// Unregister sbpc device region
	if (sbpc_major_reg) {
		unregister_chrdev_region(MKDEV(sbpc_major, 0), SBPC_DEVICE_MAXCOUNT);
	}
	// Destroy sbpc device class
	if (sbpc_class) {
		class_destroy(sbpc_class);
	}
	return rc;
}

static void __exit sbpc_exit(void)
{
	// Unregister pci driver
	pci_unregister_driver(&sbpc_driver);
	// Unregister sbpc device region
	unregister_chrdev_region(MKDEV(sbpc_major, 0), SBPC_DEVICE_MAXCOUNT);
	// Destroy sbpc device class
	class_destroy(sbpc_class);

	verbose("stopped\n");
}

module_init(sbpc_init);
module_exit(sbpc_exit);

/* end of sbpc-base.c */
