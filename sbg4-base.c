/******************************************************************************/
/* sbg4-base.c                                                                */
/******************************************************************************/

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/version.h>

#include "../arch/arm/include/asm/io.h"
#include "../arch/arm/mach-at91/include/mach/hardware.h"
#include "../arch/arm/mach-at91/include/mach/io.h"
#include "../arch/arm/mach-at91/include/mach/at91_pio.h"
#include "../arch/arm/mach-at91/include/mach/at91sam9260_matrix.h"

#include "simbank/simcard-def.h"
#include "simbank/version.h"

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@gmail.com>");
MODULE_DESCRIPTION("Polygator Linux module for SBG4 device");
MODULE_LICENSE("GPL");

static int sbg4_major = 0;
module_param(sbg4_major, int, 0);
MODULE_PARM_DESC(sbg4_major, "Major number for Polygator SBG4 device");

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
	static struct class *sbg4_class = NULL;
#else
	static struct class_simple *sbg4_class = NULL;
	#define class_create(_a, _b) class_simple_create(_a, _b)
	#define class_destroy(_a) class_simple_destroy(_a)
#endif

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "sbg4-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "sbg4-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

#define SBG4_DEVICE_MAXCOUNT		512
#define SBG4_BOARD_MAXCOUNT			2
#define SBG4_BOARD_SIM_MAXCOUNT		100 /* 100 */

#define SBG4_SIM_DATA_SOFT_BUFF_SIZE	256 /* 1024 */
#define SBG4_SIM_DATA_HARD_BUFF_SIZE	512

#define SBG4_RESET_BUFF_POINTERS

#define SBG4_BOARD_CTRL_BASE		0x80000
#define SBG4_BOARD_CTRL_MONITOR		0x002
#define SBG4_BOARD_TEST_REG_0		0x006
#define SBG4_BOARD_TEST_REG_1		0x008
#define SBG4_BOARD_TYPE				0x00a
#define SBG4_BOARD_ROM_RESET		0x00c
#define SBG4_BOARD_ROM_DATA			0x100

#define SBG4_SIM_CTRL_GENERAL		0x200

#define SBG4_SIM_CTRL_RX_BUF_RP		0x220
#define SBG4_SIM_CTRL_RX_BUF_WP		0x224
#define SBG4_SIM_CTRL_RX_BUF_FL		0x228
#define SBG4_SIM_CTRL_RX_BUF_OF		0x22c

#define SBG4_SIM_CTRL_TX_BUF_WP		0x244
#define SBG4_SIM_CTRL_TX_BUF_FL		0x248
#define SBG4_SIM_CTRL_TX_BUF_RP		0x24c

#define MAIN_BOARD_CTRL_BASE		0x00000
#define MAIN_BOARD_TEST_REG_0		0x006
#define MAIN_BOARD_TEST_REG_1		0x008
#define MAIN_BOARD_TYPE				0x00a
#define MAIN_BOARD_ROM_RESET		0x00c
#define MAIN_BOARD_ROM_DATA			0x100

static struct resource * sbg4_cs3_iomem_reg = NULL;
static void __iomem * sbg4_cs3_base_ptr = NULL;

static u_int16_t mainboard_type;
static char mainboard_rom[128];

static struct timer_list sbg4_poll_timer;

struct sbg4_board;
struct sbg4_simcard_device {

	spinlock_t lock;

	size_t usage;

	struct sbg4_board *board;
	size_t position;

	struct {
		unsigned int rdof:1;
	} flags;

	union {
		struct {
			u_int8_t reset:1;
			u_int8_t speed:2;
			u_int8_t xmit:1;
			u_int8_t res:4;
		} __attribute__((packed)) bits;
		u_int8_t full;
	} __attribute__((packed)) control;

	wait_queue_head_t poll_waitq;
	wait_queue_head_t read_waitq;
	wait_queue_head_t write_waitq;

	u_int8_t read_buf[SBG4_SIM_DATA_SOFT_BUFF_SIZE];
	size_t read_head;
	size_t read_tail;
	size_t read_count;

	u_int8_t write_buf[SBG4_SIM_DATA_SOFT_BUFF_SIZE];
	size_t write_head;
	size_t write_tail;
	size_t write_count;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	struct device *device;
#else
	struct class_device *device;
#endif
	struct cdev cdev;
	int devno;
};

struct sbg4_board {
	u_int16_t type;
	char rom[256];
	size_t position;
	struct sbg4_simcard_device *simcards[SBG4_BOARD_SIM_MAXCOUNT];
};

static struct sbg4_board *boards[SBG4_BOARD_MAXCOUNT];

static void sbg4_poll_proc(unsigned long data)
{
	size_t i, j, k;

	struct rx_buf_regs {
		u_int16_t rp;
		u_int16_t rp_h;
		u_int16_t wp;
		u_int16_t wp_h;
		u_int16_t fl;
		u_int16_t fl_h;
		u_int16_t of;
		u_int16_t of_h;
	} __attribute__((packed)) rx_buf_regs;

	struct tx_buf_regs {
		u_int16_t wp;
		u_int16_t wp_h;
		u_int16_t fl;
		u_int16_t fl_h;
		u_int16_t rp;
		u_int16_t rp_h;
	} __attribute__((packed)) tx_buf_regs;

	size_t chunk;

	struct sbg4_simcard_device *sim;

	for (k = 0; k < SBG4_BOARD_MAXCOUNT; k++) {
		if (boards[k]) {
			for (j = 0; j < SBG4_BOARD_SIM_MAXCOUNT; j++) {
				if ((sim = boards[k]->simcards[j]) && (sim->control.bits.reset)) {
					spin_lock(&sim->lock);
					// rx data
					rx_buf_regs.rp = ioread16(sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_RP);
					rx_buf_regs.wp = ioread16(sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_WP);
					rx_buf_regs.fl = ioread16(sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_FL);
					rx_buf_regs.of = ioread16(sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_OF);
					//
					while (1) {
						if (rx_buf_regs.of) {
							if (sim->flags.rdof == 0) {
								sim->flags.rdof = 1;
								log(KERN_WARNING, "%03lu: rx_buf OVERFLOW C of=%04x fl=%04x wp=%04x rp=%04x\n", (unsigned long int)(k * 100 + j), rx_buf_regs.of, rx_buf_regs.fl, rx_buf_regs.wp, rx_buf_regs.rp);
								sim->read_count = sim->read_head = sim->read_tail = 0;
								sim->write_count = sim->write_head = sim->write_tail = 0;
							}
							break;
						} else if (sim->read_count == SBG4_SIM_DATA_SOFT_BUFF_SIZE) {
							break;
						} else if (rx_buf_regs.wp > rx_buf_regs.rp) {
							while (rx_buf_regs.wp > rx_buf_regs.rp) {
								if (sim->read_head < sim->read_tail) {
									chunk = min((size_t)(sim->read_tail - sim->read_head), (size_t)(rx_buf_regs.wp - rx_buf_regs.rp));
									for (i = 0; i < chunk; i++) {
										sim->read_buf[sim->read_head + i] = ioread8(sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (1 << 17) + (j << 10) + (rx_buf_regs.rp + i) * 2);
									}
									// update device pointers
									rx_buf_regs.rp += chunk;
									if (rx_buf_regs.rp == SBG4_SIM_DATA_HARD_BUFF_SIZE) {
										rx_buf_regs.rp = 0;
									}
									iowrite16(rx_buf_regs.rp, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_RP);
									rx_buf_regs.fl = 0;
									// update buffer pointers
									sim->read_head += chunk;
									if (sim->read_head == SBG4_SIM_DATA_SOFT_BUFF_SIZE) {
										sim->read_head = 0;
									}
									sim->read_count += chunk;
								} else if ((sim->read_head > sim->read_tail) || (sim->read_count == 0)) {
									chunk = min((size_t)(SBG4_SIM_DATA_SOFT_BUFF_SIZE - sim->read_head), (size_t)(rx_buf_regs.wp - rx_buf_regs.rp));
									for (i = 0; i < chunk; i++) {
										sim->read_buf[sim->read_head + i] = ioread8(sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (1 << 17) + (j << 10) + (rx_buf_regs.rp + i) * 2);
									}
									// update device pointers
									rx_buf_regs.rp += chunk;
									if (rx_buf_regs.rp == SBG4_SIM_DATA_HARD_BUFF_SIZE) {
										rx_buf_regs.rp = 0;
									}
									iowrite16(rx_buf_regs.rp, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_RP);
									rx_buf_regs.fl = 0;
									// update buffer pointers
									sim->read_head += chunk;
									if (sim->read_head == SBG4_SIM_DATA_SOFT_BUFF_SIZE) {
										sim->read_head = 0;
									}
									sim->read_count += chunk;
								} else {
									break;
								}
							}
						} else if ((rx_buf_regs.wp < rx_buf_regs.rp) || (rx_buf_regs.fl)) {
							while ((rx_buf_regs.wp < rx_buf_regs.rp) || (rx_buf_regs.fl)) {
								if (sim->read_head < sim->read_tail) {
									chunk = min((size_t)(sim->read_tail - sim->read_head), (size_t)(SBG4_SIM_DATA_HARD_BUFF_SIZE - rx_buf_regs.rp));
									for (i = 0; i < chunk; i++) {
										sim->read_buf[sim->read_head + i] = ioread8(sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (1 << 17) + (j << 10) + (rx_buf_regs.rp + i) * 2);
									}
									// update device pointers
									rx_buf_regs.rp += chunk;
									if (rx_buf_regs.rp == SBG4_SIM_DATA_HARD_BUFF_SIZE) {
										rx_buf_regs.rp = 0;
									}
									iowrite16(rx_buf_regs.rp, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_RP);
									rx_buf_regs.fl = 0;
									// update buffer pointers
									sim->read_head += chunk;
									if (sim->read_head == SBG4_SIM_DATA_SOFT_BUFF_SIZE) {
										sim->read_head = 0;
									}
									sim->read_count += chunk;
								} else if ((sim->read_head > sim->read_tail) || (sim->read_count == 0)) {
									chunk = min((size_t)(SBG4_SIM_DATA_SOFT_BUFF_SIZE - sim->read_head), (size_t)(SBG4_SIM_DATA_HARD_BUFF_SIZE - rx_buf_regs.rp));
									for (i = 0; i < chunk; i++) {
										sim->read_buf[sim->read_head + i] = ioread8(sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (1 << 17) + (j << 10) + (rx_buf_regs.rp + i) * 2);
									}
									// update device pointers
									rx_buf_regs.rp += chunk;
									if (rx_buf_regs.rp == SBG4_SIM_DATA_HARD_BUFF_SIZE) {
										rx_buf_regs.rp = 0;
									}
									iowrite16(rx_buf_regs.rp, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_RP);
									rx_buf_regs.fl = 0;
									// update buffer pointers
									sim->read_head += chunk;
									if (sim->read_head == SBG4_SIM_DATA_SOFT_BUFF_SIZE) {
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
					// tx data
					if (sim->write_count) {
						tx_buf_regs.wp = ioread16(sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_TX_BUF_WP);
						tx_buf_regs.fl = ioread16(sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_TX_BUF_FL);
						tx_buf_regs.rp = ioread16(sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_TX_BUF_RP);
						while (1) {
							if (tx_buf_regs.fl) {
								break;
							} else if (sim->write_head > sim->write_tail) {
								while (sim->write_head > sim->write_tail) {
									if (tx_buf_regs.wp < tx_buf_regs.rp) {
										// get chunk length
										chunk = min((size_t)(tx_buf_regs.rp - tx_buf_regs.wp), (size_t)(sim->write_head - sim->write_tail));
#ifdef BUF_XMIT
										// enable xmit buffer
										sim->control.bits.xmit = 1;
										iowrite8(sim->control.full, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_GENERAL);
										sim->control.bits.xmit = 0;
#endif
										// copy data to xmit buffer
										for (i = 0; i < chunk; i++) {
											iowrite8(sim->write_buf[sim->write_tail + i], sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (1 << 17) + (j << 10) + (tx_buf_regs.wp + i) * 2);
										}
										// update device pointers
#ifdef SBG4_RESET_BUFF_POINTERS
										iowrite16(0xffff, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_WP);
										iowrite16(0x0000, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_RP);
#endif
										tx_buf_regs.wp += chunk;
										if (tx_buf_regs.wp == SBG4_SIM_DATA_HARD_BUFF_SIZE) {
											tx_buf_regs.wp = 0;
										}
										if (tx_buf_regs.wp == tx_buf_regs.rp) {
											tx_buf_regs.fl = 1;
											iowrite16(tx_buf_regs.fl, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_TX_BUF_FL);
										}
										iowrite16(tx_buf_regs.wp, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_TX_BUF_WP);
										// update buffer pointers
										sim->write_tail += chunk;
										if (sim->write_tail == SBG4_SIM_DATA_SOFT_BUFF_SIZE) {
											sim->write_tail = 0;
										}
										sim->write_count -= chunk;
									} else if ((tx_buf_regs.wp > tx_buf_regs.rp) || (tx_buf_regs.fl == 0)) {
										// get chunk length
										chunk = min((size_t)(SBG4_SIM_DATA_HARD_BUFF_SIZE - tx_buf_regs.wp), (size_t)(sim->write_head - sim->write_tail));
#ifdef BUF_XMIT
										// enable xmit buffer
										sim->control.bits.xmit = 1;
										iowrite8(sim->control.full, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_GENERAL);
										sim->control.bits.xmit = 0;
#endif
										// copy data to xmit buffer
										for (i = 0; i < chunk; i++) {
											iowrite8(sim->write_buf[sim->write_tail + i], sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (1 << 17) + (j << 10) + (tx_buf_regs.wp + i) * 2);
										}
										// update device pointers
#ifdef SBG4_RESET_BUFF_POINTERS
										iowrite16(0xffff, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_WP);
										iowrite16(0x0000, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_RP);
#endif
										tx_buf_regs.wp += chunk;
										if (tx_buf_regs.wp == SBG4_SIM_DATA_HARD_BUFF_SIZE) {
											tx_buf_regs.wp = 0;
										}
										if (tx_buf_regs.wp == tx_buf_regs.rp) {
											tx_buf_regs.fl = 1;
											iowrite16(tx_buf_regs.fl, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_TX_BUF_FL);
										}
										iowrite16(tx_buf_regs.wp, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_TX_BUF_WP);
										// update buffer pointers
										sim->write_tail += chunk;
										if (sim->write_tail == SBG4_SIM_DATA_SOFT_BUFF_SIZE) {
											sim->write_tail = 0;
										}
										sim->write_count -= chunk;
									} else {
										break;
									}
								}
							} else if ((sim->write_head < sim->write_tail) || (sim->write_count == SBG4_SIM_DATA_SOFT_BUFF_SIZE)) {
								while ((sim->write_head < sim->write_tail) || (sim->write_count == SBG4_SIM_DATA_SOFT_BUFF_SIZE)) {
									if (tx_buf_regs.wp < tx_buf_regs.rp) {
										// get chunk length
										chunk = min((size_t)(tx_buf_regs.rp - tx_buf_regs.wp), (size_t)(SBG4_SIM_DATA_SOFT_BUFF_SIZE - sim->write_tail));
#ifdef BUF_XMIT
										// enable xmit buffer
										sim->control.bits.xmit = 1;
										iowrite8(sim->control.full, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_GENERAL);
										sim->control.bits.xmit = 0;
#endif
										// copy data to xmit buffer
										for (i = 0; i < chunk; i++) {
											iowrite8(sim->write_buf[sim->write_tail + i], sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (1 << 17) + (j << 10) + (tx_buf_regs.wp + i) * 2);
										}
										// update device pointers
#ifdef SBG4_RESET_BUFF_POINTERS
										iowrite16(0xffff, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_WP);
										iowrite16(0x0000, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_RP);
#endif
										tx_buf_regs.wp += chunk;
										if (tx_buf_regs.wp == SBG4_SIM_DATA_HARD_BUFF_SIZE) {
											tx_buf_regs.wp = 0;
										}
										if (tx_buf_regs.wp == tx_buf_regs.rp) {
											tx_buf_regs.fl = 1;
											iowrite16(tx_buf_regs.fl, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_TX_BUF_FL);
										}
										iowrite16(tx_buf_regs.wp, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_TX_BUF_WP);
										// update buffer pointers
										sim->write_tail += chunk;
										if (sim->write_tail == SBG4_SIM_DATA_SOFT_BUFF_SIZE) {
											sim->write_tail = 0;
										}
										sim->write_count -= chunk;
									} else if ((tx_buf_regs.wp > tx_buf_regs.rp) || (tx_buf_regs.fl == 0)) {
										// get chunk length
										chunk = min((size_t)(SBG4_SIM_DATA_HARD_BUFF_SIZE - tx_buf_regs.wp), (size_t)(SBG4_SIM_DATA_SOFT_BUFF_SIZE - sim->write_tail));
#ifdef BUF_XMIT
										// enable xmit buffer
										sim->control.bits.xmit = 1;
										iowrite8(sim->control.full, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_GENERAL);
										sim->control.bits.xmit = 0;
#endif
										// copy data to xmit buffer
										for (i = 0; i < chunk; i++) {
											iowrite8(sim->write_buf[sim->write_tail + i], sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (1 << 17) + (j << 10) + (tx_buf_regs.wp + i) * 2);
										}
										// update device pointers
#ifdef SBG4_RESET_BUFF_POINTERS
										iowrite16(0xffff, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_WP);
										iowrite16(0x0000, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_RX_BUF_RP);
#endif
										tx_buf_regs.wp += chunk;
										if (tx_buf_regs.wp == SBG4_SIM_DATA_HARD_BUFF_SIZE) {
											tx_buf_regs.wp = 0;
										}
										if (tx_buf_regs.wp == tx_buf_regs.rp) {
											tx_buf_regs.fl = 1;
											iowrite16(tx_buf_regs.fl, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_TX_BUF_FL);
										}
										iowrite16(tx_buf_regs.wp, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (k << 18) + (0 << 17) + (j << 10) + SBG4_SIM_CTRL_TX_BUF_WP);
										// update buffer pointers
										sim->write_tail += chunk;
										if (sim->write_tail == SBG4_SIM_DATA_SOFT_BUFF_SIZE) {
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
					spin_unlock(&sim->lock);
				}
			}
		}
	}

	mod_timer(&sbg4_poll_timer, jiffies + 1);
}

static int sbg4_simcard_open(struct inode *inode, struct file *filp)
{
	size_t usage;
	struct sbg4_simcard_device *sim;

	sim = container_of(inode->i_cdev, struct sbg4_simcard_device, cdev);
	filp->private_data = sim;

	spin_lock_bh(&sim->lock);
	usage = sim->usage++;
	sim->read_count = sim->read_head = sim->read_tail = 0;
	sim->write_count = sim->write_head = sim->write_tail = 0;
	spin_unlock_bh(&sim->lock);

	if (usage) {
		return -EBUSY;
	} else {
		return 0;
	}
}

static int sbg4_simcard_release(struct inode *inode, struct file *filp)
{
	struct sbg4_simcard_device *sim = filp->private_data;

	spin_lock_bh(&sim->lock);
	sim->control.bits.reset = 0;
	sim->usage--;
	spin_unlock_bh(&sim->lock);

	return 0;
}

static ssize_t sbg4_simcard_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	size_t length;
	size_t pos, chunk, rest;
	struct simcard_data data;
	struct sbg4_simcard_device *sim = filp->private_data;

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
			goto sbg4_simcard_read_end;
		}
		// sleeping
		spin_unlock_bh(&sim->lock);
		if ((res = wait_event_interruptible(sim->read_waitq, sim->read_count))) {
			goto sbg4_simcard_read_end;
		}
		spin_lock_bh(&sim->lock);
	}

	length = sizeof(data.header) + sim->read_count;
	length = min(length, count);
	data.header.type = SIMCARD_CONTAINER_TYPE_DATA;
	data.header.length = length - sizeof(data.header);

	pos = 0;
	rest = data.header.length;
	while (rest && sim->read_count) {
		if (sim->read_head > sim->read_tail) {
			chunk = sim->read_head - sim->read_tail;
			memcpy(data.container.data + pos, sim->read_buf + sim->read_tail, chunk);
			pos += chunk;
			rest -= chunk;
			sim->read_count -= chunk;
			sim->read_tail += chunk;
			if (sim->read_tail == SBG4_SIM_DATA_SOFT_BUFF_SIZE) {
				sim->read_tail = 0;
			}
		} else if ((sim->read_head < sim->read_tail) || (sim->read_count == SBG4_SIM_DATA_SOFT_BUFF_SIZE)) {
			chunk = SBG4_SIM_DATA_SOFT_BUFF_SIZE - sim->read_tail;
			memcpy(data.container.data + pos, sim->read_buf + sim->read_tail, chunk);
			pos += chunk;
			rest -= chunk;
			sim->read_count -= chunk;
			sim->read_tail = 0;
		} else {
			break;
		}
	}

	spin_unlock_bh(&sim->lock);

	if (copy_to_user(buff, &data, length)) {
		res = -EFAULT;
		goto sbg4_simcard_read_end;
	}

	res = length;

sbg4_simcard_read_end:
	return res;
}

static ssize_t sbg4_simcard_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	size_t length;
	size_t rest, pos, chunk;
	struct simcard_data data;

	struct sbg4_simcard_device *sim = filp->private_data;

	res = 0;

	length = sizeof(struct simcard_data);
	length = min(length, count);
	if (copy_from_user(&data, buff, length)) {
		res = -EFAULT;
		goto sbg4_simcard_write_end;
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
					if (sim->write_head == SBG4_SIM_DATA_SOFT_BUFF_SIZE) {
						sim->write_head = 0;
					}
					sim->write_count += chunk;
				} else if ((sim->write_head > sim->write_tail) || (sim->write_count == 0)) {
					chunk = min((size_t)(SBG4_SIM_DATA_SOFT_BUFF_SIZE - sim->write_head), rest);
					memcpy(sim->write_buf + sim->write_head, data.container.data + pos, chunk);
					// update source pointers
					rest -= chunk;
					pos += chunk;
					// update buffer pointers
					sim->write_head += chunk;
					if (sim->write_head == SBG4_SIM_DATA_SOFT_BUFF_SIZE) {
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
			sim->control.bits.reset = data.container.reset & 1;
			iowrite8(sim->control.full, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (sim->board->position << 18) + (0 << 17) + (sim->position << 10) + SBG4_SIM_CTRL_GENERAL);
			if (!sim->control.bits.reset) {
				sim->read_count = sim->read_head = sim->read_tail = 0;
				sim->write_count = sim->write_head = sim->write_tail = 0;
				sim->flags.rdof = 0;
			}
			break;
		case SIMCARD_CONTAINER_TYPE_SPEED:
			switch (data.container.speed) {
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
			iowrite8(sim->control.full, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (sim->board->position << 18) + (0 << 17) + (sim->position << 10) + SBG4_SIM_CTRL_GENERAL);
			break;
		case SIMCARD_CONTAINER_TYPE_MONITOR:
			iowrite8(data.container.monitor, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (sim->board->position << 18) + (0 << 17) + SBG4_BOARD_CTRL_MONITOR);
			break;
		default:
			spin_unlock_bh(&sim->lock);
			res = -EINVAL;
			goto sbg4_simcard_write_end;
	}

	spin_unlock_bh(&sim->lock);

	res = length;

sbg4_simcard_write_end:
	return res;
}

static unsigned int sbg4_simcard_poll(struct file *filp, struct poll_table_struct *wait_table)
{
	unsigned int res;
	struct sbg4_simcard_device *sim = filp->private_data;

	res = 0;

	poll_wait(filp, &sim->poll_waitq, wait_table);

	spin_lock_bh(&sim->lock);

	if (sim->read_count) {
		res |= POLLIN | POLLRDNORM;
	}

	if (sim->write_count < SBG4_SIM_DATA_SOFT_BUFF_SIZE) {
		res |= POLLOUT | POLLWRNORM;
	}

	spin_unlock_bh(&sim->lock);

	return res;
}

static struct file_operations sbg4_simcard_fops = {
	.owner		= THIS_MODULE,
	.open		= sbg4_simcard_open,
	.release	= sbg4_simcard_release,
	.read		= sbg4_simcard_read,
	.write		= sbg4_simcard_write,
	.poll		= sbg4_simcard_poll,
};

static int __init sbg4_init(void)
{
	size_t i, j, k;
	u_int16_t tw16_0, tw16_1, tr16_0, tr16_1;
	dev_t devno;
	u32 tmpu32;
	char devname[64];
	int sbg4_major_reg = 0;
	int rc = 0;

	verbose("loading version \"%s\"...\n", SIMBANK_LINUX_VERSION);

	// Init subboard list
	for (j = 0; j < SBG4_BOARD_MAXCOUNT; j++) {
		boards[j] = NULL;
	}

	// Registering sbg4 device class
	if (!(sbg4_class = class_create(THIS_MODULE, "sbg4"))) {
		log(KERN_ERR, "class_create() error\n");
		goto sbg4_init_error;
	}
	// Register char device region
	if (sbg4_major) {
		devno = MKDEV(sbg4_major, 0);
		rc = register_chrdev_region(devno, SBG4_DEVICE_MAXCOUNT, "sbg4");
	} else {
		rc = alloc_chrdev_region(&devno, 0, SBG4_DEVICE_MAXCOUNT, "sbg4");
		if (rc >= 0) {
			sbg4_major = MAJOR(devno);
		}
	}
	if (rc < 0) {
		log(KERN_ERR, "register chrdev region error=%d\n", rc);
		goto sbg4_init_error;
	}
	debug("sbg4 major=%d\n", sbg4_major);
	sbg4_major_reg = 1;

	// Assign CS3, CS4 to SMC
	tmpu32 = at91_sys_read(AT91_MATRIX_EBICSA);
	tmpu32 &= ~(AT91_MATRIX_CS3A | AT91_MATRIX_CS4A);
	tmpu32 |= (AT91_MATRIX_CS3A_SMC | AT91_MATRIX_CS4A_SMC);
	at91_sys_write(AT91_MATRIX_EBICSA, tmpu32);

	// Configure PIOC for using CS3, CS4
	at91_sys_write(AT91_PIOC + PIO_PDR, (1 << 14)|(1 << 8)); /* Disable Register */
	tmpu32 = at91_sys_read(AT91_PIOC + PIO_PSR); /* Status Register */
	at91_sys_write(AT91_PIOC + PIO_ASR, (1 << 14)|(1 << 8)); /* Peripheral A Select Register */
	tmpu32 = at91_sys_read(AT91_PIOC + PIO_ABSR); /* AB Status Register */

	// Configure SMC CS3 timings
	at91_sys_write(AT91_SMC + 0x30 + 0x0, 0x01030103);
	at91_sys_write(AT91_SMC + 0x30 + 0x4, 0x0f0c0f0c);
	at91_sys_write(AT91_SMC + 0x30 + 0x8, 0x00140014);
	at91_sys_write(AT91_SMC + 0x30 + 0xc, 0x10001003);

	// Request and remap i/o memory region for cs3
	if (check_mem_region(AT91_CHIPSELECT_3, 0x10000)) {
		log(KERN_ERR, "i/o memory region for cs3 already used\n");
		rc = -ENOMEM;
		goto sbg4_init_error;
	}
	if (!(sbg4_cs3_iomem_reg = request_mem_region(AT91_CHIPSELECT_3, 0x100000, "sbg4"))) {
		log(KERN_ERR, "can't request i/o memory region for cs3\n");
		rc = -ENOMEM;
		goto sbg4_init_error;
	}
	if (!(sbg4_cs3_base_ptr = ioremap_nocache(AT91_CHIPSELECT_3, 0x100000))) {
		log(KERN_ERR, "can't remap i/o memory for cs3\n");
		rc = -ENOMEM;
		goto sbg4_init_error;
	}

	// reset mainboard ROM
	iowrite8(0, sbg4_cs3_base_ptr + MAIN_BOARD_CTRL_BASE + MAIN_BOARD_ROM_RESET);
	mdelay(1);
	iowrite8(1, sbg4_cs3_base_ptr + MAIN_BOARD_CTRL_BASE + MAIN_BOARD_ROM_RESET);
	// read mainboard ROM
	for (i = 0; i < sizeof(mainboard_rom); i++) {
		mainboard_rom[i] = ioread8(sbg4_cs3_base_ptr + MAIN_BOARD_CTRL_BASE + MAIN_BOARD_ROM_DATA + i * 2);
	}
	verbose("mainboard: ROM=[%s]\n", &mainboard_rom[1]);
	// read mainboard type
	mainboard_type = ioread16(sbg4_cs3_base_ptr + MAIN_BOARD_CTRL_BASE + MAIN_BOARD_TYPE);
	verbose("mainboard: TYPE=0x%04x\n", mainboard_type);
	// test for access to mainboard
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
		iowrite16(tw16_0, sbg4_cs3_base_ptr + MAIN_BOARD_CTRL_BASE + MAIN_BOARD_TEST_REG_0);
		iowrite16(tw16_1, sbg4_cs3_base_ptr + MAIN_BOARD_CTRL_BASE + MAIN_BOARD_TEST_REG_1);
		tr16_0 = ioread16(sbg4_cs3_base_ptr + MAIN_BOARD_CTRL_BASE + MAIN_BOARD_TEST_REG_0);
		tr16_1 = ioread16(sbg4_cs3_base_ptr + MAIN_BOARD_CTRL_BASE + MAIN_BOARD_TEST_REG_1);
		if ((tw16_0 != tr16_0) || (tw16_1 != tr16_1)) {
			verbose("mainboard: 0x%08x: write=0x%04x, read=0x%04x\n", MAIN_BOARD_CTRL_BASE + MAIN_BOARD_TEST_REG_0, tw16_0, tr16_0);
			verbose("mainboard: 0x%08x: write=0x%04x, read=0x%04x\n", MAIN_BOARD_CTRL_BASE + MAIN_BOARD_TEST_REG_1, tw16_1, tr16_1);
			log(KERN_ERR, "test for access to mainboard failed\n");
			rc = -ENOMEM;
			goto sbg4_init_error;
		}
	}
	// check for installed boards
	for (j = 0; j < SBG4_BOARD_MAXCOUNT; j++) {
		// Reset board ROM
		iowrite8(0, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (j << 18) + (0 << 17) + SBG4_BOARD_ROM_RESET);
		mdelay(1);
		iowrite8(1, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (j << 18) + (0 << 17) + SBG4_BOARD_ROM_RESET);
		// get memory for board
		if (!(boards[j] = kmalloc(sizeof(struct sbg4_board), GFP_KERNEL))) {
			log(KERN_ERR, "can't get memory for struct sbg4_board\n");
			goto sbg4_init_error;
		}
		memset(boards[j], 0, sizeof(struct sbg4_board));
		// read board ROM
		for (i = 0; i < sizeof(boards[j]->rom); i++) {
			boards[j]->rom[i] = ioread8(sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (j << 18) + (0 << 17) + SBG4_BOARD_ROM_DATA + i * 2);
		}
		// read board type
		boards[j]->type = ioread16(sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (j << 18) + (0 << 17) + SBG4_BOARD_TYPE);
		// test for access to board
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
			iowrite16(tw16_0, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (j << 18) + (0 << 17) + SBG4_BOARD_TEST_REG_0);
			iowrite16(tw16_1, sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (j << 18) + (0 << 17) + SBG4_BOARD_TEST_REG_1);
			tr16_0 = ioread16(sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (j << 18) + (0 << 17) + SBG4_BOARD_TEST_REG_0);
			tr16_1 = ioread16(sbg4_cs3_base_ptr + SBG4_BOARD_CTRL_BASE + (j << 18) + (0 << 17) + SBG4_BOARD_TEST_REG_1);
			if ((tw16_0 != tr16_0) || (tw16_1 != tr16_1)) {
#if 0
				verbose("board %lu: 0x%08x: write=0x%04x, read=0x%04x\n", (unsigned long int)j, SBG4_BOARD_CTRL_BASE + (j << 18) + (0 << 17) + SBG4_BOARD_TEST_REG_0, tw16_0, tr16_0);
				verbose("board %lu: 0x%08x: write=0x%04x, read=0x%04x\n", (unsigned long int)j, SBG4_BOARD_CTRL_BASE + (j << 18) + (0 << 17) + SBG4_BOARD_TEST_REG_1, tw16_1, tr16_1);
#endif
				// delete board data
				kfree(boards[j]);
				boards[j] = NULL;
				break;
			}
		}
		if (boards[j]) {
			// Check for supported board type
			if (boards[j]->type == 0xc905) {
				// Out board data
				verbose("board %lu: TYPE=0x%04x\n", (unsigned long int)j, boards[j]->type);
				verbose("board %lu: ROM=[%s]\n", (unsigned long int)j, &boards[j]->rom[1]);
				// Set board position
				boards[j]->position = j;
				// Create SIM-card device
				for (k = 0; k < SBG4_BOARD_SIM_MAXCOUNT; k++) {
					// get memory for simcard
					if (!(boards[j]->simcards[k] = kmalloc(sizeof(struct sbg4_simcard_device), GFP_KERNEL))) {
						log(KERN_ERR, "can't get memory for struct sbg4_simcard_device\n");
						goto sbg4_init_error;
					}
					memset(boards[j]->simcards[k], 0, sizeof(struct sbg4_simcard_device));
					// Set SIM owner board end position
					boards[j]->simcards[k]->board = boards[j];
					boards[j]->simcards[k]->position = k;
					// init simcard data
					devno = boards[j]->simcards[k]->devno = MKDEV(sbg4_major, k + (j * SBG4_BOARD_SIM_MAXCOUNT));
					spin_lock_init(&boards[j]->simcards[k]->lock);
					init_waitqueue_head(&boards[j]->simcards[k]->poll_waitq);
					init_waitqueue_head(&boards[j]->simcards[k]->read_waitq);
					init_waitqueue_head(&boards[j]->simcards[k]->write_waitq);
					// Add char device to system
					cdev_init(&boards[j]->simcards[k]->cdev, &sbg4_simcard_fops);
					boards[j]->simcards[k]->cdev.owner = THIS_MODULE;
					boards[j]->simcards[k]->cdev.ops = &sbg4_simcard_fops;
					if ((rc = cdev_add(&boards[j]->simcards[k]->cdev, devno, 1)) < 0) {
						log(KERN_ERR, "cdev_add() error=%d\n", rc);
						goto sbg4_init_error;
					}
					snprintf(devname, sizeof(devname), "simbank!sim%d", MINOR(boards[j]->simcards[k]->devno));
					if (!(boards[j]->simcards[k]->device = CLASS_DEV_CREATE(sbg4_class, devno, NULL, devname))) {
						log(KERN_ERR, "class_dev_create() error\n");
						goto sbg4_init_error;
					}
				}
			}
		}
	}

	init_timer(&sbg4_poll_timer);
	sbg4_poll_timer.function = sbg4_poll_proc;
	sbg4_poll_timer.data = 0;
	sbg4_poll_timer.expires = jiffies + 1;
	add_timer(&sbg4_poll_timer);

	verbose("loaded successfull\n");
	return rc;

sbg4_init_error:
	// Release boards data
	for (j = 0; j < SBG4_BOARD_MAXCOUNT; j++) {
		if (boards[j]) {
			// release simcard data
			for (k = 0; k < SBG4_BOARD_SIM_MAXCOUNT; k++) {
				if (boards[j]->simcards[k]) {
					CLASS_DEV_DESTROY(sbg4_class, boards[j]->simcards[k]->devno);
					cdev_del(&boards[j]->simcards[k]->cdev);
					kfree(boards[j]->simcards[k]);
				}
			}
			kfree(boards[j]);
		}
	}
	// Release CS3 memory region
	if (sbg4_cs3_iomem_reg) {
		release_mem_region(AT91_CHIPSELECT_3, 0x100000);
	}
	if (sbg4_cs3_base_ptr) {
		iounmap(sbg4_cs3_base_ptr);
	}
	// Unregister SBG4 device region
	if (sbg4_major_reg) {
		unregister_chrdev_region(MKDEV(sbg4_major, 0), SBG4_DEVICE_MAXCOUNT);
	}
	// Destroy SBG4 device class
	if (sbg4_class) {
		class_destroy(sbg4_class);
	}
	return rc;
}

static void __exit sbg4_exit(void)
{
	size_t j, k;

	// Delete SBG4 polling timer
	del_timer_sync(&sbg4_poll_timer);
	// Release boards data
	for (j = 0; j < SBG4_BOARD_MAXCOUNT; j++) {
		if (boards[j]) {
			// release simcard data
			for (k = 0; k < SBG4_BOARD_SIM_MAXCOUNT; k++) {
				if (boards[j]->simcards[k]) {
					CLASS_DEV_DESTROY(sbg4_class, boards[j]->simcards[k]->devno);
					cdev_del(&boards[j]->simcards[k]->cdev);
					kfree(boards[j]->simcards[k]);
				}
			}
			kfree(boards[j]);
		}
	}
	// Release CS3 memory region
	iounmap(sbg4_cs3_base_ptr);
	release_mem_region(AT91_CHIPSELECT_3, 0x100000);
	// Unregister SBG4 device region
	unregister_chrdev_region(MKDEV(sbg4_major, 0), SBG4_DEVICE_MAXCOUNT);
	// Destroy SBG4 device class
	class_destroy(sbg4_class);

	verbose("stopped\n");
}

module_init(sbg4_init);
module_exit(sbg4_exit);

/******************************************************************************/
/* end of sbg4-base.c                                                         */
/******************************************************************************/
