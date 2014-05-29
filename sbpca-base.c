/* sbpca-base.c */

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
MODULE_DESCRIPTION("Polygator Linux module for simbank device PCI(Altera)");
MODULE_LICENSE("GPL");

static int sbpca_major = 0;
module_param(sbpca_major, int, 0);
MODULE_PARM_DESC(sbpca_major, "Major number for Polygator simbank device PCI(Altera)");

static int sim_start = 0;
module_param(sim_start, int, 0);
MODULE_PARM_DESC(sim_start, "Start SIM Number");

static int sim_end = 199;
module_param(sim_end, int, 0);
MODULE_PARM_DESC(sim_end, "End SIM Number");

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
	static struct class *sbpca_class = NULL;
#else
	static struct class_simple *sbpca_class = NULL;
	#define class_create(_a, _b) class_simple_create(_a, _b)
	#define class_destroy(_a) class_simple_destroy(_a)
#endif

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, THIS_MODULE->name, ## _args)
#define log(_level, _fmt, _args...) printk(_level "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "sbpca-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, THIS_MODULE->name, "sbpca-base.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

#define SBPCA_DEVICE_MAXCOUNT		256
#define SBPCA_BOARD_SIM_MAXCOUNT	200

#define SBPCA_SIM_DATA_BUFF_SIZE	1024

#define SBPCA_SIM_CTRL_BASE			0xfc00
#define SBPCA_SIM_FLAG_BASE			0xfe00

union simcard_control_register {
	struct {
		u_int32_t reset:1;
		u_int32_t led:1;
		u_int32_t reserved1:6;
		u_int32_t speed:8;
		u_int32_t reserved0:16;
	} __attribute__((packed)) bits;
	u_int32_t full;
} __attribute__((packed));

union simcard_flag_register {
	struct {
		u_int32_t rx_wp:9;
		u_int32_t reserved:23;
	} __attribute__((packed)) bits;
	u_int32_t full;
} __attribute__((packed));

struct sbpca_simcard_device {
	spinlock_t lock;
	size_t usage;

	struct sbpca_board *board;
	size_t position;

	union simcard_control_register *control_register;

	wait_queue_head_t poll_waitq;
	wait_queue_head_t read_waitq;
	wait_queue_head_t write_waitq;

	u_int8_t read_buf[SBPCA_SIM_DATA_BUFF_SIZE];
	size_t read_head;
	size_t read_tail;
	size_t read_count;
	size_t read_overflow;

	u_int8_t write_buf[SBPCA_SIM_DATA_BUFF_SIZE];
	size_t write_head;
	size_t write_tail;
	size_t write_count;
	size_t write_overflow;

	size_t rx_buf_wp;
	size_t rx_buf_rp;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	struct device *device;
#else
	struct class_device *device;
#endif
	int devno;
	struct cdev cdev;
	int cdev_req;
};

struct sbpca_board {

	void __iomem *iomem_base[2];
	int iomem_req[2];

	union simcard_control_register control_space[2][100];

	struct sbpca_simcard_device *simcards[SBPCA_BOARD_SIM_MAXCOUNT];

	struct timer_list poll_timer;
};

static void sbpca_poll_proc(unsigned long data)
{
	union simcard_flag_register flag_space[2][100];
	union simcard_flag_register *flag_register;

	void __iomem *mem;
	size_t num;
	size_t i;
	size_t chunk;
	struct sbpca_simcard_device *sim;
	struct sbpca_board *board = (struct sbpca_board *)data;

	for (i = 0; i < 2; i++) {
		memcpy_fromio(flag_space[i], board->iomem_base[i] + SBPCA_SIM_FLAG_BASE, sizeof(union simcard_flag_register) * 100);
	}

	for (i = sim_start; i <= sim_end; i++) {
		if ((sim = board->simcards[i])) {
			num = i % 100;
			if (i / 100) {
				mem = board->iomem_base[0];
				flag_register = &flag_space[0][num];
			} else {
				mem = board->iomem_base[1];
				flag_register = &flag_space[1][num];
			}
			spin_lock(&sim->lock);
			if (sim->control_register->bits.reset) {
				sim->rx_buf_wp = flag_register->bits.rx_wp;
				while (1) {
					if (sim->read_count == SBPCA_SIM_DATA_BUFF_SIZE) {
						break;
					} else if (sim->rx_buf_wp > sim->rx_buf_rp) {
						while (sim->rx_buf_wp > sim->rx_buf_rp) {
							if (sim->read_head < sim->read_tail) {
								chunk = min((size_t)(sim->read_tail - sim->read_head), (size_t)(sim->rx_buf_wp - sim->rx_buf_rp));
								memcpy_fromio(&sim->read_buf[sim->read_head], mem + num * 0x200 + sim->rx_buf_rp, chunk);
								// update device pointers
								sim->rx_buf_rp += chunk;
								// update buffer pointers
								sim->read_head += chunk;
								if (sim->read_head == SBPCA_SIM_DATA_BUFF_SIZE) {
									sim->read_head = 0;
								}
								sim->read_count += chunk;
							} else if ((sim->read_head > sim->read_tail) || (sim->read_count == 0)) {
								chunk = min((size_t)(SBPCA_SIM_DATA_BUFF_SIZE - sim->read_head), (size_t)(sim->rx_buf_wp - sim->rx_buf_rp));
								memcpy_fromio(&sim->read_buf[sim->read_head], mem + num * 0x200 + sim->rx_buf_rp, chunk);
								// update device pointers
								sim->rx_buf_rp += chunk;
								// update buffer pointers
								sim->read_head += chunk;
								if (sim->read_head == SBPCA_SIM_DATA_BUFF_SIZE) {
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
			}
			spin_unlock(&sim->lock);
		}
	}
	mod_timer(&board->poll_timer, jiffies + 1);
}

static int sbpca_simcard_open(struct inode *inode, struct file *filp)
{
	size_t usage;
	struct sbpca_simcard_device *sim;

	sim = container_of(inode->i_cdev, struct sbpca_simcard_device, cdev);
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

static int sbpca_simcard_release(struct inode *inode, struct file *filp)
{
	struct sbpca_simcard_device *sim = filp->private_data;

	spin_lock_bh(&sim->lock);
	sim->usage--;
	spin_unlock_bh(&sim->lock);

	return 0;
}

static ssize_t sbpca_simcard_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	size_t length;
	size_t rest, pos, chunk;
	struct simcard_data data;
	struct sbpca_simcard_device *sim = filp->private_data;

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
			goto sbpca_simcard_read_end;
		}
		// sleeping
		spin_unlock_bh(&sim->lock);
		if ((res = wait_event_interruptible(sim->read_waitq, sim->read_count))) {
			goto sbpca_simcard_read_end;
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
			if (sim->read_tail == SBPCA_SIM_DATA_BUFF_SIZE) {
				sim->read_tail = 0;
			}
			sim->read_count -= chunk;
		} else if ((sim->read_head < sim->read_tail) || (sim->read_count == SBPCA_SIM_DATA_BUFF_SIZE)) {
			chunk = min((size_t)(SBPCA_SIM_DATA_BUFF_SIZE - sim->read_tail), rest);
			memcpy(data.container.data + pos, sim->read_buf + sim->read_tail, chunk);
			pos += chunk;
			rest -= chunk;
			sim->read_tail += chunk;
			if (sim->read_tail == SBPCA_SIM_DATA_BUFF_SIZE) {
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
		goto sbpca_simcard_read_end;
	}

	res = length;

sbpca_simcard_read_end:
	return res;
}

static ssize_t sbpca_simcard_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	void __iomem *mem;
	size_t num;
	ssize_t res;
	size_t length;
// 	size_t rest, pos, chunk;
	struct simcard_data data;

	struct sbpca_simcard_device *sim = filp->private_data;

	res = 0;

	num = sim->position % 100;
	if (sim->position / 100) {
		mem = sim->board->iomem_base[0];
	} else {
		mem = sim->board->iomem_base[1];
	}

	length = sizeof(struct simcard_data);
	length = min(length, count);
	if (copy_from_user(&data, buff, length)) {
		res = -EFAULT;
		goto sbpca_simcard_write_end;
	}

	spin_lock_bh(&sim->lock);

	switch (data.header.type) {
		case SIMCARD_CONTAINER_TYPE_DATA:
			memcpy_toio(mem + num * 0x200, data.container.data, data.header.length);
			sim->rx_buf_wp = 0;
			sim->rx_buf_rp = 0;
			break;
		case SIMCARD_CONTAINER_TYPE_RESET:
			sim->control_register->bits.reset = data.container.reset & 1;
			iowrite32(sim->control_register->full, mem + SBPCA_SIM_CTRL_BASE + num * 4);
			// init buffer pointers
			if (!sim->control_register->bits.reset) {
				sim->read_count = sim->read_head = sim->read_tail = 0;
				sim->read_overflow = 0;
				sim->write_count = sim->write_head = sim->write_tail = 0;
				sim->write_overflow = 0;
				sim->rx_buf_wp = 0;
				sim->rx_buf_rp = 0;
			}
			break;
		case SIMCARD_CONTAINER_TYPE_SPEED:
			sim->control_register->bits.speed = data.container.speed & 0xff;
			iowrite32(sim->control_register->full, mem + SBPCA_SIM_CTRL_BASE + num * 4);
			break;
		case SIMCARD_CONTAINER_TYPE_MONITOR:
			break;
		case SIMCARD_CONTAINER_TYPE_LED:
			sim->control_register->bits.led = data.container.led & 1;
			iowrite32(sim->control_register->full, mem + SBPCA_SIM_CTRL_BASE + num * 4);
			break;
		default:
			spin_unlock_bh(&sim->lock);
			res = -EINVAL;
			goto sbpca_simcard_write_end;
	}

	spin_unlock_bh(&sim->lock);

	res = length;

sbpca_simcard_write_end:
	return res;
}

static unsigned int sbpca_simcard_poll(struct file *filp, struct poll_table_struct *wait_table)
{
	unsigned int res;
	struct sbpca_simcard_device *sim = filp->private_data;

	res = 0;

	poll_wait(filp, &sim->poll_waitq, wait_table);

	spin_lock_bh(&sim->lock);

	if (sim->read_count) {
		res |= POLLIN | POLLRDNORM;
	}

	if (sim->write_count < SBPCA_SIM_DATA_BUFF_SIZE) {
		res |= POLLOUT | POLLWRNORM;
	}

	spin_unlock_bh(&sim->lock);

	return res;
}

static struct file_operations sbpca_simcard_fops = {
	.owner		= THIS_MODULE,
	.open		= sbpca_simcard_open,
	.release	= sbpca_simcard_release,
	.read		= sbpca_simcard_read,
	.write		= sbpca_simcard_write,
	.poll		= sbpca_simcard_poll,
};

static struct pci_device_id sbpca_board_id_table[] = {
	{ PCI_DEVICE(0x1172, 0x2704), .driver_data = 1, },
	{ 0, },
};
MODULE_DEVICE_TABLE(pci, sbpca_board_id_table);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static int sbpca_board_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
#else
static int __devinit sbpca_board_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
#endif
{
	char devname[64];
	size_t i;
	int rc;
	u8 rev;
	struct sbpca_board *board;

	void __iomem *mem;
	size_t num;

	// get memory for sbpca board
	if (!(board = kmalloc(sizeof(struct sbpca_board), GFP_KERNEL))) {
		dev_err(&pdev->dev, "can't get memory for struct sbpca_board\n");
		rc = -ENOMEM;
		goto sbpca_board_probe_error;
	}
	memset(board, 0, sizeof(struct sbpca_board));

	rc = pci_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev, "can't enable pci device\n");
		goto sbpca_board_probe_error;
	}

	for (i = 0; i < 2; i++) {
		rc = pci_request_region(pdev, i, "sbpca");
		if (rc) {
			dev_err(&pdev->dev, "can't request I/O region=%lu\n", (long unsigned int)i);
			goto sbpca_board_probe_error;
		}
		board->iomem_req[i] = 1;
		if (!(board->iomem_base[i] = pci_iomap(pdev, i, 0x10000))) {
			dev_err(&pdev->dev, "can't map pci device memory\n");
			rc = -ENOMEM;
			goto sbpca_board_probe_error;
		}
		memcpy_fromio(board->control_space[i], board->iomem_base[i] + SBPCA_SIM_CTRL_BASE, sizeof(union simcard_control_register) * 100);
	}

	// Create SIM-card device
	for (i = sim_start; i <= sim_end; i++) {
		// get memory for simcard
		if (!(board->simcards[i] = kmalloc(sizeof(struct sbpca_simcard_device), GFP_KERNEL))) {
			log(KERN_ERR, "can't get memory for struct sbpca_simcard_device\n");
			goto sbpca_board_probe_error;
		}
		memset(board->simcards[i], 0, sizeof(struct sbpca_simcard_device));
		// Set SIM owner board end position
		board->simcards[i]->board = board;
		board->simcards[i]->position = i;
		// init simcard data
		board->simcards[i]->devno = MKDEV(sbpca_major, i);
		spin_lock_init(&board->simcards[i]->lock);
		init_waitqueue_head(&board->simcards[i]->poll_waitq);
		init_waitqueue_head(&board->simcards[i]->read_waitq);
		init_waitqueue_head(&board->simcards[i]->write_waitq);
		// Add char device to system
		cdev_init(&board->simcards[i]->cdev, &sbpca_simcard_fops);
		board->simcards[i]->cdev.owner = THIS_MODULE;
		board->simcards[i]->cdev.ops = &sbpca_simcard_fops;
		if ((rc = cdev_add(&board->simcards[i]->cdev, board->simcards[i]->devno, 1)) < 0) {
			log(KERN_ERR, "cdev_add() error=%d\n", rc);
			goto sbpca_board_probe_error;
		}
		board->simcards[i]->cdev_req = 1;
		snprintf(devname, sizeof(devname), "simbank!sim%d", MINOR(board->simcards[i]->devno));
		if (!(board->simcards[i]->device = CLASS_DEV_CREATE(sbpca_class, board->simcards[i]->devno, NULL, devname))) {
			log(KERN_ERR, "class_dev_create() error\n");
			goto sbpca_board_probe_error;
		}
		// init control register
		num = i % 100;
		if (i / 1) {
			board->simcards[i]->control_register = &board->control_space[1][num];
			mem = board->iomem_base[0];
		} else {
			board->simcards[i]->control_register = &board->control_space[0][num];
			mem = board->iomem_base[1];
		}
		
		board->simcards[i]->control_register->bits.reset = 1;
		iowrite32(board->simcards[i]->control_register->full, mem + SBPCA_SIM_CTRL_BASE + num * 4);
	}
	init_timer(&board->poll_timer);
	board->poll_timer.function = sbpca_poll_proc;
	board->poll_timer.data = (unsigned long)board;
	board->poll_timer.expires = jiffies + 1;
	add_timer(&board->poll_timer);

	pci_read_config_byte(pdev, PCI_REVISION_ID, &rev);
	verbose("board (rev.%u) found\n", (unsigned int)rev);

	pci_set_drvdata(pdev, board);
	return 0;

sbpca_board_probe_error:
	if (board) {
		// release simcard data
		for (i = 0; i < SBPCA_BOARD_SIM_MAXCOUNT; i++) {
			if (board->simcards[i]) {
				if (board->simcards[i]->device) {
					CLASS_DEV_DESTROY(sbpca_class, board->simcards[i]->devno);
				}
				if (board->simcards[i]->cdev_req) {
					cdev_del(&board->simcards[i]->cdev);
				}
				kfree(board->simcards[i]);
			}
		}
		for (i = 0; i < 2; i++) {
			if (board->iomem_req[i]) {
				pci_release_region(pdev, i);
			}
			if (board->iomem_base[i]) {
				pci_iounmap(pdev, board->iomem_base[i]);
			}
		}
		kfree(board);
	}
	return rc;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static void sbpca_board_remove(struct pci_dev *pdev)
#else
static void __devexit sbpca_board_remove(struct pci_dev *pdev)
#endif
{
	size_t i;
	struct sbpca_board *board = pci_get_drvdata(pdev);

	if (board) {
		del_timer_sync(&board->poll_timer);
		// release simcard data
		for (i = 0; i < SBPCA_BOARD_SIM_MAXCOUNT; i++) {
			if (board->simcards[i]) {
				if (board->simcards[i]->device) {
					CLASS_DEV_DESTROY(sbpca_class, board->simcards[i]->devno);
				}
				if (board->simcards[i]->cdev_req) {
					cdev_del(&board->simcards[i]->cdev);
				}
				kfree(board->simcards[i]);
			}
		}
		for (i = 0; i < 2; i++) {
			if (board->iomem_req[i]) {
				pci_release_region(pdev, i);
			}
			if (board->iomem_base[i]) {
				pci_iounmap(pdev, board->iomem_base[i]);
			}
		}
		kfree(board);
	}

	verbose("board removed\n");
}

static struct pci_driver sbpca_driver = {
	.name = "sbpca",
	.id_table = sbpca_board_id_table,
	.probe = sbpca_board_probe,
	.remove = sbpca_board_remove,
};

static int __init sbpca_init(void)
{
	dev_t devno;
	int sbpca_major_reg = 0;
	int rc = 0;

	verbose("loading version \"%s\"...\n", SIMBANK_LINUX_VERSION);

	if (HZ < 1000) {
		log(KERN_ERR, "HZ=%d to low, set to >=1000\n", HZ);
		goto sbpca_init_error;
	}

	if ((sim_start < 0) || (sim_start >= SBPCA_BOARD_SIM_MAXCOUNT)) {
		sim_start = 0;
	}
	if ((sim_end < sim_start) || (sim_end >= SBPCA_BOARD_SIM_MAXCOUNT)) {
		sim_end = SBPCA_BOARD_SIM_MAXCOUNT - 1;
	}
	if ((sim_start != 0) || (sim_end != (SBPCA_BOARD_SIM_MAXCOUNT - 1))) {
		verbose("SIM start=%d, end=%d\n", sim_start, sim_end);
	}

	// Registering sbpca device class
	if (!(sbpca_class = class_create(THIS_MODULE, "sbpca"))) {
		log(KERN_ERR, "class_create() error\n");
		goto sbpca_init_error;
	}
	// Register char device region
	if (sbpca_major) {
		devno = MKDEV(sbpca_major, 0);
		rc = register_chrdev_region(devno, SBPCA_DEVICE_MAXCOUNT, "sbpca");
	} else {
		rc = alloc_chrdev_region(&devno, 0, SBPCA_DEVICE_MAXCOUNT, "sbpca");
		if (rc >= 0) {
			sbpca_major = MAJOR(devno);
		}
	}
	if (rc < 0) {
		log(KERN_ERR, "register chrdev region error=%d\n", rc);
		goto sbpca_init_error;
	}
	debug("sbpca major=%d\n", sbpca_major);
	sbpca_major_reg = 1;

	if ((rc = pci_register_driver(&sbpca_driver)) < 0) {
		log(KERN_ERR, "can't register pci driver\n");
		goto sbpca_init_error;
	}

	verbose("loaded successfull\n");
	return 0;

sbpca_init_error:
	// Unregister sbpca device region
	if (sbpca_major_reg) {
		unregister_chrdev_region(MKDEV(sbpca_major, 0), SBPCA_DEVICE_MAXCOUNT);
	}
	// Destroy sbpca device class
	if (sbpca_class) {
		class_destroy(sbpca_class);
	}
	verbose("loading failed\n");
	return rc;
}

static void __exit sbpca_exit(void)
{
	// Unregister pci driver
	pci_unregister_driver(&sbpca_driver);
	// Unregister sbpca device region
	unregister_chrdev_region(MKDEV(sbpca_major, 0), SBPCA_DEVICE_MAXCOUNT);
	// Destroy sbpca device class
	class_destroy(sbpca_class);

	verbose("stopped\n");
}

module_init(sbpca_init);
module_exit(sbpca_exit);

/* end of sbpca-base.c */
