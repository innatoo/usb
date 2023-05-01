/*****************************************************************************
*
*      quancom.c --
*
*      QUANCOM USB Driver ( http://www.quancom.de )
*	
*      Copyright (C) 2004 Michael Reimer <linux@quancom.de>
*      Copyright (C) 2003 Marcus Maul <Marcus.Maul@post.rwth-aachen.de>
*      Based on the cyport.c dabusb.c and cytherm.c driver, see below.
*
*      ========
*      cytherm.c  --  Cypress' Thermometer USB Driver
*
*                     Copyright (C) 2001 Romain LIEVIN <rlievin@mail.com>
*
*      dabusb.c   --  By D.Fliegl
*
*      cyport.c   --  By Marcus Maul <Marcus.Maul@post.rwth-aachen.de>
*
*      quancom.c  --  By Michael Reimer <linux@quancom.de>
*
*      ========
* 
* Ported to the 2.6.x kernel by Marc van Selm <selm AT cistron DOT nl>
* Note:
*   1. I only made this port because I wanted to run a USBREL8 with 
*      a 2.6.7 kernel for personal use. This is the only platfor
*      that I tested with.
*   2. This code is NOT actively maintained by me but Quancom has taken
*      up the maintanance of the 2.6.x port.
*
*   3. Tested under SUSE 9.3 Kernel 2.6.11 by QUANCOM
*   4. Added support for USBOPTOREL32 and USBOPTO32IO
*   5. Added support for USBOPTO64IN, USBOPTO64OUT and USBREL64
*
* The 2.6.x port is based on the USB Skeleton driver -1.1 (see the
* copyright notice below)
*
*      ========
*
* USB Skeleton driver - 1.1
*
* Copyright (C) 2001-2003 Greg Kroah-Hartman (greg@kroah.com)
*
*      This program is free software; you can redistribute it and/or
*      modify it under the terms of the GNU General Public License as
*      published by the Free Software Foundation, version 2.
*
*
* This driver is to be used as a skeleton driver to be able to create a
* USB driver quickly.  The design of it is based on the usb-serial and
* dc2xx drivers.
*
* Thanks to Oliver Neukum, David Brownell, and Alan Stern for their help
* in debugging this driver.
*
*      ========
*
*      This driver handles the following QUANCOM USB devices:
*
*      USBWDOG1 USB Watchdog
*              - http://www.quancom.de/qprod01/eng/pb/usbwdog1.htm
*
*      USBWDOG2 USB Watchdog
*              - http://www.quancom.de/qprod01/eng/pb/usbwdog2.htm
*
*      USBWDOG3 USB Watchdog
*              - http://www.quancom.de/qprod01/eng/pb/usbwdog3.htm
*
*      USBOPTOREL8 USB Relay Module
*              - http://www.quancom.de/qprod01/eng/pb/usboptorel8.htm
*              - 8 Relays + 8 Opto Inputs
*              - Input change detection
*              - Watchdog ( deactivates outputs in case of software crash )
*              - DIP Switch for the module address
*              - Up to 15 USB devices can be used parallel
*
*      USBOPTO8IO USB I/O Module
*              - http://www.quancom.de/qprod01/eng/pb/usbopto8io.htm
*              - 8 Opto Outputs + 8 Opto Inputs
*              - Input change detection
*              - HW Watchdog ( deactivates outputs in case of software crash )
*              - DIP Switch for the module address
*              - Up to 15 USB devices can be used parallel
*
*      USBOPTOREL16 USB Relay Module
*              - http://www.quancom.de/qprod01/eng/pb/usboptorel16.htm
*              - 16 Relays + 16 Opto Inputs
*              - Input change detection
*              - Watchdog ( deactivates outputs in case of software crash )
*              - DIP Switch for the module address
*              - Up to 15 USB devices can be used parallel
*
*      USBOPTO16IO USB I/O Module
*              - http://www.quancom.de/qprod01/eng/pb/usbopto16io.htm
*              - 16 Opto Outputs + 16 Opto Inputs
*              - Input change detection
*              - HW Watchdog ( deactivates outputs in case of software crash )
*              - DIP Switch for the module address
*              - Up to 15 USB devices can be used parallel
*
*      USBOPTOREL32 USB Relay Module
*              - http://www.quancom.de/qprod01/eng/pb/usboptorel32.htm
*              - 32 Relays + 32 Opto Inputs
*              - Input change detection
*              - Watchdog ( deactivates outputs in case of software crash )
*              - DIP Switch for the module address
*              - Up to 15 USB devices can be used parallel
*
*      USBOPTO32IO USB I/O Module
*              - http://www.quancom.de/qprod01/eng/pb/usbopto32io.htm
*              - 32 Opto Outputs + 32 Opto Inputs
*              - Input change detection
*              - HW Watchdog ( deactivates outputs in case of software crash )
*              - DIP Switch for the module address
*              - Up to 15 USB devices can be used parallel
*
*      USBOPTO64IN USB I Module
*              - http://www.quancom.de/qprod01/eng/pb/usbopto64in.htm
*              - 64 Opto Inputs
*              - Input change detection
*              - HW Watchdog ( deactivates outputs in case of software crash )
*              - DIP Switch for the module address
*              - Up to 15 USB devices can be used parallel
*
*      USBOPTO64OUT USB O Module
*              - http://www.quancom.de/qprod01/eng/pb/usbopto64out.htm
*              - 64 Opto Outputs
*              - HW Watchdog ( deactivates outputs in case of software crash )
*              - DIP Switch for the module address
*              - Up to 15 USB devices can be used parallel
*
*      USBREL64 USB Relay Module
*              - http://www.quancom.de/qprod01/eng/pb/usbrel64.htm
*              - 64 Relays
*              - Watchdog ( deactivates outputs in case of software crash )
*              - DIP Switch for the module address
*              - Up to 15 USB devices can be used parallel
*
*
*      USBREL8 Mini USB Relay Module with 8 Relays
*              - http://www.quancom.de/qprod01/eng/pb/usb_relay.htm
*
*      USBOPTO8 Mini USB Optocoupler Module with 8 Inputs
*              - http://www.quancom.de/qprod01/eng/pb/usbopto8.htm
*
*      USBREL8LC Mini USB Relay Module with 8 Relays
*              - http://www.quancom.de/qprod01/eng/pb/usbrel8lc.htm
*
*      USBOPTO8LC Mini USB Optocoupler Module with 8 Inputs
*              - http://www.quancom.de/qprod01/eng/pb/usbopto8lc.htm
*
*      USBTASTMAUS Keyboard and Mouse Lock Device with Opto Input ( i.e. Cardreader ) 
*           for Public Internet Terminals
*              - Keyboard and Mouse are routed to the host PC if
*                a valid ID-Card is inserted in the Cardreader
*              - Removing the card from the Cardreader will
*                disconnect the mouse and keyboard
*
*      USBGPIB GPIB / IEEE-488.2 Interface
*              - http://www.quancom.de/qprod01/eng/pb/usb_gpib_1.htm
*              - Integrated Hardware FIFO
*              - Controller for 15 GPIB devices
*
*      USBAD8DAC2 AD / DA conversion Board
*              - http://www.quancom.de/qprod01/eng/pb/usb_ad8dac2.htm
*
*      ========
*
*  This file is part of the Quancom driver module for Linux 2.6.x
*
*  The Quancom driver for Linux 2.6.x is free software; you can
*  redistribute it and/or modify it under the terms of the GNU General
*  Public License as published by the Free Software Foundation; either
*  version 2 of the License, or (at your option) any later version.
*
*  The Quancom driver for Linux 2.6.x is distributed in the hope that
*  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
*  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
*  the GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with Foobar; if not, write to the Free Software
*  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
*  ======================================================================
*
*      quancom_usb.c ChangeLog :
*
*      * 20030121 (v0.01) : Device probing, configuration, claiming.
*      * 20040503 (v1.01) : Modified the driver for the QUANCOM USB Modules
*      * 20040816 (v1.01-2.6-port) : Ported the driver towards the 2.6.7
*                                    kernel. Minor clean-up of code.
*      * 20050523 (v1.02) : Readme, Makefile, Added support for USBOPTOREL32 
*                           and USBOPTO32IO
*      * 20060419 (v1.03) : Readme, Makefile, Added support for USBOPTO64IN, 
*                           USBOPTO64OUT, USBREL64 IDs
*      * 20060427 (v1.1.0) : Readme, Makefile, Added support for USBGPIB and
*                            USBAD8DAC2 (not just IDs). minor code fixes
*                            Linux versioning scheme
*      * 20060428 (v1.2.0) : Readme, additional /dev/watchdog device for 
*                            first USBWDOG*, real library QLIB with Makefile
*      * 20060608 (v1.3.0) : retrigger watchdog cards with writing on device 
*                            node
*      * 20060706 (v1.3.1) : minor code fix because of simpler struct usb_driver
*      * 20060724 (v1.3.2) : added some device IDs
*
*/


/*****************************************************************************/

//#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
//#include <linux/smp_lock.h>    /* For (un)lock_kernel */
#include <linux/completion.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/uaccess.h>

#include "quancom.h"

#define VENDOR_QUANCOM 0xa7c

#if 0
#ifdef CONFIG_USB_DEBUG
        static int debug = 1;
#else
        static int debug;
#endif
#endif

static int debug = 1;

#define MODNAME     "qusbdrv"

#undef dbg
#define dbg(format, arg...)                                     \
        do {                                                    \
                if (debug)                                      \
                        printk (KERN_DEBUG "%s: " format "\n",  \
                                MODNAME , ## arg);              \
        } while (0)
//if 0
#if 1
#define err(format, arg...) printk(KERN_ERR "%s: " format "\n", MODNAME , ## arg)
#define info(format, arg...) printk(KERN_INFO "%s: " format "\n", MODNAME , ## arg)
#define warn(format, arg...) printk(KERN_WARNING "%s: " format "\n", MODNAME , ## arg)
#endif

/* static quancom_t quancom[MAXPORTS]; */
static int maxtime = TIMEOUT;

/* Structure to hold all of our device specific stuff */
struct usb_quancom{
    struct usb_device    *udev;			    /* save off the usb device pointer */
    struct usb_interface *interface;		/* the interface for this device */
    unsigned char		  minor;			/* the starting minor number for this device */
    unsigned char		  num_ports;		/* the number of ports this device has */
    char			      num_interrupt_in;	/* number of interrupt in endpoints we have */
    char			      num_bulk_in;		/* number of bulk in endpoints we have */
    char			      num_bulk_out;		/* number of bulk out endpoints we have */

    unsigned char        *bulk_in_buffer;		 /* the buffer to receive data */
    size_t			      bulk_in_size;		     /* the size of the receive buffer */
    __u8			      bulk_in_endpointAddr;	 /* the address of the bulk in endpoint */

    unsigned char        *bulk_out_buffer;	     /* the buffer to send data */
    size_t			      bulk_out_size;		 /* the size of the send buffer */
    struct urb           *write_urb;		     /* the urb used to send data */
    __u8			      bulk_out_endpointAddr; /* the address of the bulk out endpoint */
    atomic_t		      write_busy;		     /* true iff write urb is busy */
    struct completion	  write_finished;		 /* wait for the write to finish */

    int			          open;			/* if the port is open or not */
    int			          present;		/* if the device is not disconnected */
    struct semaphore	  sem;			/* locks this structure */
    int			          deviceid;
};

/* Function definitions */
static int quancom_fops_release(struct inode *inode, struct file *file);
static int quancom_probe (struct usb_interface *interface, const struct usb_device_id *id);
static void quancom_disconnect (struct usb_interface *inteface);
static inline void quancom_delete (struct usb_quancom *dev);
static int quancom_fops_open(struct inode *inode, struct file *file);
static ssize_t quancom_fops_read(struct file *file, char *buf, size_t count, loff_t * ppos);
static ssize_t quancom_fops_write(struct file *file, const char *buf, size_t count, loff_t * ppos);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
static long quancom_fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
#else
static int quancom_fops_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
#endif

/* Supported device IDs */
static struct usb_device_id quancom_ids [] = {
    { USB_DEVICE (VENDOR_QUANCOM, 0x1) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x2) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x3) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x5) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x6) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x7) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x0) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x9) },
    { USB_DEVICE (VENDOR_QUANCOM, 0xA) },
    { USB_DEVICE (VENDOR_QUANCOM, 0xB) },
    { USB_DEVICE (VENDOR_QUANCOM, 0xC) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x10) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x11) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x12) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x13) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x14) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x15) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x16) },
    { USB_DEVICE (VENDOR_QUANCOM, 0xFE) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x0100) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x0101) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x0102) },
    { USB_DEVICE (VENDOR_QUANCOM, 0x0103) },
    { }                                                /* Terminating
    entry */
};


MODULE_DEVICE_TABLE (usb, quancom_ids);

/*
* File operations needed when we register this driver.
* This assumes that this driver NEEDS file operations,
* of course, which means that the driver is expected
* to have a node in the /dev directory. If the USB
* device were for a network interface then the driver
* would use "struct net_driver" instead, and a serial
* device would use "struct tty_driver".
*/
static struct file_operations quancom_fops = {
        /*
    * The owner field is part of the module-locking
    * mechanism. The idea is that the kernel knows
    * which module to increment the use-counter of
    * BEFORE it calls the device's open() function.
    * This also means that the kernel can decrement
    * the use-counter again before calling release()
    * or should the open() function fail.
        */
    .owner =	THIS_MODULE,
    .read =		quancom_fops_read,
    .write =	quancom_fops_write,
    .unlocked_ioctl = quancom_fops_ioctl,
/*.ioctl =	quancom_fops_ioctl,*/
    .open =		quancom_fops_open,
    .release =	quancom_fops_release,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,75)
/* 
* usb class driver info in order to get a minor number from the usb core,
* and to have the device registered with devfs and the driver core
*/
static struct usb_class_driver quancom_class = {
    .name =		"usb/qusb%d",
    .fops =		&quancom_fops,
    .minor_base =	TIGLUSB_MINOR,
};
#else
/* the global usb devfs handle */
extern devfs_handle_t usb_devfs_handle;
#endif

/* usb specific object needed to register this driver with the usb subsystem */
static struct usb_driver quancom_driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
    .owner =        THIS_MODULE,
#endif
    .name =         MODNAME,
    .probe =        quancom_probe,
    .disconnect =   quancom_disconnect,
    .id_table =     quancom_ids,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,75)
    .minor =        TIGLUSB_MINOR,
    .fops =         &quancom_fops,
#endif
};

/* for watchdog module */
static int watchdog_count = 0;
static int watchdog_real_minor = 0;
static struct miscdevice watchdog_dev = {
    .minor = WATCHDOG_MINOR,
    .name  = "watchdog",
    .fops  = &quancom_fops
};


/* prevent races between open() and disconnect() */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
    static DECLARE_MUTEX (disconnect_sem);
#else
    static DEFINE_SEMAPHORE(disconnect_sem);
#endif


/* --------------------------------------------------------------------- */

/* Re-initialize device */
static int clear_device(struct usb_device *udev)
{
    if (usb_reset_configuration (udev) < 0)
    {
        printk("quancom driver: clear_device failed\n");
        return -1;
    }

    return 0;
}

/* Issue a specific vendor command */

int vendor_command(struct usb_device *usbdev, __u8 request, __u16 value, __u16 index, void *iobuf)
{
    int retval;
    unsigned int pipe;
unsigned char *p = (unsigned char*)iobuf;

    pipe = usb_rcvctrlpipe(usbdev, 0); /* upstream */
	printk ("request : %x   value : %x   index : %x",request,value,index);
    retval = usb_control_msg(usbdev, pipe, request, 
            USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_OTHER, value, index, iobuf, 8,
            HZ/(maxtime/10)); //100

printk("iobuf: %x %x %x %x %x %x %x %x\n", p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
printk("result: %i\n", retval);

    if(retval < 0) {
        printk("quancom driver: vendor_command usb_control_msg error 0x%x\n",
                retval);
    }

    return retval;
}

int ep0_command(struct usb_device *udev, ep0_request_t *args)
{
    int retval = 0;
    unsigned int pipe;
    printk("passato 1");
    if (args->Request == (__u8) QUANCOM_RXCMD) {
        pipe = usb_rcvctrlpipe(udev, 0);
        printk(" req : %x  val : %x  index : %x   size  : %x \n",args->Request, args->Value,args->Index,args->Size);
	printk("buffer  %s",args->Buffer);        
/* FIXME do some sanity checks to not overflow the buffer */
        retval = usb_control_msg(udev, pipe, args->Request, 
                args->RequestTypeReservedBits, args->Value, args->Index,
                args->Buffer, args->Size, HZ*10);

     } else if ((args->Request == QUANCOM_TXCMD_BUFF) || 
            (args->Request == QUANCOM_TXCMD_NOBUFF)) {
        
        pipe = usb_sndctrlpipe(udev, 0);

        retval = usb_control_msg(udev, pipe, args->Request, 
                args->RequestTypeReservedBits, args->Value, args->Index,
                args->Buffer, args->Size, HZ*10);
    }

    if(retval < 0) {
        printk("quancom driver: ep0_command usb_control_msg error 0x%x\n",
                -retval);
    }

    return retval;
}

int ep_bulk(struct usb_device *udev, bulk_ep_request_t *args)
{
    int retval = 0;
    unsigned int pipe;
    char *buff;

    buff = kmalloc((args->bufflen + 1) * sizeof(char), GFP_KERNEL);

    if (!buff) {
        err("out of memory");
        return -ENOMEM;
    }

    printk("direction: %d, ep: %d, bufflen: %ld",
            args->direction, args->epNumber, args->bufflen);
   printk("passato 2");
    if (args->direction) {
        pipe = usb_sndbulkpipe(udev, args->epNumber);

        if (copy_from_user(buff, (void *)args->buffer, args->bufflen)) {
            retval = -EFAULT;
            goto out;
        }

        buff[args->bufflen] = 0;
        dbg("buffer: %d %d %d%d %s", buff[0], buff[1], buff[2], buff[3],
                buff+4);

        retval = usb_bulk_msg(udev, pipe, buff, args->bufflen, 
                (int *) &args->nOutput, HZ*10);
    } else {
        pipe = usb_rcvbulkpipe(udev, args->epNumber);

        /* FIXME do some sanity checks to not overflow the buffer */
        retval = usb_bulk_msg(udev, pipe, buff, args->bufflen, 
                (int *) &args->nOutput, HZ*10);

        buff[args->bufflen] = 0;
        dbg("buffer: %s", buff);

        if (copy_to_user((void *)args->buffer, buff, args->bufflen)) {
            retval = -EFAULT;
            goto out;
        }
    }

    if(retval < 0) {
        printk("quancom driver: ep_bulk usb_bulk_msg error 0x%x\n",
                -retval);
    }

    dbg("nOutput: %ld", args->nOutput);

out:
    kfree(buff);
    return retval;
}


/* --------------------------------------------------------------------- */

/*
* file operations-related functions
*/

static ssize_t quancom_fops_read(struct file *file, char *buf, size_t count, loff_t * ppos)
{
    struct usb_quancom *dev;
    ssize_t ret = 0;
    int bytes_to_read = 0;
    int bytes_read = 0;
    int result = 0;
    char buffer[8];
    int temp, sign;
    char iobuf[8];

    printk("quancom driver: read\n");

    dev= (struct usb_quancom *) file->private_data;

    /* lock this object */
    down(&dev->sem);

    /* Verify is the device wasn't unplugged */
    if (!dev->present)  {
        ret = -ENODEV;
        goto out;
    }

    if (*ppos)
        return -ESPIPE;

    bytes_to_read = (count >= CTRL_RCV_MAX) ? CTRL_RCV_MAX : count;
    temp = sign = 0;

    result = vendor_command(dev->udev, 0x02, 0x33, 0, iobuf);
    temp = iobuf[1];
    result = vendor_command(dev->udev, 0x02, 0x34, 0, iobuf);
    sign = iobuf[1];
    sprintf(buffer, "%c%i.%i", sign ? '-' : '+', temp >> 1, 5*(temp - ((temp >> 1) << 1)));
    if (result == -ETIMEDOUT)
    { /* NAK */
        ret = result;
        goto out;
    }
    else if (result == -EPIPE)
    { /* STALL -- shouldn't happen */
        ret = result;
        goto out;
    }
    else if (result < 0)
    { /* We should not get any I/O errors */
        printk("quancom driver: funky result: %d. Please notify the maintainer.",result);
        ret = -EIO;
        goto out;
    }

    printk("quancom driver: read data from device done");

    bytes_read = strlen(buffer);
    if (copy_to_user(buf, buffer, bytes_read))
    {
        ret = -EFAULT;
        goto out;
    }

    ret = bytes_read;

out:
    /* unlock and return */
    up(&dev->sem);
    return ret;
}


static ssize_t quancom_fops_write(struct file *file, const char *buf, size_t count, loff_t * ppos)
{
    struct usb_quancom *dev;
    ssize_t ret = 0;
    int result = 0;
    char iobuf[8];

 printk("quancom driver: write\n");

    dev = (struct usb_quancom *) file->private_data;

    /* can't seek on this device */
    if (ppos != &file->f_pos)
        return -ESPIPE;

    /* lock the object */
    down(&dev->sem);

    /* verify that the device wasn't unplugged */
    if (!dev->present)  {
        ret=-ENODEV;
        goto out;
    }

    if ((dev->deviceid == USBWDOG1) || (dev->deviceid == USBWDOG2) 
            || (dev->deviceid == USBWDOG3)) {
        /* retrigger watchdog card */
        if (buf) {
            result = vendor_command(dev->udev, 16, 0, 252, iobuf);

            if (result == -ETIMEDOUT)
            { 
                /* NAK */
                ret = result;
                goto out;
            }
            else if (result == -EPIPE)
            { 
                /* STALL -- shouldn't happen */
                ret = result;
                goto out;
            }
            else if (result < 0)
            { 
                /* We should not get any I/O errors */
                err("%s - funky result: %d. Please notify the maintainer.",
                        __FUNCTION__, result);
                ret = -EIO;
                goto out;
            }
            ret = result;
        }
    }


// printk("quancom driver: read data to device done");

out:
    /* unlock and return */
    up(&dev->sem);
    return ret;
}


static long quancom_fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg)

{
    struct usb_quancom *dev;
    int ret = 0;
    unsigned char iobuf[8];
    int result = 0;
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
    //lock_kernel();
    #elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
    #lock_kernel();
    #else
    
    #endif
    dev = (struct usb_quancom *) file->private_data;

    /* lock this object */
    down(&dev->sem);
    printk("fops ioctl");
    /* verify if the device is not unplugged */
    if (!dev->present)  {
        ret = -ENODEV;
        goto out;
    }

    /* Needed?? */
    if (_IOC_TYPE(cmd) != 'N')        /* new-style ioctl number */
    {	
        err("%s: ioctl failed", __FUNCTION__);
        ret = -EINVAL;
        goto out;
    }

 printk("_ioctl: cmd = %i\n", cmd);

    switch (cmd)
    {
        case 0:
            break;

        case IOCTL_QUANCOM_REQUEST:
        {
            lIn_t args;

            printk("IOCTL_QUANCOM_REQUEST\n");

            if (copy_from_user(&args, (void *)arg, sizeof(args)))
            {
                ret = -EFAULT;
                goto out;
            }

            printk("_ioctl: function = 0x%02x\n", args.bFunction);

            switch(args.bFunction)
            {
                case QUANCOM_READ_RAM:
                    result = vendor_command(dev->udev, 17, args.bValue1, 0, iobuf);
                    printk ("read ram val arg %x",args.bValue1);
			args.bFunction = (result > 0) ? 0 : 1;
                    args.bValue1 = iobuf[1];
                    break;

                case QUANCOM_WRITE_RAM:
                  printk("write ram \n");
			  result = vendor_command(dev->udev, 16, args.bValue1, args.bValue2,iobuf);
                    args.bFunction = (result > 0) ? 0 : 1;
                    break;

                case QUANCOM_READ_DEVICEID:
			printk("read device\n");
                    args.bValue1 = dev->deviceid;
                    break;

            }

            if (copy_to_user((void *)arg, &args, sizeof(args)))
            {
                ret = -EFAULT;
                goto out;
            }
        }
        break;

        case IOCTL_QUANCOM_EP0_REQUEST:
        {
            ep0_request_t args;
             printk("\n");
		printk("1   %x   %x   %x   %x   %x   %s   %x \n",args.Request,args.Value,args.Index,args.Size,
								args.RequestTypeReservedBits,args.Buffer,args.result);
            if (copy_from_user(&args, (void *)arg, sizeof(args))) {
                ret = -EFAULT;
                goto out;
            }
            printk("2   %x    %x   %x    %x    %x     %s   %x   \n",args.Request,args.Value,args.Index,args.Size,
								args.RequestTypeReservedBits,args.Buffer,args.result);
            result = ep0_command(dev->udev, &args);
            args.result = result;
		printk("3   %x   %x   %x   %x   %x   %s   %x  \n",args.Request,args.Value,args.Index,args.Size,
									args.RequestTypeReservedBits,args.Buffer,args.result);
            if (copy_to_user((void *)arg, &args, sizeof(args))) {
                ret = -EFAULT;
                goto out;
            }
printk("usccito da copy to user");
        }
        break;

        case IOCTL_QUANCOM_BULK_EP_REQUEST:
        {
            bulk_ep_request_t args;

            if (copy_from_user(&args, (void *)arg, sizeof(args))) {
                ret = -EFAULT;
                goto out;
            }

            result = ep_bulk(dev->udev, &args);
            args.result = result;

            if (copy_to_user((void *)arg, &args, sizeof(args))) {
                ret = -EFAULT;
                goto out;
            }
        }
        break;

        case IOCTL_QUANCOM_GET_DESCRIPTOR:
            ret = -ENOIOCTLCMD;
            break;

        case IOCTL_QUANCOM_GET_STATUS:
            ret = -ENOIOCTLCMD;
            break;

        case IOCTL_QUANCOM_RESET_DEVICE:
            result = clear_device(dev->udev);
            if (result < 0) 
                ret = -EIO;
            break;

        case IOCTL_QUANCOM_RECONFIGURE:
            ret = -ENOIOCTLCMD;
            break;

        case IOCTL_QUANCOM_UNCONFIGURE:
            ret = -ENOIOCTLCMD;
            break;

        case O_NONBLOCK:
            file->f_flags |= O_NONBLOCK;
            break;

        default:
            ret = -ENOIOCTLCMD;
            break;
    }

out:
    /* unlock device */
    up (&dev->sem);
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
    //unlock_kernel();
    #elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36)
    #unlock_kernel();
    #else
    
    #endif
    return ret;
}

/*
static loff_t quancom_fops_lseek (struct file *file, loff_t offset,
                                int origin)
{
return -ESPIPE;
}
*/

static int quancom_fops_open(struct inode *inode, struct file *file)
{
    struct usb_quancom *dev = NULL;
    struct usb_interface *interface;
    int minor;
    int retval=0;

    if (imajor(inode) == MISC_MAJOR) 
        minor = watchdog_real_minor;
    else
        minor=iminor(inode);

// printk("quancom: _open, minor = %d\n", minor);

    if (minor < TIGLUSB_MINOR || minor >= (TIGLUSB_MINOR + MAXPORTS))
        return -EIO;

    /* prevent disconnects */
    down(&disconnect_sem);

    /* Get interface/device */
    interface=usb_find_interface(&quancom_driver, minor);
    if (!interface)  {
        err("%s - error, can't find device for minor %d",
            __FUNCTION__, minor);
        retval=-ENODEV;
        goto exit_no_device;
    }
    dev=usb_get_intfdata(interface);
    if (!dev)  {
        err("%s - error, can't get device structure for minor %d",
            __FUNCTION__, minor);
        retval=-ENODEV;
        goto exit_no_device;
    }

    /* Lock this device */
    down(&dev->sem);

    /* Increment our usage count for the driver */
    ++dev->open;

    /* Save our object in the file's private structure */
    file->private_data=dev;

    /* Unlock this device */
    up(&dev->sem);

exit_no_device:
    up(&disconnect_sem);
    return retval;
}


/*	quancom_delete
*/
static inline void quancom_delete (struct usb_quancom *dev)
{
        /* did not allocate so should also not free
    kfree (dev->bulk_in_buffer);
        
    usb_buffer_free (dev->udev, dev->bulk_out_size,
    dev->bulk_out_buffer,
    dev->write_urb->transfer_dma);
    usb_free_urb (dev->write_urb);
        */
    printk("quancom driver: Driver data structure release\n");
    kfree (dev);
}

static int quancom_fops_release(struct inode *inode, struct file *file)
{
    struct usb_quancom *s;
    int retval=0;

    s = (struct usb_quancom *) file->private_data;

    if (s == NULL) {
        dbg ("%s - object is NULL", __FUNCTION__);
        return -ENODEV;
    }

    dbg("%s - minor %d", __FUNCTION__, s->minor);

    /* lock our device */
    down (&s->sem);

    if (s->open <= 0) {
        dbg ("%s - device not opened", __FUNCTION__);
        retval = -ENODEV;
        up(&s->sem);
    } else {

        --s->open;
        up (&s->sem);

        if (!s->present && !s->open) {
            /* the device was unplugged before the file was released */
            quancom_delete (s);
            return 0;
        }
    }

    return retval;
}


/* --------------------------------------------------------------------- */
static int quancom_probe (struct usb_interface *interface, const struct usb_device_id *id)
{
    struct usb_device *usbdev = interface_to_usbdev(interface);
    /* int minor; */
    struct usb_quancom *s = NULL;
    /* struct usb_host_interface *iface_desc; */
    int deviceid = 0;
    int retval = -ENOMEM;

    printk("quancom driver: probing vendor id 0x%x, device id 0x%x\n",
            usbdev->descriptor.idVendor,usbdev->descriptor.idProduct);

    if (usbdev->descriptor.idVendor != VENDOR_QUANCOM)
    {       
        printk("quancom driver: this driver does not support vendor id 0x%x\n",
            usbdev->descriptor.idVendor);
        return -ENODEV;
    }

    if (usbdev->descriptor.bNumConfigurations != 1)
    {
        printk("quancom driver: Error: usbdev->descriptor.bNumConfigurations %d != 1\n", 
                usbdev->descriptor.bNumConfigurations);
        return -ENODEV;
    }

    switch (usbdev -> descriptor.idProduct)
    {
        case 0x0001:
            deviceid = USBWDOG1;
            printk("quancom driver: QUANCOM USBWDOG detected.\n");
            break;
        case 0x0002:
            deviceid= USBOPTOREL16;
            printk("quancom driver: QUANCOM USBOPTOREL16 detected.\n");
            break;
        case 0x0003:
            deviceid = USBOPTO16IO;
            printk("quancom driver: QUANCOM USBOPTO16IO detected.\n");
            break;
        case 0x0005:
            deviceid = USBWDOG1;
            printk("quancom driver: QUANCOM USBWDOG1 detected.\n");
            break;
        case 0x0006:
            deviceid = USBWDOG2;
            printk("quancom driver: QUANCOM USBWDOG2 detected.\n");
            break;
        case 0x0007:
            deviceid = USBWDOG3;
            printk("quancom driver: QUANCOM USBWDOG3 detected.\n");
            break;
        case 0x0009:
            deviceid = USBREL8;
            printk("quancom driver: QUANCOM USBREL8 detected.\n");
            break;
        case 0x000A:
            deviceid = USBOPTO8;
            printk("quancom driver: QUANCOM USBOPTO8 detected.\n");
            break;
        case 0x000B:
            deviceid = USBREL8LC;
            printk("quancom driver: QUANCOM USBREL8LC detected.\n");
            break;
        case 0x000C:
            deviceid = USBOPTO8LC;
            printk("quancom driver: QUANCOM USBOPTO8LC detected.\n");
            break;
        case 0x0010:
            deviceid = USBOPTOREL32;
            printk("quancom driver: QUANCOM USBOPTOREL32 detected.\n");
            break;
        case 0x0011:
            deviceid = USBOPTO32IO;
            printk("quancom driver: QUANCOM USBOPTO32IO detected.\n");
            break;
        case 0x0012:
            deviceid = USBOPTO64IN;
            printk("quancom driver: QUANCOM USBOPTO64IN detected.\n");
            break;
        case 0x0013:
            deviceid = USBOPTO64OUT;
            printk("quancom driver: QUANCOM USBOPTO64OUT detected.\n");
            break;
        case 0x0014:
            deviceid = USBREL64;
            printk("quancom driver: QUANCOM USBREL64 detected.\n");
            break;
        case 0x0015:
            deviceid = USBOPTOREL8;
            printk("quancom driver: QUANCOM USBOPTOREL8 detected.\n");
            break;
        case 0x0016:
            deviceid = USBOPTO8IO;
            printk("quancom driver: QUANCOM USBOPTO8IO detected.\n");
            break;
        case 0x00FE:
            deviceid = TASTMAUS1;
            printk("quancom driver: QUANCOM TASTMAUS1 detected.\n");
            break;
        case 0x0100:
            deviceid = USBGPIB;
            printk("quancom driver: QUANCOM USBGPIB detected.\n");
            break;
        case 0x0101:
            deviceid = USBAD8DAC2LC;
            printk("quancom driver: QUANCOM USBAD8DAC2/LC detected.\n");
            break;
        case 0x0102:
            deviceid = USBAD8DAC214;
            printk("quancom driver: QUANCOM USBAD8DAC2/14 detected.\n");
            break;
        case 0x0103:
            deviceid = USBTTL24;
            printk("quancom driver: QUANCOM USBTTL24 detected.\n");
            break;

        default:
            return 0;

    }


    /* allocate memory for our device state and initialize it */
    s = kmalloc (sizeof(struct usb_quancom), GFP_KERNEL);
    if (s == NULL) {
        err ("Out of memory");
        return -ENOMEM;
    }
    memset (s, 0x00, sizeof (*s));

    s->deviceid = deviceid;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
    init_MUTEX (&s->sem);
#else
    sema_init(&s->sem, 1);
#endif

    s->udev = usbdev;
    s->interface = interface;
    /* iface_desc = &interface->altsetting[0]; */

    /* allow device read, write and ioctl */
    s->present = 1;

    /* we can register the device now, as it is ready */
    usb_set_intfdata (interface, s);
    retval = usb_register_dev (interface, &quancom_class);
    if (retval) {
        /* something prevented us from registering this driver */
        err ("Not able to get a minor for this device.");
        usb_set_intfdata (interface, NULL);

        quancom_delete(s);
    } else {

        s->minor = interface->minor;

        printk("quancom driver: registering: major = %d, minor = %d, node = qusb%d\n",
                USB_MAJOR, s->minor, s->minor - TIGLUSB_MINOR );

        // usb_devfs_handle
        /*
           s->devfs = devfs_register(NULL, nodename, DEVFS_FL_DEFAULT, USB_MAJOR,(TIGLUSB_MINOR + s->minor), S_IFCHR |  S_IRUSR | S_IWUSR | S_IRGRP |S_IWGRP, &quancom_fops, NULL);
           */
        /* Display firmware version */

        printk("quancom driver: firmware version %i.%02x\n", usbdev->descriptor.bcdDevice >> 8,
                usbdev->descriptor.bcdDevice & 0xff);
    }

    if ((deviceid == USBWDOG1) || (deviceid == USBWDOG2) || (deviceid == USBWDOG3)) {
        if (watchdog_count) {
            printk("quancom driver: just one module can be /dev/watchdog\n");
            watchdog_count++;
        } else {
            if (misc_register(&watchdog_dev)) {
                printk("quancom driver: couldn't register as /dev/watchdog\n");
            } else {
                printk("quancom driver: additionally registered as /dev/watchdog\n");
                watchdog_count++;
                watchdog_real_minor = s->minor;
            }
        }
    }

    return retval;
}

static void quancom_disconnect (struct usb_interface *interface)
{

    struct usb_quancom *s;
    int minor;

    printk("quancom driver: try to disconnect\n");
        
    /* prevent races with open() */
    down (&disconnect_sem);

    s = usb_get_intfdata (interface);
    usb_set_intfdata (interface, NULL);

    down (&s->sem);

    minor = s->minor;

    /* give back our minor */
    usb_deregister_dev (interface, &quancom_class);

    if ((s->deviceid == USBWDOG1) || (s->deviceid == USBWDOG2) 
            || (s->deviceid == USBWDOG3)) {
        if (watchdog_count) {
            watchdog_count--;
            if (!watchdog_count) { /* no more watchdogs */
                misc_deregister(&watchdog_dev);
            }
        }
    }

    /* prevent device read, write and ioctl */
    s->present = 0;

    up (&s->sem);

    /* if the device is opened, skel_release will clean this up */
    if (!s->open)
        quancom_delete (s);

    up (&disconnect_sem);

    info("quancom driver: #%d now disconnected", minor);  

    printk("quancom driver: device disconnected\n");

}


/* --------------------------------------------------------------------- */

static int __init quancom_init (void)
{
    int result;

    /* register device */
    result=usb_register(&quancom_driver);

    if (result)
    {	
        err("quancom.c usb_register failed. Error number %d", result);
    }
    else
    {
        printk ("quancom driver: quancom_init: driver registered\n");
        info(DRIVER_VERSION ":" DRIVER_DESC);
    }

    return result;
}

static void __exit quancom_cleanup (void)
{
    printk ("quancom driver: quancom_cleanup\n");
    usb_deregister (&quancom_driver);
}

/* --------------------------------------------------------------------- */

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

module_init(quancom_init);
module_exit(quancom_cleanup);

/* --------------------------------------------------------------------- */
