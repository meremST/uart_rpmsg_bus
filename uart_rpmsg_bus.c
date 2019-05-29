// SPDX-License-Identifier: GPL-2.0
/*
 * Uart based remote processor messaging bus
 *
 * Copyright (C) STMicroelectronics 2019
 * Author: Maxime MERE for STMicroelectronics.
 *
 * Based on virtio_rpmsg_bus.c
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/idr.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/rpmsg.h>
#include <linux/mutex.h>
#include <linux/of_device.h>

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/types.h>

#include <linux/serdev.h>

#include "rpmsg_internal.h"

/**
 * buffer_manager - buffer manager instance structure
 * @rx_raw_tail:	pointer to the last receive raw byte
 * @rx_raw_left:	byte left inside the raw buffer
 * @rx_raw_buffer:	raw buffer data
 * @buf_size:		size of one rx buffer
 * @rbuf:		address of rx buffers
 *
 * This structure aim to manage data before receiving or sending it. The idea is
 * a whole new service wich then is going to be used as adapter to permit
 * multiples rpmsg busses types.
 */
struct buffer_manager {
	unsigned char *rx_raw_tail;		/* pointer to next byte */
	int rx_raw_left;			/* bytes left in queue  */
	unsigned char *rx_raw_buffer;
	bool flag_msg_recv;
	bool first_byte_rx;
	unsigned int buf_size;

	unsigned char msg_type;
	int msg_len;

	void *rbuf;
};

/**
 * struct serdev_info - virtual remote processor state
 * @serdev:	the serdev device
 * @bm:		buffer_manager instance
 * @endpoints:	idr of local endpoints, allows fast retrieval
 * @endpoints_lock: lock of the endpoints set
 * @ns_ept:	the bus's name service endpoint
 *
 * This structure stores the rpmsg state of a given serial device (there might
 * be several rpmsg_bus for each physical remote processor).
 */
struct serdev_info {
	struct serdev_device *serdev;
	struct buffer_manager *bm;
	struct idr endpoints;
	struct mutex endpoints_lock;
	struct rpmsg_endpoint *ns_ept;
};

/* The rpmsg feature */
#define RPMSG_F_NS_SUPPORT	(1) /* we supports name service notifications */

/**
 * struct rpmsg_hdr - common header for all rpmsg messages
 * @src: source address
 * @dst: destination address
 * @reserved: reserved for future use
 * @len: length of payload (in bytes)
 * @flags: message flags
 * @data: @len bytes of message payload data
 *
 * Every message sent(/received) on the rpmsg bus begins with this header.
 */
struct rpmsg_hdr {
	u32 src;
	u32 dst;
	u32 reserved;
	u16 len;
	u16 flags;
	u8 data[0];
} __packed;

/**
 * struct serdev_rproc_hdr - header for all serial communications
 * @magic_number: magic number
 * @len: length of the message (in bytes)
 * @msg_type: the kind of message that will be send
 *
 * Every message sent(/received) with serial communication use this header
 */
struct serdev_rproc_hdr {
	u16 magic_number; //0xbe57 #beST
	u16 len;
	u8 msg_type;
} __packed;

/*The magic number used to identify rpmsg communications through uart*/
#define	SERDEV_RPMSG_MAGIC_NUMBER	(0xbe57)

/**
 * enum serdev_rproc_type - types of messages that can be sent through uart
 *
 * @SERDEV_RPROC_RPMSG: standard rpmsg
 */
enum serdev_rproc_type {
	SERDEV_RPROC_RPMSG	= 0,
};

/**
 * struct rpmsg_ns_msg - dynamic name service announcement message
 * @name: name of remote service that is published
 * @addr: address of remote service that is published
 * @flags: indicates whether service is created or destroyed
 *
 * This message is sent across to publish a new service, or announce
 * about its removal. When we receive these messages, an appropriate
 * rpmsg channel (i.e device) is created/destroyed. In turn, the ->probe()
 * or ->remove() handler of the appropriate rpmsg driver will be invoked
 * (if/as-soon-as one is registered).
 */
struct rpmsg_ns_msg {
	char name[RPMSG_NAME_SIZE];
	u32 addr;
	u32 flags;
} __packed;

/**
 * enum rpmsg_ns_flags - dynamic name service announcement flags
 *
 * @RPMSG_NS_CREATE: a new remote service was just created
 * @RPMSG_NS_DESTROY: a known remote service was just destroyed
 */
enum rpmsg_ns_flags {
	RPMSG_NS_CREATE		= 0,
	RPMSG_NS_DESTROY	= 1,
};

/**
 * @srp: the remote processor this channel belongs to
 */
struct serdev_rpmsg_channel {
	struct rpmsg_device rpdev;

	struct serdev_info *srp;
};

#define to_serdev_rpmsg_channel(_rpdev) \
	container_of(_rpdev, struct serdev_rpmsg_channel, rpdev)

/*
 * We're allocating buffers of 512 bytes each for communications.
 *
 * Each buffer will have 16 bytes for the msg header and 496 bytes for
 * the payload.
 *
 */
#define MAX_RPMSG_BUF_SIZE	(512)

/*
 * Local addresses are dynamically allocated on-demand.
 * We do not dynamically assign addresses from the low 1024 range,
 * in order to reserve that address range for predefined services.
 */
#define RPMSG_RESERVED_ADDRESSES	(1024)

/* Address 53 is reserved for advertising remote services */
#define RPMSG_NS_ADDR			(53)

static void uart_rpmsg_destroy_ept(struct rpmsg_endpoint *ept);
static int uart_rpmsg_send(struct rpmsg_endpoint *ept, void *data, int len);
static int uart_rpmsg_sendto(struct rpmsg_endpoint *ept, void *data, int len,
			     u32 dst);
static int uart_rpmsg_send_offchannel(struct rpmsg_endpoint *ept, u32 src,
				      u32 dst, void *data, int len);
static int uart_rpmsg_trysend(struct rpmsg_endpoint *ept, void *data, int len);
static int uart_rpmsg_trysendto(struct rpmsg_endpoint *ept, void *data,
				int len, u32 dst);
static int uart_rpmsg_trysend_offchannel(struct rpmsg_endpoint *ept, u32 src,
					 u32 dst, void *data, int len);
static int uart_get_buffer_size(struct rpmsg_endpoint *ept);

static const struct rpmsg_endpoint_ops uart_endpoint_ops = {
	.destroy_ept = uart_rpmsg_destroy_ept,
	.send = uart_rpmsg_send,
	.sendto = uart_rpmsg_sendto,
	.send_offchannel = uart_rpmsg_send_offchannel,
	.trysend = uart_rpmsg_trysend,
	.trysendto = uart_rpmsg_trysendto,
	.trysend_offchannel = uart_rpmsg_trysend_offchannel,
	.get_buffer_size = uart_get_buffer_size,
};

/**
 * __ept_release() - deallocate an rpmsg endpoint
 * @kref: the ept's reference count
 *
 * This function deallocates an ept, and is invoked when its @kref refcount
 * drops to zero.
 *
 * Never invoke this function directly!
 */
static void __ept_release(struct kref *kref)
{
	struct rpmsg_endpoint *ept = container_of(kref, struct rpmsg_endpoint,
						  refcount);
	/*
	 * At this point no one holds a reference to ept anymore,
	 * so we can directly free it
	 */
	kfree(ept);
}

/* for more info, see below documentation of rpmsg_create_ept() */
static struct rpmsg_endpoint *__rpmsg_create_ept(struct serdev_info *srp,
						 struct rpmsg_device *rpdev,
						 rpmsg_rx_cb_t cb,
						 void *priv, u32 addr)
{
	int id_min, id_max, id;
	struct rpmsg_endpoint *ept;
	struct device *dev = rpdev ? &rpdev->dev : &srp->serdev->dev;

	ept = kzalloc(sizeof(*ept), GFP_KERNEL);
	if (!ept)
		return NULL;

	kref_init(&ept->refcount);
	mutex_init(&ept->cb_lock);

	ept->rpdev = rpdev;
	ept->cb = cb;
	ept->priv = priv;
	ept->ops = &uart_endpoint_ops;

	/* do we need to allocate a local address ? */
	if (addr == RPMSG_ADDR_ANY) {
		id_min = RPMSG_RESERVED_ADDRESSES;
		id_max = 0;
	} else {
		id_min = addr;
		id_max = addr + 1;
	}

	mutex_lock(&srp->endpoints_lock);

	/* bind the endpoint to an rpmsg address (and allocate one if needed) */
	id = idr_alloc(&srp->endpoints, ept, id_min, id_max, GFP_KERNEL);
	if (id < 0) {
		dev_err(dev, "idr_alloc failed: %d\n", id);
		goto free_ept;
	}
	ept->addr = id;

	mutex_unlock(&srp->endpoints_lock);

	return ept;

free_ept:
	mutex_unlock(&srp->endpoints_lock);
	kref_put(&ept->refcount, __ept_release);
	return NULL;
}

static struct rpmsg_endpoint *uart_create_ept(struct rpmsg_device *rpdev,
					      rpmsg_rx_cb_t cb,
					      void *priv,
					      struct rpmsg_channel_info chinfo)
{
	struct serdev_rpmsg_channel *sch = to_serdev_rpmsg_channel(rpdev);

	return __rpmsg_create_ept(sch->srp, rpdev, cb, priv, chinfo.src);
}

/**
 * __rpmsg_destroy_ept() - destroy an existing rpmsg endpoint
 * @srp: virtproc which owns this ept
 * @ept: endpoing to destroy
 *
 * An internal function which destroy an ept without assuming it is
 * bound to an rpmsg channel. This is needed for handling the internal
 * name service endpoint, which isn't bound to an rpmsg channel.
 * See also __rpmsg_create_ept().
 */
static void
__rpmsg_destroy_ept(struct serdev_info *srp, struct rpmsg_endpoint *ept)
{
	/* make sure new inbound messages can't find this ept anymore */
	mutex_lock(&srp->endpoints_lock);
	idr_remove(&srp->endpoints, ept->addr);
	mutex_unlock(&srp->endpoints_lock);

	/* make sure in-flight inbound messages won't invoke cb anymore */
	mutex_lock(&ept->cb_lock);
	ept->cb = NULL;
	mutex_unlock(&ept->cb_lock);

	kref_put(&ept->refcount, __ept_release);
}

static void uart_rpmsg_destroy_ept(struct rpmsg_endpoint *ept)
{
	struct serdev_rpmsg_channel *sch = to_serdev_rpmsg_channel(ept->rpdev);

	__rpmsg_destroy_ept(sch->srp, ept);
}

/*announcement are made when a new rpmsg driver is probbed or removed*/
static int uart_rpmsg_announce_create(struct rpmsg_device *rpdev)
{
	//struct serdev_rpmsg_channel *sch = to_serdev_rpmsg_channel(rpdev);
	//struct serdev_info *srp = sch->srp;
	struct device *dev = &rpdev->dev;
	int err = 0;

	/* need to tell remote processor's name service about this channel ? */
	if (rpdev->announce && rpdev->ept && RPMSG_F_NS_SUPPORT) {
		struct rpmsg_ns_msg ns_msg;
		int ns_size = sizeof(ns_msg);

		strncpy(ns_msg.name, rpdev->id.name, RPMSG_NAME_SIZE);
		ns_msg.addr = rpdev->ept->addr;
		ns_msg.flags = RPMSG_NS_CREATE;

		err = rpmsg_sendto(rpdev->ept, &ns_msg, ns_size, RPMSG_NS_ADDR);
		if (err)
			dev_err(dev, "failed to announce service %d\n", err);
	}

	return err;
}

static int uart_rpmsg_announce_destroy(struct rpmsg_device *rpdev)
{
	//struct serdev_rpmsg_channel *sch = to_serdev_rpmsg_channel(rpdev);
	//struct serdev_info *srp = sch->srp;
	struct device *dev = &rpdev->dev;
	int err = 0;

	/* tell remote processor's name service we're removing this channel */
	if (rpdev->announce && rpdev->ept && RPMSG_F_NS_SUPPORT) {
		struct rpmsg_ns_msg nsm;

		strncpy(nsm.name, rpdev->id.name, RPMSG_NAME_SIZE);
		nsm.addr = rpdev->ept->addr;
		nsm.flags = RPMSG_NS_DESTROY;

		err = rpmsg_sendto(rpdev->ept, &nsm, sizeof(nsm), RPMSG_NS_ADDR);
		if (err)
			dev_err(dev, "failed to announce service %d\n", err);
	}

	return err;
}

static const struct rpmsg_device_ops uart_rpmsg_ops = {
	.create_ept = uart_create_ept,
	.announce_create = uart_rpmsg_announce_create,
	.announce_destroy = uart_rpmsg_announce_destroy,
};

static void uart_rpmsg_release_device(struct device *dev)
{
	struct rpmsg_device *rpdev = to_rpmsg_device(dev);
	struct serdev_rpmsg_channel *sch = to_serdev_rpmsg_channel(rpdev);

	kfree(sch);
}

/*
 * create an rpmsg channel using its name and address info.
 * this function will be used to create both static and dynamic
 * channels.
 */
static struct rpmsg_device *rpmsg_create_channel(struct serdev_info *srp,
						 struct rpmsg_channel_info *chinfo)
{
	struct serdev_rpmsg_channel *sch;
	struct rpmsg_device *rpdev;
	struct device *tmp, *dev = &srp->serdev->dev;
	int ret;

	/* make sure a similar channel doesn't already exist */
	tmp = rpmsg_find_device(dev, chinfo);
	if (tmp) {
		/* decrement the matched device's refcount back */
		put_device(tmp);
		dev_err(dev, "channel %s:%x:%x already exist\n",
			chinfo->name, chinfo->src, chinfo->dst);
		return NULL;
	}

	sch = kzalloc(sizeof(*sch), GFP_KERNEL);
	if (!sch)
		return NULL;

	/* Link the channel to our srp */
	sch->srp = srp;

	/* Assign public information to the rpmsg_device */
	rpdev = &sch->rpdev;
	rpdev->src = chinfo->src;
	rpdev->dst = chinfo->dst;
	rpdev->ops = &uart_rpmsg_ops;

	/*
	 * rpmsg server channels has predefined local address (for now),
	 * and their existence needs to be announced remotely
	 */
	rpdev->announce = rpdev->src != RPMSG_ADDR_ANY;

	strncpy(rpdev->id.name, chinfo->name, RPMSG_NAME_SIZE);

	rpdev->dev.parent = &srp->serdev->dev;
	rpdev->dev.release = uart_rpmsg_release_device;
	ret = rpmsg_register_device(rpdev);
	if (ret)
		return NULL;

	return rpdev;
}

/**
 * rpmsg_send_offchannel_raw() - send a message across to the remote processor
 * @rpdev: the rpmsg channel
 * @src: source address
 * @dst: destination address
 * @data: payload of message
 * @len: length of payload
 * @wait: indicates whether caller should block in case no TX buffers available
 *
 * This function is the base implementation for all of the rpmsg sending API.
 *
 * It will send @data of length @len to @dst, and say it's from @src. The
 * message will be sent to the remote processor which the @rpdev channel
 * belongs to.
 *
 * The message is sent via the uart link there is no buffer like in the virtio
 * bus.
 *
 * The @wait value is unused but it still here for compatibility reasons.
 *
 *
 * Normally drivers shouldn't use this function directly; instead, drivers
 * should use the appropriate rpmsg_{try}send{to, _offchannel} API
 * (see include/linux/rpmsg.h).
 *
 * Returns 0 on success and an appropriate error value on failure.
 */
static int rpmsg_send_offchannel_raw(struct rpmsg_device *rpdev,
				     u32 src, u32 dst,
				     void *data, int len, bool wait)
{
	struct serdev_rpmsg_channel *sch = to_serdev_rpmsg_channel(rpdev);
	struct serdev_info *srp = sch->srp;
	struct buffer_manager *bm = srp->bm;
	struct serdev_device *serdev = srp->serdev;
	struct device *dev = &rpdev->dev;
	struct rpmsg_hdr *msg;
	struct serdev_rproc_hdr s_hdr;
	int msg_size = sizeof(struct rpmsg_hdr) + len;
	int s_hdr_len = sizeof(struct serdev_rproc_hdr);
	int ret;

	/* bcasting isn't allowed */
	if (src == RPMSG_ADDR_ANY || dst == RPMSG_ADDR_ANY) {
		dev_err(dev, "invalid addr (src 0x%x, dst 0x%x)\n", src, dst);
		return -EINVAL;
	}

	/*
	 * We currently use fixed-sized buffers, and therefore the payload
	 * length is limited.
	 *
	 * One of the possible improvements here is either to support
	 * user-provided buffers (and then we can also support zero-copy
	 * messaging), or to improve the buffer allocator, to support
	 * variable-length buffer sizes.
	 */
	if (len > bm->buf_size - sizeof(struct rpmsg_hdr)) {
		dev_err(dev, "message is too big (%d)\n", len);
		return -EMSGSIZE;
	}

	/*
	 * With uart there is no shared memory so there is no need of buffers
	 * we're allocating memory to a pointer wich will be used by a serdev
	 * instead. Moreover we don't need to wait for a buffer.
	 */
	msg = kmalloc(msg_size, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	s_hdr.magic_number = SERDEV_RPMSG_MAGIC_NUMBER;
	s_hdr.len = msg_size;
	s_hdr.msg_type = SERDEV_RPROC_RPMSG;

	msg->len = len;
	msg->flags = 0;
	msg->src = src;
	msg->dst = dst;
	msg->reserved = 0;
	memcpy(msg->data, data, len);

	dev_dbg(dev, "TX From 0x%x, To 0x%x, Len %d, Flags %d, Reserved %d\n",
		msg->src, msg->dst, msg->len, msg->flags, msg->reserved);
#if defined(CONFIG_DYNAMIC_DEBUG)
	dynamic_hex_dump("rpmsg_uart TX: ", DUMP_PREFIX_NONE, 16, 1,
			 msg, sizeof(*msg) + msg->len, true);
#endif
	/*
	 * The serdev_rproc_hdr is sent first to indicate what kind and how
	 * much data the remote proc is going to receive.
	 */
	ret = serdev_device_write(serdev, (uint8_t *)&s_hdr, s_hdr_len, 0xFFFF);
	if (ret) {
		dev_err(&serdev->dev, "uart_rpmsg_send failed: %d\n", ret);
		goto err_send;
	}

	/*Then the RPMS itself is sent*/
	ret = serdev_device_write(serdev, (uint8_t *)msg, msg_size, 0xFFFF);
	if (ret) {
		dev_err(&serdev->dev, "uart_rpmsg_send failed: %d\n", ret);
		goto err_send;
	}

err_send:
	kfree(msg);
	return ret;
}

static int uart_rpmsg_send(struct rpmsg_endpoint *ept, void *data, int len)
{
	struct rpmsg_device *rpdev = ept->rpdev;
	u32 src = ept->addr, dst = rpdev->dst;

	return rpmsg_send_offchannel_raw(rpdev, src, dst, data, len, true);
}

static int uart_rpmsg_sendto(struct rpmsg_endpoint *ept, void *data, int len,
			     u32 dst)
{
	struct rpmsg_device *rpdev = ept->rpdev;
	u32 src = ept->addr;

	return rpmsg_send_offchannel_raw(rpdev, src, dst, data, len, true);
}

static int uart_rpmsg_send_offchannel(struct rpmsg_endpoint *ept, u32 src,
				      u32 dst, void *data, int len)
{
	struct rpmsg_device *rpdev = ept->rpdev;

	return rpmsg_send_offchannel_raw(rpdev, src, dst, data, len, true);
}

static int uart_rpmsg_trysend(struct rpmsg_endpoint *ept, void *data, int len)
{
	struct rpmsg_device *rpdev = ept->rpdev;
	u32 src = ept->addr, dst = rpdev->dst;

	return rpmsg_send_offchannel_raw(rpdev, src, dst, data, len, false);
}

static int uart_rpmsg_trysendto(struct rpmsg_endpoint *ept, void *data,
				int len, u32 dst)
{
	struct rpmsg_device *rpdev = ept->rpdev;
	u32 src = ept->addr;

	return rpmsg_send_offchannel_raw(rpdev, src, dst, data, len, false);
}

static int uart_rpmsg_trysend_offchannel(struct rpmsg_endpoint *ept, u32 src,
					 u32 dst, void *data, int len)
{
	struct rpmsg_device *rpdev = ept->rpdev;

	return rpmsg_send_offchannel_raw(rpdev, src, dst, data, len, false);
}

static int uart_get_buffer_size(struct rpmsg_endpoint *ept)
{
	struct rpmsg_device *rpdev = ept->rpdev;
	struct serdev_rpmsg_channel *sch = to_serdev_rpmsg_channel(rpdev);
	struct serdev_info *srp = sch->srp;
	struct buffer_manager *bm = srp->bm;

	return bm->buf_size;
}

/*send data to the appropriate device*/
static int rpmsg_recv_single(struct serdev_info *srp, struct device *dev,
			     struct rpmsg_hdr *msg, unsigned int len)
{
	struct buffer_manager *bm = srp->bm;
	struct rpmsg_endpoint *ept;

	dev_dbg(dev, "From: 0x%x, To: 0x%x, Len: %d, Flags: %d, Reserved: %d\n",
		msg->src, msg->dst, msg->len, msg->flags, msg->reserved);
#if defined(CONFIG_DYNAMIC_DEBUG)
	dynamic_hex_dump("rpmsg_virtio RX: ", DUMP_PREFIX_NONE, 16, 1,
			 msg, sizeof(*msg) + msg->len, true);
#endif

	/*
	 * We currently use fixed-sized buffers, so trivially sanitize
	 * the reported payload length.
	 */
	if (len > bm->buf_size ||
	    msg->len > (len - sizeof(struct rpmsg_hdr))) {
		dev_warn(dev, "inbound msg too big: (%d, %d)\n", len, msg->len);
		return -EINVAL;
	}

	/* use the dst addr to fetch the callback of the appropriate user */
	mutex_lock(&srp->endpoints_lock);

	ept = idr_find(&srp->endpoints, msg->dst);

	/* let's make sure no one deallocates ept while we use it */
	if (ept)
		kref_get(&ept->refcount);

	mutex_unlock(&srp->endpoints_lock);

	if (ept) {
		/* make sure ept->cb doesn't go away while we use it */
		mutex_lock(&ept->cb_lock);

		if (ept->cb)
			ept->cb(ept->rpdev, msg->data, msg->len, ept->priv,
				msg->src);

		mutex_unlock(&ept->cb_lock);

		/* farewell, ept, we don't need you anymore */
		kref_put(&ept->refcount, __ept_release);
	} else {
		dev_warn(dev, "msg received with no recipient\n");
	}

	/*Buffer gesture Here?*/

	return 0;
}

/*Called when rx buffer is ready to be read.*/
static void rpmsg_recv_done(struct serdev_info *srp)
{
	struct buffer_manager *bm = srp->bm;
	struct device dev = srp->serdev->dev;
	struct rpmsg_hdr *msg;
	int err;

	/*transform the void* message in the appropriate type*/
	if (bm->msg_type == SERDEV_RPROC_RPMSG) {
		msg = (struct rpmsg_hdr *)bm->rbuf;

		err = rpmsg_recv_single(srp, &dev, msg, bm->msg_len);
		if (err)
			dev_err(&dev, "recv: something went wrong (%d)", err);

	} else {
		dev_err(&dev, "invalid message type");
	}
}

/*buffer manager functions*/
/**
 * uart_rpmsg_append_data - return the count of data received or an error
 * ...
 */
static int
uart_rpmsg_append_data(struct serdev_info *srp, unsigned char *dat, size_t len)
{
	struct buffer_manager *bm = srp->bm;
	struct serdev_rproc_hdr s_hdr;
	struct device dev = srp->serdev->dev;
	int *byte_left = &bm->rx_raw_left;
	int ret, i;
	int byte_stored = 0;
	size_t old_len = len;

	if (!bm->first_byte_rx) {
		/*searching for the first transmit byte*/
		for (i = 0; i < old_len; i++) {
			if (*((uint16_t *)dat) == 0xbe57) {
			/*when done, we start append process*/
				bm->first_byte_rx = true;
				memcpy(bm->rx_raw_tail, dat, len);
				*byte_left -= len;
				bm->rx_raw_tail += len;
				break;
			} else {
			/*in case non valid data we trash it*/
				len--;
				dat++;
			}
		}
	} else {
		/*usual case of data reception*/
		memcpy(bm->rx_raw_tail, dat, len);
		*byte_left -= len;
		bm->rx_raw_tail += len;
	}

	/*byte_left can't be negative*/
	/* Enter in this situation means that we received more data that the msg
	 * is supposed to have.
	 * For now the driver is not able to receive a second message while the
	 * first one isn't over. 
	 */
	if (*byte_left < 0) {
		dev_err(&dev, "Too much data received");
		ret = -ERANGE;
		goto err_bm;
	}

	byte_stored = bm->buf_size - *byte_left;
	if (!bm->flag_msg_recv && byte_stored >= sizeof(s_hdr)) {
		/*Check in the header to know how many data we're waiting for*/
		memcpy(&s_hdr, bm->rx_raw_buffer, sizeof(s_hdr));
		if (s_hdr.len > bm->buf_size) {
			dev_err(&dev, "msg length too high");
			ret = -ENOMEM;
			goto err_bm;
		}
		/* byte_left will be now equal to the size of the rpmsgmsg minus
		 * the byte already received
		 */
		*byte_left = s_hdr.len - (byte_stored - sizeof(s_hdr));
		bm->msg_len = s_hdr.len;
		bm->msg_type = s_hdr.msg_type;
		bm->flag_msg_recv = true;
	} else if (*byte_left == 0) {
		/*the message is fully received*/
		unsigned char *msg_start = bm->rx_raw_buffer + sizeof(s_hdr);

		/*the message is complete, we can copy it in rbuf*/
		memcpy(bm->rbuf, msg_start, bm->msg_len);

		/*clean buffer manager*/
		bm->flag_msg_recv = false;
		bm->first_byte_rx = false;
		*byte_left = bm->buf_size;

		/*put the tail to the beginning of the buffer*/
		bm->rx_raw_tail = bm->rx_raw_buffer;

		/*notify rpmsg_recv_done*/
		/* for the moment the function is directly called but
		 * it's possible to add it in a workqueue and add a
		 * mutex for the rbuf acces.
		 */
		rpmsg_recv_done(srp);
	}

	return old_len;
err_bm:
	/*reset of buffer manager*/
	bm->flag_msg_recv = false;
	bm->first_byte_rx = false;
	*byte_left = bm->buf_size;
	bm->rx_raw_tail = bm->rx_raw_buffer;

	return ret;
}

/*serdev receive callback called when somme raw data are received*/
static int uart_rpmsg_receive(struct serdev_device *serdev,
			      const unsigned char *data, size_t count)
{
	struct serdev_info *srp = serdev_device_get_drvdata(serdev);

	return uart_rpmsg_append_data(srp, (uint8_t *)data, count);
}

static struct serdev_device_ops rpmsg_serdev_ops = {
	.receive_buf = uart_rpmsg_receive,
	.write_wakeup = serdev_device_write_wakeup,
};

static const struct of_device_id rpmsg_uart_of_match[] = {
	{
	 .compatible = "st,stm32f4-uart", //change it later
	},
	{}
};
MODULE_DEVICE_TABLE(of, rpmsg_uart_of_match);

/* invoked when a name service announcement arrives */
static int rpmsg_ns_cb(struct rpmsg_device *rpdev, void *data, int len,
		       void *priv, u32 src)
{
	struct rpmsg_ns_msg *msg = data;
	struct rpmsg_device *newch;
	struct rpmsg_channel_info chinfo;
	struct serdev_info *srp = priv;
	struct device *dev = &srp->serdev->dev;
	int ret;

#if defined(CONFIG_DYNAMIC_DEBUG)
	dynamic_hex_dump("NS announcement: ", DUMP_PREFIX_NONE, 16, 1,
			 data, len, true);
#endif

	if (len != sizeof(*msg)) {
		dev_err(dev, "malformed ns msg (%d)\n", len);
		return -EINVAL;
	}

	/*
	 * the name service ept does _not_ belong to a real rpmsg channel,
	 * and is handled by the rpmsg bus itself.
	 * for sanity reasons, make sure a valid rpdev has _not_ sneaked
	 * in somehow.
	 */
	if (rpdev) {
		dev_err(dev, "anomaly: ns ept has an rpdev handle\n");
		return -EINVAL;
	}

	/* don't trust the remote processor for null terminating the name */
	msg->name[RPMSG_NAME_SIZE - 1] = '\0';

	dev_info(dev, "%sing channel %s addr 0x%x\n",
		 msg->flags & RPMSG_NS_DESTROY ? "destroy" : "creat",
		 msg->name, msg->addr);

	strncpy(chinfo.name, msg->name, sizeof(chinfo.name));
	chinfo.src = RPMSG_ADDR_ANY;
	chinfo.dst = msg->addr;

	if (msg->flags & RPMSG_NS_DESTROY) {
		ret = rpmsg_unregister_device(&srp->serdev->dev, &chinfo);
		if (ret)
			dev_err(dev, "rpmsg_destroy_channel failed: %d\n", ret);
	} else {
		newch = rpmsg_create_channel(srp, &chinfo);
		if (!newch)
			dev_err(dev, "rpmsg_create_channel failed\n");
	}

	return 0;
}

/*Buffer manager dedicated functions*/

/**
 * buffer_manager_init - initialize buffer manager object
 * @bm:		the buffer manager instance
 * @buf_size:	the internal buffer size
 */
static inline int buffer_manager_init(struct buffer_manager *bm,
			       const int buf_size)
{
	int err = 0;
	uint raw_b_size = buf_size + sizeof(struct serdev_rproc_hdr);

	if (buf_size < 0)
		return -EINVAL;

	/*Memory alloc rx_raw_left and rbuf*/
	bm->rx_raw_buffer = kzalloc(raw_b_size, GFP_KERNEL);
	if (!(bm->rx_raw_buffer))
		return -ENOMEM;

	bm->rbuf = kzalloc(buf_size, GFP_KERNEL);
	if (!(bm->rbuf)) {
		err = -ENOMEM;
		goto err_rbuf;
	}

	bm->buf_size = buf_size;
	bm->flag_msg_recv = false;
	bm->first_byte_rx = false;

	bm->rx_raw_tail = bm->rx_raw_buffer;
	bm->rx_raw_left = bm->buf_size;

	return 0;

err_rbuf:
	kfree(bm->rx_raw_buffer);
	return err;
}

/**
 * buffer_manager_deinit - clear buffer manager object
 * @bm:		the buffer manager instance
 */
static inline void buffer_manager_deinit(struct buffer_manager *bm)
{
	kfree(bm->rx_raw_buffer);
	kfree(bm->rbuf);
}

static int uart_rpmsg_probe(struct serdev_device *serdev)
{
	/*
	 * A faster speed can causes problems:
	 * some unexpected 0 appear in the data randomly.
	 */
	u32 speed = 57600;
	u32 max_speed;
	struct serdev_info *srp;
	struct device_node *np = serdev->dev.of_node;
	struct buffer_manager *bm;
	int err = 0;

	err = of_property_read_u32(np, "max-speed", &max_speed);
	if (err) {
		dev_err(&serdev->dev, "Bad device description\n");
		return err;
	}
	if (speed > max_speed)
		speed = max_speed;

	srp = kzalloc(sizeof(*srp), GFP_KERNEL);
	if (!srp)
		return -ENOMEM;

	srp->serdev = serdev;
	serdev_device_set_drvdata(serdev, srp);

	/*
	 * Instanciation of buf_manager, a new entity in charge of the data
	 * gesture.
	 */
	bm = kzalloc(sizeof(*bm), GFP_KERNEL);
	if (!bm) {
		err = -ENOMEM;
		goto err_bm;
	}

	err = buffer_manager_init(bm, MAX_RPMSG_BUF_SIZE);
	if (err)
		goto err_bm_init;

	srp->bm = bm;

	idr_init(&srp->endpoints);
	mutex_init(&srp->endpoints_lock);

	serdev_device_set_client_ops(serdev, &rpmsg_serdev_ops);
	err = serdev_device_open(serdev);
	if (err) {
		dev_err(&serdev->dev, "Unable to open device\n");
		goto err_serdev_open;
	}

	speed = serdev_device_set_baudrate(serdev, speed);
	dev_info(&serdev->dev, "Using baudrate: %u\n", speed);

	serdev_device_set_parity(serdev, SERDEV_PARITY_ODD);

	serdev_device_set_flow_control(serdev, false);

	/* if supported by the remote processor, enable the name service */
	if (RPMSG_F_NS_SUPPORT) {
		/* a dedicated endpoint handles the name service msgs */
		srp->ns_ept = __rpmsg_create_ept(srp, NULL, rpmsg_ns_cb,
						 srp, RPMSG_NS_ADDR);
		if (!srp->ns_ept) {
			dev_err(&serdev->dev, "failed to create the ns ept\n");
			err = -ENOMEM;
			goto err_ept_create;
		}
	}

	dev_info(&serdev->dev, "rpmsg host is online\n");

	/*succes*/
	return 0;

err_ept_create:
	serdev_device_close(serdev);
err_serdev_open:
	buffer_manager_deinit(bm);
err_bm_init:
	kfree(bm);
err_bm:
	kfree(srp);

	return err;
}

static int rpmsg_remove_device(struct device *dev, void *data)
{
	device_unregister(dev);

	return 0;
}

static void uart_rpmsg_remove(struct serdev_device *serdev)
{
	struct serdev_info *srp = serdev_device_get_drvdata(serdev);
	struct buffer_manager *bm = srp->bm;
	int ret;

	serdev_device_close(serdev);

	ret = device_for_each_child(&serdev->dev, NULL, rpmsg_remove_device);
	if (ret)
		dev_warn(&serdev->dev, "can't remove rpmsg device: %d\n", ret);

	if (srp->ns_ept)
		__rpmsg_destroy_ept(srp, srp->ns_ept);

	/*free memory of buffer manager*/
	buffer_manager_deinit(bm);
	kfree(bm);

	idr_destroy(&srp->endpoints);

	kfree(srp);
}

static struct serdev_device_driver rpmsg_uart_driver = {
	.probe = uart_rpmsg_probe,
	.remove = uart_rpmsg_remove,
	.driver = {
		.name = "uart_rpmsg",
		.of_match_table = of_match_ptr(rpmsg_uart_of_match),
	},
};

int __init rpmsg_uart_init(void)
{
	return serdev_device_driver_register(&rpmsg_uart_driver);
}

void __exit rpmsg_uart_exit(void)
{
	serdev_device_driver_unregister(&rpmsg_uart_driver);
}

subsys_initcall(rpmsg_uart_init);
module_exit(rpmsg_uart_exit);

MODULE_AUTHOR("Maxime Mere <maxime.mere@st.com>");
MODULE_DESCRIPTION("serdev-based remote processor messaging bus");
MODULE_LICENSE("GPL v2");
