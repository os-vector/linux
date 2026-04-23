/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */

#ifndef _UAPI_LINUX_VECTOR_KEYMASTER_H
#define _UAPI_LINUX_VECTOR_KEYMASTER_H

#include <linux/ioctl.h>
#include <linux/types.h>

// sizeof(struct qcom_km_key_blob)
#define VKM_KEYBLOB_LEN		1604

#define VKM_MAX_SIG_LEN		512
#define VKM_MAX_DATA_LEN	256

struct vkm_gen_keyblob {
	__u32	length;
	__u32	reserved;
	__u8	keyblob[VKM_KEYBLOB_LEN];
};

struct vkm_sign {
	// in
	__u32	keyblob_len;
	__u8	keyblob[VKM_KEYBLOB_LEN];

	// data to sign
	__u32	data_len;
	__u8	data[VKM_MAX_DATA_LEN];

	// out
	__u32	sig_len;
	__u8	sig[VKM_MAX_SIG_LEN];
};

#define VKM_IOC_MAGIC		'V'
#define VKM_IOC_GEN_KEYBLOB	_IOR(VKM_IOC_MAGIC, 1, struct vkm_gen_keyblob)
#define VKM_IOC_SIGN		_IOWR(VKM_IOC_MAGIC, 2, struct vkm_sign)

#endif /* _UAPI_LINUX_VECTOR_KEYMASTER_H */
