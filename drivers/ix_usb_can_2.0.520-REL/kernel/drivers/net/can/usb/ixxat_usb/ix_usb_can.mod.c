#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_MITIGATION_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif



static const char ____versions[]
__used __section("__versions") =
	"\x18\x00\x00\x00\xac\xd5\xa6\x21"
	"usb_alloc_urb\0\0\0"
	"\x18\x00\x00\x00\x16\xe8\x92\x1b"
	"usb_anchor_urb\0\0"
	"\x18\x00\x00\x00\x1e\x83\x0c\x65"
	"usb_free_urb\0\0\0\0"
	"\x1c\x00\x00\x00\x0c\xfc\x27\xe7"
	"can_free_echo_skb\0\0\0"
	"\x1c\x00\x00\x00\x91\xc9\xc5\x52"
	"__kmalloc_noprof\0\0\0\0"
	"\x1c\x00\x00\x00\x47\xf1\xb3\x1c"
	"can_put_echo_skb\0\0\0\0"
	"\x14\x00\x00\x00\xd7\x95\x00\x13"
	"consume_skb\0"
	"\x1c\x00\x00\x00\xbd\x31\xdf\xec"
	"can_get_echo_skb\0\0\0\0"
	"\x18\x00\x00\x00\x58\xba\x4d\x53"
	"alloc_canfd_skb\0"
	"\x1c\x00\x00\x00\xe8\x75\xa3\xef"
	"usb_register_driver\0"
	"\x10\x00\x00\x00\xba\x0c\x7a\x03"
	"kfree\0\0\0"
	"\x20\x00\x00\x00\x0b\x05\xdb\x34"
	"_raw_spin_lock_irqsave\0\0"
	"\x14\x00\x00\x00\xb6\xdd\x4f\xb9"
	"open_candev\0"
	"\x14\x00\x00\x00\x25\x71\xff\x98"
	"netdev_err\0\0"
	"\x14\x00\x00\x00\xbb\x6d\xfb\xbd"
	"__fentry__\0\0"
	"\x18\x00\x00\x00\x3d\x46\xb2\x33"
	"dev_addr_mod\0\0\0\0"
	"\x24\x00\x00\x00\x97\x70\x48\x65"
	"__x86_indirect_thunk_rax\0\0\0\0"
	"\x10\x00\x00\x00\x7e\x3a\x2c\x12"
	"_printk\0"
	"\x1c\x00\x00\x00\xcb\xf6\xfd\xf0"
	"__stack_chk_fail\0\0\0\0"
	"\x18\x00\x00\x00\x2d\x62\x3b\x65"
	"alloc_can_skb\0\0\0"
	"\x20\x00\x00\x00\xe1\x8a\x2c\x96"
	"usb_kill_anchored_urbs\0\0"
	"\x1c\x00\x00\x00\x2d\xfe\xf4\x1c"
	"netif_device_detach\0"
	"\x1c\x00\x00\x00\xea\xd9\xcb\x4b"
	"unregister_candev\0\0\0"
	"\x18\x00\x00\x00\xf3\xe8\xf3\x11"
	"usb_submit_urb\0\0"
	"\x14\x00\x00\x00\xb0\x75\x9e\x71"
	"_dev_info\0\0\0"
	"\x18\x00\x00\x00\x7a\xb0\x2b\x0f"
	"can_change_mtu\0\0"
	"\x28\x00\x00\x00\xb3\x1c\xa2\x87"
	"__ubsan_handle_out_of_bounds\0\0\0\0"
	"\x20\x00\x00\x00\xb6\x2b\xc6\x78"
	"can_dropped_invalid_skb\0"
	"\x14\x00\x00\x00\x38\x24\xc3\xb0"
	"_dev_err\0\0\0\0"
	"\x14\x00\x00\x00\x68\x98\x46\x1e"
	"free_candev\0"
	"\x1c\x00\x00\x00\x16\xc9\xf0\x2e"
	"sk_skb_reason_drop\0\0"
	"\x1c\x00\x00\x00\x77\xed\xb1\xf8"
	"alloc_candev_mqs\0\0\0\0"
	"\x18\x00\x00\x00\xe6\xed\x47\x60"
	"can_fd_len2dlc\0\0"
	"\x1c\x00\x00\x00\x63\xa5\x03\x4c"
	"random_kmalloc_seed\0"
	"\x18\x00\x00\x00\x6d\xa1\x26\x97"
	"netdev_printk\0\0\0"
	"\x18\x00\x00\x00\xcd\x71\x07\x58"
	"usb_control_msg\0"
	"\x18\x00\x00\x00\x87\x93\x2d\xf1"
	"can_fd_dlc2len\0\0"
	"\x1c\x00\x00\x00\x62\x43\x8a\xfa"
	"sysfs_create_group\0\0"
	"\x18\x00\x00\x00\x55\xd5\xa4\xe2"
	"usb_deregister\0\0"
	"\x18\x00\x00\x00\xb5\x79\xca\x75"
	"__fortify_panic\0"
	"\x24\x00\x00\x00\x70\xce\x5c\xd3"
	"_raw_spin_unlock_irqrestore\0"
	"\x1c\x00\x00\x00\x7c\x17\x1d\xf2"
	"netif_tx_wake_queue\0"
	"\x18\x00\x00\x00\xa5\x46\x09\x72"
	"close_candev\0\0\0\0"
	"\x1c\x00\x00\x00\xca\x39\x82\x5b"
	"__x86_return_thunk\0\0"
	"\x20\x00\x00\x00\x54\xea\xa5\xd9"
	"__init_waitqueue_head\0\0\0"
	"\x14\x00\x00\x00\x8a\x32\x40\xe8"
	"netif_rx\0\0\0\0"
	"\x14\x00\x00\x00\x93\x19\xd1\xe8"
	"can_bus_off\0"
	"\x1c\x00\x00\x00\x1d\x95\x91\xa0"
	"usb_unanchor_urb\0\0\0\0"
	"\x10\x00\x00\x00\xa6\x50\xba\x15"
	"jiffies\0"
	"\x1c\x00\x00\x00\xb6\x2c\xb1\xfc"
	"sysfs_remove_group\0\0"
	"\x10\x00\x00\x00\xfd\xf9\x3f\x3c"
	"sprintf\0"
	"\x2c\x00\x00\x00\x61\xe5\x48\xa6"
	"__ubsan_handle_shift_out_of_bounds\0\0"
	"\x20\x00\x00\x00\xee\xfb\xb4\x10"
	"__kmalloc_cache_noprof\0\0"
	"\x18\x00\x00\x00\x18\x01\x47\x56"
	"__warn_printk\0\0\0"
	"\x1c\x00\x00\x00\xda\x4b\xed\x92"
	"alloc_can_err_skb\0\0\0"
	"\x2c\x00\x00\x00\xc6\xfa\xb1\x54"
	"__ubsan_handle_load_invalid_value\0\0\0"
	"\x18\x00\x00\x00\x46\xc4\x9a\x9a"
	"register_candev\0"
	"\x10\x00\x00\x00\xf9\x82\xa4\xf9"
	"msleep\0\0"
	"\x20\x00\x00\x00\x12\xda\xf0\xc4"
	"ktime_get_with_offset\0\0\0"
	"\x18\x00\x00\x00\xaf\xfc\x16\x7b"
	"kmalloc_caches\0\0"
	"\x14\x00\x00\x00\x1d\xce\x26\x6f"
	"netdev_info\0"
	"\x18\x00\x00\x00\xde\x9f\x8a\x25"
	"module_layout\0\0\0"
	"\x00\x00\x00\x00\x00\x00\x00\x00";

MODULE_INFO(depends, "can-dev");

MODULE_ALIAS("usb:v08D8p0008d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p0009d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p000Ad*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p000Bd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p001Fd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p0014d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p0016d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p0017d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p001Bd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8p001Cd*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8pFF12d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v08D8pFF13d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "17D724792B529134D49739E");
