#include <linux/platform_device.h>
#include "ba315_base.h"

static
struct platform_driver ba315_driver = {
    .remove = __devexit_p(ba315_remove_ctrl),
	.driver = {
		.name   = "p6-nand",
		.owner  = THIS_MODULE
	}
};

static
int __init
ba315_init_drv(void)
{
    return platform_driver_probe(&ba315_driver, &ba315_probe_ctrl);
}

static
void __exit
ba315_fini_drv(void)
{
    platform_driver_unregister(&ba315_driver);
}

module_init(ba315_init_drv);
module_exit(ba315_fini_drv);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Boirie Gregor <gregor.boirie@parrot.com>");
MODULE_DESCRIPTION("Platform driver for Barco-silex BA315 NAND flash controller");
MODULE_ALIAS("platform:p6-nand");
