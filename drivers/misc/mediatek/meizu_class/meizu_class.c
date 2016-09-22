/*
 * The meizu public class for mtk 6755
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include "meizu_class.h"

struct class meizu_class = {
	.name		= "meizu",
};
EXPORT_SYMBOL_GPL(meizu_class);

static int __init meizu_init(void)
{
	int err;

	err = class_register(&meizu_class);
	if (err) {
		pr_err("unable to register meizu class\n");
		return err;
	}

	return 0;
}

static void __exit meizu_exit(void)
{
	class_unregister(&meizu_class);
}

subsys_initcall(meizu_init);
module_exit(meizu_exit);

