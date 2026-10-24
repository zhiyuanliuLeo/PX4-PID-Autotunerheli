#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>

#include <drivers/drv_hrt.h>

#include <stdlib.h>

#include <nuttx/i2c/i2c_master.h>

extern "C" __EXPORT int vl53addr_main(int argc, char *argv[]);

int vl53addr_main(int argc, char *argv[])
{
	int bus = 2;
	int old_addr = 0x29;
	int new_addr = 0x2A;

	int myoptind = 1;
	int ch = 0;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:a:n:", &myoptind, &myoptarg)) != EOF) {

		switch (ch) {

		case 'b':
			bus = strtol(myoptarg, nullptr, 0);
			break;

		case 'a':
			old_addr = strtol(myoptarg, nullptr, 0);
			break;

		case 'n':
			new_addr = strtol(myoptarg, nullptr, 0);
			break;

		default:
			PX4_ERR("usage: vl53addr -b <bus> -a <old> -n <new>");
			return PX4_ERROR;
		}
	}

	struct i2c_master_s *i2c_dev = px4_i2cbus_initialize(bus);

	if (i2c_dev == nullptr) {
		PX4_ERR("bad bus %d", bus);
		return PX4_ERROR;
	}

	uint8_t tx[3];

	tx[0] = 0x00;
	tx[1] = 0x01;
	tx[2] = new_addr;

	i2c_msg_s msgv[1] {};

	msgv[0].frequency = 400000;
	msgv[0].addr = old_addr;
	msgv[0].flags = 0;
	msgv[0].buffer = tx;
	msgv[0].length = sizeof(tx);

	int ret = I2C_TRANSFER(i2c_dev, &msgv[0], 1);

	px4_i2cbus_uninitialize(i2c_dev);

	if (ret != PX4_OK) {
		PX4_ERR("I2C_TRANSFER failed: %d", ret);
		return PX4_ERROR;
	}

	PX4_INFO("changed 0x%02x -> 0x%02x", old_addr, new_addr);

	return PX4_OK;
}
