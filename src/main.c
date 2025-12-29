/*
 * Roadio Firmware - Main Application
 * Target: ST Nucleo-H723ZG
 * Zephyr RTOS v4.3.0
 *
 * Hardware:
 * - SN65HVD230 CAN transceiver -> FDCAN1
 * - W25Q128 SPI flash (16 MB) -> SPI1
 * - LM2596 buck converter (power supply)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>

LOG_MODULE_REGISTER(roadio, LOG_LEVEL_INF);

// CAN device - defined in devicetree overlay
#define CAN_NODE DT_ALIAS(canbus0)

// External flash device
#define FLASH_NODE DT_ALIAS(spi_flash0)

// CAN message filter for OBD-II (example: filter for diagnostic responses)
static const struct can_filter obd_filter = {
	.flags = 0,       /* Standard 11-bit ID (CAN_FILTER_IDE not set) */
	.id = 0x7E8,      /* OBD-II standard response ID */
	.mask = 0x7F8,    /* Match 0x7E8-0x7EF */
};

// CAN callback for received messages
static void can_rx_callback(const struct device *dev, struct can_frame *frame,
			     void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	LOG_INF("CAN RX: ID=0x%03x DLC=%d", frame->id, frame->dlc);
	LOG_HEXDUMP_INF(frame->data, frame->dlc, "Data:");
}

// CAN state change callback
static void can_state_change_callback(const struct device *dev,
				      enum can_state state,
				      struct can_bus_err_cnt err_cnt,
				      void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	LOG_INF("CAN state changed: %d (TX err: %d, RX err: %d)",
		state, err_cnt.tx_err_cnt, err_cnt.rx_err_cnt);

	if (state == CAN_STATE_BUS_OFF) {
		LOG_ERR("CAN bus-off! Manual recovery required (stop/start).");
		// Note: can_recover() may not be available in all drivers
	}
}

// Initialize CAN interface
static int init_can(void)
{
	const struct device *can_dev = DEVICE_DT_GET(CAN_NODE);
	int ret;

	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN device not ready");
		return -ENODEV;
	}

	LOG_INF("Initializing CAN on %s", can_dev->name);

	// Set CAN mode to normal (loopback mode for testing without bus)
#ifdef CONFIG_CAN_LOOPBACK
	ret = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
	LOG_INF("CAN loopback mode enabled for testing");
#else
	ret = can_set_mode(can_dev, CAN_MODE_NORMAL);
#endif

	if (ret != 0) {
		LOG_ERR("Failed to set CAN mode: %d", ret);
		return ret;
	}

	// Register state change callback
	can_set_state_change_callback(can_dev, can_state_change_callback, NULL);

	// Add receive filter for OBD-II responses
	ret = can_add_rx_filter(can_dev, can_rx_callback, NULL, &obd_filter);
	if (ret < 0) {
		LOG_ERR("Failed to add CAN RX filter: %d", ret);
		return ret;
	}

	// Start CAN controller
	ret = can_start(can_dev);
	if (ret != 0) {
		LOG_ERR("Failed to start CAN: %d", ret);
		return ret;
	}

	LOG_INF("CAN initialized successfully");
	return 0;
}

// Initialize external flash
static int init_flash(void)
{
#if DT_HAS_COMPAT_STATUS_OKAY(jedec_spi_nor)
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
	int ret;

	if (!device_is_ready(flash_dev)) {
		LOG_WRN("External flash device not ready (check wiring)");
		return -ENODEV;
	}

	uint64_t flash_size = 0;

	LOG_INF("External flash initialized: %s", flash_dev->name);

	ret = flash_get_size(flash_dev, &flash_size);
	if (ret == 0) {
		LOG_INF("Flash size: %llu bytes", flash_size);
	}

	// Test flash access (optional)
	uint8_t test_data[16];
	ret = flash_read(flash_dev, 0, test_data, sizeof(test_data));
	if (ret == 0) {
		LOG_HEXDUMP_DBG(test_data, sizeof(test_data), "Flash read @ 0x00:");
	}

	return 0;
#else
	LOG_INF("External flash not configured in devicetree (see overlay file)");
	return -ENOTSUP;
#endif
}

// Main application entry point
int main(void)
{
	int ret;

	uint32_t version = sys_kernel_version_get();

	LOG_INF("==========================================");
	LOG_INF("Roadio Firmware Starting...");
	LOG_INF("Board: Nucleo-H723ZG");
	LOG_INF("Zephyr Version: %u.%u.%u",
		SYS_KERNEL_VER_MAJOR(version),
		SYS_KERNEL_VER_MINOR(version),
		SYS_KERNEL_VER_PATCHLEVEL(version));
	LOG_INF("==========================================");

	// Initialize CAN bus
	ret = init_can();
	if (ret != 0) {
		LOG_ERR("CAN initialization failed: %d", ret);
		// Continue anyway for now
	}

	// Initialize external flash
	ret = init_flash();
	if (ret != 0) {
		LOG_WRN("Flash initialization failed or not available: %d", ret);
	}

	// Main application loop
	LOG_INF("Entering main loop...");

	while (1) {
		// Heartbeat
		LOG_DBG("Heartbeat");

		/* TODO: Add application logic here:
		 * - Read OBD-II data from CAN bus
		 * - Process vehicle data
		 * - Store data to flash
		 * - Communicate over network (Wi-Fi)
		 * - etc.
		 */

		k_sleep(K_SECONDS(5));
	}

	return 0;
}
