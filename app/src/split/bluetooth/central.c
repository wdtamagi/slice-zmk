/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/types.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <sys/byteorder.h>

#include <logging/log.h>

// Register custom logging module. This doesn't share with ZMK so this file is
// easy to debug without being inundated with log messages from the rest of ZMK.
LOG_MODULE_REGISTER(slicemk, 4);

#include <zmk/ble.h>
#include <zmk/split/bluetooth/uuid.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <init.h>

static int start_scan(void);

// Define array for state of peripheral keys. Each array value is initialized to
// 255 inside zmk_split_bt_central_init() as no key is pressed initially. The
// array value corresponds to the peripheral_conns index for pressed keys.
#define POSITION_STATE_DATA_LEN 16
static uint8_t position_state[8 * POSITION_STATE_DATA_LEN];

// Define array for holding peripheral connections.
static struct bt_conn *peripheral_conns[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];

static struct bt_uuid *service_uuid = BT_UUID_DECLARE_128(ZMK_SPLIT_BT_SERVICE_UUID);
static struct bt_uuid *characteristic_uuid =
    BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_POSITION_STATE_UUID);

// Track whether central is currently scanning for peripherals.
static bool is_scanning = false;

static struct bt_gatt_discover_params discover_params[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];
static struct bt_gatt_subscribe_params subscribe_params[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];
static struct bt_gatt_discover_params ccc_params[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];

K_MSGQ_DEFINE(peripheral_event_msgq, sizeof(struct zmk_position_state_changed),
              CONFIG_ZMK_SPLIT_BLE_CENTRAL_POSITION_QUEUE_SIZE, 4);

void peripheral_event_work_callback(struct k_work *work) {
    struct zmk_position_state_changed ev;
    while (k_msgq_get(&peripheral_event_msgq, &ev, K_NO_WAIT) == 0) {
        LOG_DBG("Trigger key position state change for %d", ev.position);
        ZMK_EVENT_RAISE(new_zmk_position_state_changed(ev));
    }
}

K_WORK_DEFINE(peripheral_event_work, peripheral_event_work_callback);

static void split_set_position(int peripheral_id, int position, bool is_pressed) {
    bool was_pressed = position_state[position] != 255;

    // Determine new state for position. Ignore released key if it was pressed
    // on a different peripheral.
    if (is_pressed) {
        position_state[position] = peripheral_id;
    } else if (position_state[position] == peripheral_id) {
        position_state[position] = 255;
    } else {
        return;
    }

    // Handle event if state changed.
    bool changed = was_pressed != is_pressed;
    if (changed) {
        struct zmk_position_state_changed ev = {
            .position = position, .state = is_pressed, .timestamp = k_uptime_get()};
        k_msgq_put(&peripheral_event_msgq, &ev, K_NO_WAIT);
        k_work_submit(&peripheral_event_work);
    }
}

static int split_central_get_peripheral_id(struct bt_conn *conn) {
    for (int i = 0; i < CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS; i++) {
        if (peripheral_conns[i] == conn) {
            return i;
        }
    }
    return -1;
}

static uint8_t split_central_notify_func(struct bt_conn *conn,
                                         struct bt_gatt_subscribe_params *params, const void *data,
                                         uint16_t length) {
    if (!data) {
        LOG_DBG("[UNSUBSCRIBED]");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[NOTIFICATION] data %p length %u", data, length);

    int peripheral_id = split_central_get_peripheral_id(conn);
    if (peripheral_id == -1) {
        LOG_ERR("unable to identify peripheral connection");
        return BT_GATT_ITER_STOP;
    }
    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        for (int j = 0; j < 8; j++) {
            int position = (i * 8) + j;
            bool is_pressed = ((uint8_t *)data)[i] & BIT(j);
            split_set_position(peripheral_id, position, is_pressed);
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t split_central_discovery_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                            struct bt_gatt_discover_params *params) {
	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

	if (!attr) {
		LOG_INF("completed Bluetooth service discovery on %s", log_strdup(addr_str));
		return BT_GATT_ITER_STOP;
	}
    LOG_DBG("handle %u on %s", attr->handle, log_strdup(addr_str));

    int peripheral_id = split_central_get_peripheral_id(conn);
    if (peripheral_id == -1) {
        LOG_ERR("unable to identify peripheral connection");
        return BT_GATT_ITER_STOP;
    }

	if (!bt_uuid_cmp(params->uuid, characteristic_uuid)) {
		uint16_t value_handle = bt_gatt_attr_value_handle(attr);
		struct bt_gatt_subscribe_params *sub_params = &subscribe_params[peripheral_id];

		// Define notification subscription parameters.
		sub_params->disc_params = &ccc_params[peripheral_id];
		sub_params->end_handle = params->end_handle;
		sub_params->value_handle = value_handle;
		sub_params->notify = split_central_notify_func;
		sub_params->value = BT_GATT_CCC_NOTIFY;

		// Subscribe to notifications.
		int err = bt_gatt_subscribe(conn, sub_params);
		if (err == -EALREADY) {
			LOG_INF("notification subscription already exists for %s", log_strdup(addr_str));
		} else if (err) {
			LOG_ERR("failed to subscribe to notifications for %s (err %d)", log_strdup(addr_str), err);
		} else {
			LOG_INF("subscribed to notifications on %s", log_strdup(addr_str));
		}

		return BT_GATT_ITER_CONTINUE;
	}

    return BT_GATT_ITER_STOP;
}

static void split_central_process_connection(struct bt_conn *conn, int peripheral_id) {
    LOG_DBG("Current security for connection: %d", bt_conn_get_security(conn));

	// Set security for connection.
    int err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (err) {
        LOG_ERR("Failed to set security (reason %d)", err);
        return;
    }

	// Define service discovery parameters.
    LOG_DBG("Starting discovery for peripheral #%d", peripheral_id);
	struct bt_gatt_discover_params *disc_params = &discover_params[peripheral_id];
    disc_params->uuid = characteristic_uuid;
    disc_params->start_handle = 0x0001;
    disc_params->end_handle = 0xffff;
    disc_params->func = split_central_discovery_func;
    disc_params->type = BT_GATT_DISCOVER_CHARACTERISTIC;
	err = bt_gatt_discover(conn, disc_params);
    if (err) {
        LOG_ERR("Discover failed(err %d)", err);
        return;
    }

    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);
    LOG_DBG("New connection params: Interval: %d, Latency: %d, PHY: %d", info.le.interval,
            info.le.latency, info.le.phy->rx_phy);
}


static bool split_central_eir_found(const bt_addr_le_t *addr) {
	LOG_DBG("Found the split service");

	// Store peripheral address. If this operation fails, the peripheral
	// must not match any of the known peripherals. Return false to stop
	// parsing advertising data for peripheral.
	int peripheral_i = zmk_ble_put_peripheral_addr(addr);
	if (peripheral_i == -1) {
		return false;
	}

	// Stop scanning so we can connect to the peripheral device.
	LOG_INF("Stopping peripheral scanning");
	is_scanning = false;
	int err = bt_le_scan_stop();
	if (err) {
		LOG_ERR("Stop LE scan failed (err %d)", err);
		return true;
	}

	// Create connection to peripheral with the given connection
	// parameters.
	struct bt_le_conn_param *param = BT_LE_CONN_PARAM(6, 6, 399, 900);
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param,
			&peripheral_conns[peripheral_i]);
	if (err) {
		LOG_ERR("Create conn failed (err %d) (create conn? 0x%04x)", err,
				BT_HCI_OP_LE_CREATE_CONN);
		start_scan();
		return false;
	}

	// TODO TODO TODO doesn't seem to do anything from here. try once zmk is on
	// zephyr 2.7? define callback and double check?
	err = bt_conn_le_phy_update(peripheral_conns[peripheral_i], BT_CONN_LE_PHY_PARAM_2M);
	if (err) {
		LOG_ERR("Update phy conn failed (err %d)", err);
		start_scan();
		return false;
	}

	// Stop processing advertisement data.
	return false;
}

static bool split_central_eir_parse(struct bt_data *data, void *user_data) {
    bt_addr_le_t *addr = user_data;
    LOG_DBG("[AD]: %u data_len %u", data->type, data->data_len);

    switch (data->type) {
    case BT_DATA_UUID128_SOME:
    case BT_DATA_UUID128_ALL:
        if (data->data_len % 16 != 0U) {
            LOG_ERR("AD malformed");
            return true;
        }

        for (int i = 0; i < data->data_len; i += 16) {
            struct bt_uuid_128 uuid;
            if (!bt_uuid_create(&uuid.uuid, &data->data[i], 16)) {
                LOG_ERR("Unable to load UUID");
                continue;
            }

            if (bt_uuid_cmp(&uuid.uuid, service_uuid)) {
                char uuid_str[BT_UUID_STR_LEN];
                char service_uuid_str[BT_UUID_STR_LEN];

                bt_uuid_to_str(&uuid.uuid, uuid_str, sizeof(uuid_str));
                bt_uuid_to_str(service_uuid, service_uuid_str, sizeof(service_uuid_str));
                LOG_DBG("UUID %s does not match split UUID: %s", log_strdup(uuid_str),
                        log_strdup(service_uuid_str));
                continue;
            }

			return split_central_eir_found(addr);
        }
    }

    return true;
}

static bool is_static_peripheral(const bt_addr_le_t *addr) {
	if (strlen(CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_ADDR) == 0) {
		return false;
	}
	bt_addr_le_t peripheral_addr;
	int err = bt_addr_le_from_str(CONFIG_ZMK_SPLIT_BLE_PERIPHERAL_ADDR, "random", &peripheral_addr);
	if (err) {
		return false;
	}
	return !bt_addr_le_cmp(addr, &peripheral_addr);
}

static void split_central_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                                       struct net_buf_simple *ad) {
    char dev[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(addr, dev, sizeof(dev));
    LOG_DBG("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i", log_strdup(dev), type, ad->len,
            rssi);

	// Check if address matches hardcoded peripheral MAC address. This is only
	// used while developing.
	if (is_static_peripheral(addr)) {
		// If no existing connection to peripheral, connect to it. This check is
		// necessary since my development peripheral will advertise
		// continuously. This avoids connecting to it again instead of
		// connecting to the second peripheral.
		struct bt_conn *conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr);
		if (conn == NULL) {
			split_central_eir_found(addr);
		} else {
			bt_conn_unref(conn);
		}
		return;
	}

	// We're only interested in connectable events.
	if (type == BT_GAP_ADV_TYPE_ADV_IND || type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		bt_data_parse(ad, split_central_eir_parse, (void *) addr);
	}
}

static int start_scan(void) {
    // No action is necessary if central is already scanning.
    if (is_scanning) {
        LOG_INF("scanning is already on");
        return 0;
    }

    // If all the devices are connected, there is no need to scan.
    bool has_unconnected = false;
    for (int i = 0; i < CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS; i++) {
        if (peripheral_conns[i] == NULL) {
            has_unconnected = true;
            break;
        }
    }
    if (!has_unconnected) {
        LOG_INF("all devices are connected");
        return 0;
    }

    // Start scanning otherwise.
    is_scanning = true;
    int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, split_central_device_found);
    if (err) {
        LOG_ERR("Scanning failed to start (err %d)", err);
        return err;
    }

    LOG_INF("Scanning successfully started");
    return 0;
}

static void split_central_connected(struct bt_conn *conn, uint8_t conn_err) {
    // Only handle connection if it corresponds to a peripheral.
    for (int i = 0; i < CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS; i++) {
        if (conn == peripheral_conns[i]) {
			// Restart scanning if necessary.
            start_scan();

			char addr[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

            if (conn_err) {
                LOG_ERR("Failed to connect to %s (%u)", log_strdup(addr), conn_err);
                bt_conn_unref(peripheral_conns[i]);
                peripheral_conns[i] = NULL;
            } else {
                LOG_INF("Connected: %s", log_strdup(addr));
                split_central_process_connection(conn, i);
            }
        }
    }
}

void start_scan_work_handler(struct k_work *work) {
	start_scan();
}

K_WORK_DEFINE(start_scan_work, start_scan_work_handler);

void start_scan_timer_handler(struct k_timer *dummy) {
	k_work_submit(&start_scan_work);
}

K_TIMER_DEFINE(start_scan_timer, start_scan_timer_handler, NULL);

static void split_central_disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr_str[BT_ADDR_LE_STR_LEN];
	const bt_addr_le_t *addr = bt_conn_get_dst(conn);
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    LOG_INF("Disconnected: %s (reason %d)", log_strdup(addr_str), reason);

    // Only handle connection if it corresponds to a peripheral.
    for (int i = 0; i < CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS; i++) {
        if (conn == peripheral_conns[i]) {
            // Unset peripheral connection.
            bt_conn_unref(peripheral_conns[i]);
            peripheral_conns[i] = NULL;

            // Release all keys that were held by the peripheral.
            for (int position = 0; position < 8 * POSITION_STATE_DATA_LEN; position++) {
                split_set_position(i, position, false);
            }

            // Start scanning again if necessary. TODO TODO TODO for my personal
			// peripheral, wait a few seconds before restarting scanning. It
			// advertises continuously and this increases the probability that
			// the disconnect is acknowledged there before the reconnect.
			// Otherwise repeatedly connecting and disconnecting occasionally
			// results in an assert, likely due to a Zephyr bug. Reevaluate this
			// edge case after upgrading ZMK to Zephyr 2.7.
			if (is_static_peripheral(addr)) {
				k_timer_start(&start_scan_timer, K_SECONDS(5), K_NO_WAIT);
			} else {
				start_scan();
			}
        }
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = split_central_connected,
    .disconnected = split_central_disconnected,
};

int zmk_split_bt_central_init(const struct device *_arg) {
    // Initialize array for state of peripheral keys. Set array to 255, which
    // signifies that no key is active.
    (void)memset(position_state, 255, 8 * POSITION_STATE_DATA_LEN);

    bt_conn_cb_register(&conn_callbacks);
    return start_scan();
}

SYS_INIT(zmk_split_bt_central_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);
