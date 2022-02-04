/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <device.h>
#include <init.h>

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <settings/settings.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci_err.h>

#if IS_ENABLED(CONFIG_SETTINGS)

#include <settings/settings.h>

#endif

#include <logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/ble.h>
#include <zmk/keys.h>
#include <zmk/split/bluetooth/uuid.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>

static struct bt_conn *auth_passkey_entry_conn;
static uint8_t passkey_entries[6] = {0, 0, 0, 0, 0, 0};
static uint8_t passkey_digit = 0;

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL)
#define PROFILE_COUNT (CONFIG_BT_MAX_PAIRED - CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS)
#else
#define PROFILE_COUNT CONFIG_BT_MAX_PAIRED
#endif

enum advertising_type {
    ZMK_ADV_NONE,
    ZMK_ADV_DIR,
    ZMK_ADV_CONN,
} advertising_status;

#define CURR_ADV(adv) (adv << 4)

#define ZMK_ADV_CONN_NAME                                                                          \
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_ONE_TIME, BT_GAP_ADV_FAST_INT_MIN_2, \
                    BT_GAP_ADV_FAST_INT_MAX_2, NULL)

static struct zmk_ble_profile profiles[PROFILE_COUNT];
static uint8_t active_profile;

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

BUILD_ASSERT(DEVICE_NAME_LEN <= 16, "ERROR: BLE device name is too long. Max length: 16");

#define IS_HOST_PERIPHERAL                                                                         \
    (!IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL))
#define IS_SPLIT_PERIPHERAL                                                                        \
    (IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL))

static const struct bt_data zmk_ble_ad[] = {
#if IS_HOST_PERIPHERAL
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0xC1, 0x03),
#endif
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_SOME,
#if IS_HOST_PERIPHERAL
                  0x12, 0x18, /* HID Service */
#endif
                  0x0f, 0x18 /* Battery Service */
                  ),
#if IS_SPLIT_PERIPHERAL
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, ZMK_SPLIT_BT_SERVICE_UUID)
#endif
};

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL)

static bt_addr_le_t peripheral_addrs[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];

#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL) */

static void raise_profile_changed_event() {
    ZMK_EVENT_RAISE(new_zmk_ble_active_profile_changed((struct zmk_ble_active_profile_changed){
        .index = active_profile, .profile = &profiles[active_profile]}));
}

static void raise_profile_changed_event_callback(struct k_work *work) {
    raise_profile_changed_event();
}

K_WORK_DEFINE(raise_profile_changed_event_work, raise_profile_changed_event_callback);

bool zmk_ble_active_profile_is_open() {
    return !bt_addr_le_cmp(&profiles[active_profile].peer, BT_ADDR_LE_ANY);
}

void set_profile_address(uint8_t index, const bt_addr_le_t *addr) {
    char setting_name[15];
    char addr_str[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    memcpy(&profiles[index].peer, addr, sizeof(bt_addr_le_t));
    sprintf(setting_name, "ble/profiles/%d", index);
    LOG_DBG("Setting profile addr for %s to %s", log_strdup(setting_name), log_strdup(addr_str));
    settings_save_one(setting_name, &profiles[index], sizeof(struct zmk_ble_profile));
    k_work_submit(&raise_profile_changed_event_work);
}

bool zmk_ble_active_profile_is_connected() {
    struct bt_conn *conn;
    bt_addr_le_t *addr = zmk_ble_active_profile_addr();
    if (!bt_addr_le_cmp(addr, BT_ADDR_LE_ANY)) {
        return false;
    } else if ((conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr)) == NULL) {
        return false;
    }

    bt_conn_unref(conn);

    return true;
}

#define CHECKED_ADV_STOP()                                                                         \
    err = bt_le_adv_stop();                                                                        \
    advertising_status = ZMK_ADV_NONE;                                                             \
    if (err) {                                                                                     \
        LOG_ERR("Failed to stop advertising (err %d)", err);                                       \
        return err;                                                                                \
    }

#define CHECKED_DIR_ADV()                                                                          \
    addr = zmk_ble_active_profile_addr();                                                          \
    conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr);                                            \
    if (conn != NULL) { /* TODO: Check status of connection */                                     \
        LOG_DBG("Skipping advertising, profile host is already connected");                        \
        bt_conn_unref(conn);                                                                       \
        return 0;                                                                                  \
    }                                                                                              \
    err = bt_le_adv_start(BT_LE_ADV_CONN_DIR_LOW_DUTY(addr), zmk_ble_ad, ARRAY_SIZE(zmk_ble_ad),   \
                          NULL, 0);                                                                \
    if (err) {                                                                                     \
        LOG_ERR("Advertising failed to start (err %d)", err);                                      \
        return err;                                                                                \
    }                                                                                              \
    advertising_status = ZMK_ADV_DIR;

#define CHECKED_OPEN_ADV()                                                                         \
    err = bt_le_adv_start(ZMK_ADV_CONN_NAME, zmk_ble_ad, ARRAY_SIZE(zmk_ble_ad), NULL, 0);         \
    if (err) {                                                                                     \
        LOG_ERR("Advertising failed to start (err %d)", err);                                      \
        return err;                                                                                \
    }                                                                                              \
    advertising_status = ZMK_ADV_CONN;

int update_advertising() {
    int err = 0;
    bt_addr_le_t *addr;
    struct bt_conn *conn;
    enum advertising_type desired_adv = ZMK_ADV_NONE;

    if (zmk_ble_active_profile_is_open()) {
        desired_adv = ZMK_ADV_CONN;
    } else if (!zmk_ble_active_profile_is_connected()) {
        desired_adv = ZMK_ADV_CONN;
        // Need to fix directed advertising for privacy centrals. See
        // https://github.com/zephyrproject-rtos/zephyr/pull/14984 char
        // addr_str[BT_ADDR_LE_STR_LEN]; bt_addr_le_to_str(zmk_ble_active_profile_addr(), addr_str,
        // sizeof(addr_str));

        // LOG_DBG("Directed advertising to %s", log_strdup(addr_str));
        // desired_adv = ZMK_ADV_DIR;
    }
    LOG_DBG("advertising from %d to %d", advertising_status, desired_adv);

    switch (desired_adv + CURR_ADV(advertising_status)) {
    case ZMK_ADV_NONE + CURR_ADV(ZMK_ADV_DIR):
    case ZMK_ADV_NONE + CURR_ADV(ZMK_ADV_CONN):
        CHECKED_ADV_STOP();
        break;
    case ZMK_ADV_DIR + CURR_ADV(ZMK_ADV_DIR):
    case ZMK_ADV_DIR + CURR_ADV(ZMK_ADV_CONN):
        CHECKED_ADV_STOP();
        CHECKED_DIR_ADV();
        break;
    case ZMK_ADV_DIR + CURR_ADV(ZMK_ADV_NONE):
        CHECKED_DIR_ADV();
        break;
    case ZMK_ADV_CONN + CURR_ADV(ZMK_ADV_DIR):
        CHECKED_ADV_STOP();
        CHECKED_OPEN_ADV();
        break;
    case ZMK_ADV_CONN + CURR_ADV(ZMK_ADV_NONE):
        CHECKED_OPEN_ADV();
        break;
    }

    return 0;
};

static void update_advertising_callback(struct k_work *work) { update_advertising(); }

K_WORK_DEFINE(update_advertising_work, update_advertising_callback);

int zmk_ble_clear_bonds() {
    LOG_DBG("");

    if (bt_addr_le_cmp(&profiles[active_profile].peer, BT_ADDR_LE_ANY)) {
        LOG_DBG("Unpairing!");
        bt_unpair(BT_ID_DEFAULT, &profiles[active_profile].peer);
        set_profile_address(active_profile, BT_ADDR_LE_ANY);
    }

    update_advertising();

    return 0;
};

int zmk_ble_active_profile_index() { return active_profile; }

#if IS_ENABLED(CONFIG_SETTINGS)
static void ble_save_profile_work(struct k_work *work) {
    settings_save_one("ble/active_profile", &active_profile, sizeof(active_profile));
}

static struct k_delayed_work ble_save_work;
#endif

static int ble_save_profile() {
#if IS_ENABLED(CONFIG_SETTINGS)
    k_delayed_work_cancel(&ble_save_work);
    return k_delayed_work_submit(&ble_save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
#else
    return 0;
#endif
}

int zmk_ble_prof_select(uint8_t index) {
    if (index >= PROFILE_COUNT) {
        return -ERANGE;
    }

    LOG_DBG("profile %d", index);
    if (active_profile == index) {
        return 0;
    }

    active_profile = index;
    ble_save_profile();

    update_advertising();

    raise_profile_changed_event();

    return 0;
};

int zmk_ble_prof_next() {
    LOG_DBG("");
    return zmk_ble_prof_select((active_profile + 1) % PROFILE_COUNT);
};

int zmk_ble_prof_prev() {
    LOG_DBG("");
    return zmk_ble_prof_select((active_profile + PROFILE_COUNT - 1) % PROFILE_COUNT);
};

bt_addr_le_t *zmk_ble_active_profile_addr() { return &profiles[active_profile].peer; }

char *zmk_ble_active_profile_name() { return profiles[active_profile].name; }

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL)

int zmk_ble_put_peripheral_addr(const bt_addr_le_t *addr) {
    for (int i = 0; i < CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS; i++) {
        // If the address is recognized and already stored in settings, return
        // index and no additional action is necessary.
        if (!bt_addr_le_cmp(&peripheral_addrs[i], addr)) {
            return i;
        }

        // If the peripheral address slot is open, store new peripheral in the
        // slot and return index. This compares against BT_ADDR_LE_ANY as that
        // is the zero value.
        if (!bt_addr_le_cmp(&peripheral_addrs[i], BT_ADDR_LE_ANY)) {
            char addr_str[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
            LOG_DBG("Storing peripheral %s in slot %d", log_strdup(addr_str), i);

            bt_addr_le_copy(&peripheral_addrs[i], addr);

            char setting_name[32];
            sprintf(setting_name, "ble/peripheral_addresses/%d", i);
            settings_save_one(setting_name, addr, sizeof(bt_addr_le_t));

            return i;
        }
    }

    // The peripheral in question does not match any of the known peripherals.
    return -1;
}

#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL) */

#if IS_ENABLED(CONFIG_SETTINGS)

static int ble_profiles_handle_set(const char *name, size_t len, settings_read_cb read_cb,
                                   void *cb_arg) {
    const char *next;

    LOG_DBG("Setting BLE value %s", log_strdup(name));

    if (settings_name_steq(name, "profiles", &next) && next) {
        char *endptr;
        uint8_t idx = strtoul(next, &endptr, 10);
        if (*endptr != '\0') {
            LOG_WRN("Invalid profile index: %s", log_strdup(next));
            return -EINVAL;
        }

        if (len != sizeof(struct zmk_ble_profile)) {
            LOG_ERR("Invalid profile size (got %d expected %d)", len,
                    sizeof(struct zmk_ble_profile));
            return -EINVAL;
        }

        if (idx >= PROFILE_COUNT) {
            LOG_WRN("Profile address for index %d is larger than max of %d", idx, PROFILE_COUNT);
            return -EINVAL;
        }

        int err = read_cb(cb_arg, &profiles[idx], sizeof(struct zmk_ble_profile));
        if (err <= 0) {
            LOG_ERR("Failed to handle profile address from settings (err %d)", err);
            return err;
        }

        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&profiles[idx].peer, addr_str, sizeof(addr_str));

        LOG_DBG("Loaded %s address for profile %d", log_strdup(addr_str), idx);
    } else if (settings_name_steq(name, "active_profile", &next) && !next) {
        if (len != sizeof(active_profile)) {
            return -EINVAL;
        }

        int err = read_cb(cb_arg, &active_profile, sizeof(active_profile));
        if (err <= 0) {
            LOG_ERR("Failed to handle active profile from settings (err %d)", err);
            return err;
        }
    }
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL)
    else if (settings_name_steq(name, "peripheral_addresses", &next) && next) {
        if (len != sizeof(bt_addr_le_t)) {
            return -EINVAL;
        }

        int i = atoi(next);
        if (i >= CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS) {
            LOG_ERR("Failed to store peripheral address");
        } else {
            int err = read_cb(cb_arg, &peripheral_addrs[i], sizeof(bt_addr_le_t));
            if (err <= 0) {
                LOG_ERR("Failed to handle peripheral address from settings (err %d)", err);
                return err;
            }
        }
    }
#endif

    return 0;
};

struct settings_handler profiles_handler = {.name = "ble", .h_set = ble_profiles_handle_set};
#endif /* IS_ENABLED(CONFIG_SETTINGS) */

static bool is_conn_active_profile(const struct bt_conn *conn) {
    return bt_addr_le_cmp(bt_conn_get_dst(conn), &profiles[active_profile].peer) == 0;
}

static void connected(struct bt_conn *conn, uint8_t err) {
    // Skip if MAC address matches a peripheral.
    const bt_addr_le_t *addr = bt_conn_get_dst(conn);
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL)
    for (int i = 0; i < CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS; i++) {
        if (!bt_addr_le_cmp(&peripheral_addrs[i], addr)) {
            return;
        }
    }
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL) */

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    LOG_DBG("Connected thread: %p", k_current_get());

    advertising_status = ZMK_ADV_NONE;

    if (err) {
        LOG_WRN("Failed to connect to %s (%u)", log_strdup(addr_str), err);
        update_advertising();
        return;
    }

    LOG_DBG("Connected %s", log_strdup(addr_str));

    err = bt_conn_le_param_update(conn, BT_LE_CONN_PARAM(0x0006, 0x000c, 30, 400));
    if (err) {
        LOG_WRN("Failed to update LE parameters (err %d)", err);
    }

#if IS_SPLIT_PERIPHERAL
    bt_conn_le_phy_update(conn, BT_CONN_LE_PHY_PARAM_2M);
#endif

    if (bt_conn_set_security(conn, BT_SECURITY_L2)) {
        LOG_ERR("Failed to set security");
    }

    update_advertising();

    if (is_conn_active_profile(conn)) {
        LOG_DBG("Active profile connected");
        k_work_submit(&raise_profile_changed_event_work);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    // Skip if MAC address if it matches a peripheral.
    char addr_str[BT_ADDR_LE_STR_LEN];
    const bt_addr_le_t *addr = bt_conn_get_dst(conn);
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL)
    for (int i = 0; i < CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS; i++) {
        if (!bt_addr_le_cmp(&peripheral_addrs[i], addr)) {
            return;
        }
    }
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_ROLE_CENTRAL) */
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    LOG_DBG("Disconnected from %s (reason 0x%02x)", log_strdup(addr_str), reason);

    // We need to do this in a work callback, otherwise the advertising update will still see the
    // connection for a profile as active, and not start advertising yet.
    k_work_submit(&update_advertising_work);

    if (is_conn_active_profile(conn)) {
        LOG_DBG("Active profile disconnected");
        k_work_submit(&raise_profile_changed_event_work);
    }
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        LOG_DBG("Security changed: %s level %u", log_strdup(addr), level);
    } else {
        LOG_ERR("Security failed: %s level %u err %d", log_strdup(addr), level, err);
    }
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                             uint16_t timeout) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_DBG("%s: interval %d latency %d timeout %d", log_strdup(addr), interval, latency, timeout);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .security_changed = security_changed,
    .le_param_updated = le_param_updated,
};

/*
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_DBG("Passkey for %s: %06u", log_strdup(addr), passkey);
}
*/

#ifdef CONFIG_ZMK_BLE_PASSKEY_ENTRY

static void auth_passkey_entry(struct bt_conn *conn) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_DBG("Passkey entry requested for %s", log_strdup(addr));
    auth_passkey_entry_conn = bt_conn_ref(conn);
}

#endif

static void auth_cancel(struct bt_conn *conn) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (auth_passkey_entry_conn) {
        bt_conn_unref(auth_passkey_entry_conn);
        auth_passkey_entry_conn = NULL;
    }

    passkey_digit = 0;

    LOG_DBG("Pairing cancelled: %s", log_strdup(addr));
}

#if IS_HOST_PERIPHERAL
static enum bt_security_err auth_pairing_accept(struct bt_conn *conn,
                                                const struct bt_conn_pairing_feat *const feat) {
    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);

    LOG_DBG("role %d, open? %s", info.role, zmk_ble_active_profile_is_open() ? "yes" : "no");
    if (info.role == BT_CONN_ROLE_SLAVE && !zmk_ble_active_profile_is_open()) {
        LOG_WRN("Rejecting pairing request to taken profile %d", active_profile);
        return BT_SECURITY_ERR_PAIR_NOT_ALLOWED;
    }

    return BT_SECURITY_ERR_SUCCESS;
};
#endif /* IS_HOST_PERIPHERAL */

static void auth_pairing_complete(struct bt_conn *conn, bool bonded) {
    struct bt_conn_info info;
    char addr[BT_ADDR_LE_STR_LEN];
    const bt_addr_le_t *dst = bt_conn_get_dst(conn);

    bt_addr_le_to_str(dst, addr, sizeof(addr));
    bt_conn_get_info(conn, &info);

    if (info.role != BT_CONN_ROLE_SLAVE) {
        LOG_DBG("SKIPPING FOR ROLE %d", info.role);
        return;
    }

#if IS_HOST_PERIPHERAL
    if (!zmk_ble_active_profile_is_open()) {
        LOG_ERR("Pairing completed but current profile is not open: %s", log_strdup(addr));
        bt_unpair(BT_ID_DEFAULT, dst);
        return;
    }
#endif /* IS_HOST_PERIPHERAL */

    set_profile_address(active_profile, dst);
    update_advertising();
};

static struct bt_conn_auth_cb zmk_ble_auth_cb_display = {
#if IS_HOST_PERIPHERAL
    .pairing_accept = auth_pairing_accept,
#endif /* IS_HOST_PERIPHERAL */
    .pairing_complete = auth_pairing_complete,
// .passkey_display = auth_passkey_display,

#ifdef CONFIG_ZMK_BLE_PASSKEY_ENTRY
    .passkey_entry = auth_passkey_entry,
#endif
    .cancel = auth_cancel,
};

static void zmk_ble_ready(int err) {
    LOG_DBG("ready? %d", err);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    update_advertising();
}

static int zmk_ble_init(const struct device *_arg) {
    int err = bt_enable(NULL);

    if (err) {
        LOG_ERR("BLUETOOTH FAILED (%d)", err);
        return err;
    }

#if IS_ENABLED(CONFIG_SETTINGS)
    settings_subsys_init();

    err = settings_register(&profiles_handler);
    if (err) {
        LOG_ERR("Failed to setup the profile settings handler (err %d)", err);
        return err;
    }

    k_delayed_work_init(&ble_save_work, ble_save_profile_work);

    settings_load_subtree("ble");
    settings_load_subtree("bt");

#endif

#if IS_ENABLED(CONFIG_ZMK_BLE_CLEAR_BONDS_ON_START)
    LOG_WRN("Clearing all existing BLE bond information from the keyboard");

    for (int i = 0; i < 10; i++) {
        bt_unpair(i, NULL);
    }

    for (int i = 0; i < PROFILE_COUNT; i++) {
        char setting_name[15];
        sprintf(setting_name, "ble/profiles/%d", i);

        err = settings_delete(setting_name);
        if (err) {
            LOG_ERR("Failed to delete setting: %d", err);
        }
    }
#endif

    bt_conn_cb_register(&conn_callbacks);
    bt_conn_auth_cb_register(&zmk_ble_auth_cb_display);

    zmk_ble_ready(0);

    return 0;
}

int zmk_ble_unpair_all() {
    int resp = 0;
    for (int i = BT_ID_DEFAULT; i < CONFIG_BT_ID_MAX; i++) {

        int err = bt_unpair(BT_ID_DEFAULT, NULL);
        if (err) {
            resp = err;
            LOG_ERR("Failed to unpair devices (err %d)", err);
        }
    }

    return resp;
};

bool zmk_ble_handle_key_user(struct zmk_key_event *key_event) {
    zmk_key_t key = key_event->key;

    if (!auth_passkey_entry_conn) {
        return true;
    }

    if (key < NUMBER_1 || key > NUMBER_0) {
        return true;
    }

    uint32_t val = (key == NUMBER_0) ? 0 : (key - NUMBER_1 + 1);

    passkey_entries[passkey_digit++] = val;

    if (passkey_digit == 6) {
        uint32_t passkey = 0;
        for (int i = 5; i >= 0; i--) {
            passkey = (passkey * 10) + val;
        }
        bt_conn_auth_passkey_entry(auth_passkey_entry_conn, passkey);
        bt_conn_unref(auth_passkey_entry_conn);
        auth_passkey_entry_conn = NULL;
    }

    return false;
}

SYS_INIT(zmk_ble_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);
