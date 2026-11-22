// BLE トランスポート実装（UWC_TRANSPORT_BLE）。
// NUS 相当 GATT サービスで UART バイトストリームを透過する。

#include <stdio.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/printk.h>

#include "device/link/link.hpp"

namespace uwc {

namespace {

// Nordic UART Service 互換 UUID
// NOLINTBEGIN: BT_UUID_* マクロ展開に伴う警告
#define UWC_NUS_SVC_UUID \
  BT_UUID_128_ENCODE(0x6E400001, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)
#define UWC_NUS_RX_UUID \
  BT_UUID_128_ENCODE(0x6E400002, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)
#define UWC_NUS_TX_UUID \
  BT_UUID_128_ENCODE(0x6E400003, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)

struct bt_uuid_128 nus_svc_uuid = BT_UUID_INIT_128(UWC_NUS_SVC_UUID);
struct bt_uuid_128 nus_rx_uuid = BT_UUID_INIT_128(UWC_NUS_RX_UUID);
struct bt_uuid_128 nus_tx_uuid = BT_UUID_INIT_128(UWC_NUS_TX_UUID);
// NOLINTEND

// link 実装の内部状態（無名名前空間で内部結合）。
// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
struct {
  struct ring_buf* rx_ring;
  struct k_sem* rx_sem;
  void (*event_cb)(link::event);
  struct bt_conn* conn;
  link::state st;
  bool subscribed;
  uint32_t rx_drop_count;
} impl = {nullptr, nullptr, nullptr, nullptr, link::state::IDLE, false, 0};
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

// RX 特性（アプリ→モジュール、Write）: 受信バイトをリングへ積む
ssize_t nus_rx_write_cb(struct bt_conn* conn, const struct bt_gatt_attr* attr,
                        const void* buf, uint16_t len, uint16_t offset,
                        uint8_t flags) {
  ARG_UNUSED(conn);
  ARG_UNUSED(attr);
  ARG_UNUSED(offset);
  ARG_UNUSED(flags);
  const uint32_t n =
      ring_buf_put(impl.rx_ring, static_cast<const uint8_t*>(buf), len);
  impl.rx_drop_count += static_cast<uint32_t>(len) - n;
  if (n > 0) {
    k_sem_give(impl.rx_sem);
  }
  return len;
}

// TX 特性 CCC 変化: 購読状態を追跡する
void nus_tx_ccc_changed_cb(const struct bt_gatt_attr* attr, uint16_t value) {
  ARG_UNUSED(attr);
  impl.subscribed = (value == BT_GATT_CCC_NOTIFY);
}

// NOLINTBEGIN: Zephyr GATT 定義マクロ由来の警告
// Zephyr の GATT/conn 定義マクロは -Wmissing-field-initializers を出すため
// この区間のみ抑制する。
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
BT_GATT_SERVICE_DEFINE(
    nus_svc, BT_GATT_PRIMARY_SERVICE(&nus_svc_uuid),
    BT_GATT_CHARACTERISTIC(&nus_rx_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, nullptr, &nus_rx_write_cb,
                           nullptr),
    BT_GATT_CHARACTERISTIC(&nus_tx_uuid.uuid, BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE, nullptr, nullptr, nullptr),
    BT_GATT_CCC(&nus_tx_ccc_changed_cb,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));
#pragma GCC diagnostic pop
// NOLINTEND
// attrs: 0=primary, 1=RX decl, 2=RX value, 3=TX decl, 4=TX value, 5=TX CCC

// 切断後の再 advertising までの待ち時間と、adv 開始失敗時のリトライ間隔。
// 切断直後はコントローラ側の後処理中で adv 再開が失敗し得るため待つ。
constexpr uint32_t ADV_RESTART_DELAY_MS = 100;
constexpr uint32_t ADV_RETRY_DELAY_MS = 1000;

const struct bt_data AD[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

const struct bt_data SD[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, UWC_NUS_SVC_UUID),
};

// 再 advertising 開始を system workqueue で実行する遅延ワーク。
// BT conn コールバックの実行コンテキストから HCI コマンドを直に発行せず、
// 失敗時は一定間隔でリトライする（FR-02）。
void adv_work_cb(struct k_work* work);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables,clang-diagnostic-missing-designated-field-initializers)
K_WORK_DELAYABLE_DEFINE(adv_work, adv_work_cb);

// connectable advertising を開始する。開始済みを含め成功時 0、失敗時は
// bt_le_adv_start のエラーを返す。成功時は WAITING 遷移イベントを通知する。
int start_advertising() {
  const int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, AD, ARRAY_SIZE(AD), SD,
                                  ARRAY_SIZE(SD));
  if (err != 0 && err != -EALREADY) {
    printk("uwc: adv start failed (%d)\n", err);
    return err;
  }
  impl.st = link::state::WAITING;
  if (impl.event_cb != nullptr) impl.event_cb(link::event::WAITING_STARTED);
  return 0;
}

void adv_work_cb(struct k_work* work) {
  ARG_UNUSED(work);
  // advertising 中・接続中（IDLE 以外）では再開は不要。
  if (impl.st != link::state::IDLE) {
    return;
  }
  if (start_advertising() != 0) {
    (void)k_work_reschedule(&adv_work, K_MSEC(ADV_RETRY_DELAY_MS));
  }
}

void mtu_exchange_cb(struct bt_conn* conn, uint8_t err,
                     struct bt_gatt_exchange_params* params) {
  ARG_UNUSED(conn);
  ARG_UNUSED(params);
  if (err == 0 && impl.event_cb != nullptr) {
    impl.event_cb(link::event::MTU_UPDATED);
  }
}

void request_link_params(struct bt_conn* conn) {
  // 接続間隔 7.5-30 ms 要求（NFR-01）、DLE・2M PHY・MTU 拡大を要求する。
  // 失敗しても既定値で継続する。
  const struct bt_le_conn_param param = {
      .interval_min = 6,
      .interval_max = 24,
      .latency = 0,
      .timeout = 400,
  };
  (void)bt_conn_le_param_update(conn, &param);
  static const struct bt_conn_le_phy_param PHY_PARAM = {
      .options = BT_CONN_LE_PHY_OPT_NONE,
      .pref_tx_phy = BT_GAP_LE_PHY_2M,
      .pref_rx_phy = BT_GAP_LE_PHY_2M,
  };
  (void)bt_conn_le_phy_update(conn, &PHY_PARAM);
  static const struct bt_conn_le_data_len_param DL_PARAM = {
      .tx_max_len = 251,
      .tx_max_time = 2120,
  };
  (void)bt_conn_le_data_len_update(conn, &DL_PARAM);
  static struct bt_gatt_exchange_params mtu_params = {};
  mtu_params.func = mtu_exchange_cb;
  (void)bt_gatt_exchange_mtu(conn, &mtu_params);
}

void connected_cb(struct bt_conn* conn, uint8_t err) {
  if (err != 0) {
    printk("uwc: connect failed (0x%02x)\n", err);
    // 接続確立に失敗した場合も advertising を再開して待ちに戻る（FR-02）
    impl.st = link::state::IDLE;
    (void)k_work_reschedule(&adv_work, K_MSEC(ADV_RESTART_DELAY_MS));
    return;
  }
  impl.conn = bt_conn_ref(conn);
  impl.st = link::state::CONNECTED;
  impl.subscribed = false;
  request_link_params(conn);
  if (impl.event_cb != nullptr) impl.event_cb(link::event::CONNECTED);
}

void disconnected_cb(struct bt_conn* conn, uint8_t reason) {
  ARG_UNUSED(conn);
  ARG_UNUSED(reason);
  if (impl.conn != nullptr) {
    bt_conn_unref(impl.conn);
    impl.conn = nullptr;
  }
  impl.st = link::state::IDLE;
  impl.subscribed = false;
  if (impl.event_cb != nullptr) impl.event_cb(link::event::DISCONNECTED);
  // 自動的に再 advertising する（FR-02）。コールバックコンテキストからの
  // 直接実行は避け、system workqueue で失敗時リトライ付きで再開する。
  (void)k_work_reschedule(&adv_work, K_MSEC(ADV_RESTART_DELAY_MS));
}

// NOLINTBEGIN: BT_CONN_CB_DEFINE マクロ展開に伴う警告
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
BT_CONN_CB_DEFINE(conn_cbs) = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
};
#pragma GCC diagnostic pop
// NOLINTEND

void mtu_updated_cb(struct bt_conn* conn, uint16_t tx, uint16_t rx) {
  ARG_UNUSED(conn);
  ARG_UNUSED(tx);
  ARG_UNUSED(rx);
  if (impl.event_cb != nullptr) impl.event_cb(link::event::MTU_UPDATED);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
// bt_gatt_cb_register は非 const 参照を要求するため const 化できない。
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
struct bt_gatt_cb gatt_cbs = {
    .att_mtu_updated = mtu_updated_cb,
};
#pragma GCC diagnostic pop

}  // namespace

bool link::init(const config& cfg) {
  impl.rx_ring = cfg.rx_ring;
  impl.rx_sem = cfg.rx_sem;
  impl.event_cb = cfg.event_cb;
  bt_gatt_cb_register(&gatt_cbs);
  const int err = bt_enable(nullptr);
  if (err != 0 && err != -EALREADY) {
    printk("uwc: bt_enable failed (%d)\n", err);
    return false;
  }
  if (start_advertising() != 0) {
    // 起動直後の一時的失敗も workqueue 側のリトライで回復を図る
    (void)k_work_reschedule(&adv_work, K_MSEC(ADV_RETRY_DELAY_MS));
  }
  return impl.st == link::state::WAITING;
}

bool link::connected() {
  return impl.st == link::state::CONNECTED;
}

link::state link::link_state() {
  return impl.st;
}

uint16_t link::max_payload() {
  if (!connected() || impl.conn == nullptr) {
    return 0;
  }
  const uint16_t mtu = bt_gatt_get_mtu(impl.conn);
  return (mtu > 3) ? static_cast<uint16_t>(mtu - 3) : 0;
}

bool link::send(const uint8_t* data, uint16_t len) {
  if (!connected() || !impl.subscribed || impl.conn == nullptr) {
    return false;
  }
  return bt_gatt_notify(impl.conn, &nus_svc.attrs[4], data, len) == 0;
}

uint32_t link::rx_dropped() {
  return impl.rx_drop_count;
}

const char* link::describe(char* buf, uint32_t len) {
  if (impl.conn != nullptr && connected()) {
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(impl.conn), addr, sizeof(addr));
    (void)snprintf(buf, len, "ble peer=%s mtu=%u notify=%s", addr,
                   static_cast<unsigned>(bt_gatt_get_mtu(impl.conn)),
                   impl.subscribed ? "on" : "off");
    return buf;
  }
  (void)snprintf(buf, len, "ble state=%u", static_cast<unsigned>(impl.st));
  return buf;
}

}  // namespace uwc
