#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOSConfig.h"
/* BLE */
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "modlog/modlog.h"
#include "nimble/ble.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gatt/ble_svc_gatt.h"

#include "freertos/queue.h"
#include "driver/gpio.h"

//#include "esp_sleep.h"
#include "esp_wifi.h"

#define LED_PIN 16

static int svc_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
uint16_t handler_f002;
uint16_t led_settings=0;
int16_t led_prescaler=0;
uint16_t clicks=0x5;

static void set_led(uint16_t ls){
  if(ls&1)
    gpio_set_level(LED_PIN, 0);
  else
    gpio_set_level(LED_PIN, 1);
  led_settings=ls;
  led_prescaler=0;
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
  { //main service

    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = BLE_UUID16_DECLARE(0xf001),
    .characteristics = (struct ble_gatt_chr_def[])
    { {
        .uuid = BLE_UUID16_DECLARE(0xf002),
        .access_cb = svc_cb,
        .val_handle = &handler_f002,
        .flags = BLE_GATT_CHR_F_NOTIFY,
      }, {
        .uuid = BLE_UUID16_DECLARE(0xf003),
        .access_cb = svc_cb,
        .flags = BLE_GATT_CHR_F_READ,
      }, { 0 }
    }
  },
  { //button press times to alarm
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = BLE_UUID16_DECLARE(0xf011),
    .characteristics = (struct ble_gatt_chr_def[])
    { {
        .uuid = BLE_UUID16_DECLARE(0xf012),
        .access_cb = svc_cb,
        .flags = BLE_GATT_CHR_F_WRITE|BLE_GATT_CHR_F_READ
      }, { 0 }
    }
  },
  { //led status
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = BLE_UUID16_DECLARE(0xf021),
    .characteristics = (struct ble_gatt_chr_def[])
    { {
        .uuid = BLE_UUID16_DECLARE(0xf022),
        .access_cb = svc_cb,
        .flags = BLE_GATT_CHR_F_WRITE|BLE_GATT_CHR_F_READ
      }, { 0 }
    }
  },
  { 0 }
};

static xQueueHandle gpio_evt_queue = NULL;
volatile uint32_t cnt=0;
volatile uint32_t acnt=0xffffffff;

static xTimerHandle sec_tmr;
static xTimerHandle led_tmr;


static bool notify_state;

static uint16_t conn_handle;

static const char *devname = "BRB1";

static int gap_event(struct ble_gap_event *event, void *arg);

static uint8_t bbb_addr_type;

static void next_cnt(void){
  cnt++;
  //if(cnt==0x80000000) cnt=0;
  if(acnt<0xffffffff) acnt++;
}

void cnt_as_bin(void *ptr);
void cnt_as_bin(void *ptr) {
  if(!notify_state)
    next_cnt();
  uint8_t *resp=(uint8_t*)ptr;
  resp[0] = (acnt>>24)&0xff;
  resp[1] = (acnt>>16)&0xff;
  resp[2] = (acnt>>8)&0xff;
  resp[3] = (acnt)&0xff;
  resp[4] = (cnt>>24)&0xff;
  resp[5] = (cnt>>16)&0xff;
  resp[6] = (cnt>>8)&0xff;
  resp[7] = (cnt)&0xff;
};

/**
 * Utility function to log an array of bytes.
 */
void print_bytes(const uint8_t *bytes, int len) {
  int i;
  for (i = 0; i < len; i++) {
    MODLOG_DFLT(INFO, "%s0x%02x", i != 0 ? ":" : "", bytes[i]);
  }
}

void print_addr(const void *addr) {
  const uint8_t *u8p;

  u8p = addr;
  MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
      u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}


/*
 * Enables advertising with parameters:
 *     o General discoverable mode
 *     o Undirected connectable mode
 */
static void advertise(void) {
  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields fields;
  int rc;

  /*
   *  Set the advertisement data included in our advertisements:
   *     o Flags (indicates advertisement type and other general info)
   *     o Advertising tx power
   *     o Device name
   */
  memset(&fields, 0, sizeof(fields));

  /*
   * Advertise two flags:
   *      o Discoverability in forthcoming advertisement (general)
   *      o BLE-only (BR/EDR unsupported)
   */
  fields.flags = BLE_HS_ADV_F_DISC_GEN |
    BLE_HS_ADV_F_BREDR_UNSUP;

  /*
   * Indicate that the TX power level field should be included; have the
   * stack fill this value automatically.  This is done by assigning the
   * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
   */
  fields.tx_pwr_lvl_is_present = 1;
  fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

  fields.name = (uint8_t *)devname;
  fields.name_len = strlen(devname);
  fields.name_is_complete = 1;

  rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
    return;
  }

  /* Begin advertising */
  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  rc = ble_gap_adv_start(bbb_addr_type, NULL, BLE_HS_FOREVER,
      &adv_params, gap_event, NULL);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
    return;
  }
}

static void led_tmr_init(void) {
  int rc;

  if (xTimerReset(led_tmr, 250 / portTICK_PERIOD_MS ) == pdPASS) {
    rc = 0;
  } else {
    rc = 1;
  }
  assert(rc == 0);
}

static void sec_tmr_init(void) {
  int rc;

  if (xTimerReset(sec_tmr, 1000 / portTICK_PERIOD_MS ) == pdPASS) {
    rc = 0;
  } else {
    rc = 1;
  }

  assert(rc == 0);

}

static void sec_timer(xTimerHandle ev) {
  static uint8_t resp[8];
  int rc;

  if (notify_state) {
    printf("Timer %d %d\n",cnt,acnt);
    cnt_as_bin(&resp);

    next_cnt();
    struct os_mbuf *om = ble_hs_mbuf_from_flat(resp, sizeof(resp));
    rc = ble_gattc_notify_custom(conn_handle, handler_f002, om);

    assert(rc == 0);
  }
  sec_tmr_init();
  /*
  esp_sleep_enable_timer_wakeup(1000000L);
  esp_light_sleep_start();
  */
}


static void led_timer(xTimerHandle ev) {
  if(led_settings>1){
    uint8_t secs=(led_settings&0xf00)>>8;
    uint8_t active=(led_settings&0xf000)>>12;
    led_prescaler--;
    if(led_prescaler<1){
      led_prescaler=secs;
      gpio_set_level(LED_PIN, 1);
    }
    if(led_prescaler==active){
      gpio_set_level(LED_PIN, 0);
    }

    printf("%d %d %d\n",secs,active,led_prescaler);
  }
  led_tmr_init();

}

static int gap_event(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      /* A new connection was established or a connection attempt failed */
      set_led(0);
      MODLOG_DFLT(INFO, "connection %s; status=%d\n",
          event->connect.status == 0 ? "established" : "failed",
          event->connect.status);

      if (event->connect.status != 0) {
        /* Connection failed; resume advertising */
        advertise();
      }
      conn_handle = event->connect.conn_handle;
      break;

    case BLE_GAP_EVENT_DISCONNECT:
      MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

      /* Connection terminated; resume advertising */
      advertise();
      break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
      MODLOG_DFLT(INFO, "adv complete\n");
      advertise();
      break;

    case BLE_GAP_EVENT_SUBSCRIBE:
      MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
          "val_handle=%d\n",
          event->subscribe.cur_notify, handler_f002);
      if (event->subscribe.attr_handle == handler_f002) {
        notify_state = event->subscribe.cur_notify;
      } else if (event->subscribe.attr_handle != handler_f002) {
        notify_state = event->subscribe.cur_notify;
      }
      ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", conn_handle);
      break;

    case BLE_GAP_EVENT_MTU:
      MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d mtu=%d\n",
          event->mtu.conn_handle,
          event->mtu.value);
      break;

  }

  return 0;
}

static void on_sync(void) {
  int rc;

  rc = ble_hs_id_infer_auto(0, &bbb_addr_type);
  assert(rc == 0);

  uint8_t addr_val[6] = {0};
  rc = ble_hs_id_copy_addr(bbb_addr_type, addr_val, NULL);

  MODLOG_DFLT(INFO, "Device Address: ");
  print_addr(addr_val);
  MODLOG_DFLT(INFO, "\n");

  advertise();
}

static void on_rst(int reason) {
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

void bluetooth_task(void *param) {
    printf("BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
  uint32_t gpio_num = (uint32_t) arg;
  gpio_num=0;
  uint32_t msg=(gpio_get_level(gpio_num)?1:0)|(gpio_num<<1);
  xQueueSendFromISR(gpio_evt_queue, &msg, NULL);
  //ESP_LOGE(LOG_TAG, "%s GPIO INT\n", __func__);
}

static void gpio_task_example(void* arg) {
  uint32_t active=0;
  uint32_t io_num;
  for(;;) {
    if(xQueueReceive(gpio_evt_queue, &io_num, 100)) {
      if(io_num & 1 && active<clicks)
        active++;
      if(active==clicks){
        printf("Triggered\n");
        acnt=0;
      }
      printf("GPIO intr, val: %d %d\n", io_num,active);
    }else{
      if(active){
        printf("GPIO inactive %d\n",active);
        active--;
      }
    }
  }
}

static int svc_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {

  uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);

  /*
     *     o  BLE_GATT_ACCESS_OP_READ_CHR
     *     o  BLE_GATT_ACCESS_OP_WRITE_CHR
     *     o  BLE_GATT_ACCESS_OP_READ_DSC
     *     o  BLE_GATT_ACCESS_OP_WRITE_DSC
     *     */
 
  printf("op %04x ch %04x ah %04x uuid %04x\n",ctxt->op,conn_handle,attr_handle,uuid);

  if (ctxt->op==BLE_GATT_ACCESS_OP_WRITE_CHR) {
    printf("Write len %d\n",ctxt->om->om_len);
    int i;
    for(i=0;i<ctxt->om->om_len;i++){
      printf("%02x ",ctxt->om->om_data[i]);
    }
    printf("\n");
  }

  if (uuid == 0xf002 && ctxt->op==BLE_GATT_ACCESS_OP_READ_CHR) {
    char buf[8];
    cnt_as_bin(buf);
    int rc = os_mbuf_append(ctxt->om, buf, sizeof(buf));

    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  if (uuid == 0xf012 && ctxt->op==BLE_GATT_ACCESS_OP_WRITE_CHR && ctxt->om->om_len==2) {
    uint16_t tclicks=htons(*(uint16_t*)ctxt->om->om_data);
    printf("Tmp clicks %d\n",tclicks);
    if(tclicks<15 && tclicks>0){
      printf("set clicks %d\n",tclicks);
      clicks=tclicks;
      return 0;
    }else
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }

  if (uuid == 0xf012 && ctxt->op==BLE_GATT_ACCESS_OP_READ_CHR) {
    uint16_t beclicks=htons(clicks);
    int rc = os_mbuf_append(ctxt->om, &beclicks, sizeof(beclicks));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  if (uuid == 0xf022 && ctxt->op==BLE_GATT_ACCESS_OP_WRITE_CHR && ctxt->om->om_len==2) {
    uint16_t tledset=htons(*(uint16_t*)ctxt->om->om_data);
    printf("Tmo led set %04x\n",tledset);
    set_led(tledset);
    return 0;
  }


  if (uuid == 0xf022 && ctxt->op==BLE_GATT_ACCESS_OP_READ_CHR) {
    uint16_t bels=htons(led_settings);
    int rc = os_mbuf_append(ctxt->om, &bels, sizeof(bels));

    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }


  return BLE_ATT_ERR_UNLIKELY;

}

int gatt_svr_init(void) {
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}


void app_main(void) {
  int rc;


  esp_wifi_stop();
  /* Initialize NVS — it is used to store PHY calibration data */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

  nimble_port_init();
  /* Initialize the NimBLE host configuration */
  ble_hs_cfg.sync_cb = on_sync;
  ble_hs_cfg.reset_cb = on_rst;

  /* name, period/time,  auto reload, timer ID, callback */
  sec_tmr = xTimerCreate("second_timer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, sec_timer);
  sec_tmr_init();
  led_tmr = xTimerCreate("led_timer", pdMS_TO_TICKS(250), pdTRUE, (void *)0, led_timer);
  led_tmr_init();

  rc = gatt_svr_init();
  assert(rc == 0);

  /* Set the default device name */
  rc = ble_svc_gap_device_name_set(devname);
  assert(rc == 0);

  /* Start the task */
  nimble_port_freertos_init(bluetooth_task);

  set_led(1);
  {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL<<LED_PIN));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
  }

  {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = 1ULL; //GPIO0
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
  }


  //change gpio intrrupt type for one pin
  gpio_set_intr_type(0, GPIO_INTR_ANYEDGE);

  //create a queue to handle gpio event from isr
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  //start gpio task
  xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

  //install gpio isr service

#define ESP_INTR_FLAG_DEFAULT 0
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(0, gpio_isr_handler, (void*) 0);

}
