// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
// Additions Copyright (C) Copyright 2016 pcbreflux, Apache 2.0 License.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



/****************************************************************************
*
* This file is for gatt client. It can scan ble device, connect one device,
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "controller.h"

#include "bt.h"
#include "bt_trace.h"
#include "bt_types.h"
#include "btm_api.h"
#include "bta_api.h"
#include "bta_gatt_api.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#define BT_BD_ADDR_STR         "%02x:%02x:%02x:%02x:%02x:%02x"
#define BT_BD_ADDR_HEX(addr)   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID "2805"
#define EXAMPLE_WIFI_PASS "HAH888888"
/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "192.168.1.160"
#define WEB_PORT 80
#define WEB_URL "http://192.168.1.160/esp32.php"

static esp_gatt_if_t client_if;
static uint16_t client_conn = 0;
esp_gatt_status_t status = ESP_GATT_ERROR;
bool connet = false;
bool written = false;
uint16_t simpleClient_id = 0xEE;
char * get_request;
const char device_name[] = "TEMP 12";

static esp_bd_addr_t server_dba;

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = ESP_PUBLIC_ADDR,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};

static esp_gatt_srvc_id_t nrf51_service_id = {
    .id = {
        .uuid = {
            .len = ESP_UUID_LEN_128,
            .uuid = {.uuid128 = {0xa5, 0xa5, 0x00, 0x5b, 0x02, 0x00, 0x23, 0x9b, 0xe1, 0x11, 0x02, 0xd1, 0x00, 0x1c, 0x00, 0x00},},
        },
        .inst_id = 0,
    },
    .is_primary = true,
};

static esp_gatt_id_t nrf51_char_id = {
    .uuid = {
        .len = ESP_UUID_LEN_128,
        .uuid = {.uuid128 = {0xa5, 0xa5, 0x00, 0x5b, 0x02, 0x00, 0x23, 0x9b, 0xe1, 0x11, 0x02, 0xd1, 0x01, 0x1c, 0x00, 0x00},},
    },
    .inst_id = 0,
};



/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;



//static const char *TAG = "example";

static const char *REQUEST = "GET " WEB_URL " HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "User-Agent: esp-idf/1.0 esp32\n"
    "\n";
static const char* get_request_start = "GET /esp32.php?subject=C&web=esp32.com&rawdata=";
static const char* get_request_end =
    " HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "Connection: close\n"
    "User-Agent: esp32 / esp-idf\n"
    "\n";



static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];

    while(1) {
        /* Wait for the callback to set the CONNECTED_BIT in the
           event group.
        */
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                            false, true, portMAX_DELAY);
        ESP_LOGI(TAG, "Connected to AP");

        int err = getaddrinfo(WEB_SERVER, "80", &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket\r\n");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);

        if (write(s, get_request, strlen(get_request)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        /* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d\r\n", r, errno);
        close(s);
        for(int countdown = 100; countdown >= 0; countdown--) {
            ESP_LOGI(TAG, "%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        free(get_request);
        
        ESP_LOGI(TAG, "Starting again!");
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 10;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            LOG_INFO("BDA %x,%x,%x,%x,%x,%x:",scan_result->scan_rst.bda[0],
            		scan_result->scan_rst.bda[1],scan_result->scan_rst.bda[2],
					scan_result->scan_rst.bda[3],scan_result->scan_rst.bda[4],
					scan_result->scan_rst.bda[5]);
            for (int i = 0; i < 6; i++) {
                server_dba[i]=scan_result->scan_rst.bda[i];
            }
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            LOG_INFO("adv_name_len=%x\n", adv_name_len);
            for (int j = 0; j < adv_name_len; j++) {
                LOG_INFO("a%d %x %c = d%d %x %c",j, adv_name[j], adv_name[j],j, device_name[j], device_name[j]);
            }

            if (adv_name != NULL) {
                if (strncmp((char *)adv_name, device_name,adv_name_len) == 0) {
                    LOG_INFO("the name eque to %s.",device_name);
                    if (status ==  ESP_GATT_OK && connet == false) {
                        connet = true;
                        LOG_INFO("Connet to the remote device.");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(client_if, scan_result->scan_rst.bda, true);
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}


static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    uint16_t conn_id = 0;
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    LOG_INFO("esp_gattc_cb, event = %x", event);
    switch (event) {
    case ESP_GATTC_REG_EVT:
        status = p_data->reg.status;
        
        client_if= gattc_if;
        LOG_INFO("ESP_GATTC_REG_EVT status = %x, client_if = %x", status, gattc_if);
        break;
    case ESP_GATTC_OPEN_EVT:
        conn_id = p_data->open.conn_id;
        client_conn = conn_id;
        LOG_INFO("ESP_GATTC_OPEN_EVT conn_id %d, if %d, status %d", conn_id, gattc_if, p_data->open.status);
        esp_ble_gattc_search_service(gattc_if, conn_id, NULL);
        break;
    case ESP_GATTC_READ_CHAR_EVT: {
        esp_gatt_srvc_id_t *srvc_id = &p_data->read.srvc_id;
        esp_gatt_id_t *char_id = &p_data->read.char_id;
        conn_id = p_data->read.conn_id;
        LOG_INFO("READ CHAR: open.conn_id = %x search_res.conn_id = %x  read.conn_id = %x", conn_id,p_data->search_res.conn_id,p_data->read.conn_id);
        LOG_INFO("READ CHAR: read.status = %x inst_id = %x value_len = %x", p_data->read.status, char_id->inst_id, p_data->read.value_len);
        if (p_data->read.status==0) {
			if (char_id->uuid.len == ESP_UUID_LEN_16) {
				LOG_INFO("Char UUID16: %x", char_id->uuid.uuid.uuid16);
			} else if (char_id->uuid.len == ESP_UUID_LEN_32) {
				LOG_INFO("Char UUID32: %x", char_id->uuid.uuid.uuid32);
			} else if (char_id->uuid.len == ESP_UUID_LEN_128) {
				LOG_INFO("Char UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x", char_id->uuid.uuid.uuid128[0],
						 char_id->uuid.uuid.uuid128[1], char_id->uuid.uuid.uuid128[2], char_id->uuid.uuid.uuid128[3],
						 char_id->uuid.uuid.uuid128[4], char_id->uuid.uuid.uuid128[5], char_id->uuid.uuid.uuid128[6],
						 char_id->uuid.uuid.uuid128[7], char_id->uuid.uuid.uuid128[8], char_id->uuid.uuid.uuid128[9],
						 char_id->uuid.uuid.uuid128[10], char_id->uuid.uuid.uuid128[11], char_id->uuid.uuid.uuid128[12],
						 char_id->uuid.uuid.uuid128[13], char_id->uuid.uuid.uuid128[14], char_id->uuid.uuid.uuid128[15]);
			} else {
				LOG_ERROR("Char UNKNOWN LEN %d\n", char_id->uuid.len);
			}
            for (int i = 0; i < p_data->read.value_len; i++) {
                LOG_INFO("%x:", p_data->read.value[i]);
            }
            esp_ble_gattc_register_for_notify(gattc_if,server_dba,srvc_id,char_id);
        }
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT: {
        uint8_t value[]={0x01,0x06};
        
        uint16_t notify_en = 0x0106;
        esp_gatt_srvc_id_t *srvc_id = &p_data->write.srvc_id;
        esp_gatt_id_t *char_id = &p_data->write.char_id;
        esp_gatt_id_t *descr_id = &p_data->write.descr_id;
        conn_id = p_data->open.conn_id;
        LOG_INFO("WRITE CHAR: open.conn_id = %x search_res.conn_id = %x  write.conn_id = %x", conn_id,p_data->search_res.conn_id,p_data->write.conn_id);
        LOG_INFO("WRITE CHAR: write.status = %x inst_id = %x", p_data->write.status, char_id->inst_id);
        if (p_data->write.status==0) {
			if (char_id->uuid.len == ESP_UUID_LEN_16) {
				LOG_INFO("Char UUID16: %x", char_id->uuid.uuid.uuid16);
			} else if (char_id->uuid.len == ESP_UUID_LEN_32) {
				LOG_INFO("Char UUID32: %x", char_id->uuid.uuid.uuid32);
			} else if (char_id->uuid.len == ESP_UUID_LEN_128) {
				LOG_INFO("Char UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x", char_id->uuid.uuid.uuid128[0],
						 char_id->uuid.uuid.uuid128[1], char_id->uuid.uuid.uuid128[2], char_id->uuid.uuid.uuid128[3],
						 char_id->uuid.uuid.uuid128[4], char_id->uuid.uuid.uuid128[5], char_id->uuid.uuid.uuid128[6],
						 char_id->uuid.uuid.uuid128[7], char_id->uuid.uuid.uuid128[8], char_id->uuid.uuid.uuid128[9],
						 char_id->uuid.uuid.uuid128[10], char_id->uuid.uuid.uuid128[11], char_id->uuid.uuid.uuid128[12],
						 char_id->uuid.uuid.uuid128[13], char_id->uuid.uuid.uuid128[14], char_id->uuid.uuid.uuid128[15]);
			} else {
				LOG_ERROR("Char UNKNOWN LEN %d", char_id->uuid.len);
			}
			if (srvc_id->id.uuid.len == ESP_UUID_LEN_16) {
				LOG_INFO("Decr UUID16: %x", srvc_id->id.uuid.uuid.uuid16);
			} else if (srvc_id->id.uuid.len == ESP_UUID_LEN_32) {
				LOG_INFO("Decr UUID32: %x", srvc_id->id.uuid.uuid.uuid32);
			} else if (srvc_id->id.uuid.len == ESP_UUID_LEN_128) {
				LOG_INFO("Decr UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x", srvc_id->id.uuid.uuid.uuid128[0],
						 srvc_id->id.uuid.uuid.uuid128[1], srvc_id->id.uuid.uuid.uuid128[2], srvc_id->id.uuid.uuid.uuid128[3],
						 srvc_id->id.uuid.uuid.uuid128[4], srvc_id->id.uuid.uuid.uuid128[5], srvc_id->id.uuid.uuid.uuid128[6],
						 srvc_id->id.uuid.uuid.uuid128[7], srvc_id->id.uuid.uuid.uuid128[8], srvc_id->id.uuid.uuid.uuid128[9],
						 srvc_id->id.uuid.uuid.uuid128[10], srvc_id->id.uuid.uuid.uuid128[11], srvc_id->id.uuid.uuid.uuid128[12],
						 srvc_id->id.uuid.uuid.uuid128[13], srvc_id->id.uuid.uuid.uuid128[14], srvc_id->id.uuid.uuid.uuid128[15]);
			} else {
				LOG_ERROR("Decr UNKNOWN LEN %d", srvc_id->id.uuid.len);
			}
            //written=true;
            if(written==false){
                esp_ble_gattc_register_for_notify(gattc_if,server_dba,&nrf51_service_id,&nrf51_char_id);
                //esp_ble_gattc_close(gattc_if, client_conn);
            }
            
            
        }
        break;
    }
    case ESP_GATTC_SEARCH_RES_EVT: {
        esp_gatt_srvc_id_t *srvc_id = &p_data->search_res.srvc_id;
        conn_id = p_data->open.conn_id;
        LOG_INFO("SEARCH RES: open.conn_id = %x search_res.conn_id = %x", conn_id,p_data->search_res.conn_id);
        if (srvc_id->id.uuid.len == ESP_UUID_LEN_16) {
            LOG_INFO("UUID16: %x", srvc_id->id.uuid.uuid.uuid16);
        } else if (srvc_id->id.uuid.len == ESP_UUID_LEN_32) {
            LOG_INFO("UUID32: %x", srvc_id->id.uuid.uuid.uuid32);
        } else if (srvc_id->id.uuid.len == ESP_UUID_LEN_128) {
            LOG_INFO("UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x", srvc_id->id.uuid.uuid.uuid128[0],
                     srvc_id->id.uuid.uuid.uuid128[1], srvc_id->id.uuid.uuid.uuid128[2], srvc_id->id.uuid.uuid.uuid128[3],
                     srvc_id->id.uuid.uuid.uuid128[4], srvc_id->id.uuid.uuid.uuid128[5], srvc_id->id.uuid.uuid.uuid128[6],
                     srvc_id->id.uuid.uuid.uuid128[7], srvc_id->id.uuid.uuid.uuid128[8], srvc_id->id.uuid.uuid.uuid128[9],
                     srvc_id->id.uuid.uuid.uuid128[10], srvc_id->id.uuid.uuid.uuid128[11], srvc_id->id.uuid.uuid.uuid128[12],
                     srvc_id->id.uuid.uuid.uuid128[13], srvc_id->id.uuid.uuid.uuid128[14], srvc_id->id.uuid.uuid.uuid128[15]);
            if(srvc_id->id.uuid.uuid.uuid128[12]==0x0)
                esp_ble_gattc_get_characteristic(gattc_if,p_data->search_res.conn_id,srvc_id,NULL);
        } else {
            LOG_ERROR("UNKNOWN LEN %d", srvc_id->id.uuid.len);
        }
        break;
    }
    case ESP_GATTC_WRITE_DESCR_EVT: {
        uint8_t value[]={0x01,0x0a,0xaa,0xbb,0xcc,0xdd};
        esp_gatt_srvc_id_t *srvc_id = &p_data->write.srvc_id;
        esp_gatt_id_t *char_id = &p_data->write.char_id;
        esp_gatt_id_t *descr_id = &p_data->write.descr_id;
        conn_id = p_data->write.conn_id;
        LOG_INFO("WRITE DESCR: open.conn_id = %x search_res.conn_id = %x  write.conn_id = %x", conn_id,p_data->search_res.conn_id,p_data->write.conn_id);
        LOG_INFO("WRITE DESCR: write.status = %x inst_id = %x open.gatt_if = %x", p_data->write.status, char_id->inst_id,gattc_if);
        if (p_data->write.status==0) {
			if (char_id->uuid.len == ESP_UUID_LEN_16) {
				LOG_INFO("Char UUID16: %x", char_id->uuid.uuid.uuid16);
			} else if (char_id->uuid.len == ESP_UUID_LEN_32) {
				LOG_INFO("Char UUID32: %x", char_id->uuid.uuid.uuid32);
			} else if (char_id->uuid.len == ESP_UUID_LEN_128) {
				LOG_INFO("Char UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x", char_id->uuid.uuid.uuid128[0],
						 char_id->uuid.uuid.uuid128[1], char_id->uuid.uuid.uuid128[2], char_id->uuid.uuid.uuid128[3],
						 char_id->uuid.uuid.uuid128[4], char_id->uuid.uuid.uuid128[5], char_id->uuid.uuid.uuid128[6],
						 char_id->uuid.uuid.uuid128[7], char_id->uuid.uuid.uuid128[8], char_id->uuid.uuid.uuid128[9],
						 char_id->uuid.uuid.uuid128[10], char_id->uuid.uuid.uuid128[11], char_id->uuid.uuid.uuid128[12],
						 char_id->uuid.uuid.uuid128[13], char_id->uuid.uuid.uuid128[14], char_id->uuid.uuid.uuid128[15]);
			} else {
				LOG_ERROR("Char UNKNOWN LEN %d", char_id->uuid.len);
			}
			if (descr_id->uuid.len == ESP_UUID_LEN_16) {
				LOG_INFO("Decr UUID16: %x", descr_id->uuid.uuid.uuid16);
			} else if (descr_id->uuid.len == ESP_UUID_LEN_32) {
				LOG_INFO("Decr UUID32: %x", descr_id->uuid.uuid.uuid32);
			} else if (descr_id->uuid.len == ESP_UUID_LEN_128) {
				LOG_INFO("Decr UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x", descr_id->uuid.uuid.uuid128[0],
						 descr_id->uuid.uuid.uuid128[1], descr_id->uuid.uuid.uuid128[2], descr_id->uuid.uuid.uuid128[3],
						 descr_id->uuid.uuid.uuid128[4], descr_id->uuid.uuid.uuid128[5], descr_id->uuid.uuid.uuid128[6],
						 descr_id->uuid.uuid.uuid128[7], descr_id->uuid.uuid.uuid128[8], descr_id->uuid.uuid.uuid128[9],
						 descr_id->uuid.uuid.uuid128[10], descr_id->uuid.uuid.uuid128[11], descr_id->uuid.uuid.uuid128[12],
						 descr_id->uuid.uuid.uuid128[13], descr_id->uuid.uuid.uuid128[14], descr_id->uuid.uuid.uuid128[15]);
			} else {
				LOG_ERROR("Decr UNKNOWN LEN %d", descr_id->uuid.len);
			}
			if (srvc_id->id.uuid.len == ESP_UUID_LEN_16) {
				LOG_INFO("SRVC UUID16: %x", srvc_id->id.uuid.uuid.uuid16);
			} else if (srvc_id->id.uuid.len == ESP_UUID_LEN_32) {
				LOG_INFO("SRVC UUID32: %x", srvc_id->id.uuid.uuid.uuid32);
			} else if (srvc_id->id.uuid.len == ESP_UUID_LEN_128) {
				LOG_INFO("SRVC UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x", srvc_id->id.uuid.uuid.uuid128[0],
						 srvc_id->id.uuid.uuid.uuid128[1], srvc_id->id.uuid.uuid.uuid128[2], srvc_id->id.uuid.uuid.uuid128[3],
						 srvc_id->id.uuid.uuid.uuid128[4], srvc_id->id.uuid.uuid.uuid128[5], srvc_id->id.uuid.uuid.uuid128[6],
						 srvc_id->id.uuid.uuid.uuid128[7], srvc_id->id.uuid.uuid.uuid128[8], srvc_id->id.uuid.uuid.uuid128[9],
						 srvc_id->id.uuid.uuid.uuid128[10], srvc_id->id.uuid.uuid.uuid128[11], srvc_id->id.uuid.uuid.uuid128[12],
						 srvc_id->id.uuid.uuid.uuid128[13], srvc_id->id.uuid.uuid.uuid128[14], srvc_id->id.uuid.uuid.uuid128[15]);
			} else {
				LOG_ERROR("SRVC UNKNOWN LEN %d", srvc_id->id.uuid.len);
			}
	        LOG_INFO("WRITE DESCR: gattc_if = %x",gattc_if);
            LOG_INFO("remote_bda %x,%x,%x,%x,%x,%x:",p_data->open.remote_bda[0],
            		p_data->open.remote_bda[1],p_data->open.remote_bda[2],
					p_data->open.remote_bda[3],p_data->open.remote_bda[4],
					p_data->open.remote_bda[5]);
            LOG_INFO("server_dba %x,%x,%x,%x,%x,%x:",server_dba[0],
            		server_dba[1],server_dba[2],
					server_dba[3],server_dba[4],
					server_dba[5]);
			//esp_ble_gattc_register_for_notify(gattc_if,server_dba,srvc_id,char_id);
            //esp_ble_gattc_close(gattc_if, conn_id);
//esp_ble_gattc_write_char(client_if, client_conn, &nrf51_service_id, &nrf51_char_id, sizeof(value), value, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);

        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
        // esp_gatt_srvc_id_t *srvc_id = &p_data->read.srvc_id;
        esp_gatt_id_t *char_id = &p_data->notify.char_id;
        conn_id = p_data->open.conn_id;
        LOG_INFO("NOTIFY: open.conn_id = %x search_res.conn_id = %x  notify.conn_id = %x", conn_id,p_data->search_res.conn_id,p_data->notify.conn_id);
        LOG_INFO("NOTIFY: notify.is_notify = %x inst_id = %x", p_data->notify.is_notify, char_id->inst_id);
        if (p_data->notify.is_notify==1) {
			if (char_id->uuid.len == ESP_UUID_LEN_16) {
				LOG_INFO("Char UUID16: %x", char_id->uuid.uuid.uuid16);
			} else if (char_id->uuid.len == ESP_UUID_LEN_32) {
				LOG_INFO("Char UUID32: %x", char_id->uuid.uuid.uuid32);
			} else if (char_id->uuid.len == ESP_UUID_LEN_128) {
				LOG_INFO("Char UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x", char_id->uuid.uuid.uuid128[0],
						 char_id->uuid.uuid.uuid128[1], char_id->uuid.uuid.uuid128[2], char_id->uuid.uuid.uuid128[3],
						 char_id->uuid.uuid.uuid128[4], char_id->uuid.uuid.uuid128[5], char_id->uuid.uuid.uuid128[6],
						 char_id->uuid.uuid.uuid128[7], char_id->uuid.uuid.uuid128[8], char_id->uuid.uuid.uuid128[9],
						 char_id->uuid.uuid.uuid128[10], char_id->uuid.uuid.uuid128[11], char_id->uuid.uuid.uuid128[12],
						 char_id->uuid.uuid.uuid128[13], char_id->uuid.uuid.uuid128[14], char_id->uuid.uuid.uuid128[15]);
			} else {
				LOG_ERROR("Char UNKNOWN LEN %d\n", char_id->uuid.len);
			}
            char notify_value[2];
            for (int i = 0; i < p_data->notify.value_len; i++) {
                LOG_INFO("NOTIFY: V%d %02x:", i, p_data->notify.value[i]);
                sprintf(notify_value, "%02x", p_data->notify.value[i]);
                strcat(get_request,notify_value);
            }
            
            if(p_data->notify.value[0]==0xaa &&p_data->notify.value[1]==0xaa &&p_data->notify.value[2]==0x55 &&p_data->notify.value[3]==0x55){
                esp_ble_gattc_close(client_if, client_conn);
                strcat(get_request,get_request_end);
                xTaskCreate(&http_get_task, "http_get_task", 2048, NULL, 5, NULL);
            }
        }
        break;
    }
    case ESP_GATTC_GET_CHAR_EVT: {
        esp_gatt_srvc_id_t *srvc_id = &p_data->get_char.srvc_id;
        esp_gatt_id_t *char_id = &p_data->get_char.char_id;
        conn_id = p_data->open.conn_id;
        uint8_t value[]={0x01,0x0a,0xaa,0xbb,0xcc,0xdd};
        
        LOG_INFO("GET CHAR: open.conn_id = %x search_res.conn_id = %x  get_char.conn_id = %x", conn_id,p_data->search_res.conn_id,p_data->get_char.conn_id);
        LOG_INFO("GET CHAR: get_char.char_prop = %x get_char.status = %x inst_id = %x open.gatt_if = %x", p_data->get_char.char_prop, p_data->get_char.status, char_id->inst_id,gattc_if);
        LOG_INFO("remote_bda %x,%x,%x,%x,%x,%x:",p_data->open.remote_bda[0],
        		p_data->open.remote_bda[1],p_data->open.remote_bda[2],
				p_data->open.remote_bda[3],p_data->open.remote_bda[4],
				p_data->open.remote_bda[5]);
        if (p_data->get_char.status==0) {
			if (char_id->uuid.len == ESP_UUID_LEN_16) {
				LOG_INFO("UUID16: %x", char_id->uuid.uuid.uuid16);
			} else if (char_id->uuid.len == ESP_UUID_LEN_32) {
				LOG_INFO("UUID32: %x", char_id->uuid.uuid.uuid32);
			} else if (char_id->uuid.len == ESP_UUID_LEN_128) {
				LOG_INFO("UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x", char_id->uuid.uuid.uuid128[0],
						 char_id->uuid.uuid.uuid128[1], char_id->uuid.uuid.uuid128[2], char_id->uuid.uuid.uuid128[3],
						 char_id->uuid.uuid.uuid128[4], char_id->uuid.uuid.uuid128[5], char_id->uuid.uuid.uuid128[6],
						 char_id->uuid.uuid.uuid128[7], char_id->uuid.uuid.uuid128[8], char_id->uuid.uuid.uuid128[9],
						 char_id->uuid.uuid.uuid128[10], char_id->uuid.uuid.uuid128[11], char_id->uuid.uuid.uuid128[12],
						 char_id->uuid.uuid.uuid128[13], char_id->uuid.uuid.uuid128[14], char_id->uuid.uuid.uuid128[15]);
                if(char_id->uuid.uuid.uuid128[12]==0x1){
                    LOG_INFO("value: %p %p %d %d", value,&value[5],client_if, client_conn);
                    //esp_ble_gattc_get_descriptor(gattc_if,conn_id,srvc_id,char_id,NULL);
                    //esp_ble_gattc_write_char_descr (gattc_if,conn_id,srvc_id,char_id,descr_id,2,&value[0],ESP_GATT_WRITE_TYPE_NO_RSP,ESP_GATT_AUTH_REQ_NONE);
                    //esp_ble_gattc_read_char(gattc_if,conn_id,srvc_id,char_id,ESP_GATT_AUTH_REQ_NONE);
                    //esp_ble_gattc_write_char(gattc_if, conn_id, srvc_id, char_id, 6, &value[0], ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
                    //esp_ble_gattc_write_char(client_if, client_conn, &nrf51_service_id, &nrf51_char_id, sizeof(value), value, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
                    //esp_ble_gattc_register_for_notify(gattc_if,server_dba,srvc_id,char_id);    
				//if (p_data->get_char.char_prop==18) {
					esp_ble_gattc_get_descriptor(gattc_if,conn_id,srvc_id,char_id,NULL);
				} else {
					esp_ble_gattc_get_characteristic(gattc_if,conn_id,srvc_id,char_id);
				}
			} else {
				LOG_ERROR("UNKNOWN LEN %d", char_id->uuid.len);
			}
        }
        break;
    }
      
    case ESP_GATTC_GET_DESCR_EVT: {
        esp_gatt_srvc_id_t *srvc_id = &p_data->get_descr.srvc_id;
        esp_gatt_id_t *char_id = &p_data->get_descr.char_id;
        esp_gatt_id_t *descr_id = &p_data->get_descr.descr_id;
        conn_id = p_data->open.conn_id;
        LOG_INFO("GET DESCR: open.conn_id = %x search_res.conn_id = %x  get_descr.conn_id = %x", conn_id,p_data->search_res.conn_id,p_data->get_descr.conn_id);
        LOG_INFO("GET DESCR: get_descr.status = %x inst_id = %x open.gatt_if = %x", p_data->get_descr.status, char_id->inst_id,gattc_if);
        //uint8_t value[]={'a','b','c'};
        uint8_t value[]={0x01,0x0a,0xaa,0xbb,0xcc,0xdd};
        if (p_data->get_descr.status==0) {
			if (char_id->uuid.len == ESP_UUID_LEN_16) {
				LOG_INFO("Char UUID16: %x", char_id->uuid.uuid.uuid16);
			} else if (char_id->uuid.len == ESP_UUID_LEN_32) {
				LOG_INFO("Char UUID32: %x", char_id->uuid.uuid.uuid32);
			} else if (char_id->uuid.len == ESP_UUID_LEN_128) {
				LOG_INFO("Char UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x", char_id->uuid.uuid.uuid128[0],
						 char_id->uuid.uuid.uuid128[1], char_id->uuid.uuid.uuid128[2], char_id->uuid.uuid.uuid128[3],
						 char_id->uuid.uuid.uuid128[4], char_id->uuid.uuid.uuid128[5], char_id->uuid.uuid.uuid128[6],
						 char_id->uuid.uuid.uuid128[7], char_id->uuid.uuid.uuid128[8], char_id->uuid.uuid.uuid128[9],
						 char_id->uuid.uuid.uuid128[10], char_id->uuid.uuid.uuid128[11], char_id->uuid.uuid.uuid128[12],
						 char_id->uuid.uuid.uuid128[13], char_id->uuid.uuid.uuid128[14], char_id->uuid.uuid.uuid128[15]);
			} else {
				LOG_ERROR("Char UNKNOWN LEN %d", char_id->uuid.len);
			}
			if (descr_id->uuid.len == ESP_UUID_LEN_16) {
				LOG_INFO("Decr UUID16: %x", descr_id->uuid.uuid.uuid16);
			} else if (descr_id->uuid.len == ESP_UUID_LEN_32) {
				LOG_INFO("Decr UUID32: %x", descr_id->uuid.uuid.uuid32);
			} else if (descr_id->uuid.len == ESP_UUID_LEN_128) {
				LOG_INFO("Decr UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x", descr_id->uuid.uuid.uuid128[0],
						 descr_id->uuid.uuid.uuid128[1], descr_id->uuid.uuid.uuid128[2], descr_id->uuid.uuid.uuid128[3],
						 descr_id->uuid.uuid.uuid128[4], descr_id->uuid.uuid.uuid128[5], descr_id->uuid.uuid.uuid128[6],
						 descr_id->uuid.uuid.uuid128[7], descr_id->uuid.uuid.uuid128[8], descr_id->uuid.uuid.uuid128[9],
						 descr_id->uuid.uuid.uuid128[10], descr_id->uuid.uuid.uuid128[11], descr_id->uuid.uuid.uuid128[12],
						 descr_id->uuid.uuid.uuid128[13], descr_id->uuid.uuid.uuid128[14], descr_id->uuid.uuid.uuid128[15]);
			} else {
				LOG_ERROR("Decr UNKNOWN LEN %d", descr_id->uuid.len);
			}
            LOG_INFO("sizeof value: %d", sizeof(value));
			//esp_ble_gattc_write_char_descr (gattc_if,conn_id,srvc_id,char_id,descr_id,4,&value[0],ESP_GATT_WRITE_TYPE_RSP,ESP_GATT_AUTH_REQ_NONE);
            //esp_ble_gattc_write_char_descr (client_if, client_conn, &nrf51_service_id, &nrf51_char_id,descr_id,sizeof(value),value,ESP_GATT_WRITE_TYPE_RSP,ESP_GATT_AUTH_REQ_NONE);
            esp_ble_gattc_write_char(client_if, client_conn, &nrf51_service_id, &nrf51_char_id, sizeof(value), value, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        }
        break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        uint8_t value[]={0x01,0x06};
        LOG_INFO("NOTIFY_EVT: open.conn_id = %x ", p_data->open.conn_id);
        LOG_INFO("NOTIFY_EVT: reg_for_notify.status = %x ", p_data->reg_for_notify.status);
        esp_gatt_srvc_id_t *srvc_id = &p_data->reg_for_notify.srvc_id;
        esp_gatt_id_t *char_id = &p_data->reg_for_notify.char_id;
        if (p_data->reg_for_notify.status==0) {
			if (char_id->uuid.len == ESP_UUID_LEN_16) {
				LOG_INFO("UUID16: %x", char_id->uuid.uuid.uuid16);
			} else if (char_id->uuid.len == ESP_UUID_LEN_32) {
				LOG_INFO("UUID32: %x", char_id->uuid.uuid.uuid32);
			} else if (char_id->uuid.len == ESP_UUID_LEN_128) {
				LOG_INFO("UUID128: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x", char_id->uuid.uuid.uuid128[0],
						 char_id->uuid.uuid.uuid128[1], char_id->uuid.uuid.uuid128[2], char_id->uuid.uuid.uuid128[3],
						 char_id->uuid.uuid.uuid128[4], char_id->uuid.uuid.uuid128[5], char_id->uuid.uuid.uuid128[6],
						 char_id->uuid.uuid.uuid128[7], char_id->uuid.uuid.uuid128[8], char_id->uuid.uuid.uuid128[9],
						 char_id->uuid.uuid.uuid128[10], char_id->uuid.uuid.uuid128[11], char_id->uuid.uuid.uuid128[12],
						 char_id->uuid.uuid.uuid128[13], char_id->uuid.uuid.uuid128[14], char_id->uuid.uuid.uuid128[15]);
			} else {
				LOG_ERROR("UNKNOWN LEN %d", char_id->uuid.len);
			}
            written=true;
            esp_ble_gattc_write_char(client_if, client_conn, &nrf51_service_id, &nrf51_char_id,sizeof(value), value, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        conn_id = p_data->search_cmpl.conn_id;
        LOG_INFO("SEARCH_CMPL: conn_id = %x, status %d", conn_id, p_data->search_cmpl.status);
        break;
    default:
        break;
    }
}

void ble_client_appRegister(void)
{
    LOG_INFO("register callback");

    //register the scan callback function to the Generic Access Profile (GAP) module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        LOG_ERROR("gap register error, error code = %x", status);
        return;
    }

    //register the callback function to the Generic Attribute Profile (GATT) Client (GATTC) module
    if ((status = esp_ble_gattc_register_callback(esp_gattc_cb)) != ESP_OK) {
        LOG_ERROR("gattc register error, error code = %x", status);
        return;
    }
    esp_ble_gattc_app_register(simpleClient_id);
    esp_ble_gap_set_scan_params(&ble_scan_params);
}

void gattc_client_test(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    ble_client_appRegister();
}

void app_main()
{
    get_request = malloc(2057);
    strcpy(get_request, get_request_start);

    ESP_ERROR_CHECK( nvs_flash_init() );
    initialise_wifi();
    esp_bt_controller_init();
    esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    
   
    gattc_client_test();
}

