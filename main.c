#include "config.h"

#include <lwip/init.h>
#include <bsp/board.h>
#include <pico/multicore.h>
#include <lwip/udp.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"

#include "tusb.h"
#include "usb_descriptors.h"

#define UDP_REPORT_SIZE 16

#define LED_SET(x)  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, (int)x)
#define LED_ON      LED_SET(true)
#define LED_OFF     LED_SET(false)
#define HALT        while (true) { LED_ON; sleep_ms(100); LED_OFF; sleep_ms(100); }

int connect_wifi();
void hid_task(void);
void copy_next_report();
static void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

uint8_t udp_buffer[UDP_REPORT_SIZE] = {0};
uint8_t next_report[UDP_REPORT_SIZE] = {0};
mutex_t report_mutex;

int main() {
    // Mutex
    mutex_init(&report_mutex);

    // STDIO
    stdio_init_all();

    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    // Wifi and LED.
    if (cyw43_arch_init()) {
        printf("[main] cyw43_arch_init failed");
        return 1;
    }

    // Initialize wifi code.
    int attempt = 0;
    do {
        sleep_ms(500);
    } while (connect_wifi() && ++attempt < 3);
    if (attempt >= 3) {
        printf("[main] Giving up.\n");
        HALT;
    }

    // Initialize UDP listener.
    struct udp_pcb *pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (udp_bind(pcb, IP_ANY_TYPE, 9999)) {
        printf("[udp] Fail to bind.\n");
        HALT;
    }
    udp_recv(pcb, udp_recv_callback, NULL);
    printf("[udp] Ready.\n");

    // Initialize TinyUSB
    tusb_init();

    // Main loop.
    while (1) {
        // Wifi polling
        cyw43_arch_poll();

        // TinyUSB task
        tud_task();

        // HID task
        hid_task();

        // Report <- UDP
        copy_next_report();

        // Print the next report for debugging
        if (next_report[0] > 0) {
            printf("[udp] Next report: ");
            for (int i = 0; i < UDP_REPORT_SIZE; i++) {
                printf("%02x ", next_report[i]);
            }
            printf("\n");
        }
    }
}

void copy_next_report() {
    // Try to acquire the mutex so we can manipulate the buffer safely
    if (!mutex_try_enter(&report_mutex, NULL)) {
        printf("[usb] Can't acquire mutex.\n");
        return;
    }

    if (udp_buffer[0] > 0) {
        // Copy the next report from the UDP buffer
        memcpy(next_report, udp_buffer, UDP_REPORT_SIZE);
        memset(udp_buffer, 0, UDP_REPORT_SIZE);
    }

    // Exit the mutex
    mutex_exit(&report_mutex);
}

// -------------------------------------------------------------------
// Wifi
// -------------------------------------------------------------------

int connect_wifi() {
    // Set client mode on wifi.
    cyw43_arch_enable_sta_mode();

    // Connect to it.
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("[wifi] Can't connect to wifi.\n");
        return 1;
    } else {
        printf("[wifi] Connected.\n");
        return 0;
    }
}

// -------------------------------------------------------------------
// Device callbacks
// -------------------------------------------------------------------

// Invoked when device is mounted
void tud_mount_cb(void)
{
    printf("[usb] Device mounted\n");
    LED_ON;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    printf("[usb] Device unmounted\n");
    LED_OFF;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    printf("[usb] Device suspended\n");
    LED_OFF;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    printf("[usb] Device resumed\n");
    LED_ON;
}

// -------------------------------------------------------------------
// USB HID
// -------------------------------------------------------------------

static void send_hid_report(uint8_t report_id) {
    // Skip if hid is not ready yet
    if (!tud_hid_ready()) return;

    if (next_report[0] == 0) {
        return;
    }

    // Check if we need to do something with the mouse.
    if (report_id == REPORT_ID_MOUSE) {
        tud_hid_mouse_report(REPORT_ID_MOUSE,
                             next_report[1],
                             (int8_t) next_report[2],
                             (int8_t) next_report[3],
                             0,
                             0);
    }

    if (report_id == REPORT_ID_KEYBOARD) {
        tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
    }

    // Clean up the report if we have sent all of it.
    if (report_id == REPORT_ID_COUNT-1) {
        next_report[0] = 0;
    }
}

// Every 10ms, we will sent one report for each HID profile (keyboard, mouse etc ..)
// tud_hid_report_complete_cb() is used to send the next report after previous one is complete
void hid_task(void) {
    // Poll every 10ms
    const uint32_t interval_ms = 10;
    static uint32_t start_ms = 0;

    if (board_millis() - start_ms < interval_ms) return; // not enough time
    start_ms += interval_ms;

    // Remote wakeup
    if (tud_suspended()) {
        // Wake up host if we are in suspend mode
        // and REMOTE_WAKEUP feature is enabled by host
        tud_remote_wakeup();
    } else {
        send_hid_report(REPORT_ID_KEYBOARD);
//        send_hid_report(REPORT_ID_MOUSE);
    }
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
    uint8_t next_report_id = report[0] + 1;

    if (next_report_id < REPORT_ID_COUNT) {
        send_hid_report(next_report_id);
    }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    printf("[hid] Not implemented: GET_REPORT\n");

    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    // Do nothing.
    printf("[hid] Not implemented: SET_REPORT\n");
}

// -------------------------------------------------------------------
// UDP
// -------------------------------------------------------------------

static void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    if (mutex_try_enter(&report_mutex, NULL)) {
        if (p != NULL && p->len <= sizeof(udp_buffer)) {
            // Copy the received data to our buffer
            memcpy(udp_buffer, p->payload, p->len);
        }
        mutex_exit(&report_mutex);
    } else {
        printf("[udp] Dropped packet\n");
    }

    // Free the received pbuf
    pbuf_free(p);
}
