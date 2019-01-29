/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2014 Daniel Thompson <daniel@redfelineninja.org.uk>
 * Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <libopencm3/usb/dwc/otg_hs.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/audio.h>
#include <libopencm3/usb/midi.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


static int rx_count = 0;
static int tx_count = 0;
static int error_count = 0;

static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,    /* was 0x0110 in Table B-1 example descriptor */
    .bDeviceClass = 0,   /* device defined at interface level */
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x6666,  /* Prototype product vendor ID */
    .idProduct = 0x1239, /* dd if=/dev/random bs=2 count=1 | hexdump */
    .bcdDevice = 0x0001,
    .iManufacturer = 1,  /* index to string desc */
    .iProduct = 2,       /* index to string desc */
    .iSerialNumber = 3,  /* index to string desc */
    .bNumConfigurations = 1,
};


/*
 * Standard endpoint descriptors
 */
static const struct usb_endpoint_descriptor bulk_endp[] = {{
    /* Table B-11: MIDI Adapter Standard Bulk OUT Endpoint Descriptor */
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,
    .bmAttributes = USB_ENDPOINT_ATTR_ISOCHRONOUS | USB_ENDPOINT_ATTR_ASYNC,
    .wMaxPacketSize = 0x40,
    .bInterval = 0x01
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_ISOCHRONOUS | USB_ENDPOINT_ATTR_ASYNC,
    .wMaxPacketSize = 0x40,
    .bInterval = 0x01
} };

static const struct usb_interface_descriptor audio_control_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_VENDOR,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,
    .endpoint = bulk_endp

} };

static const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = audio_control_iface,
}};

/*
 * Table B-2: MIDI Adapter Configuration Descriptor
 */
static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0, /* can be anything, it is updated automatically
                  when the usb code prepares the descriptor */
    .bNumInterfaces = 1, /* control and data */
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80, /* bus powered */
    .bMaxPower = 0x32,

    .interface = ifaces
};

static char usb_serial_number[25]; /* 12 bytes of desig and a \0 */

static const char *usb_strings[] = {
    "libopencm3.org",
    "MIDI demo2",
    "10.23.4432"
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

#define GP  0x107   /* x^8 + x^2 + x + 1 */
#define DI  0x07

#define unchar unsigned char

unsigned short lfsr = 0xACE1u;
unsigned bit;

unsigned my_rand(void)
{
    bit  = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5) ) & 1;
    return lfsr =  (lfsr >> 1) | (bit << 15);
}

static unsigned char crc8_table[0x0100];     /* 8-bit table */

static void crc8_symb(unsigned char *crc, unsigned char m)
{
    *crc = crc8_table[(*crc) ^ m];
    *crc &= 0xFF;
}

unchar crc8(unchar *buff, int len) {
    unchar result = 0;
    int i;
    for (i = 0; i < len; ++i) {
        crc8_symb(&result, buff[i]);
    }
    return result;
}

static void init_crc8(void)
{
    static int is_initialized = false;
    int i,j;
    unsigned char crc;

    if (!is_initialized) {
        for (i = 0;i < 256; i++) {
            crc = i;
            for (j = 0; j < 8; j++)
                crc = (crc << 1) ^ ((crc & 0x80) ? DI : 0);
            crc8_table[i] = crc & 0xFF;
        }
        is_initialized = true;
    }
}

void rnd(unsigned char *buff, int len) {
    unsigned char random;
    int i = 0;
    for (i = 0; i < len; ++i) {
        if (i % 2 == 0) {
            random = my_rand();
            buff[i] = random & 0xff;
        } else {
            buff[i] = (random >> 8) & 0xff;
        }
    }
}

static void usbmidi_data_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    tx_count++;
}



static void usbmidi_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;
    rx_count++;

    char buf[64];
    int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

    // uchar

    if (len) {

        while (usbd_ep_write_packet(usbd_dev, 0x82, buf,
                        64) == 0);
    }

    gpio_toggle(GPIOC, GPIO5);
}

static void button_send_event(usbd_device *usbd_dev, int pressed)
{
    char buf[4] = { 0x08, /* USB framing: virtual cable 0, note on */
            0x80, /* MIDI command: note on, channel 1 */
            60,   /* Note 60 (middle C) */
            64,   /* "Normal" velocity */
    };

    buf[0] |= pressed;
    buf[1] |= pressed << 4;

    while (usbd_ep_write_packet(usbd_dev, 0x81, buf, sizeof(buf)) == 0);
}

static void button_poll(usbd_device *usbd_dev)
{
    static uint32_t button_state = 0;

    /* This is a simple shift based debounce. It's simplistic because
     * although this implements debounce adequately it does not have any
     * noise suppression. It is also very wide (32-bits) because it can
     * be polled in a very tight loop (no debounce timer).
     */
    uint32_t old_button_state = button_state;
    button_state = (button_state << 1) | (GPIOA_IDR & 1);
    if ((0 == button_state) != (0 == old_button_state)) {
        button_send_event(usbd_dev, !!button_state);
    }
}

static void usbmidi_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;

    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_ISOCHRONOUS, 64,
                  usbmidi_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_ISOCHRONOUS, 64,
                  usbmidi_data_tx_cb);
}

int main(void)
{
    usbd_device *usbd_dev;

    rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_120MHZ]);

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_OTGHS);

    /* USB pins */
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO14 | GPIO15);
    gpio_set_af(GPIOB, GPIO_AF12, GPIO14 | GPIO15);

    desig_get_unique_id_as_string(usb_serial_number, sizeof(usb_serial_number));

    /* Button pin */
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);

    usbd_dev = usbd_init(&otghs_usb_driver, &dev, &config,
            usb_strings, 3,
            usbd_control_buffer, sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(usbd_dev, usbmidi_set_config);
    while (1) {
        int rx = rx_count;
        int tx = tx_count;
        usbd_poll(usbd_dev);
        button_poll(usbd_dev);
    }
}
