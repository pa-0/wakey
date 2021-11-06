#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>
#include <libopencm3/efm32/timer.h>
#include <libopencm3/efm32/common/prs_common.h>
#include <libopencm3/efm32/common/acmp_common.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "captouch.h"
#include "capsenseconfig.h"

#include <toboot.h>
TOBOOT_CONFIGURATION(0);
// Use below line instead of above to autorun program on inserting Tomu board
// You will need to connect the outer two button tracks to reprogram again
// TOBOOT_CONFIGURATION(TOBOOT_CONFIG_FLAG_AUTORUN);

#define CAP0B_PORT GPIOE
#define CAP0B_PIN GPIO12
#define CAP1B_PORT GPIOE
#define CAP1B_PIN GPIO13

// Minimum values for the capsense detect to work
#define CAPSENSE_DETECT_MIN 600

#pragma warning "Re-defining TIMER_CC_CTRL_INSEL because it's wrong"
#undef TIMER_CC_CTRL_INSEL
#define TIMER_CC_CTRL_INSEL (1 << 20)

#define EFM_ASSERT(x)

/* Systick interrupt frequency, Hz */
#define SYSTICK_FREQUENCY 100

/* Default AHB (core clock) frequency of Tomu board */
#define AHB_FREQUENCY 14000000

#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN GPIO0
#define LED_RED_PORT GPIOB
#define LED_RED_PIN GPIO7

#define VENDOR_ID 0x17ef  /* lenovo */
#define PRODUCT_ID 0x6047 /* keyboard */
#define DEVICE_VER 0x0101 /* Program version */

/**************************************************************************/ /**
* @brief This vector stores the latest read values from the ACMP
* @param ACMP_CHANNELS Vector of channels.
*****************************************************************************/
static volatile uint32_t g_channel_values[4] = {0};

/**************************************************************************/ /**
* @brief  This stores the maximum values seen by a channel
* @param ACMP_CHANNELS Vector of channels.
*****************************************************************************/
static volatile uint32_t channelMaxValues[4] = {0};

/** The current channel we are sensing. */
static volatile uint8_t g_current_channel;

/** Which generation of capsense we're on.  Monotonically increasing. */
static volatile uint32_t g_capsense_generation;

/** Set to true when we're freerunning capsense */
static volatile bool g_capsense_running = false;

/***************************************************************************/ /**
* @brief
*   Sets the ACMP channel used for capacative sensing.
*
* @note
*   A basic example of capacative sensing can be found in the STK BSP
*   (capsense demo).
*
* @param[in] acmp
*   Pointer to ACMP peripheral register block.
*
* @param[in] channel
*   The ACMP channel to use for capacative sensing (Possel).
******************************************************************************/

// Function prototypes so that our main loop can be at the top of the file for readability purposes
static void ACMP_CapsenseChannelSet(uint32_t channel);
static void CAPSENSE_Measure(uint32_t channel);
void timer0_isr(void);
void capsense_start(void);
void capsense_stop(void);
void setup_acmp_capsense(const struct acmp_capsense_init *init);
static void setup_capsense(void);
static void setup(void);

// Declare functions
void injkeys(char *source, uint8_t mod);
void send_key(int key, uint8_t mod);

bool g_usbd_is_connected = false;
bool once = true;
usbd_device *g_usbd_dev = 0;

static const struct usb_device_descriptor dev_descr = {
   .bLength = USB_DT_DEVICE_SIZE,
   .bDescriptorType = USB_DT_DEVICE,
   .bcdUSB = 0x0200,
   .bDeviceClass = 0,
   .bDeviceSubClass = 0,
   .bDeviceProtocol = 0,
   .bMaxPacketSize0 = 64,
   .idVendor = VENDOR_ID,
   .idProduct = PRODUCT_ID,
   .bcdDevice = DEVICE_VER,
   .iManufacturer = 1,
   .iProduct = 2,
   .iSerialNumber = 3,
   .bNumConfigurations = 1,
};

static const uint8_t hid_report_descriptor[] = {
   0x05, 0x01, // USAGE_PAGE (Generic Desktop)
   0x09, 0x06, // USAGE (Keyboard)
   0xa1, 0x01, // COLLECTION (Application)
   0x05, 0x07, // USAGE_PAGE (Keyboard)
   0x19, 0xe0, // USAGE_MINIMUM (Keyboard LeftControl)
   0x29, 0xe7, // USAGE_MAXIMUM (Keyboard Right GUI)
   0x15, 0x00, // LOGICAL_MINIMUM (0)
   0x25, 0x01, // LOGICAL_MAXIMUM (1)
   0x75, 0x01, // REPORT_SIZE (1)
   0x95, 0x08, // REPORT_COUNT (8)
   0x81, 0x02, // INPUT (Data,Var,Abs) //1 byte

   0x95, 0x01, // REPORT_COUNT (1)
   0x75, 0x08, // REPORT_SIZE (8)
   0x81, 0x03, // INPUT (Cnst,Var,Abs) //1 byte

   0x95, 0x06, // REPORT_COUNT (6)
   0x75, 0x08, // REPORT_SIZE (8)
   0x15, 0x00, // LOGICAL_MINIMUM (0)
   0x25, 0x65, // LOGICAL_MAXIMUM (101)
   0x05, 0x07, // USAGE_PAGE (Keyboard)
   0x19, 0x00, // USAGE_MINIMUM (Reserved (no event indicated))
   0x29, 0x65, // USAGE_MAXIMUM (Keyboard Application)
   0x81, 0x00, // INPUT (Data,Ary,Abs) //6 bytes

   0xc0, // END_COLLECTION
};

static const struct
{
   struct usb_hid_descriptor hid_descriptor;
   struct
   {
      uint8_t bReportDescriptorType;
      uint16_t wDescriptorLength;
   } __attribute__((packed)) hid_report;
} __attribute__((packed)) hid_function = {
   .hid_descriptor = {
      .bLength = sizeof(hid_function),
      .bDescriptorType = USB_DT_HID,
      .bcdHID = 0x0100,
      .bCountryCode = 0,
      .bNumDescriptors = 1,
   },
   .hid_report = {
      .bReportDescriptorType = USB_DT_REPORT,
      .wDescriptorLength = sizeof(hid_report_descriptor),
   }};

const struct usb_endpoint_descriptor hid_endpoint = {
   .bLength = USB_DT_ENDPOINT_SIZE,
   .bDescriptorType = USB_DT_ENDPOINT,
   .bEndpointAddress = 0x81,
   .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
   .wMaxPacketSize = 8,
   .bInterval = 0x20,
};

const struct usb_interface_descriptor hid_iface = {
   .bLength = USB_DT_INTERFACE_SIZE,
   .bDescriptorType = USB_DT_INTERFACE,
   .bInterfaceNumber = 0,
   .bAlternateSetting = 0,
   .bNumEndpoints = 1,
   .bInterfaceClass = USB_CLASS_HID,
   .bInterfaceSubClass = 1, /* boot */
   .bInterfaceProtocol = 1, // 1=keyboard, 2=mouse
   .iInterface = 0,

   .endpoint = &hid_endpoint,

   .extra = &hid_function,
   .extralen = sizeof(hid_function),
};

const struct usb_interface ifaces[] = {{
   .num_altsetting = 1,
   .altsetting = &hid_iface,
}};

const struct usb_config_descriptor config = {
   .bLength = USB_DT_CONFIGURATION_SIZE,
   .bDescriptorType = USB_DT_CONFIGURATION,
   .wTotalLength = 0,
   .bNumInterfaces = 1,
   .bConfigurationValue = 1,
   .iConfiguration = 0,
   .bmAttributes = 0xC0,
   .bMaxPower = 0x32,
   .interface = ifaces,
};

static const char *usb_strings[] = {
   "Tomu",
   "HID keyboard Demo",
   "DEMO",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
                                                         void (**complete)(usbd_device *, struct usb_setup_data *))
{
   (void)complete;
   (void)dev;

   if ((req->bmRequestType != 0x81) ||
      (req->bRequest != USB_REQ_GET_DESCRIPTOR) ||
      (req->wValue != 0x2200))
      return 0;

   /* Handle the HID report descriptor. */
   *buf = (uint8_t *)hid_report_descriptor;
   *len = sizeof(hid_report_descriptor);

   /* Dirty way to know if we're connected */
   g_usbd_is_connected = true;

   return 1;
}

static void hid_set_config(usbd_device *dev, uint16_t wValue)
{
   (void)wValue;
   (void)dev;

   usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 8, NULL);

   usbd_register_control_callback(
      dev,
      USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
      USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
      hid_control_request);
}

void usb_isr(void)
{
   usbd_poll(g_usbd_dev);
}

void hard_fault_handler(void)
{
   while (1)
      ;
}

// HID Usage Tables
// https://www.usb.org/sites/default/files/documents/hut1_12v2.pdf

void injkeys(char *source, uint8_t mod)
{
   // 0, 1 modifier
   // 2 key
   // 3-7 keep zer not sure what its for
   // 0, 1 modifier
   // 2 key
   // 3-7 keep zer not sure what its for

   static uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};         // key pressed
   static uint8_t key_release[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Key released
   int i;
   if (g_usbd_is_connected)
   {
      buf[0] = mod; // Key modifier, 2=LeftShift

      int lstr = strlen(source);
      // change ascii to keyboard map
      for (int j = 0; j < lstr; j++)
      {
            if (source[j] > 48 && source[j] < 58) // numbers 1-9
               buf[2] = source[j] - 19;
            else if (source[j] == 48) // number 0
               buf[2] = 39;
            else if (source[j] == 32) // space bar
               buf[2] = 44;
            else if (source[j] == 46) // .
               buf[2] = 55;
            else if (source[j] == '\r') // CR (Enter)
               buf[2] = 40;
            else if (source[j] == '\t') // tab
               buf[2] = 43;
            else if (source[j] == '!') // bang
            {
               buf[2] = 30;
               buf[0] = 2;  // lshift key modifier
            }
            else if (source[j] >= 'A' && source[j] <= 'Z')
            { // uppercase
               buf[2] = tolower(source[j]) - 93;
               buf[0] = 2;
            }

            else
            {
               buf[2] = source[j] - 93; // lowercase letters
            }

            usbd_ep_write_packet(g_usbd_dev, 0x81, buf, 8); //key down
            for (i = 0; i != 150; ++i)
               __asm__("nop");
            usbd_ep_write_packet(g_usbd_dev, 0x81, key_release, 8); //key up
            for (i = 0; i != 100000; ++i)                           //Wait a little
               __asm__("nop");
            usbd_ep_write_packet(g_usbd_dev, 0x81, key_release, 8); // just in case
            for (i = 0; i != 100000; ++i)                           //Wait a little
               __asm__("nop");

            // reset modifier
            buf[0] = 0;
      }
   }
}

void send_key(int key, uint8_t mod)
{
   // 0, 1 modifier
   // 2 key
   // 3-7 keep zer not sure what its for
   // 0, 1 modifier
   // 2 key
   // 3-7 keep zer not sure what its for

   static uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};         // key pressed
   static uint8_t key_release[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Key released
   int i;
   if (g_usbd_is_connected)
   {
      buf[0] = mod; // Key modifier, 2=LeftShift

      // send code out , you know what you are doing
      buf[2] = key;

      usbd_ep_write_packet(g_usbd_dev, 0x81, buf, 8); //key down
      for (i = 0; i != 150; ++i)
            __asm__("nop");
      usbd_ep_write_packet(g_usbd_dev, 0x81, key_release, 8); //key up
      for (i = 0; i != 100000; ++i)                           //Wait a little
            __asm__("nop");
      usbd_ep_write_packet(g_usbd_dev, 0x81, key_release, 8); // just in case
      for (i = 0; i != 100000; ++i)                           //Wait a little
            __asm__("nop");

      // reset modifier
      buf[0] = 0;
   }
}

int main(void)
{
   uint32_t last_generation = 0;
   uint32_t tick_count = 0;
   uint32_t wait_count = 0;
   uint32_t sum = 0;
   /* Make sure the vector table is relocated correctly (after the Tomu bootloader) */
   SCB_VTOR = 0x4000;

   /* Disable the watchdog that the bootloader started. */
   WDOG_CTRL = 0;

   // Setup much of the functionality necessary for the program
   setup();

   // Start the capsense functionality
   capsense_start();

   // Set both LEDs to turn them off
   gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
   gpio_set(LED_RED_PORT, LED_RED_PIN);

   /* GPIO peripheral clock is necessary for us to set up the GPIO pins as outputs */
   cmu_periph_clock_enable(CMU_GPIO);

   /* Configure the USB core & stack */
   g_usbd_dev = usbd_init(&efm32hg_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
   usbd_register_set_config_callback(g_usbd_dev, hid_set_config);

   /* Enable USB IRQs */
   nvic_set_priority(NVIC_USB_IRQ, 0x40);
   nvic_enable_irq(NVIC_USB_IRQ);

   /* Configure the system tick, at lower priority than USB IRQ */
   systick_set_frequency(SYSTICK_FREQUENCY, AHB_FREQUENCY);
   systick_counter_enable();
   systick_interrupt_enable();
   nvic_set_priority(NVIC_SYSTICK_IRQ, 0x10);

   gpio_set(LED_RED_PORT, LED_RED_PIN);
   while (1)
   {

      // send f15 key every ~5 seconds
      if (wait_count >= 7200000)
      {
            //7200000 should be about 10 minutes
            send_key(0x6a, 0); // f15
            // send_key(0x1b, 0); // x
            wait_count = 0;
      }
      else
      {
            wait_count++;
      }

      // Function to delay until the newest value from the capacitive buttons has come through
      // Notice the use of == instead of >= as at some point this variable will overflow and still
      // needs to operate
      while (g_capsense_generation == last_generation)
      {
      };

      // Change the generation number to show that it is a new generation
      last_generation = g_capsense_generation;

      // Go through each of the capsense values to determine when the buttons have been pressed
      for (int i = 0; i < 4; i++)
      {
            // Use the sum of their values to determine if a button has been pressed
            sum += g_channel_values[i];
      }
      // Minimum threshold for the button press to register
      if (sum > CAPSENSE_DETECT_MIN)
      {
            injkeys("Capacitive Sense touch detected... Hellow World!\n", 0);
      }
      // Reset the tick count to use as a timer
      tick_count = 0;

      // Reset the sum value so it doesn't grow across multiple loops
      sum = 0;
   }

   // Increment the tick counter so that LED timing can be preserved
   // an improvement would be to increment this in a system interrupt
   tick_count++;
}

static void ACMP_CapsenseChannelSet(uint32_t channel)
{
   g_current_channel = channel;

   if (channel == 0)
   {
      MMIO32(ACMP0_INPUTSEL) = (acmpResistor0 << _ACMP_INPUTSEL_CSRESSEL_SHIFT) | ACMP_INPUTSEL_CSRESEN | (false << _ACMP_INPUTSEL_LPREF_SHIFT) | (0x3f << _ACMP_INPUTSEL_VDDLEVEL_SHIFT) // 0x3f for channel 0 and 0x3d for channel 1
                                 | ACMP_INPUTSEL_NEGSEL(ACMP_INPUTSEL_NEGSEL_CAPSENSE) | (channel << _ACMP_INPUTSEL_POSSEL_SHIFT);
   }
   else if (channel == 1)
   {
      MMIO32(ACMP0_INPUTSEL) = (acmpResistor0 << _ACMP_INPUTSEL_CSRESSEL_SHIFT) | ACMP_INPUTSEL_CSRESEN | (false << _ACMP_INPUTSEL_LPREF_SHIFT) | (0x3d << _ACMP_INPUTSEL_VDDLEVEL_SHIFT) // 0x3f for channel 0 and 0x3d for channel 1
                                 | ACMP_INPUTSEL_NEGSEL(ACMP_INPUTSEL_NEGSEL_CAPSENSE) | (channel << _ACMP_INPUTSEL_POSSEL_SHIFT);
   }
   else if (channel == 2)
   {
   }
   else if (channel == 3)
   {
   }
   else
      while (1)
            ;
}

/**************************************************************************/ /**
* @brief
*   Start a capsense measurement of a specific channel and waits for
*   it to complete.
*****************************************************************************/
static void CAPSENSE_Measure(uint32_t channel)
{
   /* Set up this channel in the ACMP. */
   ACMP_CapsenseChannelSet(channel);

   /* Reset timers */
   TIMER0_CNT = 0;
   TIMER1_CNT = 0;

   /* Start timers */
   TIMER0_CMD = TIMER_CMD_START;
   TIMER1_CMD = TIMER_CMD_START;

   if (channel == 2)
   {
      gpio_mode_setup(CAP0B_PORT, GPIO_MODE_PUSH_PULL, CAP0B_PIN);
      gpio_set(CAP0B_PORT, CAP0B_PIN);
      gpio_mode_setup(CAP0B_PORT, GPIO_MODE_INPUT, CAP0B_PIN);
      while (gpio_get(CAP0B_PORT, CAP0B_PIN) && (TIMER0_CNT < (TIMER0_TOP - 5)))
            ;
      g_channel_values[channel] = TIMER0_CNT;
   }
   else if (channel == 3)
   {
      gpio_mode_setup(CAP1B_PORT, GPIO_MODE_PUSH_PULL, CAP1B_PIN);
      gpio_set(CAP1B_PORT, CAP1B_PIN);
      gpio_mode_setup(CAP1B_PORT, GPIO_MODE_INPUT, CAP1B_PIN);
      while (gpio_get(CAP1B_PORT, CAP1B_PIN) && (TIMER0_CNT < (TIMER0_TOP - 5)))
            ;
      g_channel_values[channel] = TIMER0_CNT;
   }
}

/**************************************************************************/ /**
* @brief
*   TIMER0 interrupt handler.
*
* @detail
*   When TIMER0 expires the number of pulses on TIMER1 is inserted into
*   channelValues. If this values is bigger than what is recorded in
*   channelMaxValues, channelMaxValues is updated.
*   Finally, the next ACMP channel is selected.
*****************************************************************************/
void timer0_isr(void)
{
   uint32_t count;

   /* Stop timers */
   TIMER0_CMD = TIMER_CMD_STOP;
   TIMER1_CMD = TIMER_CMD_STOP;

   /* Clear interrupt flag */
   TIMER0_IFC = TIMER_IFC_OF;

   /* Read out value of TIMER1 */
   count = TIMER1_CNT;

   /* Store value in channelValues */
   g_channel_values[g_current_channel] = count;

   /* Update channelMaxValues */
   if (count > channelMaxValues[g_current_channel])
      channelMaxValues[g_current_channel] = count;

   if (g_capsense_running)
   {
      if (g_current_channel >= 3)
      {
            g_capsense_generation++;
            g_current_channel = 0;
      }
      else
      {
            g_current_channel++;
      }
      CAPSENSE_Measure(g_current_channel);
   }
   else
   {
      /* Disable the ACMP, since capsense is no longer running */
      MMIO32(ACMP0_CTRL) &= ~ACMP_CTRL_EN;
   }
}

void capsense_start(void)
{
   g_capsense_running = true;

   /* Set the "Enable" Bit in ACMP, so we can make analog measurements */
   MMIO32(ACMP0_CTRL) |= ACMP_CTRL_EN;

   CAPSENSE_Measure(0);
}

void capsense_stop(void)
{
   g_capsense_running = false;
}

/***************************************************************************/ /**
* @brief
*   Sets up the ACMP for use in capacative sense applications.
*
* @details
*   This function sets up the ACMP for use in capacacitve sense applications.
*   To use the capacative sense functionality in the ACMP you need to use
*   the PRS output of the ACMP module to count the number of oscillations
*   in the capacative sense circuit (possibly using a TIMER).
*
* @note
*   A basic example of capacative sensing can be found in the STK BSP
*   (capsense demo).
*
* @param[in] acmp
*   Pointer to ACMP peripheral register block.
*
* @param[in] init
*   Pointer to initialization structure used to configure ACMP for capacative
*   sensing operation.
******************************************************************************/

void setup_acmp_capsense(const struct acmp_capsense_init *init)
{
   /* Make sure the module exists on the selected chip */
   EFM_ASSERT(ACMP_REF_VALID(acmp));

   /* Make sure that vddLevel is within bounds */
   EFM_ASSERT(init->vddLevel < 64);

   /* Make sure biasprog is within bounds */
   EFM_ASSERT(init->biasProg <=
               (_ACMP_CTRL_BIASPROG_MASK >> _ACMP_CTRL_BIASPROG_SHIFT));

   /* Set control register. No need to set interrupt modes */
   MMIO32(ACMP0_CTRL) = (init->fullBias << _ACMP_CTRL_FULLBIAS_SHIFT) | (init->halfBias << _ACMP_CTRL_HALFBIAS_SHIFT) | (init->biasProg << _ACMP_CTRL_BIASPROG_SHIFT) | (init->warmTime << _ACMP_CTRL_WARMTIME_SHIFT) | (init->hysteresisLevel << _ACMP_CTRL_HYSTSEL_SHIFT);

   /* Select capacative sensing mode by selecting a resistor and enabling it */
   MMIO32(ACMP0_INPUTSEL) = (init->resistor << _ACMP_INPUTSEL_CSRESSEL_SHIFT) | ACMP_INPUTSEL_CSRESEN | (init->lowPowerReferenceEnabled << _ACMP_INPUTSEL_LPREF_SHIFT) | (init->vddLevel << _ACMP_INPUTSEL_VDDLEVEL_SHIFT) | ACMP_INPUTSEL_NEGSEL(ACMP_INPUTSEL_NEGSEL_CAPSENSE);

   /* Enable ACMP if requested. */
   if (init->enable)
      MMIO32(ACMP0_CTRL) |= (1 << _ACMP_CTRL_EN_SHIFT);
}

static void setup_capsense(void)
{
   const struct acmp_capsense_init capsenseInit = ACMP_CAPSENSE_INIT_DEFAULT;
   CMU_HFPERCLKDIV |= CMU_HFPERCLKDIV_HFPERCLKEN;
   //cmu_periph_clock_enable(CMU_HFPER);
   cmu_periph_clock_enable(CMU_TIMER0);
   cmu_periph_clock_enable(CMU_TIMER1);

   CMU_HFPERCLKEN0 |= ACMP_CAPSENSE_CLKEN;
   cmu_periph_clock_enable(CMU_PRS);

   /* Initialize TIMER0 - Prescaler 2^9, top value 10, interrupt on overflow */
   TIMER0_CTRL = TIMER_CTRL_PRESC(TIMER_CTRL_PRESC_DIV512);
   TIMER0_TOP = 10;
   TIMER0_IEN = TIMER_IEN_OF;
   TIMER0_CNT = 0;

   /* Initialize TIMER1 - Prescaler 2^10, clock source CC1, top value 0xFFFF */
   TIMER1_CTRL = TIMER_CTRL_PRESC(TIMER_CTRL_PRESC_DIV1024) | TIMER_CTRL_CLKSEL(TIMER_CTRL_CLKSEL_CC1);
   TIMER1_TOP = 0xFFFF;

   /* Set up TIMER1 CC1 to trigger on PRS channel 0 */
   TIMER1_CC1_CTRL = TIMER_CC_CTRL_MODE(TIMER_CC_CTRL_MODE_INPUTCAPTURE)     /* Input capture      */
                     | TIMER_CC_CTRL_PRSSEL(TIMER_CC_CTRL_PRSSEL_PRSCH0)     /* PRS channel 0      */
                     | TIMER_CC_CTRL_INSEL                                   /* PRS input selected */
                     | TIMER_CC_CTRL_ICEVCTRL(TIMER_CC_CTRL_ICEVCTRL_RISING) /* PRS on rising edge */
                     | TIMER_CC_CTRL_ICEDGE(TIMER_CC_CTRL_ICEDGE_BOTH);      /* PRS on rising edge */

   /*Set up PRS channel 0 to trigger on ACMP0 output*/
   PRS_CH0_CTRL = PRS_CH_CTRL_EDSEL_POSEDGE                                    /* Posedge triggers action */
                  | PRS_CH_CTRL_SOURCESEL(PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE) /* PRS source */
                  | PRS_CH_CTRL_SIGSEL(PRS_CH_CTRL_SIGSEL_ACMPOUT_CAPSENSE);   /* PRS signal */

   /* Set up ACMP0 in capsense mode */
   setup_acmp_capsense(&capsenseInit);

   /* Enable TIMER0 interrupt */
   nvic_enable_irq(NVIC_TIMER0_IRQ);
}

static void setup(void)
{
   /* GPIO peripheral clock is necessary for us to set up the GPIO pins as outputs */
   cmu_periph_clock_enable(CMU_GPIO);

   /* Set up both LEDs as outputs */
   gpio_mode_setup(LED_RED_PORT, GPIO_MODE_WIRED_AND, LED_RED_PIN);
   gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_WIRED_AND, LED_GREEN_PIN);

   // Disable GPIO for pin PC1 (CAP1A, red LED button).
   // This pin was enabled as Output by Toboot bootloader, it interferes with the analog comparator.
   gpio_mode_setup(GPIOC, GPIO_MODE_DISABLE, GPIO1);

   setup_capsense();
}