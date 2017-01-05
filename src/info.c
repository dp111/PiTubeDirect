#include <stdio.h>
#include <string.h>
#include "info.h"
#include "tube-defs.h"

static char cmdline[PROP_SIZE];

static char info_string[PROP_SIZE];

void print_tag_value(char *name, rpi_mailbox_property_t *buf, int hex) {
   int i;
   LOG_INFO("%20s : ", name);
   if (buf == NULL) {
      LOG_INFO("*** failed ***");
   } else {
      for (i = 0;  i < (buf->byte_length + 3) >> 2; i++) {
         if (hex) {
            LOG_INFO("%08x ", buf->data.buffer_32[i]);
         } else {
            LOG_INFO("%8d ", buf->data.buffer_32[i]);
         }
      }
   }
   LOG_INFO("\r\n");
}

int get_revision() {
   rpi_mailbox_property_t *buf;
   RPI_PropertyInit();
   RPI_PropertyAddTag(TAG_GET_BOARD_REVISION);
   RPI_PropertyProcess();
   buf = RPI_PropertyGet(TAG_GET_BOARD_REVISION);
   if (buf) {
      return buf->data.buffer_32[0];
   } else {
      return 0;
   }
}

int get_clock_rate(int clk_id) {
   rpi_mailbox_property_t *buf;
   RPI_PropertyInit();
   RPI_PropertyAddTag(TAG_GET_CLOCK_RATE, clk_id);
   RPI_PropertyProcess();
   buf = RPI_PropertyGet(TAG_GET_CLOCK_RATE);
   if (buf) {
      return buf->data.buffer_32[1];
   } else {
      return 0;
   }
}

float get_temp() {
   rpi_mailbox_property_t *buf;
   RPI_PropertyInit();
   RPI_PropertyAddTag(TAG_GET_TEMPERATURE, 0);
   RPI_PropertyProcess();
   buf = RPI_PropertyGet(TAG_GET_TEMPERATURE);
   if (buf) {
      return ((float)buf->data.buffer_32[1]) / 1E3F;
   } else {
      return 0.0F;
   }
}

float get_voltage(int component_id) {
   rpi_mailbox_property_t *buf;
   RPI_PropertyInit();
   RPI_PropertyAddTag(TAG_GET_VOLTAGE, component_id);
   RPI_PropertyProcess();
   buf = RPI_PropertyGet(TAG_GET_VOLTAGE);
   if (buf) {
      return ((float) buf->data.buffer_32[1]) / 1E6F;
   } else {
      return 0.0F;
   }
}

// Model
// Speed
// Temp

char *get_info_string() {
   static int read = 0;
   if (!read) {
      sprintf(info_string, "%x %04d/%03dMHz %2.1fC", get_revision(), get_clock_rate(ARM_CLK_ID) / 1000000, get_clock_rate(CORE_CLK_ID) / 1000000, get_temp());
      read = 1;
   }
   return info_string;
}

char *get_cmdline() {
   static int read = 0;
   if (!read) {
      memset(cmdline, 0, PROP_SIZE);
      rpi_mailbox_property_t *buf;
      RPI_PropertyInit();
      RPI_PropertyAddTag(TAG_GET_COMMAND_LINE, 0);
      RPI_PropertyProcess();
      buf = RPI_PropertyGet(TAG_GET_COMMAND_LINE);
      if (buf) {
         memcpy(cmdline, buf->data.buffer_8, buf->byte_length);
         cmdline[buf->byte_length] = 0;
      } else {
         cmdline[0] = 0;
      }
      read = 1;
   }
   return cmdline;
}

char *get_cmdline_prop(char *prop) {
   static char ret[PROP_SIZE];
   char *retptr = ret;
   char *cmdline = get_cmdline();
   char *cmdptr = cmdline;
   int proplen = strlen(prop);

   // continue until the end terminator
   while (cmdptr && *cmdptr) {      
      // compare the property name
      if (strncasecmp(cmdptr, prop, proplen) == 0) {
         // check for an equals in the expected place
         if (*(cmdptr + proplen) == '=') {
            // skip the equals
            cmdptr += proplen + 1;
            // copy the property value to the return buffer
            while (*cmdptr != ' ' && *cmdptr != '\0') {
               *retptr++ = *cmdptr++;
            }
            *retptr = '\0';
            return ret;
         }
      }
      // Skip to the next property
      cmdptr = index(cmdptr, ' ');
      while (cmdptr && *cmdptr == ' ') {
         cmdptr++;
      }
   }
   return NULL;
}

clock_info_t * get_clock_rates(int clk_id) {
   static clock_info_t result;
   int *rp = (int *) &result;
   rpi_mailbox_tag_t tags[] = {
      TAG_GET_CLOCK_RATE,
      TAG_GET_MIN_CLOCK_RATE,
      TAG_GET_MAX_CLOCK_RATE
   };
   int i;
   int n = sizeof(tags) / sizeof(rpi_mailbox_tag_t);

   rpi_mailbox_property_t *buf;
   RPI_PropertyInit();
   for (i = 0; i < n; i++) {
      RPI_PropertyAddTag(tags[i], clk_id);
   }
   RPI_PropertyProcess();
   for (i = 0; i < n; i++) {
      buf = RPI_PropertyGet(tags[i]);
      *rp++ = buf ? buf->data.buffer_32[1] : 0;
   }
   return &result;
}

void dump_useful_info() {
   int i;
   rpi_mailbox_property_t *buf;
   clock_info_t *clk_info;

   rpi_mailbox_tag_t tags[] = {
        TAG_GET_FIRMWARE_VERSION
      , TAG_GET_BOARD_MODEL
      , TAG_GET_BOARD_REVISION
      , TAG_GET_BOARD_MAC_ADDRESS
      , TAG_GET_BOARD_SERIAL
      //, TAG_GET_ARM_MEMORY
      //, TAG_GET_VC_MEMORY
      //, TAG_GET_DMA_CHANNELS
      //, TAG_GET_CLOCKS
      //, TAG_GET_COMMAND_LINE
   };

   char *tagnames[] = {
      "FIRMWARE_VERSION"
      , "BOARD_MODEL"
      , "BOARD_REVISION"
      , "BOARD_MAC_ADDRESS"
      , "BOARD_SERIAL"
      //, "ARM_MEMORY"
      //, "VC_MEMORY"
      //, "DMA_CHANNEL"
      //, "CLOCKS"
      //, "COMMAND_LINE"
   };

   char *clock_names[] = {
      "RESERVED",
      "EMMC",
      "UART",
      "ARM",
      "CORE",
      "V3D",
      "H264",
      "ISP",
      "SDRAM",
      "PIXEL",
      "PWM"
   };

   int n = sizeof(tags) / sizeof(rpi_mailbox_tag_t);

   RPI_PropertyInit();
   for (i = 0; i < n ; i++) {
      RPI_PropertyAddTag(tags[i]);
   }

   RPI_PropertyProcess();
   
   for (i = 0; i < n; i++) {
      buf = RPI_PropertyGet(tags[i]);
      print_tag_value(tagnames[i], buf, 1);
   }

   for (i = MIN_CLK_ID; i <= MAX_CLK_ID; i++) {
      clk_info = get_clock_rates(i);
      LOG_INFO("%15s_FREQ : %10.3f MHz %10.3f MHz %10.3f MHz\r\n",
             clock_names[i],
             (double) (clk_info->rate)  / 1.0e6,
             (double) (clk_info->min_rate)  / 1.0e6,
             (double) (clk_info->max_rate)  / 1.0e6
         );
   }

   LOG_INFO("           CORE TEMP : %6.2f °C\r\n", get_temp());
   LOG_INFO("        CORE VOLTAGE : %6.2f V\r\n", get_voltage(COMPONENT_CORE));
   LOG_INFO("     SDRAM_C VOLTAGE : %6.2f V\r\n", get_voltage(COMPONENT_SDRAM_C));
   LOG_INFO("     SDRAM_P VOLTAGE : %6.2f V\r\n", get_voltage(COMPONENT_SDRAM_P));
   LOG_INFO("     SDRAM_I VOLTAGE : %6.2f V\r\n", get_voltage(COMPONENT_SDRAM_I));

   LOG_INFO("            CMD_LINE : %s\r\n", get_cmdline());

   LOG_INFO("               COPRO : %s\r\n", get_cmdline_prop("copro"));

}
