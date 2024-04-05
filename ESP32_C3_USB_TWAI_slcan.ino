/*=========================================================
  ESP32C3 USB-CAN CDC https://espressif-docs.readthedocs-hosted.com/
  Мост SLCAN https://github.com/mintynet/esp32-slcan
  ESP32-c3-MINI-1 can2tty, tty2can, SLCAN CAN parsing, SSH
  ОИМ (m) 20.07.23 ESP32C3 2M USB CDC On Boot Enable
  ОИМ (m) 20.07.23 ESP32C3 4M USB CDC On Boot Enable
  ============================================================*/

#include "driver/gpio.h"
#include "driver/twai.h"

#define LED_RED 03
#define LED_GRN 04
#define LED_BLU 05
#define USB_MIN 18
#define USB_PLS 19
#define CAN_RX GPIO_NUM_7
#define CAN_TX GPIO_NUM_6

//#define DEBUG

static uint8_t hexval[17] = "0123456789ABCDEF";
static char cmdbuf[32];
static int cmdidx = 0;
int ser_length;

bool connected = false;
bool working = false;
bool bluetooth = false;
bool timestamp = false;
bool disp_cnt = false;
bool cr = false;

int can_speed = 250;
int ser_speed = 115200;
int msg_cnt_in = 0;
int msg_cnt_out = 0;
twai_message_t messagerx, messagetx;

// Initialize configuration structures using macro initializers
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

//----------------------------------------------------------------
void slcan_ack() {
  Serial.write('\r');  // CR
}  // slcan_ack()

//----------------------------------------------------------------
void slcan_nack() {
  Serial.write('\a');
}  // slcan_nack()

//----------------------------------------------------------------
void pars_slcancmd(char *buf) {
  switch (buf[0]) {
    case 'O':  // Open CAN channel
      if (!working) {
        if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {  // Check install TWAI driver
          if (twai_start() != ESP_OK) {                                        // Start TWAI driver
            slcan_nack();
            digitalWrite(LED_RED, HIGH);
          } else {  // Install and start successfull
            working = true;
            slcan_ack();
            digitalWrite(LED_BLU, HIGH);
            delay(100);
            digitalWrite(LED_BLU, LOW);
          }
        } else {
          slcan_nack();
          digitalWrite(LED_RED, HIGH);
        }
      }
      break;
    case 'C':       // Close CAN channel
      if (working)  // check if working
      {
        if (twai_stop() != ESP_OK)  // stop TWAI driver
        {
          slcan_nack();
          digitalWrite(LED_RED, HIGH);
        } else {
          if (twai_driver_uninstall() != ESP_OK) {  // Uninstall TWAI driver
            slcan_nack();
            digitalWrite(LED_RED, HIGH);
          } else {  // Stop and uninstall successfull
            working = false;
            slcan_ack();
            digitalWrite(LED_GRN, HIGH);
            delay(100);
            digitalWrite(LED_GRN, LOW);
            delay(100);
            digitalWrite(LED_GRN, HIGH);
            delay(100);
            digitalWrite(LED_GRN, LOW);
          }
        }
      }
      break;
    case 't':  // SEND STD FRAME
      send_canmsg(buf, false, false);
      slcan_ack();
      break;
    case 'T':  // SEND EXT FRAME
      send_canmsg(buf, false, true);
      slcan_ack();
      break;
    case 'r':  // SEND STD RTR FRAME
      send_canmsg(buf, true, false);
      slcan_ack();
      break;
    case 'R':  // SEND EXT RTR FRAME
      send_canmsg(buf, true, true);
      slcan_ack();
      break;
    case 'Z':  // ENABLE TIMESTAMPS
      switch (buf[1]) {
        case '0':  // TIMESTAMP OFF
          timestamp = false;
          slcan_ack();
          break;
        case '1':  // TIMESTAMP ON
          timestamp = true;
          slcan_ack();
          break;
        default:
          break;
      }
      break;
    case 'M':  /// set ACCEPTANCE CODE ACn REG
      slcan_ack();
      break;
    case 'm':  // set ACCEPTANCE CODE AMn REG
      slcan_ack();
      break;
    case 's':  // CUSTOM CAN bit-rate
      slcan_nack();
      break;
    case 'S':  // CAN bit-rate
      if (working) {
        break;
      } else {
        switch (buf[1]) {
          case '0':  // 10k
            t_config = TWAI_TIMING_CONFIG_10KBITS();
            slcan_ack();
            break;
          case '1':  // 20k
            t_config = TWAI_TIMING_CONFIG_20KBITS();
            slcan_ack();
            break;
          case '2':  // 50k
            t_config = TWAI_TIMING_CONFIG_50KBITS();
            slcan_ack();
            break;
          case '3':  // 100k
            t_config = TWAI_TIMING_CONFIG_100KBITS();
            slcan_ack();
            break;
          case '4':  // 125k
            t_config = TWAI_TIMING_CONFIG_125KBITS();
            slcan_ack();
            break;
          case '5':  // 250k
            t_config = TWAI_TIMING_CONFIG_250KBITS();
            slcan_ack();
            break;
          case '6':  // 500k
            t_config = TWAI_TIMING_CONFIG_500KBITS();
            slcan_ack();
            break;
          case '7':  // 800k
            t_config = TWAI_TIMING_CONFIG_800KBITS();
            slcan_ack();
            break;
          case '8':  // 1Mbit
            t_config = TWAI_TIMING_CONFIG_1MBITS();
            slcan_ack();
            break;
          default:
            slcan_nack();
            digitalWrite(LED_RED, HIGH);
            break;
        }
      }
      break;
    case 'F':  // STATUS FLAGS
      Serial.print("F00");
      slcan_ack();
      break;
    case 'V':  // HARD VERSION NUMBER
      Serial.print("V1234");
      slcan_ack();
      break;
    case 'v':  // SOFT VERSION NUMBER
      Serial.print("V0003");
      slcan_ack();
      break;      
    case 'N':  // SERIAL NUMBER
      Serial.print("N2208");
      slcan_ack();
      break;
    default:
      slcan_nack();
      digitalWrite(LED_RED, HIGH);
      break;
  }
}  // pars_slcancmd()

//----------------------------------------------------------------
void send_canmsg(char *buf, boolean rtr, boolean ext) {
  if (!working) {
    //print_error(0);
    //print_status();
  } else {
    //CAN_frame_t tx_frame;
    int msg_id = 0;
    int msg_ide = 0;
    if (rtr) {
      if (ext) {
        sscanf(&buf[1], "%04x%04x", &msg_ide, &msg_id);
        messagetx.rtr = true;
        messagetx.extd = true;
      } else {
        sscanf(&buf[1], "%03x", &msg_id);
        messagetx.rtr = true;
        messagetx.extd = false;
      }
    } else {
      if (ext) {
        sscanf(&buf[1], "%04x%04x", &msg_ide, &msg_id);
        messagetx.rtr = false;
        messagetx.extd = true;
      } else {
        sscanf(&buf[1], "%03x", &msg_id);
        messagetx.rtr = false;
        messagetx.extd = false;
      }
    }
    messagetx.identifier = msg_ide * 65536 + msg_id;

    int msg_len = 0;
    if (ext) {
      sscanf(&buf[9], "%01x", &msg_len);
    } else {
      sscanf(&buf[4], "%01x", &msg_len);
    }

    messagetx.data_length_code = msg_len;
    int candata = 0;
    if (ext) {
      for (int i = 0; i < msg_len; i++) {
        sscanf(&buf[10 + (i * 2)], "%02x", &candata);
        messagetx.data[i] = candata;
      }
    } else {
      for (int i = 0; i < msg_len; i++) {
        sscanf(&buf[5 + (i * 2)], "%02x", &candata);
        messagetx.data[i] = candata;
      }
    }
    if (twai_transmit(&messagetx, 0) == ESP_OK) {  //10 tick?
      //DEBUG_PRINT('+');
    } else {
      //DEBUG_PRINT('-');
    }
    msg_cnt_out++;
  }
}  // send_canmsg()

//----------------------------------------------------------------
void setup() {
  Serial0.begin(115200, SERIAL_8N1, 20, 21);  // UART;
  Serial.begin();                             // USB CDC
  delay(1000);

  Serial0.print("\n\nESP32-C3 USB_TWAI_SLCAN3_SSH\n\r");
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_BLU, OUTPUT);

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial0.println("Driver installed\n");
  } else {
    Serial0.println("Failed to install driver\n");
    return;
  }
  if (twai_start() == ESP_OK) {  // Start TWAI driver
    Serial0.println("Driver started\n");
  } else {
    Serial0.println("Failed to start driver\n");
    return;
  }

  digitalWrite(LED_GRN, HIGH);
  delay(100);
  digitalWrite(LED_GRN, LOW);

  working = true;
}

//----------------------------------------------------------------
void transfer_can2usb() {
  twai_message_t rx_frame;
  String command = "";
  long time_now = 0;
  // receive next CAN frame from queue
  if (twai_receive(&rx_frame, 0) == ESP_OK) {
    if (working) {  // do stuff!
      digitalWrite(LED_BLU, HIGH);
      if (rx_frame.extd) {   // Message is extended
        if (rx_frame.rtr) {  // Message is Remote Transmission Request
          command = command + "R";
        } else {  // Message is Data Frame
          command = command + "T";
        }
        command = command + char(hexval[(rx_frame.identifier >> 28) & 1]);
        command = command + char(hexval[(rx_frame.identifier >> 24) & 15]);
        command = command + char(hexval[(rx_frame.identifier >> 20) & 15]);
        command = command + char(hexval[(rx_frame.identifier >> 16) & 15]);
        command = command + char(hexval[(rx_frame.identifier >> 12) & 15]);
        command = command + char(hexval[(rx_frame.identifier >> 8) & 15]);
        command = command + char(hexval[(rx_frame.identifier >> 4) & 15]);
        command = command + char(hexval[rx_frame.identifier & 15]);
        command = command + char(hexval[rx_frame.data_length_code]);
      } else {
        // Message is standard
        if (rx_frame.rtr) {
          // Message is Remote Transmission Request
          command = command + "r";
        } else {
          // Message is Data Frame
          command = command + "t";
        }
        command = command + char(hexval[(rx_frame.identifier >> 8) & 15]);
        command = command + char(hexval[(rx_frame.identifier >> 4) & 15]);
        command = command + char(hexval[rx_frame.identifier & 15]);
        command = command + char(hexval[rx_frame.data_length_code]);
      }
      for (int i = 0; i < rx_frame.data_length_code; i++) {
        command = command + char(hexval[rx_frame.data[i] >> 4]);
        command = command + char(hexval[rx_frame.data[i] & 15]);
        // printf("%c\t", (char)rx_frame.data.u8[i]);
      }
      if (timestamp) {
        time_now = millis() % 60000;
        command = command + char(hexval[(time_now >> 12) & 15]);
        command = command + char(hexval[(time_now >> 8) & 15]);
        command = command + char(hexval[(time_now >> 4) & 15]);
        command = command + char(hexval[time_now & 15]);
      }
      command = command + '\r';
      Serial.print(command);
      digitalWrite(LED_BLU, LOW);
    }
  }
}  // transfer_can2tty()

//----------------------------------------------------------------
void transfer_usb2can() {
  int ser_length;
  static char cmdbuf[32];
  static int cmdidx = 0;
  if ((ser_length = Serial.available()) > 0) {
    for (int i = 0; i < ser_length; i++) {
      char val = Serial.read();
      cmdbuf[cmdidx++] = val;
      if (cmdidx == 32) {
        slcan_nack();
        cmdidx = 0;
      } else if (val == '\r') {
        pars_slcancmd(cmdbuf);
        //Serial0.write('>');
        //Serial0.print(cmdbuf);  // DEBUG
        cmdidx = 0;
      }
    }
  }
}  // transfer_usb2can()

//----------------------------------------------------------------
void loop() {
  transfer_can2usb();
  transfer_usb2can();
}  // loop()