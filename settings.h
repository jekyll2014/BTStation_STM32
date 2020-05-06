#pragma once
// ������ ��������, ����� ������� � ����
#define FW_VERSION          109

#define UART_SPEED          115200
#define CHANNEL_USB         0
#define CHANNEL_UART1       1

#define BT_COMMAND_ENABLE   PA8  // BT module command mode
#define BT_CONNECTED        PB12 // BT connection state
#define BT_TX               PA9  // to BT RX
#define BT_RX               PA10 // to BT TX

#define RED_LED_PIN         PB13 // ��������� ������ (�������)
#define GREEN_LED_PIN       PB14 // ��������� �����
#define BUZZER_PIN          PB15 // �������

#define FLASH_SS_PIN        PB8 // SPI SELECT pin
#define FLASH_ENABLE_PIN    PB9 // SPI enable pin

#define RFID_RST_PIN        PA3 // ���� ������ reset
#define RFID_SS_PIN         PA4 // ���� ������ chip_select
#define RFID_MOSI_PIN       PA7 // ���� ������
#define RFID_MISO_PIN       PA6 // ���� ������
#define RFID_SCK_PIN        PA5 // ���� ������

#define BATTERY_PIN         PA1 // ����� ���������� �������

#define I2C_ADDRESS         0x3C

// ����-��� ������ ������� � ������� ������
#define RECEIVE_TIMEOUT     1000

// ������������� ������ ����
#define RFID_READ_PERIOD    1000

// ����-��� ������ ����� �� �������
#define DISPLAY_TIMEOUT     5000

// ������ ������ ��������� ������
#define LAST_TEAMS_LENGTH   10
