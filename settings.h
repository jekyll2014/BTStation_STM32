#pragma once
// версия прошивки, номер пишется в чипы
#define FW_VERSION          109

#define UART_SPEED          115200
#define CHANNEL_USB         0
#define CHANNEL_UART1       1

#define BT_COMMAND_ENABLE   PA8  // BT module command mode
#define BT_CONNECTED        PB12 // BT connection state
#define BT_TX               PA9  // to BT RX
#define BT_RX               PA10 // to BT TX

#define RED_LED_PIN         PB13 // светодиод ошибки (красный)
#define GREEN_LED_PIN       PB14 // светодиод синий
#define BUZZER_PIN          PB15 // пищалка

#define FLASH_SS_PIN        PB8 // SPI SELECT pin
#define FLASH_ENABLE_PIN    PB9 // SPI enable pin

#define RFID_RST_PIN        PA3 // рфид модуль reset
#define RFID_SS_PIN         PA4 // рфид модуль chip_select
#define RFID_MOSI_PIN       PA7 // рфид модуль
#define RFID_MISO_PIN       PA6 // рфид модуль
#define RFID_SCK_PIN        PA5 // рфид модуль

#define BATTERY_PIN         PA1 // замер напряжения батареи

#define I2C_ADDRESS         0x3C

// тайм-аут приема команды с момента начала
#define RECEIVE_TIMEOUT     1000

// периодичность поиска чипа
#define RFID_READ_PERIOD    1000

// тайм-аут вывода строк на дисплей
#define DISPLAY_TIMEOUT     5000

// размер буфера последних команд
#define LAST_TEAMS_LENGTH   10
