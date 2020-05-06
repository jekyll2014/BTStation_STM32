#pragma once
// номер станции в eeprom памяти
#define EEPROM_STATION_NUMBER   00 // 1 byte

// номер режима в eeprom памяти
#define EEPROM_STATION_MODE     10 // 1 byte

// коэфф. пересчета значения ADC в вольты = 0,00587
#define EEPROM_VOLTAGE_KOEFF    20 // 4 byte

// усиление сигнала RFID
#define EEPROM_GAIN             40 // 1 byte

// тип чипа, с которым должна работать станция
#define EEPROM_CHIP_TYPE        50 // 1 byte

// размер блока на флэше под данные команды
#define EEPROM_TEAM_BLOCK_SIZE  60 // 2 byte

// размер стираемого блока на флэше
#define EEPROM_FLASH_BLOCK_SIZE 70 // 2 byte

// минимальное напряжение батареи
#define EEPROM_BATTERY_LIMIT    80 // 4 byte
// включить автоотчет о новых сканах
#define EEPROM_AUTOREPORT       100 // 1 byte
