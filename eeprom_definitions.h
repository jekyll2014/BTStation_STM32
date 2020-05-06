#pragma once
// ����� ������� � eeprom ������
#define EEPROM_STATION_NUMBER   00 // 1 byte

// ����� ������ � eeprom ������
#define EEPROM_STATION_MODE     10 // 1 byte

// �����. ��������� �������� ADC � ������ = 0,00587
#define EEPROM_VOLTAGE_KOEFF    20 // 4 byte

// �������� ������� RFID
#define EEPROM_GAIN             40 // 1 byte

// ��� ����, � ������� ������ �������� �������
#define EEPROM_CHIP_TYPE        50 // 1 byte

// ������ ����� �� ����� ��� ������ �������
#define EEPROM_TEAM_BLOCK_SIZE  60 // 2 byte

// ������ ���������� ����� �� �����
#define EEPROM_FLASH_BLOCK_SIZE 70 // 2 byte

// ����������� ���������� �������
#define EEPROM_BATTERY_LIMIT    80 // 4 byte
// �������� ��������� � ����� ������
#define EEPROM_AUTOREPORT       100 // 1 byte
