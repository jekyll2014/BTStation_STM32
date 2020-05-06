#pragma once
// �������� � ����. 0-7 ���������, 8-... ��� �������
#define PAGE_UID         0
#define PAGE_CHIP_SYS    3 // system[2] + ���_����[1] + system[1]
#define PAGE_CHIP_NUM    4 // �����_����[2] + ���_����[1] + ������_��������[1]
#define PAGE_INIT_TIME   5 // ����� �������������[4]
#define PAGE_TEAM_MASK   6 // ����� �������[2] + resserved[2]
//#define PAGE_RESERVED2   7 // reserved for future use[4]
#define PAGE_DATA_START  8 // 1st data page: ����� ��[1] + ����� ��������� ��[3]

#define NTAG213_ID       0x12
#define NTAG213_MARK     213
#define NTAG213_MAX_PAGE 40

#define NTAG215_ID       0x3e
#define NTAG215_MARK     215
#define NTAG215_MAX_PAGE 130

#define NTAG216_ID       0x6d
#define NTAG216_MARK     216
#define NTAG216_MAX_PAGE 226
