#include "sd_util.h"

#define C_INFO ""
#define C_NORM ""
#define C_PROC ""
#define C_ERR  ""
#define C_DATA ""

u8 sdbuf[64];
u16 bitmax;
u32 sd_size;

// menus
enum
{
  M_NONE,
  M_MAIN,
  M_INFO,
  M_CID,
  M_CSD,
  M_OCR,
  M_SCR,
  M_SD_STATUS,
  M_LOCK,
  M_LOCK_SET_PWD,
  M_LOCK_CLR_PWD,
  M_LOCK_ERASE,
  M_ERASE,
  M_BFORCE
};

void print_rc(const char *last_op, u8 rc)
{
  u8 i;
  u8 r = rc;
  for (i = 0; i < countof (r1_err_txt); i++)
  {
    if (r & 1) break;
    r >>= 1;
  }

  printf(C_ERR "\r\n%s, rc: " C_DATA "(%02X) %s\r\n", last_op, rc, r1_err_txt[i]);
}

void print_yes_no(u8 a)
{
  printf("%s\r\n", a ? "Yes" : "No");
}

void print_cq(u8 a)
{
  if (a)
    printf("Yes, queue depth %u\r\n", a + 1);
  else
    printf("No\r\n");
}

void print_reserved(u16 a)
{
  printf("Reserved (%04X)\r\n", a);
}

void print_sd_type(u16 a)
{
  switch (a)
  {
    case 0:
      printf("Regular SD RD/WR\r\n");
    break;
    
    case 1:
      printf("SD ROM\r\n");
    break;
    
    case 2:
      printf("OTP\r\n");
    break;
    
    default:
      print_reserved(a);
  }
}

void print_speed_class(u8 a)
{
  if (a >= 5)
    print_reserved(a);
  else
  {
    const u8 cls[] = {0, 2, 4, 6, 10};
    printf("Class %u\r\n", cls[a]);
  }
}

void print_uhs_speed_grade(u8 a)
{
  if (a == 2 || a >= 4)
    print_reserved(a);
  else
  switch (a)
  {
    case 0:
      printf("Less than 10MB/sec\r\n");
    break;
    
    case 1:
      printf("10MB/sec and above\r\n");
    break;
    
    case 3:
      printf("30MB/sec and above\r\n");
    break;
  }
}

#if 0
u8 submenu_init()
{
  sd_size = 0;

  u8 rc = disk_initialize();
  printf(C_NORM "Card init: " C_DATA "%s   ", sd_init_txt[rc]);
  printf(C_NORM "Card type: " C_DATA "%s   ", sd_type_txt[ctype & 7]);

  if (rc == STA_INIT)
  {
    if (rc = read_status())
    {
      print_rc("read_status()", rc);
      return rc;
    }

    u8 st = sd_rbuf[0];
    printf(C_NORM "STATUS: " C_DATA "%slocked (%02X)   ", (st & 1) ? "" : "un", st);
  }

  if (!read_csd())
  {
    bitmax = 127;

    u8 ver = get_bitfield_bits(127, 2);
    if (ver > 1)    // reserved version of CSD
      goto exit;

    if (ver == 0)
      sd_size = (u32)(get_bitfield_bits(73, 12) + 1) * power(2, (get_bitfield_bits(49, 3) + 2));
    else
      sd_size = (u32)(get_bitfield_bits(69, 22) + 1) << 10;

    printf(C_NORM "LBA: " C_DATA "%lu\r\n", sd_size);
  }

exit:
  return rc;
}

void submenu_sd_status()
{
  printf(C_INFO "\r\nSD_STATUS: " C_DATA);
  hexstr(sdbuf, 16);      printf("\r\n           ");
  hexstr(sdbuf + 16, 16); printf("\r\n           ");
  hexstr(sdbuf + 32, 16); printf("\r\n           ");
  hexstr(sdbuf + 48, 16);
  printf("\r\n\n");
}

void submenu_cid()
{
  printf(C_INFO "\r\nCID: " C_DATA);
  hexstr(sdbuf, 16);
  printf("\r\n\n");

  bitmax = 127;
  u8 buf[16];

  u8 mid = get_bitfield_bits(127, 8);
  printf(C_INFO "Manufactorer ID:    " C_DATA "%s (%02X)\r\n", look_up_mid(mid), mid);

  get_bitfield_bytes(119, 2, buf);
  printf(C_INFO "OEM/Application ID: " C_DATA "%.2s\r\n", buf);

  get_bitfield_bytes(103, 5, buf);
  printf(C_INFO "Product name:       " C_DATA "%.5s\r\n", buf);

  get_bitfield_bytes(63, 1, buf);
  printf(C_INFO "Product revision:   " C_DATA "%d.%d\r\n", buf[0] >> 4, buf[0] & 0x0F);

  get_bitfield_bytes(55, 4, buf);
  printf(C_INFO "Product serial num: " C_DATA);
  hexstr(buf, 4);

  u16 mfd = get_bitfield_bits(19, 12);
  printf(C_INFO "\r\nManufacturing date: " C_DATA "%s %u\r\n", month_txt[(mfd & 0x0F) - 1], (mfd >> 4) + 2000);
}

void submenu_csd()
{
  printf(C_INFO "\r\nCSD: " C_DATA);
  hexstr(sdbuf, 16);
  printf("\r\n\n");

  bitmax = 127;

  u8 ver = get_bitfield_bits(127, 2);
  printf(C_INFO "CSD Version: " C_DATA "%s (%02X)\r\n", csd_ver_txt[ver], ver);
  if (ver > 1) return;  // reserved version of CSD

  TRAN_SPEED spd;
  spd.byte = get_bitfield_bits(103, 8);
  printf(C_INFO "Max data transfer rate per one line: " C_DATA "%u%sbit/s (%02X)\r\n", spd_val[spd.value] * spd_mul[spd.unit], spd_unit_txt[spd.unit], spd.byte);

  if (ver == 0)
  {
    u16 c_size = get_bitfield_bits(73, 12);
    u8 c_size_mult = get_bitfield_bits(49, 3);
    u8 read_bl_len = get_bitfield_bits(83, 4);

    u16 mult = power(2, (c_size_mult + 2));
    u16 bl_len = power(2, read_bl_len);
    u32 blocknr = (u32)(c_size + 1) * mult;
    u64 byte_size = blocknr * bl_len;

    printf(C_INFO "Read Block Length: " C_DATA "%u bytes (%02X)\r\n", bl_len, read_bl_len);
    printf(C_INFO "Device size: " C_DATA "%u MB (C_SIZE = %03X, C_SIZE_MULT = %02X)\r\n", (u16)(byte_size / 1024 / 1024) , c_size, c_size_mult);
  }
  else
  {
    u32 c_size = get_bitfield_bits(69, 22);
    u64 kbyte_size = (u64)(c_size + 1) * 512;

    printf(C_INFO "Device size: " C_DATA "%lu MB / %u GB (C_SIZE = %s)\r\n", (u32)(kbyte_size / 1024), (u16)(kbyte_size / 1024 / 1024), hex(&c_size, 3));
  }
}

void submenu_ocr()
{
  printf(C_INFO "\r\n\nOCR: " C_DATA);
  hexstr(sd_rbuf, 4);

  bitmax = 31;
}

void submenu_scr()
{
  printf(C_INFO "\r\n\nSCR: " C_DATA);
  hexstr(sdbuf, 8);

  bitmax = 63;
}

void menu_main()
{
  cls();
  xy(3, 0);
  printf(C_HEAD "SD Card Utility");
  x0(5); y(3);
  printf(C_BUTN "1. " C_MENU "Card Info\r\n\n");
  printf(C_BUTN "2. " C_MENU "SD Status\r\n\n");
  printf(C_BUTN "0. " C_ACHT "Lock/Erase\r\n\n");
}

void menu_info()
{
  u8 rc;

  cls();
  xy(3, 0);
  printf(C_HEAD "Card Info");
  xy(0, 2);
  submenu_init();

  // CID
  if (rc = read_cid())
    print_rc("read_cid()", rc);
  else
    submenu_cid();

  // CSD
  if (rc = read_csd())
    print_rc("\r\nread_csd()", rc);
  else
    submenu_csd();

  // SD_STATUS
  if (rc = read_sd_status())
    print_rc("\r\read_sd_status()", rc);
  else
    submenu_sd_status();
  
  // OCR
  // if (rc = read_ocr())
    // print_rc("\r\nread_ocr()", rc);
  // else
    // submenu_ocr();

  // SCR
  // if (rc = read_scr())
    // print_rc("\r\nread_scr()", rc);
  // else
    // submenu_scr();
}

void menu_sd_status()
{
  u8 rc;

  cls();
  xy(3, 0);
  printf(C_HEAD "SD Status");
  xy(0, 2);
  submenu_init();

  if (rc = read_sd_status())
  {
    print_rc("\r\read_sd_status()", rc);
    return;
  }
  
  bitmax = 511;
  printf("\r\n");
  
  u8 a8;
  u16 a16;
  u32 a32;
  
  a16 = get_bitfield_bits(495, 16); printf(C_INFO "SD_CARD_TYPE:        " C_DATA); print_sd_type(a16);
  a8 =  get_bitfield_bits(447, 8);  printf(C_INFO "SPEED_CLASS:         " C_DATA); print_speed_class(a8);
  a8 =  get_bitfield_bits(391, 8);  printf(C_INFO "VIDEO_SPEED_CLASS:   " C_DATA "%u\r\n", a8 );
  a8 =  get_bitfield_bits(339, 4);  printf(C_INFO "APP_PERF_CLASS:      " C_DATA "%u\r\n", a8 );
  a8 =  get_bitfield_bits(399, 4);  printf(C_INFO "UHS_SPEED_GRADE:     " C_DATA); print_uhs_speed_grade(a8);
  printf("\r\n");
  a8 =  get_bitfield_bits(335, 5);  printf(C_INFO "CQ_SUPPORT:          " C_DATA); print_cq(a8);
  a8 =  get_bitfield_bits(330, 1);  printf(C_INFO "CACHE_SUPPORT:       " C_DATA); print_yes_no(a8);
  a8 =  get_bitfield_bits(329, 1);  printf(C_INFO "HOST_MT-NCE_SUPPORT: " C_DATA); print_yes_no(a8);
  a8 =  get_bitfield_bits(328, 1);  printf(C_INFO "CARD_MT-NCE_SUPPORT: " C_DATA); print_yes_no(a8);
  a8 =  get_bitfield_bits(313, 1);  printf(C_INFO "DISCARD_SUPPORT:     " C_DATA); print_yes_no(a8);
  a8 =  get_bitfield_bits(312, 1);  printf(C_INFO "FULE_SUPPORT:        " C_DATA); print_yes_no(a8);
  printf("\r\n");
  a8 =  get_bitfield_bits(509, 1);  printf(C_INFO "SECURED_MODE:        " C_DATA); print_yes_no(a8);
  a8 =  get_bitfield_bits(511, 2);  printf(C_INFO "DAT_BUS_WIDTH:       " C_DATA "(%01X)\r\n", a8 );
  a32 = get_bitfield_bits(479, 32); printf(C_INFO "SIZE_OF_PROTCD_AREA: " C_DATA "(%08X)\r\n", a32); // !!!
  a8 =  get_bitfield_bits(439, 8);  printf(C_INFO "PERFORMANCE_MOVE:    " C_DATA "(%02X)\r\n", a8 );
  a16 = get_bitfield_bits(423, 16); printf(C_INFO "ERASE_SIZE:          " C_DATA "(%04X)\r\n", a16);
  a8 =  get_bitfield_bits(407, 6);  printf(C_INFO "ERASE_TIMEOUT:       " C_DATA "(%02X)\r\n", a8 );
  a8 =  get_bitfield_bits(401, 2);  printf(C_INFO "ERASE_OFFSET:        " C_DATA "(%01X)\r\n", a8 );
  a8 =  get_bitfield_bits(431, 4);  printf(C_INFO "AU_SIZE:             " C_DATA "(%01X)\r\n", a8 );
  a8 =  get_bitfield_bits(395, 4);  printf(C_INFO "UHS_AU_SIZE:         " C_DATA "(%01X)\r\n", a8 );
  a16 = get_bitfield_bits(377, 10); printf(C_INFO "VSC_AU_SIZE:         " C_DATA "(%03X)\r\n", a16);
  a32 = get_bitfield_bits(367, 22); printf(C_INFO "SUS_ADDR:            " C_DATA "(%06X)\r\n", a32); // !!!
  
  // printf(C_INFO ":  " C_DATA "(%02X)\r\n", get_bitfield_bits(, ));
}

void menu_lock()
{
  cls();
  xy(3, 0);
  printf(C_HEAD "Card lock/erase");
  xy(0, 2);
  u8 rc = submenu_init();

  if (rc == STA_INIT)
  {
    printf(C_BUTN "\r\n\n1. " C_MENU "Set password\r\n\n");
    printf(C_BUTN "2. " C_MENU "Clear password\r\n\n");
    printf(C_BUTN "3. " C_ACHT "Erase card\r\n\n");
    printf(C_BUTN "4. " C_ACHT "Erase sectors\r\n\n");
    printf(C_BUTN "5. " C_MENU "Bruteforce password\r\n\n");
    printf(C_BUTN "\r\n0. " C_MENU "Re-detect card\r\n\n\n\n");
  }
  else
  {
    menu = M_NONE;
    return;
  }

  sd_cs_on();
  sd_cs_off();
}

void menu_bforce()
{
  xy(0, 20);
  printf(C_PROC "Bruteforcing:");

  char pw[16];
  memset(pw, 0, sizeof(pw));
  u64 try = 0;

  for (u8 len = 1; len < 17; len++)
  {
  l1:
    xy(15, 20);
    printf(C_INFO "'" C_DATA "%s" C_INFO "' , try: %s" C_DATA, pw, hex(&try, 8));

    for (u8 ltr = 32; ltr; ltr++)
    {
      pw[len - 1] = ltr;

      // try to clear PWD
      try++;
      lock_unlock_sd_(SDC_CLR_PWD, pw, len);
      read_status();
      if (!sd_rbuf[0] & 1)
      {
        xy(15, 20);
        printf("\r\n\nDone! '%s'", pw);
        return;
      }
    }

    // move to the next PWD
    pw[len - 1] = 32;

    for (u8 i = len - 1; i; i--)
    {
      if (++pw[i - 1])
        goto l1;

      pw[i - 1] = 32;
    }
  }

  printf("\r\n\nOops...");
}

const char paswd[] = "123";

void menu_lock_set_pwd()
{
  printf(C_PROC "Setting password... ");

  u8 rc;
  if (rc = set_password_sd(paswd))
    print_rc("set_password_sd()", rc);
  else
    printf(C_OK "OK\r\n\n");
}

void menu_lock_clr_pwd()
{
  printf(C_PROC "Removing password... ");

  u8 rc;
  if (rc = clear_password_sd(paswd))
    print_rc("clear_password_sd()", rc);
  else
    printf(C_OK "OK\r\n\n");
}

void menu_lock_erase()
{
  read_status();
  if (!(sd_rbuf[0] & 1))
  {
    printf(C_ERR "Error: " C_INFO "card must be locked\r\n");
    printf(C_INFO "Set password, then remove card from the slot and insert it again");
  }
  else
  {
    printf(C_PROC "Erasing card... ");

    u8 rc;
    if (rc = erase_sd())
      print_rc("erase_sd()", rc);
    else
      printf(C_OK "OK\r\n\n");
  }
}

void menu_erase()
{
  read_status();
  printf(C_PROC "Erasing sectors... \r\n");

  u8 rc;
  if (rc = erase_sec())
    print_rc("\r\nerase_sd()", rc);
  else
    printf(C_OK "\r\nOK\r\n\n");
}

void menu_disp()
{
  switch (menu)
  {
    case M_MAIN:          menu_main(); break;
    case M_INFO:          menu_info(); break;
    case M_SD_STATUS:     menu_sd_status(); break;
    case M_LOCK:          menu_lock(); break;
    case M_LOCK_SET_PWD:  menu_lock_set_pwd(); break;
    case M_LOCK_CLR_PWD:  menu_lock_clr_pwd(); break;
    case M_LOCK_ERASE:    menu_lock_erase(); break;
    case M_ERASE:         menu_erase(); break;
    case M_BFORCE:        menu_bforce(); break;
  }
}

bool key_disp()
{
  bool rc = false;
  u8 key = getkey();

  if (req_unpress)
  {
    if (key == KEY_NONE)
      req_unpress = false;
  }
  else switch (menu)
  {
    case M_MAIN:
      switch(key)
      {
        case KEY_1: menu = M_INFO; rc = true; break;
        case KEY_2: menu = M_SD_STATUS; rc = true; break;
        case KEY_0: menu = M_LOCK; rc = true; break;
      }
    break;

    case M_LOCK_SET_PWD:
    case M_LOCK_CLR_PWD:
    case M_LOCK_ERASE:
      if (key != KEY_NONE)
        { menu = M_LOCK; rc = true; break; }
    break;

    case M_LOCK:
      switch(key)
      {
        case KEY_1: menu = M_LOCK_SET_PWD; rc = true; break;
        case KEY_2: menu = M_LOCK_CLR_PWD; rc = true; break;
        case KEY_3: menu = M_LOCK_ERASE; rc = true; break;
        case KEY_4: menu = M_ERASE; rc = true; break;
        case KEY_5: menu = M_BFORCE; rc = true; break;
        case KEY_0: menu = M_LOCK; rc = true; break;

        default:
          if (key != KEY_NONE)
            { menu = M_MAIN; rc = true; break; }
      }
    break;

    default:
      if (key != KEY_NONE)
        { menu = M_MAIN; rc = true; break; }
  }

  return rc;
}
#endif
