/*
 * STM 32 SCSI based on ArdSCSino. 
 */
#include <SPI.h>
#include "SdFat.h"
// To save some instructions and increase speed a bit it is possible to not compile the parity stuff. But most systems
// need it...
#define PARITY 1
// Uncomment the following line to include debuging over USB Serial port.
//#define DEBUG 1
// Make sure to use SdFat v1.x not v2.x
// Make sure to add ENABLE_EXTENDED_TRANSFER_CLASS 1 to the libraries/SdFat/SdFatConfig.h file.

SPIClass SPI_1(1);
SdFatEX  SD(&SPI_1);
#ifdef DEBUG 
#define LOG(XX)     Serial.print(XX)
#define LOGHEX(XX)  Serial.print(XX, HEX)
#define LOGN(XX)    Serial.println(XX)
#define LOGHEXN(XX) Serial.println(XX, HEX)
#else 
#define LOG(XX)     //Serial.print(XX)
#define LOGHEX(XX)  //Serial.print(XX, HEX)
#define LOGN(XX)    //Serial.println(XX)
#define LOGHEXN(XX) //Serial.println(XX, HEX)
#endif
#define high 0
#define low 1

#define isHigh(XX) ((XX) == high)
#define isLow(XX) ((XX) != high)

#define gpio_mode(pin,val) gpio_set_mode(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, val);
#define gpio_write(pin,val) gpio_write_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, val)
#define gpio_read(pin) gpio_read_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit)

//#define DB0       PB8     // SCSI:DB0
//#define DB1       PB9     // SCSI:DB1
//#define DB2       PB10    // SCSI:DB2
//#define DB3       PB11    // SCSI:DB3
//#define DB4       PB12    // SCSI:DB4
//#define DB5       PB13    // SCSI:DB5
//#define DB6       PB14    // SCSI:DB6
//#define DB7       PB15    // SCSI:DB7
//#define DBP       PB0     // SCSI:DBP

#define ATN       PA15      // SCSI:ATN
#define BSY       PB3      // SCSI:BSY
#define ACK       PB4     // SCSI:ACK
#define RST       PB5     // SCSI:RST
#define MSG       PB6      // SCSI:MSG
#define SEL       PB7      // SCSI:SEL
#define CD        PA2      // SCSI:I/O
#define REQ       PA1      // SCSI:REQ
#define IO        PA0      // SCSI:C/D

#define SD_CS     PA4      // SDCARD:CS
#define LED       PC13     // LED

#define UNITSELECT0 PC14
#define UNITSELECT1 PC15
#define UNITSELECT2 PA3
#define IMAGESELECT0 PA8
#define IMAGESELECT1 PB1

int SCSIID;             

#define BLOCKSIZE 512               // 1BLOCKサイズ
uint8_t       m_senseKey = 0;       //センスキー
volatile bool m_isBusReset = false; //バスリセット

#define HDIMG_FILE "HD.HDS"         // HDイメージファイル名
File          m_file;               // ファイルオブジェクト
uint32_t      m_fileSize;           // ファイルサイズ
byte          m_buf[BLOCKSIZE];     // 汎用バッファ

int           m_msc;
bool          m_msb[256];

/*
 * IO読み込み.
 */
inline byte readIO(void)
{
  return 0xff & (~(GPIOB->regs->IDR >> 8));
}

/* 
 * IO書き込み.
 */
inline void writeIO(byte v)
{
  #ifdef PARITY
  GPIOB->regs->ODR = (GPIOB->regs->ODR & 0x00fe) | (0xff00 & ((~v) <<8) )| parity(v) ;
  #else
  GPIOB->regs->ODR = GPIOB->regs->ODR & 0x00ff;
  #endif
}

/*
 * 初期化.
 *  パリティチェック
 */

#ifdef PARITY 
inline int parity(byte val) {
  val ^= val >> 4;
  val ^= val >> 2;
  val ^= val >> 1;
  return val & 0x00000001;
}
#endif

int getUnitSelectJumpers () {
  return 7-(digitalRead(UNITSELECT0) | digitalRead(UNITSELECT1) << 1 |  digitalRead(UNITSELECT2) << 2);
}

int getImageSelectJumpers () {
  return 3-(digitalRead(IMAGESELECT0) | digitalRead(IMAGESELECT1) << 1);
}

void blinkLED (int length) {
    gpio_write(LED, high);
    delay(length); 
    gpio_write(LED, low);
    delay(length);
}

/*
 *  Setup code
 */
void setup()
{
  char unitNo[2];
  char imageNo[2];
  int i;
  char fullFileName [13];
  fullFileName[0]=0;
  // PA15 / PB3 / PB4 is needed.
  // But only disable JTAG not SWD
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  // Activity LED
  gpio_mode(LED, GPIO_OUTPUT_OD);
  blinkLED(500);
  SCSIID = getUnitSelectJumpers();
  for (i=0;i<SCSIID;i++) {
    blinkLED(200);  
  }
  #ifdef DEBUG
  Serial.begin(9600);
  while (!Serial);
  blinkLED(200);
  blinkLED(200);
  Serial.print("SCSIID: ");
  Serial.println(SCSIID);
  #endif
  
  unitNo[0] = 0x30 + SCSIID;
  unitNo[1]=0;
  imageNo[0] = 0x30 + getImageSelectJumpers();
  imageNo[1] = 0;
  strcat(fullFileName, "DISK");
  strcat(fullFileName, unitNo);
  strcat(fullFileName, imageNo);
  strcat(fullFileName, ".DSK");
  
  GPIOB->regs->CRH = 0x88888888; // SET INPUT W/ PUPD on PB15-PB8
  gpio_mode(UNITSELECT0, GPIO_INPUT_PU);
  gpio_mode(UNITSELECT1, GPIO_INPUT_PU);
  gpio_mode(UNITSELECT2, GPIO_INPUT_PU);
  gpio_mode(IMAGESELECT0, GPIO_INPUT_PU);
  gpio_mode(IMAGESELECT1, GPIO_INPUT_PU);
  gpio_mode(ATN, GPIO_INPUT_PU);
  gpio_mode(BSY, GPIO_INPUT_PU);
  gpio_mode(ACK, GPIO_INPUT_PU);
  gpio_mode(RST, GPIO_INPUT_PU);
  gpio_mode(SEL, GPIO_INPUT_PU);
  
  gpio_mode(MSG, GPIO_OUTPUT_PP);
  gpio_mode(CD, GPIO_OUTPUT_PP);
  gpio_mode(REQ, GPIO_OUTPUT_PP);
  gpio_mode(IO, GPIO_OUTPUT_PP);

  gpio_write(MSG, low);
  gpio_write(CD, low);
  gpio_write(REQ, low);
  gpio_write(IO, low);

  //RST
  attachInterrupt(RST, onBusReset, FALLING);

  // Start up the SD card
  if(!SD.begin(SD_CS,SPI_FULL_SPEED)) {
    #ifdef DEBUG
    Serial.println("SD initialization failed!");
    #endif
    signalErrorCode(1);
  }
  #ifdef DEBUG 
  Serial.print("Trying to open file: ");
  Serial.println(fullFileName);
  #endif
  //Open the image -  file first try the filename based on the specified unit and image number.
  m_file = SD.open(fullFileName, O_RDWR);
  if (!m_file) {
    m_file = SD.open(HDIMG_FILE, O_RDWR);
    if(!m_file) {
      #ifdef DEBUG
      Serial.println("Error: open hdimg");
      #endif
      signalErrorCode(2);
    }
  }
  m_fileSize = m_file.size();
  #ifdef DEBUG
  Serial.println("Found Valid HD Image File.");
  Serial.print(m_fileSize);
  Serial.println("byte");
  Serial.print(m_fileSize / 1024);
  Serial.println("KB");
  Serial.print(m_fileSize / 1024 / 1024);
  Serial.println("MB");
  #endif
}




/*
 * Signal code # of short blinks followed by a long blink.
 */
void signalErrorCode(int code)
{
  while(true) {
    for (int i=0; i< code; i++) {
      blinkLED(200);
    }
    blinkLED(1000);
  }
}

/*
 * バスリセット割り込み.
 */
void onBusReset(void)
{
  if(isHigh(gpio_read(RST))) {
    delayMicroseconds(20);
    if(isHigh(gpio_read(RST))) {
      LOGN("BusReset!");
      m_isBusReset = true;
    }
  }
}

inline void setREQActive() {
  GPIOA->regs->BRR = 1 << 1;  
}

inline void setREQInactive() {
  GPIOA->regs->BSRR = 1 << 1;  
}

inline int isACKActive() {
 return GPIOB->regs->IDR & (1 << 4);
}

/*
 * Handshake one byte from the host.
 */
inline byte readHandshake(void)
{
  setREQActive();
  while(isACKActive()) {
    if(m_isBusReset) {
      return 0;
    }
  }
  byte r = readIO();
  setREQInactive();
  while(!isACKActive()) {
    if(m_isBusReset) {
      return 0;
    }
  }
  return r;  
}


/*
 *  Handshake of one single byte towardss the host.
 */
inline void writeHandshake(byte d)
{
  writeIO(d);
  setREQActive();
  while(isACKActive()) {
    if(m_isBusReset) {
      return;
    }
  }
  setREQInactive();
  while(!isACKActive()) {
    if(m_isBusReset) {
      return;
    }
  }
}

/*
 * データインフェーズ.
 *  データ配列 p を len バイト送信する。
 */
void writeDataPhase(int len, byte* p)
{
  LOGN("DATAIN PHASE");
  gpio_write(MSG, low);
  gpio_write(CD, low);
  gpio_write(IO, high);
  GPIOB->regs->CRL |= 0x00000003; // SET OUTPUT W/ PUPD on PA7-PB0 50MHz
  GPIOB->regs->CRH = 0x33333333; // SET OUTPUT W/ PUPD on PB15-PB8 50MHz
  for (int i = 0; i < len; i++) {
    if(m_isBusReset) {
      return;
    }
    writeHandshake(p[i]);
  }
}

/* 
 * データインフェーズ.
 *  SDカードからの読み込みながら len ブロック送信する。
 */
void writeDataPhaseSD(uint32_t adds, uint32_t len)
{
  LOGN("DATAIN PHASE(SD)");
  uint32_t pos = adds * BLOCKSIZE;
  m_file.seek(pos);
  gpio_write(MSG, low);
  gpio_write(CD, low);
  gpio_write(IO, high);
  GPIOB->regs->CRL |= 0x00000003; // SET OUTPUT W/ PUPD on PA7-PB0 50MHz
  GPIOB->regs->CRH = 0x33333333; // SET OUTPUT W/ PUPD on PB15-PB8 50MHz
  for(uint32_t i = 0; i < len; i++) {
    m_file.read(m_buf, BLOCKSIZE);
    for(int j = 0; j < BLOCKSIZE; j++) {
      if(m_isBusReset) {
        return;
      }
      writeHandshake(m_buf[j]);
    }
  }
}

/*
 * データアウトフェーズ.
 *  len ブロック読み込みながら SDカードへ書き込む。
 */
void readDataPhaseSD(uint32_t adds, uint32_t len)
{
  LOGN("DATAOUT PHASE(SD)");
  uint32_t pos = adds * BLOCKSIZE;
  m_file.seek(pos);
  gpio_write(MSG, low);
  gpio_write(CD, low);
  gpio_write(IO, low);
  GPIOB->regs->CRH = 0x88888888; // SET INPUT W/ PUPD on PB15-PB8
  for(uint32_t i = 0; i < len; i++) {
    for(int j = 0; j < BLOCKSIZE; j++) {
      if(m_isBusReset) {
        return;
      }
      m_buf[j] = readHandshake();
    }
    m_file.write(m_buf, BLOCKSIZE);
  }
  m_file.flush();
}

/*
 * INQUIRY コマンド処理.
 */
void onInquiryCommand(byte len)
{
  byte buf[36] = {
    0x00, //デバイスタイプ
    0x00, //RMB = 0
    0x01, //ISO,ECMA,ANSIバージョン
    0x01, //レスポンスデータ形式
    35 - 4, //追加データ長
    0, 0, //Reserve
    0x00, //サポート機能
    'T', 'N', 'B', ' ', ' ', ' ', ' ', ' ',
    'A', 'r', 'd', 'S', 'C', 'S', 'i', 'n', 'o', ' ', ' ',' ', ' ', ' ', ' ', ' ',
    '0', '0', '1', '0',
  };
  writeDataPhase(len < 36 ? len : 36, buf);
}

/*
 * REQUEST SENSE コマンド処理.
 */
void onRequestSenseCommand(byte len)
{
  byte buf[18] = {
    0x70,   //CheckCondition
    0,      //セグメント番号
    0x00,   //センスキー
    0, 0, 0, 0,  //インフォメーション
    17 - 7 ,   //追加データ長
    0,
  };
  buf[2] = m_senseKey;
  m_senseKey = 0;
  writeDataPhase(len < 18 ? len : 18, buf);  
}

/*
 * READ CAPACITY コマンド処理.
 */
void onReadCapacityCommand(byte pmi)
{
  uint32_t bc = m_fileSize / BLOCKSIZE;
  uint32_t bl = BLOCKSIZE;
  uint8_t buf[8] = {
    bc >> 24, bc >> 16, bc >> 8, bc,
    bl >> 24, bl >> 16, bl >> 8, bl    
  };
  writeDataPhase(8, buf);
}

/*
 * READ6/10 コマンド処理.
 */
byte onReadCommand(uint32_t adds, uint32_t len)
{
  LOGN("-R");
  LOGHEXN(adds);
  LOGHEXN(len);
  gpio_write(LED, high);
  writeDataPhaseSD(adds, len);
  gpio_write(LED, low);
  return 0; //sts
}

/*
 * WRITE6/10 コマンド処理.
 */
byte onWriteCommand(uint32_t adds, uint32_t len)
{
  LOGN("-W");
  LOGHEXN(adds);
  LOGHEXN(len);
  gpio_write(LED, high);
  readDataPhaseSD(adds, len);
  gpio_write(LED, low);
  return 0; //sts
}

/*
 * MODE SENSE コマンド処理.
 */
void onModeSenseCommand(byte dbd, int pageCode, uint32_t len)
{
  memset(m_buf, 0, sizeof(m_buf)); 
  int a = 4;
  if(dbd == 0) {
    uint32_t bc = m_fileSize / BLOCKSIZE;
    uint32_t bl = BLOCKSIZE;
    byte c[8] = {
      0,//デンシティコード
      bc >> 16, bc >> 8, bc,
      0, //Reserve
      bl >> 16, bl >> 8, bl    
    };
    memcpy(&m_buf[4], c, 8);
    a += 8;
    m_buf[3] = 0x08;
  }
  switch(pageCode) {
  case 0x3F:
  case 0x03:  //ドライブパラメータ
    m_buf[a + 0] = 0x03; //ページコード
    m_buf[a + 1] = 0x16; // ページ長
    m_buf[a + 11] = 0x3F;//セクタ数/トラック
    a += 24;
    if(pageCode != 0x3F) {
      break;
    }
  case 0x04:  //ドライブパラメータ
    {
      uint32_t bc = m_fileSize / BLOCKSIZE;
      m_buf[a + 0] = 0x04; //ページコード
      m_buf[a + 1] = 0x16; // ページ長
      m_buf[a + 2] = bc >> 16;// シリンダ長
      m_buf[a + 3] = bc >> 8;
      m_buf[a + 4] = bc;
      m_buf[a + 5] = 1;   //ヘッド数
      a += 24;
    }
    if(pageCode != 0x3F) {
      break;
    }
  default:
    break;
  }
  m_buf[0] = a - 1;
  writeDataPhase(len < a ? len : a, m_buf);
}

/*
 * MsgIn2.
 */
void MsgIn2(int msg)
{
  LOGN("MsgIn2");
  gpio_write(MSG, high);
  gpio_write(CD, high);
  gpio_write(IO, high);
  GPIOB->regs->CRL |= 0x00000003; // SET OUTPUT W/ PUPD on PA7-PB0 50MHz
  GPIOB->regs->CRH = 0x33333333; // SET OUTPUT W/ PUPD on PB15-PB8 50MHz
  writeHandshake(msg);
}

/*
 * MsgOut2.
 */
void MsgOut2()
{
  LOGN("MsgOut2");
  gpio_write(MSG, high);
  gpio_write(CD, high);
  gpio_write(IO, low);
  GPIOB->regs->CRH = 0x88888888; // SET INPUT W/ PUPD on PB15-PB8
  m_msb[m_msc] = readHandshake();
  m_msc++;
  m_msc %= 256;
}

/*
 * メインループ.
 */
void loop() 
{
  int sts = 0;
  int msg = 0;

  //BSY,SELが+はバスフリー
  // セレクションチェック
  // BSYが-の間ループ
  if(isHigh(gpio_read(BSY))) {
    return;
  }
  // SELが+の間ループ
  if(isLow(gpio_read(SEL))) {
    return;
  }
  // BSY+ SEL-
  GPIOB->regs->CRH = 0x88888888; // SET INPUT W/ PUPD on PB15-PB8
  byte db = readIO();  
  LOGN("SELECTION");
  LOGHEX(db);
  LOGN("");
  LOGHEX(SCSIID);
  if((db & (1 << SCSIID)) == 0) {
    return;
  }

  LOGN("Selection");
  m_isBusReset = false;
  // セレクトされたらBSYを-にする
  gpio_mode(BSY, GPIO_OUTPUT_PP);
  gpio_write(BSY, high);
  while(isHigh(gpio_read(SEL))) {
    if(m_isBusReset) {
      goto BusFree;
    }
  }
  if(isHigh(gpio_read(ATN))) {
    bool syncenable = false;
    int syncperiod = 50;
    int syncoffset = 0;
    m_msc = 0;
    memset(m_msb, 0x00, sizeof(m_msb));
    while(isHigh(gpio_read(ATN))) {
      MsgOut2();
    }
    for(int i = 0; i < m_msc; i++) {
      // ABORT
      if (m_msb[i] == 0x06) {
        goto BusFree;
      }
      // BUS DEVICE RESET
      if (m_msb[i] == 0x0C) {
        syncoffset = 0;
        goto BusFree;
      }
      // IDENTIFY
      if (m_msb[i] >= 0x80) {
      }
      // 拡張メッセージ
      if (m_msb[i] == 0x01) {
        // 同期転送が可能な時だけチェック
        if (!syncenable || m_msb[i + 2] != 0x01) {
          MsgIn2(0x07);
          break;
        }
        // Transfer period factor(50 x 4 = 200nsに制限)
        syncperiod = m_msb[i + 3];
        if (syncperiod > 50) {
          syncoffset = 50;
        }
        // REQ/ACK offset(16に制限)
        syncoffset = m_msb[i + 4];
        if (syncoffset > 16) {
          syncoffset = 16;
        }
        // STDR応答メッセージ生成
        MsgIn2(0x01);
        MsgIn2(0x03);
        MsgIn2(0x01);
        MsgIn2(syncperiod);
        MsgIn2(syncoffset);
        break;
      }
    }
  }

  LOGN("Command");
  gpio_write(MSG, low);
  gpio_write(CD, high);
  gpio_write(IO, low);
  int len;
  byte cmd[12];
  GPIOB->regs->CRH = 0x88888888; // SET INPUT W/ PUPD on PB15-PB8
  cmd[0] = readHandshake();
  LOGHEX(cmd[0]);
  len = 1;
  switch(cmd[0] >> 5) {
  case 0b000:
    len = 6;
    break;
  case 0b001:
    len = 10;
    break;
  case 0b010:
    len = 10;
    break;
  case 0b101:
    len = 12;
    break;
  default:
    break;
  }
  GPIOB->regs->CRH = 0x88888888; // SET INPUT W/ PUPD on PB15-PB8
  for(int i = 1; i < len; i++ ) {
    cmd[i] = readHandshake();
    LOGHEX(cmd[i]);
  }
  LOGN("");

  switch(cmd[0]) {
  case 0x00:
    LOGN("[Test Unit]");
    break;
  case 0x01:
    LOGN("[Rezero Unit]");
    break;
  case 0x03:
    LOGN("[RequestSense]");
    onRequestSenseCommand(cmd[4]);
    break;
  case 0x04:
    LOGN("[FormatUnit]");
    break;
  case 0x06:
    LOGN("[FormatUnit]");
    break;
  case 0x07:
    LOGN("[ReassignBlocks]");
    break;
  case 0x08:
    LOGN("[Read6]");
    sts = onReadCommand((((uint32_t)cmd[1] & 0x1F) << 16) | ((uint32_t)cmd[2] << 8) | cmd[3], (cmd[4] == 0) ? 0x100 : cmd[4]);
    break;
  case 0x0A:
    LOGN("[Write6]");
    sts = onWriteCommand((((uint32_t)cmd[1] & 0x1F) << 16) | ((uint32_t)cmd[2] << 8) | cmd[3], (cmd[4] == 0) ? 0x100 : cmd[4]);
    break;
  case 0x0B:
    LOGN("[Seek6]");
    break;
  case 0x12:
    LOGN("[Inquiry]");
    onInquiryCommand(cmd[4]);
    break;
  case 0x1A:
    LOGN("[ModeSense6]");
    onModeSenseCommand(cmd[1]&0x80, cmd[2] & 0x3F, cmd[4]);
    break;
  case 0x1B:
    LOGN("[StartStopUnit]");
    break;
  case 0x1E:
    LOGN("[PreAllowMed.Removal]");
    break;
  case 0x25:
    LOGN("[ReadCapacity]");
    onReadCapacityCommand(cmd[8]);
    break;
  case 0x28:
    LOGN("[Read10]");
    sts = onReadCommand(((uint32_t)cmd[2] << 24) | ((uint32_t)cmd[3] << 16) | ((uint32_t)cmd[4] << 8) | cmd[5], ((uint32_t)cmd[7] << 8) | cmd[8]);
    break;
  case 0x2A:
    LOGN("[Write10]");
    sts = onWriteCommand(((uint32_t)cmd[2] << 24) | ((uint32_t)cmd[3] << 16) | ((uint32_t)cmd[4] << 8) | cmd[5], ((uint32_t)cmd[7] << 8) | cmd[8]);
    break;
  case 0x2B:
    LOGN("[Seek10]");
    break;
  case 0x5A:
    LOGN("[ModeSense10]");
    onModeSenseCommand(cmd[1] & 0x80, cmd[2] & 0x3F, ((uint32_t)cmd[7] << 8) | cmd[8]);
    break;
  default:
    LOGN("[*Unknown]");
    sts = 2;
    m_senseKey = 5;
    break;
  }
  if(m_isBusReset) {
     goto BusFree;
  }

  LOGN("Sts");
  gpio_write(MSG, low);
  gpio_write(CD, high);
  gpio_write(IO, high);
  GPIOB->regs->CRL |= 0x00000003; // SET OUTPUT W/ PUPD on PA7-PB0 50MHz
  GPIOB->regs->CRH = 0x33333333; // SET OUTPUT W/ PUPD on PB15-PB8 50MHz
  writeHandshake(sts);
  if(m_isBusReset) {
     goto BusFree;
  }

  LOGN("MsgIn");
  gpio_write(MSG, high);
  gpio_write(CD, high);
  gpio_write(IO, high);
  GPIOB->regs->CRL |= 0x00000003; // SET OUTPUT W/ PUPD on PA7-PB0 50MHz
  GPIOB->regs->CRH = 0x33333333; // SET OUTPUT W/ PUPD on PB15-PB8 50MHz
  writeHandshake(msg);

BusFree:
  LOGN("BusFree");
  m_isBusReset = false;
  gpio_write(REQ, low);
  gpio_write(MSG, low);
  gpio_write(CD, low);
  gpio_write(IO, low);
//  gpio_write(BSY, low);
  gpio_mode(BSY, GPIO_INPUT_PU);
}
