/*
 * SCSI-HDデバイスエミュレータ
 */
#include <SPI.h>
#include "SdFat.h"

//ENABLE_EXTENDED_TRANSFER_CLASSを1に設定する
//libraries/SdFat/SdFatConfig.h
SPIClass SPI_1(1);
SdFatEX  SD(&SPI_1);

#define LOG(XX)     //Serial.print(XX)
#define LOGHEX(XX)  //Serial.print(XX, HEX)
#define LOGN(XX)    //Serial.println(XX)
#define LOGHEXN(XX) //Serial.println(XX, HEX)

#define high 0
#define low 1
#define active   1
#define inactive 0

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

#define ATN       PA8      // SCSI:ATN
#define BSY       PA9      // SCSI:BSY
#define ACK       PA10     // SCSI:ACK
#define RST       PA15     // SCSI:RST
#define MSG       PB3      // SCSI:MSG
#define SEL       PB4      // SCSI:SEL
#define CD        PB5      // SCSI:C/D
#define REQ       PB6      // SCSI:REQ
#define IO        PB7      // SCSI:I/O

#define SD_CS     PA4      // SDCARD:CS
#define LED       PC13     // LED

// GPIOレジスタポート
#define PAREG GPIOA->regs
#define PBREG GPIOB->regs

// 仮想ピン（Arduio互換は遅いのでMCU依存にして）
#define PA(BIT)       (BIT)
#define PB(BIT)       (BIT+16)
// 仮想ピンのデコード
#define GPIOREG(VPIN)    ((VPIN)>=16?PBREG:PAREG)
#define BITMASK(VPIN) (1<<((VPIN)&15))

#define vATN       PA(8)      // SCSI:ATN
#define vBSY       PA(9)      // SCSI:BSY
#define vACK       PA(10)     // SCSI:ACK
#define vRST       PA(15)     // SCSI:RST
#define vMSG       PB(3)      // SCSI:MSG
#define vSEL       PB(4)      // SCSI:SEL
#define vCD        PB(5)      // SCSI:C/D
#define vREQ       PB(6)      // SCSI:REQ
#define vIO        PB(7)      // SCSI:I/O
#define vSD_CS     PA(4)      // SDCARD:CS

// SCSI 出力ピン制御 : opendrain active LOW (direct pin drive)
#define SCSI_OUT(VPIN,ACTIVE) { GPIOREG(VPIN)->BSRR = BITMASK(VPIN)<<((ACTIVE)?16:0); }

// SCSI 入力ピン確認(inactive=0,avtive=1)
#define SCSI_IN(VPIN) ((~GPIOREG(VPIN)->IDR>>(VPIN&15))&1)

// GPIO mode
// IN , FLOAT      : 4
// IN , PU/PD      : 8
// OUT, PUSH/PULL  : 3
// OUT, OD         : 1
//#define DB_MODE_OUT 3
#define DB_MODE_OUT 1
#define DB_MODE_IN  8

// DB,DPを出力モードにする
#define SCSI_DB_OUTPUT() { PBREG->CRL=(PBREG->CRL &0xfffffff0)|DB_MODE_OUT; PBREG->CRH = 0x11111111*DB_MODE_OUT; }
// DB,DPを入力モードにする
#define SCSI_DB_INPUT()  { PBREG->CRL=(PBREG->CRL &0xfffffff0)|DB_MODE_IN ; PBREG->CRH = 0x11111111*DB_MODE_IN;  }

// BSYだけ出力をON にする
#define SCSI_BSY_ACTIVE()      { gpio_mode(BSY, GPIO_OUTPUT_OD); SCSI_OUT(vBSY,  active) }
// BSY,REQ,MSG,CD,IO 出力をON にする (ODの場合は変更不要）
#define SCSI_TARGET_ACTIVE()   { }
// BSY,REQ,MSG,CD,IO 出力をOFFにする、BSYは最後、入力に
#define SCSI_TARGET_INACTIVE() { SCSI_OUT(vREQ,inactive); SCSI_OUT(vMSG,inactive); SCSI_OUT(vCD,inactive);SCSI_OUT(vIO,inactive); SCSI_OUT(vBSY,inactive); gpio_mode(BSY, GPIO_INPUT_PU); }



#define SCSIID    0                 // SCSI-ID 

#define BLOCKSIZE 512               // 1BLOCKサイズ
uint8_t       m_senseKey = 0;       //センスキー
volatile bool m_isBusReset = false; //バスリセット

#define HDIMG_FILE "HD.HDS"         // HDイメージファイル名
File          m_file;               // ファイルオブジェクト
uint32_t      m_fileSize;           // ファイルサイズ
byte          m_buf[BLOCKSIZE];     // 汎用バッファ

int           m_msc;
bool          m_msb[256];

// パリティービット生成
#define PTY(V)   (1^((V)^((V)>>1)^((V)>>2)^((V)>>3)^((V)>>4)^((V)>>5)^((V)>>6)^((V)>>7))&1)

// データバイト to BSRRレジスタ設定値変換テーブル
// BSRR[31:24] =  DB[7:0]
// BSRR[   16] =  PTY(DB)
// BSRR[15: 8] = ~DB[7:0]
// BSRR[    0] = ~PTY(DB)

// DBPのセット、REQ=inactiveにする
#define DBP(D)    ((((((uint32_t)(D)<<8)|PTY(D))*0x00010001)^0x0000ff01)|BITMASK(vREQ))

#define DBP8(D)   DBP(D),DBP(D+1),DBP(D+2),DBP(D+3),DBP(D+4),DBP(D+5),DBP(D+6),DBP(D+7)
#define DBP32(D)  DBP8(D),DBP8(D+8),DBP8(D+16),DBP8(D+24)

// DBのセット,DPのセット,REQ=H(inactrive) を同時に行うBSRRレジスタ制御値
static const uint32_t db_bsrr[256]={
  DBP32(0x00),DBP32(0x20),DBP32(0x40),DBP32(0x60),
  DBP32(0x80),DBP32(0xA0),DBP32(0xC0),DBP32(0xE0)
};
// パリティービット取得
#define PARITY(DB) (db_bsrr[DB]&1)


/*
 * IO読み込み.
 */
inline byte readIO(void)
{
  //GPIO(SCSI BUS)初期化
  //ポート設定レジスタ（下位）
//  GPIOB->regs->CRL |= 0x000000008; // SET INPUT W/ PUPD on PAB-PB0
  //ポート設定レジスタ（上位）
  
//  GPIOB->regs->ODR = 0x0000FF00; // SET PULL-UPs on PB15-PB8
  //ポート入力データレジスタ
/*  uint32 ret = GPIOB->regs->IDR;
  byte bret =  0x00;
  bret |= (!(ret & (1<<15))) << 7;
  bret |= (!(ret & (1<<14))) << 6;
  bret |= (!(ret & (1<<13))) << 5;
  bret |= (!(ret & (1<<12))) << 4;
  bret |= (!(ret & (1<<11))) << 3;
  bret |= (!(ret & (1<<10))) << 2;
  bret |= (!(ret & (1<<9)))  << 1;
  bret |= (!(ret & (1<<8)))  << 0;
  return bret; */
  return 0xff & (~(GPIOB->regs->IDR >> 8));
}

/* 
 * IO書き込み.
 */
inline void writeIO(byte v)
{
  //GPIO(SCSI BUS)初期化
  //ポート設定レジスタ（下位）
//  GPIOB->regs->ODR != 0x0000FF00; // SET PULL-UPs on PB15-PB8
/*  LOGHEXN((v));
  LOGHEXN((GPIOB->regs->ODR & 0x00ff));
  LOGHEXN((  (GPIOB->regs->ODR & 0x00ff) | (0xff00 & (v <<8)))); */
GPIOB->regs->ODR = (GPIOB->regs->ODR & 0x00fe) | (0xff00 & ((~v) <<8) )| parity(v) ;
/*
  if(!parity(v)) {
    GPIOB->regs->BRR = 1;
  } else {
    GPIOB->regs->BSRR = 1;
  }
*/
/*
  uint32 retL =  0x00;
  uint32 retH =  0x00;
  LOGHEXN(v);
  if(!parity(v)) {
    retL |= (1<<0);
  } else {
    retH |= (1<<0);
  }
  /*
  if(v & ( 1 << 7 )) {
    retL |= (1<<15);
  } else {
    retH |= (1<<15);
  }  
  if(v & ( 1 << 6 )) {
    retL |= (1<<14);
  } else {
    retH |= (1<<14);
  }
  if(v & ( 1 << 5 )) {
    retL |= (1<<13);
  } else {
    retH |= (1<<13);
  }
  if(v & ( 1 << 4 )) {
    retL |= (1<<12);
  } else {
    retH |= (1<<12);
  }
  if(v & ( 1 << 3 )) {
    retL |= (1<<11);
  } else {
    retH |= (1<<11);
  }
  if(v & ( 1 << 2 )) {
    retL |= (1<<10);
  } else {
    retH |= (1<<10);
  }
  if(v & ( 1 << 1 )) {
    retL |= (1<<9);
  } else {
    retH |= (1<<9);
  }
  if(v & ( 1 << 0 )) {
    retL |= (1<<8);
  } else {
    retH |= (1<<8);
  } */
  /*retL |= v<<8;
  retH |= 0xffff & ((~v)<<8); 
  LOGHEXN(retL);
  LOGHEXN(retH);
  //ビットがLOWに設定される
  GPIOB->regs->BRR = retL ;
  // ビットがHIGHに設定される
  GPIOB->regs->BSRR = retH ; */ 
}

/*
 * 初期化.
 *  パリティチェック
 */
inline int parity(byte val) {
  val ^= val >> 4;
  val ^= val >> 2;
  val ^= val >> 1;

  return val & 0x00000001;
}

/*
 * 初期化.
 *  バスの初期化、PINの向きの設定を行う
 */
void setup()
{
  // PA15 / PB3 / PB4 が使えない
  // But only disable JTAG not SWD
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  //シリアル初期化
  //Serial.begin(9600);
  //while (!Serial);

  //PINの初期化
  gpio_mode(LED, GPIO_OUTPUT_OD);
  gpio_write(LED, low);

 //GPIO(SCSI BUS)初期化
  //ポート設定レジスタ（下位）
//  GPIOB->regs->CRL |= 0x000000008; // SET INPUT W/ PUPD on PAB-PB0
  //ポート設定レジスタ（上位）
  GPIOB->regs->CRH = 0x88888888; // SET INPUT W/ PUPD on PB15-PB8
//  GPIOB->regs->ODR = 0x0000FF00; // SET PULL-UPs on PB15-PB8

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

  //RSTピンの状態がHIGHからLOWに変わったときに発生
  attachInterrupt(PIN_MAP[RST].gpio_bit, onBusReset, FALLING);
  
  if(!SD.begin(SD_CS,SPI_FULL_SPEED)) {
    Serial.println("SD initialization failed!");
    onFalseInit();
  }
  //HDイメージファイル
  m_file = SD.open(HDIMG_FILE, O_RDWR);
  if(!m_file) {
    Serial.println("Error: open hdimg");
    onFalseInit();
  }
  m_fileSize = m_file.size();
  Serial.println("Found Valid HD Image File.");
  Serial.print(m_fileSize);
  Serial.println("byte");
  Serial.print(m_fileSize / 1024);
  Serial.println("KB");
  Serial.print(m_fileSize / 1024 / 1024);
  Serial.println("MB");
}

/*
 * 初期化失敗.
 */
void onFalseInit(void)
{
  while(true) {
    gpio_write(LED, high);
    delay(500); 
    gpio_write(LED, low);
    delay(500);
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

/*
 * ハンドシェイクで読み込む.
 */
inline byte readHandshake(void)
{
  
  gpio_write(REQ, high);
  while(isLow(gpio_read(ACK))) {
    if(m_isBusReset) {
      return 0;
    }
  }
  byte r = readIO();
  gpio_write(REQ, low);
  while(isHigh(gpio_read(ACK))) {
    if(m_isBusReset) {
      return 0;
    }
  }
  return r;  
}

/*
 * ハンドシェイクで書込み.
 */
inline void writeHandshake(byte d)
{
  writeIO(d);
  GPIOB->regs->BRR = 1 << 6;
  //gpio_write(REQ, high);
  while(GPIOA->regs->IDR & (1 << 10)) {
  //while(isLow(gpio_read(ACK))) {
    if(m_isBusReset) {
      return;
    }
  }
  GPIOB->regs->BSRR = 1<<6;
  //gpio_write(REQ, low);
  //while(isHigh(gpio_read(ACK))) {
  while(!(GPIOA->regs->IDR & (1<<10))) {
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
  //ポート設定レジスタ（上位）
  GPIOB->regs->CRH = 0x33333333; // SET OUTPUT W/ PUPD on PB15-PB8 50MHz

  for (int i = 0; i < len; i++) {
    if(m_isBusReset) {
      return;
    }
    writeHandshake(p[i]);
  }
}

/*

void writeDataPhaseSD(uint32_t adds, uint32_t len)
{
  LOGN("DATAIN PHASE(SD)");
  uint32_t pos = adds * BLOCKSIZE;
  m_file.seek(pos);

  SCSI_OUT(vMSG,inactive) //  gpio_write(MSG, low);
  SCSI_OUT(vCD ,inactive) //  gpio_write(CD, low);
  SCSI_OUT(vIO ,  active) //  gpio_write(IO, high);
  for(uint32_t i = 0; i < len; i++) {
      // 非同期リードにすれば速くなるんだけど...
    m_file.read(m_buf, BLOCKSIZE);


//#define REQ_ON() SCSI_OUT(vREQ,active)
#define REQ_ON() (*db_dst = BITMASK(vREQ)<<16)
#define FETCH_SRC()   (src_byte = *srcptr++)
#define FETCH_BSRR_DB() (bsrr_val = bsrr_tbl[src_byte])
#define REQ_OFF_DB_SET(BSRR_VAL) *db_dst = BSRR_VAL
#define WAIT_ACK_ACTIVE()   while(!m_isBusReset && !SCSI_IN(vACK))
#define WAIT_ACK_INACTIVE() do{ if(m_isBusReset) return; }while(SCSI_IN(vACK)) 

    SCSI_DB_OUTPUT()
    register byte *srcptr= m_buf;                 // ソースバッファ
    register byte *endptr= m_buf +  BLOCKSIZE; // 終了ポインタ
    
    /*register*/  /*byte src_byte;                       // 送信データバイト
    register const uint32_t *bsrr_tbl = db_bsrr;  // BSRRに変換するテーブル
    register uint32_t bsrr_val;                   // 出力するBSRR値(DB,DBP,REQ=ACTIVE)
    register volatile uint32_t *db_dst = &(GPIOB->regs->BSRR); // 出力ポート

    // prefetch & 1st out
    src_byte = *srcptr++;
    (bsrr_val = bsrr_tbl[src_byte];
    *db_dst = bsrr_val;
    // DB.set to REQ.F setup 100ns max (DTC-510B)
    // ここには多少のウェイトがあったほうがいいかも
    //　WAIT_ACK_INACTIVE();
    do{
      // 0
      *db_dst = BITMASK(vREQ)<<16;
      src_byte = *srcptr++;
      bsrr_val = bsrr_tbl[src_byte];
      while(!m_isBusReset && !SCSI_IN(vACK));
      // ACK.F  to REQ.R       500ns typ. (DTC-510B)
      *db_dst = bsrr_val;
      do{ if(m_isBusReset) return; }while(SCSI_IN(vACK));
      // 1
      REQ_ON();
      FETCH_SRC();
      FETCH_BSRR_DB();
      WAIT_ACK_ACTIVE();
      REQ_OFF_DB_SET(bsrr_val);
      WAIT_ACK_INACTIVE();
      // 2
      REQ_ON();
      FETCH_SRC();
      FETCH_BSRR_DB();
      WAIT_ACK_ACTIVE();
      REQ_OFF_DB_SET(bsrr_val);
      WAIT_ACK_INACTIVE();
      // 3
      REQ_ON();
      FETCH_SRC();
      FETCH_BSRR_DB();
      WAIT_ACK_ACTIVE();
      REQ_OFF_DB_SET(bsrr_val);
      WAIT_ACK_INACTIVE();
      // 4
      REQ_ON();
      FETCH_SRC();
      FETCH_BSRR_DB();
      WAIT_ACK_ACTIVE();
      REQ_OFF_DB_SET(bsrr_val);
      WAIT_ACK_INACTIVE();
      // 5
      REQ_ON();
      FETCH_SRC();
      FETCH_BSRR_DB();
      WAIT_ACK_ACTIVE();
      REQ_OFF_DB_SET(bsrr_val);
      WAIT_ACK_INACTIVE();
      // 6
      REQ_ON();
      FETCH_SRC();
      FETCH_BSRR_DB();
      WAIT_ACK_ACTIVE();
      REQ_OFF_DB_SET(bsrr_val);
      WAIT_ACK_INACTIVE();
      // 7
      REQ_ON();
      FETCH_SRC();
      FETCH_BSRR_DB();
      WAIT_ACK_ACTIVE();
      REQ_OFF_DB_SET(bsrr_val);
      WAIT_ACK_INACTIVE();
    }while(srcptr < endptr);
    SCSI_DB_INPUT()
  }
}
*/


/*
 * データインフェーズ.
 *  SDカードからの読み込みながら len ブロック送信する。
 */
void writeDataPhaseSD(uint32_t adds, uint32_t len)
{
  register volatile uint32_t *GPIOBBSRR = &(GPIOB->regs->BSRR);
  register const uint32_t *bsrr_tbl = db_bsrr;
  register volatile uint32_t *GPIOAIDR = &(GPIOA->regs->IDR);
  register volatile bool * m_isBusResetPtr = &m_isBusReset;
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
    for(int j = 0; j < BLOCKSIZE; j+=8) {
        register uint32_t wordData = * ((uint32_t *) (m_buf+j));
        register uint8_t src = (uint8_t) wordData;
        register uint32_t tmp;

        // First Byte First word
        *GPIOBBSRR = bsrr_tbl[src];
        *GPIOBBSRR = 1 << 22;
        while(*GPIOAIDR & (1 << 10) && !*m_isBusResetPtr);
        *GPIOBBSRR = 1<<6;
        src = (uint8_t) (wordData >> 8);
        tmp = bsrr_tbl[src];
        while(!(*GPIOAIDR & (1<<10)) && !*m_isBusResetPtr);

        // Second Byte First Word
        *GPIOBBSRR = tmp;
        *GPIOBBSRR = 1 << 22;
        while(*GPIOAIDR & (1 << 10) && !*m_isBusResetPtr);
        *GPIOBBSRR = 1<<6;
        src = (uint8_t) (wordData >> 16);
        tmp = bsrr_tbl[src];
        while(!(*GPIOAIDR & (1<<10)) && !*m_isBusResetPtr);

         // Third Byte First Word
        *GPIOBBSRR = tmp;
        *GPIOBBSRR = 1 << 22;
        while(*GPIOAIDR & (1 << 10) && !*m_isBusResetPtr);
        *GPIOBBSRR = 1<<6;
        src = (uint8_t) (wordData >> 24);
        tmp = bsrr_tbl[src];
        while(!(*GPIOAIDR & (1<<10)) && !*m_isBusResetPtr);

        // Fourth Byte First Word
        *GPIOBBSRR = tmp;
        *GPIOBBSRR = 1 << 22;
        while(*GPIOAIDR & (1 << 10) && !*m_isBusResetPtr);
        *GPIOBBSRR = 1<<6;
        wordData = * ((uint32_t *) (m_buf+j+4));
        src = (uint8_t) (wordData);
        tmp = bsrr_tbl[src];
        while(!(*GPIOAIDR & (1<<10)) && !*m_isBusResetPtr);

        // First Byte Second Word
        *GPIOBBSRR = tmp;
        *GPIOBBSRR = 1 << 22;
        while(*GPIOAIDR & (1 << 10) && !*m_isBusResetPtr);
        *GPIOBBSRR = 1<<6;
        src = (uint8_t) (wordData >> 8);
        tmp = bsrr_tbl[src];
        while(!(*GPIOAIDR & (1<<10)) && !*m_isBusResetPtr);

        // Second Byte Second Word
        *GPIOBBSRR = tmp;
        *GPIOBBSRR = 1 << 22;
        while(*GPIOAIDR & (1 << 10) && !*m_isBusResetPtr);
        *GPIOBBSRR = 1<<6;
        src = (uint8_t) (wordData >> 16);
        tmp = bsrr_tbl[src];
        while(!(*GPIOAIDR & (1<<10)) && !*m_isBusResetPtr);

        // Third Byte Second Word
        *GPIOBBSRR = tmp;
        *GPIOBBSRR = 1 << 22;
        while(*GPIOAIDR & (1 << 10) && !*m_isBusResetPtr);
        *GPIOBBSRR = 1<<6;
        src = (uint8_t) (wordData >> 24);
        tmp = bsrr_tbl[src];
        while(!(*GPIOAIDR & (1<<10)) && !*m_isBusResetPtr);

        // Fourth Byte Second Word
        *GPIOBBSRR = tmp;
        *GPIOBBSRR = 1 << 22;
        while(*GPIOAIDR & (1 << 10) && !*m_isBusResetPtr);
        *GPIOBBSRR = 1<<6;
        while(!(*GPIOAIDR & (1<<10)) && !*m_isBusResetPtr);       
        if(*m_isBusResetPtr) return;
    }
  }
}


void readDataPhaseSD(uint32_t adds, uint32_t len)
{
  LOGN("DATAOUT PHASE(SD)");
  uint32_t pos = adds * BLOCKSIZE;
  m_file.seek(pos);
  SCSI_OUT(vMSG,inactive) //  gpio_write(MSG, low);
  SCSI_OUT(vCD ,inactive) //  gpio_write(CD, low);
  SCSI_OUT(vIO ,inactive) //  gpio_write(IO, low);
  GPIOB->regs->CRH = 0x88888888; // SET INPUT W/ PUPD on PB15-PB8
  for(uint32_t i = 0; i < len; i++) {
  register byte *dstptr= m_buf;
  register byte *endptr= m_buf + BLOCKSIZE;

    for(dstptr=m_buf;dstptr<endptr;dstptr+=8) {
      dstptr[0] = readHandshake();
      dstptr[1] = readHandshake();
      dstptr[2] = readHandshake();
      dstptr[3] = readHandshake();
      dstptr[4] = readHandshake();
      dstptr[5] = readHandshake();
      dstptr[6] = readHandshake();
      dstptr[7] = readHandshake();
      if(m_isBusReset) {
        return;
      }
    }
    m_file.write(m_buf, BLOCKSIZE);
  }
  m_file.flush();
}



/*
 * データアウトフェーズ.
 *  len ブロック読み込みながら SDカードへ書き込む。
 */
/*void readDataPhaseSD(uint32_t adds, uint32_t len)
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
}*/

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
  //ポート設定レジスタ（上位）
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
  GPIOB->regs->CRH = 0x88888888; // SET INPUT W/ PUPD on PB15-PB8
  int len;
  byte cmd[12];
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
  //ポート設定レジスタ（上位）
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
  //ポート設定レジスタ（上位）
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
