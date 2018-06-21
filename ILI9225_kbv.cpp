#include <SPI.h>
#include "ILI9225_kbv.h"
//#include "serial_kbv.h"

#if 1
// PASTE is awkward.   Not too difficult to write macros longhand
#define PIN_OUTPUT(x) pinMode(x, OUTPUT)
#define CD_COMMAND *_dcport &= ~_dcpinmask
#define CD_DATA    *_dcport |= _dcpinmask
#define CD_OUTPUT  PIN_OUTPUT(_dc)
#define CS_ACTIVE  *_csport &= ~_cspinmask
#define CS_IDLE    *_csport |= _cspinmask
#define CS_OUTPUT  PIN_OUTPUT(_cs)
#define RESET_ACTIVE  *_rstport &= ~_rstpinmask
#define RESET_IDLE    *_rstport |= _rstpinmask
#define RESET_OUTPUT  PIN_OUTPUT(_rst)
#define SCK_LO     *_sclkport &= ~_sclkpinmask
#define SCK_HI     *_sclkport |= _sclkpinmask
#define SCK_OUT    PIN_OUTPUT(_sclk)
#define MOSI_LO    *_mosiport &= ~_mosipinmask
#define MOSI_HI    *_mosiport |= _mosipinmask
#define MOSI_OUT   PIN_OUTPUT(_mosi)
#else
//#define PASTE(x, y) (x ## y)
//#define PIN_LOW(x) *(PASTE(x, port)) &= ~(PASTE(x, pinmask))
//#define PIN_HIGH(x) *(PASTE(x, port)) |= (PASTE(x, pinmask))
#define PIN_LOW(x) digitalWrite(x, LOW)
#define PIN_HIGH(x) digitalWrite(x, HIGH)
#define PIN_OUTPUT(x) pinMode(x, OUTPUT)
#define CD_COMMAND PIN_LOW(_dc)
#define CD_DATA    PIN_HIGH(_dc)
#define CD_OUTPUT  PIN_OUTPUT(_dc)
#define CS_ACTIVE  PIN_LOW(_cs)
#define CS_IDLE    PIN_HIGH(_cs)
#define CS_OUTPUT  PIN_OUTPUT(_cs)
#define RESET_ACTIVE  PIN_LOW(_rst)
#define RESET_IDLE    PIN_HIGH(_rst)
#define RESET_OUTPUT  PIN_OUTPUT(_rst)
#define SCK_LO     PIN_LOW(_sclk)
#define SCK_HI     PIN_HIGH(_sclk)
#define SCK_OUT    PIN_OUTPUT(_sclk)
#define MOSI_LO    PIN_LOW(_mosi)
#define MOSI_HI    PIN_HIGH(_mosi)
#define MOSI_OUT   PIN_OUTPUT(_mosi)
#endif

#define WriteCmd(x)  { CD_COMMAND; xchg8_1(0); xchg8_1(x); CD_DATA; }
#define WriteData(x)  { uint8_t hi = (x) >> 8, lo = (x); xchg8_1(hi); xchg8_1(lo); }

static uint8_t _MC, _MP, _SC, _EC, _SP, _EP, _hwspi;
static uint8_t _cs, _dc, _mosi, _sclk, _rst;
#ifdef __AVR__
typedef uint8_t RwReg;
#else
typedef uint32_t RwReg;
#endif
static volatile RwReg *_csport, *_dcport, *_mosiport, *_sclkport, *_rstport;
static RwReg   _cspinmask, _dcpinmask, _mosipinmask, _sclkpinmask, _rstpinmask;

static void xchg8_1(uint8_t c) 
{
    if (_hwspi) {
		SPI.transfer(c);
		return;
	}
#if 1
	for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) {
	      MOSI_HI; 
      } else {
	      MOSI_LO; 
      }
	  SCK_HI; 
	  SCK_LO; 
    }
#endif
}

static inline void write16_N(uint16_t color, int16_t n)
{
    uint8_t hi = color >> 8, lo = color;
    while (n-- > 0) {
//        CS_ACTIVE;
		xchg8_1(hi);
        xchg8_1(lo);
//		CS_IDLE;
	}
}

ILI9225_kbv::ILI9225_kbv(int8_t cs, int8_t dc, int8_t rst):Adafruit_GFX(176, 220)
{
    _cs = cs;
	_dc = dc;
	_rst = rst;
	_hwspi = 1;
    _csport    = portOutputRegister(digitalPinToPort(_cs));
    _cspinmask = digitalPinToBitMask(_cs);
    _dcport    = portOutputRegister(digitalPinToPort(_dc));
    _dcpinmask = digitalPinToBitMask(_dc);
    _rstport    = portOutputRegister(digitalPinToPort(_rst));
    _rstpinmask = digitalPinToBitMask(_rst);
}

ILI9225_kbv::ILI9225_kbv(int8_t cs, int8_t dc, int8_t mosi, int8_t sclk, int8_t rst):Adafruit_GFX(176, 220)
{
    _cs = cs;
	_dc = dc;
	_mosi = mosi;
	_sclk = sclk; 
	_rst = rst;
	_hwspi = 0;
    _csport    = portOutputRegister(digitalPinToPort(_cs));
    _cspinmask = digitalPinToBitMask(_cs);
    _dcport    = portOutputRegister(digitalPinToPort(_dc));
    _dcpinmask = digitalPinToBitMask(_dc);
    _rstport    = portOutputRegister(digitalPinToPort(_rst));
    _rstpinmask = digitalPinToBitMask(_rst);
        _sclkport     = portOutputRegister(digitalPinToPort(_sclk));
        _sclkpinmask  = digitalPinToBitMask(_sclk);
        _mosiport    = portOutputRegister(digitalPinToPort(_mosi));
        _mosipinmask = digitalPinToBitMask(_mosi);
}

void ILI9225_kbv::reset(void)
{
    { CS_IDLE; RESET_IDLE; CS_OUTPUT; CD_OUTPUT; RESET_OUTPUT; }
    if (_hwspi) {
		SPI.begin();  //needed for AVR
		SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
    } else {
		MOSI_OUT; SCK_OUT;
	}
	CS_IDLE;
    RESET_IDLE;
	delay(50);
	RESET_ACTIVE;
	delay(100);
	RESET_IDLE;
	delay(100);
}

void ILI9225_kbv::WriteCmdData(uint16_t cmd, uint16_t dat)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    WriteData(dat);
    CS_IDLE;
}

#define ILI9225_DRIVER_OUTPUT_CTRL      (0x01u)  // Driver Output Control
#define ILI9225_LCD_AC_DRIVING_CTRL     (0x02u)  // LCD AC Driving Control
#define ILI9225_ENTRY_MODE            	(0x03u)  // Entry Mode
#define ILI9225_DISP_CTRL1          	(0x07u)  // Display Control 1
#define ILI9225_BLANK_PERIOD_CTRL1      (0x08u)  // Blank Period Control
#define ILI9225_FRAME_CYCLE_CTRL        (0x0Bu)  // Frame Cycle Control
#define ILI9225_INTERFACE_CTRL          (0x0Cu)  // Interface Control
#define ILI9225_OSC_CTRL             	(0x0Fu)  // Osc Control
#define ILI9225_POWER_CTRL1            	(0x10u)  // Power Control 1
#define ILI9225_POWER_CTRL2           	(0x11u)  // Power Control 2
#define ILI9225_POWER_CTRL3            	(0x12u)  // Power Control 3
#define ILI9225_POWER_CTRL4            	(0x13u)  // Power Control 4
#define ILI9225_POWER_CTRL5            	(0x14u)  // Power Control 5
#define ILI9225_VCI_RECYCLING          	(0x15u)  // VCI Recycling
#define ILI9225_RAM_ADDR_SET1           (0x20u)  // Horizontal GRAM Address Set
#define ILI9225_RAM_ADDR_SET2           (0x21u)  // Vertical GRAM Address Set
#define ILI9225_GRAM_DATA_REG           (0x22u)  // GRAM Data Register
#define ILI9225_GATE_SCAN_CTRL          (0x30u)  // Gate Scan Control Register
#define ILI9225_VERTICAL_SCROLL_CTRL1   (0x31u)  // Vertical Scroll Control 1 Register
#define ILI9225_VERTICAL_SCROLL_CTRL2   (0x32u)  // Vertical Scroll Control 2 Register
#define ILI9225_VERTICAL_SCROLL_CTRL3   (0x33u)  // Vertical Scroll Control 3 Register
#define ILI9225_PARTIAL_DRIVING_POS1    (0x34u)  // Partial Driving Position 1 Register
#define ILI9225_PARTIAL_DRIVING_POS2    (0x35u)  // Partial Driving Position 2 Register
#define ILI9225_HORIZONTAL_WINDOW_ADDR1 (0x36u)  // Horizontal Address END Position   HEA
#define ILI9225_HORIZONTAL_WINDOW_ADDR2	(0x37u)  // Horizontal Address START Position HSA
#define ILI9225_VERTICAL_WINDOW_ADDR1   (0x38u)  // Vertical Address END Position     VEA
#define ILI9225_VERTICAL_WINDOW_ADDR2   (0x39u)  // Vertical Address START Position   VSA
#define ILI9225_GAMMA_CTRL1            	(0x50u)  // Gamma Control 1
#define ILI9225_GAMMA_CTRL2             (0x51u)  // Gamma Control 2
#define ILI9225_GAMMA_CTRL3            	(0x52u)  // Gamma Control 3
#define ILI9225_GAMMA_CTRL4            	(0x53u)  // Gamma Control 4
#define ILI9225_GAMMA_CTRL5            	(0x54u)  // Gamma Control 5
#define ILI9225_GAMMA_CTRL6            	(0x55u)  // Gamma Control 6
#define ILI9225_GAMMA_CTRL7            	(0x56u)  // Gamma Control 7
#define ILI9225_GAMMA_CTRL8            	(0x57u)  // Gamma Control 8
#define ILI9225_GAMMA_CTRL9             (0x58u)  // Gamma Control 9
#define ILI9225_GAMMA_CTRL10            (0x59u)  // Gamma Control 10

#define ILI9225C_INVOFF  0x20
#define ILI9225C_INVON   0x21

uint16_t ILI9225_kbv::readReg(uint16_t reg)
{
    return 0;
}

uint32_t ILI9225_kbv::readReg32(uint16_t reg)
{
	return 0;
}

int16_t ILI9225_kbv::readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h)
{
    return 0;
}

void ILI9225_kbv::setRotation(uint8_t r)
{
    uint16_t mac;
    Adafruit_GFX::setRotation(r & 3);
    switch (rotation) {
    case 0:
        mac = 0x0800;
        break;
    case 1:
        mac = 0x6800;
        break;
    case 2:
        mac = 0xD800;
        break;
    case 3:
        mac = 0xB800;
        break;
    }
    if (mac & 0x2000) {
	    _MC = ILI9225_RAM_ADDR_SET2, _MP = ILI9225_RAM_ADDR_SET1;
		_SC = ILI9225_VERTICAL_WINDOW_ADDR2, _EC = ILI9225_VERTICAL_WINDOW_ADDR1; 
		_SP = ILI9225_HORIZONTAL_WINDOW_ADDR2, _EP = ILI9225_HORIZONTAL_WINDOW_ADDR1;
         WriteCmdData(ILI9225_ENTRY_MODE, 0x1038);
	} else {
	    _MC = ILI9225_RAM_ADDR_SET1, _MP = ILI9225_RAM_ADDR_SET2;
		_SC = ILI9225_HORIZONTAL_WINDOW_ADDR2, _EC = ILI9225_HORIZONTAL_WINDOW_ADDR1;
		_SP = ILI9225_VERTICAL_WINDOW_ADDR2, _EP = ILI9225_VERTICAL_WINDOW_ADDR1; 
         WriteCmdData(ILI9225_ENTRY_MODE, 0x1030);
	}
    mac ^= 0x4000;   //flip SS
	WriteCmdData(ILI9225_DRIVER_OUTPUT_CTRL, ((mac & 0xC000) >> 6) | 0x001C);
}

void ILI9225_kbv::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    // ILI934X just plots at edge if you try to write outside of the box:
    if (x < 0 || y < 0 || x >= width() || y >= height())
        return;
    WriteCmdData(_MC, x);
    WriteCmdData(_MP, y);
    WriteCmdData(ILI9225_GRAM_DATA_REG, color);
}

void ILI9225_kbv::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
    WriteCmdData(_SC, x);
    WriteCmdData(_EC, x1);
    WriteCmdData(_SP, y);
    WriteCmdData(_EP, y1);
    WriteCmdData(_MC, x);
    WriteCmdData(_MP, y);
}

void ILI9225_kbv::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    int16_t end;
    if (w < 0) {
        w = -w;
        x -= w;
    }                           //+ve w
    end = x + w;
    if (x < 0)
        x = 0;
    if (end > width())
        end = width();
    w = end - x;
    if (h < 0) {
        h = -h;
        y -= h;
    }                           //+ve h
    end = y + h;
    if (y < 0)
        y = 0;
    if (end > height())
        end = height();
    h = end - y;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    CS_ACTIVE;
    WriteCmd(ILI9225_GRAM_DATA_REG);
    CD_DATA;
	if (h > w) { end = h; h = w; w = end; } 
    while (h-- > 0) {
        write16_N(color, w);
    }
    CS_IDLE;
    setAddrWindow(0, 0, width() - 1, height() - 1);
}

void ILI9225_kbv::pushColors(uint16_t * block, int16_t n, bool first)
{
    uint16_t color;
    CS_ACTIVE;
    if (first) {
        WriteCmd(ILI9225_GRAM_DATA_REG);
    }
    CD_DATA;
    while (n-- > 0) {
        color = *block++;
        WriteData(color);
    }
    CS_IDLE;
}

void ILI9225_kbv::pushColors(uint8_t * block, int16_t n, bool first)
{
    uint16_t color;
    uint8_t h, l;
    CS_ACTIVE;
    if (first) {
        WriteCmd(ILI9225_GRAM_DATA_REG);
    }
    CD_DATA;
    while (n-- > 0) {
        h = (*block++);
        l = (*block++);
        color = (h << 8) | l;
        WriteData(color);
    }
    CS_IDLE;
}

void ILI9225_kbv::pushColors(const uint8_t * block, int16_t n, bool first, bool bigend)
{
    uint16_t color;
	uint8_t h, l;
	CS_ACTIVE;
    if (first) {
        WriteCmd(ILI9225_GRAM_DATA_REG);
    }
    CD_DATA;
    while (n-- > 0) {
        l = pgm_read_byte(block++);
        h = pgm_read_byte(block++);
        color = (bigend) ? (l << 8 ) | h : (h << 8) | l;
		WriteData(color);
    }
    CS_IDLE;
}

void ILI9225_kbv::invertDisplay(boolean i)
{
    WriteCmdData(ILI9225_DISP_CTRL1, i ? 0x1013 : 0x1017);
}
    
void ILI9225_kbv::vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
    int16_t bfa = HEIGHT - top - scrollines;  // bottom fixed area
    int16_t vsp;
    int16_t sea = top;
    vsp = top + offset; // vertical start position
    if (offset < 0)
        vsp += scrollines;          //keep in unsigned range
    sea = top + scrollines - 1;
    WriteCmdData(ILI9225_VERTICAL_SCROLL_CTRL1, sea);       //SEA
    WriteCmdData(ILI9225_VERTICAL_SCROLL_CTRL2, top);       //SSA
    WriteCmdData(ILI9225_VERTICAL_SCROLL_CTRL3, vsp - top);       //SST
}

#define TFTLCD_DELAY 0xFFFF
static const uint16_t ILI9225_regValues[] PROGMEM = {
	/* Start Initial Sequence */
	/* Set SS bit and direction output from S528 to S1 */
	ILI9225_POWER_CTRL1, 0x0000, // Set SAP,DSTB,STB
	ILI9225_POWER_CTRL2, 0x0000, // Set APON,PON,AON,VCI1EN,VC
	ILI9225_POWER_CTRL3, 0x0000, // Set BT,DC1,DC2,DC3
	ILI9225_POWER_CTRL4, 0x0000, // Set GVDD
	ILI9225_POWER_CTRL5, 0x0000, // Set VCOMH/VCOML voltage
	TFTLCD_DELAY, 40, 

	// Power-on sequence
	ILI9225_POWER_CTRL2, 0x0018, // Set APON,PON,AON,VCI1EN,VC
	ILI9225_POWER_CTRL3, 0x6121, // Set BT,DC1,DC2,DC3
	ILI9225_POWER_CTRL4, 0x006F, // Set GVDD   /*007F 0088 */
	ILI9225_POWER_CTRL5, 0x495F, // Set VCOMH/VCOML voltage
	ILI9225_POWER_CTRL1, 0x0800, // Set SAP,DSTB,STB
	TFTLCD_DELAY, 10,
	ILI9225_POWER_CTRL2, 0x103B, // Set APON,PON,AON,VCI1EN,VC
	TFTLCD_DELAY, 50,

	ILI9225_DRIVER_OUTPUT_CTRL, 0x011C, // set the display line number and display direction
	ILI9225_LCD_AC_DRIVING_CTRL, 0x0100, // set 1 line inversion
	ILI9225_ENTRY_MODE, 0x1030, // set GRAM write direction and BGR=1.
	ILI9225_DISP_CTRL1, 0x0000, // Display off
	ILI9225_BLANK_PERIOD_CTRL1, 0x0808, // set the back porch and front porch
	ILI9225_FRAME_CYCLE_CTRL, 0x1100, // set the clocks number per line
	ILI9225_INTERFACE_CTRL, 0x0000, // CPU interface
	ILI9225_OSC_CTRL, 0x0D01, // Set Osc  /*0e01*/
	ILI9225_VCI_RECYCLING, 0x0020, // Set VCI recycling
	ILI9225_RAM_ADDR_SET1, 0x0000, // RAM Address
	ILI9225_RAM_ADDR_SET2, 0x0000, // RAM Address

	/* Set GRAM area */
	ILI9225_GATE_SCAN_CTRL, 0x0000, 
	ILI9225_VERTICAL_SCROLL_CTRL1, 0x00DB, 
	ILI9225_VERTICAL_SCROLL_CTRL2, 0x0000, 
	ILI9225_VERTICAL_SCROLL_CTRL3, 0x0000, 
	ILI9225_PARTIAL_DRIVING_POS1, 0x00DB, 
	ILI9225_PARTIAL_DRIVING_POS2, 0x0000, 
	ILI9225_HORIZONTAL_WINDOW_ADDR1, 0x00AF, 
	ILI9225_HORIZONTAL_WINDOW_ADDR2, 0x0000, 
	ILI9225_VERTICAL_WINDOW_ADDR1, 0x00DB, 
	ILI9225_VERTICAL_WINDOW_ADDR2, 0x0000, 

	/* Set GAMMA curve */
	ILI9225_GAMMA_CTRL1, 0x0000, 
	ILI9225_GAMMA_CTRL2, 0x0808, 
	ILI9225_GAMMA_CTRL3, 0x080A, 
	ILI9225_GAMMA_CTRL4, 0x000A, 
	ILI9225_GAMMA_CTRL5, 0x0A08, 
	ILI9225_GAMMA_CTRL6, 0x0808, 
	ILI9225_GAMMA_CTRL7, 0x0000, 
	ILI9225_GAMMA_CTRL8, 0x0A00, 
	ILI9225_GAMMA_CTRL9, 0x0710, 
	ILI9225_GAMMA_CTRL10, 0x0710, 

	ILI9225_DISP_CTRL1, 0x0012, 
	TFTLCD_DELAY, 50, 
	ILI9225_DISP_CTRL1, 0x1017,
};

static void init_table16(const void *table, int16_t size)
{
    uint16_t *p = (uint16_t *) table;
    while (size > 0) {
        uint16_t cmd = pgm_read_word(p++);
        uint16_t d = pgm_read_word(p++);
        if (cmd == TFTLCD_DELAY)
            delay(d);
        else {
            CS_ACTIVE;
            WriteCmd(cmd);
            WriteData(d);
            CS_IDLE;
        }
        size -= 2 * sizeof(int16_t);
    }
}

void ILI9225_kbv::begin(uint16_t ID)
{
    _lcd_ID = ID;
    reset();
    init_table16(ILI9225_regValues, sizeof(ILI9225_regValues));
    setRotation(0);             //PORTRAIT
}

