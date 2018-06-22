#if USE_TFT_22_ILI9225 
// Include application, user and local libraries
#include "SPI.h"
#include "TFT_22_ILI9225.h"
#include "SD.h"

#if defined (ARDUINO_ARCH_STM32F1)
#define TFT_RST PA1
#define TFT_RS  PA2
#define TFT_CS  PA0 // SS
#define TFT_SDI PA7 // MOSI
#define TFT_CLK PA5 // SCK
#define TFT_LED 0 // 0 if wired to +5V directly
#else
#define TFT_RST 8
#define TFT_RS  9
#define TFT_CS  10  // SS
#define TFT_SDI 11  // MOSI
#define TFT_CLK 13  // SCK
#define TFT_LED 3   // 0 if wired to +5V directly
#define SD_CS   7
#endif

#define TFT_BRIGHTNESS 200 // Initial brightness of TFT backlight (optional)

/* the TFT_22_ILI9225 library only has private methods for
 * _setWindow() and _writeData()
 * 
 * edit TFT_22_ILI9225.h to make these protected instead of private
 * 
 * then we can create a GLUE class that simplifies porting bmpDraw()
 */

class ILI9225_super : public TFT_22_ILI9225 {
    public:
        ILI9225_super(int8_t RST, int8_t RS, int8_t CS, int8_t LED, uint8_t brightness) :
            TFT_22_ILI9225(RST, RS, CS, LED, brightness) {}

        int width() {
            return maxX();
        }
        int height() {
            return maxY();
        }
        void setAddrWindow(int x, int y, int x1, int y1) {
            _setWindow(x, y, x1, y1);
        }
        uint16_t Color565(uint8_t r, uint8_t g, uint8_t b) {
            return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
        }
        void pushColor(uint16_t color) {
            _writeData(color >> 8, color);
        }
};

// Use hardware SPI (faster - on Uno: 13-SCK, 12-MISO, 11-MOSI)
//TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_LED, TFT_BRIGHTNESS);
ILI9225_super tft = ILI9225_super(TFT_RST, TFT_RS, TFT_CS, TFT_LED, TFT_BRIGHTNESS);

File root;
void setup(void)
{
    Serial.begin(9600);
    tft.begin();
    Serial.print("Initializing SD card...");
    if (!SD.begin(SD_CS)) {
        Serial.println("failed!");
        return;
    }
    Serial.println("OK!");
    root = SD.open("");
}

void loop()
{
    char namebuf[32], *nm = namebuf;
    File f = root.openNextFile();
    if (f != NULL) {
        strcpy(nm, (char *)f.name());
        f.close();
        strlwr(nm);
        if (strstr(nm, ".bmp")) {
            bmpDraw(namebuf, 0, 0);
        }
    }
    else root.rewindDirectory();
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File f) {
    uint16_t result;
    ((uint8_t *)&result)[0] = f.read(); // LSB
    ((uint8_t *)&result)[1] = f.read(); // MSB
    return result;
}

uint32_t read32(File f) {
    uint32_t result;
    ((uint8_t *)&result)[0] = f.read(); // LSB
    ((uint8_t *)&result)[1] = f.read();
    ((uint8_t *)&result)[2] = f.read();
    ((uint8_t *)&result)[3] = f.read(); // MSB
    return result;
}

// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

#define BUFFPIXEL 20

void bmpDraw(char *filename, uint8_t x, uint8_t y) {

    File     bmpFile;
    int      bmpWidth, bmpHeight;   // W+H in pixels
    uint8_t  bmpDepth;              // Bit depth (currently must be 24)
    uint32_t bmpImageoffset;        // Start of image data in file
    uint32_t rowSize;               // Not always = bmpWidth; may have padding
    uint8_t  sdbuffer[3 * BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
    uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
    boolean  goodBmp = false;       // Set to true on valid header parse
    boolean  flip    = true;        // BMP is stored bottom-to-top
    int      w, h, row, col;
    uint8_t  r, g, b;
    uint32_t pos = 0, startTime = millis();

    if ((x >= tft.width()) || (y >= tft.height())) return;

    Serial.println();
    Serial.print("Loading image '");
    Serial.print(filename);
    Serial.println('\'');

    // Open requested file on SD card
    if ((bmpFile = SD.open(filename)) == NULL) {
        Serial.print("File not found");
        return;
    }

    // Parse BMP header
    if (read16(bmpFile) == 0x4D42) { // BMP signature
        Serial.print("File size: "); Serial.println(read32(bmpFile));
        (void)read32(bmpFile); // Read & ignore creator bytes
        bmpImageoffset = read32(bmpFile); // Start of image data
        Serial.print("Image Offset: "); Serial.println(bmpImageoffset, DEC);
        // Read DIB header
        Serial.print("Header size: "); Serial.println(read32(bmpFile));
        bmpWidth  = read32(bmpFile);
        bmpHeight = read32(bmpFile);
        if (read16(bmpFile) == 1) { // # planes -- must be '1'
            bmpDepth = read16(bmpFile); // bits per pixel
            Serial.print("Bit Depth: "); Serial.println(bmpDepth);
            if ((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

                goodBmp = true; // Supported BMP format -- proceed!
                Serial.print("Image size: ");
                Serial.print(bmpWidth);
                Serial.print('x');
                Serial.println(bmpHeight);

                // BMP rows are padded (if needed) to 4-byte boundary
                rowSize = (bmpWidth * 3 + 3) & ~3;

                // If bmpHeight is negative, image is in top-down order.
                // This is not canon but has been observed in the wild.
                if (bmpHeight < 0) {
                    bmpHeight = -bmpHeight;
                    flip      = false;
                }

                // Crop area to be loaded
                w = bmpWidth;
                h = bmpHeight;
                if ((x + w - 1) >= tft.width())  w = tft.width()  - x;
                if ((y + h - 1) >= tft.height()) h = tft.height() - y;

                // Set TFT address window to clipped image bounds
                tft.setAddrWindow(x, y, x + w - 1, y + h - 1);

                for (row = 0; row < h; row++) { // For each scanline...

                    // Seek to start of scan line.  It might seem labor-
                    // intensive to be doing this on every line, but this
                    // method covers a lot of gritty details like cropping
                    // and scanline padding.  Also, the seek only takes
                    // place if the file position actually needs to change
                    // (avoids a lot of cluster math in SD library).
                    if (flip) // Bitmap is stored bottom-to-top order (normal BMP)
                        pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
                    else     // Bitmap is stored top-to-bottom
                        pos = bmpImageoffset + row * rowSize;
                    if (bmpFile.position() != pos) { // Need seek?
                        bmpFile.seek(pos);
                        buffidx = sizeof(sdbuffer); // Force buffer reload
                    }

                    for (col = 0; col < w; col++) { // For each pixel...
                        // Time to read more pixel data?
                        if (buffidx >= sizeof(sdbuffer)) { // Indeed
                            bmpFile.read(sdbuffer, sizeof(sdbuffer));
                            buffidx = 0; // Set index to beginning
                        }

                        // Convert pixel from BMP to TFT format, push to display
                        b = sdbuffer[buffidx++];
                        g = sdbuffer[buffidx++];
                        r = sdbuffer[buffidx++];
                        tft.pushColor(tft.Color565(r, g, b));
                    } // end pixel
                } // end scanline
                Serial.print("Loaded in ");
                Serial.print(millis() - startTime);
                Serial.println(" ms");
            } // end goodBmp
        }
    }

    bmpFile.close();
    if (!goodBmp) Serial.println("BMP format not recognized.");
}
#endif

