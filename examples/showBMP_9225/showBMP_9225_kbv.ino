#if USE_TFT_22_ILI9225 == 0 
// Include application, user and local libraries
#include "SPI.h"
#include "ILI9225_kbv.h"
#include "SD.h"

#define TFT_RST 8
#define TFT_RS  9
#define TFT_CS  10  // SS
#define TFT_SDI 11  // MOSI
#define TFT_CLK 13  // SCK
#define SD_CS   7

// Use hardware SPI (faster - on Uno: 13-SCK, 12-MISO, 11-MOSI)
ILI9225_kbv tft(TFT_CS, TFT_RS, TFT_RST);       //hardware SPI default tft(10, 9, 8)
//ILI9225_kbv tft(10, 9, 11, 13, 8); //software SPI.  no default.
//ILI9225_kbv tft(A5, A3, A2, A1, A4); //software SPI.  no default.

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
    uint8_t first = 1;

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
//                        tft.pushColor(tft.Color565(r, g, b));
                        uint16_t color = tft.color565(r, g, b);
                        tft.pushColors(&color, 1, first);
                        first = 0;
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

