#include <Wire.h>
#include "wiring_private.h"

#include "ydlidar-gs2.cpp"
#include "oled.h"

// i2c wire for the OLED
TwoWire oledWire(&sercom1, 5, 25);

// Serial4 + SERCOM is used for the lidar. No reason why the default
// serial couldn't be used. The following will need to be removed for
// non-SAMD51 boards
Uart Serial4 (&sercom4, A3, A2, SERCOM_RX_PAD_1, UART_TX_PAD_0);
void SERCOM4_0_Handler()
{
  Serial4.IrqHandler();
}
void SERCOM4_1_Handler()
{
  Serial4.IrqHandler();
}
void SERCOM4_2_Handler()
{
  Serial4.IrqHandler();
}
void SERCOM4_3_Handler()
{
  Serial4.IrqHandler();
}

OLED display(OLED::W_128,OLED::H_64);

YdlidarGs2 lidar;


void setup()
{
  // Begin serial debug
  Serial.begin(19200);

  // wait 2 seconds for debug serial, otherwise just go reguardless
  uint32_t start = millis();
  while ((!Serial) && ((millis() - start) < 2000)) {
    delay(10);
  }
  Serial.println("YDLIDAR GS2 demo");

  // setup display
  oledWire.begin();
  oledWire.setClock(1600000L);

  // Assign pins 5 & 25 to SERCOM functionality
  // for i2c display
  pinPeripheral(5, PIO_SERCOM);
  pinPeripheral(25, PIO_SERCOM);

  // setup serial for lidar
  while (!Serial4);
  Serial4.begin(921600, SERIAL_8N1);
  Serial4.setTimeout(5);

  // // Assign pins 0 & 1 SERCOM functionality for serial to lidar
  pinPeripheral(A2, PIO_SERCOM_ALT);
  pinPeripheral(A3, PIO_SERCOM_ALT);

  // Begin display
  display.begin(oledWire);
  display.set_contrast(40);

  // draw something
  display.clear();
  display.draw_string(0, 0, "Starting up...");
  display.display();

  delay(100);

  // begin lidar scanning
  lidar.setup(Serial4);
  lidar.startScan();
}


void drawLidar(uint16_t distances[]) {
  display.clear();

  for (uint8_t i = 0; i < 160; i++) {
    // 128 as the screen is only 128 wide
    uint8_t scaledX = (128 * ((float)i)/160.0);
    uint8_t scaledDistance = (64 * ((float)distances[i])/300.0);
    // values returned from lidar may exceed 300, so threshold it here
    if (scaledDistance >= 64) {
      scaledDistance = 63;
    } else if (scaledDistance < 0) {
      scaledDistance = 0;
    }
    display.draw_pixel(scaledX, scaledDistance - 1);
    display.draw_pixel(scaledX, scaledDistance);
    display.draw_pixel(scaledX, scaledDistance + 1);
  }

  display.display();
}


void loop()
{
  // fetch the latest scan data
  lidar.loop();
  // draw distances to the OLED
  drawLidar(lidar.distances);
}
