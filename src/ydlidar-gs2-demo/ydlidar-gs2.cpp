#include "USB/USBAPI.h"
#include "variant.h"
#include <stdint.h>
#include <Arduino.h>

// size of the buffer used by the LIDAR scan packet
#define LIDAR_BUFFER_SIZE 331
// number of points (distanced) returned in each scan packet
#define LIDAR_POINTS = 160

// As defined by YDLIDAR docs
#define Angle_Px   1.22
#define Angle_Py   5.315
#define Angle_PAngle   22.5


class YdlidarGs2 {

  private:
    Uart* lserial;

    uint8_t _isScanning;
    uint8_t _rxBuf[LIDAR_BUFFER_SIZE+9];
    uint16_t _rxBufPos;

    uint32_t _lastReceive;


  public:
    // float compK0, compK1, compB0, compB1, bias;
    double d_compensateK0, d_compensateK1, d_compensateB0, d_compensateB1;
    uint16_t u_compensateK0, u_compensateK1, u_compensateB0, u_compensateB1;
    double bias;

    uint16_t distances[160];
    float thetas_deg[160];
    uint8_t sorted_indexes[160];

  float leftCamThetaCalc(uint8_t measurement_point) {
    float tempTheta;
    measurement_point = 80 - measurement_point;
    if (d_compensateB0 > 1.0) {
      tempTheta = d_compensateK0 * measurement_point - d_compensateB0;
    } else {
      tempTheta = atan(d_compensateK0 * measurement_point - d_compensateB0) * 180.0 / M_PI;
    }
    return tempTheta;
  }

  float rightCamThetaCalc(uint8_t measurement_point) {
    float tempTheta;
    measurement_point = 160 - measurement_point;
    if (d_compensateB1 > 1) {
      tempTheta = d_compensateK1 * measurement_point - d_compensateB1;
    } else {
      tempTheta = atan(d_compensateK1 * measurement_point - d_compensateB1) * 180 / M_PI;
    }
    return tempTheta;
  }

  void getDeviceParams() {
    char getParamsData[] = {0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x61, 0x00, 0x00, 0x61};
    for (int i = 0; i < 9; i++) {
      lserial->write(getParamsData[i]);
    }
  }

  void startScan() {
    Serial.println("start scans");
    char initData[] = {0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x63, 0x00, 0x00, 0x63};
    for (int i = 0; i < 9; i++) {
      lserial->write(initData[i]);
    }
  }

  void serialFlush(){
    while(lserial->available()) {
      char t = lserial->read();
    }
  }

  void stopScan() {
    uint8_t stopData[] = {0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x64, 0x00, 0x00, 0x64};
    for (int i = 0; i < 9; i++) {
      lserial->write(stopData[i]);
    }
  }

  void reset() {
    Serial.println("reset");
    uint8_t resetData[] = {0xA5, 0xA5, 0xA5, 0xA5, 0x01, 0x67, 0x00, 0x00, 0x68};
    for (int i = 0; i < 9; i++) {
      lserial->write(resetData[i]);
    }
    Serial.println("reset sent");
    delay(10);

    while (lserial->available()) {
      char bufferi[2];
      sprintf (bufferi, "%02x", lserial->read());
      Serial.print(bufferi);
    }
    Serial.println();
  }

  void setBaudRate() {
    serialFlush();
    lserial->flush();
    
    Serial.println("set baud rate");
    uint8_t baudRateData[] = {0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x68, 0x01, 0x00, 0x00, 0x69};
    for (int i = 0; i < 9; i++) {
      lserial->write(baudRateData[i]);
    }

    delay(10);

    char buffer[10];
    lserial->readBytes(buffer, 10);

    for(int i = 0; i < 10; i++) {
      char bufferi[2];
      sprintf (bufferi, "%02x", buffer[i]);
      Serial.print(bufferi);
    }
    Serial.println();
  }

  void setup(Uart &serial) {
    lserial = &serial;

    while (!lserial->availableForWrite()) {
      delay(10);
    }

    _isScanning = 0;
  }

  bool processScanData(uint8_t buffer[]) {
    // NOTE: This code doesn't do the proper distance calibrations as described
    // in the YDLIDAR dev docs. The maths were computationally expensive, and
    // possibly overkill for what I needed.

    for (u_int16_t i = 0; i < (LIDAR_BUFFER_SIZE - 11) / 2; ++i) {
      u_int16_t d = (buffer[10 + 2 * i + 1] << 8 | buffer[10 + 2 * i]) & 0x01FF;
      distances[i] = d;
    }

    // scan data comes out so that the first 80 distances are in the second
    // half of the array, and vice versa. Here is where they are switched.
    // See YDLIDAR GS2 developer docs for more information
    uint16_t temp;
    for (uint8_t i = 0; i < 80; ++i) {
      temp = distances[i];
      distances[i] = distances[80 + i];
      distances[80 + i] = temp;
    }

    // here we attempt to correct for the cross-eyed vision of the two lidar
    // cameras. We computed the sorted_indexes array above, which should give
    // close to the actual order of distances.
    for (uint8_t i = 0; i < 160; ++i) {
      temp = distances[i];
      distances[i] = distances[sorted_indexes[i]];
      distances[sorted_indexes[i]] = temp;
    }

    return true;
  }

  void shiftRxBuffer(uint8_t cnt)
  {
    // If removing the whole thing, just set pos to 0
    if (cnt >= _rxBufPos)
    {
      _rxBufPos = 0;
      return;
    }

    // Otherwise do the slow shift down
    uint8_t *src = &_rxBuf[cnt];
    uint8_t *dst = &_rxBuf[0];
    _rxBufPos -= cnt;
    uint8_t left = _rxBufPos;
    while (left--)
      *dst++ = *src++;
  }

  void processDeviceParameter() {
    uint8_t addressNumber = _rxBuf[4];
    uint8_t commandTypeNumber = _rxBuf[5];
    uint8_t dataLengthNumber = (_rxBuf[7] << 8 | _rxBuf[6]);

    u_compensateK0 = _rxBuf[9] << 8 | _rxBuf[8];
    u_compensateB0 = _rxBuf[11] << 8 | _rxBuf[10];
    u_compensateK1 = _rxBuf[13] << 8 | _rxBuf[12];
    u_compensateB1 = _rxBuf[15] << 8 | _rxBuf[14];
    d_compensateK0 = u_compensateK0 / 10000.00;
    d_compensateK1 = u_compensateK1 / 10000.00;
    d_compensateB0 = u_compensateB0 / 10000.00;
    d_compensateB1 = u_compensateB1 / 10000.00;
    bias = double(_rxBuf[16]) * 0.1;

    // below code does the following
    // 1. based on calibration params calculate the theta (angle) of each distance
    //    for both cameras relative to each cameras
    // 2. using a `crossover` angle, calculate the angle of each distance relative
    //    to the origin of the sensor (so absolute). Due to the cameras being
    //    cross-eyed (each camera sees a bit of the same middle), these angles
    //    are not in asending order
    // 3. calculate a list of indexes that would give the angles in asending order
    // 4. later on, use this list to reorder the distances so they follow the real
    //    angle ordering
    float t;
    for (uint8_t i = 0; i < 160; i++) {
      if (i < 80) {
        t = leftCamThetaCalc(i);
      } else {
        t = rightCamThetaCalc(i);
      }
      thetas_deg[i] = t;
    }

    float real_angles_deg[160];
    float crossover = 22.0;
    for (uint8_t i = 0; i < 160; i++) {
      t = thetas_deg[i];
      if (i < 80) {
        t = t + crossover;
      } else {
        t = t - crossover;
      }
      real_angles_deg[i] = t;
    }

    for (uint8_t i = 0; i < 160; ++i) {
        sorted_indexes[i] = i;
    }

    for (int i = 0; i < 160 - 1; ++i) {
      int minIndex = i;
      for (int j = i + 1; j < 160; ++j) {
        if (real_angles_deg[sorted_indexes[j]] > real_angles_deg[sorted_indexes[minIndex]]) {
          minIndex = j;
        }
      }

      // Swap indexes
      int temp = sorted_indexes[i];
      sorted_indexes[i] = sorted_indexes[minIndex];
      sorted_indexes[minIndex] = temp;
    }
  }

  void printDeviceParameter() {
    Serial.print("getDeviceParams -> ");
    Serial.print("k0 ");
    Serial.print(d_compensateK0);
    Serial.print(" ");
    Serial.print(" k1 ");
    Serial.print(d_compensateK1);
    Serial.print(" ");
    Serial.print(" b0 ");
    Serial.print(d_compensateB0);
    Serial.print(" ");
    Serial.print(" b1 ");
    Serial.print(d_compensateB1);
    Serial.print(" ");
    Serial.print(" bias ");
    Serial.print(bias);
    Serial.println("");
  }

  void printScanBuffer() {
    for (uint8_t i = 0; i < 160; ++i) {
      Serial.print(distances[i]);
      Serial.print(" ");
    }
    Serial.println();
  }

  void processPacket(uint16_t len) {
    uint8_t command = _rxBuf[5];

    uint8_t chkValue = _rxBuf[len+9-1];
    uint8_t chkSum = 0;
    for(uint16_t i = 4; i < _rxBufPos - 1; i++) {
      chkSum += _rxBuf[i];
    }

    if (chkValue != chkSum) {
      // bad packet
      Serial.print("bad");
      for(int i = 0; i <_rxBufPos; i++) {
        char bufferi[2];
        sprintf (bufferi, "%02x", _rxBuf[i]);
        Serial.print(bufferi);
      }
      Serial.println();
      Serial.print(chkValue);
      Serial.print(" ");
      Serial.println(chkSum);
      return;
    }



    if (command == 0x61) {
      //Obtain Device Parameter Command
      processDeviceParameter();
    } else if (command == 0x63) {
      // start scan command response
      if (len == 0) {
        // then it's just the command response
        _isScanning = 1;
      } else {
        // then it's scan data
        processScanData(_rxBuf);
      }
    } else if (command == 0x64) {
      // stop scanning
      _isScanning = 0;
    }
  }

  void shiftArrayLeft() {
    for (int i = 0; i < _rxBufPos; i++) {
      _rxBuf[i] = _rxBuf[i + 1];
    }
    _rxBufPos--;
  }

  void handleByteReceived()
  {
    if (_rxBuf[0] == 0xA5 && _rxBuf[1] == 0xA5 && _rxBuf[2] == 0xA5 && _rxBuf[3] == 0xA5) {
      // then start of valid buffer
      if (_rxBufPos >= 8) {
        uint16_t len = ((uint16_t)_rxBuf[7] << 8) | _rxBuf[6];
        // Serial.println(len);
        if (_rxBufPos >= len + 9) {
          processPacket(len);
          _rxBufPos = 0;
        }
      }
    } else if (_rxBuf[0] == 0xA5) {
      // this could be the start of a valida packet
    } else {
      // not a valid packet so dispose
      shiftArrayLeft();
    }
  }

  void loop() {
    while (lserial->available())
    {
      uint8_t b = lserial->read();
      _lastReceive = millis();

      _rxBuf[_rxBufPos++] = b;

      // for(int i = 0; i <_rxBufPos; i++) {
      //   char bufferi[2];
      //   sprintf (bufferi, "%02x", _rxBuf[i]);
      //   Serial.print(bufferi);
      // }
      // Serial.println();

      handleByteReceived();
    }
  }

};
