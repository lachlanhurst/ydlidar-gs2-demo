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

  public:
    // float compK0, compK1, compB0, compB1, bias;
    double d_compensateK0, d_compensateK1, d_compensateB0, d_compensateB1;
    uint16_t u_compensateK0, u_compensateK1, u_compensateB0, u_compensateB1;
    double bias;

    uint16_t distances[160];
    float thetas_deg[160];
    uint8_t sorted_indexes[160];

  void getDeviceParams() {
    char getParamsData[] = {0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x61, 0x00, 0x00, 0x61};
    for (int i = 0; i < 9; i++) {
      lserial->write(getParamsData[i]);
    }

    char buffer[18];
    lserial->readBytes(buffer, 18);

    uint8_t addressNumber = buffer[4];
    uint8_t commandTypeNumber = buffer[5];
    uint8_t dataLengthNumber = (buffer[7] << 8 | buffer[6]);

    u_compensateK0 = buffer[9] << 8 | buffer[8];
    u_compensateB0 = buffer[11] << 8 | buffer[10];
    u_compensateK1 = buffer[13] << 8 | buffer[12];
    u_compensateB1 = buffer[15] << 8 | buffer[14];
    d_compensateK0 = u_compensateK0 / 10000.00;
    d_compensateK1 = u_compensateK1 / 10000.00;
    d_compensateB0 = u_compensateB0 / 10000.00;
    d_compensateB1 = u_compensateB1 / 10000.00;
    bias = double(buffer[16]) * 0.1;

    Serial.print("getDeviceParams -> ");
    Serial.print(addressNumber);
    Serial.print(" ");
    Serial.print(commandTypeNumber, HEX);
    Serial.print(" ");
    Serial.print(dataLengthNumber);
    Serial.print(" ");
    Serial.println();

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

    for(int i = 0; i < 18; i++) {
      char bufferi[2];
      sprintf (bufferi, "%02x", buffer[i]);
      Serial.print(bufferi);
    }
    Serial.println();

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
    float crossover = 26.0;
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

  void startScan() {
    Serial.println("start scans");
    char initData[] = {0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x63, 0x00, 0x00, 0x63};
    for (int i = 0; i < 9; i++) {
      lserial->write(initData[i]);
    }
    Serial.println("sent start scans");

    while (lserial->available() < 9) {
      delay(5);
    }

    char buffer[9];
    lserial->readBytes(buffer, 9);

    for(int i = 0; i < 9; i++) {
      char bufferi[2];
      sprintf (bufferi, "%02x", buffer[i]);
      Serial.print(bufferi);
    }
    Serial.println();
  }

  void serialFlush(){
    while(lserial->available()) {
      char t = lserial->read();
    }
  }

  void stopScan() {
    Serial.println("stop scan");
    uint8_t stopData[] = {0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x64, 0x00, 0x00, 0x64};
    for (int i = 0; i < 9; i++) {
      lserial->write(stopData[i]);
    }
    delay(15);
    while (lserial->available() < 9) {
      delay(5);
    }

    char buffer[9];
    lserial->readBytes(buffer, 9);

    for(int i = 0; i < 9; i++) {
      char bufferi[2];
      sprintf (bufferi, "%02x", buffer[i]);
      Serial.print(bufferi);
    }
    Serial.println();
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

    stopScan();

    lserial->flush();
    serialFlush();

    getDeviceParams();
  }

  bool checkBuffer(uint8_t buffer[]) {
    if (buffer[0] == 0xA5 && buffer[1] == 0xA5 && buffer[2] == 0xA5 && buffer[3] == 0xA5) {
      uint8_t sum = 0;
      for(int i = 4; i < LIDAR_BUFFER_SIZE-1; i++) {
        sum += ((uint8_t)buffer[i]);
      }
      if (sum == buffer[LIDAR_BUFFER_SIZE-1]) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  bool processBuffer(uint8_t buffer[]) {
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

  void loop() {
    // only read out of serial if there's enough bytes
    // to give a full scan packet. Otherwise we'll
    // grab it next time loop is called.
    if (lserial->available() >= LIDAR_BUFFER_SIZE) {
      uint8_t buffer[LIDAR_BUFFER_SIZE];
      lserial->readBytes(buffer, LIDAR_BUFFER_SIZE);

      // for(int i = 0; i < LIDAR_BUFFER_SIZE; i++) {
      //   char bufferi[2];
      //   sprintf (bufferi, "%02x", buffer[i]);
      //   Serial.print(bufferi);
      // }
      // Serial.println();

      processBuffer(buffer);
    }
  }

};
