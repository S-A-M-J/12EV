

void setBatPos() {
  int index = 0;
  for (int k = 0; k < 3; k++) {
    for (int i = 0; i < 5; i++) {
      batPos[index][0] = xBatPosInitial + i * xBatPosOffset + xOFFSET_BAT * i;
      batPos[index][1] = yBatPosInitial + k * yBatPosOffset + yOFFSET_BAT * i;
      index++;
    }
  }
}

ColorStruct getColorValues() {
  ColorStruct sensorValues;
  colorSensor.setInterrupt(false);
  delay(60);  // takes 50ms to read
  colorSensor.getRGB(&sensorValues.red, &sensorValues.green, &sensorValues.blue);
  colorSensor.setInterrupt(true);  // turn off LED
  /*
  Serial.print("R:\t"); Serial.print(int(red));
  Serial.print("\tG:\t"); Serial.print(int(green));
  Serial.print("\tB:\t"); Serial.print(int(blue));
  Serial.print("\n");
  */
  return sensorValues;
}

ColorStruct color;

void moveToHome() {
  iCH1 = 0;
  iCH2 = 1990;
  iCH3 = 5000;
  iCH4 = 0;
  iCH5 = 0;
  setMove();
}

void setMove() {
  mode = 2;
  working = true;
  newMessage++;
}

void moveToBin(int n) {
  iCH1 = xPOS_BINS;
  iCH2 = yPosBin[n];
  iCH3 = zPOS_BINS;
  iCH4 = 1;
  iCH5 = 1;
  setMove();
}

void moveToColorsensor() {
  iCH1 = xPOS_COLORSENSOR;
  iCH2 = yPOS_COLORSENSOR;
  iCH3 = zPOS_COLORSENSOR;
  iCH4 = 1;
  iCH5 = 0;
  setMove();
}

void setCoordinates(int16_t x, int16_t y, int16_t z, int16_t em, int16_t elbow) {
  iCH1 = x;
  iCH2 = y;
  iCH3 = z;
  iCH4 = em;
  iCH5 = elbow;
  setMove();
}

void moveZ(int16_t z, bool EMActive) {
  if (EMActive)
    digitalWrite(EM_PIN, HIGH);
  else
    digitalWrite(EM_PIN, LOW);
  target_position_M3 = (z / 100.0) * M3_AXIS_STEPS_PER_UNIT;
}
//runs through the disassembly
//interimstages:
//0 - moves to battery
//1 - lifts battery straight up
//2 - moves to color sensor
//3 - moves to bin and drops battery
//4 - deactivates electromagnet
void EV12Disassembly() {
  if (EV12interimStage == 6) {
    EV12interimStage = 0;
    EV12stage++;
  }
  if (EV12stage == 16) {
    EV12running = false;
    EV12interimStage = 0;
    EV12stage = 0;
    return;
  }
  if (EV12stage == 0) {
    switch (EV12interimStage) {
      case 0:
        setCoordinates(batPos[7][0]+40, batPos[7][1], zLID_POS, 0, 0);
        break;
      case 1:
        moveZ(zLID_POS+2000,true);
        break;
      case 2:
        setCoordinates(batPos[7][0], -300, zLID_POS+1000, 1, 0);
        break;
      case 3:
        moveZ(500,true);
        break;
      case 4:
        moveZ(1000,false);
        break;
      case 5:
        moveZ(1000,false);
        break;
    }
  } else {
    switch (EV12interimStage) {
      case 0:
        setCoordinates(batPos[EV12stage - 1][0], batPos[EV12stage - 1][1], zBAT_POS, 0, 0);
        break;
      case 1:
        delay(100);
        moveZ(5000, true);
        break;
      case 2:
        delay(100);
        moveToColorsensor();
        break;
      case 3:
        delay(100);
        color = getColorValues();
        Serial.print(color.red);
        Serial.print("\t");
        Serial.print(color.green);
        Serial.print("\t");
        Serial.println(color.blue);
        binNumber = 0;
        if(color.green > 105){
          Serial.println("detected green");
        }else{ 
          Serial.println("detected red");
          binNumber = 1;
        }
        moveZ(4000,true);
        break;
      case 4:
        delay(100);
        moveToBin(binNumber);
        break;
      case 5:
        digitalWrite(EM_PIN, LOW);
        delay(100);
        moveToHome();
        break;
    }
  }
}

void setDelay(int ms) {
  while (1) {
    if (esp_timer_get_time() - timestamp > ms * 1000) {
      break;
    }
  }
}