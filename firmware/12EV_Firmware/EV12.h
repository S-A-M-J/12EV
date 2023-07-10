int EV12interimStage = 0;
int EV12stage = 0;
int binNumber = 0;

bool EV12running = false;


struct ColorStruct {
  float red;
  float green;
  float blue;
};

#define xBatPosInitial 980.0
#define yBatPosInitial 540.0
#define zBAT_POS 600.0
#define zLID_POS 740.0
#define xPOS_BINS -1450.0
#define zPOS_BINS 2000
#define xPOS_COLORSENSOR 50.0
#define yPOS_COLORSENSOR 1730.0
#define zPOS_COLORSENSOR 1500.0
#define xOFFSET_BAT -5
#define yOFFSET_BAT 5

int16_t batPos[15][2];
int16_t xBatPosOffset = 200; //in mm
int16_t yBatPosOffset = 200; //in mm

int16_t yPosBin[2] = {50.0, 900.0};