
#include <BLE_Server2.h>

#include <TFT_eSPI.h>
#include <ui.h>
#include <CST816S.h>

#define BAT_ADC_PIN  1

unsigned long startMillis;  
// unsigned long currentMillis;
const unsigned long period = 50;  //the value is a number of milliseconds

const int GUP = 1;
const int GDN = 2;
const int GRT = 3;
const int GLT = 4;

extern lv_obj_t * ui_gpointer;
extern lv_obj_t * ui_maxgpointer;
extern lv_obj_t * ui_mingpointer;
extern lv_obj_t * ui_horizon;
extern lv_obj_t * ui_hourhand;
extern lv_obj_t * ui_minhand;
extern lv_obj_t * ui_sechand;
extern lv_obj_t * ui_acsymbol;
extern lv_obj_t * ui_accellerationbar;
extern lv_obj_t * ui_rot;  //deg/min
extern lv_obj_t * ui_lblSpeed; //knots
extern lv_obj_t * ui_lblCrs;  //degrees
extern lv_obj_t * ui_lblAcceleration;
extern lv_obj_t * ui_lblCenplAc;
extern lv_obj_t * ui_lblThousandsHt;
extern lv_obj_t * ui_lblUnitsHt;
extern lv_obj_t * ui_lblGS;
extern lv_obj_t * ui_ArcCPAcc;
extern lv_obj_t * ui_lblCRS;

// Array to store screen objects
lv_obj_t* screens[5];

extern int reset_minmax;
extern int re_start;
extern int gotosleep;
extern int synctime;
extern int timeplusonehr;
extern int timeminusonehr;

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

CST816S touch(6, 7, 13, 5);	// sda, scl, rst, irq
TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

void my_touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data){

  static uint16_t first_x = 0;
  static uint16_t first_y = 0;
  static uint16_t last_x = 0;
  static uint16_t last_y = 0;
  static boolean firsttouch = true;
  static int guesture = 0;
  static uint8_t current_screen = 0;

  if(touch.available()) {
    uint16_t  gui_x;
    uint16_t  gui_y;

    switch (GUIROTATION){
      case 0:
        gui_x = touch.data.x;
        gui_y = touch.data.y;
        break;
      case 1:
        gui_x = touch.data.y;
        gui_y = 240 - touch.data.x;
        break;
      }

    if(firsttouch == true ){
      first_x = gui_x;
      first_y = gui_y;
      firsttouch = false;
    }
    last_x = gui_x;
    last_y = gui_y;
    data->point.x = last_x;
    data->point.y = last_y;

    // Serial.print ("X, Y : ");
    // Serial.print(last_x);
    // Serial.print(", ");
    // Serial.println(last_y);

    if((last_x - first_x) > 20)  guesture = GRT;
    if((first_x - last_x) > 20)  guesture = GLT;
    if((last_y - first_y) > 20)  guesture = GDN;
    if((first_y - last_y) > 20)  guesture = GUP;

    if (guesture == 0) data->state = LV_INDEV_STATE_PR;

  }else{
    //touch has ended
    // if (guesture > 0){
    //   last_x = 0;
    //   last_y = 0;
    // } 
    firsttouch = true;
    data->point.x = last_x;
    data->point.y = last_y;
    data->state = LV_INDEV_STATE_REL;
    if(guesture > 0){
      switch (guesture) {
      case GUP:
        //Serial.println("GUP");
        break;
      case GDN:
        //Serial.println("GDN");
        break;
      case GRT:
        //Serial.print("GRT ");
        current_screen = (current_screen + 1) % 5;
        //Serial.println(current_screen);
        lv_scr_load(screens[current_screen]);
        break;
      case GLT:
        //Serial.print("GLT ");
        current_screen = (current_screen + 4) % 5;
        //Serial.println(current_screen);
        lv_scr_load(screens[current_screen]);
        break;
      }
      guesture = 0;
      // Serial.print("start; ");
      // Serial.print(first_x);
      // Serial.print(", ");
      // Serial.println(first_y);
      // Serial.print("finish; ");
      // Serial.print(last_x);
      // Serial.print(", ");
      // Serial.println(last_y);
    }
  }
}

// Function to get the current time using millis() since last synchronization
void getCurrentTime(float &hours, float &minutes, float &seconds) {
  // Calculate the time elapsed since the last synchronization
  unsigned long currentMillis = millis();
  unsigned long elapsedMillis = currentMillis - lastSyncMillis;

  // Update seconds, minutes, and hours
  float elapsedSeconds = elapsedMillis / 1000.0;

  // Add elapsed time to the synchronized time
  float totalSeconds = syncSeconds + elapsedSeconds;
  seconds = fmod(totalSeconds, 60);
  float totalMinutes = syncMinutes + (totalSeconds / 60);
  minutes = fmod(totalMinutes, 60);
  hours = fmod(syncHours + (totalMinutes / 60), 12);  // Wrap around after 24 hours

  #ifdef PRINTGUI
    // Debugging: Print the updated time
    Serial.print("Current Time: ");
    Serial.print(hours);
    Serial.print(" hours, ");
    Serial.print(minutes);
    Serial.print(" minutes, ");
    Serial.print(seconds);
    Serial.println(" seconds.");
  #endif
}

void setupGUI(){

  touch.begin();

  lv_init();

  #if LV_USE_LOG != 0
      lv_log_register_print_cb( my_print ); /* register print function for debugging */
  #endif

  tft.begin();          /* TFT init */
  tft.setRotation( GUIROTATION ); /* Landscape orientation, flipped */

  lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register( &disp_drv );

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init( &indev_drv );
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register( &indev_drv );

  //get gmt offset from EEPROM, this to avoid reading EEPROM constantly
  gmtoffset = preferences.getUInt("gmtoffset", 0);

  ui_init();

  lv_img_set_angle(ui_gpointer, 0);
  lv_img_set_angle(ui_maxgpointer, 0);
  lv_img_set_angle(ui_mingpointer, 0);
  lv_img_set_angle(ui_AOAneedle, 0);

  screens[0] = ui_gmeter;
  screens[1] = ui_aoa;
  screens[2] = ui_attitude_indicator;
  screens[3] = ui_watch;
  screens[4] = ui_setup;

  startMillis = millis();  //initial start time
}


void doGUI(){

  unsigned long  currentMillis = millis();
  if (currentMillis - startMillis >= period){  //test whether the period has elapsed
    startMillis = currentMillis;

    //rate of turn scales at +/- 360 deg/sec
    int indicated_value = (int)(rot_deg_min);
  
    lv_label_set_text(ui_lblSpeed, String(1.94384 * speed).c_str());
    lv_label_set_text(ui_lblCrs,   String(course).c_str());
    lv_label_set_text(ui_lblAcceleration,  String(accelXGPS).c_str());
    lv_label_set_text(ui_lblCenplAc,  String(centripetalAccel).c_str());

    int height = (int)ble_floatValues[ALT];
    int thousands = height/1000;
    int units = height - thousands;
    lv_label_set_text(ui_lblThousandsHt, String(thousands).c_str());
    lv_label_set_text(ui_lblUnitsHt, String(units).c_str());
    int speedknotsgs =  (int)(1.94384 * speed);
    lv_label_set_text(ui_lblGS, String(speedknotsgs).c_str());
    lv_bar_set_value(ui_accellerationbar, accelXGPS * 50, LV_ANIM_ON);
    lv_arc_set_value(ui_ArcCPAcc,  accYangle * 30);
    lv_arc_set_value(ui_rot, indicated_value);
    int hdgTrue = (int)course;
    lv_label_set_text(ui_lblCRS, String(hdgTrue).c_str());

    //time
    float currentHours, currentMinutes, currentSeconds;
    int hourAngle, minAngle, secAngle;

    //adjust UTC to local time
    int offset = (int)gmtoffset;
    lv_label_set_text(ui_lblhours, String(offset).c_str());

    if(timeplusonehr == 1){
      gmtoffset++;
      preferences.putUInt("gmtoffset", gmtoffset);
      timeplusonehr = 0;
    }
    if(timeminusonehr == 1){
      gmtoffset--;
      preferences.putUInt("gmtoffset", gmtoffset);
      timeminusonehr = 0;
    }

    getCurrentTime(currentHours, currentMinutes, currentSeconds);
    
    currentHours = currentHours + (float)offset;
    hourAngle = int(3600 * (currentHours/12));
    minAngle = int(3600 * (currentMinutes/60));
    secAngle = int(3600 * (currentSeconds/60));
    lv_img_set_angle(ui_hourhand, hourAngle);
    lv_img_set_angle(ui_minhand, minAngle);
    lv_img_set_angle(ui_sechand, secAngle);


    //gmeter
    int g = int((zacceleration - 1) * 450);
    static int gmax = 0;
    static int gmin = 0;
    lv_img_set_angle(ui_gpointer, g);
    if (reset_minmax != 0) {
      gmax = g;
      gmin = g;
      reset_minmax = 0;
    }
    if (gmax < g) gmax = g;
    lv_img_set_angle(ui_maxgpointer, gmax);
    if (gmin > g) gmin = g;
    lv_img_set_angle(ui_mingpointer, gmin);

    //horizon
    float filterval = 0.7;
    int roll = 0;
    int pitch = 0;
    if (wifiConnected){
      roll = gdlroll * 10;
      pitch = gdlpitch * 3.3;
    }else{
      roll = imuroll * 10;
      pitch = imupitch * 3.3;      
    }

    lv_obj_set_y(ui_Image1, pitch);
    lv_img_set_pivot(ui_Image1, 120, 240 - pitch);
    lv_img_set_angle(ui_Image1, roll);

    //aoa
    // int aoa = map(ble_intValues[0], 0, 255, 2700, 1200);
    int aoa = map(ble_floatValues[0], 0, 255, 2700, 1200);
    lv_img_set_angle(ui_AOAneedle, aoa);

    lv_obj_move_background(ui_imglowersemicircle);
    lv_obj_move_background(ui_imguppersemicircle);
    lv_obj_move_background(ui_imgyellowchevron);
    lv_obj_move_background(ui_imgredchevron);
    lv_obj_move_background(ui_imggreentriangle);

    //aoa = map(ble_intValues[0], 0, 255, 0, 16);
    aoa = map(ble_floatValues[0], 0, 255, 0, 16);
    if (aoa > 12) lv_obj_move_foreground(ui_imgredchevron);
    if ((aoa > 10)&&(aoa < 14)) lv_obj_move_foreground(ui_imguppersemicircle);
    if ((aoa > 8)&& (aoa < 12)) lv_obj_move_foreground(ui_imglowersemicircle);
    if ((aoa > 4)&& (aoa < 10)) lv_obj_move_foreground(ui_imgyellowchevron);
    if ((aoa >= 0)&&(aoa < 6)) lv_obj_move_foreground(ui_imggreentriangle);

    //voltage
    float value = analogReadMilliVolts(BAT_ADC_PIN);
    value = (3.3 * value)/4096;
    value = value * 4.05;
    lv_label_set_text(ui_lblVolts, String(value).c_str());

    //back light, set on voltage
    int bl_value = 50;
    if (value > 3.2) bl_value = 100;
    touch.setBL(bl_value);

    //temperature
    //float temperature = IMU.getTemp();
    //Serial.print (temperature);

    #ifdef PRINTGUI
      Serial.print("Pitch =  ");
      Serial.print(pitch );
      Serial.print(", Roll  = ");
      Serial.println(roll);

      Serial.print("aoa = ");
      Serial.print(aoa);
      Serial.print(", course = ");
      Serial.print(ble_floatValues[4]);
      Serial.print(", speed = ");
      Serial.println(ble_floatValues[5]);
    #endif

    //sleep
    if (gotosleep > 0){
      //touch deep sleep and prepare for wake on touch
      touch.deepsleepprep();
      //close the IMU, put into deep sleep
      //Serial.print("Close IMU"); Serial .println(IMU.close());
      //IMU.close();
      esp_deep_sleep_start();   //finally deepsleep the ESP32
      
      Serial.println ("should not get here");
    }

    //to recalibrate gyro restart the ESP
    if (re_start == 1){
      Serial.print("*********re_start = ");
      Serial.println(re_start);
      re_start = 0;
      IMUCalibrate();
    }

    lv_timer_handler(); /* let the GUI do its work */
  }
}

//#endif