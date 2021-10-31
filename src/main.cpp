#include <limits.h>
#include <math.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <FastCRC.h>
#include <ServoEasing.h>  //arduino library

constexpr auto CENTER = 90;
constexpr auto STOP_INTERVAL = 300;  // half a second
constexpr auto X_LIMIT = 45;
constexpr auto Y_LIMIT = 55;

static FastCRC32 CRC32;

// TODO:
// -> axis angles
// -> movement generation

static int clamp(int value, int max) {
  if (value > max) return max;
  if (value < -max) return -max;
  return value;
}

static inline void init_servo(ServoEasing& servo, int pin, int trim) {
  servo.write(CENTER);
  servo.attach(pin);
  servo.setReverseOperation(false);
  servo.setSpeed(360);
  servo.setEaseTo(EASE_CUBIC_IN_OUT);
  servo.setTrim(CENTER);
  servo.write(0);
}

struct eye {
  ServoEasing x, y;
  float angle;
  float angle_cos, angle_sin;
  float x_tgt, y_tgt;

  eye() : angle(0), angle_cos(1), angle_sin(0), x_tgt(0), y_tgt(0) {};

  inline void attach(int x_pin, int x_bias, int y_pin, int y_bias) {
    init_servo(x, x_pin, x_bias);
    init_servo(y, y_pin, y_bias);
  }
  inline void saccade(float x_pos, float y_pos) {
    x_tgt = x_pos; y_tgt = y_pos;
    
    // Apply angle correction
    x_pos = x_pos * angle_cos - y_pos * angle_sin;
    y_pos = y_pos * angle_cos + x_pos * angle_sin;

    x_pos = clamp(x_pos, X_LIMIT);
    y_pos = clamp(y_pos, Y_LIMIT);

    Serial.print("X: ");
    Serial.print(x_pos);
    Serial.print(" Y: ");
    Serial.println(y_pos);
    x.startEaseTo(x_pos);
    y.startEaseTo(y_pos);
  }

  inline void set_x_trim(int x_trim) {
    x.setTrim(CENTER+x_trim, true);
  }

  inline void set_y_trim(int y_trim) {
    y.setTrim(CENTER+y_trim, true);
  }

  inline void set_angle(float n_angle) {
    angle = n_angle;
    auto angle_rad = angle * M_PI / 180;
    angle_cos = cos(angle_rad);
    angle_sin = sin(angle_rad);
    saccade(x_tgt, y_tgt); 
  }
};

static eye left, right;

struct settings_t {
  float val[6]; // angle, x, y trim, two eyes
};

static void look_together(int x, int y) {
  left.saccade(x, y);
  right.saccade(x, y);
}

void save_eeprom() {
  auto settings = settings_t {};
  settings.val[0] = left.angle;
  settings.val[1] = left.x.mTrimMicrosecondsOrUnits;
  settings.val[2] = left.y.mTrimMicrosecondsOrUnits;
  settings.val[3] = right.angle;
  settings.val[4] = right.x.mTrimMicrosecondsOrUnits;
  settings.val[5] = right.y.mTrimMicrosecondsOrUnits;
  auto* setting_ptr = reinterpret_cast<uint8_t*>(&settings);

  for(auto ptr = 0U; ptr < sizeof(settings); ++ptr) {
    EEPROM.update(ptr, setting_ptr[ptr]);
  }

  auto crc = CRC32.crc32(setting_ptr, sizeof(settings));
  auto crc_ptr = reinterpret_cast<uint8_t*>(&crc);
  for(auto ptr = 0U; ptr < sizeof(settings) + sizeof(crc); ++ptr) {
    EEPROM.update(sizeof(settings) + ptr, crc_ptr[ptr]);
  }
};

int load_eeprom() {
  constexpr auto EEPROM_SIZE = sizeof(settings_t) + sizeof(uint32_t);
  uint8_t buf[EEPROM_SIZE];
  for(auto ptr = 0U; ptr < sizeof(buf); ++ptr) {
    buf[ptr] = EEPROM.read(ptr);
  };
  // Check CRC
  uint32_t eeprom_crc;
  memcpy(&eeprom_crc, &buf[sizeof(buf) - sizeof(eeprom_crc)], sizeof(eeprom_crc));
  auto computed_crc = CRC32.crc32(buf, sizeof(buf) - sizeof(eeprom_crc));
  if (eeprom_crc == computed_crc) {
    auto settings = settings_t {};
    memcpy(&settings, buf, sizeof(settings));
    left.x.setTrimMicrosecondsOrUnits(settings.val[1]);
    left.y.setTrimMicrosecondsOrUnits(settings.val[2]);
    right.x.setTrimMicrosecondsOrUnits(settings.val[4]);
    right.y.setTrimMicrosecondsOrUnits(settings.val[5]);
    left.set_angle(settings.val[0]);
    right.set_angle(settings.val[3]);
    Serial.println("load ok");
    return 0;
  } else {
    Serial.println("load CRC failure");
    return -1;
  }
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(5000);      //ensures the the arduino does not read serial for too long
  // Send control panel specifications
  Serial.println("*.kwl");
  Serial.println("clear_panel()");
  Serial.println("set_grid_size(16,8)");
  // Main control
  Serial.println("add_touch_pad(1,1,6,-85,85,0,100,,*)");
  // Accelerometer
  Serial.println("add_accelerometer(12,1,5,150,A,*)");
  Serial.println("add_text(13,3,large,R,Accelerometer control:,200,200,200,)");
  Serial.println("add_switch(14,3,2,B,b,0,0)");
  // Backup 4-way pad
  Serial.println("add_4way_pad(12,4,1,2,3,4,0,150,,*)");
  // Sensor output
  Serial.println("add_gauge(1,0,5,0,100,0,T,,,10,5)");
  // Notes
  Serial.println("set_panel_notes(Racer Robot,Remote control for Littlebot,,)");
  Serial.println("run()");
  Serial.println("*");

  // Fill in biases
  left.attach(2, 0, 3, 0);
  right.attach(4, 0, 5, 0);
  load_eeprom();

  randomSeed(analogRead(0)); /* creates some random values using analog noise from a floating
  analog pin */
}


float parabola(float a) {
  float result;
  result = -4 * sq(a - 0.5) + 1;
  return result;
}

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void rollEye() {
  // Rolls Eye One Time
  Serial.println("rollEye");
  for (float i = 0; i <= 1; i = i + 0.05) {
    float x = i;
    float y = parabola(x);

    x = floatMap(x, 0, 1, -X_LIMIT, X_LIMIT);
    y = floatMap(y, 0, 1, Y_LIMIT, -Y_LIMIT);

    look_together(x, y);

    delay(30);
  }
  for (float i = 1; i >= 0; i = i - 0.05) {
    float x = i;

    x = floatMap(x, 0, 1, -X_LIMIT, X_LIMIT);

    look_together(x, Y_LIMIT);

    delay(25);
  }
}

void randomTwitching() {
  auto randomH = random(-X_LIMIT, X_LIMIT); // random x position for horizontal servo
  auto randomV = random(-Y_LIMIT, Y_LIMIT); // random y position for vertical servo
  Serial.println("random");
  look_together(randomH, randomV);
}

void cross_eyes() {
  Serial.println("cross");
}

static int get_second_value(char c) {
  auto command = '\0';
  int result = 0;
  while (command != '*') {
    if (Serial.available()) {
      command = Serial.read(); //Get next character from bluetooth
      if (command == c) result = Serial.parseInt();
    }
  }
  // EOC happened first
  return result;
}


void loop() 
{ 
  static unsigned long next_time = 15000UL;
  static auto acc_on = false; // Accelerometer support
  static auto* active_eye = &left;

  //Check Bluetooth for new Instructions
  if (Serial.available()){
      bool reset_timer = true;
      auto command = Serial.read(); //Get next character from bluetooth
     
      //**** Accelerometer  -  sends 'Aroll,pitch*' every 150 ms
      switch(command) {
        case 'A':
        {
          auto roll = Serial.parseInt(); 
          auto pitch = get_second_value(',');         
          if (acc_on){ 
            look_together(roll, pitch);
          }
        }
        break;

        case 'B':
          acc_on = true;
          break;
        case 'b':
          acc_on = false;
          break;

        // X, Y position
        // Format: Xnnn,Ynnn*
        case 'X': {
          auto pad_x = Serial.parseInt();
          auto pad_y = -get_second_value('Y');
          look_together(pad_x, pad_y);
        };
        break;

        // Control pad
        case '0': // release
          look_together(0, 0);
          break;
          
        case '1': // up
          look_together(0, 50);
          break;

        case '2': // right
          look_together(50, 0);
          break;

        case '3': // down
          look_together(0, -50);
          break;

        case '4': // left
          look_together(-50, 0);
          break;

        case 'T': // test
        {
          auto pad_x = Serial.parseInt();
          look_together(pad_x, pad_x);
        }
        break;

        case 'R':
          rollEye();
          break;

        case 'Z':
          randomTwitching();
          break;

        case 'e':
          active_eye = &left;
          break;
        case 'E':
          active_eye = &right;
          break;

        case 'D': {
          auto angle = Serial.parseInt();
          active_eye->set_angle(angle);
          break;
        };

        case 'x': {
          auto trim = Serial.parseInt();
          active_eye->set_x_trim(trim);
          break;
        };
        
        case 'y': {
          auto trim = Serial.parseInt();
          active_eye->set_y_trim(trim);
          break;
        };

        case 's':
          save_eeprom();
          break;

        case 'S':
          load_eeprom();
          break;
        
        case 'g': // go
          reset_timer = false;
          next_time = 0;
          break;
    };  // switch(command) 
    Serial.println("done");
    if (reset_timer) {
      next_time = millis() + (1000UL * 60UL * 3UL);
    }
  } else if (millis() > next_time) {
    // Do a random behavior
    auto t = random(0, 1000);
    if (t < 980) {
      randomTwitching();
      next_time = millis() + random(300, 1500);
    } else if (t < 990) {
      rollEye();
      next_time = millis() + 200;
    } else {
      cross_eyes();
      next_time = millis() + 100;
    }
  }

} // end of primary loop
