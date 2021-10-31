#include <limits.h>
#include <math.h>
#include <Arduino.h>
#include <ServoEasing.h>  //arduino library

constexpr auto CENTER = 90;
constexpr auto STOP_INTERVAL = 300;  // half a second
constexpr auto X_LIMIT = 45;
constexpr auto Y_LIMIT = 55;

volatile unsigned long stop_time = 0U;

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
    angle_cos = cos(angle);
    angle_sin = sin(angle);
    saccade(x_tgt, y_tgt); 
  }
};

static eye left, right;

static void look_together(int x, int y) {
  left.saccade(x, y);
  right.saccade(x, y);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //Serial.setTimeout(50);      //ensures the the arduino does not read serial for too long
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

  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

static void stop() {
  //TODO
  ;
}

SIGNAL(TIMER0_COMPA_vect) 
{
  // "dead man switch": if we haven't received a movement command in a while, stop.
  if (millis() > stop_time) {
    stop_time = ULONG_MAX;
    stop();
    Serial.println("halted");    
  }
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
  look_together(randomH, randomV);
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
  static auto acc_on = false; // Accelerometer support
  static auto* active_eye = &left;

  //Check Bluetooth for new Instructions
  if (Serial.available()){
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


    };  // switch(command) 
    stop_time = millis();
  };  // if byte is availablE

} // end of primary loop
