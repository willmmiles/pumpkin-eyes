#include <limits.h>
#include <Arduino.h>
#include <ServoEasing.h>  //arduino library

constexpr auto CENTER = 90;
constexpr auto STOP_INTERVAL = 300;  // half a second
volatile unsigned long stop_time = 0U;

static inline void init_servo(ServoEasing& servo, int pin, int trim) {
  servo.write(90);
  servo.attach(pin);
  servo.setReverseOperation(false);
  servo.setSpeed(30);
  servo.setTrim(CENTER, true);
  servo.write(0);
}

struct eye {
  ServoEasing x, y;

  eye() {};

  inline void attach(int x_pin, int x_bias, int y_pin, int y_bias) {
    init_servo(x, x_pin, x_bias);
    init_servo(y, y_pin, y_bias);
  }
  inline void saccade(int x_pos, int y_pos) {
    //x.startEaseTo(x_pos);
    //y.startEaseTo(y_pos);
    x.write(x_pos);
    y.write(y_pos);
  }
};

static eye left, right;

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

static int clamp(int value, int max) {
  if (value > max) return max;
  if (value < -max) return -max;
  return value;
}

static void look_together(int x, int y) {
  x = clamp(x, 90);
  y = clamp(y, 90);
  left.saccade(x, y);
  right.saccade(x, y);
}

void loop() 
{ 
  static auto acc_on = false; // Accelerometer support

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
    };  // switch(command) 
  };  // if byte is availablE

} // end of primary loop
