// [Tools] -> [Programmer] -> "USBtinyISP"
// [Tools] -> [Board] -> "Pro Trinket 3V/12 Mhz (USB)"

#include <Servo.h>
#include <NewPing.h>

//setup sonar
#define TRIGGER_PIN  9  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// setup servo
int servoPin = 9;
int PEN_DOWN = 170; // angle of servo when pen is down
int PEN_UP = 80;   // angle of servo when pen is up
Servo penServo;

int wheel_dia = 63.85; //      # mm (increase = spiral out)
int wheel_base = 112; //,    # mm (increase = spiral in)
int steps_rev = 128; //,     # 512 for 64x gearbox, 128 for 16x gearbox
int delay_time = 6; //            # time between steps in ms

double const pi = 3.14159265359;

// Stepper sequence org->pink->blue->yel
int L_stepper_pins[] = {10, 12, 13, 11};
int R_stepper_pins[] = {3, 5, 6, 4};

int fwd_mask[][4] =  {{1, 0, 1, 0},
  {0, 1, 1, 0},
  {0, 1, 0, 1},
  {1, 0, 0, 1}
};

int rev_mask[][4] =  {{1, 0, 0, 1},
  {0, 1, 0, 1},
  {0, 1, 1, 0},
  {1, 0, 1, 0}
};


void setup() {
  randomSeed(analogRead(1));
  // put your setup code here, to run once:
  Serial.begin(115200);
  for (int pin = 0; pin < 4; pin++) {
    pinMode(L_stepper_pins[pin], OUTPUT);
    digitalWrite(L_stepper_pins[pin], LOW);
    pinMode(R_stepper_pins[pin], OUTPUT);
    digitalWrite(R_stepper_pins[pin], LOW);
  }
  penServo.attach(servoPin);
  Serial.println("setup");
}


void loop() { 
  //----- Measure Arena -----
  unsigned int prelength = sonar.ping_cm();
  unsigned int Length = prelength + 80; //update it to be the length from sensor to end of vehicle
  forward(prelength - 5);
  left(90);
  unsigned int prewidth = sonar.ping_cm();
  unsigned int Width = prewidth + 80; //update it " "

  //----- Calculate Proportions -----
  double totalWhiteSpace = Length * (1.0 / 10);
  double WhiteSpace = totalWhiteSpace / 5;

  double remainingLength = Length - totalWhiteSpace;
  double horizArm = remainingLength / 6;

  double header = Width * (.2);
  double remainingWidth = Width - (header * 2);
  double vertArm = remainingWidth / 2;

  //----- Move Robot to Start Position -----
  left(180);
  forward(Width);
  right(90);
  forward(Length);

  //----- Calibration Squares -----
  /*for(int x=0; x<12; x++){
    forward(100);
    left(90);
    }*/
  //----- Cool Spiral -----
  /*for(int x=1; x<90; x+=3){
    forward(x);
    left(45);
    }*/
  //----- ECE 202 -----
  /*//E
  forward(horizArm); right(90); forward(vertArm * 2); right(90); forward(horizArm); penup(); right(90); forward(vertArm); right(90); forward(horizArm * 0.4); pendown(); forward(horizArm * 0.6); penup();

  right(90); forward(vertArm); right(90); forward(horizArm + WhiteSpace + horizArm); right(180);
  //C
  pendown(); forward(horizArm); left(90); forward(vertArm * 2); left(90); forward(horizArm); penup();

  forward(WhiteSpace + horizArm); right(180);
  //E
  pendown(); forward(horizArm); right(90); forward(vertArm * 2); right(90); forward(horizArm); penup(); right(90); forward(vertArm); right(90); forward(horizArm * 0.4); pendown(); forward(horizArm * 0.6); penup();

  right(90); forward(vertArm); right(90); forward(horizArm + WhiteSpace);
  //2
  pendown(); forward(horizArm); right(90); forward(vertArm); right(90); forward(horizArm); left(90); forward(vertArm); left(90); forward(vertArm); penup();

  forward(WhiteSpace);
  //0
  pendown(); forward(horizArm); left(90); forward(vertArm * 2); left(90); forward(horizArm); left(90); forward(vertArm * 2); penup();

  left(90); forward(horizArm + WhiteSpace); left(90); forward(vertArm * 2); right(90);
  //2
  pendown(); forward(horizArm); right(90); forward(vertArm); right(90); forward(horizArm); left(90); forward(vertArm); left(90); forward(horizArm); penup();*/

  done();      // releases stepper motor
  while (1);   // wait for reset
}


// ----- HELPER FUNCTIONS -----------
int step(float distance) {
  int steps = distance * steps_rev / (wheel_dia * 3.1412); //24.61
  /*
    Serial.print(distance);
    Serial.print(" ");
    Serial.print(steps_rev);
    Serial.print(" ");
    Serial.print(wheel_dia);
    Serial.print(" ");
    Serial.println(steps);
    delay(1000);*/
  return steps;
}


void forward(float distance) {
  int steps = step(distance);
  Serial.println(steps);
  for (int step = 0; step < steps; step++) {
    for (int mask = 0; mask < 4; mask++) {
      for (int pin = 0; pin < 4; pin++) {
        digitalWrite(L_stepper_pins[pin], rev_mask[mask][pin]);
        digitalWrite(R_stepper_pins[pin], fwd_mask[mask][pin]);
      }
      delay(delay_time);
    }
  }
}


void backward(float distance) {
  int steps = step(distance);
  for (int step = 0; step < steps; step++) {
    for (int mask = 0; mask < 4; mask++) {
      for (int pin = 0; pin < 4; pin++) {
        digitalWrite(L_stepper_pins[pin], fwd_mask[mask][pin]);
        digitalWrite(R_stepper_pins[pin], rev_mask[mask][pin]);
      }
      delay(delay_time);
    }
  }
}


void right(float degrees) {
  float rotation = degrees / 360.0;
  float distance = wheel_base * 3.1412 * rotation;
  int steps = step(distance);
  for (int step = 0; step < steps; step++) {
    for (int mask = 0; mask < 4; mask++) {
      for (int pin = 0; pin < 4; pin++) {
        digitalWrite(R_stepper_pins[pin], rev_mask[mask][pin]);
        digitalWrite(L_stepper_pins[pin], rev_mask[mask][pin]);
      }
      delay(delay_time);
    }
  }
}


void left(float degrees) {
  float rotation = degrees / 360.0;
  float distance = wheel_base * 3.1412 * rotation;
  int steps = step(distance);
  for (int step = 0; step < steps; step++) {
    for (int mask = 0; mask < 4; mask++) {
      for (int pin = 0; pin < 4; pin++) {
        digitalWrite(R_stepper_pins[pin], fwd_mask[mask][pin]);
        digitalWrite(L_stepper_pins[pin], fwd_mask[mask][pin]);
      }
      delay(delay_time);
    }
  }
}


void done() { // unlock stepper to save battery
  for (int mask = 0; mask < 4; mask++) {
    for (int pin = 0; pin < 4; pin++) {
      digitalWrite(R_stepper_pins[pin], LOW);
      digitalWrite(L_stepper_pins[pin], LOW);
    }
    delay(delay_time);
  }
}


void penup() {
  delay(250);
  Serial.println("PEN_UP()");
  penServo.write(PEN_UP);
  delay(250);
}


void pendown() {
  delay(250);
  Serial.println("PEN_DOWN()");
  penServo.write(PEN_DOWN);
  delay(250);
}
