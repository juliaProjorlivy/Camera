#include <DIYables_IRcontroller.h> // DIYables_IRcontroller library
#define IR_RECEIVER_PIN 7 // The Arduino pin connected to IR controller
#define SIGNAl_PORT 10 // The Arduino pin connected to Raspberry pi 16 pin (to control turning on)

DIYables_IRcontroller_21 irController(IR_RECEIVER_PIN, 200); // debounce time is 200ms

void setup() {
  pinMode(SIGNAL_PORT, OUTPUT);
  Serial.begin(9600);
  irController.begin();
}

void loop() {
  Key21 command = irController.getKey();
  if (command != Key21::NONE) {
    switch (command) {
      case Key21::KEY_CH_MINUS:
        Serial.println("CH-");
        digitalWrite(SIGNAL_PORT, HIGH);    // turn the LED on by making the voltage HIGH
        delay(1000);                       // wait for a second
        digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_CH:
        Serial.println("CH");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_CH_PLUS:
        Serial.println("CH+");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_PREV:
        Serial.println("<<");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_NEXT:
        Serial.println(">>");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_PLAY_PAUSE:
        Serial.println(">||");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_VOL_MINUS:
        Serial.println("–");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_VOL_PLUS:
        Serial.println("+");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_EQ:
        Serial.println("EQ");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_100_PLUS:
        Serial.println("100+");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_200_PLUS:
        Serial.println("200+");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_0:
        Serial.println("0");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_1:
        Serial.println("1");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_2:
        Serial.println("2");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_3:
        Serial.println("3");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_4:
        Serial.println("4");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_5:
        Serial.println("5");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_6:
        Serial.println("6");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_7:
        Serial.println("7");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_8:
        Serial.println("8");
        // TODO: YOUR CONTROL
        break;

      case Key21::KEY_9:
        Serial.println("9");
        // TODO: YOUR CONTROL
        break;

      default:
        Serial.println("WARNING: undefined command:");
        break;
    }
  }
}
