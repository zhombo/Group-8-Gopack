int buttonPin = 2;
volatile int count = 0;

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);  // Set the button pin as input with pull-up
  attachInterrupt(digitalPinToInterrupt(buttonPin), countButtonPresses, FALLING);
}

void loop() {
  Serial.print("Button Press Count: ");
  //int v = digitalRead(2);
  //Serial.println(v);
  Serial.println(count);
  delay(500);
}

void countButtonPresses() {
  count++;  // Increase count each time button is pressed
}