#include <Deque.h>

Deque<char, 10> queue;

void setup() {
  Serial.begin(230400);
}

void loop() {
  while(Serial.available()) {
    queue.push_back(Serial.read());
  }

  Serial.print(millis() / 1000);
  Serial.print(": ");
  int size = queue.size();
  if (size > 4) {
    Serial.print("Found ");
    Serial.print(size);
    Serial.print(" items. '");
    Serial.print(queue.front());
    Serial.print("' is the oldest. The next oldest is '");
    queue.pop_front();
    Serial.print(queue.front());
    Serial.print(". '");
    Serial.print(queue.back());
    Serial.print("' is the most new. The next most new is '");
    queue.pop_back();
    Serial.print(queue.back());
    Serial.println("'.");
  } else {
    Serial.println("Nothing to process..."); 
  }
  delay(2000);
}