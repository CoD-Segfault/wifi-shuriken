#include <Arduino.h>  // Framework entry-point linkage only.
#include "controller/application.h"

void setup() { controller::application().begin(); }
void loop() { controller::application().poll(); }
