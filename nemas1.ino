const int dirPin = 5;
const int stepPin = 4;
 
const int steps = 200;
int stepDelay;
 
void setup() {
   // Marcar los pines como salida
   pinMode(dirPin, OUTPUT);
   pinMode(stepPin, OUTPUT);
}
 
void loop() {
   //Activar una direccion y fijar la velocidad con stepDelay
   digitalWrite(dirPin, HIGH);
   stepDelay = 600; //
   // Giramos 200 pulsos para hacer una vuelta completa
   for (int x = 0; x < 400; x++) { //aqui esta dando solo 200 una vuelta y 70 para patear 
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepDelay);
   }
   delay(200);
 
   //Cambiamos la direccion y aumentamos la velocidad
   digitalWrite(dirPin, LOW);
   stepDelay = 600; //250
   // Giramos 400 pulsos para hacer dos vueltas completas
   for (int x = 0; x < 400; x++) { //aqui esta dando 200 una vuelta 
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepDelay);
   }
   delay(200);
}
