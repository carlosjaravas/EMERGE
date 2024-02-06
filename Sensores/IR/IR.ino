const int pinIR = A0;
const float VCC = 5.0;

void setup() {
  Serial.begin(250000);  // Inicia la comunicación serial a 9600 baudios
}

void loop() {
  // Lee la entrada serial y realiza alguna acción
  if (Serial.available() > 0) {
    String comando = Serial.readString();
    
    // Si se recibe 'IR', lee la entrada analógica y envía el valor
    if (comando == 'IR') {
      int IR_read = analogRead(pinIR);
      float dist_mV = IR_read * (VCC/1023.0);
      float dist = 49.745*pow(dist_mV,4) - 304.1*pow(dist_mV,3) + 701.24*pow(dist_mV,2) - 752.48*dist_mV + 357.17;
      Serial.println(dist);
    }

    else if (comando == 'Gyro') {
      int IR_read = analogRead(pinIR);
      float dist_mV = IR_read * (VCC/1023.0);
      Serial.println(dist_mV);
    }
  }
}