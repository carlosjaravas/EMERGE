const int pinEntradaAnalogica = A0;

void setup() {
  Serial.begin(9600);  // Inicia la comunicación serial a 9600 baudios
}

void loop() {
  // Lee la entrada serial y realiza alguna acción
  if (Serial.available() > 0) {
    char comando = Serial.read();
    
    // Si se recibe 'A', lee la entrada analógica y envía el valor
    if (comando == 'A') {
      int valorAnalogico = analogRead(pinEntradaAnalogica);
      Serial.println(valorAnalogico);
    }
  }

  // Tu código Arduino normal aquí
}