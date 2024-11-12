const int INPUT_PIN = A0;
const int OUTPUT_PIN = 9;

double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;
int req_voltage = 8;
int r1 = 390;
int r2 = 120;
double sd_voltage = req_voltage * (r2/(r1 + r2));
double setpoint = sd_voltage * (1023/5);

void setup()
{ 
  pinMode(OUTPUT_PIN, OUTPUT); // Pin 9 is associated with Timer1
  TCCR1A = 0; 
  TCCR1B = 0;
  OCR1A = 200; // TOP value for 15kHz
  TCCR1A |= (1 << WGM11) | (1 << COM1A1);  // Fast PWM, non-inverting mode
  TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11); // Fast PWM, prescaler 8
  ICR1 = 132; // Set the TOP value to OCR1A
  kp = 0.0171063017142104;
  ki = 0.75176340731197;
  kd = 0;
  last_time = 0;
  Serial.begin(9600);
 
  for(int i = 0; i < 50; i++)
  {
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(0);
    delay(100);
  }
  delay(100);
}

void loop()
{
  double now = millis();
  dt = (now - last_time)/1000.00;
  last_time = now;

  double actual = analogRead(INPUT_PIN);
  double error = setpoint - actual;
  output = pid(error);
  double f_output = constrain(output, 2 ,253);
  double ff_output = map(f_output, 2, 253, 0, 1 );

  OCR1A = ICR1 * ff_output; 

  // Setpoint VS Actual
  Serial.print(setpoint);
  Serial.print(",");
  Serial.println(actual);

  // Error
  //Serial.println(error);

  delay(150);
}

double pid(double error)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}