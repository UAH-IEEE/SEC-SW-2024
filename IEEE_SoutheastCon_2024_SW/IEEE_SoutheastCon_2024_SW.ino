void setup() {
  // set timer to 200 Hz

  TCCR1A = 0; // set entire TCCR2A register to 0
  TCCR1B = 0; // same for TCCR2B
  TCNT1  = 0; //initialize counter value to 0
  // set compare match register for 200 Hz increments
  OCR1A = 77; // 77.125 = (16*10^6) / (200*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  // starting serial connection with 115200 baud
  serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

}


ISR(TIMER1_COMPA_vect) {
  //timer1 interrupt 1Hz toggles pin 13 (LED)
  //generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  static char p = 0;
  p++;
  switch (p) {
    case 1: 
      digitalWrite(13,LOW);
      digitalWrite(12,LOW);
      digitalWrite(11,HIGH);
      break;
    case 2:
      digitalWrite(13,LOW);
      digitalWrite(12,HIGH);
      digitalWrite(11,LOW);
      break;
    case 3:
      digitalWrite(13,HIGH);
      digitalWrite(12,LOW);
      digitalWrite(11,LOW);
      p = 0;
      break;
    default:
    break;
  }
}