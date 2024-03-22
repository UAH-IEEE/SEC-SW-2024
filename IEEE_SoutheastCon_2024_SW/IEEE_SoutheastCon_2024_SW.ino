#include <avr/wdt.h>

#define DEBUG true // Debug Mode optional, allows full send of real time data streams


#define PPG_ANALOG_INPUT_PIN 0                // PPG Signal Pin Number
#define VIBRATION_SENSOR_ANALOG_INPUT_PIN 1   // Vibration Sensor Pin Number
#define BUZZER_DIGITAL_OUTPUT_PIN 7           // Buzzer Signal Pin Number
#define STOPLIGHT_DIGITAL_OUTPUT_GREEN_PIN 13
#define STOPLIGHT_DIGITAL_OUTPUT_YELLOW_PIN 12
#define STOPLIGHT_DIGITAL_OUTPUT_RED_PIN 11

// Length of history Buffers
#define VIBRATION_HIST_LENGTH 20
#define PPG_HIST_LENGTH 5
//Interbeat Interval Length
#define IBI_HIST_LENGTH 5
#define HB_THRESHOLD 3
// 335 Milliseconds for Threshold
#define IBI_HARD_THRESHOLD 335 



// FN Prototypes
float variance(int a[], int n); // Calculates Variance of an array
int totalGain(int a[], int n, int Tail); // Calculates the accumulative rise in a time series of data
int average(int a[], int n); // Calculates average of array

bool NEW_DATA = false;               // Indicate when new Data is available

int NEXT_PPG_SAMPLE = 0;             // Storage for new PPG module serial data
int FILTERED_NEXT_PPG = 0;           // Storage for PPG serial data after filtering
int PPG_HISTORY[PPG_HIST_LENGTH];     // History for PPG signal to extract the Interbeat Intervals
unsigned char PPG_INDEX = 0;        
int DPG_CURRENT = 0;

float CURRENT_HEART_RATE = 0;
int IBI_HIST[IBI_HIST_LENGTH];      // History of Interbeat Intervals used to calculate Heart Rate
int IBI_INDEX = 0;

int NEXT_VIBRATION_SENSOR_SAMPLE = 0;
int VIBRATION_HISTORY[VIBRATION_HIST_LENGTH];   // History of Vibrations to detect tremors
unsigned char VIBRATION_INDEX = 0;
int VIBRATION_COUNT = 0;

// For Calibration Implementation
int WALK_TREMOR_THRESHOLD = 700; // From Testing
float ABNORMAL_HEART_RATE_THRESHOLD = 90.0; // Typical Abnormally high heart rate

// Stoplight Flag
bool STOPLIGHT_ENABLE = false;

// Real Time Track
int T = 0;
int T_ms = 0; 
int HeartTime = 0; // Milliseconds between Heartbeats
#define MS_PER_DATA_SAMPLE 5

// State Tracking
char state = 'F';  // F for Fine, T for Tremor, P for Predict Tremor.
char nextState = 'F';

void setup() {
  wdt_enable(WDTO_2S); // Enable 2 second Watchdog timer for our main loop
  // set timer1 to 200 Hz
  TCCR1A = 0; // set TCCR2A register to 0
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
  Serial.begin(115200);
  Serial.println("INITIALIZING!!!");

  // Setting up digital output ports
  pinMode(BUZZER_DIGITAL_OUTPUT_PIN, OUTPUT);
  pinMode(STOPLIGHT_DIGITAL_OUTPUT_GREEN_PIN, OUTPUT);
  pinMode(STOPLIGHT_DIGITAL_OUTPUT_YELLOW_PIN, OUTPUT);
  pinMode(STOPLIGHT_DIGITAL_OUTPUT_RED_PIN, OUTPUT);
}



void loop() {
  if (NEW_DATA) {
    NEW_DATA = false;
    FILTERED_NEXT_PPG = FILTERED_NEXT_PPG * 0.85 + NEXT_PPG_SAMPLE * 0.15;  // implements crude low pass filter
    PPG_HISTORY[PPG_INDEX] = FILTERED_NEXT_PPG;
    // DPG_CURRENT = variance(PPG_HISTORY, PPG_HIST_LENGTH);
    DPG_CURRENT = totalGain(PPG_HISTORY, PPG_HIST_LENGTH, PPG_INDEX); // see new Gain from PPG

    // Keep array of Heartbeats
    if (DPG_CURRENT >= HB_THRESHOLD) { // Threshold Found through testing
      if ((HeartTime > IBI_HARD_THRESHOLD)) {
        IBI_HIST[IBI_INDEX] = HeartTime; // Record Heartbeat
        IBI_INDEX +=1;
        if (IBI_INDEX >= IBI_HIST_LENGTH) { // Reset Index
          IBI_INDEX = 0;
        }
        HeartTime = 0; // Reset IBI Time

        float avgHR = 60000.0 / ((float) average(IBI_HIST, IBI_HIST_LENGTH));
        if (avgHR > ABNORMAL_HEART_RATE_THRESHOLD) {
          nextState = 'P'; // Predict Gait Freeze
        } else if (state != 'T') {
          nextState = 'F'; // Otherwise we are fine (At least until Tremor Check)
        }
        if (DEBUG) {
          Serial.print(">HR:");
          Serial.println(avgHR);
        }
        
      }
      
    }

    PPG_INDEX +=1;
    if (PPG_INDEX >= PPG_HIST_LENGTH) {
      PPG_INDEX = 0; // Reset Index for cyclic Buffer
    }
    
    if (DEBUG) {
      Serial.print(">DPG:");
      Serial.println(DPG_CURRENT);

      Serial.print(">PPG:");
      Serial.println(FILTERED_NEXT_PPG);  // prints PPG serial for plotting
      
      Serial.print(">VS:");
      Serial.println(NEXT_VIBRATION_SENSOR_SAMPLE);
    }
  }



  // Calculate variance of Vibrations to detect state of Patient
  if (VIBRATION_COUNT >= VIBRATION_HIST_LENGTH) {
    VIBRATION_COUNT = 0;
    float var = variance(VIBRATION_HISTORY, VIBRATION_HIST_LENGTH);
    if (var >  WALK_TREMOR_THRESHOLD) {
      nextState = 'T';
    } else if (nextState != 'P'){ // no tremors, no predict
      nextState = 'F';            // all good.
    }
    // Serial.print(">VAR:");
    // Serial.println(var);
  }



  // State updates
  if (state != nextState) {
    state = nextState;
    switch (state) {
    case 'F':
      // State of well being, No Buzzer, no Extra Light stimulation.
      digitalWrite(BUZZER_DIGITAL_OUTPUT_PIN, LOW);
      STOPLIGHT_ENABLE = false;
      break;
    case 'T':
      // State of Tremor, Full Blast Stimulation with Buzzer and Stoplight shifting
      digitalWrite(BUZZER_DIGITAL_OUTPUT_PIN, HIGH);
      STOPLIGHT_ENABLE = true;
      break;
    case 'P':
      // State of Anticipated Tremor, (Spike in Heart Rate), Stoplight Shifting to help with extra stimulation.
      digitalWrite(BUZZER_DIGITAL_OUTPUT_PIN, LOW);
      STOPLIGHT_ENABLE = true;
      break;
    default: // ??? State, Force Reset by timing out WDT
      // Two Second Watchdog Timer Timeout still provides adequate Real Time responsiveness for this application.
      while (true) {
      // wait on WDT
      }
      break;
    }
  }


  // Don't Get Bit
  wdt_reset();

}

/*
  ISR generated by Timer 1
  Timer 1 timeouts for a 200 Hz pulse
  Reads PPG analog input
*/
ISR(TIMER1_COMPA_vect) {
  // Interrupt performs Data collection, as well as movement of LEDs of stoplight
  NEW_DATA = true;

  // Real Time Data Tracking
  T_ms +=MS_PER_DATA_SAMPLE;
  HeartTime += MS_PER_DATA_SAMPLE;
  if (T_ms >= 1000) {
    T +=1;
    T_ms -=1000;
  }


  VIBRATION_COUNT+=1;
  NEXT_PPG_SAMPLE = analogRead(PPG_ANALOG_INPUT_PIN);
  NEXT_VIBRATION_SENSOR_SAMPLE = analogRead(VIBRATION_SENSOR_ANALOG_INPUT_PIN);
  VIBRATION_HISTORY[VIBRATION_INDEX] = NEXT_VIBRATION_SENSOR_SAMPLE;
  VIBRATION_INDEX +=1;
  if (VIBRATION_INDEX == VIBRATION_HIST_LENGTH) { // Cyclic Buffer for Vibration History
    VIBRATION_INDEX = 0;
  }

  static byte sl_state = 0;
  static int last_T = 0;
  if (last_T != T) {
    last_T = T;
    if (STOPLIGHT_ENABLE) {
      // Do Stoplight State Stuff
      switch (sl_state) {
      case (0) :
        sl_state+=1;
        digitalWrite(STOPLIGHT_DIGITAL_OUTPUT_GREEN_PIN, HIGH);
        digitalWrite(STOPLIGHT_DIGITAL_OUTPUT_YELLOW_PIN, LOW);
        digitalWrite(STOPLIGHT_DIGITAL_OUTPUT_RED_PIN, LOW);
        break;
      case (1):
        sl_state+=1;
        digitalWrite(STOPLIGHT_DIGITAL_OUTPUT_GREEN_PIN, LOW);
        digitalWrite(STOPLIGHT_DIGITAL_OUTPUT_YELLOW_PIN, HIGH);
        digitalWrite(STOPLIGHT_DIGITAL_OUTPUT_RED_PIN, LOW);
        break;
      case (2):
        sl_state = 0;
        digitalWrite(STOPLIGHT_DIGITAL_OUTPUT_GREEN_PIN, LOW);
        digitalWrite(STOPLIGHT_DIGITAL_OUTPUT_YELLOW_PIN, LOW);
        digitalWrite(STOPLIGHT_DIGITAL_OUTPUT_RED_PIN, HIGH);
        break;
      default:
        sl_state = 0;
        break;
      }

    } else {
      // Turn Off Stoplight
      sl_state = 0;
      digitalWrite(STOPLIGHT_DIGITAL_OUTPUT_GREEN_PIN, LOW);
      digitalWrite(STOPLIGHT_DIGITAL_OUTPUT_YELLOW_PIN, LOW);
      digitalWrite(STOPLIGHT_DIGITAL_OUTPUT_RED_PIN, LOW);
    }
  }
}

float variance(int a[], int n) 
{   
    // Compute mean (average of elements) 
    double sum = 0; 
    
    for (int i = 0; i < n; i++) sum +=  (double) a[i];    
    double mean = (double)sum / (double)n; 
    // Compute sum squared differences with mean. 
    double sqDiff = 0; 
    for (int i = 0; i < n; i++) 
        sqDiff += ((double)a[i] - mean) * ((double)a[i] - mean); 
    return (float)sqDiff / n; 
} 

int totalGain(int a[], int n, int Tail) {

  int i = 0;
  int tail = Tail; // Make a copy to use
  int before_tail = Tail - 1;
  

  int sum = 0;

  for (i=0; i < (n-2); i++) { // Sum of differences of circular Buffer.
    if (tail <= 0) {
      tail = n-1;
    }
    if (before_tail <= 0) {
      before_tail = n-1;
    }
    
    sum += (a[tail] - a[before_tail]) ;
    tail-=1;
    before_tail-=1;
  }
  return sum;
}

int average(int a[], int n) {
  int sum = 0;
  int i = 0;
  for (i = 0; i < n; i++) {
    sum += a[i];
  }
  return (sum/n);
}