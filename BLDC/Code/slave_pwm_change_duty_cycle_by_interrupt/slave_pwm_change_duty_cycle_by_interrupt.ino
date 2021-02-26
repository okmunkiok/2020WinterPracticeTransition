// common to master
#define IF_PRINT_SERIAL_ON_MONITOR

#define PIN_RX_SERIAL (16)
#define PIN_TX_SERIAL (17)
#define PIN_INTERRUPT_BETWEEN_ESP32 (2)
//

#define PIN_PWM (4)
// common to master
unsigned int index_DUTY_CYCLE_ARRAY = 0;
#define MAX_PWM (80)
#define MIN_PWM (0)
//

unsigned int frequency[] = {400, 500, 600};
unsigned int period_us[] = {0, 0, 0};
unsigned int index_freq = 1;
unsigned int duty[MAX_PWM - MIN_PWM + 1];
unsigned int run_us[MAX_PWM - MIN_PWM + 1];
unsigned int rest_us[MAX_PWM - MIN_PWM + 1];

// y = m * x + n
// m = SLOPE
// n = Y_INTERSECT
#define SLOPE (0.0078)
#define Y_INTERSECT (6.0397)

#define UNIT_TIME_PWM_MICROS (10000000)
#define TIME_0_100_MICROS (3000000)
#define INDEX_ACCELERATION (1)
#define INDEX_KEEP_STABLE (1)
#define INDEX_DECELERATION (0)


//int period_us = (int) ((double) 1 / frequency * 1000 * 1000);
//int period_us = 5*1000*1000;
//int 
//int duty = 10;
//int duty[] = {0, 10, 20, 30, 40, 50, 60, 70, 80};


//int run_us = (int) ((double) period_us * duty / 100);
//int rest_us = (int) ((double) period_us * (100 - duty) / 100);
//int one_time = 210;
//int target_RPM_on_display = 2300;
//int target_duty_index = 0;
  
void setup()
{
  #ifdef IF_PRINT_SERIAL_ON_MONITOR
  Serial.begin(115200);
  #endif
  Serial1.begin(115200, SERIAL_8N1, PIN_RX_SERIAL, PIN_TX_SERIAL);
  pinMode(PIN_INTERRUPT_BETWEEN_ESP32, INPUT_PULLUP);
  pinMode(PIN_PWM, OUTPUT);
  
  for(int i = 0; i < sizeof(period_us) / sizeof(period_us[0]); i++){
    period_us[i] = (int) ((double) 1 / frequency[i] * 1000 * 1000);
  }
  for(int i = 0; i < sizeof(duty) / sizeof(duty[0]); i++){
    duty[i] = MIN_PWM + i;
  }
  for(int i = 0; i < sizeof(run_us) / sizeof(run_us[0]); i++){
    run_us[i] = (int) ((double) period_us[index_freq] * duty[i] / 100);  
    rest_us[i] = (int) ((double) period_us[index_freq] * (100 - duty[i]) / 100);
  }

//  target_duty_index = (SLOPE * target_RPM_on_display + Y_INTERSECT) - MIN_PWM;
  
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT_BETWEEN_ESP32), set_duty_cycle_by_interrupt, CHANGE);
//  index_DUTY_CYCLE_ARRAY = 10;
}

void loop()
{
  digitalWrite(PIN_PWM, HIGH);
  delayMicroseconds(run_us[index_DUTY_CYCLE_ARRAY]);
  digitalWrite(PIN_PWM, LOW);
  delayMicroseconds(rest_us[index_DUTY_CYCLE_ARRAY]);
}

void set_duty_cycle_by_interrupt(){
  if(Serial1.available()){
    index_DUTY_CYCLE_ARRAY = Serial1.read();
    #ifdef IF_PRINT_SERIAL_ON_MONITOR
    Serial.print("index_DUTY_CYCLE_ARRAY was set \"");
    Serial.print(index_DUTY_CYCLE_ARRAY);
    Serial.println("\" by interrupt");
    #endif
  }
}
