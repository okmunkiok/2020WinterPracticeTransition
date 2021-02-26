// common to slave
#define IF_PRINT_SERIAL_ON_MONITOR

#define PIN_DIGITAL_HALL_A (4)
#define PIN_DIGITAL_HALL_B (33)
#define PIN_DIGITAL_HALL_C (12)
// common to slave
#define PIN_RX_SERIAL (16)
#define PIN_TX_SERIAL (17)
#define PIN_INTERRUPT_BETWEEN_ESP32 (2)
//

#define UNIT_DEGREE (15)
#define UNIT_LOOP_TIME_MILLIS (1000)
//#define UNIT_LOOP_TIME_MILLIS (10)

bool hallA = 0;
bool hallB = 0;
bool hallC = 0;
uint32_t deg_CCW = 0;
uint32_t deg_CW = 0;
uint32_t deg_CCW_before_UNIT_LOOP_TIME_MILLIS = 0;
uint32_t deg_CW_before_UNIT_LOOP_TIME_MILLIS = 0;
uint32_t deg_CCW_delta = 0;
uint32_t deg_CW_delta = 0;
uint32_t time_stamp = 0;
uint32_t angular_v_CCW = 0;
uint32_t angular_v_CW = 0;
uint32_t time_stamp_for_calc_if = 0;
uint32_t freq_UNIT_LOOP_TIME_MILLIS = 0;
uint32_t period_UNIT_LOOP_TIME_MILLIS = 0;

TaskHandle_t Task1;
TaskHandle_t Task2;

uint32_t tmp_angular_velocity = 0;
uint32_t tmp_angular_distance = 0;
uint32_t target_angular_velocity = 0;
uint32_t target_angular_distance = 0;
bool input_complete = 0;
uint32_t tmp_input = 0;
bool change_target_velocity_or_distance = 0;

// common to slave
unsigned int index_DUTY_CYCLE_ARRAY = 0;
#define MAX_PWM (80)
#define MIN_PWM (10)
//
bool output_state_for_interrupt = 0;
#define buffer_DISTANCE (0)
#define DELAY_BETWEEN_INTERRUPT (500)

void setup() {
  #ifdef IF_PRINT_SERIAL_ON_MONITOR
  Serial.begin(115200);
  #endif
//  Serial.begin(115200);
  
  Serial1.begin(115200, SERIAL_8N1, PIN_RX_SERIAL, PIN_TX_SERIAL);
  pinMode(PIN_INTERRUPT_BETWEEN_ESP32, OUTPUT);
  digitalWrite(PIN_INTERRUPT_BETWEEN_ESP32, output_state_for_interrupt);
  
  pinMode(PIN_DIGITAL_HALL_A, INPUT_PULLUP);
  pinMode(PIN_DIGITAL_HALL_B, INPUT_PULLUP);
  pinMode(PIN_DIGITAL_HALL_C, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_HALL_A), hallA_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_HALL_B), hallB_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_HALL_C), hallC_change, CHANGE);

  freq_UNIT_LOOP_TIME_MILLIS = (uint32_t) 1000 / UNIT_LOOP_TIME_MILLIS;
  period_UNIT_LOOP_TIME_MILLIS = (uint32_t) UNIT_LOOP_TIME_MILLIS / 1000;
  
  xTaskCreatePinnedToCore(
                  Task1code,   /* Task function. */
                  "Task1",     /* name of task. */
                  10000,       /* Stack size of task */
                  NULL,        /* parameter of the task */
                  1,           /* priority of the task */
                  &Task1,      /* Task handle to keep track of created task */
                  0);          /* pin task to core 0 */                  
  delay(500); 
}

void Task1code( void * pvParameters ){
  for(;;){
    if(Serial.available()){
      tmp_input = Serial.parseInt();
      if(!change_target_velocity_or_distance){
        target_angular_velocity = tmp_input;
        change_target_velocity_or_distance = !change_target_velocity_or_distance;
      }
      else{
        target_angular_distance = tmp_input;
        change_target_velocity_or_distance = !change_target_velocity_or_distance;
      }
//      if(tmp_input < 0){
//        target_angular_distance = abs(tmp_input);
//      }
//      else{
//        target_angular_velocity = tmp_input;
////        tmp_angular_velocity = tmp_input;
////        if(tmp_angular_distance != 0)
////          input_complete = 1;
////        else
////          input_complete = 0;
//      }
    }
//    if(input_complete){
//      target_angular_velocity = tmp_angular_velocity;
//      target_angular_distance = tmp_angular_distance;
//      tmp_angular_velocity = 0;
//      tmp_angular_distance = 0;
//      input_complete = 0;
//    }

//    Serial.print("target_angular_velocity = ");
//    Serial.println(target_angular_velocity);
//    Serial.print("target_angular_distance = ");
//    Serial.println(target_angular_distance);
//    Serial.println("------------------------------------");

//    if(deg_CCW - target_angular_distance > buffer_DISTANCE){
//      if(index_DUTY_CYCLE_ARRAY != 0){
//        #ifdef IF_PRINT_SERIAL_ON_MONITOR
//        Serial.print("deg_CCW - target_angular_distance = ");
//        Serial.println(deg_CCW - target_angular_distance);
//        Serial.println("stop");
//        #endif
//        index_DUTY_CYCLE_ARRAY = 0;
//        write_pwm_interrupt();
//        delay(100);
//      }
//    }

    if(target_angular_velocity > angular_v_CCW){
      #ifdef IF_PRINT_SERIAL_ON_MONITOR
//      Serial.println("accel");
      #endif
      if(index_DUTY_CYCLE_ARRAY >= 0){
        index_DUTY_CYCLE_ARRAY = (index_DUTY_CYCLE_ARRAY + 1) % (MAX_PWM - MIN_PWM);
        write_pwm_interrupt();
        delay(DELAY_BETWEEN_INTERRUPT);
      }
    }
    else{
      if(index_DUTY_CYCLE_ARRAY > 0){
        #ifdef IF_PRINT_SERIAL_ON_MONITOR 
//        Serial.println("decel");
        #endif
        index_DUTY_CYCLE_ARRAY--;
        write_pwm_interrupt();
        delay(DELAY_BETWEEN_INTERRUPT);
      }
    }
    delay(1);
  }
}

void loop() {
  if(millis() - time_stamp > UNIT_LOOP_TIME_MILLIS){
    time_stamp_for_calc_if = micros();

    #ifdef IF_PRINT_SERIAL_ON_MONITOR
    Serial.print("deg_CCW: ");
    Serial.println(deg_CCW);
    Serial.print("deg_CW: ");
    Serial.println(deg_CW);
    #endif

    #ifdef IF_PRINT_SERIAL_ON_MONITOR
    Serial.print("angular_v_CCW: ");
    #endif
    angular_v_CCW = (deg_CCW - deg_CCW_before_UNIT_LOOP_TIME_MILLIS) / period_UNIT_LOOP_TIME_MILLIS;
//    angular_v_CCW = (deg_CCW - deg_CCW_before_UNIT_LOOP_TIME_MILLIS) * freq_UNIT_LOOP_TIME_MILLIS;
    #ifdef IF_PRINT_SERIAL_ON_MONITOR
    Serial.println(angular_v_CCW);
    Serial.print("angular_v_CW: ");
    #endif
    angular_v_CW = (deg_CW - deg_CW_before_UNIT_LOOP_TIME_MILLIS) / period_UNIT_LOOP_TIME_MILLIS;
//    angular_v_CW = (deg_CW - deg_CW_before_UNIT_LOOP_TIME_MILLIS) * freq_UNIT_LOOP_TIME_MILLIS;
    #ifdef IF_PRINT_SERIAL_ON_MONITOR
    Serial.println(angular_v_CW);
    #endif

    #ifdef IF_PRINT_SERIAL_ON_MONITOR
    Serial.print("elapsed time in if (micros): ");
    Serial.println(micros() - time_stamp_for_calc_if);
    Serial.println("");
    #endif

    deg_CCW_before_UNIT_LOOP_TIME_MILLIS = deg_CCW;
    deg_CW_before_UNIT_LOOP_TIME_MILLIS = deg_CW;


    #ifdef IF_PRINT_SERIAL_ON_MONITOR
    Serial.print("target_angular_velocity = ");
    Serial.println(target_angular_velocity);
    Serial.print("target_angular_distance = ");
    Serial.println(target_angular_distance);
    Serial.println("------------------------------------");
    #endif
    
    time_stamp = millis();
  }
}

void hallA_change(){
  hallA = 1;
  deg_CCW += hallB * UNIT_DEGREE;
  deg_CW += hallC * UNIT_DEGREE;
  hallB = 0;
  hallC = 0;
}

void hallB_change(){
  hallB = 1;
  deg_CCW += hallC * UNIT_DEGREE;
  deg_CW += hallA * UNIT_DEGREE;
  hallC = 0;
  hallA = 0;
}

void hallC_change(){
  hallC = 1;
  deg_CCW += hallA * UNIT_DEGREE;
  deg_CW += hallB * UNIT_DEGREE;
  hallA = 0;
  hallB = 0;
}

void accelerate_interrupt(){
  if(index_DUTY_CYCLE_ARRAY < (MAX_PWM - MIN_PWM)){
    index_DUTY_CYCLE_ARRAY++;
  }
  else
    return;
  Serial1.write(index_DUTY_CYCLE_ARRAY);
  output_state_for_interrupt = !output_state_for_interrupt;
  digitalWrite(PIN_INTERRUPT_BETWEEN_ESP32, output_state_for_interrupt);
}

void decelerate_interrupt(){
  if(index_DUTY_CYCLE_ARRAY > 0){
    index_DUTY_CYCLE_ARRAY--;
  }
  else
    return;
  Serial1.write(index_DUTY_CYCLE_ARRAY);
  output_state_for_interrupt = !output_state_for_interrupt;
  digitalWrite(PIN_INTERRUPT_BETWEEN_ESP32, output_state_for_interrupt);
}

void stop_interrupt(){
  index_DUTY_CYCLE_ARRAY = 0;
}

void write_pwm_interrupt(){
  Serial1.write(index_DUTY_CYCLE_ARRAY);
  output_state_for_interrupt = !output_state_for_interrupt;
  digitalWrite(PIN_INTERRUPT_BETWEEN_ESP32, output_state_for_interrupt);
}
