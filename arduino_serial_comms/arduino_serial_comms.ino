//define combined motor frequency
#define TOTAL_FREQUENCY 50

//class for motors
//contains pins, step counter, frequency, and milliseconds since last step (wait)
//functions for setting direction, number of steps, and updating the frequency
class Motor {
  public:
    long step_count;
    float frequency;
    float wait;
    int step_pin;
    int dir_pin;
    int enable_pin;
    
    Motor(int a, int b, int c) {
      step_pin = a;
      dir_pin = b;
      enable_pin = c;
      step_count = 0.0;
      frequency = 0.0;
      wait = 0.0;
      pinMode(step_pin, OUTPUT);
      pinMode(dir_pin, OUTPUT); 
      pinMode(enable_pin, OUTPUT);
      digitalWrite(dir_pin, HIGH);
      digitalWrite(enable_pin, HIGH);
    }

    void SetDirection(char dir) {
      switch(dir){
        case '+': 
          digitalWrite(dir_pin, HIGH);
          break;
        case '-': 
          digitalWrite(dir_pin, LOW);
          break;
        default:
          break;  //error
      }
      /*
      Serial.print("dir: ");
      Serial.println(dir);
      */
    }

    void SetSteps(long steps) {
      step_count = steps;
      /*
      Serial.print("steps: ");
      Serial.println(steps);
      */
    }

    void UpdateFrequency(long step_count_2) {
      float total_distance = sqrt(pow(step_count, 2) + pow(step_count_2, 2));
      frequency = (step_count / total_distance) * TOTAL_FREQUENCY;
    }
};

//create two motor objects
Motor az(11, 10, 9);
Motor elv(7, 6, 5);

//create a char array to store incoming commands
const byte cmd_len = 32;
char cmd[cmd_len];

boolean new_data = false;

void setup() {
    //begin serial communication
    Serial.begin(9600);
    Serial.println("Arduino is ready");
}

void loop() { 
  //if no command has been recieved
  while(!Serial.available()){
    /*
    Serial.print(az.step_count);
    Serial.print(" ");
    Serial.println(elv.step_count);
    */

    //check if there are steps left
    if(az.step_count > 0 && az.wait == 0){
      digitalWrite(az.step_pin, HIGH);
      az.wait = millis();
      Serial.println("az HIGH");
    }
    if(elv.step_count > 0 && elv.wait == 0){
      digitalWrite(elv.step_pin, HIGH);
      elv.wait = millis();
      Serial.println("elv HIGH");
    }

    //decrement counter if motor has stepped
    if(az.step_count > 0 && (millis() - az.wait) >= (1000/az.frequency)){
      digitalWrite(az.step_pin, LOW);
      az.step_count--;
      az.wait = 0;
      Serial.println("az LOW");
    }
    
    if(elv.step_count > 0 && (millis() - elv.wait) >= (1000/elv.frequency)){
      digitalWrite(elv.step_pin, LOW);
      elv.step_count--;
      elv.wait = 0;
      Serial.println("elv LOW");
    }
  }

  //exit while loop if a command has been recieved
  recv();
  if (new_data == true) {
    Serial.println("message recieved");
    show_new_data();
    //check which motor is specified in the second character of the command
    //update the step counter with the integer value in the tail of the command
    //update the direction of the motor based on the first character of the command
    /*
    Serial.print("motor: ");
    Serial.println(cmd[0]);
    */
    switch(cmd[0]){
      case 'a':
        az.SetDirection(cmd[1]);
        az.SetSteps(strtol(&cmd[2], NULL, 10));
        break;
      case 'b':
        elv.SetDirection(cmd[1]);
        elv.SetSteps(strtol(&cmd[2], NULL, 10));
        break;
      default: break;//error
    }
     update_frequencies(&az, &elv);
     //Serial.println("done.");
  }
  new_data = false;
}

void update_frequencies(Motor* az, Motor* elv) {
  //must update twice since the changed motor needs to update before the unchanged motor
  az->UpdateFrequency(elv->step_count);
  elv->UpdateFrequency(az->step_count);
  az->UpdateFrequency(elv->step_count);
  elv->UpdateFrequency(az->step_count);

  /*
  Serial.println("updating frequencies...");
  Serial.print(az->frequency);
  Serial.print(" ");
  Serial.println(elv->frequency);
  Serial.println("updated.");
  */
}

void recv() {
    static boolean recv_in_progress = false;
    static byte i = 0;
    char start_marker = '<';
    char end_marker = '>';
    char new_char;
 
    while (Serial.available() > 0 && new_data == false) {
        new_char = Serial.read();

        if (recv_in_progress == true) {
            if (new_char != end_marker) {
                cmd[i] = new_char;
                i++;
                if (i >= cmd_len) {
                    i = cmd_len - 1;
                }
            }
            else {
                cmd[i] = '\0'; // terminate the string
                recv_in_progress = false;
                i = 0;
                new_data = true;
            }
        }

        else if (new_char == start_marker) {
            recv_in_progress = true;
        }
    }
}

void show_new_data() {
  Serial.print("command: ");
  Serial.println(cmd);
}
