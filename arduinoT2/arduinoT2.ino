
int mR1 = 2;
int mR2 = 3; 
int mL1 = 4;
int mL2 = 5;
int pwmR = 6;
int pwmL = 7;

float x, y;

void setup() {
  Serial.begin(250000);
  Serial.setTimeout(1);

  pinMode(mR1, OUTPUT);
  pinMode(mR2, OUTPUT);
  pinMode(mL1, OUTPUT);
  pinMode(mL2, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(pwmL, OUTPUT);
}

void loop() {
      
    String input = Serial.readString();
    input.trim();
    int commaIndex = input.indexOf(',');
    if (commaIndex >= 0) {
       String str1 = input.substring(0, commaIndex);
       String str2 = input.substring(commaIndex + 1);
       x = str1.toFloat();
       y = str2.toFloat();
    }

      if (x < 0) {
        digitalWrite(mL1, HIGH);
        digitalWrite(mL2, LOW);
        digitalWrite(pwmL, 100);

        digitalWrite(mR1, HIGH);
        digitalWrite(mR2, LOW);
        digitalWrite(pwmR, 100);
        
      }
      else if (x>0){
        digitalWrite(mL2, HIGH);
        digitalWrite(mL1, LOW);
        digitalWrite(pwmL, 100);

        digitalWrite(mR2, HIGH);
        digitalWrite(mR1, LOW);
        digitalWrite(pwmR, 100);
      }

      else if (y < 0){
        digitalWrite(mL2, HIGH);
        digitalWrite(mL1, LOW);
        digitalWrite(pwmL, 100);

        digitalWrite(mR2, LOW);
        digitalWrite(mR1, HIGH);
        digitalWrite(pwmR, 70);
       }

       else if (y > 0){
        digitalWrite(mL2, LOW);
        digitalWrite(mL1, HIGH);
        digitalWrite(pwmL, 100);

        digitalWrite(mR2, HIGH);
        digitalWrite(mR1, LOW);
        digitalWrite(pwmR, 70);
       }
      else if (x==0 || y==0){
        digitalWrite(mL1, LOW);
        digitalWrite(mL2, LOW);

        digitalWrite(mR1, LOW);
        digitalWrite(mR2, LOW);
      }

      
}
