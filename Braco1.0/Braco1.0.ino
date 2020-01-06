#include <Servo.h>
#include <PS2X_lib.h>
PS2X ps2x;
Servo Base;
Servo Braco1;
Servo Braco2;
Servo AntiBraco;
Servo Pulso;
Servo Garra;
int error = 0;
byte type = 0; 
int posPulso = 90;
int posBase = 90;
int posBraco = 90;
int posAntiBraco = 50;
int GarraAntes;
int GarraAtual;
const int Delay = 10; // ms delay for servo motions
int VELOC = 6 ; // Velocidade 
int VELOCBRACO = 3 ; // Velocidade Braco

int vibrate;


void setup() {
  // put your setup code here, to run once:
  
  
  error = ps2x.config_gamepad(13,11,10,12, true, true);  // Define os pinos onde está conectado o controle.
  type = ps2x.readType();
  Serial.begin(57600);
  Serial.println("BRACO ROBOTICO 1.0 - CONTROLE REMOTO PS2");
  Serial.println("CONTROLES:");

  Base.attach(2);
  Braco1.attach(22);
  Braco2.attach(24);  
  AntiBraco.attach(5);
  Pulso.attach(6);
  Garra.attach(7);

  GarraAntes = ps2x.Analog(PSS_RX);  
}

void loop() {
  // put your main code here, to run repeatedly:

 
 if(error == 1) //skip loop if no controller found
 { 
   Serial.println("No Controller Found,check wiring");
   return;
 }
  ps2x.read_gamepad(false, vibrate);

  if (ps2x.Analog(PSS_RX)>130){
  vibrate = ps2x.Analog(PSS_RX);
  } else { vibrate = 0; }

// GARRA
  if (ps2x.Analog(PSS_RX)>=127) {
  Garra.write(map(ps2x.Analog(PSS_RX), 0, 255, 20, 148));
  GarraAtual = ps2x.Analog(PSS_RX);
  if (GarraAtual == 255 && ps2x.Button(PSB_R1) == false ) {
      Serial.println ("GARRA: Aberta");
    }
  if (GarraAtual <= 200 && GarraAntes == 255 && ps2x.Button(PSB_R1) == false) {
      Serial.println ("GARRA: Fechada");
    }
  GarraAntes = GarraAtual;  
  }

//Pulso
  if (ps2x.Button(PSB_L2)){  // DESCENDO 
        posPulso += VELOC; 
        if (posPulso>=179) {posPulso = 179;}
        Pulso.write(posPulso); 
        delay(Delay);    
        Serial.println ("PULSO: Descendo ");   
    }
      if (ps2x.Button(PSB_R2)){  // SUBINDO
        posPulso -= VELOC; 
        if (posPulso < 0){ posPulso = 0;}
        Pulso.write(posPulso); 
        delay(Delay);  
        Serial.println ("PULSO: Subindo ");   
   
    }

//Base
      if (ps2x.Button(PSB_PAD_LEFT)){  // ESQUERDA 
        posBase += VELOC; 
        if (posBase>=179) {posBase = 179;}
        Base.write(posBase); 
        delay(Delay);    
        Serial.println ("BASE: Anti-Horario ");   
    }
      if (ps2x.Button(PSB_PAD_RIGHT)){  //  DIREITA 
        posBase -= VELOC; 
        if (posBase < 2){ posBase = 2;}
        Base.write(posBase); 
        delay(Delay);  
        Serial.println ("BASE: Horario ");   
   
    }

    
//Anti-Braco
      if (ps2x.Button(PSB_GREEN)){  // ESQUERDA 
        posAntiBraco += VELOC; 
        if (posAntiBraco>=179) {posAntiBraco = 179;}
        AntiBraco.write(posAntiBraco); 
        delay(Delay);    
        Serial.println ("ANTI-BRACO: Subindo ");   
    }
      if (ps2x.Button(PSB_BLUE)){  //  DIREITA 
        posAntiBraco -= VELOC; 
        if (posAntiBraco < 0){ posAntiBraco = 0;}
        AntiBraco.write(posAntiBraco); 
        delay(Delay);  
        Serial.println ("ANTI-BRACO: Descendo ");   
   
    }

//BRACO
    if (ps2x.Button(PSB_PAD_UP)){  // SUBINDO
        posBraco += VELOCBRACO; 
        if (posBraco > 180){ posBraco = 180;} 
        Braco1.write(posBraco); 
        Braco2.write(180-posBraco);
        delay(Delay);  
        Serial.println ("BRACO: Subindo ");   

    }
  if (ps2x.Button(PSB_PAD_DOWN)){   // DESCENDO
        posBraco -= VELOCBRACO; 
        if (posBraco < 0){ posBraco = 0;}
        Braco1.write(posBraco); 
        Braco2.write(180-posBraco);
        delay(Delay);     
        Serial.println ("BRACO: Descendo ");   
 
    }
    

    



  

  
// MONITOR SERIAL JOYSTICK
  if (ps2x.Button(PSB_R1)) {
     Serial.print("Valores Joystick:");
        Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
        Serial.print(",");
        Serial.print(ps2x.Analog(PSS_LX), DEC); 
        Serial.print(",");
        Serial.print(ps2x.Analog(PSS_RY), DEC); 
        Serial.print(",");
        Serial.println(ps2x.Analog(PSS_RX), DEC); 
  }
  
     delay(50);
    

}
