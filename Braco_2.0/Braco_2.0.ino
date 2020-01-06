#include <PS2X_lib.h>  // Bibliotecas & Objetos 
#include <VarSpeedServo.h>
#include <EEPROM.h>
PS2X ps2x;


#define espacoMemoria 199
#define tempoPausaEntreMovimentos 500  //configura o tempo de pausa entre cada movimento

#define pinPulso  13  // servo 1
#define pinAntiBraco  12 // servo 2
#define pinBraco1  11   // servo 3 -  de fora 
#define pinBase  10  // servo 4 
#define pinGarra  9  // servo 5
#define pinBraco2  8  // servo 6 -  de dentro

#define pinBotao1     3  // Portas Botoes 
#define pinBotao2     2
#define pinSwitch  30


#define pinLedA       5  // Portas Led
#define pinLedB       4

#define pinPot1      A5  // Portas Potenciometros 
#define pinPot2      A4
#define pinPot3      A3
#define pinPot4      A0
#define pinPot5      A1
#define pinPot6      A2

VarSpeedServo Base;  // Declaracao Servos 
VarSpeedServo Braco1;
VarSpeedServo Braco2;
VarSpeedServo AntiBraco;
VarSpeedServo Pulso;
VarSpeedServo Garra;

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
int VELOCBRACO = 6 ; // Velocidade Braco

int vibrate;
int Wireless;

byte pinBotao1Modo();  // Funcoes 
bool pinBotao2Retencao();
bool pinBotao2Apertado();
void pinLedAPisca(bool reset = false);
void setMemoria(byte posicao, byte servo, byte valor);
byte readMemoria(byte posicao, byte servo);
int ultMemoria;
int buttonState;         // variable for reading the pushbutton status



void setup() {

  error = ps2x.config_gamepad(22,30,26,34, true, true);  // Define os pinos onde está conectado o controle.
  type = ps2x.readType();
  Serial.begin(57600);
  Serial.println("BRACO ROBOTICO 1.5 - CONTROLE REMOTO PS2 + MODO AUTOMATICO");
  
  
  Base.attach(pinBase); 
  Braco1.attach(pinBraco1);
  Braco2.attach(pinBraco2);  
  AntiBraco.attach(pinAntiBraco);
  Pulso.attach(pinPulso);
  Garra.attach(pinGarra);

  pinMode(pinLedA, OUTPUT);
  pinMode(pinLedB, OUTPUT);

  pinMode(pinBotao1, INPUT_PULLUP);
  pinMode(pinBotao2, INPUT_PULLUP);
  pinMode(pinSwitch, INPUT_PULLUP);

  
  ultMemoria = EEPROM.read(0);
  
  GarraAntes = ps2x.Analog(PSS_RX); 
   
}

void loop() {
static byte modo = 0;
static byte modoAnt;
static byte movimento = 0;
static byte posMemoria = 0;
static unsigned long delayPausa;

  buttonState = digitalRead(pinSwitch);


if(error == 1) //Controle não encontrado
 { 
   Serial.println("No Controller Found,check wiring");
   return;
 }

  ps2x.read_gamepad(false, vibrate);

   if (buttonState == HIGH) {
      Wireless = 0;
      digitalWrite (pinLedA, HIGH);
      digitalWrite (pinLedB, HIGH);

   } else {
      Wireless = 1;
   }

  

  if (ps2x.Analog(PSS_RX)>130){
  vibrate = ps2x.Analog(PSS_RX);
  } else { vibrate = 0; }

if (Wireless == 0) { // MODO WIRELESS;
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


 if (Wireless == 1) { 
  // MODO AUTOMATICO 
  
  //Modo Normal
  if (modo == 0) {
     digitalWrite(pinLedA, LOW);

     if (pinBotao2Retencao()) {
        digitalWrite(pinLedB, HIGH);

        //executa um movimento  
        if (movimento == 0) {
          byte velocidade = map(analogRead(pinPot6),0,1023,0,255);  
          Pulso.slowmove(readMemoria(posMemoria,0), velocidade);
          AntiBraco.slowmove(readMemoria(posMemoria,1), velocidade);
          Braco1.slowmove(readMemoria(posMemoria,2), velocidade);
          Base.slowmove(readMemoria(posMemoria,3), velocidade);
          Garra.slowmove(readMemoria(posMemoria,4), velocidade);
          //servo6.slowmove(readMemoria(posMemoria,2), velocidade);
          Braco2.slowmove( map(readMemoria(posMemoria,2), 0, 179, 179, 0), velocidade);  //Servo sincronizado inversamente ao Servo3    
          movimento = 1;
        } 

        //aguarda término de um movimento para selecionar o próximo movimento 
        if (movimento == 1) {
          if ( (Pulso.read() == readMemoria(posMemoria,0)) &&
               (AntiBraco.read() == readMemoria(posMemoria,1)) &&
               (Braco1.read() == readMemoria(posMemoria,2)) &&
               (Base.read() == readMemoria(posMemoria,3)) &&
               (Garra.read() == readMemoria(posMemoria,4)) ) {
  
             posMemoria++;
             if (posMemoria > ultMemoria) { posMemoria = 0; }
             
             delayPausa = millis();
             movimento = 2;
          } 
        }

        //pausa entre movimentos
        if (movimento == 2) {
           if ((millis() - delayPausa) > tempoPausaEntreMovimentos) {
              movimento = 0;
           }
        }
        
     } else {
        digitalWrite(pinLedB, LOW);
        
        if (pinBotao1Modo() == 2) {
           modo = 1;
        }
     }
  }

  //Modo Gravação
  if (modo == 1) {

     if (modoAnt == 0) {
        pinLedAPisca(true);
        ultMemoria = -1;      
        EEPROM.write(0, ultMemoria);   
     }
    
     digitalWrite(pinLedB, LOW);
     pinLedAPisca();

     if (pinBotao1Modo() == 2) {
        modo = 0;
     }

     Pulso.write( map(analogRead(pinPot1),0,1023,0,179) );
     AntiBraco.write( map(analogRead(pinPot2),0,1023,0,179) );
     Braco1.write( map(analogRead(pinPot3),0,1023,0,179) );
     Base.write( map(analogRead(pinPot4),0,1023,0,179) );     
     Garra.write( map(analogRead(pinPot5),0,1023,0,179) );  
     //servo6.write( map(analogRead(pinPot3),0,1023,0,179) ); 
     Braco2.write( map(analogRead(pinPot3),0,1023,179,0) );  //Servo sincronizado inversamente ao Servo3    

     if (pinBotao2Apertado()) {
        ultMemoria++;
        EEPROM.write(0, ultMemoria);
        setMemoria(ultMemoria, 0, map(analogRead(pinPot1),0,1023,0,179));
        setMemoria(ultMemoria, 1, map(analogRead(pinPot2),0,1023,0,179));
        setMemoria(ultMemoria, 2, map(analogRead(pinPot3),0,1023,0,179));
        setMemoria(ultMemoria, 3, map(analogRead(pinPot4),0,1023,0,179));
        setMemoria(ultMemoria, 4, map(analogRead(pinPot5),0,1023,0,179));

        digitalWrite(pinLedB, HIGH);
        delay(250);
        digitalWrite(pinLedB, LOW);

        if ( ultMemoria == (espacoMemoria - 1)) {
            modo = 0; 
        }
     }
  }

  modoAnt = modo;
  }
 
}
byte pinBotao1Modo() {
#define tempoDebounce 40 //(tempo para eliminar o efeito Bounce EM MILISEGUNDOS)
#define tempoBotao    1500

   bool estadoBotao;
   static bool estadoBotaoAnt; 
   static byte estadoRet = 0;
   static unsigned long delayBotao = 0;
   static byte enviado = 0;

   if ( (millis() - delayBotao) > tempoDebounce ) {
       estadoBotao = digitalRead(pinBotao1);
       if (estadoRet == 0) {
          if ( !estadoBotao && (estadoBotao != estadoBotaoAnt) ) {
             estadoRet = 1;
             delayBotao = millis();
          }
       }

       if (estadoRet == 1) {
          if ( estadoBotao && (estadoBotao != estadoBotaoAnt) ) {
             estadoRet = 0;
             delayBotao = millis();
          }
       }
       estadoBotaoAnt = estadoBotao;       
   }

   if (estadoRet == 1) {
      if ((millis() - delayBotao) > tempoBotao) {
         estadoRet = 2;
         delayBotao = millis();
       }
   }

   if (estadoRet == 2) {
      enviado++;
          
      if (enviado >= 2) {
         estadoRet = 0;
         delayBotao = millis();
         enviado = 0;
      }        
   }

   return estadoRet;
}

bool pinBotao2Retencao() {
#define tempoDebounce 40 //(tempo para eliminar o efeito Bounce EM MILISEGUNDOS)

   bool estadoBotao;
   static bool estadoBotaoAnt; 
   static bool estadoRet = false;
   static unsigned long delayBotao = 0;

   if ( (millis() - delayBotao) > tempoDebounce ) {
       estadoBotao = digitalRead(pinBotao2);
       if ( !estadoBotao && (estadoBotao != estadoBotaoAnt) ) {
          estadoRet = !estadoRet;
          delayBotao = millis();
       }
       if ( estadoBotao && (estadoBotao != estadoBotaoAnt) ) {
          delayBotao = millis();
       }
       estadoBotaoAnt = estadoBotao;
   }

   return estadoRet;
}

bool pinBotao2Apertado() {
#define tempoDebounce 40 //(tempo para eliminar o efeito Bounce EM MILISEGUNDOS)

   bool estadoBotao;
   static bool estadoBotaoAnt; 
   static bool estadoRet;
   static unsigned long delayBotao = 0;

   estadoRet = false;
   if ( (millis() - delayBotao) > tempoDebounce ) {
       estadoBotao = digitalRead(pinBotao2);
       if ( !estadoBotao && (estadoBotao != estadoBotaoAnt) ) {
          estadoRet = true;
          delayBotao = millis();
       }
       if ( estadoBotao && (estadoBotao != estadoBotaoAnt) ) {
          delayBotao = millis();
       }
       estadoBotaoAnt = estadoBotao;
   }

   return estadoRet;
}

void pinLedAPisca(bool reset) {
static unsigned long delayPisca = 0;

   if (reset) {
      delayPisca = millis();
   } else {

     if ((millis() - delayPisca) < 500) {
        digitalWrite(pinLedA, LOW);
     } else {
        digitalWrite(pinLedA, HIGH);
     }
     
     if ((millis() - delayPisca) >= 1000) {
        delayPisca = millis();
     }  
   }
}

void setMemoria(byte posicao, byte servo, byte valor) {
int posMem;

    posMem = ((posicao * 5) + servo) + 1;
    EEPROM.write(posMem, valor);
}

byte readMemoria(byte posicao, byte servo) {
int posMem;

    posMem = ((posicao * 5) + servo) + 1;
    return EEPROM.read(posMem);
}

  
  
