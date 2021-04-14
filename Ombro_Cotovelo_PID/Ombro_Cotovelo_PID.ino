////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                        //
//    Desenvolvido por: Jonathan Cerbaro                                                  //
//    UTFPR - Universidade Tecnológica Federal do Paraná                                  //
//    CPGEI - Programa de Pós-Graduação em Engenharia Elétrica e Informática Industrial   //
//    LASER - Laboratório Avançado de Sistemas Embarcados e Robótica                      //
//    Orientador: Prof. Dr. André Schneider de Oliveira                                   //
//    Coorientador: Prof. Dr. João Alberto Fabro                                          //
//    Data: XX/XX/2020                                                                    //
//                                                                                        //
////////////////////////////////////////////////////////////////////////////////////////////


//GRAVAR EM /ttyUSB0


//Bibliotecas do ROS.
#include <ros.h>
#include <custom_msg/set_angles.h>
#include <custom_msg/status_arm.h>

//Definições para manipular os motores mais facilmente.
#define PARAR               0
#define HOR                 1
#define ANTHOR              2

//Pinos de acionamento dos motores.
#define PONTE_OMB_A         4
#define PONTE_OMB_B         9
#define PONTE_COT_A         7
#define PONTE_COT_B         8

//Pinos de PWM dos motores.
#define PIN_PWM_OMB         6
#define PIN_PWM_COT         5

//Pinos que habilitam os motores.
#define EN_OMB              A0
#define EN_COT              A1

//Definição de cada motor para facilitar a manipulação.
#define MOTOR_OMB           0
#define MOTOR_COT           1

//Pinos de sinal dos encoders.
#define ENC_OMB_A           18
#define ENC_OMB_B           19
#define ENC_COT_A           20
#define ENC_COT_B           21

//Constantes que definem quantos pulsos de encoder por grau de rotação cada motor possui.
#define DEG2PUL_OMB         196
#define DEG2PUL_COT         325

//Constantes que representam o "zero" dos encoders, em graus. É a pose default do manipulador.
#define DEFAULT_OMB         -40
#define DEFAULT_COT         150

//Constantes para os PID do ombro.
#define kP_OMB              0.20
#define kI_OMB              0.01
#define kD_OMB              0.01

//Constantes para os PID do cotovelo.
#define kP_COT              0.20    //----------------------------------TESTAR EMPIRICAMENTE MELHORES VALORES.
#define kI_COT              0.01    //----------------------------------TESTAR EMPIRICAMENTE MELHORES VALORES.
#define kD_COT              0.01    //----------------------------------TESTAR EMPIRICAMENTE MELHORES VALORES.

//Máximo PWM.
#define PWM_MAX             255

//Variáveis de controle.
long enc_OMB =              0;
long enc_COT =              0;

long setpoint_OMB =         0;
long setpoint_COT =         0;

long erro_OMB =             0;
long erro_COT =             0;

long output_OMB =           0;
long output_COT =           0;

long tolerance_OMB =        0;
long tolerance_COT =        0;

long soma_erro_OMB =        0;
long soma_erro_COT =        0;

long last_erro_OMB =        0;
long last_erro_COT =        0;

long var_erro_OMB =         0;
long var_erro_COT =         0;

float timelapse =           0;
bool EMERGENCY_STOP =       false;

//Handler do nó ROS.
ros::NodeHandle nh;

//Subscrição no tópico /cmd_3R, onde se recebe os ângulos das juntas.
void Callback(const custom_msg::set_angles &rec_msg);
ros::Subscriber<custom_msg::set_angles> sub("/cmd_3R", &Callback);

//Publicador no tópico /status_OMB, onde se publica a situação do ombro.
custom_msg::status_arm pub_msg_OMB;
ros::Publisher pub_OMB("/status_OMB", &pub_msg_OMB);

//Publicador no tópico /status_COT, onde se publica a situação do cotovelo.
custom_msg::status_arm pub_msg_COT;
ros::Publisher pub_COT("/status_COT", &pub_msg_COT);

//Variáveis de tempo.
ros::Time last_time;
ros::Time actual_time;

void setup()
{
  //Pinos que mandam comando para as pontes são saídas.
  pinMode(PONTE_OMB_A, OUTPUT);
  pinMode(PONTE_OMB_B, OUTPUT);
  pinMode(PONTE_COT_A, OUTPUT);
  pinMode(PONTE_COT_B, OUTPUT);

  //Pinos que mandam o PWM são saídas.
  pinMode(PIN_PWM_OMB, OUTPUT);
  pinMode(PIN_PWM_COT, OUTPUT);

  //Pinos que habilitam o uso dos motores são saídas e devem estar com nível lógico alto.
  pinMode(EN_OMB, OUTPUT);
  pinMode(EN_COT, OUTPUT);
  digitalWrite(EN_OMB, HIGH);
  digitalWrite(EN_COT, HIGH);

  //Pinos que recebem os pulsos dos encoders são entradas.
  pinMode(ENC_OMB_A, INPUT);
  pinMode(ENC_OMB_B, INPUT);
  pinMode(ENC_COT_A, INPUT);
  pinMode(ENC_COT_B, INPUT);

  //Seta a tolerância como 1/4 de grau.
  tolerance_OMB = DEG2PUL_OMB/4;
  tolerance_COT = DEG2PUL_COT/4;

  //Habilita as interrupções nos pinos "A" dos encoders, acionadas na borda de subida (por opção de projeto).
  attachInterrupt(digitalPinToInterrupt(ENC_OMB_A), CheckEncoder_OMB, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_COT_A), CheckEncoder_COT, RISING);

  Serial.begin(9600);      //Inicia a comunicação serial.
  nh.initNode();           //Inicia o nó ROS.
  nh.subscribe(sub);       //Subscreve-se no tópico, conforme "sub".
  nh.advertise(pub_OMB);   //Passa a publicar no tópico, conforme "pub_OMB".
  nh.advertise(pub_COT);   //Passa a publicar no tópico, conforme "pub_COT".

  //Inicia as variáveis de controle.
  last_time = nh.now();
  last_erro_OMB = erro_OMB;
  last_erro_COT = erro_COT;
  
  //Aciona a rotina de reset do manipulador.
  reset_OMBCOT();
}

void loop()
{ 
  nh.spinOnce();
  //BRINCAR DE NH1=NH
  if (nh.connected()) {

    //Executa somente se a mensagem não contem aviso de parada emergencial.
    if (EMERGENCY_STOP) {
      motorGo(MOTOR_OMB, PARAR, 0);
      motorGo(MOTOR_COT, PARAR, 0);
    }else{

      //Calcula o delta tempo.
      actual_time = nh.now();
      timelapse = (actual_time.toSec() - last_time.toSec());
      last_time = actual_time;
      
      //Calcula as variáveis para o PID do ombro.
      erro_OMB = enc_OMB - setpoint_OMB;
      soma_erro_OMB = soma_erro_OMB + (erro_OMB * timelapse);
      var_erro_OMB = (erro_OMB - last_erro_OMB) / timelapse;
      last_erro_OMB = erro_OMB;

      //Calcula as variáveis para o PID do cotovelo.
      erro_COT = enc_COT - setpoint_COT;
      soma_erro_COT = soma_erro_COT + (erro_COT * timelapse);
      var_erro_COT = (erro_COT - last_erro_COT) / timelapse;
      last_erro_COT = erro_COT;

      //Verifica se o ombro tem que acionar.
      if (abs(erro_OMB) > tolerance_OMB) {
        //Calcula o PWM do ombro.
        if(abs(erro_OMB) > DEG2PUL_OMB*10){
          //Se o erro for maior que 10 graus, aciona o PWM máximo.
          output_OMB = PWM_MAX;
          soma_erro_OMB = 0;
        }else{
          //Se o erro for menor que 10 graus, calcula o PID.
          output_OMB = abs(kP_OMB*erro_OMB + kI_OMB*soma_erro_OMB + kD_OMB*var_erro_OMB);
        }
        
        //Verifica para qual lado tem que girar. Se o output é positivo, gira no sentido horário.
        if (erro_OMB > 0) {
          motorGo(MOTOR_OMB, HOR, output_OMB);
        }else{
          motorGo(MOTOR_OMB, ANTHOR, output_OMB);
        }
      }else{
        //Se dentro da tolerância, mantém parado.
        motorGo(MOTOR_OMB, PARAR, 0);
        soma_erro_OMB = 0;
      }

      //Verifica se o cotovelo tem que acionar.
      if (abs(erro_COT) > tolerance_COT) {
        //Calcula o PWM do cotovelo.
        if(abs(erro_COT) > DEG2PUL_COT*10){
          //Se o erro for maior que 10 graus, aciona o PWM máximo.
          output_COT = PWM_MAX;
          soma_erro_COT = 0;
        }else{
          //Se o erro for menor que 10 graus, calcula o PID.
          output_COT = abs(kP_COT*erro_COT + kI_COT*soma_erro_COT + kD_COT*var_erro_COT);
        }
        
        //Verifica para qual lado tem que girar. Se o erro é positivo, gira no sentido horário.
        if (erro_COT > 0) {
          motorGo(MOTOR_COT, HOR, output_COT);
        }else{
          motorGo(MOTOR_COT, ANTHOR, output_COT);
        }
      }else{
        //Se dentro da tolerância, mantém parado.
        motorGo(MOTOR_COT, PARAR, 0);
        soma_erro_COT = 0;
      }
    }

    //Publica as informações sobre o ombro.
    pub_msg_OMB.junta = "Ombro";
    pub_msg_OMB.pulsos_setpoint = setpoint_OMB;
    pub_msg_OMB.pulsos_contados = enc_OMB;
    pub_msg_OMB.pulsos_erro = erro_OMB;
    pub_msg_OMB.output_PID = output_OMB;
    pub_OMB.publish(&pub_msg_OMB);
    nh.spinOnce();

    //Publica as informações sobre o cotovelo.
    pub_msg_COT.junta = "Cotovelo";
    pub_msg_COT.pulsos_setpoint = setpoint_COT;
    pub_msg_COT.pulsos_contados = enc_COT;
    pub_msg_COT.pulsos_erro = erro_COT;
    pub_msg_COT.output_PID = output_COT;
    pub_COT.publish(&pub_msg_COT);
    nh.spinOnce();
  }
}

//Função que trata a mensagem recebida, convertendo o ângulo recebido em contagem de pulsos.
void Callback(const custom_msg::set_angles & rec_msg) {
  setpoint_OMB = (rec_msg.set_OMB - DEFAULT_OMB) * DEG2PUL_OMB;
  setpoint_COT = (rec_msg.set_COT - DEFAULT_COT) * DEG2PUL_COT;
  EMERGENCY_STOP = rec_msg.emergency_stop;
}

//Função que comanda direção e velocidade dos motores.
void motorGo(int motor, int dir, int pwm)
{
  switch (motor)
  {
    case MOTOR_OMB: {
        switch (dir)
        {
          case HOR: {
              digitalWrite(PONTE_OMB_A, HIGH);
              digitalWrite(PONTE_OMB_B, LOW);
              break;
            }
          case ANTHOR: {
              digitalWrite(PONTE_OMB_A, LOW);
              digitalWrite(PONTE_OMB_B, HIGH);
              break;
            }
          case PARAR: {
              digitalWrite(PONTE_OMB_A, LOW);
              digitalWrite(PONTE_OMB_B, LOW);
              break;
            }
        }
        analogWrite(PIN_PWM_OMB, pwm);
      }

    case MOTOR_COT: {
        switch (dir)
        {
          case HOR: {
              digitalWrite(PONTE_COT_A, HIGH);
              digitalWrite(PONTE_COT_B, LOW);
              break;
            }
          case ANTHOR: {
              digitalWrite(PONTE_COT_A, LOW);
              digitalWrite(PONTE_COT_B, HIGH);
              break;
            }
          case PARAR: {
              digitalWrite(PONTE_COT_A, LOW);
              digitalWrite(PONTE_COT_B, LOW);
              break;
            }
        }
        analogWrite(PIN_PWM_COT, pwm);
      }
  }
}

//Interrupções dos encoders apenas ocorre na subida do canal A.
//Se o canal B está "high" é porque ele acionou antes. O motor está indo no sentido HORÁRIO. Conta como "-1" na posição do pulso.
//Se o canal B está "low" é porque ele acionará logo em seguida. O motor está indo no sentido ANTI-HORÁRIO. Conta como "+1" na posição do pulso.
void CheckEncoder_OMB() {
  enc_OMB += digitalRead(ENC_OMB_B) == HIGH ? -1 : +1;
}
void CheckEncoder_COT() {
  enc_COT += digitalRead(ENC_COT_B) == HIGH ? -1 : +1;
}

void reset_OMBCOT() {
  //Programar reset.
}
