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


//GRAVAR EM /ttyUSB1


//Biblioteca do ROS.
#include <ros.h>
#include <custom_msg/set_angles.h>
#include <custom_msg/status_arm.h>

//Definições para manipular os motores mais facilmente.
#define PARAR               0
#define HOR                 1
#define ANTHOR              2

//Pinos de acionamento dos motores.
#define PONTE_PUN_A         4
#define PONTE_PUN_B         9
#define PONTE_GAR_A         7
#define PONTE_GAR_B         8

//Pinos de PWM dos motores.
#define PIN_PWM_PUN         6
#define PIN_PWM_GAR         5

//Pinos que habilitam os motores.
#define EN_PUN              A0
#define EN_GAR              A1

//Definição de cada motor para facilitar a manipulação.
#define MOTOR_PUN           0
#define MOTOR_GAR           1

//Pinos de sinal dos encoders.
#define ENC_PUN_A           18
#define ENC_PUN_B           19

//Constantes que definem quantos pulsos de encoder por grau de rotação cada motor possui.
#define DEG2PUL_PUN         148

//Constantes que representam o "zero" dos encoders, em graus. É a pose default do manipulador.
#define DEFAULT_PUN         -133

//Constantes para os PID do punho.
#define kP_PUN              0.50
#define kI_PUN              0.08
#define kD_PUN              0.06

//Máximo PWM.
#define PWM_MAX             255

//Variáveis de controle.
long enc_PUN =              0;
long setpoint_PUN =         0;
long erro_PUN =             0;
long output_PUN =           0;
long tolerance_PUN =        0;
long soma_erro_PUN =        0;
long last_erro_PUN =        0;
long var_erro_PUN =         0;
bool EMERGENCY_STOP =       false;

//Handler do nó ROS.
ros::NodeHandle nh;

//Subscrição no tópico /cmd_3R, onde se recebe os ângulos das juntas.
void Callback(const custom_msg::set_angles &rec_msg);
ros::Subscriber<custom_msg::set_angles> sub("/cmd_3R", &Callback);

//Publicador no tópico /status_PUN, onde se publica a situação do punho.
custom_msg::status_arm pub_msg_PUN;
ros::Publisher pub_PUN("/status_PUN", &pub_msg_PUN);

//Variáveis de tempo.
ros::Time last_time;
ros::Time actual_time;
float timelapse =     0;

//Variáveis de tempo para parada .
long init_time = 0;
long pulse;
long last;
long actual;
long lapsed = 0;
long time_to_stop = 1500;

void setup()
{
  //Pinos que mandam comando para as pontes são saídas.
  pinMode(PONTE_PUN_A, OUTPUT);
  pinMode(PONTE_PUN_B, OUTPUT);
  //pinMode(PONTE_GAR_A, OUTPUT);
  //pinMode(PONTE_GAR_B, OUTPUT);

  //Pinos que mandam o PWM são saídas.
  pinMode(PIN_PWM_PUN, OUTPUT);
  //pinMode(PIN_PWM_GAR, OUTPUT);

  //Pinos que habilitam o uso dos motores são saídas e devem estar com nível lógico alto.
  pinMode(EN_PUN, OUTPUT);
  //pinMode(EN_GAR, OUTPUT);
  digitalWrite(EN_PUN, HIGH);
  //digitalWrite(EN_GAR, HIGH);

  //Pinos que recebem os pulsos dos encoders são entradas.
  pinMode(ENC_PUN_A, INPUT);
  pinMode(ENC_PUN_B, INPUT);

  //Seta a tolerância como 1/4 de grau.
  tolerance_PUN = DEG2PUL_PUN/4;

  //Habilita as interrupções nos pinos "A" dos encoders, acionadas na borda de subida (por opção de projeto).
  attachInterrupt(digitalPinToInterrupt(ENC_PUN_A), CheckEncoder_PUN, RISING);

  Serial.begin(9600);      //Inicia a comunicação serial.
  nh.initNode();           //Inicia o nó ROS.
  nh.subscribe(sub);       //Subscreve-se no tópico, conforme "sub".
  nh.advertise(pub_PUN);   //Passa a publicar no tópico, conforme "pub_PUN".

  //Inicia as variáveis de controle.
  last_time = nh.now();
  last_erro_PUN = erro_PUN;
  
  //Aciona a rotina de reset do manipulador.
  init_time = millis();
  reset_PUNGAR();
}

void loop()
{ 
  nh.spinOnce();

  if (nh.connected()) {

    //Executa somente se a mensagem não contem aviso de parada emergencial.
    if (EMERGENCY_STOP) {
      motorGo(MOTOR_PUN, PARAR, 0);
      motorGo(MOTOR_GAR, PARAR, 0);
    }else{

      //Calcula o delta tempo.
      actual_time = nh.now();
      timelapse = (actual_time.toSec() - last_time.toSec());
      last_time = actual_time;
      
      //Calcula as variáveis para o PID do punho.
      erro_PUN = enc_PUN - setpoint_PUN;
      soma_erro_PUN = soma_erro_PUN + (erro_PUN * timelapse);
      var_erro_PUN = (erro_PUN - last_erro_PUN) / timelapse;
      last_erro_PUN = erro_PUN;

      //Verifica se o punho tem que acionar.
      if (abs(erro_PUN) > tolerance_PUN) {
        //Calcula o PWM do punho.
        if(abs(erro_PUN) > DEG2PUL_PUN*10){
          //Se o erro for maior que 10 graus, aciona o PWM máximo.
          output_PUN = PWM_MAX;
        }else{
          //Se o erro for menor que 10 graus, calcula o PID.
          output_PUN = abs(kP_PUN*erro_PUN + kI_PUN*soma_erro_PUN + kD_PUN*var_erro_PUN);
        }
        
        //Verifica para qual lado tem que girar. Se o output é positivo, gira no sentido horário.
        if (erro_PUN > 0) {
          motorGo(MOTOR_PUN, HOR, output_PUN);
        }else{
          motorGo(MOTOR_PUN, ANTHOR, output_PUN);
        }
      }else{
        //Se dentro da tolerância, mantém parado.
        motorGo(MOTOR_PUN, PARAR, 0);
        soma_erro_PUN = 0;
      }


      //AQUI VAI A LÓGICA PRA ABRIR OU FECHAR A GARRA.


    }

    //Publica as informações sobre o punho.
    pub_msg_PUN.junta = "Punho";
    pub_msg_PUN.pulsos_setpoint = setpoint_PUN;
    pub_msg_PUN.pulsos_contados = enc_PUN;
    pub_msg_PUN.pulsos_erro = erro_PUN;
    pub_msg_PUN.output_PID = output_PUN;
    pub_PUN.publish(&pub_msg_PUN);
    
    //Calcula tempo entre pulsos
    actual = millis() - init_time;
    init_time = actual;
    lapsed = actual - last; 
    pulse = pulse + lapsed;
    last = pulse;
    if (pulse >= time_to_stop){
       motorGo(MOTOR_PUN, PARAR, 0);
    }
    nh.spinOnce();
  }else{
     motorGo(MOTOR_PUN, PARAR, 0);
    }
}

//Função que trata a mensagem recebida, convertendo o ângulo recebido em contagem de pulsos.
void Callback(const custom_msg::set_angles & rec_msg) {
  setpoint_PUN = (rec_msg.set_PUN - DEFAULT_PUN) * DEG2PUL_PUN;
  EMERGENCY_STOP = rec_msg.emergency_stop;
}

//Função que comanda direção e velocidade dos motores.
void motorGo(int motor, int dir, int pwm)
{
  switch (motor)
  {
    case MOTOR_PUN: {
        switch (dir)
        {
          case HOR: {
              digitalWrite(PONTE_PUN_A, HIGH);
              digitalWrite(PONTE_PUN_B, LOW);
              break;
            }
          case ANTHOR: {
              digitalWrite(PONTE_PUN_A, LOW);
              digitalWrite(PONTE_PUN_B, HIGH);
              break;
            }
          case PARAR: {
              digitalWrite(PONTE_PUN_A, LOW);
              digitalWrite(PONTE_PUN_B, LOW);
              break;
            }
        }
        analogWrite(PIN_PWM_PUN, pwm);
      }

    case MOTOR_GAR: {
        switch (dir)
        {
          case HOR: {
              digitalWrite(PONTE_GAR_A, HIGH);
              digitalWrite(PONTE_GAR_B, LOW);
              break;
            }
          case ANTHOR: {
              digitalWrite(PONTE_GAR_A, LOW);
              digitalWrite(PONTE_GAR_B, HIGH);
              break;
            }
          case PARAR: {
              digitalWrite(PONTE_GAR_A, LOW);
              digitalWrite(PONTE_GAR_B, LOW);
              break;
            }
        }
        analogWrite(PIN_PWM_GAR, pwm);
      }
  }
}

//Interrupções dos encoders apenas ocorre na subida do canal A.
//Se o canal B está "high" é porque ele acionou antes. O motor está indo no sentido HORÁRIO. Conta como "-1" na posição do pulso.
//Se o canal B está "low" é porque ele acionará logo em seguida. O motor está indo no sentido ANTI-HORÁRIO. Conta como "+1" na posição do pulso.
void CheckEncoder_PUN() {
  enc_PUN += digitalRead(ENC_PUN_B) == HIGH ? -1 : +1;
  pulse = 0;
}

void reset_PUNGAR() {
  //Programar reset.
}
