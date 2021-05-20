////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                        //
//    Desenvolvido por: Jonathan Cerbaro, Dieisson Martinelli                             //
//    UTFPR - Universidade Tecnológica Federal do Paraná                                  //
//    CPGEI - Programa de Pós-Graduação em Engenharia Elétrica e Informática Industrial   //
//    LASER - Laboratório Avançado de Sistemas Embarcados e Robótica                      //
//    Orientador: Prof. Dr. André Schneider de Oliveira                                   //
//    Coorientador: Prof. Dr. João Alberto Fabro                                          //
//    Última Modificação: XX/XX/2020                                                      //
//                                                                                        //
////////////////////////////////////////////////////////////////////////////////////////////


//GRAVAR EM /ttyUSB0


//Biblioteca do ROS.
#include <ros.h>
#include <custom_msg/set_angles.h>
#include <custom_msg/status_arm.h>
#include <custom_msg/reset_COT.h>

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
#define DEG2PUL_OMB         200
#define DEG2PUL_COT         325//290//300//325

//Constantes que representam o "zero" dos encoders, em graus. É a pose default do manipulador.
#define DEFAULT_OMB         -40
#define DEFAULT_COT         145

//Constantes para os PID do OMBRO.
#define kP_OMB              0.100
#define kI_OMB              0.010
#define kD_OMB              0.050

//Constantes para os PID do COTOVELO.
#define kP_COT              0.050
#define kI_COT              0.001
#define kD_COT              0.020

//Limites do PWM.
#define PWM_MAX             255
#define PWM_MIN_OMB          50
#define PWM_MIN_COT         100

//Variáveis de controle do OMBRO.
long enc_OMB =              0;
long setpoint_OMB =         0;
long last_setpoint_OMB =    0;
long erro_OMB =             0;
long output_OMB =           0;
long tolerance_OMB =        0;
long soma_erro_OMB =        0;
long last_erro_OMB =        0;
long var_erro_OMB =         0;
bool PID_enable_OMB =       true;
bool reset_COT =            false;

//Variáveis de controle do COTOVELO.
long enc_COT =              0;
long setpoint_COT =         0;
long last_setpoint_COT =    0;
long erro_COT =             0;
long output_COT =           0;
long tolerance_COT =        0;
long soma_erro_COT =        0;
long last_erro_COT =        0;
long var_erro_COT =         0;
bool PID_enable_COT =       true;

long time_PID =             0;
long start_PID =            0;

bool EMERGENCY_STOP =       false;

//Auxiliar para log de variáveis.
char buff[16];

//Handler do nó ROS.
ros::NodeHandle nh;

//Subscrição no tópico /cmd_3R, onde se recebe os ângulos das juntas.
void Callback(const custom_msg::set_angles &rec_msg);
ros::Subscriber<custom_msg::set_angles> sub("/cmd_3R", &Callback);

//Publicador no tópico /status_OMB, onde se publica a situação do OMBRO.
custom_msg::status_arm pub_msg_OMB;
ros::Publisher pub_OMB("/status_OMB", &pub_msg_OMB);

//Publicador no tópico /status_COT, onde se publica a situação do COTOVELO.
custom_msg::status_arm pub_msg_COT;
ros::Publisher pub_COT("/status_COT", &pub_msg_COT);

//Publicador no tópico /reset_COT, onde o COTOVELO avisa se já resetou.
custom_msg::reset_COT pub_msg_reset_COT;
ros::Publisher pub_reset_COT("/reset_COT", &pub_msg_reset_COT);

//Constantes de tempo para caso de obstáculo ou reset.
long time_to_stop =         500;  //Tempo que o motor pode forçar antes de indicar que é uma colisão.
long delay_lock =           3000; //Intervalo de tempo que o motor leva para tentar reativar o PID depois de colidir.
long counter_max =          800;  //Contador de pulsos para zerar o timeout, caso contrário é sensível demais.

//Variáeis para controlar colisões e resets.
unsigned long pulse_timout_OMB;
unsigned long start_OMB;
unsigned long lock_OMB;
long counter_OMB =          0;
long retries_OMB =          0;
bool working_OMB =          false;

unsigned long pulse_timout_COT;
unsigned long start_COT;
unsigned long lock_COT;
long counter_COT =          0;
long retries_COT =          0;
bool working_COT =          false;

bool RESET =                false;
bool RESET_OMB =            false;
bool RESET_COT =            false;

bool RETRY =                false;
bool RETRY_OMB =            false;
bool RETRY_COT =            false;

bool already_reset =        false;

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

  //Seta a tolerância como 1/2 de grau.
  tolerance_OMB = DEG2PUL_OMB/2;
  tolerance_COT = DEG2PUL_COT/2;

  //Habilita as interrupções nos pinos "A" dos encoders, acionadas na borda de subida (por opção de projeto).
  attachInterrupt(digitalPinToInterrupt(ENC_OMB_A), CheckEncoder_OMB, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_COT_A), CheckEncoder_COT, RISING);

  Serial.begin(9600);             //Inicia a comunicação serial.
  nh.initNode();                  //Inicia o nó ROS.
  nh.subscribe(sub);              //Subscreve-se no tópico, conforme "sub".
  nh.advertise(pub_OMB);          //Passa a publicar no tópico, conforme "pub_OMB".
  nh.advertise(pub_COT);          //Passa a publicar no tópico, conforme "pub_OMB".
  nh.advertise(pub_reset_COT);    //Passa a publicar no tópico, conforme "pub_reset_COT".
}

void loop()
{ 
  nh.spinOnce();

  //Publica as informações sobre o OMBRO.
  pub_msg_OMB.junta = "OMBRO";
  pub_msg_OMB.pulsos_setpoint = setpoint_OMB;
  pub_msg_OMB.pulsos_contados = enc_OMB;
  pub_msg_OMB.pulsos_erro = erro_OMB;
  pub_msg_OMB.output_P = kP_OMB*erro_OMB;
  //pub_msg_OMB.output_I = implementar!;
  pub_msg_OMB.output_D = kD_OMB*var_erro_OMB;
  pub_msg_OMB.output_PID = output_OMB;
  pub_msg_OMB.loop_time = time_PID;
  pub_OMB.publish(&pub_msg_OMB);
  nh.spinOnce();

  //Publica as informações sobre o COTOVELO.
  pub_msg_COT.junta = "COTOVELO";
  pub_msg_COT.pulsos_setpoint = setpoint_COT;
  pub_msg_COT.pulsos_contados = enc_COT;
  pub_msg_COT.pulsos_erro = erro_COT;
  pub_msg_COT.output_P = kP_COT*erro_COT;
  //pub_msg_COT.output_I = implementar!;
  pub_msg_COT.output_D = kD_COT*var_erro_COT;
  pub_msg_COT.output_PID = output_COT;
  pub_msg_COT.loop_time = time_PID;
  pub_COT.publish(&pub_msg_COT);
  nh.spinOnce();

  //Ececuta controle somente se o ROS está conectado e ao menos um dos PID está habilitado.
  if(nh.connected() && PID_enable_OMB && PID_enable_COT){
    already_reset = false;

    //Calcula o tempo entre loops de controle.
    time_PID = millis() - start_PID;
    start_PID = millis();
    
    if(PID_enable_OMB){
    //Bloco de controle do OMBRO.
        //Calcula as variáveis para o PID do OMBRO.
        erro_OMB = enc_OMB - setpoint_OMB;
        var_erro_OMB = (erro_OMB - last_erro_OMB) / time_PID;
        last_erro_OMB = erro_OMB;
    
        //Verifica se o OMBRO tem que acionar.
        if (abs(erro_OMB) > tolerance_OMB){

          pub_msg_OMB.IsDone = false;
          pub_OMB.publish(&pub_msg_OMB);
    
          //Controla o reset do timeout de colisão, caso o PID esteja começando uma operação agora.
          if (!working_OMB){
            working_OMB = true;
            start_OMB = millis();
          }
          
          //Calcula o PWM do OMBRO.
          if(abs(erro_OMB) > DEG2PUL_OMB*90){
            //Se o erro for maior que 90 graus, aciona o PWM máximo.
            output_OMB = PWM_MAX;
          }else{
            //Se o erro for menor que 90 graus, calcula o PID.
            //output_OMB = min(abs(kP_OMB*erro_OMB + kI_OMB*soma_erro_OMB + kD_OMB*var_erro_OMB),PWM_MAX);
            output_OMB = max(min(abs(kP_OMB*erro_OMB + kD_OMB*var_erro_OMB),PWM_MAX),PWM_MIN_OMB);
            //output_OMB = min(abs(kP_OMB*erro_OMB),PWM_MAX);
          }
    
          //Verifica para qual lado tem que girar. Se o output é positivo, gira no sentido horário.
          if (erro_OMB > 0){
            motorGo(MOTOR_OMB, HOR, output_OMB);
          }else{
            motorGo(MOTOR_OMB, ANTHOR, output_OMB);
          }
        }else{
          //Se dentro da tolerância, mantém parado e marca a flag de tarefa concluída.
          //motorGo(MOTOR_OMB, PARAR, 0);
          motorGo(MOTOR_OMB, ANTHOR, 20);
          output_OMB = 0;
          working_OMB = false;
          retries_OMB = 0;
          pub_msg_OMB.IsDone = true;
          pub_OMB.publish(&pub_msg_OMB);
        }
        
        //Acumula o tempo sem pulsos de encoder, caso o PID esteja executando alguma tarefa.
        //Se estourar o tempo entre pulsos enquanto o PID está trabalhando, é porque o motor está forçando em algum obstáculo, então para e desativa o PID.
        if(working_OMB){
    
          pulse_timout_OMB = millis() - start_OMB;
           
          if(pulse_timout_OMB >= time_to_stop){
            //DETECÇÃO DE COLISÕES NO OMBRO FOI DESATIVADA!!!
            //Para reativar, basta descomentar as próximas linhas.
            
             //motorGo(MOTOR_OMB, PARAR, 0);
             //PID_enable_OMB = false;
             //lock_OMB = millis();
             ////nh.logwarn("PID desativado devido a uma colisão no OMBRO.");
          }
        }
    }

    if(PID_enable_COT){
    //Bloco de controle do COTOVELO.
        //Calcula as variáveis para o PID do COTOVELO.
        erro_COT = enc_COT - setpoint_COT;
        var_erro_COT = (erro_COT - last_erro_COT) / time_PID;
        last_erro_COT = erro_COT;
    
        //Verifica se o COTOVELO tem que acionar.
        if (abs(erro_COT) > tolerance_COT){
          pub_msg_COT.IsDone = false;
          pub_COT.publish(&pub_msg_COT);
    
          //Controla o reset do timeout de colisão, caso o PID esteja começando uma operação agora.
          if (!working_COT){
            working_COT = true;
            start_COT = millis();
          }
          
          //Calcula o PWM do COTOVELO.
          if(abs(erro_COT) > DEG2PUL_COT*90){
            //Se o erro for maior que 90 graus, aciona o PWM máximo.
            output_COT = PWM_MAX;
          }else{
            //Se o erro for menor que 90 graus, calcula o PID.
            //output_COT = min(abs(kP_COT*erro_COT + kI_COT*soma_erro_COT + kD_COT*var_erro_COT),PWM_MAX);
            output_COT = max(min(abs(kP_COT*erro_COT + kD_COT*var_erro_COT),PWM_MAX),PWM_MIN_COT);
            //output_COT = min(abs(kP_COT*erro_COT),PWM_MAX);
          }
    
          //Verifica para qual lado tem que girar. Se o output é positivo, gira no sentido horário.
          if (erro_COT > 0){
            motorGo(MOTOR_COT, HOR, output_COT);
          }else{
            motorGo(MOTOR_COT, ANTHOR, output_COT);
          }
        }else{
          //Se dentro da tolerância, mantém parado e marca a flag de tarefa concluída.
          //motorGo(MOTOR_COT, PARAR, 0);
          motorGo(MOTOR_COT, ANTHOR, 20);
          output_COT = 0;
          working_COT = false;
          retries_COT = 0;
          pub_msg_COT.IsDone = true;
          pub_COT.publish(&pub_msg_COT);
        }
        //Acumula o tempo sem pulsos de encoder, caso o PID esteja executando alguma tarefa.
        //Se estourar o tempo entre pulsos enquanto o PID está trabalhando, é porque o motor está forçando em algum obstáculo, então para e desativa o PID.
        if(working_COT){
    
          pulse_timout_COT = millis() - start_COT;
           
          if(pulse_timout_COT >= time_to_stop){
             motorGo(MOTOR_COT, PARAR, 0);
             PID_enable_COT = false;
             lock_COT = millis();
             //nh.logwarn("PID desativado devido a uma colisão no COTOVELO.");
          }
        }
    }
  }else{
     //Se o ROS não está conectado, reseta.
     if(!nh.connected()){
        if(!already_reset){
          already_reset = true;
          reset_OMBCOT();
        }
     }

     //Se o ROS está conectado, porém o PID está desativado, verifica se o obstáculo foi removido a cada intervalo "delay_lock" de tempo.
     //Essa verificação é feita reativando temporariamente o PID. Caso ainda exista bloqueio, irá cair nesta condicional novamente.
     if(nh.connected() && (!PID_enable_OMB || !PID_enable_COT)){
        if(!PID_enable_OMB){
        //Bloco para tentar reativar o OMBRO.
            if(millis() - lock_OMB >= delay_lock){
                PID_enable_OMB = true;
                //nh.logwarn("Tentativa de ativar o PID do OMBRO...");
    
                //Tenta reativar o PID 5 vezes. Se falhar, ativa o RETRY 3 vezes. Se falhar, ativa o RESET.
                retries_OMB += 1;
                if(retries_OMB >= 5){
                  RETRY = 1;
                  //nh.logwarn("rotina de \"RETRY\" ativada para o OMBRO...");
                }
    
                if(retries_OMB == 8){
                  //nh.logwarn("Não foi possível atingir a posição desejada para o OMBRO. Rotina de RESET ativada.");
                  retries_OMB = 0;
                  RETRY = 0;
                  RESET = 1;
                }
                
                start_OMB = millis();
            }
        }

        if(!PID_enable_COT){
        //Bloco para tentar reativar o COTOVELO.
            if(millis() - lock_COT >= delay_lock){
                PID_enable_COT = true;
                //nh.logwarn("Tentativa de ativar o PID do COTOVELO...");
    
                //Tenta reativar o PID 5 vezes. Se falhar, ativa o RETRY 3 vezes. Se falhar, ativa o RESET.
                retries_COT += 1;
                if(retries_COT >= 5){
                  RETRY = 1;
                  //nh.logwarn("rotina de \"RETRY\" ativada para o COTOVELO...");
                }
    
                if(retries_COT == 8){
                  //nh.logwarn("Não foi possível atingir a posição desejada para o COTOVELO. Rotina de RESET ativada.");
                  retries_COT = 0;
                  RETRY = 0;
                  RESET = 1;
                }
                
                start_COT = millis();
            }
        }
        nh.spinOnce();
     }
   }
}

//Função que trata a mensagem recebida, convertendo o ângulo recebido em contagem de pulsos.
void Callback(const custom_msg::set_angles & rec_msg) {
  setpoint_OMB = (rec_msg.set_OMB - DEFAULT_OMB) * DEG2PUL_OMB;
  setpoint_COT = (rec_msg.set_COT - DEFAULT_COT) * DEG2PUL_COT;
  EMERGENCY_STOP = rec_msg.emergency_stop;
  RESET = rec_msg.reset;
  RETRY = rec_msg.retry;

  if(EMERGENCY_STOP){
    motorGo(MOTOR_OMB, PARAR, 0);
    motorGo(MOTOR_COT, PARAR, 0);
  }

  if(RESET)
    reset_OMBCOT();
    
  if(RETRY)
    retry_OMBCOT();
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

  //Contabiliza os pulsos para cada direção. Apenas zera o timeout de pulsos a cada "counter_max" pulsos em uma das direções.
  if (digitalRead(ENC_OMB_B) == HIGH){
    counter_OMB -= 1;
  }else{
    counter_OMB += 1;
  }
     
  if(abs(counter_OMB) == counter_max){
    start_OMB = millis();
    counter_OMB = 0;
  }
}

void CheckEncoder_COT() {
  enc_COT += digitalRead(ENC_COT_B) == HIGH ? -1 : +1;

  //Contabiliza os pulsos para cada direção. Apenas zera o timeout de pulsos a cada "counter_max" pulsos em uma das direções.
  if (digitalRead(ENC_COT_B) == HIGH){
    counter_COT -= 1;
  }else{
    counter_COT += 1;
  }
     
  if(abs(counter_COT) == counter_max){
    start_COT = millis();
    counter_COT = 0;
  }
}

//Função para resetar os motores. Rotaciona até detectar uma colisão. Esquece os últimos setpoints indicados.
void reset_OMBCOT() {
  
  delay(1);
  RESET_COT = true;
  start_COT = millis();

  setpoint_OMB = 0;
  setpoint_COT = 0;
  
  while(RESET_COT){
    motorGo(MOTOR_OMB, ANTHOR, 15);
    motorGo(MOTOR_COT, ANTHOR, 160);

    //Bloco para finalizar o RESET do COTOVELO.
    pulse_timout_COT = millis() - start_COT;
    if(pulse_timout_COT >= time_to_stop){
      
      motorGo(MOTOR_COT, PARAR, 0);
      
      enc_COT =              0;
      erro_COT =             0;
      output_COT =           0;
      soma_erro_COT =        0;
      last_erro_COT =        0;
      var_erro_COT =         0;
      counter_COT =          0;
      working_COT =          false;

      RESET_COT = false;
      //nh.logwarn("Reset completo no COTOVELO.");
    }
    pub_msg_COT.pulsos_setpoint = setpoint_COT;
    pub_msg_COT.pulsos_contados = enc_COT;
    pub_COT.publish(&pub_msg_COT);
    nh.spinOnce();
  }

  delay(1);
  RESET_OMB = true;
  start_OMB = millis();

  while(RESET_OMB){
    motorGo(MOTOR_OMB, HOR, 100);
    motorGo(MOTOR_COT, ANTHOR, 120);

    //Bloco para finalizar o RESET do OMBRO.
    pulse_timout_OMB = millis() - start_OMB;
    if(pulse_timout_OMB >= time_to_stop){
      
      motorGo(MOTOR_OMB, PARAR, 0);
      
      enc_OMB =              0;
      erro_OMB =             0;
      output_OMB =           0;
      soma_erro_OMB =        0;
      last_erro_OMB =        0;
      var_erro_OMB =         0;
      counter_OMB =          0;
      working_OMB =          false;

      RESET_OMB = false;
      //nh.logwarn("Reset completo no OMBRO.");
    }
    pub_msg_OMB.pulsos_setpoint = setpoint_OMB;
    pub_msg_OMB.pulsos_contados = enc_OMB;
    pub_OMB.publish(&pub_msg_OMB);
    nh.spinOnce();
  }

  RESET = false;
}

//Função para reiniciar o robô e tentar novamente ir até o setpoint.
void retry_OMBCOT(){
  //Salva o setpoint atual.
  last_setpoint_OMB = setpoint_OMB;
  last_setpoint_COT = setpoint_COT;

  //Chama a função de reset.
  reset_OMBCOT();

  //Atribui novamente o último setpoint.
  //nh.logwarn("Nova tentativa a partir da origem do OMBRO e do COTOVELO.");
  setpoint_OMB = last_setpoint_OMB;
  setpoint_COT = last_setpoint_COT;

  //Informa o PUNHO que o reset do COTOVELO terminou.
  reset_COT = true;
  pub_msg_reset_COT.reset_COT = reset_COT;
  pub_reset_COT.publish(&pub_msg_reset_COT);
  nh.spinOnce();

  reset_COT = false;
  RETRY = false;
}
