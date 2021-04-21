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
#define DEFAULT_PUN         0// Era -133, porém a função de reset torna isso desnecessário.

//Constantes para os PID do punho.
#define kP_PUN              0.50
#define kI_PUN              0.08
#define kD_PUN              0.06

//Máximo PWM.
#define PWM_MAX             255

//Variáveis de controle.
long enc_PUN =              0;
long setpoint_PUN =         0;
long last_setpoint_PUN =    0;
long erro_PUN =             0;
long output_PUN =           0;
long tolerance_PUN =        0;
long soma_erro_PUN =        0;
long last_erro_PUN =        0;
long var_erro_PUN =         0;
bool PID_enable_PUN =       true;
bool EMERGENCY_STOP =       false;

bool setpoint_GAR =         false;
bool status_GAR =         false;

//Auxiliar para log de variáveis.
char buf[16];

//Handler do nó ROS.
ros::NodeHandle nh;

//Subscrição no tópico /cmd_3R, onde se recebe os ângulos das juntas.
void Callback(const custom_msg::set_angles &rec_msg);
ros::Subscriber<custom_msg::set_angles> sub("/cmd_3R", &Callback);

//Publicador no tópico /status_PUN, onde se publica a situação do punho.
custom_msg::status_arm pub_msg_PUN;
ros::Publisher pub_PUN("/status_PUN", &pub_msg_PUN);

//Constantes de tempo para caso de obstáculo ou reset.
long time_to_stop =         500;  //Tempo que o motor pode forçar antes de indicar que é uma colisão.
long delay_lock =           3000; //Intervalo de tempo que o motor leva para tentar reativar o PID depois de colidir.
long counter_max =          800;  //Contador de pulsos para zerar o timeout, caso contrário é sensível demais.

//Variáeis para controlar colisões e resets.
unsigned long pulse_timout_PUN;
unsigned long start_PUN;
unsigned long lock_PUN;
long counter_PUN =          0;
long retries_PUN =          0;
bool working_PUN =          false;

bool RESET =                false;
bool RESET_PUN =            false;
bool RESET_GAR =            false;

bool RETRY =                false;
bool RETRY_PUN =            false;
bool RETRY_GAR =            false;

void setup()
{
  //Pinos que mandam comando para as pontes são saídas.
  pinMode(PONTE_PUN_A, OUTPUT);
  pinMode(PONTE_PUN_B, OUTPUT);
  pinMode(PONTE_GAR_A, OUTPUT);
  pinMode(PONTE_GAR_B, OUTPUT);

  //Pinos que mandam o PWM são saídas.
  pinMode(PIN_PWM_PUN, OUTPUT);
  pinMode(PIN_PWM_GAR, OUTPUT);

  //Pinos que habilitam o uso dos motores são saídas e devem estar com nível lógico alto.
  pinMode(EN_PUN, OUTPUT);
  pinMode(EN_GAR, OUTPUT);
  digitalWrite(EN_PUN, HIGH);
  digitalWrite(EN_GAR, HIGH);

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
}

void loop()
{ 
  nh.spinOnce();

  //Publica as informações sobre o punho.
  pub_msg_PUN.junta = "Punho";
  pub_msg_PUN.pulsos_setpoint = setpoint_PUN;
  pub_msg_PUN.pulsos_contados = enc_PUN;
  pub_msg_PUN.pulsos_erro = erro_PUN;
  pub_msg_PUN.output_PID = output_PUN;
  pub_PUN.publish(&pub_msg_PUN);
  nh.spinOnce();

  //Executa somente se a mensagem não contem aviso de parada emergencial.
  if(EMERGENCY_STOP){
    motorGo(MOTOR_PUN, PARAR, 0);
    motorGo(MOTOR_GAR, PARAR, 0);
  }

  //Verifica se houve comando de RESET via ROS.
  if(RESET)
    reset_PUNGAR();
    
  //Verifica se houve comando de RETRY via ROS.
  if(RETRY)
    retry_PUNGAR();

  //Ececuta controle somente se o ROS está conectado e o PID habilitado.
  if(nh.connected() && PID_enable_PUN){
      
    //Calcula as variáveis para o PID do punho.
    erro_PUN = enc_PUN - setpoint_PUN;

    //Verifica se o punho tem que acionar.
    if (abs(erro_PUN) > tolerance_PUN){

      //Controla o reset do timeout de colisão, caso o PID esteja começando uma operação agora.
      if (!working_PUN){
        working_PUN = true;
        start_PUN = millis();
      }
      
      //Calcula o PWM do punho.
      if(abs(erro_PUN) > DEG2PUL_PUN*90){
        //Se o erro for maior que 90 graus, aciona o PWM máximo.
        output_PUN = PWM_MAX;
      }else{
        //Se o erro for menor que 90 graus, calcula o PID.
        //output_PUN = min(abs(kP_PUN*erro_PUN + kI_PUN*soma_erro_PUN + kD_PUN*var_erro_PUN),PWM_MAX);
        output_PUN = min(abs(kP_PUN*erro_PUN),PWM_MAX);
      }

      //Verifica para qual lado tem que girar. Se o output é positivo, gira no sentido horário.
      if (erro_PUN > 0){
        motorGo(MOTOR_PUN, HOR, output_PUN);
      }else{
        motorGo(MOTOR_PUN, ANTHOR, output_PUN);
      }
    }else{
      //Se dentro da tolerância, mantém parado e marca a flag de tarefa concluída.
      motorGo(MOTOR_PUN, PARAR, 0);
      output_PUN = 0;
      working_PUN = false;
      retries_PUN = 0;

      //Apenas move a garra se o punho está parado.
      if(setpoint_GAR != status_GAR){
        if(setpoint_GAR){
          motorGo(MOTOR_GAR, ANTHOR, PWM_MAX);
          delay(5000);
          nh.logwarn("Garra aberta.");
          setpoint_GAR = status_GAR;
        }else{
          motorGo(MOTOR_GAR, HOR, PWM_MAX);
          delay(5000);
          nh.logwarn("Garra fechada.");
          setpoint_GAR = status_GAR;
        }
      }
    }

    //Acumula o tempo sem pulsos de encoder, caso o PID esteja executando alguma tarefa.
    //Se estourar o tempo entre pulsos enquanto o PID está trabalhando, é porque o motor está forçando em algum obstáculo, então para e desativa o PID.
    if(working_PUN){
      
      pulse_timout_PUN = millis() - start_PUN;
       
      if(pulse_timout_PUN >= time_to_stop){
         motorGo(MOTOR_PUN, PARAR, 0);
         PID_enable_PUN = false;
         lock_PUN = millis();       
         nh.logwarn("PID desativado devido a uma colisão no PUNHO.");
      }
    }
  }else{
     //Se o ROS não está conectado, para os motores.
     motorGo(MOTOR_PUN, PARAR, 0);

     //Se o ROS está conectado, porém o PID está desativado, verifica se o obstáculo foi removido a cada intervalo "delay_lock" de tempo.
     //Essa verificação é feita reativando temporariamente o PID. Caso ainda exista bloqueio, irá cair nesta condicional novamente.
     if(nh.connected() && !PID_enable_PUN){
        if(millis() - lock_PUN >= delay_lock){
            PID_enable_PUN = true;
            nh.logwarn("Tentativa de ativar o PID do PUNHO...");
            
            //Tenta reativar o PID 5 vezes. Se falhar, ativa o RETRY 3 vezes. Se falhar, ativa o RESET.
            retries_PUN += 1;
            if(retries_PUN >= 5){
              RETRY = 1;
              nh.logwarn("rotina de \"RETRY\" ativada para o PUNHO...");
            }

            if(retries_PUN == 8){
              nh.logwarn("Não foi possível atingir a posição desejada para o PUNHO. Rotina de RESET ativada.");
              retries_PUN = 0;
              RETRY = 0;
              RESET = 1;
            }
         
            start_PUN = millis();
        }
        nh.spinOnce();
     }
   }
}

//Função que trata a mensagem recebida, convertendo o ângulo recebido em contagem de pulsos.
void Callback(const custom_msg::set_angles & rec_msg) {
  setpoint_PUN = (rec_msg.set_PUN - DEFAULT_PUN) * DEG2PUL_PUN;
  setpoint_GAR = rec_msg.set_GAR;
  EMERGENCY_STOP = rec_msg.emergency_stop;
  RESET = rec_msg.reset;
  RETRY = rec_msg.retry;
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

  //Contabiliza os pulsos para cada direção. Apenas zera o timeout de pulsos a cada "counter_max" pulsos em uma das direções.
  if (digitalRead(ENC_PUN_B) == HIGH){
    counter_PUN -= 1;
  }else{
    counter_PUN += 1;
  }
     
  if(abs(counter_PUN) == counter_max){
    start_PUN = millis();
    counter_PUN = 0;
  }
}

//Função para resetar os motores. Rotaciona até detectar uma colisão. Esquece os últimos setpoints indicados.
void reset_PUNGAR() {

  RESET_PUN = true;
  RESET_GAR = true;

  start_PUN = millis();
  
  while(RESET_PUN){
    motorGo(MOTOR_PUN, HOR, 70);
    //Botar comando para a garra fechar/abrir, conforme o que definir de padrão.

    pulse_timout_PUN = millis() - start_PUN;

    //nh.logwarn(itoa(pulse_timout_PUN,buf,10));
    
    if(pulse_timout_PUN >= time_to_stop){
      
      motorGo(MOTOR_PUN, PARAR, 0);

      delay(100);
      
      setpoint_PUN =         DEFAULT_PUN;
      enc_PUN =              0;
      erro_PUN =             0;
      output_PUN =           0;
      soma_erro_PUN =        0;
      last_erro_PUN =        0;
      var_erro_PUN =         0;
      counter_PUN =          0;
      working_PUN =          false;

      RESET_PUN = false;
      nh.logwarn("Reset completo no PUNHO.");
    }
    nh.spinOnce();
  }

  RESET = false;
}

//Função para reiniciar o robô e tentar novamente ir até o setpoint.
void retry_PUNGAR(){
  //Salva o setpoint atual.
  last_setpoint_PUN = setpoint_PUN;

  //Chama a função de reset.
  reset_PUNGAR();

  //Atribui novamente o último setpoint.
  nh.logwarn("Nova tentativa a partir da origem.");
  setpoint_PUN = last_setpoint_PUN;
  
  RETRY = false;
}
