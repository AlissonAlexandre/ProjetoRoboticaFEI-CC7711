#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/device.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#define test 0
#define LEFT 0
#define EPUCK_RADIUS 0.037
#define RIGHT 1
#define THETA_ERRO 1
#define MAX_SPEED 6.28
static WbDeviceTag left_motor, right_motor;
#define DISTANCE_SENSORS_NUMBER 4
#define LEDS_NUMBER 10
#define VELOCIDADE_ANGULAR_GRAUS 278.237392796
static WbDeviceTag leds[LEDS_NUMBER];
static const char *leds_names[LEDS_NUMBER] = {"led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"};
#define GROUND_SENSORS_NUMBER 3
static WbDeviceTag ground_sensors[GROUND_SENSORS_NUMBER];
static const char *ground_sensors_names[GROUND_SENSORS_NUMBER] = {"gs0", "gs1", "gs2"};
static WbDeviceTag distance_sensors[DISTANCE_SENSORS_NUMBER];
static const char *distance_sensors_names[DISTANCE_SENSORS_NUMBER] = {"ps0", "ps1", "ps6", "ps7"};



static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}
static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}


static double calcula_distancia(const double *pos1, const double *pos2) {
  return sqrt(pow(pos1[0] - pos2[0], 2) + pow(pos1[1] - pos2[1], 2) + pow(pos1[2] - pos2[2], 2));
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

static void init_devices() {
  int i;
  for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
    wb_distance_sensor_enable(distance_sensors[i], get_time_step());
  }
  for (i = 0; i < LEDS_NUMBER; i++)
    leds[i] = wb_robot_get_device(leds_names[i]);

  for (i = 0; i < GROUND_SENSORS_NUMBER; i++)
    ground_sensors[i] = (WbDeviceTag)0;
  int ndevices = wb_robot_get_number_of_devices();
  for (i = 0; i < ndevices; i++) {
    WbDeviceTag dtag = wb_robot_get_device_by_index(i);
    const char *dname = wb_device_get_name(dtag);
    WbNodeType dtype = wb_device_get_node_type(dtag);
    if (dtype == WB_NODE_DISTANCE_SENSOR && strlen(dname) == 3 && dname[0] == 'g' && dname[1] == 's') {
      int id = dname[2] - '0';
      if (id >= 0 && id < GROUND_SENSORS_NUMBER) {
        ground_sensors[id] = wb_robot_get_device(ground_sensors_names[id]);
        wb_distance_sensor_enable(ground_sensors[id], get_time_step());
      }
    }
  }

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  step();
}


static void stop_robot()
 {
  for(int i = 0; i < 10; i++){
    wb_led_set(leds[i], true);
  }
  while(true){
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
    step();
  }
}

static const double DISTANCIA_MINIMA = 0.01;
static const double TEMPO_PARA_DETECCAO = 2.0;

static bool esta_preso(WbNodeRef robo, double *ultima_posicao, double *tempo_ultimo_movimento) {
  const double *posicao_atual = wb_supervisor_node_get_position(robo);
  double distancia = calcula_distancia(posicao_atual, ultima_posicao);

  if (distancia > DISTANCIA_MINIMA) {
    ultima_posicao[0] = posicao_atual[0];
    ultima_posicao[1] = posicao_atual[1];
    ultima_posicao[2] = posicao_atual[2];
    *tempo_ultimo_movimento = wb_robot_get_time();
    return false;
  }
  if (wb_robot_get_time() - *tempo_ultimo_movimento > TEMPO_PARA_DETECCAO) {
    return true;
  }

  return false;
}

static void recuar_e_girar_para_esquerda() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
  passive_wait(0.5);

  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
  passive_wait(0.5);
}

double getRobotBearing(WbNodeRef robo)
{
    /* Função utilizada para converter matriz de angulos de Euler para orientacao */
    const double *orientacao_robo = wb_supervisor_node_get_orientation(robo);
    double rad = atan2(orientacao_robo[7], orientacao_robo[8]);
    double bearing = (rad - 1.5708) / M_PI * 180.0;
    if (bearing < 0.0) {
        bearing = bearing + 360.0;
	}

    bearing = 360-bearing;
    bearing = bearing + 90;
    if (bearing > 360.0)
        bearing = bearing - 360.0;
    return bearing;
}

double calculaAnguloRotacao(double grausParaODestino, double orientacaoRobo){
  double anguloRotacao = grausParaODestino - orientacaoRobo;
  if (anguloRotacao > 180){
      anguloRotacao = -(360-anguloRotacao);}
  else if (anguloRotacao < -180){
      anguloRotacao = (360+anguloRotacao);}
      
  return anguloRotacao;
}

bool verificaAngulo(const double theta, const double theta2)
{
    if (fabs(theta - theta2) < THETA_ERRO)
    {
        return true;
    } else
    {
        return false;
    }
}

void girarAteOGrauDestino(const double grausAteDestino)
{
	if (!verificaAngulo(grausAteDestino, 0))
	{
		double duration = abs(grausAteDestino) / VELOCIDADE_ANGULAR_GRAUS;
		if (grausAteDestino > 0)
		{
                        wb_motor_set_velocity(left_motor, -MAX_SPEED);
                        wb_motor_set_velocity(right_motor, MAX_SPEED);
		}
		else if (grausAteDestino < 0)
		{
          	              wb_motor_set_velocity(left_motor, MAX_SPEED);
                         wb_motor_set_velocity(right_motor, -MAX_SPEED);
		}
		double start_time = wb_robot_get_time();
		do
		{
			step();
		}
		while (wb_robot_get_time() < start_time + duration);
	}
}


static void alinhar_com_caixa(WbNodeRef robo, const double *posicao_caixa) {
  const double *posicao_robo = wb_supervisor_node_get_position(robo);
  double grausParaODestino = atan2(posicao_caixa[1]-posicao_robo[1], posicao_caixa[0]-posicao_robo[0])*180/M_PI;
  double orientacaoRobo = getRobotBearing(robo);
  double grausParaGirar = calculaAnguloRotacao(grausParaODestino, orientacaoRobo);
  girarAteOGrauDestino(grausParaGirar);

  // Parar motores ao finalizar alinhamento
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
}

static bool tenta_empurrar(WbNodeRef caixa, WbNodeRef robo) {
  const double *posicao_caixa = wb_supervisor_node_get_position(caixa);
  const double *posicao_robo = wb_supervisor_node_get_position(robo);
  
  double ultima_posicao[3] = {posicao_robo[0], posicao_robo[1], posicao_robo[2]};
  double tempo_ultimo_movimento = wb_robot_get_time();

  alinhar_com_caixa(robo, posicao_caixa);

  while (calcula_distancia(posicao_robo, posicao_caixa) >= 0.1 + EPUCK_RADIUS/2) {
    if (esta_preso(robo, ultima_posicao, &tempo_ultimo_movimento)) {
      recuar_e_girar_para_esquerda();
      alinhar_com_caixa(robo, posicao_caixa);
      continue;
    }
    wb_motor_set_velocity(left_motor, -MAX_SPEED);
    wb_motor_set_velocity(right_motor, -MAX_SPEED);
    step();
    posicao_robo = wb_supervisor_node_get_position(robo);
  }
   posicao_caixa = wb_supervisor_node_get_position(caixa);
   wb_motor_set_velocity(left_motor, MAX_SPEED);
   wb_motor_set_velocity(right_motor, MAX_SPEED);
   passive_wait(0.5);
   alinhar_com_caixa(robo, posicao_caixa);
   alinhar_com_caixa(robo, posicao_caixa);
   alinhar_com_caixa(robo, posicao_caixa);

   wb_motor_set_velocity(left_motor, -MAX_SPEED);
   wb_motor_set_velocity(right_motor, -MAX_SPEED);
   passive_wait(4);

  // Verifica se a caixa se moveu
  const double *posicao_final = wb_supervisor_node_get_position(caixa);
  double distancia = calcula_distancia(posicao_caixa, posicao_final);
  printf("Distancia empurrada: %f\n", distancia);
  return distancia > 0.01;
}

int main(int argc, char **argv) {
  wb_robot_init();
  WbNodeRef robo = wb_supervisor_node_get_self();
  WbNodeRef caixa0 = wb_supervisor_node_get_from_def("wooden_box");
  WbNodeRef caixa1 = wb_supervisor_node_get_from_def("wooden_box(1)");
  WbNodeRef caixa2 = wb_supervisor_node_get_from_def("wooden_box(2)");
  WbNodeRef caixa3 = wb_supervisor_node_get_from_def("wooden_box(3)");
  WbNodeRef caixa4 = wb_supervisor_node_get_from_def("tenta_empurrarwooden_box(4)");
  WbNodeRef caixa5 = wb_supervisor_node_get_from_def("wooden_box(5)");
  WbNodeRef caixa6 = wb_supervisor_node_get_from_def("wooden_box(6)");
  WbNodeRef caixa7 = wb_supervisor_node_get_from_def("wooden_box(7)");
  WbNodeRef caixa8 = wb_supervisor_node_get_from_def("wooden_box(8)");
  WbNodeRef caixa9 = wb_supervisor_node_get_from_def("wooden_box(9)");
    
  WbNodeRef caixas[10] = {caixa0, caixa1, caixa2, caixa3, caixa4, caixa5, caixa6, caixa7, caixa8, caixa9};

  init_devices();

  printf("Iniciando teste das caixas...\n");
  bool achou = false;
  while(achou == false){
  for (int i = 0; i < 10; i++) {
  if (caixas[i] == NULL) {
      printf("Caixa %d não encontrada.\n", i);
      continue;
    }

    printf("Testando caixa %d...\n", i);
    if (tenta_empurrar(caixas[i], robo)) {
      printf("Caixa %d pode ser empurrada!\n", i);
      stop_robot();
      break;
    } else {
      printf("Caixa %d NÃO pode ser empurrada.\n", i);
    }
  }
}
  printf("Teste finalizado.\n");
  wb_robot_cleanup();
  return EXIT_SUCCESS;
}