TaskHandle_t Task1;
#include <SimpleFOC.h>
#include <ADXL345_WE.h>
#include <ADS1256.h>
////////////////////////ADS1256

#define USE_SPI SPI


ADS1256 A(33, 0, 21, 32, 2.500, &USE_SPI);  //DRDY, RESET, SYNC(PDWN), CS, VREF(float), SPI bus.  //RP2040 Pico W - OK

long rawConversion = 0;  //24-bit raw value
float voltageValue = 0;  //human-readable floating point value

int pgaValues[7] = { PGA_1, PGA_2, PGA_4, PGA_8, PGA_16, PGA_32, PGA_64 };  //Array to store the PGA settings
float pgaSelection = 0;                                                     //Number used to pick the PGA value from the above array
float pgaSelection_old = 0;

String registers[11] = {
  "STATUS",
  "MUX",
  "ADCON",
  "DRATE",
  "IO",
  "OFC0",
  "OFC1",
  "OFC2",
  "FSC0",
  "FSC1",
  "FSC2"
};  //Array to store the registers

int registerToRead = 0;        //Register number to be read
int registerToWrite = 0;       //Register number to be written
int registerValueToWrite = 0;  //Value to be written in the selected register


////////////////////Acelerometro
bool spi = true;
ADXL345_WE myAcc = ADXL345_WE(4, spi);

//////////////////Simple FOC


// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(7);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(13, 27, 14, 12);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, 5);


//target variable
float target_velocity = 0;
float motorEnable = 0;
float velocidade_real = 0;
float posicao_eixo = 0;
float tempo_velocidade = 0;
float posicao_anterior = 0;
float tempo_velocidade_anterior = 0;
float mode = 0;
bool flag_case_1 = 0;
bool flag_case_2 = 0;
int teste = 0;

float channels[2];
xyzFloat raw, g;


// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}
void doFilter(char* cmd) {
  command.scalar(&motor.LPF_velocity.Tf, cmd);
}
void doControlP(char* cmd) {
  command.scalar(&motor.PID_velocity.P, cmd);
}
void doControlI(char* cmd) {
  command.scalar(&motor.PID_velocity.I, cmd);
}
void doControlD(char* cmd) {
  command.scalar(&motor.PID_velocity.D, cmd);
}
void doPGA(char* cmd) {
  command.scalar(&pgaSelection, cmd);
}
void doMode(char* cmd) {
  command.scalar(&mode, cmd);
}
void doVoltage(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
}



void Task1code(void* parameter) {
  for (;;) {
    //Loop que roda no CORE0 da ESP32. Esse core é responsável por tarefas de comunicação, processamento e aquisição de dados.
    command.run();                      //Leitura e busca na serial por comandos
    posicao_eixo = motor.shaftAngle();  //A posição é calculada uma outra vez fora de loopFOC para maior precisão
    velocidade_real = motor.shaftVelocity();
    float torque_estimado = motor.voltage.q;


    //Testa se o PGA mudou para o set
    if (pgaSelection != pgaSelection_old) {
      A.setPGA(pgaValues[int(pgaSelection)]);
      pgaSelection_old = pgaSelection;
    }

    //switch case para configurar a aquisição de dados
    switch (int(mode)) {
      case 3:
        A.sendDirectCommand(SELFCAL);
        break;
      case 2:
        myAcc.getGValues(&g);
        break;
      case 0:
        if (!flag_case_1) {
          A.setMUX(DIFF_0_1);
          flag_case_1 = 1;
          flag_case_2 = 0;
        }
        teste = (A.readSingle());
        break;
      case 1:
        if (!flag_case_2) {
          A.setMUX(DIFF_2_3);
          flag_case_2 = 1;
          flag_case_1 = 0;
        }
        teste = (A.readSingle());

        break;
    }
    //Print dos dados
    Serial.println(String(target_velocity) + "\t" + String(velocidade_real) + "\t" + String(posicao_eixo) + "\t" + String(g.x) + "\t" + String(g.y) + "\t" + String(g.z) + "\t" + String(teste) + "\t" + String(torque_estimado));

    vTaskDelay(1);  // solução prática para evitar que os núcleos da ESP32 façam competição por acessos a periféricos comuns (ex barramento SPI), dando tempo para o CORE1 acesse o que for necessário.
  }
}


void setup() {
  //Inciando comunicação serial com baud de 230400
  Serial.begin(115200);
  // enable more verbose output for debugging
  SimpleFOCDebug::enable(&Serial);

  // inicializa AS5047
  sensor.init();
  // link motor e sensor
  motor.linkSensor(&sensor);

  // Configuração do driver
  // Tensão de entrada da fonte no driver [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link do motor e driver
  motor.linkDriver(&driver);

  // set do tipo de controle do FOC
  motor.controller = MotionControlType::velocity;

  // Configurações e parâmetos do controlador
  motor.PID_velocity.P = 0.4f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // Tensão máxima entrege ao motor. Valores acima de 3 volts, com o motor utilizado pelo LEA, aquecem muito o driver
  motor.voltage_limit = 4.5;
  // jerk control com rampa de tensão
  motor.PID_velocity.output_ramp = 7000000;

  //Filtro passa baixa da leitura de velocidade. Ideal para o motor e sensor utilizados é 0.005f (empírico)
  motor.LPF_velocity.Tf = 0.005f;

  // inicializa o motor
  motor.init();
  // encontra zero do sensor e alinha o motor
  motor.initFOC();

  // possibilidades para o commander serial
  command.add('T', doTarget, "Setpoint velocidade");
  command.add('F', doFilter, "Filtro velocidade");
  command.add('P', doControlP, "Ganho proporcional");
  command.add('I', doControlI, "Ganho integrativo");
  command.add('D', doControlD, "Ganho derivativo");
  command.add('G', doPGA, "PGA ADC");
  command.add('M', doMode, "Modo de aquisição");
  command.add('C', doVoltage, "Tensão máxima");



  Serial.println(F("Freio iniciado"));
  _delay(1000);

  //Inicializa ADXL345
  if (!myAcc.init()) {
    Serial.println("ADXL345 não encontrado!");
  }
  myAcc.setDataRate(ADXL345_DATA_RATE_3200);
  delay(100);
  Serial.print("Data rate: ");
  Serial.print(myAcc.getDataRateAsString());
  myAcc.setRange(ADXL345_RANGE_2G);
  Serial.print("  /  g-Range: ");
  Serial.println(myAcc.getRangeAsString());
  Serial.println();

  //Inicializa ADS1256
  A.InitializeADC();

  //Set inicial do ADS1256, é ajustado posteriormente via Serial
  //PGA
  A.setPGA(PGA_64);  //0b00000000 - DEC: 0
  //--------------------------------------------

  //canal selecionado pelo multiplexador
  A.setMUX(DIFF_0_1);  //0b01100111 - DEC: 103
  //--------------------------------------------

  //Set DRATE (taxa de dados)
  A.setDRATE(DRATE_2000SPS);  //0b00010011 - DEC: 19
  //--------------------------------------------

  //Confirma os valores gravados nos registradores
  Serial.print("PGA: ");
  Serial.println(A.getPGA());
  delay(100);
  //--
  Serial.print("MUX: ");
  Serial.println(A.readRegister(MUX_REG));
  delay(100);
  //--
  Serial.print("DRATE: ");
  Serial.println(A.readRegister(DRATE_REG));
  delay(100);


  xTaskCreatePinnedToCore(
    //Atribui o task1 ao CORE0 da ESP32
    Task1code,
    "Task1",
    10000,
    NULL,
    0,
    &Task1,
    0);
  delay(1000);
}



void loop() {
  // loop que roda no CORE1 da ESP32. Esse core é responsável por todas as tarefas relacionadas ao controle do rotor e somente elas.

  motor.loopFOC();              //Abstração do controle
  motor.move(target_velocity);  //Setpoint da velocidade do eixo obtida pela serial
  vTaskDelay(0);
}
