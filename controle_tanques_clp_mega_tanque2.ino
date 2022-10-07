//Codigo feito por Felipe Narimatsu Presti
//Controlador: Arduino MEGA
//tanque 01

//inclusao das bibliotecas necessarias
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>
#include <avr/wdt.h>

//inicia os pinos do display
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

//variaveis de iniciacao do cartao sd
const int chipSelect = 53;

//variaveis de auxilio para o salvamento no sd
int dias = 0;
int hora_r = 0;
int hora = 0;
bool estado_setpoint = true;
//variavel de leitura da eeprom
byte valor_eeprom;

bool estado_desce_temp = true;
bool estado_sobe_temp = true;
//variaveis dos botoes do display
int botao = 0;
int soma_cartao = 0;
//variaveis do display e botoes
unsigned long salva_att_tela = 0;
unsigned long tempo_clicar_botao = 0;
bool incremento_bool = false;
bool tela_programacao = false;
bool tela_esvazia = false;
int estado_reset = 0;
bool estado_verif = false;
//bool tela_setpoint = false;
bool aviso_temp_alta_display = false;
bool aviso_temp_baixa_display = false;
int qual_tela_mostrar = 0;

// variaveis da curva de calibração

float expoente = 2;
float fator_b;
float valor_exponencial;
float valor_sensor1, valor_sensor2, valor_sensor3, valor_sensor4, valor_sensor5;
float modulo_sensor;

//------------------------


int erros_maximos_temp; // ------------ numero maximo de leituras erradas ___________
bool tela_esvazia_confirmado = false;
bool barra_ativacoes = false;
bool estado_esvaziar_camara = false;
bool atualiza_millis = false;
bool inicia_verificacao_boias = false;
//definicao pinos do mega
const int LEDVERDE = 43;
const int LEDAMARELO = 45;
const int SIRENE = 35;
const int COMPRESSOR = 37;
const int RESISTENCIA = 42;
const int VALV_TAM = 33;
const int VALV_DISCART = 31;
const int VALV_TORRE = 41;
const int AGUA_RUA = 39;
const int BOMBA_TAMQUE = 38;
const int BOMBA_TORRE = 40;
const int sentor_tanque_alto = 34;
const int sentor_tanque_baixo = 36;
const int sentor_torre_alto = 30 ;
const int sentor_torre_baixo = 32;
const int vcc_sensor_tanque = 24;//////////////////////////// VERIFICAR PINO
const int vcc_sensor_torre = 22;///////////////////////////// VERIFICAR PINO

int fluxo_bomba = 0;
int ESTADO_LIQUIDO = 0;

//enderecamento sensores do tanque
//0x28, 0xBF, 0xD2, 0x95, 0xF0, 0x01, 0x3C, 0xD4
//0x28, 0x44, 0x97, 0x95, 0xF0, 0x01, 0x3C, 0x65
//0x28, 0x0F, 0x5B, 0x95, 0xF0, 0x01, 0x3C, 0xEB



//enderecamento sensores torre
//0x28, 0x02, 0x71, 0x95, 0xF0, 0x01, 0x3C, 0xE4
//0x28, 0x3E, 0x4E, 0x95, 0xF0, 0x01, 0x3C, 0x9F

//0x28, 0x10, 0x1F, 0x95, 0xF0, 0x01, 0x3C, 0x62
//0x28, 0xFF, 0x64, 0x1E, 0x5B, 0xE3, 0xEB, 0x38

// Addresses of 5 DS18B20s

uint8_t sensor1[8] = { 0x28, 0x93, 0x13, 0x03, 0x00, 0x00, 0x00, 0x4C }; //tanque
uint8_t sensor2[8] = { 0x28, 0xFF , 0x64 , 0x1E , 0x23 , 0xCE , 0x96 , 0xBF }; //tanque
uint8_t sensor3[8] = { 0x28, 0xE8, 0x7D, 0x02, 0x00, 0x00, 0x00, 0x10 }; //tanque
uint8_t sensor4[8] = { 0x28, 0x10, 0x1F, 0x95, 0xF0, 0x01, 0x3C, 0x62 }; //torre
uint8_t sensor5[8] = { 0x28, 0xFF, 0x64, 0x1E, 0x5B, 0xE3, 0xEB, 0x38 }; //torre

// Data wire is plugged into digital pin 2 on the Arduino
#define ONE_WIRE_BUS 49

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

int deviceCount = 0;
//float tempC;

// variable to hold device addresses
//DeviceAddress Thermometer;

//variaveis salva millis
unsigned long salva_leitura_temperaturas = 0;
unsigned long salva_leitura_nivel = 0;
unsigned long salva_print_serial = 0;
unsigned long escreve_cartao = 0;
unsigned long salva_delay_30s_ativacao_sirene = 0;
unsigned long salva_permissao_diferenca = 0;
unsigned long check_mudanca_leitura = 0;
unsigned long salva_verifica_resistencia_nivel_baixo = 0;
unsigned long salva_verifica_resistencia_nivel_baixo_repetido = 0;
unsigned long salva_nivel_baixo = 0;
unsigned long salva_verifica_nivel_baixo = 0;
unsigned long salva_delay_atualiza_tela = 0;
unsigned long salva_check_leitura_zero = 0;
unsigned long salva_reset_acc_leitura_zero_sensores = 0;
unsigned long verific_diaria = 0;
unsigned long tempo_para_ler_primeiro = 0;
unsigned long tempo_entre_verificacao_estado = 0;
unsigned long tempo_entre_ativacoes_r = 0;
//variaveis de setpoint de temperatura
float setpoint_temp = EEPROM.read(4);
float setpoint_temp_acc = setpoint_temp;
float valor_diferenca_temp;
float modulo_diferenca;
float temp_backup_tanque_final = 10;
float temp_backup_torre_final = 10;
int incremento = 1;

//variaveis de controle das resistencias
int estadoResistencias = 0;
unsigned long tempo_aquecedor_ligado = 0;
unsigned long tempo_aquecedor_desligado = 0;
unsigned long tempo_resistencia = 0;
bool resistencia_ligada = false;
bool permissao_ativar_resistencias = true;

//variaveis de controle do resfriador
int estadoResfriador = 0;
unsigned long tempo_resfriador = 0;
long acc_output;
unsigned long tempo_resfriador_ligado = 0;
unsigned long tempo_resfriador_desligado = 0;
bool permissao_ativar_compressor = true;

//variaveis de armazenamento de temperatura
float temp1_tanque;
float temp2_tanque;
float temp3_tanque;
float temp1_torre;
float temp2_torre;
float temp_media_torre;
float temp_media_tanque;

//variaveis de armazenamento de nivel
bool nivel_tanque_alto;
bool nivel_tanque_baixo;
bool nivel_torre_alto;
bool nivel_torre_baixo;
bool nivel_tanque_alto_final = 0;

bool nivel_tanque_baixo_final = 0;
bool nivel_torre_alto_final = 0;
bool nivel_torre_baixo_final = 0;
int acc_nivel_tanque_alto = 0;
int acc_nivel_tanque_baixo = 0;
int acc_nivel_torre_alto = 0;
int acc_nivel_torre_baixo = 0;

bool estadoAnteriorAlto = NULL;
bool estadoAnteriorBaixo = NULL;

bool mudancaEstado1 = false;
bool mudancaEstado2 = false;

//variaveis do cartao sd
bool maquina_sem_cartao_sd = false;

//variaveis de controle forcado
bool desliga_resistencia_compressor_forcado = false;

//variaveis de espera para o controle comecar
bool delay_inicio = true;

//variaveis de controle de print na serial
bool printou_tempo = false;

//variaveis de controle das bombas
bool controle_automatico_bombas = true;
unsigned long verificador_5_minutos_nivel_alto = 0;
int permissao_estado_ruim = 0;

//variaveis do sistema de proteaco
int acc_diferenca_torre_tanque = 0;
int diferenca_1_grau = 0;
float temp_backup;
bool permissao_verificacao_inicial = false;
int acc_verifica_resistencia_nivel_baixo = 0;
int erro_resistencia_ligada_nivel_baixo = 0;
int diferenca_1_grau_tanque = 0;
int acc_nivel_baixo = 0;
int incrementador_leitura_zero = 0;

//variaveis auxiliares do sistema de reset dos sensores no loop
unsigned long espera_reset_sensores = 0;
int estado_reset_sensores = 0;
int incrementador_leitura_zero_teste = 0;
bool reset_sensores = false;

void setup() {
  pinMode(SIRENE, OUTPUT);
  pinMode(AGUA_RUA, OUTPUT);
  pinMode(BOMBA_TAMQUE, OUTPUT);
  pinMode(BOMBA_TORRE, OUTPUT);
  pinMode(LEDVERDE, OUTPUT);
  pinMode(LEDAMARELO, OUTPUT);
  pinMode(COMPRESSOR, OUTPUT);
  pinMode(RESISTENCIA, OUTPUT);
  pinMode(VALV_TAM, OUTPUT);
  pinMode(VALV_DISCART, OUTPUT);
  pinMode(VALV_TORRE, OUTPUT);
  pinMode(sentor_tanque_alto, OUTPUT);
  pinMode(sentor_tanque_baixo, OUTPUT);
  pinMode(sentor_torre_alto, OUTPUT);
  pinMode(sentor_torre_baixo, OUTPUT);
  pinMode(vcc_sensor_tanque, OUTPUT);
  pinMode(vcc_sensor_torre, OUTPUT);

  //desligando os pinmode
  digitalWrite(SIRENE, LOW);
  digitalWrite(AGUA_RUA, LOW);
  digitalWrite(BOMBA_TAMQUE, LOW);
  digitalWrite(BOMBA_TORRE, LOW);
  digitalWrite(LEDVERDE, LOW);
  digitalWrite(LEDAMARELO, LOW);
  digitalWrite(COMPRESSOR, LOW);
  digitalWrite(RESISTENCIA, LOW);
  resistencia_ligada = false;
  digitalWrite(VALV_TAM, LOW);
  digitalWrite(VALV_DISCART, LOW);
  digitalWrite(VALV_TORRE, LOW);
  //EEPROM.write(1, 0);//(endereco, valor)  //--------- comentar isso
  //iniciando o display
  lcd.begin(16, 2);
  valor_eeprom = EEPROM.read(2);
  if (valor_eeprom == 100) {
    digitalWrite(SIRENE, HIGH);
    Serial.println("6");
    lcd.setCursor(0, 0);
    lcd.print("MAQUINA");
    lcd.setCursor(0, 1);
    lcd.print("INOPERANTE");
    while (1) {
      if (((analogRead(0)) < 800)) {
        lcd.clear();
        //  Serial.println("maquina resetada");
        EEPROM.write(2, 0);//(endereco, valor)
        lcd.setCursor(0, 0);
        lcd.print("MAQUINA");
        lcd.setCursor(0, 1);
        lcd.print("RESETADA");
        digitalWrite(SIRENE, LOW);
        //digitalWrite(LEDAMARELO, HIGH);
      }

    }
  }
  lcd.setCursor(0, 0);
  lcd.print("Iniciando...");
  //joga o vcc sensor pra low
  digitalWrite(vcc_sensor_tanque, LOW);
  digitalWrite(vcc_sensor_torre, LOW);
  delay(2500);
  //joga o vcc sensor pra high
  digitalWrite(vcc_sensor_tanque, HIGH);
  digitalWrite(vcc_sensor_torre, HIGH);
  delay(2500);
  sensors.begin();  // Start up the library
  Serial.begin(9600);
  // locate devices on the bus
  // Serial.print("Locating devices...");
  //Serial.print("Found ");
  deviceCount = sensors.getDeviceCount();
  // Serial.print(deviceCount, DEC);
  // Serial.println(" devices.");
  // Serial.println("");
  //testes do joao
  digitalWrite(AGUA_RUA, LOW);
  //delay(5000);
  digitalWrite(BOMBA_TAMQUE, LOW);
  digitalWrite(BOMBA_TORRE, LOW);
  digitalWrite(LEDVERDE, HIGH);
  digitalWrite(LEDAMARELO, LOW);
  digitalWrite(SIRENE, LOW);
  digitalWrite(COMPRESSOR, LOW);
  digitalWrite(RESISTENCIA, LOW);
  resistencia_ligada = false;
  digitalWrite(VALV_TAM, LOW);
  digitalWrite(VALV_DISCART, LOW);
  digitalWrite(VALV_TORRE, LOW);
  //    if(!SD.begin(chipSelect)){
  //    while(1){
  //      Serial.println("erro sd");
  //      }
  //  //   }
  for (int i = 0; i < 5; i) {    // ************ iniciando sensores ++++++++++

    // Send command to all the sensors for temperature conversion
    sensors.requestTemperatures();
    temp1_tanque = sensors.getTempC(sensor1);
    temp2_tanque = sensors.getTempC(sensor2);
    temp3_tanque = sensors.getTempC(sensor3);
    temp1_torre = sensors.getTempC(sensor4);
    temp2_torre = sensors.getTempC(sensor5);
    Serial.println("Temp 1: " + String(temp1_tanque));
    Serial.println("Temp 2: " + String(temp2_tanque));
    Serial.println("Temp 3: " + String(temp3_tanque));
    Serial.println("Temp 4: " + String(temp1_torre));
    Serial.println("Temp 5: " + String(temp2_torre));
    modulo_sensor = abs(temp1_tanque);
    valor_exponencial = pow(modulo_sensor, expoente);
    temp1_tanque = 1.004 * modulo_sensor + 1.3805 - 0.0003 * valor_exponencial; //curva de calibração Tdurae _ T2.1 data 11/08/22
    modulo_sensor = abs(temp2_tanque);
    valor_exponencial = pow(modulo_sensor, expoente);
    temp2_tanque =  1.0319 * modulo_sensor - 0.0005 * valor_exponencial; //curva de calibração Tdurae _ T2.2 data 11/08/22
    modulo_sensor = abs(temp3_tanque);
    valor_exponencial = pow(modulo_sensor, expoente);
    temp3_tanque = 1.0108 * modulo_sensor - 0.0003 * valor_exponencial; //curva de calibração Tdurae _ T2.3 data 11/08/22
    temp_media_tanque = (temp1_tanque + temp2_tanque + temp3_tanque) / 3;
    temp_media_torre = (temp1_torre + temp2_torre) / 2;
    salva_leitura_temperaturas = millis();
    i++;
    if (temp1_tanque < 0 || temp2_tanque < 0 || temp3_tanque < 0 || temp1_torre < 0 || temp2_torre < 0) {
      i = 0;
      erros_maximos_temp++;
      if (erros_maximos_temp > 10) {
        i = 10;
        erros_maximos_temp = 0;
      }
    }
    delay(1000);
  }
  if (!SD.begin(chipSelect)) {
    while (1) {
      if (millis() - salva_delay_atualiza_tela > 500) {
        Serial.println("erro sd");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SD AUSENTE");
        lcd.setCursor(0, 1);
        lcd.print("Continuar?");
        if (((analogRead(0)) < 800) || estado_reset == 1) {
          maquina_sem_cartao_sd = true;
          break;
        }
        else {
          soma_cartao++;
          delay(250);
          if (soma_cartao > 60) {
            soma_cartao = 0;
            break;
          }
        }
        salva_delay_atualiza_tela = millis();
      }
    }
  }

  tempo_entre_verificacao_estado = millis();
  //reseta o valor para as ativacoes comecarem 10 segundos depois de acabar o setup

  tempo_para_ler_primeiro = millis();
  wdt_enable(WDTO_8S);
}
void loop() {
  // EEPROM.write(1, 2);
  if (millis() - tempo_clicar_botao > 1000) {
    if ((analogRead(0)) <= 0) { //nao fazer nada
      botao = 0;
    }
    else if ((analogRead(0)) < 80) { // DIREITA
      incremento_bool = !incremento_bool;
      if (incremento_bool == true) {
        incremento = 10;
      }
      if (incremento_bool == false) {
        incremento = 1;
      }
    }
    else if ((analogRead(0)) < 200) { //  CIMA
      setpoint_temp_acc = setpoint_temp_acc + incremento;

      if (setpoint_temp_acc >= 85) {
        setpoint_temp_acc = 85;
      }
    }
    else if ((analogRead(0)) < 400) { //BAIXO
      setpoint_temp_acc = setpoint_temp_acc - incremento;
      if (setpoint_temp_acc <= 5) {
        setpoint_temp_acc = 5;
      }
    }
    else if ((analogRead(0)) < 600) { //ESQUERDA
      if (tela_programacao == true) {
        setpoint_temp = setpoint_temp_acc;
        EEPROM.write(4, setpoint_temp_acc);//(endereco, valor)
        tela_programacao = false;
        File file = SD.open("Tanque01", FILE_WRITE); // Cria / Abre arquivo .txt
        file.print(" Setpoint alterado para: ");
        file.println(setpoint_temp_acc);
        file.close();
      }
      if (tela_esvazia == true) {
        tela_esvazia_confirmado = true;
      }
    }
    else if ((analogRead(0)) < 800) { //SELECT

      //tela_programacao = !tela_programacao;
      qual_tela_mostrar++;
      if (qual_tela_mostrar >= 3) {
        qual_tela_mostrar = 0;
      }
      if (qual_tela_mostrar == 0) {
        //mostra tela leituras
        barra_ativacoes = false;
        estado_esvaziar_camara = false;
        tela_esvazia_confirmado = false;
        tela_programacao = false;
        tela_esvazia = false;
      }
      if (qual_tela_mostrar == 1) {
        //mostra tela setpoint
        tela_programacao = true;
        tela_esvazia = false;
      }
      if (qual_tela_mostrar == 2) {
        //mostra tela esvaziar camara
        tela_programacao = false;
        tela_esvazia = true;
      }
    }
    tempo_clicar_botao = millis();
  }
  if (millis() - salva_leitura_temperaturas > /*120000*/ 10000) {
    sensors.requestTemperatures();
    temp1_tanque = sensors.getTempC(sensor1);
    temp2_tanque = sensors.getTempC(sensor2);
    temp3_tanque = sensors.getTempC(sensor3);
    temp1_torre = sensors.getTempC(sensor4);
    temp2_torre = sensors.getTempC(sensor5);
    Serial.println("Temp 1: " + String(temp1_tanque));
    Serial.println("Temp 2: " + String(temp2_tanque));
    Serial.println("Temp 3: " + String(temp3_tanque));
    Serial.println("Temp 4: " + String(temp1_torre));
    Serial.println("Temp 5: " + String(temp2_torre));
    modulo_sensor = abs(temp1_tanque);
    valor_exponencial = pow(modulo_sensor, expoente);
    temp1_tanque = 1.004 * modulo_sensor + 1.3805 - 0.0003 * valor_exponencial; //curva de calibração Tdurae _ T2.1 data 11/08/22
    modulo_sensor = abs(temp2_tanque);
    valor_exponencial = pow(modulo_sensor, expoente);
    temp2_tanque =  1.0319 * modulo_sensor - 0.0005 * valor_exponencial; //curva de calibração Tdurae _ T2.2 data 11/08/22
    modulo_sensor = abs(temp3_tanque);
    valor_exponencial = pow(modulo_sensor, expoente);
    temp3_tanque = 1.0108 * modulo_sensor - 0.0003 * valor_exponencial; //curva de calibração Tdurae _ T2.3 data 11/08/22
    temp_media_tanque = (temp1_tanque + temp2_tanque + temp2_tanque) / 3;
    temp_media_torre = (temp1_torre + temp2_torre) / 2;

    if (millis() - salva_permissao_diferenca > 60000) {
      if ((temp1_torre >= temp2_torre + 5) || (temp2_torre >= temp1_torre + 5) || temp2_torre < 0 || temp1_torre < 0) {
        diferenca_1_grau++;
        incrementador_leitura_zero++;
        incrementador_leitura_zero_teste++;
      }
      if ((temp1_tanque >= temp2_tanque + 5) || (temp2_tanque >= temp1_tanque + 5) || (temp1_tanque >= temp3_tanque + 5) || (temp3_tanque >= temp1_tanque + 5) || (temp2_tanque >= temp3_tanque + 5) || (temp3_tanque >= temp2_tanque + 5) || temp3_tanque < 0 || temp2_tanque < 0 || temp1_tanque < 0) {
        diferenca_1_grau_tanque++;
        incrementador_leitura_zero++;
        incrementador_leitura_zero_teste++;
      }
      else {
        diferenca_1_grau = 0;
        diferenca_1_grau_tanque = 0;
        incrementador_leitura_zero = 0;
        incrementador_leitura_zero_teste = 0;

      }
      //reset sensores se as leituras forem menor que 0 por dois minutos seguidos
      if (incrementador_leitura_zero_teste >= 5) {
        reset_sensores = true;
        espera_reset_sensores = millis();
        estado_reset_sensores = 0;
      }
      if (reset_sensores == true) {
        if (estado_reset_sensores == 0) {
          digitalWrite(vcc_sensor_tanque, LOW);
          digitalWrite(vcc_sensor_torre, LOW);
          estado_reset_sensores = 1;
          espera_reset_sensores = millis();
        }
        if ((millis() - espera_reset_sensores >= 5000 ) && estado_reset_sensores == 1) {
          digitalWrite(vcc_sensor_tanque, HIGH);
          digitalWrite(vcc_sensor_torre, HIGH);
          estado_reset_sensores = 2;
          espera_reset_sensores = millis();
          salva_leitura_temperaturas = millis();
        }
        if ((millis() - espera_reset_sensores >= 5000 ) && estado_reset_sensores == 2) {
          estado_reset_sensores = 0;
          reset_sensores = false;
          incrementador_leitura_zero_teste = 0;
          sensors.begin();
        }
      }
    }
    salva_leitura_temperaturas = millis();
  }
  //----- verificção diaria
  if ((millis() - verific_diaria) > 7200000) {
    if (diferenca_1_grau >= 22) {
      //      desliga_resistencia_compressor_forcado = true;
      //      lcd.clear();
      //      lcd.setCursor(0, 0);
      //      lcd.print("FALHA SENSORES");
      //      lcd.setCursor(0, 1);
      //      lcd.print("TORRE");
      //      digitalWrite(SIRENE, HIGH);
      //      Serial.println("7");
    }
    if (diferenca_1_grau_tanque >= 22) {
      //      desliga_resistencia_compressor_forcado = true;
      //      lcd.clear();
      //      lcd.setCursor(0, 0);
      //      lcd.print("FALHA SENSORES");
      //      lcd.setCursor(0, 1);
      //      lcd.print("TANQUE");
      //      digitalWrite(SIRENE, HIGH);
      //      Serial.println("8");
    }
    if (incrementador_leitura_zero >= 22) {
      //      desliga_resistencia_compressor_forcado   = true;
      //      lcd.clear();
      //      lcd.setCursor(0, 0);
      //      lcd.print("FALHA SENSORES");
      //      lcd.setCursor(0, 1);
      //      lcd.print("GERAL");
      //      digitalWrite(SIRENE, HIGH);
      //      Serial.println("9");
    }
    verific_diaria = millis();
  }
  if (millis() - salva_reset_acc_leitura_zero_sensores > 86400000) {
    incrementador_leitura_zero = 0;
    salva_reset_acc_leitura_zero_sensores = millis();
  }

  if (millis() - check_mudanca_leitura > 3600000) {
    valor_diferenca_temp = temp_media_tanque - setpoint_temp;
    modulo_diferenca = abs(valor_diferenca_temp);
    if (modulo_diferenca < 2) {
      permissao_verificacao_inicial = false;
    }
    if (permissao_verificacao_inicial == true) {
      if (temp_media_tanque - temp_backup < 0.25) {
        //problema aquecimento, nao conseguiu aquecer
        if (temp_media_torre > 15) {
          if (temp_media_tanque > 15 && temp_media_torre < 100) {
            //            desliga_resistencia_compressor_forcado = true;
            //            lcd.clear();
            //            lcd.setCursor(0, 0);
            //            lcd.print("FALHA");
            //            lcd.setCursor(0, 1);
            //            lcd.print("AQUECIMENTO");
            //            digitalWrite(SIRENE, HIGH);
            File file = SD.open("Tanque01", FILE_WRITE); // Cria / Abre arquivo .txt
            file.println("ERR 1 ");
            file.close();
          }
        }
      }
      if (temp_media_tanque - temp_backup < -0.25) {
        if (temp_media_tanque < 35) {
          //          desliga_resistencia_compressor_forcado = true;
          //          lcd.clear();
          //          lcd.setCursor(0, 0);
          //          lcd.print("FALHA");
          //          lcd.setCursor(0, 1);
          //          lcd.print("RESFRIAMENTO");
          //          digitalWrite(SIRENE, HIGH);
          File file = SD.open("Tanque01", FILE_WRITE); // Cria / Abre arquivo .txt
          file.println("ERR 2 ");
          file.close();
        }
      }
      permissao_verificacao_inicial = true;
      temp_backup = temp_media_tanque;
      check_mudanca_leitura = millis();
    }
  }

  if (millis() - salva_leitura_nivel > 1000) {
    nivel_tanque_alto = digitalRead(sentor_tanque_alto);
    nivel_tanque_baixo = digitalRead(sentor_tanque_baixo);
    nivel_torre_alto = digitalRead(sentor_torre_alto);
    nivel_torre_baixo = digitalRead(sentor_torre_baixo);
    if (nivel_tanque_alto != nivel_tanque_alto_final) {
      acc_nivel_tanque_alto++;
      if (acc_nivel_tanque_alto >= 5) {
        nivel_tanque_alto_final = nivel_tanque_alto;
      }
    }
    else {
      acc_nivel_tanque_alto = 0;
    }
    //filtro leitura nivel tanque baixo
    if (nivel_tanque_baixo != nivel_tanque_baixo_final) {
      acc_nivel_tanque_baixo++;
      if (acc_nivel_tanque_baixo >= 5) {
        nivel_tanque_baixo_final = nivel_tanque_baixo;
      }
    }
    else {
      acc_nivel_tanque_baixo = 0;
    }
    //filtro leitura nivel torre alto
    if (nivel_torre_alto != nivel_torre_alto_final) {
      acc_nivel_torre_alto++;
      if (acc_nivel_torre_alto >= 5) {
        nivel_torre_alto_final = nivel_torre_alto;
      }
    }
    else {
      acc_nivel_torre_alto = 0;
    }
    //filtro leitura nivel torre baixo
    if (nivel_torre_baixo != nivel_torre_baixo_final) {
      acc_nivel_torre_baixo++;
      if (acc_nivel_torre_baixo >= 5) {
        nivel_torre_baixo_final = nivel_torre_baixo;
      }
    }
    else {
      acc_nivel_torre_baixo = 0;
    }
    salva_leitura_nivel = millis();
  }
  //==========CONTROLE DE TEMPERATURA
  if (nivel_tanque_baixo_final == HIGH && nivel_torre_baixo_final == HIGH && temp_media_tanque > setpoint_temp + 0.5) { //incluir variavel de processo
    // Serial.println("ligou compressor");
    if (setpoint_temp < 35 && nivel_tanque_baixo_final == HIGH) {
      if (nivel_torre_baixo_final == HIGH) {
        digitalWrite(COMPRESSOR, HIGH);
      }
      estado_setpoint = true;
      estado_desce_temp = true;
      resistencia_ligada == true;
    }
  }
  if (estado_sobe_temp == true && temp_media_tanque > setpoint_temp + 0.5 ) { //chegando na temp atravez da resistencia
    estado_setpoint = false;
    estado_sobe_temp == false;
    digitalWrite(BOMBA_TAMQUE, LOW);
    digitalWrite(BOMBA_TORRE, LOW);
    digitalWrite(LEDAMARELO, LOW);
    digitalWrite(VALV_TAM, LOW);
    digitalWrite(VALV_TORRE, LOW);
    digitalWrite(RESISTENCIA, LOW);
  }
  //ativacao das resistencias
  if (millis() - tempo_entre_ativacoes_r > 15000) {
    if (nivel_tanque_baixo_final == HIGH && temp_media_torre < 100 && temp_media_tanque < setpoint_temp - 0.5 && estado_esvaziar_camara == false) {
      if (setpoint_temp > 20 && nivel_tanque_baixo_final == HIGH) {
        if (nivel_torre_baixo_final == HIGH) {
          digitalWrite(RESISTENCIA, HIGH);
        }
        estado_setpoint = true;
        estado_sobe_temp = true;
      }
    }
    if (nivel_tanque_baixo_final == LOW || nivel_torre_baixo_final == LOW) {
      digitalWrite(RESISTENCIA, LOW);
    }
    if (estado_desce_temp == true && temp_media_tanque < setpoint_temp - 0.5) { //chegando na temp atravez do compressor
      estado_setpoint = false;
      estado_desce_temp = false;
      digitalWrite(BOMBA_TAMQUE, LOW);
      digitalWrite(BOMBA_TORRE, LOW);
      digitalWrite(LEDAMARELO, LOW);
      digitalWrite(VALV_TAM, LOW);
      digitalWrite(VALV_TORRE, LOW);
      digitalWrite(COMPRESSOR, LOW);
    }
    if (estado_setpoint == true) {
      digitalWrite(LEDAMARELO, HIGH);
    }
    tempo_entre_ativacoes_r = millis();
  }
  if (millis() - salva_verifica_resistencia_nivel_baixo > 15000) {
    if ((resistencia_ligada == true) && (nivel_torre_baixo_final == false)) {
      acc_verifica_resistencia_nivel_baixo++;
      if (acc_verifica_resistencia_nivel_baixo >= 25) {
        acc_verifica_resistencia_nivel_baixo = 0;
        digitalWrite(RESISTENCIA, LOW);
        resistencia_ligada = false;
        erro_resistencia_ligada_nivel_baixo++;
      }
    }
    else {
      acc_verifica_resistencia_nivel_baixo = 0;
    }
    salva_verifica_resistencia_nivel_baixo = millis();
  }
  if (millis() - salva_nivel_baixo > 600000 ) {
    if (nivel_tanque_baixo_final == 0 && estado_esvaziar_camara == false && ESTADO_LIQUIDO > 1) {
      acc_nivel_baixo++;
    }
    salva_nivel_baixo = millis();
  }
  if (millis() - salva_verifica_nivel_baixo > 7200000) {
    if (acc_nivel_baixo >= 11) {
      //      desliga_resistencia_compressor_forcado = true;
      //      lcd.clear();
      //      lcd.setCursor(0, 0);
      //      lcd.print("FALHA BOMBAS");
      //      digitalWrite(SIRENE, HIGH);
      File file = SD.open("Tanque01", FILE_WRITE); // Cria / Abre arquivo .txt
      file.print("ERR3");
      file.close();
    }
    acc_nivel_baixo = 0;
    salva_verifica_nivel_baixo = millis();
  }
  //=================FINAL CONTROLE TEMPERATURA
  //=================MONITORAMENTO ESTADO BOIAS - PROTOTIPO ZECA
  if (estadoAnteriorAlto == NULL && estadoAnteriorBaixo == NULL ) { //Este 'If' executa apenas no primeiro loop
    estadoAnteriorAlto = nivel_torre_alto;
    estadoAnteriorBaixo = nivel_torre_baixo;
  }

  if (estado_setpoint == false) {
    inicia_verificacao_boias = false;
    atualiza_millis = false;
    estado_verif = false;
  }
  if (estado_setpoint == true && nivel_torre_baixo == HIGH ) {
    inicia_verificacao_boias = true;
    estado_verif = true;
    if (atualiza_millis == false) {
      tempo_entre_verificacao_estado = millis();
      atualiza_millis = true;
    }
  }

  if (inicia_verificacao_boias == true) {
    //Verificações de mudança
    if (nivel_torre_baixo == HIGH) {
      if (estado_verif == false) {
        tempo_entre_verificacao_estado = millis();
      }
    }
    if (estado_verif == true) {
      if (nivel_torre_alto != estadoAnteriorAlto) {
        mudancaEstado1 = true;
        estadoAnteriorAlto = nivel_torre_alto;
      }
      if (nivel_torre_baixo != estadoAnteriorBaixo) {
        mudancaEstado2 = true;
        estadoAnteriorBaixo = nivel_torre_baixo;
      }
      //A cada minuto, caso tenha ocorrido alguma mudança, a bool "desliga_resistencia_compressor_forcado" é ativada
      if (millis() - tempo_entre_verificacao_estado > 450000) { //tempo entre verificações boias torre
        if (mudancaEstado1 != true || mudancaEstado2 != true) {
          //desliga_resistencia_compressor_forcado = true;
          mudancaEstado1 = false; //Bools verificadoras resetadas para as próximas checagens
          mudancaEstado2 = false;
        }
      }
    }
  }

  //=================FINAL MONITORAMENTO ESTADO BOIAS - PROTOTIPO ZECA
  //=================DESLIGAMENTOS FORÇADOS
  if (desliga_resistencia_compressor_forcado == true) {
    EEPROM.write(2, 100);//(endereco, valor)
    while (1) {
      Serial.println("acionou proteção!");
      wdt_reset();
      if (millis() - salva_leitura_nivel > 1000) {
        digitalWrite(SIRENE, HIGH);
        Serial.println("2");
        nivel_tanque_alto = digitalRead(sentor_tanque_alto);
        nivel_tanque_baixo = digitalRead(sentor_tanque_baixo);
        nivel_torre_alto = digitalRead(sentor_torre_alto);
        nivel_torre_baixo = digitalRead(sentor_torre_baixo);
        if (nivel_tanque_alto != nivel_tanque_alto_final) {
          acc_nivel_tanque_alto++;
          if (acc_nivel_tanque_alto >= 5) {
            nivel_tanque_alto_final = nivel_tanque_alto;
          }
        }
        else {
          acc_nivel_tanque_alto = 0;
        }
        //filtro leitura nivel tanque baixo
        if (nivel_tanque_baixo != nivel_tanque_baixo_final) {
          acc_nivel_tanque_baixo++;
          if (acc_nivel_tanque_baixo >= 5) {
            nivel_tanque_baixo_final = nivel_tanque_baixo;
          }
        }
        else {
          acc_nivel_tanque_baixo = 0;
        }
        //filtro leitura nivel torre alto
        if (nivel_torre_alto != nivel_torre_alto_final) {
          acc_nivel_torre_alto++;
          if (acc_nivel_torre_alto >= 5) {
            nivel_torre_alto_final = nivel_torre_alto;
          }
        }
        else {
          acc_nivel_torre_alto = 0;
        }
        //filtro leitura nivel torre baixo
        if (nivel_torre_baixo != nivel_torre_baixo_final) {
          acc_nivel_torre_baixo++;
          if (acc_nivel_torre_baixo >= 5) {
            nivel_torre_baixo_final = nivel_torre_baixo;
          }
        }
        else {
          acc_nivel_torre_baixo = 0;
        }
        salva_leitura_nivel = millis();
      }
      if (millis() - salva_leitura_nivel > 1000) {
        nivel_tanque_alto = digitalRead(sentor_tanque_alto);
        nivel_tanque_baixo = digitalRead(sentor_tanque_baixo);
        nivel_torre_alto = digitalRead(sentor_torre_alto);
        nivel_torre_baixo = digitalRead(sentor_torre_baixo);
        if (nivel_tanque_alto != nivel_tanque_alto_final) {
          acc_nivel_tanque_alto++;
          if (acc_nivel_tanque_alto >= 5) {
            nivel_tanque_alto_final = nivel_tanque_alto;
          }
        }
        else {
          acc_nivel_tanque_alto = 0;
        }

        //filtro leitura nivel tanque baixo
        if (nivel_tanque_baixo != nivel_tanque_baixo_final) {
          acc_nivel_tanque_baixo++;
          if (acc_nivel_tanque_baixo >= 5) {
            nivel_tanque_baixo_final = nivel_tanque_baixo;
          }
        }
        else {
          acc_nivel_tanque_baixo = 0;
        }
        //filtro leitura nivel torre alto
        if (nivel_torre_alto != nivel_torre_alto_final) {
          acc_nivel_torre_alto++;
          if (acc_nivel_torre_alto >= 5) {
            nivel_torre_alto_final = nivel_torre_alto;
          }
        }
        else {
          acc_nivel_torre_alto = 0;
        }
        if (nivel_torre_baixo != nivel_torre_baixo_final) {
          acc_nivel_torre_baixo++;
          if (acc_nivel_torre_baixo >= 5) {
            nivel_torre_baixo_final = nivel_torre_baixo;
          }
        }
        else {
          acc_nivel_torre_baixo = 0;
        }
        salva_leitura_nivel = millis();
      }
      if (digitalRead(nivel_torre_alto_final) == false) {
        digitalWrite(VALV_TORRE, HIGH);
        digitalWrite(BOMBA_TORRE, HIGH);
      }
      else {
        digitalWrite(VALV_TORRE, LOW);
        digitalWrite(BOMBA_TORRE, LOW);
      }
      digitalWrite(RESISTENCIA, LOW);
      resistencia_ligada = false;
      digitalWrite(COMPRESSOR, LOW);
      digitalWrite(VALV_TAM, LOW);
      digitalWrite(VALV_DISCART, LOW);
      digitalWrite(AGUA_RUA, LOW);
      digitalWrite(BOMBA_TAMQUE, LOW);
    }
  }
  if (nivel_torre_baixo_final == 0) {
    if (millis() - salva_delay_30s_ativacao_sirene > 30000) {
    }
    if (nivel_torre_alto_final == 1) {
    }
    if (nivel_torre_baixo_final == 0) {
    }
  }
  else {
    salva_delay_30s_ativacao_sirene = millis();
    digitalWrite(SIRENE, LOW);
  }
  if (millis() - salva_print_serial > 10000) {
    Serial.print("temp aquecedor: ");
    Serial.println(tempo_aquecedor_ligado);
    salva_print_serial = millis();
  }
  if (millis() - salva_att_tela > 300) {
    if ((aviso_temp_alta_display == false) || (aviso_temp_baixa_display == false )) {
      Serial.println("Passou if 1");
      if (tela_programacao == false && tela_esvazia == false) {
        if (temp_media_torre <= 0 && temp_media_tanque <= 10 ) {
          Serial.println("Metodo print correto");
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("comunicando");
          lcd.setCursor(0, 1);
          lcd.print("comunicando");
        }
        if (temp_media_torre > 0 && temp_media_tanque <= 10 ) {
          Serial.println("Metodo print correto");
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("T TORRE: ");
          lcd.setCursor(11, 0);
          lcd.print(temp_media_torre);
          lcd.setCursor(0, 1);
          lcd.print("comunicando");
        }
        if (temp_media_torre <= 0 && temp_media_tanque > 0 ) {
          Serial.println("Metodo print correto");
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("comunicando");
          lcd.setCursor(0, 1);
          lcd.print("T TANQUE: ");
          lcd.setCursor(11, 1);
          lcd.print(temp_media_tanque);
        }
        if (temp_media_torre > 0 && temp_media_tanque > 0 ) {
          Serial.println("Metodo print correto");
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("T TORRE: ");
          lcd.setCursor(11, 0);
          lcd.print(temp_media_torre);
          lcd.setCursor(0, 1);
          lcd.print("T TANQUE: ");
          lcd.setCursor(11, 1);
          lcd.print(temp_media_tanque);
        }

      }
    }
    if (aviso_temp_alta_display == true) {
      //      lcd.clear();
      //      lcd.setCursor(0, 0);
      //      lcd.print("TEMPERATURA MAX");
      //      lcd.setCursor(0, 1);
      //      lcd.print("ATINGIDA");
      //      digitalWrite(SIRENE, HIGH);
      //      Serial.println("3");
      File file = SD.open("Tanque01", FILE_WRITE); // Cria / Abre arquivo .txt
      file.print(temp_backup_tanque_final);
      file.print(';');
      file.print(hora);
      file.println(" temperatura maxima atingida");
      file.close();
    }
    if (aviso_temp_baixa_display == true) {
      //      lcd.clear();
      //      lcd.setCursor(0, 0);
      //      lcd.print("TEMPERATURA MIN");
      //      lcd.setCursor(0, 1);
      //      lcd.print("ATINGIDA");
      //      digitalWrite(SIRENE, HIGH);
      File file = SD.open("Tanque01", FILE_WRITE); // Cria / Abre arquivo .txt
      file.print(temp_backup_tanque_final);
      file.print(';');
      file.print(hora);
      file.println(" temperatura minima atingida");
      file.close();
    }
    if (tela_programacao == true && tela_esvazia == false) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("SP TEMP: ");
      lcd.setCursor(10, 0);
      lcd.print(setpoint_temp);
      lcd.setCursor(0, 1);
      lcd.print("NOVO SP: ");
      lcd.setCursor(10, 1);
      lcd.print(setpoint_temp_acc);
    }
    if (tela_esvazia == true) {
      if (tela_esvazia == true && tela_esvazia_confirmado == false) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("ESVAZIAR");
        lcd.setCursor(0, 1);
        lcd.print("A CAMARA?");
      }
      if (tela_esvazia == true && tela_esvazia_confirmado == true) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("ESVAZIANDO");
        estado_esvaziar_camara = true;
      }
    }
    Serial.print("torre      "); Serial.println("tanque      ");
    Serial.print(digitalRead(sentor_torre_alto)); Serial.print("           "); Serial.println(digitalRead(sentor_tanque_alto));
    Serial.print(digitalRead(sentor_torre_baixo)); Serial.print("           "); Serial.println(digitalRead(sentor_tanque_baixo));
    Serial.println();
    salva_att_tela = millis();
  }
  if (estado_esvaziar_camara == true) {
    //desliga o controle automatico das bombas
    controle_automatico_bombas = false;
    //liga a bomba de esvaziar
    if (nivel_tanque_baixo_final == HIGH) //era LOW
    {
      digitalWrite(VALV_DISCART, HIGH);
      digitalWrite(BOMBA_TORRE, HIGH);
      digitalWrite(BOMBA_TAMQUE, HIGH);
      digitalWrite(VALV_TORRE, LOW);
      digitalWrite(VALV_TAM, HIGH);
      digitalWrite(RESISTENCIA, LOW);
      resistencia_ligada = false;
      digitalWrite(COMPRESSOR, LOW);
      barra_ativacoes = true;
      acc_nivel_baixo = 0;
    }
    else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("SISTEMA");
      lcd.setCursor(0, 1);
      lcd.print("VAZIO");
      digitalWrite(VALV_DISCART, LOW);
      digitalWrite(BOMBA_TORRE, LOW);
      digitalWrite(BOMBA_TAMQUE, LOW);
      digitalWrite(VALV_TORRE, LOW);
      digitalWrite(VALV_TAM, LOW);
      digitalWrite(RESISTENCIA, LOW);
      resistencia_ligada = LOW;
      digitalWrite(COMPRESSOR, LOW);
      barra_ativacoes = true;
    }

  }
  else {
    //desliga a bomba
    digitalWrite(VALV_DISCART, LOW);
    barra_ativacoes = false;
    //liga o automatico das bombas
    controle_automatico_bombas = true;
  }
  //verifica a permissao do estado ruim, o estado ruim so entra uma vez e quando sai só ativa de novo quando o nivel do tanque vai pra high e depois volta pra low
  if (permissao_estado_ruim == 1 && nivel_tanque_baixo_final == HIGH) {
    permissao_estado_ruim = 2;
  }
  if (permissao_estado_ruim == 2 && nivel_tanque_baixo_final == LOW) {
    permissao_estado_ruim = 0;
  }

  if (controle_automatico_bombas == true) {                // --------------------- bomba -------------------------- autoo -------------------------------
    if (nivel_tanque_baixo_final == LOW && nivel_torre_baixo_final == HIGH && permissao_estado_ruim == 0) { //ENTRA AGUA ENCHE TORRE
      ESTADO_LIQUIDO = 0;
      Serial.println("ENTROU AQUI");
      permissao_estado_ruim = 1;
    }

    if (nivel_tanque_baixo_final == LOW && nivel_torre_baixo_final == LOW ) { //ENTRA AGUA ENCHE TORRE
      ESTADO_LIQUIDO = 0;
    }

    if (nivel_tanque_baixo_final == LOW && nivel_torre_alto_final == HIGH ) { //ENTRA AGUA ENCHE TANQUE
      ESTADO_LIQUIDO = 1;
    }
    if (nivel_tanque_baixo_final == HIGH) { // para de encher e funciona as duas bombas
      ESTADO_LIQUIDO = 2;
    }
    //===================== controle de nivel torre #########################
    if (ESTADO_LIQUIDO == 0 && estado_esvaziar_camara == false) { //valvulas fechadas, somente a valvula rua aberta
      digitalWrite(BOMBA_TORRE, LOW);
      digitalWrite(BOMBA_TAMQUE, LOW);
      digitalWrite(VALV_DISCART, LOW);
      digitalWrite(VALV_TORRE, LOW);
      digitalWrite(VALV_TAM, LOW);
      digitalWrite(AGUA_RUA, HIGH);
    }
    if (ESTADO_LIQUIDO == 1 && estado_esvaziar_camara == false) { // manda agua pro tanque
      digitalWrite(BOMBA_TORRE, LOW);
      digitalWrite(BOMBA_TAMQUE, HIGH);
      digitalWrite(VALV_DISCART, LOW);
      digitalWrite(VALV_TORRE, LOW);
      digitalWrite(VALV_TAM, HIGH);
      digitalWrite(AGUA_RUA, LOW);
    }
    if (ESTADO_LIQUIDO == 2) { //l circula agua
      if (nivel_torre_baixo_final == 0) { //enche a torre
        fluxo_bomba = 0;
      }
      if (nivel_torre_alto_final == 1) { //esvazia a torre
        fluxo_bomba = 1;
      }
      if (fluxo_bomba == 0 && estado_setpoint == true) {
        digitalWrite(BOMBA_TORRE, HIGH);
        digitalWrite(BOMBA_TAMQUE, LOW);
        digitalWrite(VALV_DISCART, LOW);
        digitalWrite(VALV_TORRE, HIGH);
        digitalWrite(VALV_TAM, LOW);
        digitalWrite(AGUA_RUA, LOW);
        //zera o verificador de 30 segundos do fluxo_bomba == 1
        verificador_5_minutos_nivel_alto = millis();
      }
      if (fluxo_bomba == 1 && estado_setpoint == true) {
        //verificador_30_segundos_nivel_alto = millis();
        //digitalWrite(BOMBA_TORRE, HIGH);//era LOW antes da modificacao dos 30 segundos
        digitalWrite(BOMBA_TAMQUE, HIGH);
        digitalWrite(VALV_DISCART, LOW);
        digitalWrite(VALV_TAM, HIGH);
        digitalWrite(AGUA_RUA, LOW);
        if (millis() - verificador_5_minutos_nivel_alto >= 15000) {
        }
        else {
          digitalWrite(SIRENE, LOW);
        }
        if ((millis() - verificador_5_minutos_nivel_alto >= 40000) && estado_setpoint == true) { //1,5 minutos tanque 2
          digitalWrite(BOMBA_TORRE, LOW);
          digitalWrite(VALV_TORRE, LOW);
        }
        else {
          digitalWrite(BOMBA_TORRE, HIGH);
          digitalWrite(VALV_TORRE, HIGH);
        }
      }
    }
  }

  //PLACEHOLDER
  if(nivel_torre_baixo_final == 1 && nivel_torre_alto_final == 0){
    desliga_resistencia_compressor_forcado = true;
  }
  //================= final controle de niveel ++++++++++++++++++++++++++++
  if (millis() - escreve_cartao > 600000 && maquina_sem_cartao_sd == false) {//faz a leitura na serial e salvamento no SD a cada 60 segundo
    hora = hora + 5; // variavel que salva quantos minutos de teste já se passaram
    if (hora == 60) {
      hora = 0;
      hora_r = hora_r + 1;
    }
    if (hora_r == 24) {
      dias = dias + 1;
      hora_r = 0;
    }
    Serial.println("SALVOU");
    File file = SD.open("Tanque01", FILE_WRITE); // Cria / Abre arquivo .txt
    file.print("Dias; ");
    file.print(dias);
    file.print(';');
    file.print("Horas; ");
    file.print(hora_r);
    file.print(':');
    file.println(hora_r);
    file.print(';');
    file.print("temperatura; ");
    file.println(temp_backup_tanque_final);
    file.close();
    escreve_cartao = millis();
  }
  if (millis() - tempo_para_ler_primeiro > 120000) {
    if (temp_media_torre >= 105) { // desliga limite minimo de temp
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("SUPERAQUECIMENTO");
      digitalWrite(SIRENE, HIGH);
      Serial.println("5");
      File file = SD.open("Tanque01", FILE_WRITE); // Cria / Abre arquivo .txt
      file.print("superaqueceu. em:    Torre = ");
      file.print(temp_media_torre);
      file.print("   Tanque = ");
      file.print(temp_backup_tanque_final);
      file.close();
      desliga_resistencia_compressor_forcado = true;
      aviso_temp_alta_display = true;
    }
    else {
      aviso_temp_alta_display = false;
    }
    if (temp_media_torre <= 5) { // desliga limite minimo de temp
      if (temp1_torre > 0 && temp2_torre > 0) {
        //        lcd.clear();
        //        lcd.setCursor(0, 0);
        //        lcd.print("SUPER RESFRI");
        //        digitalWrite(SIRENE, HIGH);
        //        Serial.println("5");
        File file = SD.open("Tanque01", FILE_WRITE); // Cria / Abre arquivo .txt
        file.print("super RESFRIOU em:    Torre = ");
        file.print(temp_media_torre);
        file.print("   Tanque = ");
        file.print(temp_backup_tanque_final);
        file.close();
        //        desliga_resistencia_compressor_forcado = true;
        aviso_temp_baixa_display = true;
      }
    }
    else {
      aviso_temp_baixa_display = false;
    }
  }
  if (temp1_tanque > 0 && temp2_tanque > 0 && temp3_tanque > 0) {
    temp_backup_tanque_final = temp_media_tanque;
  }

  if (temp1_torre > 0 && temp2_torre > 0) {
    temp_backup_torre_final = temp_media_torre;
  }
  wdt_reset();
  if ( millis() > 14400000) {
    while (1) {
      File file = SD.open("Tanque01", FILE_WRITE); // Cria / Abre arquivo .txt
      file.print("RST4");
      file.close();
    }
  }
} //------------------final LOOOOOOOOOOP--------------
