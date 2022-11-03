//****************** Bibliotecas *****************//
#include <ArduinoJson.h>                      // JSON
#include <TinyGPS++.h>                        // GPS
#include <SoftwareSerial.h>                   // Serial (RX/TX)
#include <ESP8266WiFi.h>                      // Wifi
#include <PubSubClient.h>                     // MQTT

//****************** Variaveis globais ***********//
const char* DEVICE_UUID      = "urn:ngsi-ld:Truck:09b5bb4b-63e7-459f-9e9a-95d4bd1a2808"; //id do dispositivo 
const char* TOPICO_SUBSCRIBE = "/SmartTruck/urn:ngsi-ld:Truck:09b5bb4b-63e7-459f-9e9a-95d4bd1a2808/cmd";   //tópico MQTT de escuta (Case sensitive - SmartTruck)
const char* TOPICO_PUBLISH   = "/SmartTruck/urn:ngsi-ld:Truck:09b5bb4b-63e7-459f-9e9a-95d4bd1a2808/attrs"; //tópico MQTT de envio de informações para Broker
const char* ID_MQTT    = DEVICE_UUID;         // id mqtt (para identificação de sessão) IMPORTANTE: este deve ser único no broker    
const int   DATA_DELAY = 120000 ;             // delay de leitura para envio dos dados (2 min)
std::vector<float> arBatteryMeasurement;      // array de medicoes da bateria
const int          BAURATE = 9600;            // baurate de comunicação serial
unsigned long      dataMillis = millis();     // start
unsigned long      checkMillis = millis();    // leitura dos valores
bool               isBrokerCallback = false;  // sinalizacao do broker para o operador 
                                
//****************** Pinagem *****************//
#define ledRed      16                        // GPIO16 D0
#define ledGreen     0                        // GPIO0  D3
#define ledBlue      2                        // GPIO2  D4
#define gpsRX        4                        // GPIO4  D2
#define gpsTX        5                        // GPIO5  D1
#define battery     A0                        // A0 
#define vibSensor   14                        // GPIO14 D5

//****************** Trator Ligado *****************//
bool isTruckOn               = false;          // Informa q o trator esta em funcionamento
bool validaTruckOn           = false;          // Valida se o trator esta ligado
const int truckDelay         = 60000;          // 1 Min
unsigned long truckStart     = 0;              // Tempo em millisegundos quando o trator iniciou
unsigned long truckFinished  = 0;              // Tempo em millisegundos quando o trator desligou
unsigned long truckDelayStop = 0;              // Tempo em millisegundos p/ desligar

//****************** Status ******************//
bool brokerNotification = false;              // Verifica se chegou alguma notificacao do servidor
enum ledsConfig {                             // Enum de sinalizacao
    startDevice,                              // Inicializacao do dispositivo
    ativo,                                    // Ativo
    mqttWifiLost,                             // Conexao perdida com o MQTT/Wifi
    manutencao,                               // Manutencao Ativa
    outGeofence,                              // Fora do talhao ou fazenda
    lowBattery                                // Bateria baixa
};
ledsConfig ledStatus = startDevice;           // Variavel de sinalizacao        
 
//******************* Wifi *******************//
WiFiClient wifiClient;                        // Objeto wifiClient para gerenciar a conexao Wifi
const char* SSID     = "Sylvio";               // Nome da rede WI-FI
const char* PASSWORD = "15041963";          // Senha da rede WI-FI
  
//******************* MQTT *******************//
PubSubClient mqtt(wifiClient);                // Objeto MQTT, conectado ao WiFI, para genrenciar a conexao MQTT
const char* BROKER_MQTT = "52.7.63.69";       // Endereco para a conexao com o Broker
int BROKER_PORT = 1883;                       // Porta do Broker MQTT

//******************** GPS *******************//
TinyGPSPlus gps;                              // Objeto gps para gerenciamento
SoftwareSerial gpsSerial(gpsRX, gpsTX);       // Objeto gpsSerial para a leitura do RX/TX da placa NEO-6M
  
//****************** Declaracao das Funcoes ******************//
void InitInputs();
void DeviceStatusLeds();
void ShutdownLeds();
void DeviceOKLeds();
void StartDeviceLeds();
void LostConnectingMQTTWiFiLeds();
void InitWiFi();
void ReconectWiFi();
void InitMQTT();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void ReconnectMQTT();
void VerificaConexoesWiFieMQTT(void);
void CheckPosition();
void UpdatePosition(char* position);
bool ValidaDelay();
 
//Function: Inicializacao do Dispositivo
//Parameters: -
//Retrun: -
void setup() 
{
    InitInputs();
    InitWiFi();
    InitMQTT();
}

//Function: loop eterno de execucao para o funcionamento do dispositivo
//Parameters: -
//Retrun: -
void loop() 
{ 
    if(!isBrokerCallback)   
    {
        VerificaConexoesWiFieMQTT();  // Sempre verifica se as conexoes MQTT e WiFi estao funcioando 
    }
    
    DeviceStatusLeds();   

    if(millis() - checkMillis > 5000) // verifica as informacoes a cada 5 seg
    {
        CheckTruckOn();               // Verifica se o trator esta em funcionamento
        CheckBattery();               // Realiza a média e envia os dados
        checkMillis = millis();
    }    
	
    if(ValidaDelay())                 //caso o tempo tenha excedido, manda os dados para o servidor
    {
        CheckPosition();              // Valida/busca a posicao do dispositivo
        UpdateBattery();
        UpdateWorkedHours();
    }
	
    mqtt.loop();
}

//Function: Inicia as entradas/saidas para acionamento e monitoramento
//Parameters: -
//Retrun: -
void InitInputs()
{
    pinMode(ledRed,    OUTPUT);
    pinMode(ledGreen,  OUTPUT);
    pinMode(ledBlue,   OUTPUT);    
    pinMode(vibSensor,  INPUT);

    gpsSerial.begin(BAURATE);
    Serial.begin(BAURATE);
    Serial.println("Esperando dados...");

    DeviceStatusLeds();
}

//Function: Verifica o estado do device para sinalizar o operador
//Parameters: -
//Retrun: -
void DeviceStatusLeds()
{    
  switch(ledStatus){
    case startDevice:
      StartDeviceLeds();
      break;
    case ativo:
      DeviceOKLeds();
      break;
    case mqttWifiLost: 
      LostConnectingMQTTWiFiLeds();
      break;
    case manutencao: 
      ManutencaoLeds();
      break;
    case outGeofence:
      OutGeofenceLeds();
      break;
    case lowBattery:
      LowBatteryLeds();
      break;
  }
}

//Function: Desliga todos os Leds de sinalizacao
//Parameters: -
//Retrun: -
void ShutdownLeds()
{
    analogWrite(ledRed,    0);
    analogWrite(ledGreen,  0);
    analogWrite(ledBlue,   0);
}

//Function: Sinalizacao que o dispositivo foi iniciado
//Parameters: -
//Retrun: -
void StartDeviceLeds()
{
    ShutdownLeds();
    for(int i = 0; i < 3; i++){
        analogWrite(ledRed, 255);     // turn the LED on (HIGH is the voltage level)
        delay(300);                       // wait for a second 200ms
        analogWrite(ledRed, 0);  
        analogWrite(ledGreen, 255);
        delay(300);                   
        analogWrite(ledGreen, 0); 
        analogWrite(ledBlue, 255);   
        delay(300);                   
        analogWrite(ledBlue, 0);
        delay(100);
    }
}

//Function: Sinalizacao que o device esta funcionando corretamente
//Parameters: -
//Retrun: -
void DeviceOKLeds()
{
    ShutdownLeds();
    analogWrite(ledGreen, 255);
}

//Function: Sinalizacao que a rede WiFi nao esta conectada
//Parameters: -
//Return: -
void LostConnectingMQTTWiFiLeds()
{
    ShutdownLeds();
    analogWrite(ledRed,   255);
    analogWrite(ledBlue,  255);
}

//Function: Sinalizacao que o trator precisa de manutencao
//Parameters: -`
//Return: -
void ManutencaoLeds()
{
    ShutdownLeds();
    analogWrite(ledRed,   255);
    analogWrite(ledGreen, 50);
}

//Function: Sinalizacao que o trator precisa de manutencao
//Parameters: -
//Return: -
void OutGeofenceLeds()
{
    ShutdownLeds();
    analogWrite(ledGreen, 255);
    analogWrite(ledBlue,  255);
}

//Function: Sinalizacao que o trator precisa de manutencao
//Parameters: -
//Return: -
void LowBatteryLeds()
{
    ShutdownLeds();
    analogWrite(ledRed,   255);
}
 
//Function: Informacao via serial de conexao com o Wifi
//Parameters: -
//Return: -
void InitWiFi() 
{
    delay(10);
    Serial.println("------Conexao WI-FI------");
    Serial.print("Conectando-se na rede: ");
    Serial.println(SSID);
    Serial.println("Aguarde");
     
    ReconectWiFi();
}

//Function: Realiza a conexao/reconexao do WiFi
//Parameters: -
//Return: -
void ReconectWiFi() 
{
    //se já está conectado a rede WI-FI, nada é feito. 
    //Caso contrário, são efetuadas tentativas de conexão
    if (WiFi.status() == WL_CONNECTED)
        return;

    ledStatus = mqttWifiLost;
         
    WiFi.begin(SSID, PASSWORD); // Conecta na rede WI-FI
     
    while (WiFi.status() != WL_CONNECTED) 
    {
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        ledStatus = ativo;
        
        Serial.println();
        Serial.print("Conectado com sucesso na rede ");
        Serial.print(SSID);
        Serial.print(" - IP obtido: ");
        Serial.println(WiFi.localIP());
    }
}
  
//Function: Inicia os parametros de conexao com o Broker MQTT
//Parameters: -
//Return: -
void InitMQTT() 
{
    mqtt.setServer(BROKER_MQTT, BROKER_PORT);   //informa qual broker e porta deve ser conectado
    mqtt.setCallback(mqtt_callback);            //atribui função de callback (função chamada quando qualquer informação de um dos tópicos subescritos chega)
}
  
//Function: Funcao de callback que sera executada quando o Broker MQTT enviar uma mensagem para o dispositivo
//Parameters: -
//Return: 
void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{
    String msg;
 
    //obtem a string do payload recebido
    for(int i = 0; i < length; i++) 
    {
       char c = (char)payload[i];
       msg += c;
    }

    Serial.print("Message MQTT: ");
    Serial.println(msg);

    isBrokerCallback = true;

    if(msg == "s|manutencao")
    {
        ledStatus = manutencao;
    }
    else if (msg == "s|outGeofence")
    {
        ledStatus = outGeofence;
    }
    else if (msg == "s|lowBattery") 
    {
        ledStatus = lowBattery;
    }
    else
    {
        ledStatus = ativo;
        isBrokerCallback = false;
    }
}
  
//Function: Realiza a conexao/reconexao do MQTT
//Parameters: -
//Return: 
void ReconnectMQTT() 
{
    ledStatus = mqttWifiLost;

    Serial.print("Tentando se conectar ao Broker MQTT: ");
    Serial.println(BROKER_MQTT);
    if (mqtt.connect(ID_MQTT)) 
    {
        Serial.println("Conectado com sucesso ao broker MQTT!");
        mqtt.subscribe(TOPICO_SUBSCRIBE); 
        ledStatus = ativo;
    } 
    else
    {
        Serial.println("Falha ao reconectar no broker.");
        Serial.println("Havera nova tentatica de conexao!");
    }    
}

//Função: verifica o estado das conexões WiFI e ao broker MQTT. 
//        Em caso de desconexão (qualquer uma das duas), a conexão
//        é refeita.
//Parâmetros: nenhum
//Retorno: nenhum
void VerificaConexoesWiFieMQTT(void)
{
    if (WiFi.status() != WL_CONNECTED)
        ReconectWiFi(); //se não há conexão com o WiFI, a conexão é refeita

    if (!mqtt.connected()) 
        ReconnectMQTT(); //se não há conexão com o Broker, a conexão é refeita
}

//Function: Realiza a leitura da posicao atual
//Parameters: -
//Return: -
void CheckPosition() {        
    //realiza a leitura do modulo GOS durante 1 min, pegando todas as infoirmações
    for (unsigned long start = millis(); millis() - start < 1000;) 
    {
      while (gpsSerial.available()) {
        char cIn = gpsSerial.read();
        gps.encode(cIn);
      }
    }

    if(gps.location.isValid())
    {
      if(gps.location.isUpdated())
      {
        char pos[100];
        float lat = gps.location.lat();
        float lng = gps.location.lng();
          
        snprintf(pos, sizeof(pos), "%.06f, %.06f", lat, lng);    
    
        Serial.print("GPS send: ");
        Serial.print(pos);
        Serial.println(".");

        UpdatePosition(pos);
      }
      else 
      {
        Serial.println("GPS não atualizado!");
      }
    }
    else
    {
        Serial.println("GPS invalido!");
    }
}

//Function: Envia o dado de localização para o Broker
//Parameters: string | position | lat e long
//Return: -
void UpdatePosition(char* pos) {
    String data = "loc|"; //topico para envio
    
    data += pos; // adiciona a posicao

    if(mqtt.publish(TOPICO_PUBLISH , data.c_str())){
        Serial.print("Localizacao enviada com sucesso: ");
        Serial.println(data.c_str());
    }
}

//Function: Realiza a leitura da baterria 
//Parameters: string | position | lat e long
//Return: -
void CheckBattery() {
    float readValue = analogRead(battery);

    //descarta numeros muito baixo
    if(readValue < 10)
    {
      Serial.println("Battery: tensão muito baixa.");
      return;  
    }

    float inputVoltage = (readValue * 3.2) / 1023; //calculo para conversao
    float batteryMeasurement = inputVoltage / 0.2; // calculo com base nos resistores (0.2)
    Serial.print(batteryMeasurement);
    Serial.println("V");

    arBatteryMeasurement.push_back(batteryMeasurement);
}

//Function: Envia a media dos valores da bateria para o Broker
//Parameters: -
//Return: -
void UpdateBattery() {
    if(arBatteryMeasurement.size() > 10)
    {
        String data = "bt|"; //topico para envio

        float sum = 0;

        for(int i = 0; i < arBatteryMeasurement.size(); i++)
            sum += arBatteryMeasurement[i];

        float batteryAvg = sum / arBatteryMeasurement.size();
        data += String(batteryAvg);

        if(mqtt.publish(TOPICO_PUBLISH , data.c_str())){
            Serial.print("Bateria enviada com sucesso: ");
            arBatteryMeasurement.erase(arBatteryMeasurement.begin());
        }
        else
        {
            Serial.print("Erro ao enviar o Bateria: ");
        }

        Serial.println(data.c_str());
    }
    else
    {
        Serial.print("Bateria size: ");
        Serial.println(arBatteryMeasurement.size());
    }
}

//Function: Verifica se o trator esta em funcionamento
//Parameters: -
//Return: -
void CheckTruckOn() {
    long vibrationMeasurement = pulseIn(vibSensor, HIGH); 
    Serial.print("Vibration value: ");
    Serial.println(vibrationMeasurement);

    if(vibrationMeasurement > 100 && !validaTruckOn && !isTruckOn) //sinaliza que o trator foi ligado
    {
        Serial.println("Vibration check");
        validaTruckOn = true;
        truckStart = millis();
    }     
    else if(vibrationMeasurement > 100 && validaTruckOn && !isTruckOn && millis() - truckStart > truckDelay) //verifica se nao ha interferenia e o trator esta realmente ligado
    {
        Serial.println("Truck On");
        isTruckOn = true;
        validaTruckOn = false;
    }
    
    if(isTruckOn && vibrationMeasurement < 100 && truckDelayStop != 0 && millis() - truckDelayStop > truckDelay) //trator desligado, armazena o millisegundos
    {
        Serial.println("Truck off");
        isTruckOn = false;
        truckFinished = millis();
        truckDelayStop = 0;
    }
    else if(isTruckOn && truckDelayStop == 0 && vibrationMeasurement < 100) //verifica se realmente o trator foi desligado
    {
        Serial.println("Truck will stop?");
        truckDelayStop = millis();
    }
    else if(isTruckOn && vibrationMeasurement > 100)
    {
        Serial.println("Truck delay stop clear");
        truckDelayStop = 0;
    }
}

//Function: Atualiza as horas trabalhadas no Broker
//Parameters: -
//Return: -
void UpdateWorkedHours() {
    if(!isTruckOn && truckStart > 0 && truckFinished > 0)
    {
        String data = "wk|"; //topico para envio
    
        data += String(truckFinished - truckStart); // adiciona o tempo em millisegundos

        if(mqtt.publish(TOPICO_PUBLISH , data.c_str())){
            Serial.print("WorkedHours enviada com sucesso: ");  
            truckStart = 0;
            truckFinished = 0;  
        }
        else
        {
            Serial.print("Erro ao enviar o WorkedHours: ");
        }

        Serial.println(data.c_str());
    }
    else
    {
        Serial.print("Truck is ");
        Serial.println(isTruckOn);

        Serial.print("Truck started at: ");
        Serial.println(truckStart);

        Serial.print("Truck finished at: ");
        Serial.println(truckFinished);
    }
}

//Function: Verifica o intervalo de transmissao das informacoes
//Parameters: -
//Return: boolean
bool ValidaDelay() {
  if ((millis() - dataMillis) >= DATA_DELAY) {
    dataMillis = millis();
    Serial.println("Delay excedido!");
    return true;
  }
  return false;
}
