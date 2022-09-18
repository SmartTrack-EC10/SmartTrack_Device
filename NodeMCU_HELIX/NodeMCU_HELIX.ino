//****************** Bibliotecas *****************//
#include <ArduinoJson.h>
#include <TinyGPS++.h>                            // GPS
#include <SoftwareSerial.h>                       // Serial (RX/TX)
#include <ESP8266WiFi.h>                          // Wifi
#include <PubSubClient.h>                         // MQTT

//****************** Variaveis globais *****************//
const char* DEVICE_UUID      = "2feefcf6-b7c8-470f-a628-d92300ef64c4"   //id do dispositivo 
const char* TOPICO_SUBSCRIBE = "/SmartTruck/" + DEVICE_UUID +  "/cmd"   //tópico MQTT de escuta (Case sensitive - SmartTruck)
const char* TOPICO_PUBLISH   = "/SmartTruck/" + DEVICE_UUID +  "/attrs" //tópico MQTT de envio de informações para Broker
const char* ID_MQTT          = DEVICE_UUID // id mqtt (para identificação de sessão) IMPORTANTE: este deve ser único no broker    
const int   DATA_DELAY       = 60000       // delay de leitura para envio dos dados

std::vector<float> arBatteryMeasurement;   // array de medicoes da bateria
const int     BAURATE = 9600;              // baurate de comunicação serial
unsigned long dataMillis = millis();       // start 
                                
//****************** Pinagem *****************//
#define ledGreen    16                        // GPIO16 D0
#define ledYellow    0                        // GPIO0  D3
#define ledRed       2                        // GPIO2  D4
#define gpsRX        4                        // GPIO4  D2
#define gpsTX        5                        // GPIO5  D1
#define battery     A0                        // A0 

//****************** Status ******************//
bool brokerNotification = false;              // Verifica se chegou alguma notificacao do servidor
enum ledsConfig {                             // Enum de sinalizacao
    startDevice,                              // Inicializacao do dispositivo
    ok,                                       // Todas conexoes OK
    mqttLost,                                 // Conexao perdida com o MQTT
    wifiLost                                  // Conexao perdida com o Wifi
};
ledsConfig ledStatus = startDevice;           // Variavel de sinalizacao        
 
//******************* Wifi *******************//
WiFiClient wifiClient;                        // Objeto wifiClient para gerenciar a conexao Wifi
const char* SSID     = "";              // Nome da rede WI-FI
const char* PASSWORD = "";            // Senha da rede WI-FI
  
//******************* MQTT *******************//
PubSubClient mqtt(wifiClient);                // Objeto MQTT, conectado ao WiFI, para genrenciar a conexao MQTT
const char* BROKER_MQTT = "";     // Endereco para a conexao com o Broker
int BROKER_PORT = 1883;                       // Porta do Broker MQTT

//******************** GPS *******************//
TinyGPSPlus gps;                              // Objeto gps para gerenciamento
SoftwareSerial gpsSerial(gpsRX, gpsTX, false);     // Objeto gpsSerial para a leitura do RX/TX da placa NEO-6M
  
//****************** Declaracao das Funcoes ******************//
void InitLeds();
void DeviceStatusLeds();
void ShutdownLeds();
void DeviceOKLeds();
void StartDeviceLeds();
void LostConnectingWiFiLeds();
void LostConnectingMQTTLeds(); 
void InitSerial();
void InitWiFi();
void ReconectWiFi();
void InitMQTT();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void ReconnectMQTT();
void VerificaConexoesWiFieMQTT(void);
void CheckPosition();
void SendPosition(char* position);
bool ValidaDelay();
 
//Function: Inicializacao do Dispositivo
//Parameters: -
//Retrun: -
void setup() 
{
    InitSerial();
    InitLeds();
    InitWiFi();
    InitMQTT();
}

//Function: loop eterno de execucao para o funcionamento do dispositivo
//Parameters: -
//Retrun: -
void loop() 
{ 
    VerificaConexoesWiFieMQTT(); // Sempre verifica se as conexoes MQTT e WiFi estao funcioando
    
    DeviceOKLeds();              // Informa que tudo esta funcionando corretamente

    //caso o tempo tenha excedido, manda os dados para o servidor
    if(ValidaDelay())
    {
        CheckBattery();          // Realiza a média e envia os dados
        CheckPosition();         // Valida/busca a posicao do dispositivo
    }
    else
    {
        Serial.println("Aguardando 1min!");
    }

    UpdateBattery();
    mqtt.loop();
    delay(5000);
}

//Function: Inicia as saidas para acionamento dos Leds
//Parameters: -
//Retrun: -
void InitLeds()
{
    pinMode(ledGreen,  OUTPUT);
    pinMode(ledYellow, OUTPUT);
    pinMode(ledRed,    OUTPUT);

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
    case ok:
      digitalWrite(ledGreen, HIGH); //tudo funcionando 
      break;
    case mqttLost: 
      LostConnectingWiFiLeds();
      break;
    case wifiLost: 
      LostConnectingMQTTLeds();
      break;
  }
}

//Function: Desliga todos os Leds de sinalizacao
//Parameters: -
//Retrun: -
void ShutdownLeds()
{
    digitalWrite(ledGreen,  LOW);
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledRed,    LOW);

    delay(200);
}

//Function: Sinalizacao que o device esta funcionando corretamente
//Parameters: -
//Retrun: -
void DeviceOKLeds(){
    digitalWrite(ledGreen, HIGH);     
    delay(200);                      
}

//Function: Sinalizacao que o dispositivo foi iniciado
//Parameters: -
//Retrun: -
void StartDeviceLeds()
{
    ShutdownLeds();

    for(int i = 0; i < 3; i++){
        digitalWrite(ledGreen, HIGH);     // turn the LED on (HIGH is the voltage level)
        delay(200);                       // wait for a second 200ms
        digitalWrite(ledGreen, LOW);  
        digitalWrite(ledYellow, HIGH);
        delay(200);                   
        digitalWrite(ledYellow, LOW); 
        digitalWrite(ledRed, HIGH);   
        delay(200);                   
        digitalWrite(ledRed, LOW);
        delay(500); 
    }
}

//Function: Sinalizacao que a rede WiFi nao esta conectada
//Parameters: -
//Return: -
void LostConnectingWiFiLeds()
{
    ShutdownLeds();

    digitalWrite(ledYellow, HIGH);     // turn the LED on (HIGH is the voltage level)
    delay(200);                        // wait for a second 100ms 
    digitalWrite(ledYellow, LOW);
    delay(200);                   
    digitalWrite(ledYellow, HIGH);   
    delay(200);                   
    digitalWrite(ledYellow, LOW);
}

//Function: Sinalizacao que a rede MQTT nao esta conectada
//Parameters: -
//Return: -
void LostConnectingMQTTLeds()
{
    ShutdownLeds();

    digitalWrite(ledYellow, HIGH);     // turn the LED on (HIGH is the voltage level)
    delay(200);                        // wait for a second 100ms 
    digitalWrite(ledYellow, LOW);
    delay(200);                   
    digitalWrite(ledYellow, HIGH);   
    delay(200);                   
    digitalWrite(ledYellow, LOW);
    delay(200);                   
    digitalWrite(ledYellow, HIGH);   
    delay(200);                   
    digitalWrite(ledYellow, LOW);     
}
  
//Function: Inicializacao da Serial p/ debug
//Parameters: -
//Return: -
void InitSerial() 
{
    gpsSerial.begin(BAURATE);
    Serial.begin(BAURATE);
    Serial.println("Esperaando dados...");    
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

    ledStatus = wifiLost;
         
    WiFi.begin(SSID, PASSWORD); // Conecta na rede WI-FI
     
    while (WiFi.status() != WL_CONNECTED) 
    {
        DeviceStatusLeds();
        delay(1000);
        Serial.print(".");
    }
   
    Serial.println();
    Serial.print("Conectado com sucesso na rede ");
    Serial.print(SSID);
    Serial.print(" - IP obtido: ");
    Serial.println(WiFi.localIP());
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
}
  
//Function: Realiza a conexao/reconexao do MQTT
//Parameters: -
//Return: 
void ReconnectMQTT() 
{
    while (!mqtt.connected()) 
    {
        ledStatus = mqttLost;

        Serial.print("* Tentando se conectar ao Broker MQTT: ");
        Serial.println(BROKER_MQTT);
        if (mqtt.connect(ID_MQTT)) 
        {
            Serial.println("Conectado com sucesso ao broker MQTT!");
            mqtt.subscribe(TOPICO_SUBSCRIBE); 
        } 
        else
        {
            DeviceStatusLeds();
            Serial.println("Falha ao reconectar no broker.");
            Serial.println("Havera nova tentatica de conexao!");
        }
    }
}

//Função: verifica o estado das conexões WiFI e ao broker MQTT. 
//        Em caso de desconexão (qualquer uma das duas), a conexão
//        é refeita.
//Parâmetros: nenhum
//Retorno: nenhum
void VerificaConexoesWiFieMQTT(void)
{
    if (!mqtt.connected()) 
        ReconnectMQTT(); //se não há conexão com o Broker, a conexão é refeita

    if (WiFi.status() != WL_CONNECTED)
        ReconectWiFi(); //se não há conexão com o WiFI, a conexão é refeita
}

//Function: Envia o dado de localização para a Broker
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
      if(gps.location.isUpdated()
      {
        char pos[100];    
        float lat = gps.location.lat();
        float lng = gps.location.lng();
          
        snprintf(pos, sizeof(pos), "%.06f, %.06f", lat, lng);    
    
        Serial.print("GPS send: ");
        Serial.print(pos);
        Serial.println(".");
      }
      else 
      {
        Serial.print("GPS não atualizado!");
      }

      SendPosition(pos);
    }
    
    Serial.println();
    Serial.println(gpsSerial.available());
}

//Function: Envia o dado de localização para a Broker
//Parameters: string | position | lat e long
//Return: -
void SendPosition(char* pos) {
    String data = "loc|"; //topico para envio

    //by test using default values
    data += pos;

    if(mqtt.publish(TOPICO_PUBLISH , data.c_str())){
        Serial.print("Localizacao enviada com sucesso: ");
        Serial.println(data.c_str());
    }
}

//Function: Realiza a leitura da bateria
//Parameters: -
//Return: -
void UpdateBattery() {
    float readValue = analogRead(battery);

    //descarta numeros muito baixo
    if(readValue < 10)
    {
      Serial.println("Battery: tensão muito baixa.")ç
      return;  
    }

    float inputVoltage = (readValue * 3.2) / 1023; //calculo para conversao
    float batteryPorcentage = inputVoltage / 0.2; // calculo com base nos resistores (0.2)
    Serial.print(measurement);
    Serial.println("V");
}


//Function: Envia o dado de localização para a Broker
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
