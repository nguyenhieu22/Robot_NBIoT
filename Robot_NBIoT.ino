#include <String.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#define RXD2 16
#define TXD2 17
#define MYPORT_TX 18    
#define MYPORT_RX 19    

TaskHandle_t TASK_HandleOne = NULL;

#define WIFI_SSID "Technology2"
#define WIFI_PASSWORD "hieu1234"

const char *ID = "iot30-ahwwcd";
const char *sub = "esp8266/client";
const char *pub = "iot/66";

const int mqtt_port = 8883;

WiFiClientSecure espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];


const char *mqtt_server ="iot30-ahwwcd.a02.usw2.aws.hivemq.cloud";
const char* mqtt_username = "DATN24"; 
const char* mqtt_password = "Hieu123@"; 

StaticJsonDocument<100> doc;//khoi tao Json.
EspSoftwareSerial::UART myPort;//UART ao;

bool isSerial2Available = true;
String rev;
int i=0;
char connect_cmd[120];
char dataLength[120];
int Netword_State=1;//Bien trang thai dung song WiFi hoac NB-IoT.
int wifi_dem=0;//Bien dem so lan ket noi WiFi.
unsigned int h2s=0;//bien du lieu khi H2S.
unsigned int co2=0, tvoc=0;//Bien due lieu CO2, TVOC.
float e,n;//Bien du lieu vi do(N), kinh do(E).

//Khai bao mot so ham con
void getDevice();//Ham con kiem tra SIM7028
bool Wait_Response(String expected_answer="OK", unsigned int timeout=4000);//Ham con doi phan hoi SIM
void TASK_ONE(void *param);

/*-------------------------------------SETUP***********************************/
void setup() {
  pinMode(2, OUTPUT);
  Serial.begin(9600);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); 
  myPort.begin(9600, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
  if (!myPort) 
  { 
  Serial.println("Loi UART STM32"); 
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");
  unsigned long ms = millis();
  while (WiFi.status() != WL_CONNECTED)//Ket noi den WiFi.
  {
    Serial.print(".");
    wifi_dem++;
    delay(300);
    if(wifi_dem>=20)//Duoc phep ket noi WiFi 21 lan.
    {
      Netword_State = 1;//Trang thai chuyen sang ket noi NB-IoT.
      break;
    }
    else 
      Netword_State = 0;//Trang thai ket noi WiFi.
  }
  switch (Netword_State)//Kiem tra trang thai.
  {
    case 0: digitalWrite(2, HIGH);
            Serial.println();
            Serial.print("Connected with IP: ");
            Serial.println(WiFi.localIP());
            Serial.println();
    
            espClient.setInsecure();
            client.setServer(mqtt_server, mqtt_port);
            client.setCallback(callback);break;
    case 1: while (isSerial2Available) 
            {
              getDevice();
            }
            isSerial2Available = true;
            Serial.print("Get Network...");
            while (isSerial2Available) 
            {
              Serial.print(".");
              SentMessage("AT+CREG?");
              delay(1000);break;
            }
  }
  //Thiet lap task RTOS
  xTaskCreate(
    TASK_ONE,
    "TaskOne",
    12 * 1024,
    NULL,
    1,
    &TASK_HandleOne);
}
unsigned long timeUpdata=millis();
void loop() {
  switch(Netword_State)
  {
    case 0:   if (!client.connected()) 
              {
                reconnect();
              }
              client.loop();
              if(millis()-timeUpdata>2000)
              {
                char msgdata[120]; 
                snprintf(msgdata,sizeof(msgdata),"{\"C\":%d,\"T\":%d,\"H\":%d,\"N\":%.5f,\"E\":%.5f}",co2, tvoc, h2s, n, e);
                publishMessage(pub, msgdata, true);
                timeUpdata=millis();
              }
              break;
    case 1:   isSerial2Available = true;             
              Serial.print("Get Network...");
              while (isSerial2Available) 
              {
                Serial.print(".");
                SentMessage("AT+CREG?");
                delay(1000);
              }
              digitalWrite(2, LOW);
              Serial.println("AT+CSQ");//Kiem tra chat luong song NB-IoT
              Serial2.println("AT+CSQ");
              Wait_Response();
              delay(800);

              Serial2.println("AT+CMQTTSTART");
              delay(1000);

              snprintf(connect_cmd, sizeof(connect_cmd), "AT+CMQTTACCQ=0,%s,1", ID);
              Serial2.println(connect_cmd);
              delay(1000);

              snprintf(connect_cmd, sizeof(connect_cmd), "AT+CMQTTCONNECT=0,tcp://%s:8883,60,1,%s,%s", mqtt_server,mqtt_username,mqtt_password);
              Serial.println("Connect to tcp:");
              Serial.println(connect_cmd);
  
              isSerial2Available = true;
              while (isSerial2Available) 
              {
                Serial.print(".");
                int k = SentMessage(connect_cmd);
                Serial.print("đang ket noi vao mqtt:");
                Serial.println(k);  
                delay(2000);
              }
              digitalWrite(2, HIGH); 
              snprintf(dataLength, sizeof(dataLength), "%d", strlen(pub));
              snprintf(connect_cmd, sizeof(connect_cmd), "AT+CMQTTTOPIC=0,%s", dataLength);
              input_message(connect_cmd, pub);
              delay(3000);

              snprintf(dataLength, sizeof(dataLength), "%d", strlen(pub));
              snprintf(connect_cmd, sizeof(connect_cmd), "AT+CMQTTSUB=0,%s,0", dataLength);
              input_message(connect_cmd, sub);
              delay(3000);

              char msgdata[100];
              snprintf(msgdata,sizeof(msgdata),"{\"C\":%d,\"T\":%d,\"H\":%d,\"N\":%.5f,\"E\":%.5f}", co2,tvoc, h2s, n, e);
              snprintf(dataLength, sizeof(dataLength), "%d", strlen(msgdata));
              snprintf(connect_cmd, sizeof(connect_cmd), "AT+CMQTTPAYLOAD=0,%s", dataLength);
              Serial.println(connect_cmd);
              input_message(connect_cmd, msgdata);
              delay(3000);

              Serial.println("");
              Serial.println("dang tai");
              Serial.println("AT+CMQTTPUB=0,,60");
              Serial2.println("AT+CMQTTPUB=0,2,60");//Dung Qos2
              Wait_Response("OK",2000);
              delay(500);

              Serial.println("AT+CMQTTDISC=0,120");
              Serial2.println("AT+CMQTTDISC=0,120");
              Wait_Response("OK",2000);
              delay(500);

              //AT+CMQTTREL=0
              Serial.println("AT+CMQTTREL=0");
              Serial2.println("AT+CMQTTREL=0");
              Wait_Response("OK",2000);
              delay(500);

              //Ket thuc Ket noi MQTT
              Serial.println("AT+CMQTTSTOP");
              Serial2.println("AT+CMQTTSTOP");
              Wait_Response();
              delay(800);
              break;
  }
}
/*---------------------------------CHUONG TRINH CON-----------------------------------*/
// RTOS
void TASK_ONE(void *param) {
  while (1) 
  {
      for(int i=0; i<2;i++){
      if(myPort.available())
      {
        String line = myPort.readStringUntil('\n') ;
        DeserializationError error = deserializeJson(doc,line);
        if(error){
          Serial.print("deserializeJson() failed: ");
          Serial.println(error.c_str());
        }
        else
        {
          co2 = doc["C"];
          tvoc= doc["T"];
          h2s = doc["H"];
          n   = doc["N"];
          e   = doc["E"];
        }
        doc.clear();
        delay(5);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2000));//Task duoc thu hien sau 2s.
  }
}
//Ham con ket noi lai MQTT.
void reconnect() {
  while (!client.connected()) 
  {
    Serial.print("Attempting MQTT connection...");
    String clientID =  "ESPClient-";
    clientID += String(random(0xffff),HEX);
    if (client.connect(clientID.c_str(), mqtt_username, mqtt_password)) 
    {
      Serial.println("connected");
      client.subscribe("iot/66");
    } 
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
//Ham con kiem tra ket noi SIM7028
void getDevice() {
  Serial2.println("AT");
  delay(100);
  rev = Serial2.readString();
  char *p_response = strstr(rev.c_str(), "\r\n");
  if (p_response != NULL) {
    p_response += 2;
    if (strstr(p_response, "OK") != NULL) {
      Serial.println("Got Sim7208!");
      isSerial2Available = false;
    }
  } else {
    Serial.println("Serial Device Not Found! Retrying...");
  }
}
//Ham con gui tin nhan den SIM
int SentMessage(const char *p_char) {
  char r_buf[100];
  Serial2.println(p_char);
  delay(1000);

  size_t i = Serial2.available();
  if(i>=90) i = 90;
  for (size_t j = 0; j < i; j++) {
    r_buf[j] = Serial2.read();
  }
  r_buf[i] = '\0';

  char *p_response = strstr(r_buf, "\r\n");
  //Serial.println(p_response);
  if (p_response != NULL) {
    p_response += 2;
    if (strstr(p_response, "+CREG: 1,") != NULL) {
      char *p_status = strstr(p_response, "+CREG: 1,") + 9;
      int status = atoi(p_status);
      if (status == 0) {
        isSerial2Available = false;
        Serial.println();
        Serial.println("NetWork Connected");
      }
    }
    if (strstr(p_response, "+CMQTTCONNECT: 0,") != NULL) {
      char *p_status = strstr(p_response, "+CMQTTCONNECT: 0,") + 17;
      int status = atoi(p_status);
      if (status == 0) {
        isSerial2Available = false;
        Serial.println();
        Serial.println("Mqtt Connected");
        return 1;
      } else if (status == 23) {
        isSerial2Available = false;
        Serial.println();
        Serial.println("Mqtt is already Connected");
        return 1;
      }
    }
  }
  return 0;
}
//Ham con gui du lieu den SIM
void input_message(const char *p_char, const char *p_data) {
  char r_buf[100];

  Serial2.println(p_char);
  delay(200);
  Serial2.print(p_data);
  delay(1000);
  size_t i = Serial2.available();
  if(i>=90) i = 90;
  for (size_t j = 0; j < i; j++) {
    r_buf[j] = Serial2.read();
  }
  r_buf[i] = '\0';

  char *p_response = strstr(r_buf, "\r\n");
  if (p_response != NULL) {
    p_response += 2;

    if (strstr(p_response, "OK") != NULL) {
      Serial.println("Write OK");
    } else if (strstr(p_response, "ERROR") != NULL) {
      Serial.println("Write ERROR");
    }
  }
}
//Ham con doc phan hoi mot so lenh tu SIM.
void getResponseData() {
  static char buffer[200];
  static int bufferIndex = 0;
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      buffer[bufferIndex] = '\0';
      if (strncmp(buffer, "+CMQTTPUB: 0,0", 14) != 0 && strcmp(buffer, "OK") != 0) {
        if (!(bufferIndex == 0 && c == '\n')) {
          if (strstr(buffer, "{") != NULL) {
            // Serial.print(buffer);
            StaticJsonDocument<200> jsonDocument;
            DeserializationError error = deserializeJson(jsonDocument, buffer);
            if (error == DeserializationError::Ok) {
              // Access the "data" object and get the "led" value
              int ledValue = jsonDocument["data"]["led"];
              if (ledValue == 1) {
                digitalWrite(5, HIGH);
              } else if (ledValue == 0) {
                digitalWrite(5, LOW);
              }
            } else {
              Serial.println("JSON parsing failed!");
            }
          }
        }
      }
      bufferIndex = 0;
    } else {
      if (bufferIndex < sizeof(buffer) - 1) {
        buffer[bufferIndex++] = c;
      }
    }
  }
}
//-----Call back Method for Receiving MQTT massage---------
void callback(char* topic, byte* payload, unsigned int length) {
  String incommingMessage = "";
  for(int i=0; i<length;i++) incommingMessage += (char)payload[i];
  Serial.println("Massage arived ["+String(topic)+"]"+incommingMessage);
}
//-----Method for Publishing MQTT Messages---------
void publishMessage(const char* topic, String payload, boolean retained){
  if(client.publish(topic,payload.c_str(),true))
    Serial.println("Message published ["+String(topic)+"]: "+payload);
}
//Ham con doi phan hoi tu SIM7028
bool Wait_Response(String expected_answer, unsigned int timeout){
  uint8_t x=0, answer=0;
  String response;
  unsigned long previous;
    
  //Clean the input buffer
  while( Serial2.available() > 0) Serial2.read();
  
  previous = millis();
  do{
    //if data in UART INPUT BUFFER, reads it
    if(Serial2.available() != 0){
        char c = Serial2.read();
        response.concat(c);
        x++;
        //Kiem tra neu (response == expected_answer)
        if(response.indexOf(expected_answer) > 0){
            answer = 1;
        }
    }
  }while((answer == 0) && ((millis() - previous) < timeout));

  Serial.println(response);
  return answer;

}

