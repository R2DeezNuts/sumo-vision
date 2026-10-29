#include <Arduino.h>   // <--- SIEMPRE el primero en PlatformIO
#include <WiFi.h>
#include <WiFiUdp.h>

#ifndef WIFI_SSID
#error "Falta WIFI_SSID. Crea un .env basado en .env.example."
#endif

#ifndef WIFI_PASSWORD
#error "Falta WIFI_PASSWORD. Crea un .env basado en .env.example."
#endif

#ifndef UDP_PORT
#define UDP_PORT 9999
#endif

#ifndef STATIC_IP
#define STATIC_IP "192.168.1.148"
#endif

#ifndef GATEWAY_IP
#define GATEWAY_IP "192.168.1.1"
#endif

#ifndef WIFI_SUBNET_MASK
#define WIFI_SUBNET_MASK "255.255.255.0"
#endif

#ifndef DNS_IP
#define DNS_IP "192.168.1.1"
#endif

#ifndef START_BOOST_MS
#define START_BOOST_MS 2000
#endif

#ifndef START_RIGHT_FWD
#define START_RIGHT_FWD 30
#endif

#ifndef START_LEFT_FWD
#define START_LEFT_FWD 23
#endif

#ifndef RUN_RIGHT_FWD
#define RUN_RIGHT_FWD 0
#endif

#ifndef RUN_LEFT_FWD
#define RUN_LEFT_FWD -7
#endif

#ifndef TRIM_RIGHT_FWD
#define TRIM_RIGHT_FWD -6
#endif

#ifndef TRIM_LEFT_FWD
#define TRIM_LEFT_FWD 0
#endif

#ifndef TRIM_RIGHT_BACK
#define TRIM_RIGHT_BACK 0
#endif

#ifndef TRIM_LEFT_BACK
#define TRIM_LEFT_BACK 0
#endif

// ================= WIFI =================
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* staticIpConfig = STATIC_IP;
const char* gatewayIpConfig = GATEWAY_IP;
const char* subnetMaskConfig = WIFI_SUBNET_MASK;
const char* dnsIpConfig = DNS_IP;

// ================= UDP =================
const uint16_t udpPort = UDP_PORT;
WiFiUDP udp;
char udpBuffer[64];

// ================= PINES MOTORES =================
// Motor A = DERECHA | Motor B = IZQUIERDA
const int A1_PIN = 33;
const int A2_PIN = 32;
const int PA_PIN = 25;
const int B1_PIN = 27;
const int B2_PIN = 14;
const int PB_PIN = 26;
const int PA_CHANNEL = 0;
const int PB_CHANNEL = 1;

// ================= PWM CONFIG =================
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;
const int PWM_MAX  = 80;
const int STOP_MS  = 150;
const int RAMP_TIME_MS = 250;
const int START_PWM_A = 20;
const int START_PWM_B = 20;
const int STOP_HOLD_PWM = 15;
const int STOP_HOLD_MS  = 40;
const unsigned long START_BOOST_DURATION_MS = START_BOOST_MS;

// ================= TRIMS =================
const int START_A_FWD_PCT = START_RIGHT_FWD;
const int START_B_FWD_PCT = START_LEFT_FWD;
const int RUN_A_FWD = RUN_RIGHT_FWD;
const int RUN_B_FWD = RUN_LEFT_FWD;
const int RUN_A_BACK = TRIM_RIGHT_BACK;
const int RUN_B_BACK = TRIM_LEFT_BACK;
const float TURN_INNER_FACTOR = 0.55f;

// ================= ESTADO MOTORES =================
int dirActualA = 0, dirActualB = 0;
int dirObjetivoA = 0, dirObjetivoB = 0;
int pwmActualA = 0, pwmActualB = 0;
int pwmObjetivoA = 0, pwmObjetivoB = 0;
unsigned long lastRampUpdate = 0;
unsigned long stopUntil = 0;
bool waitingStop = false;
unsigned long stopHoldUntil = 0;
bool stoppingHoldActive = false;
bool boostActive = false;
unsigned long boostUntil = 0;
int boostFinalDirA = 0, boostFinalDirB = 0;
int boostFinalPwmA = 0, boostFinalPwmB = 0;

bool parseIpConfig(const char* texto, IPAddress& ip) {
  return ip.fromString(texto);
}

// ================= FUNCIONES MOTORES (igual que antes) =================
void setupPWM() {
  // 1. Configurar las propiedades del canal PWM
  ledcSetup(PA_CHANNEL, PWM_FREQ, PWM_RES); // Canal 0
  ledcSetup(PB_CHANNEL, PWM_FREQ, PWM_RES); // Canal 1

  // 2. Adjuntar el canal al pin fisico
  ledcAttachPin(PA_PIN, PA_CHANNEL); // Pin del motor A al canal 0
  ledcAttachPin(PB_PIN, PB_CHANNEL); // Pin del motor B al canal 1
}

void setPWM() {
  ledcWrite(PA_CHANNEL, pwmActualA);
  ledcWrite(PB_CHANNEL, pwmActualB);
}
void setDirA(int d) {
  if (d > 0) { digitalWrite(A1_PIN, LOW); digitalWrite(A2_PIN, HIGH); }
  else if (d < 0) { digitalWrite(A1_PIN, HIGH); digitalWrite(A2_PIN, LOW); }
}
void setDirB(int d) {
  if (d > 0) { digitalWrite(B1_PIN, LOW); digitalWrite(B2_PIN, HIGH); }
  else if (d < 0) { digitalWrite(B1_PIN, HIGH); digitalWrite(B2_PIN, LOW); }
}
bool invertir(int a, int b) { return (a != 0 && b != 0 && a != b); }
int clamp(int v) { return constrain(v, 0, 255); }

int pwmDesdePorcentaje(int porcentaje) {
  int pct = constrain(porcentaje, 0, 100);
  return (PWM_MAX * pct + 50) / 100;
}

int aplicarTrim(int pwmBase, int trim) {
  if (pwmBase <= 0) return 0;
  return clamp(pwmBase + trim);
}

bool motoresParados() {
  return pwmActualA == 0 && pwmActualB == 0 && pwmObjetivoA == 0 && pwmObjetivoB == 0;
}

void setObjetivos(int dirA, int dirB, int pwmA, int pwmB) {
  dirObjetivoA = dirA; dirObjetivoB = dirB;
  pwmObjetivoA = pwmA; pwmObjetivoB = pwmB;
}

void cancelarBoost() {
  boostActive = false;
}

void iniciarBoostSiParado(int finalDirA, int finalDirB, int finalPwmA, int finalPwmB,
                          int startDirA, int startDirB, int startPwmA, int startPwmB) {
  if (START_BOOST_DURATION_MS > 0 && motoresParados()) {
    boostActive = true;
    boostUntil = millis() + START_BOOST_DURATION_MS;
    boostFinalDirA = finalDirA; boostFinalDirB = finalDirB;
    boostFinalPwmA = finalPwmA; boostFinalPwmB = finalPwmB;
    setObjetivos(startDirA, startDirB, startPwmA, startPwmB);
  } else {
    cancelarBoost();
    setObjetivos(finalDirA, finalDirB, finalPwmA, finalPwmB);
  }
}

void actualizarMotores() {
  unsigned long now = millis();
  if (boostActive && now >= boostUntil) {
    boostActive = false;
    setObjetivos(boostFinalDirA, boostFinalDirB, boostFinalPwmA, boostFinalPwmB);
  }
  if (stoppingHoldActive && now >= stopHoldUntil) {
    pwmObjetivoA = 0; pwmObjetivoB = 0;
    dirObjetivoA = 0; dirObjetivoB = 0;
    stoppingHoldActive = false;
  }
  if (waitingStop) {
    if (now >= stopUntil) {
      waitingStop = false;
      setDirA(dirObjetivoA);
      setDirB(dirObjetivoB);
      dirActualA = dirObjetivoA;
      dirActualB = dirObjetivoB;
    }
    return;
  }
  if (invertir(dirActualA, dirObjetivoA) || invertir(dirActualB, dirObjetivoB)) {
    pwmActualA = 0; pwmActualB = 0;
    setPWM();
    waitingStop = true;
    stopUntil = now + STOP_MS;
    return;
  }
  if (dirActualA != dirObjetivoA) { setDirA(dirObjetivoA); dirActualA = dirObjetivoA; }
  if (dirActualB != dirObjetivoB) { setDirB(dirObjetivoB); dirActualB = dirObjetivoB; }
  int stepDelay = max(1, RAMP_TIME_MS / max(1, PWM_MAX));
  if (now - lastRampUpdate >= (unsigned long)stepDelay) {
    lastRampUpdate = now;
    if (pwmObjetivoA > 0 && pwmActualA == 0) pwmActualA = START_PWM_A;
    else if (pwmActualA < pwmObjetivoA) pwmActualA++;
    else if (pwmActualA > pwmObjetivoA) pwmActualA--;
    if (pwmObjetivoB > 0 && pwmActualB == 0) pwmActualB = START_PWM_B;
    else if (pwmActualB < pwmObjetivoB) pwmActualB++;
    else if (pwmActualB > pwmObjetivoB) pwmActualB--;
    setPWM();
  }
}

// ================= COMANDOS =================
void cmdFwd(int porcentaje) {
  int runBase = pwmDesdePorcentaje(porcentaje);
  int startA = pwmDesdePorcentaje(START_A_FWD_PCT);
  int startB = pwmDesdePorcentaje(START_B_FWD_PCT);
  iniciarBoostSiParado(+1, +1, aplicarTrim(runBase, RUN_A_FWD), aplicarTrim(runBase, RUN_B_FWD),
                       +1, +1, startA, startB);
}
void cmdBack(int porcentaje) {
  int runBase = pwmDesdePorcentaje(porcentaje);
  cancelarBoost();
  setObjetivos(-1, -1, aplicarTrim(runBase, RUN_A_BACK), aplicarTrim(runBase, RUN_B_BACK));
}
void cmdLeft(int porcentaje) {
  int runBase = pwmDesdePorcentaje(porcentaje);
  int startA = pwmDesdePorcentaje(START_A_FWD_PCT);
  int startB = pwmDesdePorcentaje(START_B_FWD_PCT);
  iniciarBoostSiParado(+1, +1, aplicarTrim((int)(runBase * TURN_INNER_FACTOR), RUN_A_FWD), aplicarTrim(runBase, RUN_B_FWD),
                       +1, +1, (int)(startA * TURN_INNER_FACTOR), startB);
}
void cmdRight(int porcentaje) {
  int runBase = pwmDesdePorcentaje(porcentaje);
  int startA = pwmDesdePorcentaje(START_A_FWD_PCT);
  int startB = pwmDesdePorcentaje(START_B_FWD_PCT);
  iniciarBoostSiParado(+1, +1, aplicarTrim(runBase, RUN_A_FWD), aplicarTrim((int)(runBase * TURN_INNER_FACTOR), RUN_B_FWD),
                       +1, +1, startA, (int)(startB * TURN_INNER_FACTOR));
}
void cmdSoftRight(int basePorcentaje, int deltaPorcentaje) {
  int basePwm = pwmDesdePorcentaje(basePorcentaje);
  int outerPwm = pwmDesdePorcentaje(constrain(basePorcentaje + deltaPorcentaje, 0, 100));
  cancelarBoost();
  setObjetivos(+1, +1, aplicarTrim(outerPwm, RUN_A_FWD), aplicarTrim(basePwm, RUN_B_FWD));
}
void cmdSoftLeft(int basePorcentaje, int deltaPorcentaje) {
  int basePwm = pwmDesdePorcentaje(basePorcentaje);
  int outerPwm = pwmDesdePorcentaje(constrain(basePorcentaje + deltaPorcentaje, 0, 100));
  cancelarBoost();
  setObjetivos(+1, +1, aplicarTrim(basePwm, RUN_A_FWD), aplicarTrim(outerPwm, RUN_B_FWD));
}
void cmdSoftBackRight(int basePorcentaje, int deltaPorcentaje) {
  int basePwm = pwmDesdePorcentaje(basePorcentaje);
  int outerPwm = pwmDesdePorcentaje(constrain(basePorcentaje + deltaPorcentaje, 0, 100));
  cancelarBoost();
  setObjetivos(-1, -1, aplicarTrim(outerPwm, RUN_A_BACK), aplicarTrim(basePwm, RUN_B_BACK));
}
void cmdSoftBackLeft(int basePorcentaje, int deltaPorcentaje) {
  int basePwm = pwmDesdePorcentaje(basePorcentaje);
  int outerPwm = pwmDesdePorcentaje(constrain(basePorcentaje + deltaPorcentaje, 0, 100));
  cancelarBoost();
  setObjetivos(-1, -1, aplicarTrim(basePwm, RUN_A_BACK), aplicarTrim(outerPwm, RUN_B_BACK));
}
void cmdRightMotorOnly(int porcentaje) {
  int pwmBase = pwmDesdePorcentaje(porcentaje);
  cancelarBoost();
  dirObjetivoA = +1; dirObjetivoB = 0;
  pwmObjetivoA = pwmBase;
  pwmObjetivoB = 0;
}
void cmdLeftMotorOnly(int porcentaje) {
  int pwmBase = pwmDesdePorcentaje(porcentaje);
  cancelarBoost();
  dirObjetivoA = 0; dirObjetivoB = +1;
  pwmObjetivoA = 0;
  pwmObjetivoB = pwmBase;
}
void cmdStop() {
  cancelarBoost();
  pwmObjetivoA = STOP_HOLD_PWM; pwmObjetivoB = STOP_HOLD_PWM;
  dirObjetivoA = dirActualA; dirObjetivoB = dirActualB;
  stoppingHoldActive = true;
  stopHoldUntil = millis() + STOP_HOLD_MS;
}

String errorAndStop(const String& respuesta) {
  cmdStop();
  return respuesta;
}

bool parsePorcentaje(const String& texto, int startIndex, int& porcentaje) {
  String valor = texto.substring(startIndex);
  valor.trim();
  if (valor.length() == 0) return false;

  for (size_t i = 0; i < valor.length(); i++) {
    if (!isDigit(valor[i])) return false;
  }

  porcentaje = valor.toInt();
  return porcentaje >= 0 && porcentaje <= 100;
}

bool parseDosPorcentajes(const String& texto, int startIndex, int& base, int& delta) {
  String valores = texto.substring(startIndex);
  valores.trim();
  int separador = valores.indexOf(' ');
  if (separador <= 0) return false;

  String baseTexto = valores.substring(0, separador);
  String deltaTexto = valores.substring(separador + 1);
  baseTexto.trim();
  deltaTexto.trim();
  if (baseTexto.length() == 0 || deltaTexto.length() == 0) return false;

  for (size_t i = 0; i < baseTexto.length(); i++) {
    if (!isDigit(baseTexto[i])) return false;
  }
  for (size_t i = 0; i < deltaTexto.length(); i++) {
    if (!isDigit(deltaTexto[i])) return false;
  }

  base = baseTexto.toInt();
  delta = deltaTexto.toInt();
  return base >= 0 && base <= 100 && delta >= 0 && delta <= 100;
}

// ================= PROCESAR COMANDOS DESDE EL PORTATIL =================
String procesarComando(const char* comandoRaw) {
  String comando = String(comandoRaw);
  comando.trim();
  if (comando.length() == 0) return errorAndStop("ERROR comando vacio");

  String comandoUpper = comando;
  comandoUpper.toUpperCase();

  if (comandoUpper.startsWith("RR")) {
    int base = 0;
    int delta = 0;
    if (!parseDosPorcentajes(comando, 2, base, delta)) {
      Serial.print("Comando RR invalido: ");
      Serial.println(comando);
      return errorAndStop("ERROR RR invalido: " + comando);
    }
    cmdSoftRight(base, delta);
    Serial.print("Comando: GIRO SUAVE DER base=");
    Serial.print(base);
    Serial.print("% delta=");
    Serial.print(delta);
    Serial.println("%");
    return "OK GIRO SUAVE DER " + String(base) + "% +" + String(delta) + "%";
  }

  if (comandoUpper.startsWith("LL")) {
    int base = 0;
    int delta = 0;
    if (!parseDosPorcentajes(comando, 2, base, delta)) {
      Serial.print("Comando LL invalido: ");
      Serial.println(comando);
      return errorAndStop("ERROR LL invalido: " + comando);
    }
    cmdSoftLeft(base, delta);
    Serial.print("Comando: GIRO SUAVE IZQ base=");
    Serial.print(base);
    Serial.print("% delta=");
    Serial.print(delta);
    Serial.println("%");
    return "OK GIRO SUAVE IZQ " + String(base) + "% +" + String(delta) + "%";
  }

  if (comandoUpper.startsWith("BR")) {
    int base = 0;
    int delta = 0;
    if (!parseDosPorcentajes(comando, 2, base, delta)) {
      Serial.print("Comando BR invalido: ");
      Serial.println(comando);
      return errorAndStop("ERROR BR invalido: " + comando);
    }
    cmdSoftBackRight(base, delta);
    Serial.print("Comando: ATRAS SUAVE DER base=");
    Serial.print(base);
    Serial.print("% delta=");
    Serial.print(delta);
    Serial.println("%");
    return "OK ATRAS SUAVE DER " + String(base) + "% +" + String(delta) + "%";
  }

  if (comandoUpper.startsWith("BL")) {
    int base = 0;
    int delta = 0;
    if (!parseDosPorcentajes(comando, 2, base, delta)) {
      Serial.print("Comando BL invalido: ");
      Serial.println(comando);
      return errorAndStop("ERROR BL invalido: " + comando);
    }
    cmdSoftBackLeft(base, delta);
    Serial.print("Comando: ATRAS SUAVE IZQ base=");
    Serial.print(base);
    Serial.print("% delta=");
    Serial.print(delta);
    Serial.println("%");
    return "OK ATRAS SUAVE IZQ " + String(base) + "% +" + String(delta) + "%";
  }

  if (comandoUpper.startsWith("BK")) {
    int porcentaje = 0;
    if (!parsePorcentaje(comando, 2, porcentaje)) {
      Serial.print("Comando BK invalido: ");
      Serial.println(comando);
      return errorAndStop("ERROR BK invalido: " + comando);
    }
    cmdBack(porcentaje);
    Serial.print("Comando: ATRAS ");
    Serial.print(porcentaje);
    Serial.println("%");
    return "OK ATRAS " + String(porcentaje) + "%";
  }

  char accion = toupper(comando[0]);
  if (accion == 'S') {
    cmdStop();
    Serial.println("Comando: STOP");
    return "OK STOP";
  }

  if (accion != 'F' && accion != 'L' && accion != 'R' && accion != 'A' && accion != 'B') {
    Serial.print("Comando invalido: ");
    Serial.println(comando);
    return errorAndStop("ERROR comando invalido: " + comando);
  }

  int porcentaje = 0;
  if (!parsePorcentaje(comando, 1, porcentaje)) {
    Serial.print("Porcentaje invalido: ");
    Serial.println(comando);
    return errorAndStop("ERROR porcentaje invalido: " + comando);
  }

  String respuesta;
  switch(accion) {
    case 'F':
      cmdFwd(porcentaje);
      Serial.print("Comando: AVANZAR ");
      respuesta = "OK AVANZAR ";
      break;
    case 'L':
      cmdLeft(porcentaje);
      Serial.print("Comando: GIRAR IZQ ");
      respuesta = "OK GIRAR IZQ ";
      break;
    case 'R':
      cmdRight(porcentaje);
      Serial.print("Comando: GIRAR DER ");
      respuesta = "OK GIRAR DER ";
      break;
    case 'A':
      cmdRightMotorOnly(porcentaje);
      Serial.print("Comando: MOTOR DER ");
      respuesta = "OK MOTOR DER ";
      break;
    case 'B':
      cmdLeftMotorOnly(porcentaje);
      Serial.print("Comando: MOTOR IZQ ");
      respuesta = "OK MOTOR IZQ ";
      break;
  }
  Serial.print(porcentaje);
  Serial.println("%");
  respuesta += String(porcentaje);
  respuesta += "%";
  return respuesta;
}

void responderUdp(IPAddress ip, uint16_t port, const String& respuesta) {
  udp.beginPacket(ip, port);
  udp.print(respuesta);
  udp.endPacket();
}

void leerUdp() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  IPAddress remoteIp = udp.remoteIP();
  uint16_t remotePort = udp.remotePort();

  int len = udp.read(udpBuffer, sizeof(udpBuffer) - 1);
  if (len <= 0) return;

  udpBuffer[len] = '\0';
  Serial.print("UDP de ");
  Serial.print(remoteIp);
  Serial.print(":");
  Serial.print(remotePort);
  Serial.print(" -> ");
  Serial.println(udpBuffer);
  String respuesta = procesarComando(udpBuffer);
  responderUdp(remoteIp, remotePort, respuesta);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(A1_PIN, OUTPUT); pinMode(A2_PIN, OUTPUT);
  pinMode(B1_PIN, OUTPUT); pinMode(B2_PIN, OUTPUT);
  setupPWM(); setPWM();

  // Conectar a WiFi
  Serial.print("Conectando a WiFi ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);

  IPAddress localIp;
  IPAddress gatewayIp;
  IPAddress subnetMask;
  IPAddress dnsIp;
  if (!parseIpConfig(staticIpConfig, localIp) ||
      !parseIpConfig(gatewayIpConfig, gatewayIp) ||
      !parseIpConfig(subnetMaskConfig, subnetMask) ||
      !parseIpConfig(dnsIpConfig, dnsIp)) {
    Serial.println("Error configurando IP fija");
    while(1) delay(1000);
  }

  Serial.print("IP fija: ");
  Serial.println(localIp);
  if (!WiFi.config(localIp, gatewayIp, subnetMask, dnsIp)) {
    Serial.println("Error aplicando IP fija");
    while(1) delay(1000);
  }

  WiFi.begin(ssid, password);
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 30) {
    delay(500); Serial.print("."); intentos++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Error WiFi");
    while(1) delay(1000);
  }

  if (udp.begin(udpPort)) {
    Serial.print("Escuchando UDP en puerto ");
    Serial.println(udpPort);
  } else {
    Serial.println("Error iniciando UDP");
    while(1) delay(1000);
  }
}

// ================= LOOP =================
void loop() {
  actualizarMotores();
  leerUdp();
}
