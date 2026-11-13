/*
  ============================================================================
  chronodh.ino - Chronometre VTT DH maison (depart / arrivee / emetteur IR)
  ============================================================================
  Carte  : Seeed XIAO ESP32-C6 sur la PCB chronodh
  Core   : Arduino-ESP32 3.x, carte "XIAO_ESP32C6"
  Outils (Arduino IDE) :
    - USB CDC On Boot  : Enabled  (la console passe par l'USB, pas par D6/D7)
    - Partition Scheme : Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)
  Role   : CHRONO_ROLE dans config.h

  Principe : le depart horodate la poussee de la baguette avec SON horloge
  et transmet l'horodatage (pas un declenchement). L'arrivee synchronise
  en continu son horloge sur celle du depart (echange NTP a 4 horodatages,
  filtre RTT minimal, regression lineaire offset + derive) et convertit.

  Ordre impose a chaque evenement : (1) horodatage dans l'interruption,
  (2) envoi radio / journal, (3) bip depuis la tache buzzer, jamais dans
  l'interruption (radio et buzzer sur le meme 3V3).
  ============================================================================
*/

#include "config.h"

// Version du firmware (ici et non dans config.h : elle suit le code)
#undef FW_VERSION
#define FW_VERSION "1.4.0"

// Valeurs par defaut si config.h est une ancienne version
#ifndef IR_POWER_DEFAULT
#define IR_POWER_DEFAULT 10
#endif
#ifndef IR_CAL_MARGIN
#define IR_CAL_MARGIN 2
#endif
#ifndef IR_CAL_TARGET
#define IR_CAL_TARGET 95
#endif
#ifndef ROLE_RELAY
#define ROLE_RELAY 4
#endif
#ifndef HOSTNAME_RELAY
#define HOSTNAME_RELAY "chrono-relais"
#endif
#ifndef RADIO_LONG_RANGE
#define RADIO_LONG_RANGE 1
#endif
#ifndef RADIO_LR_RATE
#define RADIO_LR_RATE 250
#endif
#ifndef ANTENNA_EXTERNAL
#define ANTENNA_EXTERNAL 0
#endif
#ifndef ANT_EXT_START
#define ANT_EXT_START ANTENNA_EXTERNAL
#endif
#ifndef ANT_EXT_FINISH
#define ANT_EXT_FINISH ANTENNA_EXTERNAL
#endif
#ifndef ANT_EXT_EMITTER
#define ANT_EXT_EMITTER ANTENNA_EXTERNAL
#endif
#ifndef ANT_EXT_RELAY
#define ANT_EXT_RELAY ANTENNA_EXTERNAL
#endif
// LoRa (RFM95W) : desactive si config.h est une ancienne version
#ifndef LORA_START
#define LORA_START 0
#endif
#ifndef LORA_FINISH
#define LORA_FINISH 0
#endif
#ifndef LORA_FREQ_HZ
#define LORA_FREQ_HZ 869525000UL
#endif
#ifndef LORA_SF
#define LORA_SF 7
#endif
#ifndef LORA_TX_DBM
#define LORA_TX_DBM 14
#endif
#ifndef LORA_DUTY_PCT
#define LORA_DUTY_PCT 10
#endif
#ifndef LORA_SYNC_WORD
#define LORA_SYNC_WORD 0x12
#endif
#ifndef PIN_LORA_NSS
#define PIN_LORA_NSS D5
#endif
#ifndef PIN_LORA_RST
#define PIN_LORA_RST D6
#endif
#ifndef PIN_LORA_DIO0
#define PIN_LORA_DIO0 D7
#endif
#ifndef PIN_SPI_SCK
#define PIN_SPI_SCK D8
#endif
#ifndef PIN_SPI_MISO
#define PIN_SPI_MISO D9
#endif
#ifndef PIN_SPI_MOSI
#define PIN_SPI_MOSI D10
#endif

#if !defined(CHRONO_ROLE) || (CHRONO_ROLE != ROLE_START && CHRONO_ROLE != ROLE_FINISH && CHRONO_ROLE != ROLE_EMITTER && CHRONO_ROLE != ROLE_RELAY)
#error "CHRONO_ROLE doit valoir ROLE_START, ROLE_FINISH, ROLE_EMITTER ou ROLE_RELAY dans config.h"
#endif
#if RADIO_LR_RATE != 250 && RADIO_LR_RATE != 500
#error "RADIO_LR_RATE doit valoir 250 ou 500"
#endif
#if LORA_SF < 7 || LORA_SF > 12
#error "LORA_SF doit etre entre 7 et 12"
#endif
#if !((LORA_TX_DBM >= 2 && LORA_TX_DBM <= 17) || LORA_TX_DBM == 20)
#error "LORA_TX_DBM : de 2 a 17, ou 20 (limite du RFM95W sur PA_BOOST)"
#endif
#if !ARDUINO_USB_CDC_ON_BOOT
#error "Outils > USB CDC On Boot doit etre sur Enabled (sinon la console occupe D6/D7)"
#endif

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <esp_idf_version.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include <SPI.h>
#include <WebServer.h>
#include <Update.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <math.h>

#define LOGF(...) do { if (DEBUG_SERIAL) Serial.printf(__VA_ARGS__); } while (0)

// ============================================================================
// 1. CONSTANTES
// ============================================================================
static constexpr bool IS_START   = (CHRONO_ROLE == ROLE_START);
static constexpr bool IS_FINISH  = (CHRONO_ROLE == ROLE_FINISH);
static constexpr bool IS_EMITTER = (CHRONO_ROLE == ROLE_EMITTER);
static constexpr bool IS_RELAY   = (CHRONO_ROLE == ROLE_RELAY);
static constexpr bool HAS_WEB    = IS_START || IS_FINISH;   // point d'acces + pages web + OTA permanent
static constexpr bool LORA_ME    = IS_START ? (LORA_START != 0) : IS_FINISH ? (LORA_FINISH != 0) : false;
static constexpr bool ANT_EXT_ME = IS_START ? ANT_EXT_START : IS_FINISH ? ANT_EXT_FINISH : IS_EMITTER ? ANT_EXT_EMITTER : ANT_EXT_RELAY;

static constexpr uint16_t PKT_MAGIC      = 0xC4D5;
static constexpr uint8_t  PKT_VERSION    = 4;   // 4 : LoRa
static constexpr size_t   MAX_PKT        = 128;
static constexpr size_t   NAME_LEN       = 28;
static constexpr size_t   CAT_LEN        = 16;
static constexpr uint16_t MAX_RIDERS     = 120;
static constexpr uint16_t START_HIST     = 40;
static constexpr int      OUTBOX_SIZE    = 12;
static constexpr uint16_t MAX_RUNS       = 300;
static constexpr int      MAX_ORPHANS    = 16;
static constexpr uint8_t  SYNC_POINTS    = 30;
// En Long Range un paquet occupe le canal environ 5 ms : 4 echanges par seconde suffisent
static constexpr uint32_t SYNC_PERIOD_MS = RADIO_LONG_RANGE ? 250 : 100;
static constexpr uint32_t SYNC_WINDOW_MS = 2000;
static constexpr uint32_t SYNC_MAX_RTT_US = 80000;
static constexpr int      SEEN_SIZE      = 48;
static constexpr int      HB_HIST        = 12;
// LoRa : cadences proportionnelles a la duree d'un symbole (x2 par SF), pour rester sous LORA_DUTY_PCT
static constexpr uint32_t LORA_SYNC_MS   = 3000UL << (LORA_SF - 7);
static constexpr uint32_t LORA_HB_MS     = 5000UL << (LORA_SF - 7);
static constexpr uint32_t LORA_RETRY_MS  = 2000UL << (LORA_SF - 7);
static constexpr int      LORA_QLEN      = 6;
// Un boitier en LoRa n'entend parfois son correspondant que toutes les LORA_SYNC_MS
static constexpr int64_t  PEER_TIMEOUT_US = LORA_ME && 2.5 * LORA_SYNC_MS > LINK_TIMEOUT_MS ? (int64_t)(2.5 * LORA_SYNC_MS * 1000) : (int64_t)LINK_TIMEOUT_MS * 1000;
static constexpr uint32_t HEARTBEAT_MS   = 1000;
static constexpr int      BUZ_MAX_NOTES  = 16;
static constexpr size_t   STORE_LINE     = 224;

// Salve IR generee par le RMT a 10 MHz : une periode de porteuse = 263 ticks (38,02 kHz)
static constexpr uint32_t IR_PERIOD_US = (IR_BURST_CYCLES * 263UL) / 10UL + IR_GAP_US;
static_assert(IR_BURST_CYCLES >= 6 && IR_BURST_CYCLES <= 40, "IR_BURST_CYCLES doit rester entre 6 et 40");
static_assert(IR_POWER_DEFAULT >= 1 && IR_POWER_DEFAULT <= 10, "IR_POWER_DEFAULT doit etre entre 1 et 10");

// Puissance IR : duree "allumee" (ticks de 0,1 us) sur une periode de porteuse de 263 ticks.
// L'amplitude a 38 kHz vue par le TSSP vaut sin(pi * rapport cyclique) : environ x1,4 par niveau,
// du niveau 1 (4 %, environ -26 dB) au niveau 10 (50 %, pleine puissance).
static const uint16_t IR_LEVEL_TICKS[10] = {4, 6, 8, 11, 16, 22, 31, 45, 66, 132};
static_assert(sizeof(AP_PASSWORD) >= 9, "AP_PASSWORD doit faire au moins 8 caracteres");

static const char* FINISH_LOG  = "/cur.log";
static const char* STARTS_LOG  = "/starts.log";
static const char* RIDERS_FILE = "/riders.txt";
static const uint8_t BCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ============================================================================
// 2. TYPES (tous declares avant la premiere fonction, a cause des prototypes
//    generes automatiquement par l'IDE Arduino)
// ============================================================================
enum PktType : uint8_t {
  PT_HEARTBEAT = 1, PT_SYNC_REQ = 2, PT_SYNC_RESP = 3, PT_START_EVT = 4,
  PT_CANCEL = 5, PT_ACK = 6, PT_FINISH_NOTE = 7, PT_BEAM_STATE = 8, PT_CMD = 9
};
enum CmdCode : uint8_t {
  CMD_MAINTENANCE = 1, CMD_ALIGN_ON = 2, CMD_ALIGN_OFF = 3, CMD_LOCATE = 4,
  CMD_SET_IR_POWER = 5, CMD_WIPE_SESSION = 6, CMD_WIPE_ALL = 7
};
enum HbFlags : uint8_t { HBF_MAINT = 0x01, HBF_ALIGN = 0x02, HBF_BENCH = 0x04, HBF_SYNC_OK = 0x08, HBF_BEAM_OK = 0x10 };
enum EvtFlags : uint8_t { EVF_BENCH = 0x01, EVF_FLASH = 0x02 };
enum HwKind : uint8_t { HW_HALL = 1, HW_BENCH = 2, HW_LORA = 3 };
enum RunStatus : uint8_t { RUN_ON_COURSE = 0, RUN_FINISHED = 1, RUN_DNF = 2, RUN_CANCELLED = 3 };
enum RunFlag : uint8_t {
  RUNF_FLASH = 0x01, RUNF_SYNC_DEGRADED = 0x02, RUNF_LATE_START = 0x04, RUNF_NO_SYNC = 0x08,
  RUNF_MANUAL = 0x10, RUNF_AUTO_DNF = 0x20, RUNF_START_FLASH = 0x40
};
enum BreakKind : uint8_t { BK_NONE = 0, BK_RIDER = 1, BK_ORPHAN = 2, BK_LOCKOUT = 3, BK_PAUSED = 4 };
enum StartRecState : uint8_t { SR_PENDING = 0, SR_ACKED = 1, SR_CANCEL_PENDING = 2, SR_CANCELLED = 3 };
enum StoreOp : uint8_t { SO_APPEND = 1, SO_SAVE_RIDERS = 2, SO_RENAME = 3, SO_ARCHIVE = 4, SO_WIPE = 5 };

// ---- Paquets radio (packes, < 250 octets) ----
struct __attribute__((packed)) PktHeader {
  uint16_t magic;
  uint8_t  version;
  uint8_t  type;
  uint16_t netId;
  uint8_t  role;
  uint8_t  flags;
  uint32_t bootId;
  uint32_t seq;
  uint8_t  dst;       // role destinataire final, 0 = tous
  uint8_t  hop;       // 0 = direct, 1 = repete par le relais
};
struct __attribute__((packed)) PktHeartbeat {
  PktHeader h;
  uint16_t battMv;
  uint16_t relayBattStart;
  uint16_t relayBattEmitter;
  uint32_t uptimeS;
  uint32_t sessionId;
  uint32_t syncUncertUs;
  uint8_t  beamQuality;
  uint8_t  pending;
  uint32_t ip;
  char     fw[12];
  uint8_t  irPower;   // emetteur : niveau 1 a 10
  uint8_t  rxDir[5];  // % des messages de vie recus en direct, par role (index 1 a 4)
  uint8_t  rxRel[5];  // % recus via le relais
  uint8_t  rxAny[5];  // % recus par au moins un chemin
  int8_t   rssi[5];   // dernier RSSI direct (dBm), 0 = aucun
  uint8_t  rxLora[5]; // % des messages de vie LoRa recus
  int8_t   lrssi[5];  // dernier RSSI LoRa (dBm), 0 = aucun
};
// Requete et reponse de synchro : meme structure, donc meme taille et meme duree d'emission
struct __attribute__((packed)) PktSync {
  PktHeader h;
  uint32_t reqId;
  int64_t  t1;        // emission de la requete (horloge arrivee)
  int64_t  t2;        // reception de la requete (horloge depart)
  int64_t  t3;        // emission de la reponse (horloge depart)
  uint32_t resReq;    // temps passe dans le relais a l'aller (us)
  uint32_t resResp;   // temps passe dans le relais au retour (us)
};
struct __attribute__((packed)) PktStartEvt {
  PktHeader h;
  uint32_t startBoot;
  uint32_t runSeq;
  int64_t  tStart;
  int64_t  wallMs;
  uint16_t riderId;
  uint16_t bib;
  uint8_t  evtFlags;
  char     name[NAME_LEN];
  char     cat[CAT_LEN];
};
struct __attribute__((packed)) PktCancel     { PktHeader h; uint32_t startBoot; uint32_t runSeq; };
struct __attribute__((packed)) PktAck        { PktHeader h; uint8_t ackType; uint32_t startBoot; uint32_t runSeq; };
struct __attribute__((packed)) PktFinishNote { PktHeader h; uint32_t startBoot; uint32_t runSeq; int64_t durationUs; uint8_t status; uint8_t flags; };
struct __attribute__((packed)) PktBeamState  { PktHeader h; uint8_t quality; uint8_t beamOk; uint8_t align; };
struct __attribute__((packed)) PktCmd        { PktHeader h; uint8_t targetRole; uint8_t cmd; uint32_t arg; };
static_assert(sizeof(PktStartEvt) <= MAX_PKT, "paquet trop grand");
static_assert(sizeof(PktHeartbeat) <= MAX_PKT, "paquet trop grand");
static_assert(sizeof(PktSync) <= MAX_PKT, "paquet trop grand");

// ---- Types internes ----
struct Note     { uint16_t freq; uint16_t ms; };
struct BuzItem  { Note n[BUZ_MAX_NOTES]; };
struct InMsg    { int64_t tRx; uint8_t mac[6]; int8_t rssi; int8_t snr; uint8_t lora; uint8_t len; uint8_t data[MAX_PKT]; };
struct LoraFrame { uint8_t len; uint8_t prio; uint8_t kind; int64_t queued; uint8_t data[MAX_PKT]; };
struct HwEvt    { uint8_t kind; int64_t t; };
struct StoreMsg { uint8_t op; char path[40]; char line[STORE_LINE]; void* ptr; };

struct HbSeen  { uint32_t seq; uint32_t boot; int64_t t; uint8_t paths; };
struct SeenKey { uint8_t role; uint32_t boot; uint32_t seq; };

struct PeerInfo {
  bool     known;         // entendu, par n'importe quel chemin
  bool     macKnown;      // adresse apprise en direct
  int64_t  lastDirectUs;
  int64_t  lastRelayUs;
  uint8_t  rxDir[5];      // ce que CE boitier rapporte entendre (dans son message de vie)
  uint8_t  rxRel[5];
  uint8_t  rxAny[5];
  int8_t   rssiRep[5];
  HbSeen   hb[HB_HIST];   // messages de vie recus de ce boitier (statistiques locales)
  int64_t  lastLoraUs;
  int16_t  loraRssi;
  int8_t   loraSnr;
  int64_t  loraHbT[6];    // reception des derniers messages de vie LoRa
  uint8_t  loraHbHead;
  uint8_t  rxLoraRep[5];  // ce que CE boitier rapporte recevoir en LoRa
  int8_t   lrssiRep[5];
  uint8_t  mac[6];
  uint32_t bootId;
  int64_t  lastSeenUs;
  int8_t   rssi;
  uint16_t battMv;
  uint8_t  hbFlags;
  uint32_t uptimeS;
  uint32_t sessionId;
  uint32_t syncUncertUs;
  uint8_t  beamQuality;
  uint8_t  pending;
  uint16_t relayBattStart;
  uint16_t relayBattEmitter;
  uint32_t ip;
  char     fw[12];
  uint8_t  irPower;
};

struct Rider { uint16_t id; uint16_t bib; uint16_t runs; char name[NAME_LEN]; char cat[CAT_LEN]; };

struct StartRec {
  uint32_t startBoot;
  uint32_t runSeq;
  int64_t  tStart;
  int64_t  wallMs;
  int64_t  durationUs;
  uint16_t riderId;
  uint16_t bib;
  uint8_t  state;
  uint8_t  evtFlags;
  uint8_t  finStatus;
  bool     counted;
  char     name[NAME_LEN];
  char     cat[CAT_LEN];
};

struct OutItem {
  bool     used;
  bool     warned;
  uint8_t  type;
  uint8_t  len;
  uint16_t tries;
  uint32_t startBoot;
  uint32_t runSeq;
  int64_t  created;
  int64_t  nextTx;
  int64_t  loraNext;
  uint8_t  data[MAX_PKT];
};

struct SyncPoint { int64_t tf; int64_t off; uint32_t rtt; uint8_t path; };
struct SyncModel {
  bool      used;
  bool      valid;
  bool      winHas;
  uint8_t   n;
  uint8_t   head;
  uint32_t  startBoot;
  uint32_t  minRtt;
  uint32_t  samples;
  double    a;          // offset (us) a tRef : horloge depart - horloge arrivee
  double    b;          // derive (us/s = ppm)
  double    residUs;    // ecart type des residus
  double    excessUs;   // ecart moyen des points retenus au meilleur aller-retour du chemin
  uint32_t  floor[3];   // meilleur aller-retour vu : [0] direct, [1] via relais, [2] LoRa
  int64_t   tRef;
  int64_t   lastPointUs;
  int64_t   winStartUs;
  SyncPoint winBest;
  SyncPoint pts[SYNC_POINTS];
};

struct Run {
  uint16_t idx;
  uint8_t  status;
  uint8_t  flags;
  uint16_t riderId;
  uint16_t bib;
  uint32_t startBoot;
  uint32_t runSeq;
  uint32_t finishBoot;    // boot de l'arrivee pendant lequel tFinishF a ete pris (0 = pas d'arrivee)
  uint32_t tStartFBoot;   // boot de l'arrivee pour lequel tStartF est valide
  uint32_t uncertUs;
  int64_t  tStartRaw;     // horloge du depart
  int64_t  wallStartMs;
  int64_t  tStartF;       // depart converti dans l'horloge de l'arrivee
  int64_t  tFinishF;
  int64_t  durationUs;
  char     name[NAME_LEN];
  char     cat[CAT_LEN];
};

struct Orphan { bool used; uint8_t flags; uint16_t id; uint32_t finishBoot; int64_t tF; int64_t wallMs; };
struct BenchStats { uint32_t n; double mean; double m2; int64_t minUs; int64_t maxUs; int64_t lastUs; };

// ============================================================================
// 3. VARIABLES GLOBALES
// ============================================================================
static SemaphoreHandle_t g_mtx    = nullptr;   // protege tout l'etat de course
static QueueHandle_t     g_inQ    = nullptr;   // paquets ESP-NOW recus
static QueueHandle_t     g_hwQ    = nullptr;   // evenements des interruptions
static QueueHandle_t     g_buzQ   = nullptr;   // motifs sonores
static QueueHandle_t     g_storeQ = nullptr;   // ecritures flash

static WebServer   g_web(80);
static Preferences g_prefs;

static uint32_t g_bootId = 0;
static uint32_t g_txSeq = 0;
static uint32_t g_sessionId = 1;
static wifi_interface_t g_espIf = WIFI_IF_STA;
static bool     g_espNowOk = false;
static bool     g_maintenance = false;
static int64_t  g_maintSinceUs = 0;
static volatile bool g_otaRunning = false;
static bool     g_upOk = false;
static bool     g_upAuthFail = false;
static bool     g_fsOk = false;
static uint16_t g_battMv = 0;
static int64_t  g_lowBattBeepUs = 0;
static int64_t  g_cutoffSinceUs = 0;
static int64_t  g_wallOffsetMs = 0;
static bool     g_wallValid = false;
static int      g_tzOffsetMin = 0;
static PeerInfo g_peers[5];   // index = role (1 a 4)
static bool     g_lr = false;  // Long Range actif
static SeenKey  g_seen[SEEN_SIZE];
static uint8_t  g_seenHead = 0;
static int64_t  g_syncRelayDueUs = 0;
static uint32_t g_relayFwd = 0;
static bool     g_rxViaLora = false;   // le paquet en cours de traitement est arrive en LoRa
// LoRa
static bool      g_loraOk = false;
static bool      g_loraTx = false;
static int64_t   g_loraTxDeadlineUs = 0;
static int64_t   g_loraHoldUntilUs = 0;
static int64_t   g_loraLastDio0Us = 0;
static int64_t   g_loraTokens = 0;
static int64_t   g_loraTokT = 0;
static LoraFrame g_loraQ[LORA_QLEN];
static int       g_loraQn = 0;
static uint32_t  g_loraTxCount = 0, g_loraRxCount = 0, g_loraCrcErr = 0, g_loraDrop = 0, g_loraTimeouts = 0;
static int       g_loraLastRssi = 0;
static int8_t    g_loraLastSnr = 0;
static int64_t   g_lastLoraHbUs = 0;
static int64_t   g_lastLoraSyncUs = 0;
static bool     g_linkWasOk = false;
static bool     g_linkEverOk = false;
static int64_t  g_linkLostBeepUs = 0;
static uint32_t g_storeDrops = 0;
static int64_t  g_lastHbUs = 0;
static int64_t  g_lastSlowUs = 0;

// Fenetre de la derniere ecriture flash (pour marquer les temps douteux)
static portMUX_TYPE g_flashMux = portMUX_INITIALIZER_UNLOCKED;
static int64_t g_flashFrom = 0;
static int64_t g_flashTo = 0;
static bool    g_flashActive = false;

// Bouton
static bool     g_btnRaw = false;
static bool     g_btnStable = false;
static bool     g_btnLongDone = false;
static bool     g_btnIgnore = false;
static uint32_t g_btnChangeMs = 0;
static uint32_t g_btnDownMs = 0;

// Alignement de la barriere (arrivee et emetteur)
static bool    g_align = false;
static int64_t g_alignUntilUs = 0;
static int64_t g_alignNextBeepUs = 0;
static int64_t g_alignLocalChangeUs = 0;

// ---- Depart ----
static Rider    g_riders[MAX_RIDERS];
static uint16_t g_riderCount = 0;
static int16_t  g_nextIdx = -1;
static uint16_t g_riderNextId = 1;
static uint32_t g_ridersRev = 1;
static bool     g_ridersSavePending = false;
static StartRec g_starts[START_HIST];
static uint16_t g_startHead = 0;
static uint16_t g_startCount = 0;
static uint32_t g_startsRev = 1;
static OutItem  g_out[OUTBOX_SIZE];
static volatile bool g_hallArmed = false;
static int64_t  g_lastTrigUs = 0;
static int64_t  g_hallHighSinceUs = 0;
static uint32_t g_runSeq = 0;
static bool     g_benchRemote = false;
static bool     g_syncOkRemote = false;

// ---- Arrivee ----
static SyncModel g_models[2];
static int64_t  g_lastSyncReqUs = 0;
static uint32_t g_syncReqId = 0;
static uint32_t g_syncBeepBoot = 0;
static Run*     g_runs = nullptr;
static uint16_t g_runCount = 0;
static uint32_t g_runsRev = 1;
static Orphan   g_orphans[MAX_ORPHANS];
static uint16_t g_orphanNextId = 1;
static portMUX_TYPE g_irMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t  g_irLastUs = 0;
static volatile uint32_t g_irCount = 0;
static bool     g_beamEstablished = false;
static int64_t  g_beamEstSinceUs = 0;
static bool     g_beamBroken = false;
static int64_t  g_beamBrokenAtUs = 0;
static int64_t  g_beamRestoredUs = 0;
static bool     g_beamRearmed = true;
static int64_t  g_lockoutUntilUs = 0;
static bool     g_beamFault = false;
static int64_t  g_beamFaultBeepUs = 0;
static uint8_t  g_beamQuality = 0;
static int64_t  g_qWinStartUs = 0;
static uint32_t g_qWinCount = 0;
static uint8_t  g_setRuns = DEFAULT_RUNS;
static uint16_t g_setMinS = DEFAULT_MIN_RUN_S;
static uint16_t g_setMaxS = DEFAULT_MAX_RUN_S;
static bool     g_setBeep = true;
static bool     g_bench = false;
static BenchStats g_benchSt;
static bool     g_benchCrossValid = false;
static int64_t  g_benchCrossUs = 0;
static bool     g_benchStartValid = false;
static uint32_t g_benchStartBoot = 0;
static int64_t  g_benchStartRaw = 0;
static int64_t  g_benchStartRxUs = 0;
static volatile int64_t g_benchIsrLastUs = 0;
static int64_t  g_bestRunUs = -1;
static int32_t  g_lastFinishedIdx = -1;
static int64_t  g_lastBeamStateTxUs = 0;
// Outil de test de la barriere
static uint32_t g_breakCount = 0;
static int64_t  g_breakStartUs = 0;
static int64_t  g_lastBreakDurUs = -1;   // -1 : coupure en cours ou aucune
static uint8_t  g_lastBreakKind = BK_NONE;
// Reglage automatique de la puissance de l'emetteur
static bool     g_calRunning = false;
static bool     g_calMeasuring = false;
static uint8_t  g_calLevel = 0;
static int64_t  g_calPhaseUs = 0;
static int64_t  g_calT0 = 0;
static uint32_t g_calCnt0 = 0;
static uint8_t  g_calResult = 0;          // 0 aucun, 1 reussi, 2 echec, 3 emetteur absent
static uint8_t  g_calFound = 0;
static uint8_t  g_calFinal = 0;

// ---- Emetteur ----
static uint8_t  g_remoteBeamQ = 0;
static int64_t  g_remoteBeamQUs = 0;
static bool     g_irTxOk = false;
static uint8_t  g_irPower = IR_POWER_DEFAULT;

// ---- Depart : dernier effacement recu (les commandes arrivent en triple)
static uint32_t g_lastWipeSid = 0;
static uint32_t g_autoArchivedSid = 0;   // session archivee automatiquement au dernier changement

// ---- Depart : mode reglage du capteur (le buzzer sonne tant que l'aimant est vu, aucun depart cree)
static bool     g_hallTune = false;
static int64_t  g_hallTuneUntilUs = 0;
static uint32_t g_tuneCount = 0;
static bool     g_tuneWasLow = false;
static int64_t  g_tuneBeepUs = 0;

// Verrou recursif : un seul proprietaire de l'etat a la fois (tache coeur, pages web)
class Lock {
 public:
  Lock()  { xSemaphoreTakeRecursive(g_mtx, portMAX_DELAY); }
  ~Lock() { xSemaphoreGiveRecursive(g_mtx); }
};

// ============================================================================
// 4. MOTIFS SONORES  {frequence Hz, duree ms}, frequence 0 = silence, duree 0 = fin
// ============================================================================
static const Note PAT_BOOT1[]      = {{BUZ_HI, 90}, {0, 0}};
static const Note PAT_BOOT2[]      = {{BUZ_HI, 90}, {0, 90}, {BUZ_HI, 90}, {0, 0}};
static const Note PAT_BOOT3[]      = {{BUZ_HI, 90}, {0, 90}, {BUZ_HI, 90}, {0, 90}, {BUZ_HI, 90}, {0, 0}};
static const Note PAT_BOOT4[]      = {{BUZ_HI, 90}, {0, 90}, {BUZ_HI, 90}, {0, 90}, {BUZ_HI, 90}, {0, 90}, {BUZ_HI, 90}, {0, 0}};
static const Note PAT_LINK_OK[]    = {{BUZ_LO, 120}, {BUZ_HI, 180}, {0, 0}};
static const Note PAT_LINK_LOST[]  = {{BUZ_LO, 450}, {0, 0}};
static const Note PAT_START[]      = {{0, 15}, {BUZ_HI, 300}, {0, 0}};
static const Note PAT_NOACK[]      = {{BUZ_LO, 150}, {0, 80}, {BUZ_LO, 150}, {0, 80}, {BUZ_LO, 150}, {0, 0}};
static const Note PAT_CROSS[]      = {{0, 10}, {BUZ_HI, 160}, {0, 0}};
static const Note PAT_BEST[]       = {{0, 10}, {2200, 110}, {2450, 110}, {BUZ_HI, 110}, {0, 40}, {BUZ_HI, 300}, {0, 0}};
static const Note PAT_ARRIVED[]    = {{BUZ_HI, 70}, {0, 70}, {BUZ_HI, 70}, {0, 0}};
static const Note PAT_LOW_BATT[]   = {{BUZ_LO, 120}, {0, 100}, {BUZ_LO, 120}, {0, 100}, {BUZ_LO, 120}, {0, 0}};
static const Note PAT_CANCELLED[]  = {{BUZ_HI, 90}, {0, 40}, {BUZ_LO, 320}, {0, 0}};
static const Note PAT_ERROR[]      = {{BUZ_LO, 900}, {0, 0}};
static const Note PAT_MAINT[]      = {{BUZ_HI, 70}, {0, 70}, {BUZ_LO, 70}, {0, 70}, {BUZ_HI, 70}, {0, 0}};
static const Note PAT_BEAM_FAULT[] = {{BUZ_LO, 220}, {0, 100}, {BUZ_LO, 220}, {0, 0}};
static const Note PAT_LOCATE[]     = {{BUZ_HI, 200}, {0, 150}, {BUZ_HI, 200}, {0, 150}, {BUZ_HI, 200}, {0, 150}, {BUZ_HI, 200}, {0, 0}};
static const Note PAT_CLICK[]      = {{BUZ_HI, 25}, {0, 0}};
static const Note PAT_ALIGN_LO[]   = {{BUZ_LO, 40}, {0, 0}};
static const Note PAT_ALIGN_HI[]   = {{BUZ_HI, 60}, {0, 0}};
static const Note PAT_ALIGN_FULL[] = {{BUZ_HI, 220}, {0, 0}};
static const Note PAT_TUNE[]       = {{BUZ_HI, 100}, {0, 0}};

// ============================================================================
// 5. UTILITAIRES
// ============================================================================
static inline int64_t nowUs() { return esp_timer_get_time(); }

static const char* myHostname() {
  return IS_START ? HOSTNAME_START : IS_FINISH ? HOSTNAME_FINISH : IS_EMITTER ? HOSTNAME_EMITTER : HOSTNAME_RELAY;
}

// Copie en remplacant les separateurs de fichier, sans couper un caractere UTF-8
static void copyClean(char* dst, size_t n, const char* src) {
  size_t j = 0;
  if (src) {
    while (*src == ' ') src++;
    for (size_t i = 0; src[i] && j + 1 < n; i++) {
      char c = src[i];
      if (c == ';' || c == '\t' || c == '\r' || c == '\n' || c == '"' || c == '\\') c = ' ';
      dst[j++] = c;
    }
  }
  if (j > 0) {
    size_t k = j - 1;
    while (k > 0 && ((uint8_t)dst[k] & 0xC0) == 0x80) k--;
    uint8_t lead = (uint8_t)dst[k];
    size_t need = (lead < 0x80) ? 1 : (lead >= 0xF0) ? 4 : (lead >= 0xE0) ? 3 : (lead >= 0xC0) ? 2 : 1;
    if (k + need > j) j = k;
  }
  while (j > 0 && dst[j - 1] == ' ') j--;
  dst[j] = 0;
}

static int splitFields(char* s, char sep, char** out, int maxOut) {
  int c = 0;
  if (!s || maxOut < 1) return 0;
  out[c++] = s;
  for (char* p = s; *p && c < maxOut; p++) {
    if (*p == sep) { *p = 0; out[c++] = p + 1; }
  }
  return c;
}

static long long toLL(const char* s) { return s ? strtoll(s, nullptr, 10) : 0; }
static unsigned long toUL(const char* s) { return s ? strtoul(s, nullptr, 10) : 0; }

static void fmtDur(char* out, size_t n, int64_t us) {
  if (us < 0) { snprintf(out, n, "--"); return; }
  int64_t cs = us / 10000;   // centiemes, tronques comme un chrono
  snprintf(out, n, "%d:%02d.%02d", (int)(cs / 6000), (int)((cs / 100) % 60), (int)(cs % 100));
}

static void fmtClockLocal(char* out, size_t n, int64_t wallMs) {
  if (wallMs <= 0) { snprintf(out, n, "-"); return; }
  int64_t s = wallMs / 1000 - (int64_t)g_tzOffsetMin * 60;
  int sec = (int)(s % 86400);
  if (sec < 0) sec += 86400;
  snprintf(out, n, "%02d:%02d:%02d", sec / 3600, (sec / 60) % 60, sec % 60);
}

static int64_t wallFromLocalUs(int64_t t) { return g_wallValid ? g_wallOffsetMs + t / 1000 : 0; }

static void prefsPutU(const char* key, uint32_t v);   // defini plus bas

// ---- Fenetre des ecritures flash ----
static void flashMark(bool begin) {
  int64_t t = esp_timer_get_time();
  portENTER_CRITICAL(&g_flashMux);
  if (begin) { g_flashFrom = t; g_flashActive = true; }
  else { g_flashTo = t; g_flashActive = false; }
  portEXIT_CRITICAL(&g_flashMux);
}

// Vrai si un horodatage a pu etre retarde par une ecriture flash
// (les interruptions Arduino ne sont pas en IRAM : elles attendent la fin de l'ecriture)
static bool flashOverlap(int64_t t) {
  portENTER_CRITICAL(&g_flashMux);
  int64_t f = g_flashFrom, e = g_flashTo;
  bool a = g_flashActive;
  portEXIT_CRITICAL(&g_flashMux);
  if (f == 0) return false;
  if (a) return t >= f - 500;
  return (t >= f - 500) && (t <= e + 3000);
}

static void prefsPutU(const char* key, uint32_t v) {
  flashMark(true);
  g_prefs.putUInt(key, v);
  flashMark(false);
}

// Supprime toutes les archives (/arch). Les noms sont relus par paquets :
// on ne supprime pas un fichier pendant qu'on parcourt le dossier.
static void fsWipeArchives() {
  for (int pass = 0; pass < 16; pass++) {
    String names[16];
    int n = 0;
    {
      File dir = LittleFS.open("/arch");
      if (!dir || !dir.isDirectory()) return;
      File f = dir.openNextFile();
      while (f && n < 16) {
        names[n++] = String("/arch/") + f.name();
        f = dir.openNextFile();
      }
    }
    if (n == 0) return;
    for (int i = 0; i < n; i++) LittleFS.remove(names[i]);
  }
}

static bool fsAppend(const char* path, const char* line) {
  if (!g_fsOk) return false;
  flashMark(true);
  bool ok = false;
  File f = LittleFS.open(path, FILE_APPEND);
  if (f) {
    ok = f.print(line) > 0;
    f.print('\n');
    f.close();
  }
  flashMark(false);
  return ok;
}

static void storeAppendf(const char* path, const char* fmt, ...) {
  StoreMsg m;
  memset(&m, 0, sizeof m);
  m.op = SO_APPEND;
  strlcpy(m.path, path, sizeof m.path);
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(m.line, sizeof m.line, fmt, ap);
  va_end(ap);
  if (!g_storeQ || xQueueSend(g_storeQ, &m, pdMS_TO_TICKS(20)) != pdTRUE) g_storeDrops++;
}

static bool storeOp(uint8_t op, const char* path, const char* line, void* ptr) {
  StoreMsg m;
  memset(&m, 0, sizeof m);
  m.op = op;
  if (path) strlcpy(m.path, path, sizeof m.path);
  if (line) strlcpy(m.line, line, sizeof m.line);
  m.ptr = ptr;
  if (!g_storeQ || xQueueSend(g_storeQ, &m, pdMS_TO_TICKS(200)) != pdTRUE) { g_storeDrops++; return false; }
  return true;
}

static void storeSaveRiders() {
  if (g_ridersSavePending) return;
  g_ridersSavePending = true;
  if (!storeOp(SO_SAVE_RIDERS, nullptr, nullptr, nullptr)) g_ridersSavePending = false;   // reessaye au prochain changement
}

// ============================================================================
// 6. BUZZER
// ============================================================================
static void beep(const Note* p) {
  if (!g_buzQ || !p) return;
  BuzItem it;
  memset(&it, 0, sizeof it);
  for (int i = 0; i < BUZ_MAX_NOTES - 1 && p[i].ms; i++) it.n[i] = p[i];
  xQueueSend(g_buzQ, &it, 0);
}

static void buzzerTask(void*) {
  BuzItem it;
  for (;;) {
    if (xQueueReceive(g_buzQ, &it, portMAX_DELAY) != pdTRUE) continue;
    for (int i = 0; i < BUZ_MAX_NOTES && it.n[i].ms; i++) {
      if (g_otaRunning) break;
      ledcWriteTone(PIN_BUZZER, it.n[i].freq);
      vTaskDelay(pdMS_TO_TICKS(it.n[i].ms));
    }
    ledcWriteTone(PIN_BUZZER, 0);
  }
}

// ============================================================================
// 7. BATTERIE
// ============================================================================
static uint16_t readBatteryMv() {
  uint32_t s = 0;
  for (int i = 0; i < 16; i++) s += analogReadMilliVolts(PIN_BATT);
  float mv = (s / 16.0f) * BATT_DIVIDER * BATT_CAL;
  return (uint16_t)mv;
}

static int battPercent(uint16_t mv) {
  static const uint16_t T[][2] = {{4200, 100}, {4100, 88}, {4000, 76}, {3900, 62}, {3800, 46}, {3750, 36},
                                  {3700, 26}, {3650, 16}, {3600, 10}, {3500, 5}, {3300, 0}};
  if (mv >= 4200) return 100;
  for (size_t i = 1; i < sizeof(T) / sizeof(T[0]); i++) {
    if (mv >= T[i][0]) {
      return T[i][1] + (int)(mv - T[i][0]) * (T[i - 1][1] - T[i][1]) / (T[i - 1][0] - T[i][0]);
    }
  }
  return 0;
}

static int battLevel4() {
  if (g_battMv < BATT_ABSENT_MV) return 4;
  int p = battPercent(g_battMv);
  return p >= 75 ? 4 : p >= 50 ? 3 : p >= 25 ? 2 : 1;
}

static void loraSleep();   // section 8b

static void shutdownLowBattery() {
  LOGF("[batt] %u mV : coupure pour proteger la batterie\n", g_battMv);
  beep(PAT_ERROR);
  uint32_t t0 = millis();
  while (g_storeQ && uxQueueMessagesWaiting(g_storeQ) && millis() - t0 < 2000) delay(10);
  delay(1200);
  loraSleep();
  if (IS_EMITTER) rmtDeinit(PIN_IR_TX);
  pinMode(PIN_IR_TX, OUTPUT);
  digitalWrite(PIN_IR_TX, LOW);
  gpio_hold_en((gpio_num_t)PIN_IR_TX);
  WiFi.mode(WIFI_OFF);
  esp_deep_sleep_start();   // reveil uniquement par l'interrupteur
}

static void batteryService(int64_t now) {
  static int64_t last = 0;
  if (last && now - last < 2000000) return;
  last = now;
  uint16_t mv = readBatteryMv();
  g_battMv = g_battMv ? (uint16_t)((g_battMv * 3u + mv) / 4u) : mv;
  if (g_battMv < BATT_ABSENT_MV) { g_cutoffSinceUs = 0; return; }   // USB seul
  if (g_battMv < BATT_WARN_MV && now - g_lowBattBeepUs > 60000000LL) {
    g_lowBattBeepUs = now;
    beep(PAT_LOW_BATT);
  }
  if (BATT_CUTOFF_ENABLE && g_battMv < BATT_CUTOFF_MV) {
    if (!g_cutoffSinceUs) g_cutoffSinceUs = now;
    else if (now - g_cutoffSinceUs > 30000000LL) shutdownLowBattery();
  } else {
    g_cutoffSinceUs = 0;
  }
}

// ============================================================================
// 8. RADIO ESP-NOW
//    Tous les boitiers emettent sur l'interface station (STA), en mode Long Range
//    si RADIO_LONG_RANGE = 1. Le point d'acces des telephones (depart, arrivee) reste
//    en 802.11 b/g/n normal : un point d'acces LR n'est pas visible par un telephone.
//
//    Routage : un message vers le depart ou l'arrivee part en direct (si le lien direct
//    a ete vu recemment) ET via le relais (si un relais est present). Le premier
//    exemplaire recu est traite, les doublons sont ecartes (role, boot, numero).
// ============================================================================
static PktHeader makeHeader(uint8_t type) {
  PktHeader h;
  h.magic = PKT_MAGIC;
  h.version = PKT_VERSION;
  h.type = type;
  h.netId = NET_ID;
  h.role = CHRONO_ROLE;
  h.flags = 0;
  h.bootId = g_bootId;
  h.seq = __atomic_add_fetch(&g_txSeq, 1, __ATOMIC_RELAXED);
  h.dst = 0;
  h.hop = 0;
  return h;
}

// Debit Long Range (250 ou 500 kbit/s) pour un pair ESP-NOW
static void setPeerRate(const uint8_t* mac) {
#if RADIO_LONG_RANGE && defined(ESP_IDF_VERSION_VAL)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
  if (!g_lr) return;
  esp_now_rate_config_t rc;
  memset(&rc, 0, sizeof rc);
  rc.phymode = WIFI_PHY_MODE_LR;
  rc.rate = (RADIO_LR_RATE == 500) ? WIFI_PHY_RATE_LORA_500K : WIFI_PHY_RATE_LORA_250K;
  esp_err_t e = esp_now_set_peer_rate_config(mac, &rc);
  if (e != ESP_OK) LOGF("[radio] debit LR refuse pour un pair (erreur %d)\n", (int)e);
#endif
#endif
  (void)mac;
}

static bool ensurePeer(const uint8_t* mac) {
  if (!g_espNowOk) return false;
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p;
  memset(&p, 0, sizeof p);
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;          // canal courant
  p.ifidx = g_espIf;
  p.encrypt = false;
  if (esp_now_add_peer(&p) != ESP_OK) return false;
  setPeerRate(mac);
  return true;
}

static void sendRaw(const uint8_t* mac, const void* data, size_t len) {
  if (!g_espNowOk) return;
  esp_now_send(mac, (const uint8_t*)data, len);
}

static void patchRoute(uint8_t* buf, uint8_t dst, uint8_t hop) {
  PktHeader h;
  memcpy(&h, buf, sizeof h);
  h.dst = dst;
  h.hop = hop;
  memcpy(buf, &h, sizeof h);
}

static bool peerOk(uint8_t role, int64_t now) {
  const PeerInfo& p = g_peers[role];
  return p.known && (now - p.lastSeenUs) < PEER_TIMEOUT_US;
}

// Lien direct entendu recemment (le role a ete recu sans passer par le relais)
static bool directOk(uint8_t role, int64_t now) {
  const PeerInfo& p = g_peers[role];
  return p.macKnown && p.lastDirectUs && (now - p.lastDirectUs) < (int64_t)LINK_TIMEOUT_MS * 1000;
}

static bool viaRelayRecent(uint8_t role, int64_t now) {
  const PeerInfo& p = g_peers[role];
  return p.lastRelayUs && (now - p.lastRelayUs) < (int64_t)LINK_TIMEOUT_MS * 1000;
}

// Seuls les echanges depart <-> arrivee passent par le relais
static bool relayUsable(uint8_t role, int64_t now) {
  if (!(IS_START || IS_FINISH)) return false;
  if (role != ROLE_START && role != ROLE_FINISH) return false;
  return directOk(ROLE_RELAY, now);
}

static void sendDirect(uint8_t role, const void* data, size_t len) {
  if (len > MAX_PKT) return;
  uint8_t buf[MAX_PKT];
  memcpy(buf, data, len);
  patchRoute(buf, role, 0);
  uint8_t mac[6];
  memcpy(mac, BCAST_MAC, 6);
  {
    Lock l;
    if (role >= 1 && role <= 4 && g_peers[role].macKnown) memcpy(mac, g_peers[role].mac, 6);
  }
  sendRaw(mac, buf, len);
}

static void sendViaRelay(uint8_t role, const void* data, size_t len) {
  if (len > MAX_PKT) return;
  uint8_t buf[MAX_PKT];
  memcpy(buf, data, len);
  patchRoute(buf, role, 0);   // dst = destinataire final ; le relais le sait grace a dst
  uint8_t mac[6];
  {
    Lock l;
    if (!g_peers[ROLE_RELAY].macKnown) return;
    memcpy(mac, g_peers[ROLE_RELAY].mac, 6);
  }
  sendRaw(mac, buf, len);
}

static void sendToRole(uint8_t role, const void* data, size_t len) {
  int64_t now = nowUs();
  bool viaRelay, direct;
  {
    Lock l;
    viaRelay = relayUsable(role, now);
    direct = !viaRelay || directOk(role, now);
  }
  if (direct) sendDirect(role, data, len);
  if (viaRelay) sendViaRelay(role, data, len);
}

// Callback dans la tache Wi-Fi : on horodate, on copie, on ressort.
static void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  int64_t t = esp_timer_get_time();
  if (len < (int)sizeof(PktHeader) || len > (int)MAX_PKT) return;
  PktHeader h;
  memcpy(&h, data, sizeof h);
  if (h.magic != PKT_MAGIC || h.version != PKT_VERSION || h.netId != NET_ID) return;
  if (h.role == CHRONO_ROLE || h.role < 1 || h.role > 4) return;
  InMsg m;
  m.tRx = t;
  memcpy(m.mac, info->src_addr, 6);
  m.rssi = info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0;
  m.snr = 0;
  m.lora = 0;
  m.len = (uint8_t)len;
  memcpy(m.data, data, len);
  xQueueSend(g_inQ, &m, 0);
}

// longRange = false en mode maintenance (connexion Wi-Fi normale a l'arrivee)
static bool radioInit(wifi_interface_t ifx, bool longRange) {
  g_espIf = ifx;
  g_lr = longRange && RADIO_LONG_RANGE && ifx == WIFI_IF_STA;
  esp_wifi_set_ps(WIFI_PS_NONE);   // modem toujours eveille : reception fiable et delais stables
  if (g_lr) {
    // b/g/n gardes en plus du LR : la station reste compatible (maintenance, telephones)
    esp_err_t e = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
    if (e != ESP_OK) { LOGF("[radio] Long Range indisponible (erreur %d)\n", (int)e); g_lr = false; }
  }
  if (esp_now_init() != ESP_OK) {
    LOGF("[radio] esp_now_init a echoue\n");
    return false;
  }
  esp_now_register_recv_cb(onEspNowRecv);
  g_espNowOk = true;
#if RADIO_LONG_RANGE && defined(ESP_IDF_VERSION_VAL)
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 3, 0)
  if (g_lr) esp_wifi_config_espnow_rate(WIFI_IF_STA, (RADIO_LR_RATE == 500) ? WIFI_PHY_RATE_LORA_500K : WIFI_PHY_RATE_LORA_250K);
#endif
#endif
  ensurePeer(BCAST_MAC);
  LOGF("[radio] ESP-NOW %s\n", g_lr ? (RADIO_LR_RATE == 500 ? "Long Range 500 kbit/s" : "Long Range 250 kbit/s") : "standard");
  return true;
}

// ---- Doublons (un message peut arriver en direct et via le relais) ----
static bool seenBefore(uint8_t role, uint32_t boot, uint32_t seq) {
  for (int i = 0; i < SEEN_SIZE; i++)
    if (g_seen[i].seq == seq && g_seen[i].boot == boot && g_seen[i].role == role) return true;
  g_seen[g_seenHead].role = role;
  g_seen[g_seenHead].boot = boot;
  g_seen[g_seenHead].seq = seq;
  g_seenHead = (uint8_t)((g_seenHead + 1) % SEEN_SIZE);
  return false;
}

// ---- Qualite de liaison : messages de vie (1 par seconde) recus sur 10 s ----
static void hbRecord(PeerInfo& p, const PktHeader& h, int64_t t) {
  uint8_t bit = h.hop ? 2 : 1;
  for (int i = 0; i < HB_HIST; i++) {
    HbSeen& e = p.hb[i];
    if (e.t && e.seq == h.seq && e.boot == h.bootId) { e.paths |= bit; return; }
  }
  int k = 0;
  for (int i = 1; i < HB_HIST; i++) if (p.hb[i].t < p.hb[k].t) k = i;
  p.hb[k].seq = h.seq;
  p.hb[k].boot = h.bootId;
  p.hb[k].t = t;
  p.hb[k].paths = bit;
}

// Pourcentages : direct, via relais, par au moins un chemin
static void hbStats(const PeerInfo& p, int64_t now, uint8_t& d, uint8_t& r, uint8_t& a) {
  int nd = 0, nr = 0, na = 0;
  for (int i = 0; i < HB_HIST; i++) {
    const HbSeen& e = p.hb[i];
    if (!e.t || now - e.t > 10500000LL) continue;
    na++;
    if (e.paths & 1) nd++;
    if (e.paths & 2) nr++;
  }
  d = (uint8_t)(nd >= 10 ? 100 : nd * 10);
  r = (uint8_t)(nr >= 10 ? 100 : nr * 10);
  a = (uint8_t)(na >= 10 ? 100 : na * 10);
}

// ============================================================================
// 8b. LORA : RFM95W (SX1276) sur le connecteur prevu pour l'ecran
//     Pilote registre par registre (pas de bibliotheque) : on maitrise l'instant exact
//     ou chaque paquet part, ce qui sert a la synchro.
//     - Reception continue ; DIO0 monte a la fin d'une reception (RxDone) ou d'une
//       emission (TxDone). L'interruption ne fait qu'horodater.
//     - Emission par file d'attente avec priorites et respect du temps d'emission
//       autorise (LORA_DUTY_PCT, 10 % dans la sous-bande 869,4 - 869,65 MHz).
//     - Synchro : t1 / t3 sont pris juste avant l'ecriture du paquet dans le module,
//       t2 / t4 a l'interruption RxDone. Requete et reponse ont la meme taille : les
//       temps d'emission (environ 100 ms en SF7) sont identiques et s'annulent.
// ============================================================================
static constexpr uint8_t  LR_REG_FIFO = 0x00, LR_REG_OPMODE = 0x01, LR_REG_FRF_MSB = 0x06, LR_REG_PA_CONFIG = 0x09,
                          LR_REG_OCP = 0x0B, LR_REG_LNA = 0x0C, LR_REG_FIFO_PTR = 0x0D, LR_REG_FIFO_TX_BASE = 0x0E,
                          LR_REG_FIFO_RX_BASE = 0x0F, LR_REG_FIFO_RX_CUR = 0x10, LR_REG_IRQ_FLAGS = 0x12, LR_REG_RX_NB = 0x13,
                          LR_REG_MODEM_STAT = 0x18, LR_REG_PKT_SNR = 0x19, LR_REG_PKT_RSSI = 0x1A, LR_REG_MODEM_CFG1 = 0x1D,
                          LR_REG_MODEM_CFG2 = 0x1E, LR_REG_PREAMBLE_MSB = 0x20, LR_REG_PREAMBLE_LSB = 0x21,
                          LR_REG_PAYLOAD_LEN = 0x22, LR_REG_MODEM_CFG3 = 0x26, LR_REG_SYNC_WORD = 0x39,
                          LR_REG_DIO_MAP1 = 0x40, LR_REG_VERSION = 0x42, LR_REG_PA_DAC = 0x4D;
static constexpr uint8_t  LR_MODE_SLEEP = 0x80, LR_MODE_STDBY = 0x81, LR_MODE_TX = 0x83, LR_MODE_RXCONT = 0x85;
static constexpr uint8_t  LR_IRQ_RX_DONE = 0x40, LR_IRQ_CRC_ERR = 0x20, LR_IRQ_TX_DONE = 0x08;
static constexpr bool     LORA_LDRO = (LORA_SF >= 11);   // obligatoire au-dela de 16 ms par symbole (125 kHz)
static constexpr int64_t  LORA_BUCKET_US = 60LL * 1000000 * LORA_DUTY_PCT / 100;   // reserve de temps d'emission (1 min)

static const SPISettings g_loraSpi(8000000, MSBFIRST, SPI_MODE0);

static uint8_t loraRead(uint8_t reg) {
  SPI.beginTransaction(g_loraSpi);
  digitalWrite(PIN_LORA_NSS, LOW);
  SPI.transfer(reg & 0x7F);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(PIN_LORA_NSS, HIGH);
  SPI.endTransaction();
  return v;
}

static void loraWrite(uint8_t reg, uint8_t v) {
  SPI.beginTransaction(g_loraSpi);
  digitalWrite(PIN_LORA_NSS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(v);
  digitalWrite(PIN_LORA_NSS, HIGH);
  SPI.endTransaction();
}

static void loraFifo(bool write, uint8_t* d, uint8_t n) {
  SPI.beginTransaction(g_loraSpi);
  digitalWrite(PIN_LORA_NSS, LOW);
  SPI.transfer(write ? (LR_REG_FIFO | 0x80) : LR_REG_FIFO);
  for (uint8_t i = 0; i < n; i++) {
    uint8_t v = SPI.transfer(write ? d[i] : 0x00);
    if (!write) d[i] = v;
  }
  digitalWrite(PIN_LORA_NSS, HIGH);
  SPI.endTransaction();
}

// Duree d'emission (formule Semtech AN1200.13) : 125 kHz, CR 4/5, en-tete explicite, CRC
static uint32_t loraAirtimeUs(uint8_t len) {
  const double tsym = (double)(1UL << LORA_SF) / 125000.0 * 1e6;
  const int de = LORA_LDRO ? 1 : 0;
  double num = 8.0 * len - 4.0 * LORA_SF + 28 + 16;
  double nsym = 8 + fmax(ceil(num / (4.0 * (LORA_SF - 2 * de))) * 5.0, 0.0);
  return (uint32_t)((8 + 4.25 + nsym) * tsym);
}

static void IRAM_ATTR isrLora() {
  HwEvt e;
  e.kind = HW_LORA;
  e.t = esp_timer_get_time();
  BaseType_t w = pdFALSE;
  xQueueSendFromISR(g_hwQ, &e, &w);
  if (w) portYIELD_FROM_ISR();
}

static void loraStartRx() {
  loraWrite(LR_REG_OPMODE, LR_MODE_STDBY);
  loraWrite(LR_REG_DIO_MAP1, 0x00);      // DIO0 = RxDone
  loraWrite(LR_REG_FIFO_PTR, 0x00);
  loraWrite(LR_REG_IRQ_FLAGS, 0xFF);
  loraWrite(LR_REG_OPMODE, LR_MODE_RXCONT);
}

static void loraSleep() {
  if (g_loraOk) loraWrite(LR_REG_OPMODE, LR_MODE_SLEEP);
}

// Detection et configuration ; false si le module est absent (le chrono reste en ESP-NOW)
static bool loraInit() {
  if (!LORA_ME) return false;
  pinMode(PIN_LORA_NSS, OUTPUT);
  digitalWrite(PIN_LORA_NSS, HIGH);
  pinMode(PIN_LORA_RST, OUTPUT);
  digitalWrite(PIN_LORA_RST, LOW);       // reset : 100 us mini a l'etat bas, puis relache
  delay(1);
  pinMode(PIN_LORA_RST, INPUT);
  delay(10);
  pinMode(PIN_LORA_DIO0, INPUT);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
  uint8_t ver = loraRead(LR_REG_VERSION);
  if (ver != 0x12) {
    LOGF("[lora] module absent (version lue 0x%02X) : ESP-NOW seul\n", ver);
    SPI.end();
    pinMode(PIN_SPI_MISO, OUTPUT);         // D9 commande aussi l'etage IR : on le laisse a l'etat bas
    digitalWrite(PIN_SPI_MISO, LOW);
    return false;
  }
  loraWrite(LR_REG_OPMODE, 0x00);        // le mode LoRa ne se choisit qu'en sommeil
  loraWrite(LR_REG_OPMODE, LR_MODE_SLEEP);
  delay(2);
  uint64_t frf = ((uint64_t)LORA_FREQ_HZ << 19) / 32000000ULL;
  loraWrite(LR_REG_FRF_MSB, (uint8_t)(frf >> 16));
  loraWrite(LR_REG_FRF_MSB + 1, (uint8_t)(frf >> 8));
  loraWrite(LR_REG_FRF_MSB + 2, (uint8_t)frf);
  if (LORA_TX_DBM >= 20) {               // 20 dBm : PA_BOOST + PA_DAC haute puissance
    loraWrite(LR_REG_PA_DAC, 0x87);
    loraWrite(LR_REG_PA_CONFIG, 0x8F);
    loraWrite(LR_REG_OCP, 0x32);         // limite de courant 150 mA
  } else {
    loraWrite(LR_REG_PA_DAC, 0x84);
    loraWrite(LR_REG_PA_CONFIG, (uint8_t)(0x80 | (LORA_TX_DBM - 2)));
    loraWrite(LR_REG_OCP, 0x2B);         // 100 mA
  }
  loraWrite(LR_REG_LNA, 0x23);           // gain max + boost
  loraWrite(LR_REG_MODEM_CFG1, 0x72);    // 125 kHz, CR 4/5, en-tete explicite
  loraWrite(LR_REG_MODEM_CFG2, (uint8_t)((LORA_SF << 4) | 0x04));   // SF, CRC
  loraWrite(LR_REG_MODEM_CFG3, (uint8_t)(0x04 | (LORA_LDRO ? 0x08 : 0x00)));   // AGC auto
  loraWrite(LR_REG_PREAMBLE_MSB, 0x00);
  loraWrite(LR_REG_PREAMBLE_LSB, 0x08);
  loraWrite(LR_REG_SYNC_WORD, LORA_SYNC_WORD);
  loraWrite(LR_REG_FIFO_TX_BASE, 0x00);
  loraWrite(LR_REG_FIFO_RX_BASE, 0x00);
  attachInterrupt(digitalPinToInterrupt(PIN_LORA_DIO0), isrLora, RISING);
  g_loraOk = true;
  g_loraTokens = LORA_BUCKET_US;
  g_loraTokT = nowUs();
  loraStartRx();
  LOGF("[lora] OK : %.3f MHz, SF%d, %d dBm, synchro %u ms\n", LORA_FREQ_HZ / 1e6, LORA_SF, LORA_TX_DBM,
       (unsigned)LORA_SYNC_MS);
  return true;
}

// File d'emission : prio 0 = evenement de course, 1 = synchro, 2 = message de vie
// kind 1 = requete de synchro (t1 pose a l'emission), 2 = reponse (t3 pose a l'emission)
static void loraQueue(const uint8_t* data, uint8_t len, uint8_t prio, uint8_t kind) {
  if (!g_loraOk || len > MAX_PKT) return;
  int slot = -1;
  if (g_loraQn < LORA_QLEN) {
    slot = g_loraQn++;
  } else {
    int worst = 0;
    for (int i = 1; i < LORA_QLEN; i++)
      if (g_loraQ[i].prio > g_loraQ[worst].prio || (g_loraQ[i].prio == g_loraQ[worst].prio && g_loraQ[i].queued < g_loraQ[worst].queued))
        worst = i;
    g_loraDrop++;
    if (g_loraQ[worst].prio < prio) return;   // tout est plus important que ce paquet
    slot = worst;
  }
  LoraFrame& f = g_loraQ[slot];
  f.len = len;
  f.prio = prio;
  f.kind = kind;
  f.queued = nowUs();
  memcpy(f.data, data, len);
}

// dst = role destinataire (0 = tous)
static void loraSendTo(uint8_t dst, const void* data, size_t len, uint8_t prio, uint8_t kind) {
  if (!g_loraOk || len > MAX_PKT) return;
  uint8_t buf[MAX_PKT];
  memcpy(buf, data, len);
  patchRoute(buf, dst, 0);
  loraQueue(buf, (uint8_t)len, prio, kind);
}

static void loraTransmit(LoraFrame& f) {
  if (f.kind == 1 || f.kind == 2) {      // horodatage juste avant l'ecriture dans le module
    PktSync s;
    memcpy(&s, f.data, sizeof s);
    if (f.kind == 1) s.t1 = nowUs();
    else s.t3 = nowUs();
    memcpy(f.data, &s, sizeof s);
  }
  loraWrite(LR_REG_OPMODE, LR_MODE_STDBY);
  loraWrite(LR_REG_DIO_MAP1, 0x40);      // DIO0 = TxDone
  loraWrite(LR_REG_FIFO_PTR, 0x00);
  loraFifo(true, f.data, f.len);
  loraWrite(LR_REG_PAYLOAD_LEN, f.len);
  loraWrite(LR_REG_IRQ_FLAGS, 0xFF);
  loraWrite(LR_REG_OPMODE, LR_MODE_TX);
  uint32_t air = loraAirtimeUs(f.len);
  int64_t now = nowUs();
  g_loraTx = true;
  g_loraTxDeadlineUs = now + air + air / 4 + 50000;
  g_loraTokens -= air;
  g_loraTxCount++;
  // Apres une requete de synchro, on laisse la place a la reponse
  if (f.kind == 1) g_loraHoldUntilUs = now + 2LL * air + 300000;
}

static void handlePacket(const InMsg& m);   // section 17

// DIO0 : fin d'emission ou de reception. t = heure de l'interruption.
static void loraOnDio0(int64_t t) {
  if (!g_loraOk) return;
  g_loraLastDio0Us = t;
  uint8_t f = loraRead(LR_REG_IRQ_FLAGS);
  loraWrite(LR_REG_IRQ_FLAGS, 0xFF);
  if (g_loraTx) {
    g_loraTx = false;
    loraStartRx();
    return;
  }
  if (!(f & LR_IRQ_RX_DONE)) return;
  if (f & LR_IRQ_CRC_ERR) { g_loraCrcErr++; return; }
  uint8_t n = loraRead(LR_REG_RX_NB);
  loraWrite(LR_REG_FIFO_PTR, loraRead(LR_REG_FIFO_RX_CUR));
  if (n < sizeof(PktHeader) || n > MAX_PKT) return;
  InMsg m;
  memset(&m, 0, sizeof m);
  loraFifo(false, m.data, n);
  int snr = (int8_t)loraRead(LR_REG_PKT_SNR) / 4;
  int rssi = -157 + loraRead(LR_REG_PKT_RSSI) + (snr < 0 ? snr : 0);
  g_loraLastRssi = rssi;
  g_loraLastSnr = (int8_t)snr;
  PktHeader h;
  memcpy(&h, m.data, sizeof h);
  if (h.magic != PKT_MAGIC || h.version != PKT_VERSION || h.netId != NET_ID) return;
  if (h.role == CHRONO_ROLE || h.role < 1 || h.role > 4) return;
  m.tRx = t;
  m.rssi = (int8_t)(rssi < -128 ? -128 : rssi);
  m.snr = (int8_t)snr;
  m.len = n;
  m.lora = 1;
  g_loraRxCount++;
  handlePacket(m);
}

// Appelee toutes les ~2 ms par la tache coeur
static void loraService(int64_t now) {
  if (!g_loraOk) return;
  g_loraTokens += (now - g_loraTokT) * LORA_DUTY_PCT / 100;
  if (g_loraTokens > LORA_BUCKET_US) g_loraTokens = LORA_BUCKET_US;
  g_loraTokT = now;
  // Interruption manquee (DIO0 reste haut) : on rattrape par scrutation
  if (digitalRead(PIN_LORA_DIO0) == HIGH && now - g_loraLastDio0Us > 50000) loraOnDio0(now);
  if (g_loraTx) {
    if (now > g_loraTxDeadlineUs) { g_loraTimeouts++; g_loraTx = false; loraStartRx(); }
    return;
  }
  if (g_loraQn == 0 || now < g_loraHoldUntilUs) return;
  int best = 0;
  for (int i = 1; i < g_loraQn; i++)
    if (g_loraQ[i].prio < g_loraQ[best].prio || (g_loraQ[i].prio == g_loraQ[best].prio && g_loraQ[i].queued < g_loraQ[best].queued))
      best = i;
  LoraFrame& f = g_loraQ[best];
  // Messages de vie et synchro trop anciens : inutiles, on les jette
  if (f.prio > 0 && now - f.queued > 3LL * LORA_SYNC_MS * 1000) {
    g_loraQ[best] = g_loraQ[--g_loraQn];
    return;
  }
  if (g_loraTokens < (int64_t)loraAirtimeUs(f.len)) return;   // reserve d'emission epuisee : on attend
  // Une reception est en cours : on ne la coupe pas (l'emetteur LoRa est semi-duplex)
  if ((loraRead(LR_REG_MODEM_STAT) & 0x0B) && now - f.queued < 1000000) return;
  LoraFrame tx = f;
  g_loraQ[best] = g_loraQ[--g_loraQn];
  loraTransmit(tx);
}

// Part des messages de vie LoRa recus (un toutes les LORA_HB_MS) sur 6 periodes
static uint8_t loraPct(const PeerInfo& p, int64_t now) {
  int n = 0;
  for (int i = 0; i < 6; i++)
    if (p.loraHbT[i] && now - p.loraHbT[i] < 6LL * LORA_HB_MS * 1000 + 500000) n++;
  return (uint8_t)(n >= 6 ? 100 : n * 100 / 6);
}

static bool loraRecent(uint8_t role, int64_t now) {
  const PeerInfo& p = g_peers[role];
  return p.lastLoraUs && now - p.lastLoraUs < 3LL * LORA_SYNC_MS * 1000;
}

// ============================================================================
// 9. SYNCHRONISATION D'HORLOGES (arrivee)
//    offset = horloge depart - horloge arrivee, modele lineaire a + b * t
//    Deux chemins possibles (direct, via relais). Le relais ajoute dans le paquet le
//    temps passe chez lui (principe de l'horloge transparente du PTP) : on le retire.
//    La requete et la reponse ont la meme taille, donc la meme duree d'emission :
//    le temps de vol est symetrique et s'annule dans le calcul de l'offset.
// ============================================================================
static SyncModel* modelFind(uint32_t startBoot) {
  for (int i = 0; i < 2; i++)
    if (g_models[i].used && g_models[i].startBoot == startBoot) return &g_models[i];
  return nullptr;
}

static SyncModel* modelGet(uint32_t startBoot) {
  SyncModel* m = modelFind(startBoot);
  if (m) return m;
  int slot;
  if (!g_models[0].used) slot = 0;
  else if (!g_models[1].used) slot = 1;
  else slot = (g_models[0].lastPointUs <= g_models[1].lastPointUs) ? 0 : 1;
  memset(&g_models[slot], 0, sizeof(SyncModel));
  g_models[slot].used = true;
  g_models[slot].startBoot = startBoot;
  g_models[slot].floor[0] = g_models[slot].floor[1] = g_models[slot].floor[2] = UINT32_MAX;
  return &g_models[slot];
}

static double modelOffsetAt(const SyncModel& m, double tf) {
  return m.a + m.b * ((tf - (double)m.tRef) / 1e6);
}

// Convertit un horodatage du depart dans l'horloge de l'arrivee : tf = ts - offset(tf)
static int64_t modelStartToFinish(const SyncModel& m, int64_t ts) {
  double tf = (double)ts - m.a;
  tf = (double)ts - modelOffsetAt(m, tf);
  tf = (double)ts - modelOffsetAt(m, tf);
  return (int64_t)llround(tf);
}

// Ecart d'un aller-retour au plus court jamais vu sur le meme chemin : c'est la part
// aleatoire (files d'attente), seule capable de rendre le trajet asymetrique.
static uint32_t pointExcess(const SyncModel& m, const SyncPoint& p) {
  uint32_t f = m.floor[p.path < 3 ? p.path : 0];
  return (f == UINT32_MAX || p.rtt < f) ? 0 : p.rtt - f;
}

// Incertitude estimee : demi-ecart moyen + residus + derive possible (5 ppm) + gigue d'horodatage
static uint32_t modelUncertAt(const SyncModel& m, int64_t t) {
  if (!m.valid) return UINT32_MAX;
  double ageS = fabs((double)(t - m.lastPointUs)) / 1e6;
  double u = m.excessUs / 2.0 + m.residUs + ageS * 5.0 + 100.0;
  return u > 4.0e9 ? UINT32_MAX : (uint32_t)u;
}

static void modelFit(SyncModel& m) {
  if (m.n == 0) return;
  uint32_t minE = UINT32_MAX;
  for (uint8_t i = 0; i < m.n; i++) { uint32_t e = pointExcess(m, m.pts[i]); if (e < minE) minE = e; }
  uint32_t lim = minE * 2 + 500;
  uint8_t last = (m.head + SYNC_POINTS - 1) % SYNC_POINTS;
  int64_t tRef = m.pts[last].tf;
  int64_t oRef = m.pts[last].off;
  double sx = 0, sy = 0, sxx = 0, sxy = 0, xmin = 1e18, xmax = -1e18, se = 0;
  int cnt = 0;
  for (uint8_t i = 0; i < m.n; i++) {
    const SyncPoint& p = m.pts[i];
    uint32_t e = pointExcess(m, p);
    if (e > lim) continue;
    double x = (double)(p.tf - tRef) / 1e6;
    double y = (double)(p.off - oRef);
    sx += x; sy += y; sxx += x * x; sxy += x * y; se += e; cnt++;
    if (x < xmin) xmin = x;
    if (x > xmax) xmax = x;
  }
  if (cnt == 0) return;
  double a = sy / cnt, b = 0;
  if (cnt >= 4 && (xmax - xmin) >= 10.0) {
    double den = cnt * sxx - sx * sx;
    if (fabs(den) > 1e-9) b = (cnt * sxy - sx * sy) / den;
    if (b > 200) b = 200;
    if (b < -200) b = -200;
    a = (sy - b * sx) / cnt;
  }
  double r2 = 0;
  for (uint8_t i = 0; i < m.n; i++) {
    const SyncPoint& p = m.pts[i];
    if (pointExcess(m, p) > lim) continue;
    double x = (double)(p.tf - tRef) / 1e6;
    double e = (double)(p.off - oRef) - (a + b * x);
    r2 += e * e;
  }
  m.a = (double)oRef + a;
  m.b = b;
  m.tRef = tRef;
  m.residUs = sqrt(r2 / cnt);
  m.excessUs = se / cnt;
  m.minRtt = (m.floor[0] != UINT32_MAX) ? m.floor[0] : (m.floor[1] != UINT32_MAX) ? m.floor[1] : m.floor[2];
  m.lastPointUs = tRef;
  m.valid = true;
}

// path : 0 = direct, 1 = via relais, 2 = LoRa
static void syncOnResp(const PktSync& r, uint32_t startBoot, int64_t t4, uint8_t path) {
  if (path > 2) return;
  int64_t t1 = r.t1, t2 = r.t2, t3 = r.t3;
  int64_t rq = r.resReq, rs = r.resResp;   // temps passe dans le relais, aller et retour
  int64_t rtt = (t4 - t1) - (t3 - t2) - rq - rs;
  int64_t rttMax = (path == 2) ? 2LL * loraAirtimeUs(sizeof(PktSync)) + 300000 : (int64_t)SYNC_MAX_RTT_US;
  if (rtt < 0 || rtt > rttMax) return;
  int64_t off = ((t2 - t1 - rq) + (t3 - t4 + rs)) / 2;
  SyncModel* m = modelGet(startBoot);
  m->samples++;
  if ((uint32_t)rtt < m->floor[path]) m->floor[path] = (uint32_t)rtt;
  SyncPoint sp;
  sp.tf = t1 + (t4 - t1) / 2;
  sp.off = off;
  sp.rtt = (uint32_t)rtt;
  sp.path = path;
  if (!m->winHas || pointExcess(*m, sp) < pointExcess(*m, m->winBest)) { m->winBest = sp; m->winHas = true; }
  if (m->winStartUs == 0) m->winStartUs = t4;
  bool commit = (m->n == 0 && m->samples >= 5) || (t4 - m->winStartUs >= (int64_t)SYNC_WINDOW_MS * 1000);
  if (commit && m->winHas) {
    m->pts[m->head] = m->winBest;
    m->head = (m->head + 1) % SYNC_POINTS;
    if (m->n < SYNC_POINTS) m->n++;
    m->winHas = false;
    m->winStartUs = t4;
    modelFit(*m);
  }
}

static SyncModel* currentModel() {
  if (!g_peers[ROLE_START].known) return nullptr;
  return modelFind(g_peers[ROLE_START].bootId);
}

// 0 = aucune, 1 = bonne, 2 = degradee (ancienne ou imprecise)
static int syncStateOf(const SyncModel* m, int64_t now) {
  if (!m || !m->valid) return 0;
  const int64_t fresh = (LORA_ME && 3LL * LORA_SYNC_MS * 1000 > 10000000LL) ? 3LL * LORA_SYNC_MS * 1000 : 10000000LL;
  if (modelUncertAt(*m, now) <= SYNC_OK_UNCERT_US && now - m->lastPointUs < fresh) return 1;
  return 2;
}

// Une requete par chemin, horodatee juste avant son propre envoi
static void sendSyncReqPath(uint8_t path) {
  PktSync q;
  memset(&q, 0, sizeof q);
  q.h = makeHeader(PT_SYNC_REQ);
  q.reqId = ++g_syncReqId;
  q.t1 = nowUs();
  if (path == 0) sendDirect(ROLE_START, &q, sizeof q);
  else sendViaRelay(ROLE_START, &q, sizeof q);
}

// Requete directe, puis requete via le relais une demi-periode plus tard : les deux
// emissions ne se genent pas (sinon la seconde attendrait la fin de la premiere et
// l'aller serait plus long que le retour).
static void syncScheduler(int64_t now) {
  if (now - g_lastSyncReqUs >= (int64_t)SYNC_PERIOD_MS * 1000) {
    g_lastSyncReqUs = now;
    bool rel = relayUsable(ROLE_START, now);
    if (!rel || directOk(ROLE_START, now)) sendSyncReqPath(0);
    g_syncRelayDueUs = rel ? now + (int64_t)SYNC_PERIOD_MS * 500 : 0;
  }
  if (g_syncRelayDueUs && now >= g_syncRelayDueUs) {
    g_syncRelayDueUs = 0;
    sendSyncReqPath(1);
  }
  // LoRa : une requete toutes les LORA_SYNC_MS ; t1 est pose au moment de l'emission
  if (g_loraOk && now - g_lastLoraSyncUs >= (int64_t)LORA_SYNC_MS * 1000) {
    g_lastLoraSyncUs = now;
    PktSync q;
    memset(&q, 0, sizeof q);
    q.h = makeHeader(PT_SYNC_REQ);
    q.reqId = ++g_syncReqId;
    loraSendTo(ROLE_START, &q, sizeof q, 1, 1);
  }
}

// ============================================================================
// 10. HEARTBEAT, COMMANDES, ALIGNEMENT
// ============================================================================
static uint8_t outboxCount() {
  uint8_t n = 0;
  for (int i = 0; i < OUTBOX_SIZE; i++) if (g_out[i].used) n++;
  return n;
}

// alsoLora : copie LoRa (meme numero), toutes les LORA_HB_MS
static void sendHeartbeat(int64_t now, bool alsoLora) {
  PktHeartbeat p;
  memset(&p, 0, sizeof p);
  p.h = makeHeader(PT_HEARTBEAT);
  uint8_t fl = 0;
  {
    Lock l;
    p.battMv = g_battMv;
    p.uptimeS = (uint32_t)(now / 1000000);
    p.sessionId = g_sessionId;
    p.syncUncertUs = UINT32_MAX;
    strlcpy(p.fw, FW_VERSION, sizeof p.fw);
    if (g_maintenance) fl |= HBF_MAINT;
    if (g_align) fl |= HBF_ALIGN;
    if (IS_FINISH) {
      if (g_bench) fl |= HBF_BENCH;
      SyncModel* m = currentModel();
      if (syncStateOf(m, now) == 1) fl |= HBF_SYNC_OK;
      if (m && m->valid) p.syncUncertUs = modelUncertAt(*m, now);
      if (g_beamEstablished && !g_beamBroken) fl |= HBF_BEAM_OK;
      p.beamQuality = g_beamQuality;
      p.relayBattStart = peerOk(ROLE_START, now) ? g_peers[ROLE_START].battMv : 0;
      p.relayBattEmitter = peerOk(ROLE_EMITTER, now) ? g_peers[ROLE_EMITTER].battMv : 0;
    }
    if (IS_START) p.pending = outboxCount();
    if (IS_EMITTER) p.irPower = g_irPower;
    for (int r = 1; r <= 4; r++) {
      if (r == CHRONO_ROLE) continue;
      hbStats(g_peers[r], now, p.rxDir[r], p.rxRel[r], p.rxAny[r]);
      p.rssi[r] = directOk((uint8_t)r, now) ? g_peers[r].rssi : 0;
      p.rxLora[r] = loraPct(g_peers[r], now);
      int lr = g_peers[r].loraRssi;
      p.lrssi[r] = loraRecent((uint8_t)r, now) ? (int8_t)(lr < -128 ? -128 : lr) : 0;
    }
    if (g_maintenance && WiFi.status() == WL_CONNECTED) p.ip = (uint32_t)WiFi.localIP();
  }
  p.h.flags = fl;
  sendRaw(BCAST_MAC, &p, sizeof p);   // en diffusion : tous les boitiers apprennent les adresses
  if (alsoLora) loraSendTo(0, &p, sizeof p, 2, 0);
}

static void setAlign(bool on, bool propagate) {
  if (IS_START || IS_RELAY) return;
  int64_t now = nowUs();
  g_align = on;
  g_alignUntilUs = on ? now + (int64_t)ALIGN_TIMEOUT_MIN * 60 * 1000000LL : 0;
  g_alignLocalChangeUs = now;
  g_alignNextBeepUs = now + 600000;
  beep(on ? PAT_LINK_OK : PAT_CANCELLED);
  if (propagate) {
    PktCmd c;
    memset(&c, 0, sizeof c);
    c.h = makeHeader(PT_CMD);
    uint8_t target = IS_FINISH ? ROLE_EMITTER : ROLE_FINISH;
    c.targetRole = target;
    c.cmd = on ? CMD_ALIGN_ON : CMD_ALIGN_OFF;
    sendToRole(target, &c, sizeof c);
  }
  LOGF("[align] %s\n", on ? "ON" : "OFF");
}

static void sendCmd(uint8_t target, uint8_t cmd, uint32_t arg) {
  PktCmd c;
  memset(&c, 0, sizeof c);
  c.h = makeHeader(PT_CMD);
  c.targetRole = target;
  c.cmd = cmd;
  c.arg = arg;
  for (int i = 0; i < 3; i++) sendToRole(target, &c, sizeof c);
  if (target == ROLE_START || target == ROLE_FINISH) loraSendTo(target, &c, sizeof c, 0, 0);
}

// Le boitier redemarre en maintenance (rejoint le Wi-Fi de l'arrivee pour l'OTA)
static void requestMaintenance() {
  if (IS_FINISH) return;
  LOGF("[maint] redemarrage en maintenance\n");
  flashMark(true);
  g_prefs.putBool("maint", true);
  flashMark(false);
  beep(PAT_MAINT);
  delay(700);
  ESP.restart();
}

// Bips d'alignement : de plus en plus rapides quand le faisceau est bien recu
static void alignService(int64_t now) {
  if (!g_align || IS_START || IS_RELAY) return;
  if (g_alignUntilUs && now > g_alignUntilUs) { Lock l; setAlign(false, true); return; }
  if (now < g_alignNextBeepUs) return;
  int q = IS_FINISH ? (int)g_beamQuality : ((now - g_remoteBeamQUs < 1500000) ? (int)g_remoteBeamQ : -1);
  int intervalMs;
  if (q < 0)        { beep(PAT_ALIGN_LO);   intervalMs = 2000; }
  else if (q < 5)   { beep(PAT_ALIGN_LO);   intervalMs = 1200; }
  else if (q >= 95) { beep(PAT_ALIGN_FULL); intervalMs = 260; }
  else              { beep(PAT_ALIGN_HI);   intervalMs = 700 - q * 6; }
  g_alignNextBeepUs = now + (int64_t)intervalMs * 1000;
}

// ============================================================================
// 11. BOUTON UNIQUE
//   court       : bips d'etat (lien, synchro ou faisceau, batterie)
//   long (2 s)  : depart = annule le dernier depart ; arrivee/emetteur = alignement
//   a l'allumage: depart/emetteur = maintenance OTA
// ============================================================================
static bool btnRawPressed() { return analogReadMilliVolts(PIN_BUTTON) < BTN_PRESSED_MV; }

static void beepStatus() {
  int64_t now = nowUs();
  bool a, b;
  {
    Lock l;
    if (IS_FINISH) {
      a = peerOk(ROLE_START, now);
      b = (syncStateOf(currentModel(), now) == 1) && g_beamEstablished;
    } else if (IS_RELAY) {
      a = directOk(ROLE_START, now);    // 1er bip : le relais entend le depart
      b = directOk(ROLE_FINISH, now);   // 2e bip : le relais entend l'arrivee
    } else {
      a = peerOk(ROLE_FINISH, now);
      b = IS_START ? g_syncOkRemote : (g_remoteBeamQ >= 80 && now - g_remoteBeamQUs < 3000000);
    }
  }
  BuzItem it;
  memset(&it, 0, sizeof it);
  int k = 0;
  auto add = [&](uint16_t f, uint16_t ms) {
    if (k < BUZ_MAX_NOTES - 1) { it.n[k].freq = f; it.n[k].ms = ms; k++; }
  };
  add(a ? BUZ_HI : BUZ_LO, a ? 150 : 450);
  add(0, 250);
  add(b ? BUZ_HI : BUZ_LO, b ? 150 : 450);
  add(0, 350);
  int lv = battLevel4();
  for (int i = 0; i < lv; i++) { add(BUZ_HI, 60); add(0, 140); }
  xQueueSend(g_buzQ, &it, 0);
}

static void startCancelLast();   // defini avec la logique du depart

static void onShortPress() { beepStatus(); }

static void onLongPress() {
  if (g_maintenance) { beep(PAT_CANCELLED); delay(500); ESP.restart(); }
  Lock l;
  if (IS_START) startCancelLast();
  else if (IS_RELAY) beepStatus();   // le verrou est recursif
  else setAlign(!g_align, true);
}

static void buttonService() {
  static uint32_t lastPoll = 0;
  uint32_t ms = millis();
  if (ms - lastPoll < 10) return;
  lastPoll = ms;
  bool raw = btnRawPressed();
  if (raw != g_btnRaw) { g_btnRaw = raw; g_btnChangeMs = ms; }
  if (ms - g_btnChangeMs >= BTN_DEBOUNCE_MS && raw != g_btnStable) {
    g_btnStable = raw;
    if (raw) { g_btnDownMs = ms; g_btnLongDone = false; }
    else {
      if (!g_btnLongDone && !g_btnIgnore) onShortPress();
      g_btnIgnore = false;
    }
  }
  if (g_btnStable && !g_btnLongDone && !g_btnIgnore && ms - g_btnDownMs >= BTN_LONG_MS) {
    g_btnLongDone = true;
    onLongPress();
  }
}

static bool bootHoldDetected() {
  if (!btnRawPressed()) return false;
  uint32_t t0 = millis();
  while (millis() - t0 < BTN_BOOT_HOLD_MS) {
    if (!btnRawPressed()) return false;
    delay(20);
  }
  g_btnIgnore = true;   // le relachement ne doit pas compter comme un appui court
  g_btnStable = true;
  g_btnRaw = true;
  return true;
}

// ============================================================================
// 12. DEPART : coureurs
// ============================================================================
static int riderIndexById(uint16_t id) {
  if (!id) return -1;
  for (uint16_t i = 0; i < g_riderCount; i++) if (g_riders[i].id == id) return i;
  return -1;
}

static void ridersChanged() {
  g_ridersRev++;
  storeSaveRiders();
}

static void loadRiders() {
  g_riderCount = 0;
  g_nextIdx = -1;
  File f = LittleFS.open(RIDERS_FILE, FILE_READ);
  if (!f) return;
  char buf[160];
  char* fld[6];
  bool first = true;
  while (f.available()) {
    size_t n = f.readBytesUntil('\n', buf, sizeof(buf) - 1);
    buf[n] = 0;
    if (n && buf[n - 1] == '\r') buf[--n] = 0;
    if (!n) continue;
    if (first) {
      first = false;
      if (buf[0] == 'V') {
        int c = splitFields(buf, ';', fld, 6);
        if (c >= 3) { g_nextIdx = (int16_t)atoi(fld[1]); g_riderNextId = (uint16_t)atoi(fld[2]); }
        continue;
      }
    }
    if (g_riderCount >= MAX_RIDERS) break;
    int c = splitFields(buf, '\t', fld, 6);
    if (c < 4) continue;
    Rider& r = g_riders[g_riderCount];
    memset(&r, 0, sizeof r);
    r.id = (uint16_t)atoi(fld[0]);
    r.bib = (uint16_t)atoi(fld[1]);
    copyClean(r.name, NAME_LEN, fld[2]);
    copyClean(r.cat, CAT_LEN, fld[3]);
    if (!r.id) continue;
    if (r.id >= g_riderNextId) g_riderNextId = r.id + 1;
    g_riderCount++;
  }
  f.close();
  if (g_nextIdx >= (int)g_riderCount || g_nextIdx < 0) g_nextIdx = g_riderCount ? 0 : -1;
  LOGF("[depart] %u coureurs charges\n", g_riderCount);
}

// Appelee par la tache de stockage : copie sous verrou, ecriture hors verrou
static void saveRidersNow() {
  static Rider snap[MAX_RIDERS];
  uint16_t n;
  int16_t nx;
  uint16_t nid;
  {
    Lock l;
    n = g_riderCount;
    memcpy(snap, g_riders, n * sizeof(Rider));
    nx = g_nextIdx;
    nid = g_riderNextId;
    g_ridersSavePending = false;
  }
  flashMark(true);
  File f = LittleFS.open("/riders.tmp", FILE_WRITE);
  if (f) {
    f.printf("V;%d;%u\n", nx, nid);
    for (uint16_t i = 0; i < n; i++) f.printf("%u\t%u\t%s\t%s\n", snap[i].id, snap[i].bib, snap[i].name, snap[i].cat);
    f.close();
    LittleFS.remove(RIDERS_FILE);
    LittleFS.rename("/riders.tmp", RIDERS_FILE);
  }
  flashMark(false);
}

static bool riderAdd(uint16_t bib, const char* name, const char* cat) {
  if (g_riderCount >= MAX_RIDERS) return false;
  Rider& r = g_riders[g_riderCount++];
  memset(&r, 0, sizeof r);
  r.id = g_riderNextId++;
  r.bib = bib;
  copyClean(r.name, NAME_LEN, name);
  copyClean(r.cat, CAT_LEN, cat);
  if (g_nextIdx < 0) g_nextIdx = 0;
  return true;
}

static void riderDelete(int i) {
  if (i < 0 || i >= (int)g_riderCount) return;
  for (int k = i; k < (int)g_riderCount - 1; k++) g_riders[k] = g_riders[k + 1];
  g_riderCount--;
  if (g_riderCount == 0) g_nextIdx = -1;
  else {
    if (i < g_nextIdx) g_nextIdx--;
    if (g_nextIdx >= (int)g_riderCount) g_nextIdx = 0;
  }
}

static void riderMove(int i, int d) {
  int j = i + d;
  if (i < 0 || j < 0 || i >= (int)g_riderCount || j >= (int)g_riderCount) return;
  uint16_t nextId = (g_nextIdx >= 0) ? g_riders[g_nextIdx].id : 0;
  Rider t = g_riders[i];
  g_riders[i] = g_riders[j];
  g_riders[j] = t;
  if (nextId) g_nextIdx = (int16_t)riderIndexById(nextId);
}

// ============================================================================
// 13. DEPART : departs, file d'envoi fiable, journal
// ============================================================================
static StartRec* startRecFind(uint32_t sb, uint32_t sq) {
  for (uint16_t k = 0; k < g_startCount; k++) {
    StartRec& r = g_starts[(g_startHead + START_HIST - 1 - k) % START_HIST];
    if (r.startBoot == sb && r.runSeq == sq) return &r;
  }
  return nullptr;
}

static StartRec& startRecPush() {
  StartRec& r = g_starts[g_startHead];
  memset(&r, 0, sizeof r);
  g_startHead = (g_startHead + 1) % START_HIST;
  if (g_startCount < START_HIST) g_startCount++;
  return r;
}

static void outboxService(int64_t now) {
  static const uint16_t delaysMs[] = {20, 50, 100, 200, 500};
  for (int i = 0; i < OUTBOX_SIZE; i++) {
    OutItem& o = g_out[i];
    if (!o.used || now < o.nextTx) continue;
    // Chaque tentative porte un nouveau numero : l'arrivee ne la prend pas pour un doublon
    PktHeader hh;
    memcpy(&hh, o.data, sizeof hh);
    hh.seq = __atomic_add_fetch(&g_txSeq, 1, __ATOMIC_RELAXED);
    memcpy(o.data, &hh, sizeof hh);
    sendToRole(ROLE_FINISH, o.data, o.len);
    // LoRa : premier essai puis toutes les LORA_RETRY_MS (un paquet y occupe ~150 ms)
    if (g_loraOk && now >= o.loraNext) {
      loraSendTo(ROLE_FINISH, o.data, o.len, 0, 0);
      o.loraNext = now + (int64_t)LORA_RETRY_MS * 1000;
    }
    uint16_t d = o.tries < 5 ? delaysMs[o.tries] : 1000;
    o.tries++;
    o.nextTx = now + (int64_t)d * 1000;
    if (o.type == PT_START_EVT && !o.warned && now - o.created > 2000000) {
      o.warned = true;
      beep(PAT_NOACK);   // l'arrivee n'a pas confirme : le depart reste en file
    }
  }
}

// Ajoute un message a renvoyer jusqu'a acquittement, et l'envoie tout de suite
static void outboxAdd(uint8_t type, uint32_t sb, uint32_t sq, const void* data, uint8_t len) {
  int slot = -1, old = 0;
  int64_t oldest = INT64_MAX;
  for (int i = 0; i < OUTBOX_SIZE; i++) {
    if (!g_out[i].used) { slot = i; break; }
    if (g_out[i].created < oldest) { oldest = g_out[i].created; old = i; }
  }
  if (slot < 0) slot = old;
  OutItem& o = g_out[slot];
  memset(&o, 0, sizeof o);
  o.used = true;
  o.type = type;
  o.startBoot = sb;
  o.runSeq = sq;
  o.len = len;
  memcpy(o.data, data, len);
  o.created = nowUs();
  o.nextTx = 0;
  outboxService(o.created);
}

static void logStartCreate(const StartRec& r) {
  storeAppendf(STARTS_LOG, "S;%lu;%lu;%lu;%lld;%u;%u;%lld;%u;%s;%s", (unsigned long)g_sessionId,
               (unsigned long)r.startBoot, (unsigned long)r.runSeq, (long long)r.tStart, r.riderId, r.bib,
               (long long)r.wallMs, r.evtFlags, r.name, r.cat);
}

static void logStartState(const StartRec& r) {
  storeAppendf(STARTS_LOG, "A;%lu;%lu;%u", (unsigned long)r.startBoot, (unsigned long)r.runSeq, r.state);
}

static void buildStartPacket(const StartRec& r, PktStartEvt& p) {
  memset(&p, 0, sizeof p);
  p.h = makeHeader(PT_START_EVT);
  p.startBoot = r.startBoot;
  p.runSeq = r.runSeq;
  p.tStart = r.tStart;
  p.wallMs = r.wallMs;
  p.riderId = r.riderId;
  p.bib = r.bib;
  p.evtFlags = r.evtFlags;
  memcpy(p.name, r.name, NAME_LEN);
  memcpy(p.cat, r.cat, CAT_LEN);
}

// Appelee par la tache coeur juste apres l'interruption du capteur Hall
static void startCreate(int64_t t) {
  uint8_t ef = 0;
  if (g_benchRemote) ef |= EVF_BENCH;
  if (flashOverlap(t)) ef |= EVF_FLASH;
  Rider* rd = nullptr;
  if (!(ef & EVF_BENCH) && g_nextIdx >= 0 && g_nextIdx < (int)g_riderCount) rd = &g_riders[g_nextIdx];

  StartRec& rec = startRecPush();
  rec.startBoot = g_bootId;
  rec.runSeq = ++g_runSeq;
  rec.tStart = t;
  rec.wallMs = wallFromLocalUs(t);
  rec.durationUs = -1;
  rec.state = SR_PENDING;
  rec.evtFlags = ef;
  if (rd) {
    rec.riderId = rd->id;
    rec.bib = rd->bib;
    memcpy(rec.name, rd->name, NAME_LEN);
    memcpy(rec.cat, rd->cat, CAT_LEN);
  }

  // (1) radio d'abord
  PktStartEvt p;
  buildStartPacket(rec, p);
  outboxAdd(PT_START_EVT, rec.startBoot, rec.runSeq, &p, sizeof p);

  // (2) etat et journal
  if (rd) {
    rd->runs++;
    rec.counted = true;
    g_nextIdx = (int16_t)((g_nextIdx + 1) % g_riderCount);
    ridersChanged();
  }
  g_startsRev++;
  logStartCreate(rec);

  // (3) bip en dernier, depuis la tache buzzer (priorite plus basse que la tache coeur)
  beep(PAT_START);
  LOGF("[depart] #%lu t=%lld us %s%s\n", (unsigned long)rec.runSeq, (long long)t,
       rec.name[0] ? rec.name : "(anonyme)", (ef & EVF_BENCH) ? " [banc]" : "");
}

static bool startCancel(uint32_t sb, uint32_t sq) {
  StartRec* rec = startRecFind(sb, sq);
  if (!rec || rec->state >= SR_CANCEL_PENDING) return false;
  for (int i = 0; i < OUTBOX_SIZE; i++)
    if (g_out[i].used && g_out[i].type == PT_START_EVT && g_out[i].startBoot == sb && g_out[i].runSeq == sq) g_out[i].used = false;
  rec->state = SR_CANCEL_PENDING;
  if (rec->counted) {
    int i = riderIndexById(rec->riderId);
    if (i >= 0) {
      if (g_riders[i].runs) g_riders[i].runs--;
      g_nextIdx = (int16_t)i;   // faux depart : le meme pilote repart
    }
    rec->counted = false;
    ridersChanged();
  }
  PktCancel c;
  memset(&c, 0, sizeof c);
  c.h = makeHeader(PT_CANCEL);
  c.startBoot = sb;
  c.runSeq = sq;
  outboxAdd(PT_CANCEL, sb, sq, &c, sizeof c);
  logStartState(*rec);
  g_startsRev++;
  beep(PAT_CANCELLED);
  LOGF("[depart] annulation #%lu\n", (unsigned long)sq);
  return true;
}

static void startCancelLast() {
  int64_t now = nowUs();
  for (uint16_t k = 0; k < g_startCount; k++) {
    StartRec& r = g_starts[(g_startHead + START_HIST - 1 - k) % START_HIST];
    if (r.state >= SR_CANCEL_PENDING) continue;
    if (r.startBoot == g_bootId && now - r.tStart < (int64_t)CANCEL_WINDOW_S * 1000000LL) {
      startCancel(r.startBoot, r.runSeq);
      return;
    }
    break;
  }
  beep(PAT_ERROR);
}

// Mode reglage : la sortie du DRV5033 est lue toutes les 2 ms ; bip continu tant qu'elle est basse
static void setHallTune(bool on) {
  g_hallTune = on;
  g_hallArmed = false;          // l'interruption ne cree plus de depart
  g_hallHighSinceUs = 0;        // a la sortie du mode : tige au repos 500 ms avant de rearmer
  g_tuneWasLow = false;
  g_tuneCount = 0;
  g_hallTuneUntilUs = on ? nowUs() + 10LL * 60 * 1000000 : 0;
  g_startsRev++;
  beep(on ? PAT_LINK_OK : PAT_CANCELLED);
  LOGF("[depart] mode reglage du capteur %s\n", on ? "ON" : "OFF");
}

static void startTuneService(int64_t now) {
  if (!g_hallTune) return;
  if (now > g_hallTuneUntilUs) { setHallTune(false); return; }
  bool low = digitalRead(PIN_HALL) == LOW;
  if (low && !g_tuneWasLow) g_tuneCount++;
  g_tuneWasLow = low;
  if (low && now - g_tuneBeepUs >= 110000) { g_tuneBeepUs = now; beep(PAT_TUNE); }
}

static void startHallService(int64_t now) {
  if (g_hallTune) return;
  bool high = digitalRead(PIN_HALL) == HIGH;
  if (!high) { g_hallHighSinceUs = 0; return; }
  if (!g_hallHighSinceUs) g_hallHighSinceUs = now;
  if (!g_hallArmed && now - g_hallHighSinceUs >= (int64_t)HALL_REST_STABLE_MS * 1000 &&
      now - g_lastTrigUs >= (int64_t)HALL_LOCKOUT_MS * 1000) {
    g_hallArmed = true;
  }
}

// Nouvelle session annoncee par l'arrivee : on archive le journal et on remet les manches a zero
static void startOnSession(uint32_t sid) {
  LOGF("[depart] session %lu -> %lu\n", (unsigned long)g_sessionId, (unsigned long)sid);
  g_autoArchivedSid = g_sessionId;
  char dst[40];
  snprintf(dst, sizeof dst, "/arch/departs_%03lu.log", (unsigned long)g_sessionId);
  storeOp(SO_RENAME, dst, STARTS_LOG, nullptr);
  g_sessionId = sid;
  prefsPutU("sess", sid);
  for (uint16_t i = 0; i < g_riderCount; i++) g_riders[i].runs = 0;
  g_startCount = 0;
  g_startHead = 0;
  memset(g_out, 0, sizeof g_out);
  g_nextIdx = g_riderCount ? 0 : -1;
  ridersChanged();
  g_startsRev++;
}

// Effacement demande par l'arrivee : journal supprime (pas d'archive), manches remises a zero.
// La liste des coureurs est conservee.
static void startWipe(uint32_t sid, bool all) {
  if (sid == g_lastWipeSid) return;   // commande recue en triple
  g_lastWipeSid = sid;
  LOGF("[depart] effacement %s, session %lu\n", all ? "complet" : "de la session", (unsigned long)sid);
  storeOp(SO_WIPE, STARTS_LOG, all ? "all" : "", nullptr);
  if (!all && g_autoArchivedSid && g_sessionId == sid) {
    // Le heartbeat est arrive avant la commande : l'ancienne session vient d'etre archivee, on la supprime aussi
    char p[40];
    snprintf(p, sizeof p, "/arch/departs_%03lu.log", (unsigned long)g_autoArchivedSid);
    storeOp(SO_WIPE, p, "", nullptr);
  }
  g_autoArchivedSid = 0;
  if (g_sessionId != sid) { g_sessionId = sid; prefsPutU("sess", sid); }
  for (uint16_t i = 0; i < g_riderCount; i++) g_riders[i].runs = 0;
  g_startCount = 0;
  g_startHead = 0;
  memset(g_out, 0, sizeof g_out);
  g_nextIdx = g_riderCount ? 0 : -1;
  ridersChanged();
  g_startsRev++;
  beep(PAT_CANCELLED);
}

// Relit le journal de la session au demarrage (compteurs de manches, departs en attente)
static void startLoadLog() {
  File f = LittleFS.open(STARTS_LOG, FILE_READ);
  if (!f) return;
  char buf[256];
  char* fld[12];
  while (f.available()) {
    size_t n = f.readBytesUntil('\n', buf, sizeof(buf) - 1);
    buf[n] = 0;
    if (n && buf[n - 1] == '\r') buf[--n] = 0;
    if (!n) continue;
    int c = splitFields(buf, ';', fld, 12);
    if (buf[0] == 'S' && c >= 11) {
      if (toUL(fld[1]) != g_sessionId) continue;
      StartRec& r = startRecPush();
      r.startBoot = toUL(fld[2]);
      r.runSeq = toUL(fld[3]);
      r.tStart = toLL(fld[4]);
      r.riderId = (uint16_t)toUL(fld[5]);
      r.bib = (uint16_t)toUL(fld[6]);
      r.wallMs = toLL(fld[7]);
      r.evtFlags = (uint8_t)toUL(fld[8]);
      copyClean(r.name, NAME_LEN, fld[9]);
      copyClean(r.cat, CAT_LEN, fld[10]);
      r.state = SR_PENDING;
      r.durationUs = -1;
      int i = riderIndexById(r.riderId);
      if (i >= 0) { g_riders[i].runs++; r.counted = true; }
    } else if (buf[0] == 'A' && c >= 4) {
      StartRec* r = startRecFind(toUL(fld[1]), toUL(fld[2]));
      if (!r) continue;
      r->state = (uint8_t)toUL(fld[3]);
      if (r->state >= SR_CANCEL_PENDING && r->counted) {
        int i = riderIndexById(r->riderId);
        if (i >= 0 && g_riders[i].runs) g_riders[i].runs--;
        r->counted = false;
      }
    } else if (buf[0] == 'N' && c >= 5) {
      StartRec* r = startRecFind(toUL(fld[1]), toUL(fld[2]));
      if (!r) continue;
      r->durationUs = toLL(fld[3]);
      r->finStatus = (uint8_t)toUL(fld[4]);
    }
  }
  f.close();
  LOGF("[depart] journal relu : %u departs dans la session %lu\n", g_startCount, (unsigned long)g_sessionId);
}

// Apres l'init radio : renvoie ce qui n'avait pas ete acquitte avant le redemarrage
static void startRestorePending() {
  Lock l;
  for (uint16_t k = 0; k < g_startCount; k++) {
    StartRec& r = g_starts[(g_startHead + START_HIST - g_startCount + k) % START_HIST];
    if (r.state == SR_PENDING) {
      PktStartEvt p;
      buildStartPacket(r, p);
      outboxAdd(PT_START_EVT, r.startBoot, r.runSeq, &p, sizeof p);
    } else if (r.state == SR_CANCEL_PENDING) {
      PktCancel c;
      memset(&c, 0, sizeof c);
      c.h = makeHeader(PT_CANCEL);
      c.startBoot = r.startBoot;
      c.runSeq = r.runSeq;
      outboxAdd(PT_CANCEL, r.startBoot, r.runSeq, &c, sizeof c);
    }
  }
}

static void startOnPacket(const PktHeader& h, const InMsg& m) {
  if (h.role != ROLE_FINISH) return;
  switch (h.type) {
    case PT_HEARTBEAT: {
      const PeerInfo& p = g_peers[ROLE_FINISH];
      g_benchRemote = (p.hbFlags & HBF_BENCH) != 0;
      g_syncOkRemote = (p.hbFlags & HBF_SYNC_OK) != 0;
      if (p.sessionId && p.sessionId != g_sessionId) startOnSession(p.sessionId);
      break;
    }
    case PT_SYNC_REQ: {
      if (m.len < sizeof(PktSync)) return;
      PktSync q;
      memcpy(&q, m.data, sizeof q);
      PktSync r;
      memset(&r, 0, sizeof r);
      r.h = makeHeader(PT_SYNC_RESP);
      r.reqId = q.reqId;
      r.t1 = q.t1;
      r.t2 = m.tRx;          // reception (callback radio)
      r.resReq = q.resReq;
      r.t3 = nowUs();        // emission, au plus pres de l'envoi
      // Reponse par le meme chemin que la requete : aller et retour symetriques
      if (m.lora) loraSendTo(ROLE_FINISH, &r, sizeof r, 1, 2);   // t3 pose au moment de l'emission LoRa
      else if (h.hop == 0) sendDirect(ROLE_FINISH, &r, sizeof r);
      else sendViaRelay(ROLE_FINISH, &r, sizeof r);
      break;
    }
    case PT_ACK: {
      if (m.len < sizeof(PktAck)) return;
      PktAck a;
      memcpy(&a, m.data, sizeof a);
      uint32_t sb = a.startBoot, sq = a.runSeq;
      uint8_t at = a.ackType;
      for (int i = 0; i < OUTBOX_SIZE; i++)
        if (g_out[i].used && g_out[i].type == at && g_out[i].startBoot == sb && g_out[i].runSeq == sq) g_out[i].used = false;
      StartRec* rec = startRecFind(sb, sq);
      if (rec) {
        if (at == PT_START_EVT && rec->state == SR_PENDING) { rec->state = SR_ACKED; logStartState(*rec); }
        else if (at == PT_CANCEL && rec->state == SR_CANCEL_PENDING) { rec->state = SR_CANCELLED; logStartState(*rec); }
        g_startsRev++;
      }
      break;
    }
    case PT_FINISH_NOTE: {
      if (m.len < sizeof(PktFinishNote)) return;
      PktFinishNote n;
      memcpy(&n, m.data, sizeof n);
      StartRec* rec = startRecFind(n.startBoot, n.runSeq);
      if (!rec) return;
      int64_t d = n.durationUs;
      uint8_t s = n.status;
      if (rec->durationUs == d && rec->finStatus == s) return;   // doublon
      bool newArrival = (s == RUN_FINISHED && rec->finStatus != RUN_FINISHED);
      rec->durationUs = d;
      rec->finStatus = s;
      storeAppendf(STARTS_LOG, "N;%lu;%lu;%lld;%u", (unsigned long)rec->startBoot, (unsigned long)rec->runSeq, (long long)d, s);
      g_startsRev++;
      if (newArrival) beep(PAT_ARRIVED);
      break;
    }
    default:
      break;
  }
}

// ============================================================================
// 14. ARRIVEE : manches, passages, classement
// ============================================================================
static void logRunCreate(const Run& r) {
  storeAppendf(FINISH_LOG, "R;%u;%lu;%lu;%lld;%u;%u;%lld;%u;%s;%s", r.idx, (unsigned long)r.startBoot,
               (unsigned long)r.runSeq, (long long)r.tStartRaw, r.riderId, r.bib, (long long)r.wallStartMs,
               r.flags, r.name, r.cat);
}

static void logRunState(const Run& r) {
  storeAppendf(FINISH_LOG, "F;%u;%u;%u;%lu;%lld;%lu;%lld;%lld;%lu", r.idx, r.status, r.flags,
               (unsigned long)r.finishBoot, (long long)r.tStartF, (unsigned long)r.tStartFBoot,
               (long long)r.tFinishF, (long long)r.durationUs, (unsigned long)r.uncertUs);
}

static void logOrphan(const Orphan& o) {
  storeAppendf(FINISH_LOG, "O;%u;%lu;%lld;%u;%u;%lld", o.id, (unsigned long)o.finishBoot, (long long)o.tF,
               o.flags, o.used ? 1 : 0, (long long)o.wallMs);
}

// Depart converti dans l'horloge de l'arrivee (modele frais si disponible)
static bool runStartF(Run& r, int64_t& out) {
  SyncModel* m = modelFind(r.startBoot);
  if (m && m->valid) {
    r.tStartF = modelStartToFinish(*m, r.tStartRaw);
    r.tStartFBoot = g_bootId;
    out = r.tStartF;
    return true;
  }
  if (r.tStartFBoot == g_bootId) { out = r.tStartF; return true; }
  return false;
}

static int64_t runWall(const Run& r) {
  if (r.wallStartMs > 0) return r.wallStartMs;
  if (g_wallValid && r.tStartFBoot == g_bootId) return g_wallOffsetMs + r.tStartF / 1000;
  return 0;
}

static void runComputeDuration(Run& r) {
  if (r.finishBoot != g_bootId) return;   // arrivee prise avant un redemarrage : valeur figee
  r.flags &= ~(RUNF_NO_SYNC | RUNF_SYNC_DEGRADED);
  int64_t ts;
  if (runStartF(r, ts)) {
    r.durationUs = (r.tFinishF - (int64_t)CAL_FINISH_US) - (ts - (int64_t)CAL_START_US);
    SyncModel* m = modelFind(r.startBoot);
    r.uncertUs = m ? modelUncertAt(*m, ts) : UINT32_MAX;
    if (r.uncertUs > SYNC_OK_UNCERT_US) r.flags |= RUNF_SYNC_DEGRADED;
  } else {
    r.durationUs = -1;
    r.uncertUs = UINT32_MAX;
    r.flags |= RUNF_NO_SYNC;
  }
}

static void recomputeBest() {
  g_bestRunUs = -1;
  for (uint16_t i = 0; i < g_runCount; i++) {
    const Run& r = g_runs[i];
    if (r.status == RUN_FINISHED && r.durationUs > 0 && (g_bestRunUs < 0 || r.durationUs < g_bestRunUs)) g_bestRunUs = r.durationUs;
  }
}

static uint16_t runNumber(const Run& r) {
  if (!r.riderId) return 1;
  uint16_t k = 0;
  for (uint16_t i = 0; i <= r.idx && i < g_runCount; i++)
    if (g_runs[i].riderId == r.riderId && g_runs[i].status != RUN_CANCELLED) k++;
  return k;
}

static bool riderCumulTotal(uint16_t riderId, int64_t& total) {
  uint8_t k = 0;
  total = 0;
  for (uint16_t i = 0; i < g_runCount && k < g_setRuns; i++) {
    const Run& r = g_runs[i];
    if (r.riderId != riderId || r.status == RUN_CANCELLED) continue;
    if (r.status != RUN_FINISHED || r.durationUs < 0) return false;
    total += r.durationUs;
    k++;
  }
  return k == g_setRuns;
}

static bool isCumulLeader(uint16_t riderId) {
  int64_t mine;
  if (!riderId || !riderCumulTotal(riderId, mine)) return false;
  for (uint16_t i = 0; i < g_runCount; i++) {
    uint16_t id = g_runs[i].riderId;
    if (!id || id == riderId) continue;
    bool seen = false;
    for (uint16_t j = 0; j < i; j++) if (g_runs[j].riderId == id) { seen = true; break; }
    if (seen) continue;
    int64_t t;
    if (riderCumulTotal(id, t) && t < mine) return false;
  }
  return true;
}

static void sendFinishNote(const Run& r) {
  PktFinishNote n;
  memset(&n, 0, sizeof n);
  n.h = makeHeader(PT_FINISH_NOTE);
  n.startBoot = r.startBoot;
  n.runSeq = r.runSeq;
  n.durationUs = r.durationUs;
  n.status = r.status;
  n.flags = r.flags;
  sendToRole(ROLE_START, &n, sizeof n);
  sendToRole(ROLE_START, &n, sizeof n);
  loraSendTo(ROLE_START, &n, sizeof n, 0, 0);
}

static void sendAck(uint8_t type, uint32_t sb, uint32_t sq) {
  PktAck a;
  memset(&a, 0, sizeof a);
  a.h = makeHeader(PT_ACK);
  a.ackType = type;
  a.startBoot = sb;
  a.runSeq = sq;
  sendToRole(ROLE_START, &a, sizeof a);
  // LoRa si la demande est arrivee en LoRa, ou si le depart n'est pas joignable en ESP-NOW
  int64_t now = nowUs();
  if (g_rxViaLora || !(directOk(ROLE_START, now) || viaRelayRecent(ROLE_START, now))) loraSendTo(ROLE_START, &a, sizeof a, 0, 0);
}

// Termine une manche : ordre journal -> radio -> bip
static void finishRun(Run& r, int64_t tF, uint8_t extraFlags) {
  r.finishBoot = g_bootId;
  r.tFinishF = tF;
  r.status = RUN_FINISHED;
  r.flags |= extraFlags;
  runComputeDuration(r);
  logRunState(r);
  sendFinishNote(r);
  bool best = r.durationUs > 0 && (g_bestRunUs < 0 || r.durationUs < g_bestRunUs);
  if (best) g_bestRunUs = r.durationUs;
  bool leader = r.riderId && runNumber(r) == g_setRuns && isCumulLeader(r.riderId);
  if (g_setBeep) beep((best || leader) ? PAT_BEST : PAT_CROSS);
  g_lastFinishedIdx = r.idx;
  g_runsRev++;
  char d[16];
  fmtDur(d, sizeof d, r.durationUs);
  LOGF("[arrivee] manche %u %s : %s (incertitude %lu us)\n", r.idx + 1, r.name[0] ? r.name : "anonyme", d,
       (unsigned long)(r.uncertUs == UINT32_MAX ? 0 : r.uncertUs));
}

static Orphan* orphanAdd(int64_t tF, uint8_t flags) {
  int slot = 0;
  for (int i = 0; i < MAX_ORPHANS; i++) {
    if (!g_orphans[i].used) { slot = i; break; }
    if (g_orphans[i].tF < g_orphans[slot].tF) slot = i;
  }
  Orphan& o = g_orphans[slot];
  o.used = true;
  o.flags = flags;
  o.id = g_orphanNextId++;
  o.finishBoot = g_bootId;
  o.tF = tF;
  o.wallMs = wallFromLocalUs(tF);
  logOrphan(o);
  g_runsRev++;
  return &o;
}

// Passage detecte : attribue au plus ancien pilote en piste (FIFO)
static bool finishOnCrossing(int64_t tF, uint8_t flags) {
  int64_t minUs = (int64_t)g_setMinS * 1000000LL;
  int found = -1;
  for (uint16_t i = 0; i < g_runCount; i++) {
    Run& r = g_runs[i];
    if (r.status != RUN_ON_COURSE) continue;
    int64_t ts;
    if (runStartF(r, ts) && tF - ts < minUs) break;   // le plus ancien est trop recent : les suivants aussi
    found = i;
    break;
  }
  if (found >= 0) {
    finishRun(g_runs[found], tF, flags);
    return true;
  }
  orphanAdd(tF, flags);
  if (g_setBeep) beep(PAT_CROSS);
  LOGF("[arrivee] passage sans depart correspondant (orphelin)\n");
  return false;
}

static void assignOrphanToRun(Orphan& o, Run& r, uint8_t extra) {
  o.used = false;
  logOrphan(o);
  finishRun(r, o.tF, extra | (o.flags & RUNF_FLASH));
}

static void finishMatchOrphans(Run& r) {
  int64_t ts;
  if (!runStartF(r, ts)) return;
  int64_t minUs = (int64_t)g_setMinS * 1000000LL, maxUs = (int64_t)g_setMaxS * 1000000LL;
  int best = -1;
  for (int i = 0; i < MAX_ORPHANS; i++) {
    Orphan& o = g_orphans[i];
    if (!o.used || o.finishBoot != g_bootId) continue;
    int64_t d = o.tF - ts;
    if (d >= minUs && d <= maxUs && (best < 0 || o.tF < g_orphans[best].tF)) best = i;
  }
  if (best >= 0) assignOrphanToRun(g_orphans[best], r, RUNF_LATE_START);
}

static void benchTryPair(int64_t now) {
  if (g_benchCrossValid && now - g_benchCrossUs > 3000000) g_benchCrossValid = false;
  if (g_benchStartValid && now - g_benchStartRxUs > 3000000) g_benchStartValid = false;
  if (!g_benchCrossValid || !g_benchStartValid) return;
  g_benchCrossValid = g_benchStartValid = false;
  SyncModel* m = modelFind(g_benchStartBoot);
  if (!m || !m->valid) { LOGF("[banc] pas de synchro\n"); return; }
  int64_t ts = modelStartToFinish(*m, g_benchStartRaw);
  int64_t diff = (g_benchCrossUs - (int64_t)CAL_FINISH_US) - (ts - (int64_t)CAL_START_US);
  if (diff > 1000000 || diff < -1000000) return;
  BenchStats& s = g_benchSt;
  s.n++;
  double d = (double)diff, delta = d - s.mean;
  s.mean += delta / s.n;
  s.m2 += delta * (d - s.mean);
  if (s.n == 1 || diff < s.minUs) s.minUs = diff;
  if (s.n == 1 || diff > s.maxUs) s.maxUs = diff;
  s.lastUs = diff;
  beep(PAT_CLICK);
  LOGF("[banc] #%lu ecart %lld us  moyenne %.1f us  ecart-type %.1f us\n", (unsigned long)s.n, (long long)diff, s.mean,
       s.n > 1 ? sqrt(s.m2 / (s.n - 1)) : 0.0);
}

static void finishOnStartEvt(const PktStartEvt& p) {
  uint32_t sb = p.startBoot, sq = p.runSeq;
  for (uint16_t i = 0; i < g_runCount; i++)
    if (g_runs[i].startBoot == sb && g_runs[i].runSeq == sq) { sendAck(PT_START_EVT, sb, sq); return; }   // doublon
  if (p.evtFlags & EVF_BENCH) {
    g_benchStartBoot = sb;
    g_benchStartRaw = p.tStart;
    g_benchStartRxUs = nowUs();
    g_benchStartValid = true;
    sendAck(PT_START_EVT, sb, sq);
    benchTryPair(nowUs());
    return;
  }
  if (!g_runs || g_runCount >= MAX_RUNS) {
    LOGF("[arrivee] memoire des manches pleine : nouvelle session necessaire\n");
    sendAck(PT_START_EVT, sb, sq);
    return;
  }
  Run& r = g_runs[g_runCount];
  memset(&r, 0, sizeof r);
  r.idx = g_runCount;
  r.status = RUN_ON_COURSE;
  r.flags = (p.evtFlags & EVF_FLASH) ? RUNF_START_FLASH : 0;
  r.startBoot = sb;
  r.runSeq = sq;
  r.tStartRaw = p.tStart;
  r.riderId = p.riderId;
  r.bib = p.bib;
  r.durationUs = -1;
  r.uncertUs = UINT32_MAX;
  copyClean(r.name, NAME_LEN, p.name);
  copyClean(r.cat, CAT_LEN, p.cat);
  int64_t ts;
  r.wallStartMs = p.wallMs;
  if (runStartF(r, ts) && r.wallStartMs <= 0) r.wallStartMs = wallFromLocalUs(ts);
  g_runCount++;
  logRunCreate(r);
  logRunState(r);
  sendAck(PT_START_EVT, sb, sq);
  g_runsRev++;
  LOGF("[arrivee] depart recu : %s (manche %u)\n", r.name[0] ? r.name : "anonyme", runNumber(r));
  finishMatchOrphans(r);
}

static void finishOnCancel(const PktCancel& c) {
  uint32_t sb = c.startBoot, sq = c.runSeq;
  for (uint16_t i = 0; i < g_runCount; i++) {
    Run& r = g_runs[i];
    if (r.startBoot == sb && r.runSeq == sq && r.status != RUN_CANCELLED) {
      r.status = RUN_CANCELLED;
      logRunState(r);
      recomputeBest();
      g_runsRev++;
      break;
    }
  }
  sendAck(PT_CANCEL, sb, sq);
}

static void finishAutoDnf(int64_t now) {
  int64_t maxUs = (int64_t)g_setMaxS * 1000000LL;
  for (uint16_t i = 0; i < g_runCount; i++) {
    Run& r = g_runs[i];
    if (r.status != RUN_ON_COURSE || (r.flags & RUNF_MANUAL)) continue;
    int64_t ts;
    if (!runStartF(r, ts)) continue;
    if (now - ts > maxUs) {
      r.status = RUN_DNF;
      r.flags |= RUNF_AUTO_DNF;
      logRunState(r);
      sendFinishNote(r);
      g_runsRev++;
    }
  }
}

// Temps calcules sans synchro (depart recu hors portee) : on les complete des que possible
static void finishResolvePending() {
  for (uint16_t i = 0; i < g_runCount; i++) {
    Run& r = g_runs[i];
    if (r.status != RUN_FINISHED || !(r.flags & RUNF_NO_SYNC) || r.finishBoot != g_bootId) continue;
    runComputeDuration(r);
    if (!(r.flags & RUNF_NO_SYNC)) {
      logRunState(r);
      sendFinishNote(r);
      recomputeBest();
      g_runsRev++;
    }
  }
}

// Actions manuelles depuis la page web. Retourne "" si OK, sinon le message d'erreur.
static const char* finishRunAction(int i, const String& a, int j) {
  if (i < 0 || i >= (int)g_runCount) return "Passage inconnu";
  Run& r = g_runs[i];
  if (a == "dnf") {
    r.status = RUN_DNF;
    r.flags |= RUNF_MANUAL;
  } else if (a == "restore") {
    r.status = r.finishBoot ? RUN_FINISHED : RUN_ON_COURSE;
    r.flags |= RUNF_MANUAL;
    r.flags &= ~RUNF_AUTO_DNF;
  } else if (a == "cancel") {
    r.status = RUN_CANCELLED;
    r.flags |= RUNF_MANUAL;
  } else if (a == "clear") {
    if (r.status != RUN_FINISHED) return "Ce passage n'a pas d'arrivee";
    if (r.finishBoot == g_bootId) orphanAdd(r.tFinishF, r.flags & RUNF_FLASH);
    r.finishBoot = 0;
    r.tFinishF = 0;
    r.durationUs = -1;
    r.status = RUN_ON_COURSE;
    r.flags |= RUNF_MANUAL;
  } else if (a == "swap") {
    if (j < 0 || j >= (int)g_runCount || j == i) return "Numero de passage invalide";
    Run& o = g_runs[j];
    if (r.status != RUN_FINISHED || o.status != RUN_FINISHED) return "Les deux passages doivent etre arrives";
    if (r.finishBoot != o.finishBoot || r.tStartFBoot != r.finishBoot || o.tStartFBoot != o.finishBoot)
      return "Echange impossible : redemarrage de l'arrivee entre les deux";
    int64_t t = r.tFinishF;
    r.tFinishF = o.tFinishF;
    o.tFinishF = t;
    if (r.finishBoot == g_bootId) {
      runComputeDuration(r);
      runComputeDuration(o);
    } else {
      r.durationUs = (r.tFinishF - (int64_t)CAL_FINISH_US) - (r.tStartF - (int64_t)CAL_START_US);
      o.durationUs = (o.tFinishF - (int64_t)CAL_FINISH_US) - (o.tStartF - (int64_t)CAL_START_US);
    }
    r.flags |= RUNF_MANUAL;
    o.flags |= RUNF_MANUAL;
    logRunState(o);
    sendFinishNote(o);
  } else {
    return "Action inconnue";
  }
  logRunState(r);
  sendFinishNote(r);
  recomputeBest();
  g_runsRev++;
  return "";
}

// Relit le journal de session au demarrage (la derniere ligne F d'une manche fait foi)
static void finishLoadLog() {
  if (!g_runs) return;
  File f = LittleFS.open(FINISH_LOG, FILE_READ);
  if (!f) return;
  char buf[256];
  char* fld[12];
  while (f.available()) {
    size_t n = f.readBytesUntil('\n', buf, sizeof(buf) - 1);
    buf[n] = 0;
    if (n && buf[n - 1] == '\r') buf[--n] = 0;
    if (!n) continue;
    int c = splitFields(buf, ';', fld, 12);
    if (buf[0] == 'R' && c >= 11) {
      uint16_t idx = (uint16_t)toUL(fld[1]);
      if (idx != g_runCount || g_runCount >= MAX_RUNS) continue;
      Run& r = g_runs[g_runCount++];
      memset(&r, 0, sizeof r);
      r.idx = idx;
      r.startBoot = toUL(fld[2]);
      r.runSeq = toUL(fld[3]);
      r.tStartRaw = toLL(fld[4]);
      r.riderId = (uint16_t)toUL(fld[5]);
      r.bib = (uint16_t)toUL(fld[6]);
      r.wallStartMs = toLL(fld[7]);
      r.flags = (uint8_t)toUL(fld[8]);
      copyClean(r.name, NAME_LEN, fld[9]);
      copyClean(r.cat, CAT_LEN, fld[10]);
      r.durationUs = -1;
      r.uncertUs = UINT32_MAX;
    } else if (buf[0] == 'F' && c >= 10) {
      uint16_t idx = (uint16_t)toUL(fld[1]);
      if (idx >= g_runCount) continue;
      Run& r = g_runs[idx];
      r.status = (uint8_t)toUL(fld[2]);
      r.flags = (uint8_t)toUL(fld[3]);
      r.finishBoot = toUL(fld[4]);
      r.tStartF = toLL(fld[5]);
      r.tStartFBoot = toUL(fld[6]);
      r.tFinishF = toLL(fld[7]);
      r.durationUs = toLL(fld[8]);
      r.uncertUs = toUL(fld[9]);
    }
    // Les lignes O (orphelins) servent d'historique : apres un redemarrage leur
    // horodatage n'est plus comparable, on ne les recharge pas.
  }
  f.close();
  recomputeBest();
  for (int i = (int)g_runCount - 1; i >= 0; i--)
    if (g_runs[i].status == RUN_FINISHED) { g_lastFinishedIdx = i; break; }
  LOGF("[arrivee] journal relu : %u manches, session %lu\n", g_runCount, (unsigned long)g_sessionId);
}

// ============================================================================
// 15. ARRIVEE : barriere IR
// ============================================================================
static void IRAM_ATTR isrIrRx() {
  int64_t t = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&g_irMux);
  g_irLastUs = t;
  g_irCount++;
  portEXIT_CRITICAL_ISR(&g_irMux);
}

static void IRAM_ATTR isrBench() {
  int64_t t = esp_timer_get_time();
  if (t - g_benchIsrLastUs < 300000) return;
  g_benchIsrLastUs = t;
  HwEvt e;
  e.kind = HW_BENCH;
  e.t = t;
  BaseType_t w = pdFALSE;
  xQueueSendFromISR(g_hwQ, &e, &w);
  if (w) portYIELD_FROM_ISR();
}

static void setBench(bool on) {
  g_bench = on;
  g_benchCrossValid = g_benchStartValid = false;
  if (on) attachInterrupt(digitalPinToInterrupt(PIN_HALL), isrBench, FALLING);
  else detachInterrupt(digitalPinToInterrupt(PIN_HALL));
  LOGF("[banc] %s\n", on ? "ON" : "OFF");
}

// Appelee toutes les ~2 ms par la tache coeur. L'horodatage vient de l'ISR (derniere
// salve recue) : la latence de cette fonction ne change pas le temps mesure.
static void finishBeamService(int64_t now) {
  int64_t last;
  uint32_t cnt;
  portENTER_CRITICAL(&g_irMux);
  last = g_irLastUs;
  cnt = g_irCount;
  portEXIT_CRITICAL(&g_irMux);

  if (g_qWinStartUs == 0) { g_qWinStartUs = now; g_qWinCount = cnt; }
  if (now - g_qWinStartUs >= 200000) {
    uint32_t got = cnt - g_qWinCount;
    uint32_t expct = (uint32_t)((now - g_qWinStartUs) / IR_PERIOD_US);
    uint32_t q = expct ? (got * 100u) / expct : 0;
    g_beamQuality = (uint8_t)(q > 100 ? 100 : q);
    g_qWinStartUs = now;
    g_qWinCount = cnt;
  }

  const int64_t lostUs = (int64_t)IR_PERIOD_US * IR_LOST_PERIODS;
  bool present = (last != 0) && (now - last < lostUs);

  if (!g_beamEstablished) {
    g_beamBroken = false;
    g_beamFault = false;
    if (present && g_beamQuality >= 50) {
      if (!g_beamEstSinceUs) g_beamEstSinceUs = now;
      else if (now - g_beamEstSinceUs > 1000000) {
        g_beamEstablished = true;
        g_beamRearmed = true;
        g_beamRestoredUs = now;
        LOGF("[ir] faisceau etabli (%u %%)\n", g_beamQuality);
      }
    } else {
      g_beamEstSinceUs = 0;
    }
    return;
  }

  if (!g_beamBroken) {
    if (!present) {
      g_beamBroken = true;
      int64_t tBreak = last + (int64_t)IR_PERIOD_US / 2;   // milieu de l'intervalle d'incertitude
      g_beamBrokenAtUs = tBreak;
      g_breakCount++;
      g_breakStartUs = tBreak;
      g_lastBreakDurUs = -1;
      if (g_bench || g_calRunning) {
        g_lastBreakKind = BK_PAUSED;
      } else if (!g_beamRearmed || tBreak < g_lockoutUntilUs) {
        g_lastBreakKind = BK_LOCKOUT;
      } else {
        uint8_t fl = flashOverlap(last) ? RUNF_FLASH : 0;
        g_beamRearmed = false;
        g_lockoutUntilUs = tBreak + (int64_t)FINISH_LOCKOUT_MS * 1000;
        g_lastBreakKind = finishOnCrossing(tBreak, fl) ? BK_RIDER : BK_ORPHAN;
      }
    }
  } else {
    if (present) {
      g_beamBroken = false;
      g_beamRestoredUs = now;
      g_lastBreakDurUs = last - g_breakStartUs;
      if (g_beamFault) { g_beamFault = false; LOGF("[ir] faisceau retabli\n"); }
    } else {
      if (!g_beamFault && now - g_beamBrokenAtUs > (int64_t)BEAM_FAULT_MS * 1000) {
        g_beamFault = true;
        g_beamFaultBeepUs = now;
        beep(PAT_BEAM_FAULT);
        LOGF("[ir] faisceau bloque\n");
      } else if (g_beamFault && now - g_beamFaultBeepUs > 10000000) {
        g_beamFaultBeepUs = now;
        beep(PAT_BEAM_FAULT);
      }
      if (now - g_beamBrokenAtUs > 15000000) {
        g_beamEstablished = false;   // emetteur eteint ou desaligne : on attend un faisceau propre
        g_beamEstSinceUs = 0;
        LOGF("[ir] faisceau perdu\n");
      }
    }
  }
  if (!g_beamBroken && !g_beamRearmed && now >= g_lockoutUntilUs &&
      now - g_beamRestoredUs >= (int64_t)BEAM_REARM_MS * 1000)
    g_beamRearmed = true;
}

static void sendBeamState() {
  PktBeamState b;
  memset(&b, 0, sizeof b);
  b.h = makeHeader(PT_BEAM_STATE);
  b.quality = g_beamQuality;
  b.beamOk = (g_beamEstablished && !g_beamBroken) ? 1 : 0;
  b.align = g_align ? 1 : 0;
  sendToRole(ROLE_EMITTER, &b, sizeof b);
}

static void finishOnPacket(const PktHeader& h, const InMsg& m) {
  if (h.role != ROLE_START) return;
  switch (h.type) {
    case PT_SYNC_RESP: {
      if (m.len < sizeof(PktSync)) return;
      PktSync r;
      memcpy(&r, m.data, sizeof r);
      syncOnResp(r, h.bootId, m.tRx, m.lora ? 2 : (h.hop ? 1 : 0));
      break;
    }
    case PT_START_EVT: {
      if (m.len < sizeof(PktStartEvt)) return;
      PktStartEvt p;
      memcpy(&p, m.data, sizeof p);
      finishOnStartEvt(p);
      break;
    }
    case PT_CANCEL: {
      if (m.len < sizeof(PktCancel)) return;
      PktCancel c;
      memcpy(&c, m.data, sizeof c);
      finishOnCancel(c);
      break;
    }
    default:
      break;
  }
}

// Nouvelle session : export CSV de l'ancienne dans /arch, journal remis a zero
static String buildRunsCsv();   // defini avec les pages web

static void finishResetMemory() {
  g_runCount = 0;
  memset(g_orphans, 0, sizeof g_orphans);
  g_bestRunUs = -1;
  g_lastFinishedIdx = -1;
  memset(&g_benchSt, 0, sizeof g_benchSt);
  g_runsRev++;
}

static void finishNewSession() {
  char path[40];
  snprintf(path, sizeof path, "/arch/session_%03lu.csv", (unsigned long)g_sessionId);
  String* csv = new String(buildRunsCsv());
  if (!storeOp(SO_ARCHIVE, path, FINISH_LOG, csv)) delete csv;   // la tache de stockage ecrit puis efface le journal, dans l'ordre
  g_sessionId++;
  prefsPutU("sess", g_sessionId);
  finishResetMemory();
  LOGF("[arrivee] nouvelle session %lu\n", (unsigned long)g_sessionId);
}

// Efface les passages sans archive. all = vide aussi toutes les archives.
// Le depart recoit l'ordre (manches et derniers departs remis a zero) ; s'il est hors
// de portee, il se recale au prochain heartbeat grace au numero de session.
static void finishWipe(bool all) {
  storeOp(SO_WIPE, FINISH_LOG, all ? "all" : "", nullptr);
  g_sessionId++;
  prefsPutU("sess", g_sessionId);
  finishResetMemory();
  g_breakCount = 0;
  g_lastBreakKind = BK_NONE;
  sendCmd(ROLE_START, all ? CMD_WIPE_ALL : CMD_WIPE_SESSION, g_sessionId);
  beep(PAT_CANCELLED);
  LOGF("[arrivee] effacement %s, session %lu\n", all ? "complet" : "de la session", (unsigned long)g_sessionId);
}

// ---- Reglage automatique de la puissance de l'emetteur ----
// On monte du niveau 1 au niveau 10 ; le premier niveau qui donne un faisceau propre
// (IR_CAL_TARGET % des salves recues) est le minimum. On ajoute IR_CAL_MARGIN niveaux
// (environ x2 en amplitude) pour tenir la pluie, la poussiere et un leger desalignement.
// Trop de puissance = l'IR contourne le pilote par reflexion et la coupure n'est pas vue.
static void calApplyLevel(uint8_t level) {
  g_calLevel = level;
  sendCmd(ROLE_EMITTER, CMD_SET_IR_POWER, level);
  g_calPhaseUs = nowUs();
  g_calMeasuring = false;
}

static bool calStart() {
  if (!peerOk(ROLE_EMITTER, nowUs())) { g_calResult = 3; return false; }
  g_calRunning = true;
  g_calResult = 0;
  g_calFound = g_calFinal = 0;
  LOGF("[ir] reglage auto de la puissance\n");
  calApplyLevel(1);
  return true;
}

static void calService(int64_t now) {
  if (!g_calRunning) return;
  if (!g_calMeasuring) {
    if (now - g_calPhaseUs < 800000) return;   // l'emetteur applique le niveau, le TSSP se stabilise
    portENTER_CRITICAL(&g_irMux);
    g_calCnt0 = g_irCount;
    portEXIT_CRITICAL(&g_irMux);
    g_calT0 = now;
    g_calMeasuring = true;
    return;
  }
  if (now - g_calT0 < 600000) return;
  portENTER_CRITICAL(&g_irMux);
  uint32_t got = g_irCount - g_calCnt0;
  portEXIT_CRITICAL(&g_irMux);
  uint32_t expct = (uint32_t)((now - g_calT0) / IR_PERIOD_US);
  uint32_t q = expct ? got * 100u / expct : 0;
  LOGF("[ir] niveau %u : %lu %%\n", g_calLevel, (unsigned long)q);
  if (q >= IR_CAL_TARGET) {
    g_calFound = g_calLevel;
    g_calFinal = (uint8_t)(g_calLevel + IR_CAL_MARGIN > 10 ? 10 : g_calLevel + IR_CAL_MARGIN);
    sendCmd(ROLE_EMITTER, CMD_SET_IR_POWER, g_calFinal);
    g_calRunning = false;
    g_calResult = 1;
    g_beamEstablished = false;   // on repart d'un faisceau propre au nouveau niveau
    g_beamEstSinceUs = 0;
    beep(PAT_LINK_OK);
    LOGF("[ir] minimum %u, niveau retenu %u\n", g_calFound, g_calFinal);
  } else if (g_calLevel >= 10) {
    sendCmd(ROLE_EMITTER, CMD_SET_IR_POWER, 10);
    g_calRunning = false;
    g_calResult = 2;
    g_beamEstablished = false;
    g_beamEstSinceUs = 0;
    beep(PAT_ERROR);
    LOGF("[ir] reglage auto : faisceau insuffisant meme a pleine puissance\n");
  } else {
    calApplyLevel(g_calLevel + 1);
  }
}

// ============================================================================
// 16. EMETTEUR IR : porteuse 38 kHz en salves, generee par le RMT (0 % CPU)
// ============================================================================
static bool irTxStart(uint8_t level) {
  static rmt_data_t sym[IR_BURST_CYCLES + 1];
  if (level < 1) level = 1;
  if (level > 10) level = 10;
  if (g_irTxOk) { rmtDeinit(PIN_IR_TX); g_irTxOk = false; }   // changement de niveau : on repart de zero
  if (!rmtInit(PIN_IR_TX, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000)) return false;   // 1 tick = 0,1 us
  uint16_t on = IR_LEVEL_TICKS[level - 1];
  for (int i = 0; i < IR_BURST_CYCLES; i++) {
    sym[i].level0 = 1; sym[i].duration0 = on;
    sym[i].level1 = 0; sym[i].duration1 = 263 - on;
  }
  uint32_t gap = (uint32_t)IR_GAP_US * 10;
  sym[IR_BURST_CYCLES].level0 = 0; sym[IR_BURST_CYCLES].duration0 = gap / 2;
  sym[IR_BURST_CYCLES].level1 = 0; sym[IR_BURST_CYCLES].duration1 = gap - gap / 2;
  bool ok = rmtWriteLooping(PIN_IR_TX, sym, IR_BURST_CYCLES + 1);
  LOGF("[ir] emission %s : niveau %u, %d periodes + %d us, periode %lu us\n", ok ? "OK" : "ECHEC", level, IR_BURST_CYCLES,
       IR_GAP_US, (unsigned long)IR_PERIOD_US);
  return ok;
}

static void emitterSetPower(uint8_t level) {
  if (level < 1 || level > 10) return;
  if (level == g_irPower && g_irTxOk) return;   // commande recue en triple
  g_irPower = level;
  flashMark(true);
  g_prefs.putUChar("irpow", level);
  flashMark(false);
  g_irTxOk = irTxStart(level);
  if (!g_irTxOk) beep(PAT_ERROR);
}

static void emitterOnPacket(const PktHeader& h, const InMsg& m) {
  if (h.role != ROLE_FINISH) return;
  if (h.type == PT_BEAM_STATE && m.len >= sizeof(PktBeamState)) {
    PktBeamState b;
    memcpy(&b, m.data, sizeof b);
    g_remoteBeamQ = b.quality;
    g_remoteBeamQUs = m.tRx;
  } else if (h.type == PT_HEARTBEAT) {
    const PeerInfo& p = g_peers[ROLE_FINISH];
    if (m.tRx - g_remoteBeamQUs > 1500000) { g_remoteBeamQ = p.beamQuality; g_remoteBeamQUs = m.tRx; }
    bool fa = (p.hbFlags & HBF_ALIGN) != 0;
    if (fa != g_align && m.tRx - g_alignLocalChangeUs > 3000000) setAlign(fa, false);
  }
}

// ============================================================================
// 17. TACHE COEUR : interruptions, paquets, taches periodiques
// ============================================================================
static void IRAM_ATTR isrHall() {
  int64_t t = esp_timer_get_time();   // (1) horodatage, rien d'autre ici
  if (!g_hallArmed) return;
  g_hallArmed = false;                // le retour de la tige ne redeclenchera pas
  HwEvt e;
  e.kind = HW_HALL;
  e.t = t;
  BaseType_t w = pdFALSE;
  xQueueSendFromISR(g_hwQ, &e, &w);
  if (w) portYIELD_FROM_ISR();
}

static void onHeartbeat(uint8_t role, const InMsg& m) {
  if (m.len < sizeof(PktHeartbeat)) return;
  PktHeartbeat hb;
  memcpy(&hb, m.data, sizeof hb);
  PeerInfo& p = g_peers[role];
  p.battMv = hb.battMv;
  p.hbFlags = hb.h.flags;
  p.uptimeS = hb.uptimeS;
  p.sessionId = hb.sessionId;
  p.syncUncertUs = hb.syncUncertUs;
  p.beamQuality = hb.beamQuality;
  p.pending = hb.pending;
  p.relayBattStart = hb.relayBattStart;
  p.relayBattEmitter = hb.relayBattEmitter;
  p.ip = hb.ip;
  memcpy(p.fw, hb.fw, sizeof p.fw);
  p.fw[sizeof(p.fw) - 1] = 0;
  p.irPower = hb.irPower;
  memcpy(p.rxDir, hb.rxDir, 5);
  memcpy(p.rxRel, hb.rxRel, 5);
  memcpy(p.rxAny, hb.rxAny, 5);
  memcpy(p.rssiRep, hb.rssi, 5);
  memcpy(p.rxLoraRep, hb.rxLora, 5);
  memcpy(p.lrssiRep, hb.lrssi, 5);
}

static void onCmd(const InMsg& m) {
  if (m.len < sizeof(PktCmd)) return;
  PktCmd c;
  memcpy(&c, m.data, sizeof c);
  if (c.targetRole != CHRONO_ROLE) return;
  switch (c.cmd) {
    case CMD_MAINTENANCE: requestMaintenance(); break;
    case CMD_ALIGN_ON:    if (!g_align) setAlign(true, false); break;
    case CMD_ALIGN_OFF:   if (g_align) setAlign(false, false); break;
    case CMD_LOCATE:      beep(PAT_LOCATE); break;
    case CMD_SET_IR_POWER: if (IS_EMITTER) emitterSetPower((uint8_t)c.arg); break;
    case CMD_WIPE_SESSION: if (IS_START) startWipe(c.arg, false); break;
    case CMD_WIPE_ALL:     if (IS_START) startWipe(c.arg, true); break;
    default: break;
  }
}

// Relais : repete un paquet du depart ou de l'arrivee vers son destinataire.
// Pour la synchro, ajoute le temps passe ici (de la reception a la reemission).
static void relayForward(const PktHeader& h, const InMsg& m) {
  uint8_t buf[MAX_PKT];
  memcpy(buf, m.data, m.len);
  patchRoute(buf, h.dst, 1);
  uint8_t mac[6];
  memcpy(mac, BCAST_MAC, 6);
  if (h.dst >= 1 && h.dst <= 4 && g_peers[h.dst].macKnown) memcpy(mac, g_peers[h.dst].mac, 6);
  if ((h.type == PT_SYNC_REQ || h.type == PT_SYNC_RESP) && m.len >= sizeof(PktSync)) {
    PktSync sy;
    memcpy(&sy, buf, sizeof sy);
    uint32_t res = (uint32_t)(nowUs() - m.tRx);
    if (h.type == PT_SYNC_REQ) sy.resReq += res;
    else sy.resResp += res;
    memcpy(buf, &sy, sizeof sy);
  }
  sendRaw(mac, buf, m.len);
  g_relayFwd++;
}

static void handlePacket(const InMsg& m) {
  if (m.len < sizeof(PktHeader)) return;
  PktHeader h;
  memcpy(&h, m.data, sizeof h);
  uint8_t r = h.role;
  if (r < 1 || r > 4 || r == CHRONO_ROLE) return;
  PeerInfo& p = g_peers[r];
  bool direct = (h.hop == 0) && !m.lora;
  if (m.lora) {
    p.lastLoraUs = m.tRx;
    p.loraRssi = m.rssi;
    p.loraSnr = m.snr;
    if (h.type == PT_HEARTBEAT) { p.loraHbT[p.loraHbHead] = m.tRx; p.loraHbHead = (uint8_t)((p.loraHbHead + 1) % 6); }
  } else if (direct) {
    // L'adresse et le RSSI ne s'apprennent qu'en direct (un paquet relaye vient du relais)
    if (!p.macKnown || memcmp(p.mac, m.mac, 6) != 0) {   // nouvelle adresse (rechange, maintenance)
      memcpy(p.mac, m.mac, 6);
      p.macKnown = true;
      ensurePeer(m.mac);
    }
    p.lastDirectUs = m.tRx;
    p.rssi = m.rssi;
  } else {
    p.lastRelayUs = m.tRx;
  }
  p.known = true;
  p.lastSeenUs = m.tRx;
  p.bootId = h.bootId;
  if (h.type == PT_HEARTBEAT && !m.lora) hbRecord(p, h, m.tRx);

  // Relais : on repete tout ce qui vient du depart ou de l'arrivee et ne nous est pas destine
  if (IS_RELAY && direct && !m.lora && (r == ROLE_START || r == ROLE_FINISH) &&
      (h.dst == 0 || ((h.dst == ROLE_START || h.dst == ROLE_FINISH) && h.dst != r)))
    relayForward(h, m);

  // Doublons : la synchro en est exclue (un paquet par chemin, chacun avec son numero)
  if (h.type != PT_SYNC_REQ && h.type != PT_SYNC_RESP && seenBefore(r, h.bootId, h.seq)) return;
  if (h.dst != 0 && h.dst != CHRONO_ROLE) return;   // paquet de passage
  g_rxViaLora = m.lora != 0;

  if (h.type == PT_HEARTBEAT) onHeartbeat(r, m);
  if (h.type == PT_CMD) onCmd(m);
  if (IS_START) startOnPacket(h, m);
  if (IS_FINISH) finishOnPacket(h, m);
  if (IS_EMITTER) emitterOnPacket(h, m);
}

static void handleHw(const HwEvt& e) {
  if (e.kind == HW_LORA) { loraOnDio0(e.t); return; }
  if (IS_START && e.kind == HW_HALL) {
    g_lastTrigUs = e.t;
    g_hallHighSinceUs = 0;
    startCreate(e.t);
  }
  if (IS_FINISH && e.kind == HW_BENCH) {
    g_benchCrossUs = e.t;
    g_benchCrossValid = true;
    benchTryPair(nowUs());
  }
}

static void linkBeeps(int64_t now) {
  if (IS_FINISH) return;
  bool ok = IS_RELAY ? (directOk(ROLE_START, now) && directOk(ROLE_FINISH, now)) : peerOk(ROLE_FINISH, now);
  if (ok && !g_linkWasOk) beep(PAT_LINK_OK);
  if (!ok && g_linkWasOk) { beep(PAT_LINK_LOST); g_linkLostBeepUs = now; }
  else if (!ok && g_linkEverOk && LINK_LOST_BEEP && now - g_linkLostBeepUs > 30000000LL) {
    beep(PAT_LINK_LOST);
    g_linkLostBeepUs = now;
  }
  if (ok) g_linkEverOk = true;
  g_linkWasOk = ok;
}

static void finishSyncBeep(int64_t now) {
  SyncModel* m = currentModel();
  if (m && syncStateOf(m, now) == 1 && m->startBoot != g_syncBeepBoot) {
    g_syncBeepBoot = m->startBoot;
    beep(PAT_LINK_OK);
    LOGF("[sync] synchro OK : +/- %lu us, derive %.2f ppm\n", (unsigned long)modelUncertAt(*m, now), m->b);
  }
}

static void periodic(int64_t now) {
  loraService(now);
  if (now - g_lastHbUs >= (int64_t)HEARTBEAT_MS * 1000) {
    g_lastHbUs = now;
    bool lh = g_loraOk && now - g_lastLoraHbUs >= (int64_t)LORA_HB_MS * 1000;
    if (lh) g_lastLoraHbUs = now;
    sendHeartbeat(now, lh);
    linkBeeps(now);
  }
  if (IS_START) {
    startTuneService(now);
    startHallService(now);
    outboxService(now);
  }
  if (IS_FINISH) {
    finishBeamService(now);
    syncScheduler(now);
    if (g_align && now - g_lastBeamStateTxUs >= 200000) { g_lastBeamStateTxUs = now; sendBeamState(); }
    if (now - g_lastSlowUs >= 1000000) {
      g_lastSlowUs = now;
      finishAutoDnf(now);
      finishResolvePending();
      benchTryPair(now);
      finishSyncBeep(now);
    }
    calService(now);
  }
}

static void coreTask(void*) {
  bool wdt = (esp_task_wdt_add(nullptr) == ESP_OK);
  InMsg m;
  HwEvt e;
  for (;;) {
    bool got = (xQueueReceive(g_inQ, &m, pdMS_TO_TICKS(2)) == pdTRUE);
    {
      Lock l;
      while (xQueueReceive(g_hwQ, &e, 0) == pdTRUE) handleHw(e);
      if (got) {
        handlePacket(m);
        for (int k = 0; k < 8 && xQueueReceive(g_inQ, &m, 0) == pdTRUE; k++) handlePacket(m);
      }
      periodic(nowUs());
    }
    if (wdt) esp_task_wdt_reset();
  }
}

// ============================================================================
// 18. TACHE DE STOCKAGE (toutes les ecritures flash passent par ici, dans l'ordre)
// ============================================================================
static void storageTask(void*) {
  StoreMsg m;
  for (;;) {
    if (xQueueReceive(g_storeQ, &m, portMAX_DELAY) != pdTRUE) continue;
    if (!g_fsOk) { if (m.op == SO_ARCHIVE) delete (String*)m.ptr; continue; }
    switch (m.op) {
      case SO_APPEND:
        fsAppend(m.path, m.line);
        break;
      case SO_SAVE_RIDERS:
        saveRidersNow();
        break;
      case SO_RENAME:   // line = source, path = destination
        flashMark(true);
        if (LittleFS.exists(m.line)) { LittleFS.remove(m.path); LittleFS.rename(m.line, m.path); }
        flashMark(false);
        break;
      case SO_WIPE:   // path = fichier a supprimer, line = "all" pour vider aussi les archives
        flashMark(true);
        if (m.path[0] && LittleFS.exists(m.path)) LittleFS.remove(m.path);
        if (!strcmp(m.line, "all")) fsWipeArchives();
        flashMark(false);
        break;
      case SO_ARCHIVE: {   // path = archive CSV, line = journal a effacer, ptr = contenu
        String* s = (String*)m.ptr;
        flashMark(true);
        File f = LittleFS.open(m.path, FILE_WRITE);
        if (f) { f.print(*s); f.close(); }
        LittleFS.remove(m.line);
        flashMark(false);
        delete s;
        break;
      }
      default:
        break;
    }
  }
}

// ============================================================================
// 19. MAINTENANCE (depart et emetteur) : rejoint le Wi-Fi de l'arrivee pour l'OTA
// ============================================================================
static void otaSetup() {
  ArduinoOTA.setHostname(myHostname());
  ArduinoOTA.setPassword(ADMIN_PASSWORD);
  ArduinoOTA.onStart([]() {
    g_otaRunning = true;
    ledcWriteTone(PIN_BUZZER, 0);
    LOGF("[ota] debut\n");
  });
  ArduinoOTA.onEnd([]() { LOGF("[ota] fin, redemarrage\n"); });
  ArduinoOTA.onError([](ota_error_t e) {
    g_otaRunning = false;
    LOGF("[ota] erreur %u\n", (unsigned)e);
  });
  ArduinoOTA.begin();
}

static void webSetupCommon();   // definies avec les pages web
static void webSetupMaintenance();

static void maintenanceSetup() {
  g_maintenance = true;
  g_maintSinceUs = nowUs();
  WiFi.persistent(false);
  WiFi.setHostname(myHostname());
  WiFi.mode(WIFI_STA);
  WiFi.begin(AP_SSID_FINISH, AP_PASSWORD);
  LOGF("[maint] connexion a %s", AP_SSID_FINISH);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) { delay(250); LOGF("."); }
  if (WiFi.status() == WL_CONNECTED) {
    LOGF(" OK, IP %s\n", WiFi.localIP().toString().c_str());
    radioInit(WIFI_IF_STA, false);   // debit normal : pour annoncer l'IP a l'arrivee (affichee dans sa page)
  } else {
    char ssid[32];
    snprintf(ssid, sizeof ssid, "%s-MAJ", IS_START ? AP_SSID_START : IS_RELAY ? "CHRONO-RELAIS" : "CHRONO-EMETTEUR");
    LOGF(" echec, point d'acces %s (192.168.4.1)\n", ssid);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, AP_PASSWORD, RADIO_CHANNEL);
  }
  esp_wifi_set_ps(WIFI_PS_NONE);
  otaSetup();
  webSetupCommon();
  webSetupMaintenance();
  g_web.begin();
  beep(PAT_MAINT);
}

static void maintenanceLoop() {
  static int64_t lastHb = 0, lastBeep = 0;
  ArduinoOTA.handle();
  g_web.handleClient();
  buttonService();
  int64_t now = nowUs();
  if (g_espNowOk && now - lastHb > 1000000) { lastHb = now; sendHeartbeat(now, false); }
  if (now - lastBeep > 8000000) { lastBeep = now; beep(PAT_MAINT); }
  if (now - g_maintSinceUs > (int64_t)MAINT_TIMEOUT_MIN * 60 * 1000000LL) ESP.restart();
}

// ============================================================================
// 20. CONSOLE SERIE (USB) : tape "help"
// ============================================================================
static void consoleCmd(char* cmd) {
  int64_t now = nowUs();
  if (!strcmp(cmd, "help")) {
    Serial.println("status | sync | lora | ir | hall | irpow N | ircal | btn | batt | runs | riders | bench on|off|reset | beep | log | maint | newsession | wipe | wipeall | reboot");
  } else if (!strcmp(cmd, "status")) {
    Lock l;
    Serial.printf("role %d  boot %lu  session %lu  fw %s  batt %u mV  stockage perdu %lu\n", CHRONO_ROLE,
                  (unsigned long)g_bootId, (unsigned long)g_sessionId, FW_VERSION, g_battMv, (unsigned long)g_storeDrops);
    Serial.printf("radio : %s, relayes %lu\n", g_lr ? "Long Range" : "standard", (unsigned long)g_relayFwd);
    for (int r = 1; r <= 4; r++) {
      const PeerInfo& p = g_peers[r];
      if (!p.known || r == CHRONO_ROLE) continue;
      uint8_t d, rl, a;
      hbStats(p, now, d, rl, a);
      Serial.printf("  role %d : direct %u %% (rssi %d), via relais %u %%, total %u %%, batt %u mV, fw %s\n", r, d, p.rssi, rl, a,
                    p.battMv, p.fw);
    }
  } else if (!strcmp(cmd, "sync")) {
    Lock l;
    for (int i = 0; i < 2; i++) {
      const SyncModel& m = g_models[i];
      if (!m.used) continue;
      Serial.printf("modele boot %lu : valide %d, points %u, echantillons %lu, rtt min %lu us, derive %.2f ppm, residus %.1f us, incertitude %lu us\n",
                    (unsigned long)m.startBoot, m.valid, m.n, (unsigned long)m.samples, (unsigned long)m.minRtt, m.b, m.residUs,
                    (unsigned long)modelUncertAt(m, now));
    }
  } else if (!strcmp(cmd, "ir")) {
    Serial.printf("faisceau etabli %d coupe %d defaut %d qualite %u %%  impulsions %lu  derniere il y a %lld us  coupures %lu\n",
                  g_beamEstablished, g_beamBroken, g_beamFault, g_beamQuality, (unsigned long)g_irCount, (long long)(now - g_irLastUs),
                  (unsigned long)g_breakCount);
  } else if (!strncmp(cmd, "irpow", 5)) {
    int l = atoi(cmd + 5);
    if (l < 1 || l > 10) Serial.printf("niveau actuel : %u (1 a 10)\n", IS_EMITTER ? g_irPower : g_peers[ROLE_EMITTER].irPower);
    else if (IS_EMITTER) { Lock lk; emitterSetPower((uint8_t)l); }
    else if (IS_FINISH) sendCmd(ROLE_EMITTER, CMD_SET_IR_POWER, (uint32_t)l);
  } else if (!strcmp(cmd, "ircal") && IS_FINISH) {
    Lock l;
    Serial.println(calStart() ? "reglage auto lance" : "emetteur absent");
  } else if (!strcmp(cmd, "wipe") && IS_FINISH) {
    Lock l;
    finishWipe(false);
  } else if (!strcmp(cmd, "wipeall") && IS_FINISH) {
    Lock l;
    finishWipe(true);
  } else if (!strcmp(cmd, "lora")) {
    Lock l;
    if (!LORA_ME) Serial.println("LoRa non prevu pour ce role (config.h)");
    else if (!g_loraOk) Serial.println("LoRa : module absent");
    else
      Serial.printf("LoRa %.3f MHz SF%d %d dBm : emis %lu, recus %lu, CRC %lu, jetes %lu, delais %lu, file %d, dernier %d dBm SNR %d, reserve %lld ms\n",
                    LORA_FREQ_HZ / 1e6, LORA_SF, LORA_TX_DBM, (unsigned long)g_loraTxCount, (unsigned long)g_loraRxCount,
                    (unsigned long)g_loraCrcErr, (unsigned long)g_loraDrop, (unsigned long)g_loraTimeouts, g_loraQn, g_loraLastRssi,
                    g_loraLastSnr, (long long)(g_loraTokens / 1000));
  } else if (!strcmp(cmd, "hall")) {
    for (int i = 0; i < 20; i++) { Serial.printf("capteur : %s\n", digitalRead(PIN_HALL) == LOW ? "AIMANT" : "rien"); delay(250); }
  } else if (!strcmp(cmd, "btn")) {
    for (int i = 0; i < 10; i++) { Serial.printf("bouton : %lu mV\n", (unsigned long)analogReadMilliVolts(PIN_BUTTON)); delay(200); }
  } else if (!strcmp(cmd, "batt")) {
    Serial.printf("batterie : %u mV (%d %%), brut %lu mV\n", g_battMv, battPercent(g_battMv), (unsigned long)analogReadMilliVolts(PIN_BATT));
  } else if (!strcmp(cmd, "runs") && IS_FINISH) {
    Lock l;
    for (uint16_t i = 0; i < g_runCount; i++) {
      const Run& r = g_runs[i];
      char d[16];
      fmtDur(d, sizeof d, r.durationUs);
      Serial.printf("%3u  %-24s st %u  fl %02x  %s\n", r.idx + 1, r.name, r.status, r.flags, d);
    }
  } else if (!strcmp(cmd, "riders") && IS_START) {
    Lock l;
    for (uint16_t i = 0; i < g_riderCount; i++)
      Serial.printf("%c %3u  dossard %u  %s (%s)  manches %u\n", i == g_nextIdx ? '>' : ' ', g_riders[i].id, g_riders[i].bib,
                    g_riders[i].name, g_riders[i].cat, g_riders[i].runs);
  } else if (!strncmp(cmd, "bench", 5) && IS_FINISH) {
    Lock l;
    if (strstr(cmd, "on")) setBench(true);
    else if (strstr(cmd, "off")) setBench(false);
    else if (strstr(cmd, "reset")) memset(&g_benchSt, 0, sizeof g_benchSt);
    const BenchStats& s = g_benchSt;
    Serial.printf("banc %d : n %lu  moyenne %.1f us  ecart-type %.1f us  min %lld  max %lld\n", g_bench, (unsigned long)s.n, s.mean,
                  s.n > 1 ? sqrt(s.m2 / (s.n - 1)) : 0.0, (long long)s.minUs, (long long)s.maxUs);
  } else if (!strcmp(cmd, "beep")) {
    beepStatus();
  } else if (!strcmp(cmd, "log")) {
    File f = LittleFS.open(IS_START ? STARTS_LOG : FINISH_LOG, FILE_READ);
    if (f) { while (f.available()) Serial.write(f.read()); f.close(); }
    Serial.println("-- fin --");
  } else if (!strcmp(cmd, "maint")) {
    requestMaintenance();
  } else if (!strcmp(cmd, "newsession") && IS_FINISH) {
    Lock l;
    finishNewSession();
  } else if (!strcmp(cmd, "reboot")) {
    ESP.restart();
  } else if (cmd[0]) {
    Serial.println("commande inconnue, tape help");
  }
}

static void consolePoll() {
  static char buf[48];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { buf[n] = 0; consoleCmd(buf); n = 0; }
    else if (n < sizeof(buf) - 1) buf[n++] = c;
  }
}

// ============================================================================
// 21. PAGES WEB (hors ligne : aucune ressource externe, polices systeme)
// ============================================================================
static const char CSS_COMMON[] PROGMEM = 
  "\n"
  ":root{--bg:#060708;--card:#101215;--card2:#171a1e;--line:#262a30;--txt:#f4f5f7;--mut:#8a9099;--acc:#ffd23f;--ok:#2fd27a;--bad:#ff4d4d;--warn:#ffae1a}\n"
  "*{box-sizing:border-box}\n"
  "html,body{margin:0;background:var(--bg);color:var(--txt);font:15px/1.4 -apple-system,BlinkMacSystemFont,\"Segoe UI\",Roboto,Helvetica,Arial,sans-serif;-webkit-text-size-adjust:100%}\n"
  ".n{font-variant-numeric:tabular-nums}\n"
  ".wrap{max-width:980px;margin:0 auto;padding:0 16px 56px}\n"
  "header{position:sticky;top:0;z-index:5;background:rgba(6,7,8,.94);backdrop-filter:blur(8px);border-bottom:1px solid var(--line)}\n"
  ".hd{max-width:980px;margin:0 auto;padding:10px 16px;display:flex;align-items:center;gap:10px;flex-wrap:wrap}\n"
  ".brand{font-weight:900;letter-spacing:.14em;font-size:13px;text-transform:uppercase}\n"
  ".brand b{color:var(--acc)}\n"
  ".pills{display:flex;gap:6px;flex-wrap:wrap;margin-left:auto}\n"
  ".pill{font-size:12px;padding:4px 9px;border-radius:999px;background:var(--card2);border:1px solid var(--line);color:var(--mut);white-space:nowrap}\n"
  ".pill.ok{color:var(--ok);border-color:rgba(47,210,122,.35)}\n"
  ".pill.bad{color:var(--bad);border-color:rgba(255,77,77,.4)}\n"
  ".pill.warn{color:var(--warn);border-color:rgba(255,174,26,.4)}\n"
  ".card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px;margin-top:14px}\n"
  ".lbl{font-size:11px;letter-spacing:.14em;text-transform:uppercase;color:var(--mut);font-weight:700}\n"
  ".big{font-size:clamp(56px,16vw,128px);font-weight:900;line-height:1;letter-spacing:-.02em;font-variant-numeric:tabular-nums;margin:6px 0}\n"
  ".mid{font-size:clamp(22px,5.5vw,34px);font-weight:800;line-height:1.15}\n"
  ".acc{color:var(--acc)}.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}.mut{color:var(--mut)}\n"
  "table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}\n"
  "th{font-size:11px;letter-spacing:.1em;text-transform:uppercase;color:var(--mut);text-align:left;font-weight:700;padding:8px 6px;border-bottom:1px solid var(--line);white-space:nowrap}\n"
  "td{padding:9px 6px;border-bottom:1px solid var(--line);vertical-align:middle}\n"
  "tr:last-child td{border-bottom:none}\n"
  ".r{text-align:right}\n"
  "button,.btn{appearance:none;border:1px solid var(--line);background:var(--card2);color:var(--txt);border-radius:10px;padding:9px 14px;font:inherit;font-weight:700;cursor:pointer;text-decoration:none;display:inline-block;text-align:center}\n"
  "button:active,.btn:active{transform:translateY(1px)}\n"
  "button.pri{background:var(--acc);color:#111;border-color:var(--acc)}\n"
  "button.dng{color:var(--bad);border-color:rgba(255,77,77,.4)}\n"
  ".sm{padding:5px 9px!important;font-size:12px!important;border-radius:8px!important}\n"
  "input,select{background:#0b0c0e;color:var(--txt);border:1px solid var(--line);border-radius:10px;padding:9px 11px;font:inherit;width:100%;min-width:0}\n"
  "label{display:flex;flex-direction:column;gap:5px;font-size:12px;color:var(--mut)}\n"
  ".row{display:flex;gap:10px;flex-wrap:wrap;align-items:flex-end}\n"
  ".row>*{flex:1 1 140px}\n"
  ".tabs{display:flex;gap:6px;margin-top:16px;overflow-x:auto}\n"
  ".tabs button{flex:0 0 auto}\n"
  ".tabs button.on{background:var(--txt);color:#000;border-color:var(--txt)}\n"
  ".hide{display:none!important}\n"
  ".badge{display:inline-block;font-size:10px;font-weight:800;letter-spacing:.06em;padding:2px 6px;border-radius:6px;background:var(--card2);border:1px solid var(--line);color:var(--mut);margin-left:6px;text-transform:uppercase;vertical-align:middle}\n"
  ".badge.w{color:var(--warn);border-color:rgba(255,174,26,.4)}\n"
  ".scroll{overflow-x:auto}\n"
  "hr{border:none;border-top:1px solid var(--line);margin:18px 0}\n"
  ".toast{position:fixed;left:50%;bottom:20px;transform:translateX(-50%);background:var(--txt);color:#000;padding:10px 16px;border-radius:10px;font-weight:700;opacity:0;transition:opacity .2s;pointer-events:none;z-index:9;max-width:90vw}\n"
  ".toast.on{opacity:1}\n"
  ".dot{width:9px;height:9px;border-radius:50%;background:var(--mut);display:inline-block}\n"
  ".dot.ok{background:var(--ok)}.dot.bad{background:var(--bad)}\n"
  ".hl{background:rgba(255,210,63,.07)}\n";

static const char JS_COMMON[] PROGMEM = 
  "\n"
  "const $=s=>document.querySelector(s);\n"
  "const esc=s=>String(s==null?'':s).replace(/[&<>\"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;',\"'\":'&#39;'}[c]));\n"
  "function fmtDur(us){if(us==null||us<0)return'--';const cs=Math.floor(us/1e4),m=Math.floor(cs/6000),s=Math.floor(cs/100)%60,c=cs%100;return m+':'+String(s).padStart(2,'0')+'.'+String(c).padStart(2,'0');}\n"
  "function fmtLive(us){if(us==null||us<0)return'--';const ds=Math.floor(us/1e5),m=Math.floor(ds/600),s=Math.floor(ds/10)%60;return m+':'+String(s).padStart(2,'0')+'.'+(ds%10);}\n"
  "function fmtGap(us){if(!(us>0))return'';const t=fmtDur(us);return '+'+(t.startsWith('0:')?t.slice(2):t);}\n"
  "function fmtClock(ms){if(!ms||ms<=0)return'-';return new Date(ms).toLocaleTimeString('fr-FR');}\n"
  "function pct(mv){const t=[[4200,100],[4100,88],[4000,76],[3900,62],[3800,46],[3750,36],[3700,26],[3650,16],[3600,10],[3500,5],[3300,0]];if(mv>=4200)return 100;for(let i=1;i<t.length;i++){if(mv>=t[i][0]){const a=t[i],b=t[i-1];return Math.round(a[1]+(mv-a[0])*(b[1]-a[1])/(b[0]-a[0]));}}return 0;}\n"
  "function batt(mv){if(!mv)return'--';if(mv<2500)return'USB';return pct(mv)+'% '+(mv/1000).toFixed(2)+'V';}\n"
  "function battCls(mv){if(!mv||mv<2500)return'';return mv<3400?'bad':mv<3500?'warn':'ok';}\n"
  "function pill(id,txt,cls){const e=document.getElementById(id);if(!e)return;e.textContent=txt;e.className='pill '+(cls||'');}\n"
  "async function api(p,o){const r=await fetch(p,o);const t=await r.text();if(!r.ok)throw new Error(t||r.statusText);try{return JSON.parse(t);}catch(e){return t;}}\n"
  "function post(p,b){return api(p,{method:'POST',body:b==null?'':b,headers:{'Content-Type':'text/plain'}});}\n"
  "function toast(m){const t=$('#toast');if(!t)return;t.textContent=m;t.classList.add('on');clearTimeout(t._h);t._h=setTimeout(()=>t.classList.remove('on'),2200);}\n"
  "function act(p,ok){return post(p).then(()=>{if(ok)toast(ok);refresh(true);}).catch(e=>toast('Erreur : '+e.message));}\n"
  "function ipStr(n){return [n&255,(n>>>8)&255,(n>>>16)&255,(n>>>24)&255].join('.');}\n"
  "function loadArch(){api('/api/archives').then(a=>{const el=$('#arch');if(!el)return;el.innerHTML=a.map(x=>`<tr><td class=\"n\">${esc(x.f)}</td><td class=\"r mut n\" style=\"white-space:nowrap\">${Math.ceil(x.s/1024)} ko</td><td class=\"r\" style=\"white-space:nowrap\"><a class=\"btn sm\" href=\"/api/archive?f=${encodeURIComponent(x.f)}\">Télécharger</a> <button class=\"sm dng\" data-f=\"${esc(x.f)}\" onclick=\"if(confirm('Supprimer cette archive ?'))act('/api/archive/del?f='+encodeURIComponent(this.dataset.f),'Archive supprimée').then(loadArch)\">Suppr.</button></td></tr>`).join('')||'<tr><td class=\"mut\">Aucune archive</td></tr>';}).catch(()=>{});}\n"
  "post('/api/time?ms='+Date.now()+'&tz='+new Date().getTimezoneOffset()).catch(()=>{});\n";

static const char PAGE_UPDATE[] PROGMEM = 
  "<!doctype html><html lang=\"fr\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><title>Mise à jour</title><link rel=\"stylesheet\" href=\"/s.css\"></head><body>\n"
  "<div class=\"wrap\"><div class=\"card\"><div class=\"lbl\">Mise à jour du firmware</div><div class=\"mid\" id=\"who\" style=\"margin:6px 0 14px\"></div>\n"
  "<div class=\"mut\" style=\"font-size:13px\">Arduino IDE : Croquis, Exporter les binaires compilés, puis choisis le fichier chronodh.ino.bin (pas le .merged.bin, ni bootloader, ni partitions). Vérifie que CHRONO_ROLE dans config.h correspond à ce boîtier.</div>\n"
  "<div class=\"row\" style=\"margin-top:12px\"><input type=\"file\" id=\"f\" accept=\".bin\"><button class=\"pri\" onclick=\"up()\">Envoyer</button></div>\n"
  "<div style=\"margin-top:12px;height:10px;background:var(--card2);border-radius:6px;overflow:hidden\"><div id=\"bar\" style=\"height:100%;width:0;background:var(--acc)\"></div></div>\n"
  "<div id=\"msg\" class=\"mut\" style=\"margin-top:8px\"></div>\n"
  "<div class=\"row\" style=\"margin-top:14px\"><a class=\"btn\" href=\"/\">Accueil</a><button onclick=\"fetch('/api/reboot',{method:'POST'});document.getElementById('msg').textContent='Redémarrage...'\">Redémarrer</button></div></div></div>\n"
  "<script>\n"
  "document.getElementById('who').textContent=location.host;\n"
  "function up(){const f=document.getElementById('f').files[0];if(!f)return;const x=new XMLHttpRequest(),fd=new FormData();fd.append('firmware',f,f.name);const msg=document.getElementById('msg');msg.textContent='Envoi...';\n"
  "x.upload.onprogress=e=>{if(e.lengthComputable)document.getElementById('bar').style.width=(100*e.loaded/e.total)+'%';};\n"
  "x.onload=()=>{msg.textContent=x.status===200?'Mise à jour réussie, redémarrage en cours.':'Échec : '+x.responseText;};\n"
  "x.onerror=()=>{msg.textContent='Erreur réseau';};x.open('POST','/update');x.send(fd);}\n"
  "</script></body></html>";

static const char PAGE_START[] PROGMEM = 
  "<!doctype html><html lang=\"fr\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><meta name=\"theme-color\" content=\"#060708\"><title>Chrono DH - Départ</title><link rel=\"stylesheet\" href=\"/s.css\"></head><body>\n"
  "<header><div class=\"hd\"><span class=\"dot\" id=\"conn\"></span><div class=\"brand\">Chrono DH <b>Départ</b></div>\n"
  "<div class=\"pills\"><span class=\"pill\" id=\"pLink\">Arrivée</span><span class=\"pill\" id=\"pSync\">Synchro</span><span class=\"pill\" id=\"pBat\">Départ</span><span class=\"pill\" id=\"pBatF\">Arrivée</span><span class=\"pill\" id=\"pBatE\">Émetteur</span><span class=\"pill hide\" id=\"pRelay\">Relais</span></div></div></header>\n"
  "<div class=\"wrap\">\n"
  "<div class=\"card\"><div class=\"row\" style=\"align-items:flex-start\">\n"
  "<div style=\"flex:3 1 260px\"><div class=\"lbl\">Prochain pilote</div><div class=\"mid\" id=\"nxName\" style=\"margin-top:6px\">--</div><div class=\"mut\" id=\"nxInfo\" style=\"margin-top:4px\"></div></div>\n"
  "<div style=\"flex:1 0 120px;text-align:right\"><div class=\"lbl\">Baguette</div><div class=\"mid\" id=\"armed\" style=\"margin-top:6px\">--</div></div></div>\n"
  "<div class=\"row\" style=\"margin-top:14px\"><button onclick=\"act('/api/next/step?d=-1')\">Précédent</button><button onclick=\"act('/api/next/step?d=1')\">Suivant</button>\n"
  "<select id=\"nxSel\" onchange=\"if(this.value)act('/api/next?id='+this.value,'Prochain pilote changé');this.value=''\"></select></div></div>\n"
  "<div class=\"card\"><div class=\"lbl\">Capteur de la baguette</div>\n"
  "<div class=\"row\" style=\"align-items:flex-end;margin-top:6px\"><div style=\"flex:1 1 150px\"><div class=\"lbl\">Aimant</div><div class=\"mid\" id=\"hState\">--</div></div><div style=\"flex:1 1 90px\"><div class=\"lbl\">Détections</div><div class=\"mid n\" id=\"hCnt\">-</div></div></div>\n"
  "<div class=\"mut\" id=\"hInfo\" style=\"font-size:13px;margin:8px 0 10px\"></div>\n"
  "<div class=\"row\"><button id=\"hBtn\" onclick=\"act('/api/halltune?on='+(st.tune?0:1))\">Mode réglage</button><button onclick=\"act('/api/halltune/reset','Compteur remis à zéro')\">Remettre à zéro</button></div></div>\n"
  "<div class=\"card\"><div class=\"lbl\">Derniers départs</div><div class=\"scroll\"><table><thead><tr><th>Heure</th><th>Pilote</th><th>État</th><th class=\"r\">Temps</th><th></th></tr></thead><tbody id=\"starts\"></tbody></table></div></div>\n"
  "<div class=\"card\"><div class=\"lbl\">Coureurs <span id=\"rc\"></span></div><div class=\"scroll\"><table><thead><tr><th>#</th><th>Dos.</th><th>Pilote</th><th>Cat.</th><th class=\"r\">Manches</th><th></th></tr></thead><tbody id=\"riders\"></tbody></table></div></div>\n"
  "<div class=\"card\"><div class=\"lbl\" id=\"fTitle\">Ajouter un coureur</div>\n"
  "<div class=\"row\" style=\"margin-top:10px\"><label>Dossard<input id=\"fBib\" inputmode=\"numeric\"></label><label style=\"flex:2 1 200px\">Prénom et nom<input id=\"fName\"></label><label>Catégorie<input id=\"fCat\"></label></div>\n"
  "<div class=\"row\" style=\"margin-top:10px\"><button class=\"pri\" id=\"fBtn\" onclick=\"saveRider()\">Ajouter</button><button onclick=\"resetForm()\">Vider</button></div>\n"
  "<div class=\"mut\" style=\"margin-top:8px;font-size:12px\">Tous les champs sont facultatifs.</div></div>\n"
  "<div class=\"card\"><div class=\"lbl\">Importer un CSV</div>\n"
  "<div class=\"mut\" style=\"font-size:13px;margin:6px 0 10px\">Colonnes reconnues : dossard, prénom, nom (ou pilote), catégorie. Séparateur ; ou , détecté automatiquement. Depuis Excel, enregistre en \"CSV UTF-8\".</div>\n"
  "<div class=\"row\"><input type=\"file\" id=\"csv\" accept=\".csv,.txt,text/csv\"><select id=\"csvMode\"><option value=\"replace\">Remplacer la liste</option><option value=\"append\">Ajouter à la liste</option></select><button class=\"pri\" onclick=\"importCsv()\">Importer</button></div>\n"
  "<div id=\"csvInfo\" class=\"mut\" style=\"margin-top:8px\"></div></div>\n"
  "<div class=\"card\"><div class=\"lbl\">Fichiers et système</div>\n"
  "<div class=\"row\" style=\"margin-top:10px\"><a class=\"btn\" href=\"/api/riders.csv\">Exporter les coureurs</a><a class=\"btn\" href=\"/api/starts.csv\">Journal des départs</a><button class=\"dng\" onclick=\"if(confirm('Effacer toute la liste des coureurs ?'))act('/api/riders/clear','Liste effacée')\">Effacer la liste</button><a class=\"btn\" href=\"/update\">Mise à jour</a></div>\n"
  "<div class=\"lbl\" style=\"margin-top:16px\">Archives</div><table><tbody id=\"arch\"></tbody></table>\n"
  "<div class=\"mut\" id=\"foot\" style=\"margin-top:12px;font-size:12px\"></div></div>\n"
  "</div><div class=\"toast\" id=\"toast\"></div>\n"
  "<script src=\"/c.js\"></script><script>\n"
  "let st=null,riders=[],rRev=-1,editId=0;\n"
  "async function refresh(force){try{st=await api('/api/state');$('#conn').className='dot ok';if(force||st.rrev!==rRev){riders=await api('/api/riders');rRev=st.rrev;drawRiders();}draw();}catch(e){$('#conn').className='dot bad';}}\n"
  "function draw(){const L=st.link;\n"
  "const vias=[L.via&1?'direct':'',L.via&2?'relais':'',L.via&4?'LoRa':''].filter(Boolean);\n"
  "pill('pLink',L.ok?'Arrivée OK'+(vias.length?' ('+vias.join(' + ')+')':''):'Arrivée absente',L.ok?'ok':'bad');\n"
  "if(L.rlseen)pill('pRelay',L.rl?'Relais '+batt(L.rlbat):'Relais absent',L.rl?(battCls(L.rlbat)||'ok'):'bad');\n"
  "pill('pSync',!L.ok?'Synchro --':(L.syncok?'Synchro '+(L.u/1000).toFixed(2)+' ms':'Synchro en cours'),!L.ok?'':(L.syncok?'ok':'warn'));\n"
  "pill('pBat','Départ '+batt(st.bat),battCls(st.bat));\n"
  "pill('pBatF','Arrivée '+(L.ok?batt(L.bat):'--'),L.ok?battCls(L.bat):'');\n"
  "pill('pBatE','Émetteur '+(L.ebat?batt(L.ebat):'--'),battCls(L.ebat));\n"
  "const n=st.next;\n"
  "$('#nxName').textContent=st.bench?'Mode banc':n?(n.nm||(n.bib?'Dossard '+n.bib:'Sans nom')):'Pilote anonyme';\n"
  "$('#nxInfo').textContent=st.bench?'Activé depuis l\\'arrivée : les départs ne comptent pas':n?[n.bib?'Dossard '+n.bib:'',n.ct,'Manche '+(n.runs+1)].filter(Boolean).join('  /  '):'Ajoute des coureurs ci-dessous';\n"
  "const a=$('#armed');a.textContent=st.tune?'RÉGLAGE':st.armed?'PRÊTE':'ATTENTE';a.className='mid '+(st.tune?'warn':st.armed?'ok':'warn');\n"
  "const h=$('#hState');h.textContent=st.hall?'DÉTECTÉ':'RIEN';h.className='mid '+(st.hall?'ok':'mut');$('#hCnt').textContent=st.tune?st.tcnt:'-';\n"
  "$('#hInfo').textContent=st.tune?'Mode réglage actif : le buzzer sonne tant que l\\'aimant est détecté et aucune poussée ne crée de départ. Arrêt automatique après 10 min.':'Le mode réglage fait sonner le buzzer tant que l\\'aimant est vu, sans créer de départ : pratique pour trouver le point de déclenchement à la main.';\n"
  "const hb=$('#hBtn');hb.textContent=st.tune?'Quitter le mode réglage':'Mode réglage';hb.className=st.tune?'pri':'';\n"
  "$('#starts').innerHTML=st.starts.map(s=>{const state=['Envoi...','Reçu','Annulation...','Annulé'][s.st];const cls=s.st===1?'ok':s.st===3?'mut':'warn';\n"
  "const res=s.fs===1&&s.d>=0?fmtDur(s.d):s.fs===2?'DNF':s.fs===3?'annulé':(s.st===1?'en piste':'');\n"
  "const can=(s.st<2&&s.age>=0&&s.age<st.cw)?`<button class=\"sm dng\" onclick=\"if(confirm('Annuler ce départ ?'))act('/api/cancel?sb=${s.sb}&sq=${s.sq}','Départ annulé')\">Annuler</button>`:'';\n"
  "return `<tr><td class=\"n\">${fmtClock(s.w)}</td><td>${esc(s.nm||(s.bib?'Dossard '+s.bib:'Anonyme'))}${s.fl&1?'<span class=\"badge\">banc</span>':''}${s.fl&2?'<span class=\"badge w\">flash</span>':''}</td><td class=\"${cls}\">${state}</td><td class=\"r n\" style=\"font-weight:800\">${res}</td><td class=\"r\">${can}</td></tr>`;}).join('')||'<tr><td colspan=\"5\" class=\"mut\">Aucun départ</td></tr>';\n"
  "$('#foot').textContent='Firmware '+st.fw+'  /  session '+st.sess+'  /  '+(st.pend?st.pend+' message(s) en attente':'file d\\'envoi vide');}\n"
  "function drawRiders(){$('#rc').textContent='('+riders.length+')';const nid=st&&st.next?st.next.id:0;\n"
  "$('#riders').innerHTML=riders.map((r,i)=>`<tr class=\"${r.id===nid?'hl':''}\"><td class=\"mut n\">${i+1}</td><td class=\"n\">${r.bib||''}</td><td style=\"font-weight:700\">${esc(r.nm)}${r.id===nid?'<span class=\"badge w\">suivant</span>':''}</td><td class=\"mut\">${esc(r.ct)}</td><td class=\"r n\">${r.runs}</td><td class=\"r\"><select class=\"sm\" style=\"width:auto\" onchange=\"riderAct(${r.id},this.value);this.value=''\"><option value=\"\">Actions</option><option value=\"next\">Mettre en suivant</option><option value=\"edit\">Modifier</option><option value=\"up\">Monter</option><option value=\"down\">Descendre</option><option value=\"del\">Supprimer</option></select></td></tr>`).join('')||'<tr><td colspan=\"6\" class=\"mut\">Aucun coureur : ajoute-les ou importe un CSV.</td></tr>';\n"
  "$('#nxSel').innerHTML='<option value=\"\">Choisir le prochain...</option>'+riders.map(r=>`<option value=\"${r.id}\">${esc((r.bib?r.bib+' - ':'')+(r.nm||'Sans nom'))}</option>`).join('');}\n"
  "function riderAct(id,a){if(a==='next')act('/api/next?id='+id,'Prochain pilote changé');else if(a==='edit')edit(id);else if(a==='up')act('/api/rider/move?id='+id+'&d=-1');else if(a==='down')act('/api/rider/move?id='+id+'&d=1');else if(a==='del'&&confirm('Supprimer ce coureur ?'))act('/api/rider/del?id='+id,'Coureur supprimé');}\n"
  "function edit(id){const r=riders.find(x=>x.id===id);if(!r)return;editId=id;$('#fBib').value=r.bib||'';$('#fName').value=r.nm;$('#fCat').value=r.ct;$('#fTitle').textContent='Modifier le coureur';$('#fBtn').textContent='Enregistrer';window.scrollTo({top:$('#fTitle').getBoundingClientRect().top+scrollY-90,behavior:'smooth'});}\n"
  "function resetForm(){editId=0;['#fBib','#fName','#fCat'].forEach(s=>$(s).value='');$('#fTitle').textContent='Ajouter un coureur';$('#fBtn').textContent='Ajouter';}\n"
  "function saveRider(){const q='id='+editId+'&bib='+encodeURIComponent($('#fBib').value.replace(/\\D/g,''))+'&name='+encodeURIComponent($('#fName').value.trim())+'&cat='+encodeURIComponent($('#fCat').value.trim());act('/api/rider/save?'+q,editId?'Coureur modifié':'Coureur ajouté').then(resetForm);}\n"
  "function splitCsv(line,sep){const out=[];let cur='',q=false;for(let i=0;i<line.length;i++){const c=line[i];if(q){if(c==='\"'){if(line[i+1]==='\"'){cur+='\"';i++;}else q=false;}else cur+=c;}else if(c==='\"')q=true;else if(c===sep){out.push(cur);cur='';}else cur+=c;}out.push(cur);return out;}\n"
  "function parseCsv(text){text=text.replace(/^\\uFEFF/,'');const lines=text.split(/\\r?\\n/).filter(l=>l.trim().length);if(!lines.length)return[];\n"
  "const sep=[';',',','\\t'].map(s=>[s,lines[0].split(s).length]).sort((a,b)=>b[1]-a[1])[0][0];let rows=lines.map(l=>splitCsv(l,sep));\n"
  "const norm=s=>s.toLowerCase().normalize('NFD').replace(/[\\u0300-\\u036f]/g,'').replace(/[^a-z ]/g,'').trim();\n"
  "const idx={bib:-1,first:-1,last:-1,name:-1,cat:-1};\n"
  "rows[0].map(norm).forEach((c,i)=>{if(/^(dossard|bib|num|numero|no|n|plaque)$/.test(c))idx.bib=i;else if(/^(prenom|firstname|first name)$/.test(c))idx.first=i;else if(/^(nom|lastname|last name|surname|nom de famille)$/.test(c))idx.last=i;else if(/^(pilote|coureur|name|nom complet|rider|participant)$/.test(c))idx.name=i;else if(/^(categorie|cat|category|classe)$/.test(c))idx.cat=i;});\n"
  "if(Object.values(idx).some(v=>v>=0))rows=rows.slice(1);else{const n=rows[0].length;if(n>=4){idx.bib=0;idx.first=1;idx.last=2;idx.cat=3;}else if(n===3){idx.bib=0;idx.name=1;idx.cat=2;}else if(n===2){idx.bib=0;idx.name=1;}else idx.name=0;}\n"
  "return rows.map(r=>{const g=i=>i>=0&&i<r.length?r[i].trim():'';let name=g(idx.name);if(!name)name=(g(idx.first)+' '+g(idx.last)).trim();return{bib:g(idx.bib).replace(/\\D/g,''),name,cat:g(idx.cat)};}).filter(x=>x.bib||x.name||x.cat);}\n"
  "function sendCsv(text){const rows=parseCsv(text);if(!rows.length){toast('Aucun coureur trouvé');return;}const clean=s=>String(s).replace(/[\\t\\r\\n;\"\\\\]/g,' ').trim();\n"
  "const body=rows.map(r=>[clean(r.bib),clean(r.name),clean(r.cat)].join('\\t')).join('\\n');\n"
  "post('/api/riders/import?mode='+$('#csvMode').value,body).then(t=>{$('#csvInfo').textContent=t;toast('Import terminé');refresh(true);}).catch(e=>toast('Erreur : '+e.message));}\n"
  "function importCsv(){const f=$('#csv').files[0];if(!f){toast('Choisis un fichier');return;}const rd=new FileReader();\n"
  "rd.onload=()=>{if(rd.result.includes('\\uFFFD')){const r2=new FileReader();r2.onload=()=>sendCsv(r2.result);r2.readAsText(f,'windows-1252');}else sendCsv(rd.result);};rd.readAsText(f,'utf-8');}\n"
  "refresh(true).finally(function loop(){setTimeout(()=>refresh(false).finally(loop),st&&st.tune?250:1000);});loadArch();\n"
  "</script></body></html>";

static const char PAGE_FINISH[] PROGMEM = 
  "<!doctype html><html lang=\"fr\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><meta name=\"theme-color\" content=\"#060708\"><title>Chrono DH - Arrivée</title><link rel=\"stylesheet\" href=\"/s.css\"></head><body>\n"
  "<header><div class=\"hd\"><span class=\"dot\" id=\"conn\"></span><div class=\"brand\">Chrono DH <b>Arrivée</b></div>\n"
  "<div class=\"pills\"><span class=\"pill\" id=\"pStart\">Départ</span><span class=\"pill\" id=\"pSync\">Synchro</span><span class=\"pill\" id=\"pBeam\">Faisceau</span><span class=\"pill\" id=\"pBat\">Arrivée</span><span class=\"pill\" id=\"pBatS\">Départ</span><span class=\"pill\" id=\"pBatE\">Émetteur</span></div></div></header>\n"
  "<div class=\"wrap\">\n"
  "<div class=\"card\"><div class=\"lbl\">Dernière arrivée</div><div class=\"mid\" id=\"lName\" style=\"margin-top:6px\">En attente</div><div class=\"big acc\" id=\"lTime\">--</div><div class=\"mut\" id=\"lInfo\"></div></div>\n"
  "<div class=\"card\"><div class=\"lbl\">En piste <span id=\"ocN\"></span></div><table><tbody id=\"oc\"></tbody></table></div>\n"
  "<div class=\"tabs\"><button data-t=\"rank\" class=\"on\">Classement</button><button data-t=\"runs\">Passages</button><button data-t=\"set\">Réglages</button><button data-t=\"sys\">Système</button></div>\n"
  "<div id=\"t-rank\" class=\"card\"><div class=\"row\" id=\"cats\" style=\"gap:6px\"></div><div class=\"scroll\" style=\"margin-top:12px\"><table><thead id=\"rkH\"></thead><tbody id=\"rk\"></tbody></table></div>\n"
  "<div class=\"row\" style=\"margin-top:12px\"><button onclick=\"exportRank()\">Exporter le classement (CSV)</button></div></div>\n"
  "<div id=\"t-runs\" class=\"card hide\"><div class=\"lbl\">Passages (le plus récent en haut)</div><div class=\"scroll\"><table><thead><tr><th>N</th><th>Départ</th><th>Pilote</th><th class=\"r\">Temps</th><th>Statut</th><th></th></tr></thead><tbody id=\"runs\"></tbody></table></div>\n"
  "<div id=\"orphCard\" class=\"hide\"><hr><div class=\"lbl\">Passages sans départ (orphelins)</div><div class=\"mut\" style=\"font-size:13px;margin:6px 0\">Un passage a été vu à l'arrivée sans départ correspondant. Attribue-le à un pilote en piste.</div><table><tbody id=\"orph\"></tbody></table></div></div>\n"
  "<div id=\"t-set\" class=\"card hide\"><div class=\"lbl\">Course</div>\n"
  "<div class=\"row\" style=\"margin-top:10px\"><label>Manches au cumul<input id=\"sN\" type=\"number\" min=\"1\" max=\"9\"></label><label>Temps mini (s)<input id=\"sMin\" type=\"number\" min=\"0\"></label><label>DNF auto après (s)<input id=\"sMax\" type=\"number\" min=\"10\"></label><label>Bips à l'arrivée<select id=\"sBeep\"><option value=\"1\">Oui</option><option value=\"0\">Non</option></select></label></div>\n"
  "<div class=\"row\" style=\"margin-top:10px\"><button class=\"pri\" onclick=\"saveSet()\">Enregistrer</button></div>\n"
  "<hr><div class=\"lbl\">Test de la barrière</div><div class=\"mut\" style=\"font-size:13px;margin:6px 0 10px\">Passe devant la barrière : l'état doit passer à COUPÉ et le compteur augmenter, même sans départ. Si le signal reste à 100 % quand tu coupes le faisceau, l'émetteur est trop puissant : lance le réglage auto juste en dessous.</div>\n"
  "<div class=\"row\" style=\"align-items:flex-end\"><div style=\"flex:1 1 120px\"><div class=\"lbl\">État</div><div class=\"mid\" id=\"btState\">--</div></div><div style=\"flex:1 1 90px\"><div class=\"lbl\">Signal</div><div class=\"mid n\" id=\"btQ\">--</div></div><div style=\"flex:1 1 90px\"><div class=\"lbl\">Coupures</div><div class=\"mid n\" id=\"btN\">0</div></div></div>\n"
  "<div class=\"mut\" id=\"btLast\" style=\"margin-top:8px\"></div>\n"
  "<div class=\"row\" style=\"margin-top:10px\"><button onclick=\"act('/api/beamtest/reset','Compteur remis à zéro')\">Remettre le compteur à zéro</button></div>\n"
  "<hr><div class=\"lbl\">Puissance de l'émetteur</div><div class=\"mut\" style=\"font-size:13px;margin:6px 0 10px\">Trop de puissance et l'infrarouge contourne le pilote en rebondissant sur le sol, les murs ou le corps : la coupure n'est pas vue. Le réglage auto cherche la puissance minimale qui donne un faisceau propre, puis ajoute une marge. Lance-le avec l'émetteur à sa place définitive, aligné, et personne dans le faisceau (15 s au plus). Refais-le à chaque changement de distance.</div>\n"
  "<div class=\"row\" style=\"align-items:flex-end\"><div style=\"flex:0 0 auto;min-width:90px\"><div class=\"lbl\">Niveau</div><div class=\"mid n\" id=\"pwCur\">--</div></div><button class=\"pri\" id=\"calBtn\" onclick=\"act('/api/ircal','Réglage auto lancé')\">Réglage auto</button><select id=\"pwSel\" onchange=\"if(this.value)act('/api/irpower?l='+this.value,'Niveau envoyé');this.value=''\"><option value=\"\">Niveau manuel...</option><option value=\"1\">1 (le plus faible)</option><option value=\"2\">2</option><option value=\"3\">3</option><option value=\"4\">4</option><option value=\"5\">5</option><option value=\"6\">6</option><option value=\"7\">7</option><option value=\"8\">8</option><option value=\"9\">9</option><option value=\"10\">10 (pleine puissance)</option></select></div>\n"
  "<div class=\"mut\" id=\"calMsg\" style=\"margin-top:8px\"></div>\n"
  "<hr><div class=\"lbl\">Alignement de la barrière</div><div class=\"mut\" style=\"font-size:13px;margin:6px 0 10px\">L'émetteur et l'arrivée bipent de plus en plus vite quand le faisceau est bien reçu ; son presque continu = aligné. Arrêt automatique après 5 min.</div>\n"
  "<div class=\"row\" style=\"align-items:center\"><button id=\"alBtn\" onclick=\"act('/api/align?on='+(st.beam.align?0:1))\">Alignement</button><div class=\"mid n\" id=\"alQ\" style=\"flex:0 0 auto\"></div></div>\n"
  "<hr><div class=\"lbl\">Mode banc (précision)</div><div class=\"mut\" style=\"font-size:13px;margin:6px 0 10px\">Relie un même bouton sur TRIG_IN des deux boîtiers (départ et arrivée). Chaque appui mesure l'écart entre les deux horloges synchronisées. Pendant le banc, la barrière IR est ignorée.</div>\n"
  "<div class=\"row\"><button id=\"bBtn\" onclick=\"act('/api/bench?on='+(st.set.bench?0:1))\">Banc</button><button onclick=\"act('/api/bench/reset','Statistiques remises à zéro')\">Remettre à zéro</button></div>\n"
  "<table style=\"margin-top:10px\"><tbody id=\"bSt\"></tbody></table></div>\n"
  "<div id=\"t-sys\" class=\"card hide\"><div class=\"lbl\">Boîtiers</div><div class=\"scroll\"><table style=\"margin-top:6px\"><thead><tr><th>Boîtier</th><th>État</th><th>Batterie</th><th>Radio</th><th>Firmware</th><th></th></tr></thead><tbody id=\"boxes\"></tbody></table></div>\n"
  "<div class=\"mut\" id=\"syncInfo\" style=\"font-size:12px;margin-top:8px\"></div>\n"
  "<hr><div class=\"lbl\">Liaison radio <span class=\"badge\" id=\"lrMode\"></span></div><div class=\"mut\" id=\"loraInfo\" style=\"font-size:12px;margin-top:6px\"></div>\n"
  "<div class=\"scroll\"><table style=\"margin-top:6px\"><thead><tr><th>Liaison</th><th class=\"r\">Direct</th><th class=\"r\">Relais</th><th class=\"r\">LoRa</th><th class=\"r\">Total</th><th class=\"r\">Wi-Fi</th><th class=\"r\">LoRa</th></tr></thead><tbody id=\"links\"></tbody></table></div>\n"
  "<div class=\"mut\" style=\"font-size:12px;margin-top:8px\">Part des messages de vie (un par seconde) reçus sur les 10 dernières secondes. Total : reçu par au moins un chemin, visez 100 %. Signal Wi-Fi : au-dessus de -80 dBm confortable, de -80 à -95 dBm limite. Signal LoRa : au-dessus de -110 dBm confortable, jusqu'à -120 dBm correct en SF7. Le LoRa n'envoie un message de vie que toutes les 5 s (en SF7) : son pourcentage porte sur 30 s. Pour placer le relais, cherche l'endroit où ses deux liaisons dépassent 90 %.</div>\n"
  "<hr><div class=\"lbl\">Session <span id=\"sessN\"></span></div><div class=\"row\" style=\"margin-top:10px\"><a class=\"btn\" href=\"/api/export.csv\">Exporter les passages (CSV)</a><button onclick=\"newSession()\">Nouvelle session (archive)</button></div>\n"
  "<div class=\"row\" style=\"margin-top:10px\"><button class=\"dng\" onclick=\"wipe(0)\">Effacer les passages</button><button class=\"dng\" onclick=\"wipe(1)\">Tout effacer, archives comprises</button></div>\n"
  "<div class=\"mut\" style=\"font-size:12px;margin-top:8px\">Effacer les passages : la session en cours est supprimée sans archive et les manches des coureurs repartent à zéro au départ. Tout effacer : supprime en plus toutes les archives, sur l'arrivée et sur le départ. Dans les deux cas, la liste des coureurs est conservée.</div>\n"
  "<div class=\"lbl\" style=\"margin-top:16px\">Archives</div><table><tbody id=\"arch\"></tbody></table>\n"
  "<hr><div class=\"lbl\">Mise à jour</div><div class=\"mut\" style=\"font-size:13px;margin:6px 0 10px\">Ce boîtier apparaît dans Arduino IDE comme port réseau \"chrono-arrivee\" quand ton ordinateur est sur ce Wi-Fi. Départ et émetteur : bouton Maintenance ci-dessus (ou bouton maintenu à l'allumage), ils rejoignent ce Wi-Fi et apparaissent à leur tour.</div>\n"
  "<div class=\"row\"><a class=\"btn\" href=\"/update\">Mettre à jour l'arrivée</a></div></div>\n"
  "</div><div class=\"toast\" id=\"toast\"></div>\n"
  "<script src=\"/c.js\"></script><script>\n"
  "let st=null,data={runs:[],orph:[],best:-1},rev=-1,cat='',tab='rank',fetchT=0,setFilled=false;\n"
  "document.querySelectorAll('.tabs button').forEach(b=>b.onclick=()=>{tab=b.dataset.t;document.querySelectorAll('.tabs button').forEach(x=>x.classList.toggle('on',x===b));['rank','runs','set','sys'].forEach(t=>$('#t-'+t).classList.toggle('hide',t!==tab));if(tab==='sys')loadArch();drawData();if(st)drawState();});\n"
  "async function refresh(force){try{st=await api('/api/state');fetchT=performance.now();$('#conn').className='dot ok';if(force||st.rev!==rev){data=await api('/api/runs');rev=st.rev;drawData();}drawState();}catch(e){$('#conn').className='dot bad';}}\n"
  "const nm=r=>r.nm||(r.bib?'Dossard '+r.bib:'Pilote '+(r.i+1));\n"
  "const key=r=>r.rid>0?'r'+r.rid:'a'+r.i;\n"
  "function buildRank(){const N=st?st.set.n:2,map=new Map();\n"
  "for(const r of data.runs){if(r.st===3)continue;const k=key(r);let e=map.get(k);if(!e){e={k,bib:r.bib,nm:nm(r),ct:r.ct,runs:[]};map.set(k,e);}if(r.nm)e.nm=r.nm;if(r.bib)e.bib=r.bib;if(r.ct)e.ct=r.ct;e.runs.push(r);}\n"
  "const rows=[];for(const e of map.values()){const f=e.runs.slice(0,N);let tot=0,done=0,dnf=false;for(const r of f){if(r.st===1&&r.d>=0){tot+=r.d;done++;}else if(r.st===2)dnf=true;}Object.assign(e,{f,tot,done,dnf,full:done===N});rows.push(e);}\n"
  "rows.sort((a,b)=>(b.full-a.full)||(a.dnf-b.dnf)||(b.done-a.done)||(a.tot-b.tot));return rows;}\n"
  "function drawRank(){const N=st?st.set.n:2,all=buildRank(),cats=[...new Set(all.map(e=>e.ct).filter(Boolean))].sort();\n"
  "$('#cats').innerHTML=['',...cats].map(c=>`<button class=\"sm ${c===cat?'pri':''}\" style=\"flex:0 0 auto\" data-c=\"${esc(c)}\" onclick=\"cat=this.dataset.c;drawRank()\">${c?esc(c):'Toutes'}</button>`).join('');\n"
  "const rows=all.filter(e=>!cat||e.ct===cat);let lead=null,pos=0;\n"
  "$('#rkH').innerHTML='<tr><th>#</th><th>Dos.</th><th>Pilote</th><th>Cat.</th>'+Array.from({length:N},(_,i)=>`<th class=\"r\">M${i+1}</th>`).join('')+'<th class=\"r\">Total</th><th class=\"r\">Écart</th></tr>';\n"
  "$('#rk').innerHTML=rows.map(e=>{const rk=e.full?++pos:'';if(e.full&&lead==null)lead=e.tot;\n"
  "const cells=Array.from({length:N},(_,i)=>{const r=e.f[i];if(!r)return'<td class=\"r mut\">-</td>';if(r.st===2)return'<td class=\"r bad\">DNF</td>';if(r.st===0)return'<td class=\"r warn\">piste</td>';if(r.d<0)return'<td class=\"r warn\">?</td>';return`<td class=\"r n\">${fmtDur(r.d)}</td>`;}).join('');\n"
  "return `<tr><td class=\"n\" style=\"font-weight:900\">${rk}</td><td class=\"n\">${e.bib||''}</td><td style=\"font-weight:700\">${esc(e.nm)}</td><td class=\"mut\">${esc(e.ct||'')}</td>${cells}<td class=\"r n\" style=\"font-weight:900\">${e.full?fmtDur(e.tot):(e.done?'('+fmtDur(e.tot)+')':'--')}</td><td class=\"r n mut\">${e.full&&lead!=null&&e.tot>lead?fmtGap(e.tot-lead):''}</td></tr>`;}).join('')||`<tr><td colspan=\"${N+6}\" class=\"mut\">Aucun résultat</td></tr>`;}\n"
  "function exportRank(){const N=st.set.n,rows=buildRank().filter(e=>!cat||e.ct===cat);let pos=0,lead=null;\n"
  "const lines=['rang;dossard;pilote;categorie;'+Array.from({length:N},(_,i)=>'manche'+(i+1)).join(';')+';total;ecart'];\n"
  "for(const e of rows){const rk=e.full?++pos:'';if(e.full&&lead==null)lead=e.tot;lines.push([rk,e.bib||'',e.nm,e.ct||'',...Array.from({length:N},(_,i)=>{const r=e.f[i];return !r?'':r.st===2?'DNF':r.st===0?'piste':fmtDur(r.d);}),e.full?fmtDur(e.tot):'',e.full&&e.tot>lead?fmtGap(e.tot-lead):''].join(';'));}\n"
  "const a=document.createElement('a');a.href=URL.createObjectURL(new Blob(['\\uFEFF'+lines.join('\\n')],{type:'text/csv'}));a.download='classement'+(cat?'_'+cat:'')+'.csv';a.click();}\n"
  "function flags(f){let s='';if(f&1)s+='<span class=\"badge w\">flash</span>';if(f&2)s+='<span class=\"badge w\">synchro</span>';if(f&4)s+='<span class=\"badge\">départ tardif</span>';if(f&8)s+='<span class=\"badge w\">sans synchro</span>';if(f&16)s+='<span class=\"badge\">manuel</span>';if(f&32)s+='<span class=\"badge\">DNF auto</span>';if(f&64)s+='<span class=\"badge w\">flash départ</span>';return s;}\n"
  "function drawRuns(){const cnt=new Map();const list=data.runs.map(r=>{let m=0;if(r.st!==3){m=(cnt.get(key(r))||0)+1;cnt.set(key(r),m);}return Object.assign({m},r);}).reverse();\n"
  "$('#runs').innerHTML=list.map(r=>{const stT=['En piste','Arrivé','DNF','Annulé'][r.st];const cls=['warn','ok','bad','mut'][r.st];\n"
  "const opts=(r.st!==2?'<option value=\"dnf\">Déclarer DNF</option>':'')+(r.st>=2?'<option value=\"restore\">Rétablir</option>':'')+(r.st!==3?'<option value=\"cancel\">Annuler la manche</option>':'')+(r.st===1?'<option value=\"clear\">Retirer l\\'arrivée</option><option value=\"swap\">Échanger le temps avec...</option>':'');\n"
  "return `<tr><td class=\"mut n\">${r.i+1}</td><td class=\"n\">${fmtClock(r.w)}</td><td style=\"font-weight:700\">${esc(nm(r))}${r.m?' <span class=\"mut\">M'+r.m+'</span>':''}</td><td class=\"r n\" style=\"font-weight:800\">${r.st===1?(r.d>=0?fmtDur(r.d):'?'):''}</td><td class=\"${cls}\">${stT}${flags(r.fl)}</td><td class=\"r\"><select class=\"sm\" style=\"width:auto\" onchange=\"runAct(${r.i},this.value);this.value=''\"><option value=\"\">Actions</option>${opts}</select></td></tr>`;}).join('')||'<tr><td colspan=\"6\" class=\"mut\">Aucun passage</td></tr>';\n"
  "const onc=data.runs.filter(r=>r.st===0);$('#orphCard').classList.toggle('hide',!data.orph.length);\n"
  "$('#orph').innerHTML=data.orph.map(o=>`<tr><td class=\"n\">${fmtClock(o.w)}</td><td>Passage${flags(o.fl)}</td><td class=\"r\" style=\"white-space:nowrap\"><select class=\"sm\" style=\"width:auto\" onchange=\"if(this.value)act('/api/orphan?id=${o.id}&i='+this.value,'Passage attribué');this.value=''\"><option value=\"\">Attribuer à</option>${onc.map(r=>`<option value=\"${r.i}\">${esc(nm(r))}</option>`).join('')}</select> <button class=\"sm dng\" onclick=\"act('/api/orphan?id=${o.id}&del=1','Supprimé')\">Suppr.</button></td></tr>`).join('');}\n"
  "function runAct(i,a){if(!a)return;if(a==='swap'){const lst=data.runs.filter(r=>r.st===1&&r.i!==i).map(r=>(r.i+1)+' : '+nm(r)+' '+fmtDur(r.d)).join('\\n');const v=prompt('Échanger avec quel passage ? (numéro N)\\n'+lst);if(!v)return;act('/api/run?i='+i+'&a=swap&j='+(parseInt(v,10)-1),'Temps échangés');return;}\n"
  "if(a!=='restore'&&!confirm('Confirmer ?'))return;act('/api/run?i='+i+'&a='+a,'Fait');}\n"
  "function drawLast(){const r=data.runs.find(x=>x.i===st.last);if(!r){$('#lName').textContent='En attente';$('#lTime').textContent='--';$('#lInfo').textContent='';return;}\n"
  "$('#lName').textContent=nm(r);$('#lTime').textContent=r.st===1?fmtDur(r.d):(['--','--','DNF','Annulé'][r.st]);\n"
  "const rows=buildRank(),k=key(r);let pos=0,rk=0,lead=null,e=null;for(const x of rows){if(x.full){pos++;if(lead==null)lead=x.tot;}if(x.k===k){e=x;if(x.full)rk=pos;break;}}\n"
  "const parts=[r.ct||''];if(e){const m=e.runs.findIndex(x=>x.i===r.i)+1;if(m)parts.push('Manche '+m);if(e.full)parts.push('Cumul '+fmtDur(e.tot)+' : '+rk+(rk===1?'er':'e')+(rk>1?' '+fmtGap(e.tot-lead):''));}\n"
  "if(r.st===1&&r.d>=0&&r.d===data.best)parts.push('Meilleure manche');$('#lInfo').textContent=parts.filter(Boolean).join('  /  ');}\n"
  "function drawOnCourse(){const oc=st.oc||[],dt=(performance.now()-fetchT)*1000;$('#ocN').textContent='('+oc.length+')';\n"
  "$('#oc').innerHTML=oc.map(o=>{const r=data.runs.find(x=>x.i===o.i)||{i:o.i};const el=o.ts==null?null:(st.now-o.ts)+dt;\n"
  "return `<tr><td style=\"font-weight:700\">${esc(nm(r))}</td><td class=\"mut\">${esc(r.ct||'')}</td><td class=\"r n\" style=\"font-weight:900;font-size:22px\">${el==null?'?':fmtLive(el)}</td><td class=\"r\"><button class=\"sm dng\" onclick=\"if(confirm('Déclarer DNF ?'))act('/api/run?i=${o.i}&a=dnf','DNF')\">DNF</button></td></tr>`;}).join('')||'<tr><td class=\"mut\">Personne en piste</td></tr>';}\n"
  "function boxRow(name,b,t){const ok=b.ok,stt=b.maint?'<span class=\"warn\">Maintenance</span>':ok?'<span class=\"ok\">OK</span>':'<span class=\"bad\">Absent</span>';\n"
  "const up=b.maint&&b.ip?` <a class=\"btn sm\" href=\"http://${ipStr(b.ip)}/update\" target=\"_blank\">Mettre à jour</a>`:'';\n"
  "const btns=t?`<button class=\"sm\" onclick=\"act('/api/locate?t=${t}','Bip envoyé')\">Localiser</button> <button class=\"sm\" onclick=\"if(confirm('Passer ce boîtier en maintenance ? Il ne chronomètre plus avant le prochain redémarrage.'))act('/api/maint?t=${t}','Ordre envoyé')\">Maintenance</button>${up}`:'';\n"
  "return `<tr><td style=\"font-weight:700\">${name}</td><td>${stt}</td><td class=\"n ${ok||!t?battCls(b.bat):''}\">${ok||!t?batt(b.bat):'--'}</td><td class=\"n\">${t&&ok?b.rssi+' dBm':'-'}</td><td class=\"n mut\">${esc(b.fw||'-')}</td><td class=\"r\" style=\"white-space:nowrap\">${btns}</td></tr>`;}\n"
  "function pctCell(v,ok){if(!ok||v==null||v<0)return'<td class=\"r mut\">-</td>';return `<td class=\"r n ${v>=90?'ok':v>=50?'warn':'bad'}\">${v} %</td>`;}\n"
  "function rssiCell(v,ok){if(!ok||!v)return'<td class=\"r mut\">-</td>';return `<td class=\"r n ${v>-80?'ok':v>-95?'warn':'bad'}\">${v} dBm</td>`;}\n"
  "function lrssiCell(v,ok){if(!ok||!v)return'<td class=\"r mut\">-</td>';return `<td class=\"r n ${v>-110?'ok':v>-120?'warn':'bad'}\">${v} dBm</td>`;}\n"
  "function drawLinks(){const L=st.lk,rl=st.rl&&st.rl.seen,G=L.lora;$('#lrMode').textContent=(L.lr?'Wi-Fi Long Range':'Wi-Fi standard')+(G.en?(G.ok?' + LoRa':' + LoRa absent'):'');\n"
  "$('#loraInfo').textContent=G.en?(G.ok?'LoRa : '+G.mhz.toFixed(3).replace('.',',')+' MHz, SF'+G.sf+', '+G.dbm+' dBm, temps d\\'émission '+G.duty+' % sur la dernière minute (limite 10 %), SNR '+G.snr+' dB.':'LoRa prévu mais module non détecté au démarrage : vérifie le câblage (D9 = MISO).'):'';\n"
  "const rows=[['Départ vers arrivée',L.sf],['Arrivée vers départ',L.fs]];if(rl)rows.push(['Départ vers relais',L.rs],['Relais vers départ',L.sr],['Arrivée vers relais',L.rf],['Relais vers arrivée',L.fr]);\n"
  "$('#links').innerHTML=rows.map(([n,x])=>`<tr><td style=\"font-weight:700\">${n}</td>${pctCell(x.d,x.ok)}${pctCell(x.r,x.ok)}${pctCell(x.l,x.ok)}${pctCell(x.a,x.ok)}${rssiCell(x.s,x.ok)}${lrssiCell(x.ls,x.ok)}</tr>`).join('');}\n"
  "function drawSys(){$('#boxes').innerHTML=boxRow('Arrivée',{ok:true,bat:st.bat,fw:st.fw},'')+boxRow('Départ',st.st,'start')+boxRow('Émetteur',st.em,'emitter')+(st.rl&&st.rl.seen?boxRow('Relais',st.rl,'relay'):'');drawLinks();\n"
  "const Y=st.sync;$('#syncInfo').textContent=Y.st?'Synchro : incertitude '+(Y.u/1000).toFixed(2)+' ms, aller-retour mini '+(Y.rtt/1000).toFixed(2)+' ms, dérive '+Y.ppm.toFixed(2)+' ppm, '+Y.n+' points':'Synchro : en attente du départ';$('#sessN').textContent='n°'+st.sess;}\n"
  "function drawSet(){if(!setFilled){$('#sN').value=st.set.n;$('#sMin').value=st.set.min;$('#sMax').value=st.set.max;$('#sBeep').value=st.set.beep?'1':'0';setFilled=true;}\n"
  "$('#alBtn').textContent=st.beam.align?'Arrêter l\\'alignement':'Lancer l\\'alignement';$('#alBtn').className=st.beam.align?'pri':'';$('#alQ').textContent=st.beam.est||st.beam.q?st.beam.q+' %':'';\n"
  "$('#bBtn').textContent=st.set.bench?'Arrêter le banc':'Activer le banc';$('#bBtn').className=st.set.bench?'pri':'';\n"
  "drawBeamTest();const B=st.bench;$('#bSt').innerHTML=B.n?`<tr><td>Mesures</td><td class=\"r n\">${B.n}</td></tr><tr><td>Écart moyen</td><td class=\"r n\">${B.mean.toFixed(0)} us</td></tr><tr><td>Écart-type</td><td class=\"r n\">${B.sd.toFixed(0)} us</td></tr><tr><td>Min / max</td><td class=\"r n\">${B.min} / ${B.max} us</td></tr><tr><td>Dernière</td><td class=\"r n\">${B.last} us</td></tr>`:'<tr><td class=\"mut\">Aucune mesure</td></tr>';}\n"
  "function fmtAge(ms){return ms<1000?ms+' ms':ms<60000?(ms/1000).toFixed(1)+' s':Math.floor(ms/60000)+' min';}\n"
  "function drawBeamTest(){const B=st.beam,s=$('#btState');\n"
  "if(!B.est){s.textContent='SANS FAISCEAU';s.className='mid bad';}else if(B.br){s.textContent='COUPÉ';s.className='mid warn';}else{s.textContent='LIBRE';s.className='mid ok';}\n"
  "$('#btQ').textContent=B.q+' %';$('#btN').textContent=B.brk;\n"
  "const K=['','passage enregistré','passage sans départ (orphelin, voir Passages)','ignorée : verrouillage de 1,5 s après un passage','ignorée : banc ou réglage en cours'];\n"
  "$('#btLast').textContent=B.lbAge<0?'Aucune coupure depuis la remise à zéro.':'Dernière coupure il y a '+fmtAge(B.lbAge)+(B.lbDur>=0?', durée '+B.lbDur+' ms':', en cours')+' : '+K[B.lbKind]+'.';\n"
  "$('#pwCur').textContent=B.pw?B.pw+'/10':'--';const C=B.cal,cb=$('#calBtn');cb.disabled=C.run;cb.textContent=C.run?'Réglage...':'Réglage auto';\n"
  "$('#calMsg').textContent=C.run?'Réglage en cours : niveau '+C.lvl+'/10, ne passe pas devant la barrière.':C.res===1?'Réglage terminé : faisceau propre dès le niveau '+C.found+', niveau retenu '+C.fin+'. Vérifie maintenant avec le test de coupure.':C.res===2?'Échec : faisceau insuffisant même à pleine puissance. Vérifie l\\'alignement, la distance et que l\\'émetteur est allumé.':C.res===3?'Émetteur absent : allume-le et attends qu\\'il apparaisse dans Système.':(B.pw?'':'Émetteur absent.');}\n"
  "function wipe(all){const msg=all?'TOUT EFFACER : passages de la session et toutes les archives, sur l\\'arrivée et le départ. Tape EFFACER pour confirmer.':'Effacer tous les passages de la session en cours, sans archive ? Tape EFFACER pour confirmer.';\n"
  "const v=prompt(msg);if(v===null)return;if(v.trim().toUpperCase()!=='EFFACER'){toast('Annulé');return;}act('/api/wipe?all='+all,all?'Tout est effacé':'Passages effacés').then(loadArch);}\n"
  "function saveSet(){act('/api/settings?n='+$('#sN').value+'&min='+$('#sMin').value+'&max='+$('#sMax').value+'&beep='+$('#sBeep').value,'Réglages enregistrés').then(()=>{setFilled=false;});}\n"
  "function newSession(){if(!confirm('Archiver la session en cours et repartir de zéro ? Les manches des coureurs seront remises à zéro au départ.'))return;act('/api/session/new','Nouvelle session').then(loadArch);}\n"
  "function drawState(){const S=st.st,E=st.em,Y=st.sync,B=st.beam;\n"
  "pill('pStart',S.maint?'Départ en maintenance':S.ok?'Départ OK':'Départ absent',S.ok&&!S.maint?'ok':'bad');\n"
  "pill('pSync',Y.st===1?'Synchro '+(Y.u/1000).toFixed(2)+' ms':Y.st===2?'Synchro dégradée':'Synchro --',Y.st===1?'ok':Y.st===2?'warn':'bad');\n"
  "pill('pBeam',st.set.bench?'Banc actif':!B.est?'Faisceau absent':B.fault?'Faisceau bloqué':'Faisceau '+B.q+' %',st.set.bench?'warn':(!B.est||B.fault)?'bad':B.q>=90?'ok':'warn');\n"
  "pill('pBat','Arrivée '+batt(st.bat),battCls(st.bat));pill('pBatS','Départ '+(S.ok?batt(S.bat):'--'),S.ok?battCls(S.bat):'');pill('pBatE','Émetteur '+(E.ok?batt(E.bat):'--'),E.ok?battCls(E.bat):'');\n"
  "drawLast();drawOnCourse();if(tab==='set')drawSet();if(tab==='sys')drawSys();}\n"
  "function drawData(){if(!st)return;if(tab==='rank')drawRank();if(tab==='runs')drawRuns();drawLast();}\n"
  "setInterval(()=>{if(st)drawOnCourse();},100);\n"
  "refresh(true).finally(function loop(){setTimeout(()=>refresh(false).finally(loop),tab==='set'?300:1000);});\n"
  "</script></body></html>";

// ============================================================================
// 22. SERVEUR WEB : aides JSON
// ============================================================================
static void jEsc(String& o, const char* s) {
  o += '"';
  for (; s && *s; s++) {
    char c = *s;
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c < 0x20) { char b[8]; snprintf(b, sizeof b, "\\u%04x", c); o += b; }
    else o += c;
  }
  o += '"';
}
static void jKey(String& o, const char* k) { o += '"'; o += k; o += "\":"; }
static void jI(String& o, const char* k, long long v) { char b[24]; snprintf(b, sizeof b, "%lld", v); jKey(o, k); o += b; o += ','; }
static void jF(String& o, const char* k, double v) { char b[32]; snprintf(b, sizeof b, "%.3f", v); jKey(o, k); o += b; o += ','; }
static void jB(String& o, const char* k, bool v) { jKey(o, k); o += v ? "true," : "false,"; }
static void jS(String& o, const char* k, const char* s) { jKey(o, k); jEsc(o, s); o += ','; }
static void jEnd(String& o, char close) {
  if (o.length() && o[o.length() - 1] == ',') o.setCharAt(o.length() - 1, close);
  else o += close;
}

static void sendOk() { g_web.send(200, "text/plain", "ok"); }
static void sendErr(const char* m) { g_web.send(400, "text/plain", m); }
static bool argI(const char* name, long& v) {
  if (!g_web.hasArg(name)) return false;
  v = g_web.arg(name).toInt();
  return true;
}

static bool safeName(const String& s) {
  if (s.length() == 0 || s.length() > 40 || s.indexOf("..") >= 0) return false;
  for (unsigned i = 0; i < s.length(); i++) {
    char c = s[i];
    if (!(isalnum((unsigned char)c) || c == '_' || c == '-' || c == '.')) return false;
  }
  return true;
}

static void sendDownload(const char* path, const char* filename) {
  File f = LittleFS.open(path, FILE_READ);
  if (!f) { g_web.send(404, "text/plain", "Fichier absent"); return; }
  String cd = String("attachment; filename=\"") + filename + "\"";
  g_web.sendHeader("Content-Disposition", cd);
  g_web.streamFile(f, "text/csv");
  f.close();
}

// ============================================================================
// 23. ROUTES COMMUNES (style, script, OTA web, heure, archives)
// ============================================================================
static void webSetupCommon() {
  g_web.on("/s.css", HTTP_GET, []() { g_web.sendHeader("Cache-Control", "max-age=300"); g_web.send_P(200, "text/css", CSS_COMMON); });
  g_web.on("/c.js", HTTP_GET, []() { g_web.sendHeader("Cache-Control", "max-age=300"); g_web.send_P(200, "application/javascript", JS_COMMON); });

  g_web.on("/update", HTTP_GET, []() {
    if (!g_web.authenticate(ADMIN_USER, ADMIN_PASSWORD)) return g_web.requestAuthentication();
    g_web.send_P(200, "text/html", PAGE_UPDATE);
  });
  g_web.on("/update", HTTP_POST,
    []() {
      if (g_upAuthFail) { g_upAuthFail = false; g_web.send(401, "text/plain", "Identifiants requis"); return; }
      bool ok = g_upOk && !Update.hasError();
      g_web.sendHeader("Connection", "close");
      g_web.send(ok ? 200 : 500, "text/plain", ok ? "OK" : (String("Echec : ") + Update.errorString()));
      if (ok) { delay(400); ESP.restart(); }
      g_otaRunning = false;
    },
    []() {
      HTTPUpload& u = g_web.upload();
      if (u.status == UPLOAD_FILE_START) {
        g_upOk = false;
        g_upAuthFail = !g_web.authenticate(ADMIN_USER, ADMIN_PASSWORD);
        if (g_upAuthFail) return;
        g_otaRunning = true;
        LOGF("[ota web] %s\n", u.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
      } else if (u.status == UPLOAD_FILE_WRITE) {
        if (g_upAuthFail || !Update.isRunning()) return;
        if (Update.write(u.buf, u.currentSize) != u.currentSize) Update.printError(Serial);
      } else if (u.status == UPLOAD_FILE_END) {
        if (g_upAuthFail) return;
        g_upOk = Update.end(true);
        if (!g_upOk) Update.printError(Serial);
      } else if (u.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        g_otaRunning = false;
      }
    });

  g_web.on("/api/time", HTTP_POST, []() {
    long long ms = atoll(g_web.arg("ms").c_str());
    if (ms > 1600000000000LL) {
      Lock l;
      g_wallOffsetMs = ms - nowUs() / 1000;
      g_wallValid = true;
      g_tzOffsetMin = g_web.arg("tz").toInt();
    }
    sendOk();
  });
  g_web.on("/api/reboot", HTTP_POST, []() { sendOk(); delay(300); ESP.restart(); });

  g_web.on("/api/archives", HTTP_GET, []() {
    String o = "[";
    File dir = LittleFS.open("/arch");
    if (dir && dir.isDirectory()) {
      File f = dir.openNextFile();
      while (f) {
        o += '{';
        jS(o, "f", f.name());
        jI(o, "s", (long long)f.size());
        jEnd(o, '}');
        o += ',';
        f = dir.openNextFile();
      }
    }
    jEnd(o, ']');
    g_web.send(200, "application/json", o);
  });
  g_web.on("/api/archive", HTTP_GET, []() {
    String f = g_web.arg("f");
    if (!safeName(f)) return sendErr("Nom invalide");
    sendDownload((String("/arch/") + f).c_str(), f.c_str());
  });
  g_web.on("/api/archive/del", HTTP_POST, []() {
    String f = g_web.arg("f");
    if (!safeName(f)) return sendErr("Nom invalide");
    flashMark(true);
    LittleFS.remove(String("/arch/") + f);
    flashMark(false);
    sendOk();
  });
  g_web.onNotFound([]() { g_web.send(404, "text/plain", "Introuvable"); });
}

static void webSetupMaintenance() {
  g_web.on("/", HTTP_GET, []() {
    if (!g_web.authenticate(ADMIN_USER, ADMIN_PASSWORD)) return g_web.requestAuthentication();
    g_web.send_P(200, "text/html", PAGE_UPDATE);
  });
}

// ============================================================================
// 24. PAGES DU DEPART
// ============================================================================
static void webStartState() {
  String o;
  o.reserve(3500);
  int64_t now = nowUs();
  {
    Lock l;
    const PeerInfo& f = g_peers[ROLE_FINISH];
    bool ok = peerOk(ROLE_FINISH, now);
    o += '{';
    jS(o, "role", "start");
    jS(o, "fw", FW_VERSION);
    jI(o, "sess", g_sessionId);
    jI(o, "bat", g_battMv);
    jB(o, "armed", g_hallArmed);
    jB(o, "hall", digitalRead(PIN_HALL) == LOW);
    jB(o, "tune", g_hallTune);
    jI(o, "tcnt", g_tuneCount);
    jB(o, "bench", g_benchRemote);
    jI(o, "rrev", g_ridersRev);
    jI(o, "cw", CANCEL_WINDOW_S);
    jI(o, "pend", outboxCount());
    jKey(o, "link"); o += '{';
    jB(o, "ok", ok);
    jI(o, "bat", f.battMv);
    jI(o, "rssi", f.rssi);
    jB(o, "syncok", ok && g_syncOkRemote);
    jI(o, "u", f.syncUncertUs == UINT32_MAX ? -1 : (long long)f.syncUncertUs);
    jI(o, "ebat", ok ? f.relayBattEmitter : 0);
    jI(o, "via", (directOk(ROLE_FINISH, now) ? 1 : 0) | (viaRelayRecent(ROLE_FINISH, now) ? 2 : 0) | (loraRecent(ROLE_FINISH, now) ? 4 : 0));
    jB(o, "rlseen", g_peers[ROLE_RELAY].known);
    jB(o, "rl", directOk(ROLE_RELAY, now));
    jI(o, "rlbat", g_peers[ROLE_RELAY].battMv);
    jS(o, "fw", f.fw);
    jEnd(o, '}'); o += ',';
    jKey(o, "next");
    if (!g_benchRemote && g_nextIdx >= 0 && g_nextIdx < (int)g_riderCount) {
      const Rider& r = g_riders[g_nextIdx];
      o += '{';
      jI(o, "id", r.id); jI(o, "bib", r.bib); jS(o, "nm", r.name); jS(o, "ct", r.cat); jI(o, "runs", r.runs);
      jEnd(o, '}'); o += ',';
    } else {
      o += "null,";
    }
    jKey(o, "starts"); o += '[';
    for (uint16_t k = 0; k < g_startCount && k < 20; k++) {
      const StartRec& r = g_starts[(g_startHead + START_HIST - 1 - k) % START_HIST];
      o += '{';
      jI(o, "sb", r.startBoot); jI(o, "sq", r.runSeq); jI(o, "w", r.wallMs);
      jI(o, "age", r.startBoot == g_bootId ? (now - r.tStart) / 1000000 : -1);
      jI(o, "bib", r.bib); jS(o, "nm", r.name); jI(o, "st", r.state); jI(o, "d", r.durationUs);
      jI(o, "fs", r.finStatus); jI(o, "fl", r.evtFlags);
      jEnd(o, '}'); o += ',';
    }
    jEnd(o, ']'); o += ',';
    jEnd(o, '}');
  }
  g_web.send(200, "application/json", o);
}

static void webSetupStart() {
  g_web.on("/", HTTP_GET, []() { g_web.send_P(200, "text/html", PAGE_START); });
  g_web.on("/api/state", HTTP_GET, webStartState);
  g_web.on("/api/riders", HTTP_GET, []() {
    String o;
    o.reserve(80 + g_riderCount * 90);
    {
      Lock l;
      o += '[';
      for (uint16_t i = 0; i < g_riderCount; i++) {
        const Rider& r = g_riders[i];
        o += '{';
        jI(o, "id", r.id); jI(o, "bib", r.bib); jS(o, "nm", r.name); jS(o, "ct", r.cat); jI(o, "runs", r.runs);
        jEnd(o, '}'); o += ',';
      }
      jEnd(o, ']');
    }
    g_web.send(200, "application/json", o);
  });
  g_web.on("/api/rider/save", HTTP_POST, []() {
    long id = g_web.arg("id").toInt();
    long bib = constrain(g_web.arg("bib").toInt(), 0L, 65535L);
    String nm = g_web.arg("name"), ct = g_web.arg("cat");
    Lock l;
    if (id == 0) {
      if (!riderAdd((uint16_t)bib, nm.c_str(), ct.c_str())) return sendErr("Liste pleine");
    } else {
      int i = riderIndexById((uint16_t)id);
      if (i < 0) return sendErr("Coureur introuvable");
      g_riders[i].bib = (uint16_t)bib;
      copyClean(g_riders[i].name, NAME_LEN, nm.c_str());
      copyClean(g_riders[i].cat, CAT_LEN, ct.c_str());
    }
    ridersChanged();
    sendOk();
  });
  g_web.on("/api/rider/del", HTTP_POST, []() {
    Lock l;
    int i = riderIndexById((uint16_t)g_web.arg("id").toInt());
    if (i < 0) return sendErr("Coureur introuvable");
    riderDelete(i);
    ridersChanged();
    sendOk();
  });
  g_web.on("/api/rider/move", HTTP_POST, []() {
    Lock l;
    int i = riderIndexById((uint16_t)g_web.arg("id").toInt());
    riderMove(i, g_web.arg("d").toInt() < 0 ? -1 : 1);
    ridersChanged();
    sendOk();
  });
  g_web.on("/api/next", HTTP_POST, []() {
    Lock l;
    int i = riderIndexById((uint16_t)g_web.arg("id").toInt());
    if (i < 0) return sendErr("Coureur introuvable");
    g_nextIdx = (int16_t)i;
    ridersChanged();
    sendOk();
  });
  g_web.on("/api/next/step", HTTP_POST, []() {
    Lock l;
    if (g_riderCount) {
      int d = g_web.arg("d").toInt() < 0 ? -1 : 1;
      g_nextIdx = (int16_t)((g_nextIdx + d + g_riderCount) % g_riderCount);
      ridersChanged();
    }
    sendOk();
  });
  g_web.on("/api/riders/import", HTTP_POST, []() {
    String body = g_web.arg("plain");
    bool replace = g_web.arg("mode") != "append";
    int added = 0, skipped = 0;
    {
      Lock l;
      if (replace) { g_riderCount = 0; g_nextIdx = -1; }
      int start = 0;
      while (start < (int)body.length()) {
        int end = body.indexOf('\n', start);
        if (end < 0) end = body.length();
        String line = body.substring(start, end);
        start = end + 1;
        line.trim();
        if (!line.length()) continue;
        int t1 = line.indexOf('\t'), t2 = t1 >= 0 ? line.indexOf('\t', t1 + 1) : -1;
        String b = t1 >= 0 ? line.substring(0, t1) : String("");
        String n = t1 >= 0 ? (t2 >= 0 ? line.substring(t1 + 1, t2) : line.substring(t1 + 1)) : line;
        String c = t2 >= 0 ? line.substring(t2 + 1) : String("");
        if (riderAdd((uint16_t)constrain(b.toInt(), 0L, 65535L), n.c_str(), c.c_str())) added++;
        else skipped++;
      }
      if (g_riderCount && g_nextIdx < 0) g_nextIdx = 0;
      ridersChanged();
    }
    char msg[96];
    snprintf(msg, sizeof msg, "%d coureur(s) importe(s)%s", added, skipped ? ", liste pleine : certains ignores" : "");
    g_web.send(200, "text/plain", msg);
  });
  g_web.on("/api/riders/clear", HTTP_POST, []() {
    Lock l;
    g_riderCount = 0;
    g_nextIdx = -1;
    ridersChanged();
    sendOk();
  });
  g_web.on("/api/cancel", HTTP_POST, []() {
    Lock l;
    if (!startCancel((uint32_t)g_web.arg("sb").toInt(), (uint32_t)g_web.arg("sq").toInt())) return sendErr("Depart introuvable ou deja annule");
    sendOk();
  });
  g_web.on("/api/riders.csv", HTTP_GET, []() {
    String o = "\xEF\xBB\xBF" "dossard;pilote;categorie;manches\n";
    {
      Lock l;
      for (uint16_t i = 0; i < g_riderCount; i++) {
        char line[96];
        snprintf(line, sizeof line, "%u;%s;%s;%u\n", g_riders[i].bib, g_riders[i].name, g_riders[i].cat, g_riders[i].runs);
        o += line;
      }
    }
    g_web.sendHeader("Content-Disposition", "attachment; filename=\"coureurs.csv\"");
    g_web.send(200, "text/csv", o);
  });
  g_web.on("/api/halltune", HTTP_POST, []() { Lock l; setHallTune(g_web.arg("on").toInt() != 0); sendOk(); });
  g_web.on("/api/halltune/reset", HTTP_POST, []() { Lock l; g_tuneCount = 0; sendOk(); });
  g_web.on("/api/starts.csv", HTTP_GET, []() { sendDownload(STARTS_LOG, "journal_departs.csv"); });
}

// ============================================================================
// 25. PAGES DE L'ARRIVEE
// ============================================================================
static void peerJson(String& o, const char* k, uint8_t role, int64_t now) {
  const PeerInfo& p = g_peers[role];
  bool ok = peerOk(role, now);
  jKey(o, k);
  o += '{';
  jB(o, "ok", ok);
  jI(o, "bat", p.battMv);
  jI(o, "rssi", p.rssi);
  jB(o, "maint", ok && (p.hbFlags & HBF_MAINT));
  jB(o, "seen", p.known);
  jI(o, "ip", p.ip);
  jS(o, "fw", p.fw);
  jEnd(o, '}');
  o += ',';
}

// Une ligne de liaison : ok = donnees valables, d/r/a = % direct / via relais / total, s = RSSI
// l = % LoRa, ls = RSSI LoRa ; le total tient compte du LoRa
static void jLink(String& o, const char* k, bool ok, int d, int r, int a, int s, int l, int ls) {
  if (l > a) a = l;
  jKey(o, k);
  o += '{';
  jB(o, "ok", ok); jI(o, "d", d); jI(o, "r", r); jI(o, "a", a); jI(o, "s", s); jI(o, "l", l); jI(o, "ls", ls);
  jEnd(o, '}');
  o += ',';
}

static void linkJson(String& o, int64_t now) {
  const PeerInfo& S = g_peers[ROLE_START];
  const PeerInfo& R = g_peers[ROLE_RELAY];
  uint8_t d, r, a;
  bool sOk = peerOk(ROLE_START, now), rOk = peerOk(ROLE_RELAY, now);
  jKey(o, "lk");
  o += '{';
  jB(o, "lr", g_lr);
  hbStats(S, now, d, r, a);
  int lo = LORA_ME ? loraPct(S, now) : -1;
  jLink(o, "sf", S.known, d, r, a, directOk(ROLE_START, now) ? S.rssi : 0, lo, loraRecent(ROLE_START, now) ? S.loraRssi : 0);   // l'arrivee entend le depart
  jLink(o, "fs", sOk, S.rxDir[ROLE_FINISH], S.rxRel[ROLE_FINISH], S.rxAny[ROLE_FINISH], S.rssiRep[ROLE_FINISH],
        LORA_ME ? S.rxLoraRep[ROLE_FINISH] : -1, S.lrssiRep[ROLE_FINISH]);                                       // rapport du depart
  jLink(o, "rs", rOk, R.rxDir[ROLE_START], -1, -1, R.rssiRep[ROLE_START], -1, 0);                             // le relais entend le depart
  jLink(o, "sr", sOk, S.rxDir[ROLE_RELAY], -1, -1, S.rssiRep[ROLE_RELAY], -1, 0);                             // le depart entend le relais
  jLink(o, "rf", rOk, R.rxDir[ROLE_FINISH], -1, -1, R.rssiRep[ROLE_FINISH], -1, 0);                           // le relais entend l'arrivee
  hbStats(R, now, d, r, a);
  jLink(o, "fr", R.known, d, -1, -1, directOk(ROLE_RELAY, now) ? R.rssi : 0, -1, 0);                          // l'arrivee entend le relais
  jKey(o, "lora");
  o += '{';
  jB(o, "en", LORA_ME); jB(o, "ok", g_loraOk); jI(o, "sf", LORA_SF); jI(o, "dbm", LORA_TX_DBM);
  jF(o, "mhz", LORA_FREQ_HZ / 1e6); jI(o, "snr", g_loraLastSnr);
  jI(o, "duty", g_loraOk ? (LORA_BUCKET_US - g_loraTokens) * 100 / 60000000LL : 0);   // % d'emission sur la derniere minute
  jEnd(o, '}');
  o += ',';
  jEnd(o, '}');
  o += ',';
}

static void webFinishState() {
  String o;
  o.reserve(2048);
  int64_t now = nowUs();
  {
    Lock l;
    o += '{';
    jS(o, "role", "finish");
    jS(o, "fw", FW_VERSION);
    jI(o, "now", now);
    jI(o, "sess", g_sessionId);
    jI(o, "rev", g_runsRev);
    jI(o, "bat", g_battMv);
    jI(o, "last", g_lastFinishedIdx);
    jKey(o, "set"); o += '{';
    jI(o, "n", g_setRuns); jI(o, "min", g_setMinS); jI(o, "max", g_setMaxS); jB(o, "beep", g_setBeep); jB(o, "bench", g_bench);
    jEnd(o, '}'); o += ',';
    peerJson(o, "st", ROLE_START, now);
    peerJson(o, "em", ROLE_EMITTER, now);
    peerJson(o, "rl", ROLE_RELAY, now);
    linkJson(o, now);
    SyncModel* m = currentModel();
    jKey(o, "sync"); o += '{';
    jI(o, "st", syncStateOf(m, now));
    jI(o, "u", (m && m->valid) ? (long long)modelUncertAt(*m, now) : -1);
    jI(o, "rtt", m ? m->minRtt : 0);
    jI(o, "n", m ? m->n : 0);
    jF(o, "ppm", m ? m->b : 0.0);
    jEnd(o, '}'); o += ',';
    jKey(o, "beam"); o += '{';
    jB(o, "est", g_beamEstablished); jI(o, "q", g_beamQuality); jB(o, "br", g_beamBroken); jB(o, "fault", g_beamFault); jB(o, "align", g_align);
    jI(o, "brk", g_breakCount);
    jI(o, "lbAge", g_breakCount ? (now - g_breakStartUs) / 1000 : -1);
    jI(o, "lbDur", g_lastBreakDurUs < 0 ? -1 : g_lastBreakDurUs / 1000);
    jI(o, "lbKind", g_lastBreakKind);
    jI(o, "pw", peerOk(ROLE_EMITTER, now) ? g_peers[ROLE_EMITTER].irPower : 0);
    jI(o, "pwDef", IR_POWER_DEFAULT);
    jKey(o, "cal"); o += '{';
    jB(o, "run", g_calRunning); jI(o, "lvl", g_calLevel); jI(o, "res", g_calResult); jI(o, "found", g_calFound); jI(o, "fin", g_calFinal);
    jEnd(o, '}'); o += ',';
    jEnd(o, '}'); o += ',';
    const BenchStats& b = g_benchSt;
    jKey(o, "bench"); o += '{';
    jI(o, "n", b.n); jF(o, "mean", b.mean); jF(o, "sd", b.n > 1 ? sqrt(b.m2 / (b.n - 1)) : 0.0);
    jI(o, "min", b.minUs); jI(o, "max", b.maxUs); jI(o, "last", b.lastUs);
    jEnd(o, '}'); o += ',';
    jKey(o, "oc"); o += '[';
    for (uint16_t i = 0; i < g_runCount; i++) {
      Run& r = g_runs[i];
      if (r.status != RUN_ON_COURSE) continue;
      o += '{';
      jI(o, "i", r.idx);
      int64_t ts;
      if (runStartF(r, ts)) jI(o, "ts", ts);
      else { jKey(o, "ts"); o += "null,"; }
      jEnd(o, '}'); o += ',';
    }
    jEnd(o, ']'); o += ',';
    jEnd(o, '}');
  }
  g_web.send(200, "application/json", o);
}

static void webFinishRuns() {
  String o;
  {
    Lock l;
    o.reserve(96 + g_runCount * 150);
    o += "{\"runs\":[";
    for (uint16_t i = 0; i < g_runCount; i++) {
      const Run& r = g_runs[i];
      o += '{';
      jI(o, "i", r.idx); jI(o, "rid", r.riderId); jI(o, "bib", r.bib); jS(o, "nm", r.name); jS(o, "ct", r.cat);
      jI(o, "st", r.status); jI(o, "fl", r.flags); jI(o, "d", r.durationUs);
      jI(o, "u", r.uncertUs == UINT32_MAX ? -1 : (long long)r.uncertUs); jI(o, "w", runWall(r));
      jEnd(o, '}'); o += ',';
    }
    jEnd(o, ']');
    o += ",\"orph\":[";
    for (int i = 0; i < MAX_ORPHANS; i++) {
      const Orphan& p = g_orphans[i];
      if (!p.used || p.finishBoot != g_bootId) continue;
      o += '{';
      jI(o, "id", p.id); jI(o, "w", p.wallMs); jI(o, "fl", p.flags);
      jEnd(o, '}'); o += ',';
    }
    jEnd(o, ']');
    o += ',';
    jI(o, "best", g_bestRunUs);
    jEnd(o, '}');
  }
  g_web.send(200, "application/json", o);
}

// CSV des passages de la session (export et archive). Appeler sous verrou.
static String buildRunsCsv() {
  static const char* ST[] = {"EN PISTE", "ARRIVE", "DNF", "ANNULE"};
  String o;
  o.reserve(200 + g_runCount * 120);
  o += "\xEF\xBB\xBF" "session;passage;manche;dossard;pilote;categorie;heure_depart;temps;temps_ms;statut;incertitude_ms;remarques\n";
  for (uint16_t i = 0; i < g_runCount; i++) {
    const Run& r = g_runs[i];
    char dur[16], clk[12], ms[24], unc[16], rem[96], name[NAME_LEN + 12];
    bool fin = r.status == RUN_FINISHED && r.durationUs >= 0;
    fmtDur(dur, sizeof dur, fin ? r.durationUs : -1);
    if (!fin) dur[0] = 0;
    fmtClockLocal(clk, sizeof clk, runWall(r));
    snprintf(ms, sizeof ms, fin ? "%lld" : "", (long long)(r.durationUs / 1000));
    if (fin && r.uncertUs != UINT32_MAX) snprintf(unc, sizeof unc, "%.2f", r.uncertUs / 1000.0);
    else unc[0] = 0;
    rem[0] = 0;
    if (r.flags & RUNF_FLASH) strlcat(rem, "flash ", sizeof rem);
    if (r.flags & RUNF_START_FLASH) strlcat(rem, "flash-depart ", sizeof rem);
    if (r.flags & RUNF_SYNC_DEGRADED) strlcat(rem, "synchro-degradee ", sizeof rem);
    if (r.flags & RUNF_NO_SYNC) strlcat(rem, "sans-synchro ", sizeof rem);
    if (r.flags & RUNF_LATE_START) strlcat(rem, "depart-tardif ", sizeof rem);
    if (r.flags & RUNF_MANUAL) strlcat(rem, "manuel ", sizeof rem);
    if (r.flags & RUNF_AUTO_DNF) strlcat(rem, "dnf-auto ", sizeof rem);
    if (r.name[0]) strlcpy(name, r.name, sizeof name);
    else snprintf(name, sizeof name, "Pilote %u", r.idx + 1);
    char line[320];
    snprintf(line, sizeof line, "%lu;%u;%u;%u;%s;%s;%s;%s;%s;%s;%s;%s\n", (unsigned long)g_sessionId, r.idx + 1,
             r.status == RUN_CANCELLED ? 0 : runNumber(r), r.bib, name, r.cat, clk, dur, ms, ST[r.status & 3], unc, rem);
    o += line;
  }
  return o;
}

static void webSetupFinish() {
  g_web.on("/", HTTP_GET, []() { g_web.send_P(200, "text/html", PAGE_FINISH); });
  g_web.on("/api/state", HTTP_GET, webFinishState);
  g_web.on("/api/runs", HTTP_GET, webFinishRuns);
  g_web.on("/api/run", HTTP_POST, []() {
    long i = -1, j = -1;
    argI("i", i);
    argI("j", j);
    Lock l;
    const char* err = finishRunAction((int)i, g_web.arg("a"), (int)j);
    if (err[0]) return sendErr(err);
    sendOk();
  });
  g_web.on("/api/orphan", HTTP_POST, []() {
    long id = g_web.arg("id").toInt();
    Lock l;
    Orphan* o = nullptr;
    for (int k = 0; k < MAX_ORPHANS; k++)
      if (g_orphans[k].used && g_orphans[k].id == id && g_orphans[k].finishBoot == g_bootId) o = &g_orphans[k];
    if (!o) return sendErr("Passage introuvable");
    if (g_web.hasArg("del")) {
      o->used = false;
      logOrphan(*o);
      g_runsRev++;
      return sendOk();
    }
    long i = -1;
    argI("i", i);
    if (i < 0 || i >= g_runCount || g_runs[i].status != RUN_ON_COURSE) return sendErr("Pilote pas en piste");
    assignOrphanToRun(*o, g_runs[i], RUNF_MANUAL);
    sendOk();
  });
  g_web.on("/api/settings", HTTP_POST, []() {
    Lock l;
    long v;
    if (argI("n", v)) g_setRuns = (uint8_t)constrain(v, 1L, 9L);
    if (argI("min", v)) g_setMinS = (uint16_t)constrain(v, 0L, 3600L);
    if (argI("max", v)) g_setMaxS = (uint16_t)constrain(v, 10L, 7200L);
    if (argI("beep", v)) g_setBeep = v != 0;
    flashMark(true);
    g_prefs.putUChar("runs", g_setRuns);
    g_prefs.putUShort("min", g_setMinS);
    g_prefs.putUShort("max", g_setMaxS);
    g_prefs.putBool("beep", g_setBeep);
    flashMark(false);
    recomputeBest();
    g_runsRev++;
    sendOk();
  });
  g_web.on("/api/bench", HTTP_POST, []() { Lock l; setBench(g_web.arg("on").toInt() != 0); sendOk(); });
  g_web.on("/api/bench/reset", HTTP_POST, []() { Lock l; memset(&g_benchSt, 0, sizeof g_benchSt); sendOk(); });
  g_web.on("/api/align", HTTP_POST, []() { Lock l; setAlign(g_web.arg("on").toInt() != 0, true); sendOk(); });
  g_web.on("/api/maint", HTTP_POST, []() {
    String t = g_web.arg("t");
    if (t == "start") sendCmd(ROLE_START, CMD_MAINTENANCE, 0);
    else if (t == "emitter") sendCmd(ROLE_EMITTER, CMD_MAINTENANCE, 0);
    else if (t == "relay") sendCmd(ROLE_RELAY, CMD_MAINTENANCE, 0);
    else return sendErr("Cible inconnue");
    sendOk();
  });
  g_web.on("/api/locate", HTTP_POST, []() {
    String t = g_web.arg("t");
    sendCmd(t == "start" ? ROLE_START : t == "relay" ? ROLE_RELAY : ROLE_EMITTER, CMD_LOCATE, 0);
    sendOk();
  });
  g_web.on("/api/session/new", HTTP_POST, []() { Lock l; finishNewSession(); sendOk(); });
  g_web.on("/api/wipe", HTTP_POST, []() { Lock l; finishWipe(g_web.arg("all").toInt() != 0); sendOk(); });
  g_web.on("/api/irpower", HTTP_POST, []() {
    long lv = g_web.arg("l").toInt();
    if (lv < 1 || lv > 10) return sendErr("Niveau 1 a 10");
    Lock l;
    if (!peerOk(ROLE_EMITTER, nowUs())) return sendErr("Emetteur absent");
    g_calRunning = false;
    sendCmd(ROLE_EMITTER, CMD_SET_IR_POWER, (uint32_t)lv);
    sendOk();
  });
  g_web.on("/api/ircal", HTTP_POST, []() {
    Lock l;
    if (!calStart()) return sendErr("Emetteur absent : allume-le et attends qu'il apparaisse");
    sendOk();
  });
  g_web.on("/api/beamtest/reset", HTTP_POST, []() {
    Lock l;
    g_breakCount = 0;
    g_lastBreakKind = BK_NONE;
    g_lastBreakDurUs = -1;
    sendOk();
  });
  g_web.on("/api/export.csv", HTTP_GET, []() {
    String csv;
    { Lock l; csv = buildRunsCsv(); }
    char fn[40];
    snprintf(fn, sizeof fn, "attachment; filename=\"session_%03lu.csv\"", (unsigned long)g_sessionId);
    g_web.sendHeader("Content-Disposition", fn);
    g_web.send(200, "text/csv", csv);
  });
}

// ============================================================================
// 26. SETUP ET LOOP
// ============================================================================
// Depart et arrivee : point d'acces normal (telephones) + station pour l'ESP-NOW Long Range.
// Emetteur et relais : station seule. L'ESP-NOW passe toujours par la station.
static void wifiSetupNormal() {
  WiFi.persistent(false);
  WiFi.setHostname(myHostname());
  if (HAS_WEB) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(IS_START ? AP_SSID_START : AP_SSID_FINISH, AP_PASSWORD, RADIO_CHANNEL, 0, 4);
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE);
  }
  WiFi.setTxPower(WIFI_TX_POWER);
  radioInit(WIFI_IF_STA, true);
  LOGF("[wifi] canal %d, MAC ESP-NOW %s, antenne %s\n", RADIO_CHANNEL, WiFi.macAddress().c_str(), ANT_EXT_ME ? "externe" : "ceramique");
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);   // ne jamais bloquer si aucun ordinateur n'ecoute
  delay(200);

  // Commutateur RF du XIAO, puis broches de la carte
  pinMode(PIN_RF_ENABLE, OUTPUT);
  digitalWrite(PIN_RF_ENABLE, LOW);
  pinMode(PIN_RF_ANT, OUTPUT);
  digitalWrite(PIN_RF_ANT, ANT_EXT_ME ? HIGH : LOW);
  if (!IS_EMITTER) { pinMode(PIN_IR_TX, OUTPUT); digitalWrite(PIN_IR_TX, LOW); }   // LED IR eteintes
  if (LORA_ME) { pinMode(PIN_LORA_NSS, OUTPUT); digitalWrite(PIN_LORA_NSS, HIGH); }   // module LoRa deselectionne
  pinMode(PIN_HALL, INPUT);
  pinMode(PIN_IR_RX, INPUT_PULLUP);
  analogSetPinAttenuation(PIN_BATT, ADC_11db);
  analogSetPinAttenuation(PIN_BUTTON, ADC_11db);
  ledcAttach(PIN_BUZZER, BUZ_HI, 10);
  ledcWriteTone(PIN_BUZZER, 0);

  g_mtx = xSemaphoreCreateRecursiveMutex();
  g_inQ = xQueueCreate(16, sizeof(InMsg));
  g_hwQ = xQueueCreate(16, sizeof(HwEvt));
  g_buzQ = xQueueCreate(8, sizeof(BuzItem));
  g_storeQ = xQueueCreate(24, sizeof(StoreMsg));
  xTaskCreate(buzzerTask, "buzzer", 3072, nullptr, 6, nullptr);

  g_prefs.begin("chrono", false);
  g_bootId = g_prefs.getUInt("boot", 0) + 1;
  g_prefs.putUInt("boot", g_bootId);
  g_sessionId = g_prefs.getUInt("sess", 1);
  bool maintReq = g_prefs.getBool("maint", false);
  if (maintReq) g_prefs.putBool("maint", false);
  if (IS_FINISH) {
    g_setRuns = g_prefs.getUChar("runs", DEFAULT_RUNS);
    g_setMinS = g_prefs.getUShort("min", DEFAULT_MIN_RUN_S);
    g_setMaxS = g_prefs.getUShort("max", DEFAULT_MAX_RUN_S);
    g_setBeep = g_prefs.getBool("beep", true);
  }
  LOGF("\n[chronodh] role %d, fw %s, boot %lu, session %lu, reset %d\n", CHRONO_ROLE, FW_VERSION, (unsigned long)g_bootId,
       (unsigned long)g_sessionId, (int)esp_reset_reason());

  g_fsOk = LittleFS.begin(true);
  if (g_fsOk) { if (!LittleFS.exists("/arch")) LittleFS.mkdir("/arch"); }
  else LOGF("[fs] LittleFS indisponible : verifie le schema de partition\n");
  xTaskCreate(storageTask, "storage", 4096, nullptr, 3, nullptr);
  g_battMv = readBatteryMv();

  if (!IS_FINISH && bootHoldDetected()) maintReq = true;
  if (maintReq && !IS_FINISH) { maintenanceSetup(); return; }

  if (IS_FINISH) {
    g_runs = (Run*)calloc(MAX_RUNS, sizeof(Run));
    if (!g_runs) LOGF("[arrivee] memoire insuffisante\n");
    finishLoadLog();
  }
  if (IS_START) { loadRiders(); startLoadLog(); }

  wifiSetupNormal();
  if (LORA_ME) loraInit();

  if (IS_START) {
    startRestorePending();
    attachInterrupt(digitalPinToInterrupt(PIN_HALL), isrHall, FALLING);
  }
  if (IS_FINISH) attachInterrupt(digitalPinToInterrupt(PIN_IR_RX), isrIrRx, FALLING);
  if (IS_EMITTER) {
    g_irPower = (uint8_t)constrain((int)g_prefs.getUChar("irpow", IR_POWER_DEFAULT), 1, 10);
    g_irTxOk = irTxStart(g_irPower);
  }

  if (HAS_WEB) {
    otaSetup();
    webSetupCommon();
    if (IS_START) webSetupStart();
    else webSetupFinish();
    g_web.begin();
  }

  xTaskCreate(coreTask, "core", 8192, nullptr, 10, nullptr);

  beep(IS_START ? PAT_BOOT1 : IS_FINISH ? PAT_BOOT2 : IS_EMITTER ? PAT_BOOT3 : PAT_BOOT4);
  if (IS_EMITTER && !g_irTxOk) beep(PAT_ERROR);
}

void loop() {
  if (g_maintenance) {
    maintenanceLoop();
    delay(2);
    return;
  }
  int64_t now = nowUs();
  if (HAS_WEB) {
    ArduinoOTA.handle();
    g_web.handleClient();
  }
  buttonService();
  batteryService(now);
  alignService(now);
  consolePoll();
  delay(2);
}
