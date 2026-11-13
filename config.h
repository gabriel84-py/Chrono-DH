#pragma once
/*
  config.h - Configuration du chronometre VTT DH (chronodh)

  Un seul firmware pour tous les boitiers (depart, arrivee, emetteur, relais).
  Avant chaque televersement, change UNIQUEMENT la ligne CHRONO_ROLE (section 1).
  Tout le reste doit etre identique sur tous les boitiers d'un meme jeu.
*/

// ============================================================================
// 1. ROLE DU BOITIER
// ============================================================================
#define ROLE_START    1   // depart  : baguette + capteur Hall, Wi-Fi + page des coureurs
#define ROLE_FINISH   2   // arrivee : recepteur IR, calcul des temps, Wi-Fi + page des resultats
#define ROLE_EMITTER  3   // emetteur IR : porteuse 38 kHz face a l'arrivee, pas de page web
#define ROLE_RELAY    4   // relais : pose a mi-chemin, repete les messages depart <-> arrivee

#define CHRONO_ROLE   ROLE_FINISH

// ============================================================================
// 2. RESEAU (identique sur tous les boitiers)
// ============================================================================
#define NET_ID              0x4448    // identifiant du jeu de boitiers (change-le si un autre chrono tourne a cote)
#define RADIO_CHANNEL       6         // canal Wi-Fi commun a ESP-NOW et aux points d'acces (1, 6 ou 11)
#define WIFI_TX_POWER       WIFI_POWER_19_5dBm

// Long Range Espressif pour l'ESP-NOW : environ 4 dB de sensibilite en plus (2 a 2,5 fois la
// portee du 802.11b en theorie). Le Wi-Fi des telephones reste normal. Meme reglage sur tous
// les boitiers, sinon ils ne s'entendent plus.
#define RADIO_LONG_RANGE    1         // 1 = Long Range, 0 = standard (1 Mbit/s)
#define RADIO_LR_RATE       250       // 250 (le plus sensible) ou 500 kbit/s

// Antenne par role : 0 = ceramique du XIAO, 1 = U.FL externe.
// Attention : 1 sans antenne branchee = portee quasi nulle.
#define ANT_EXT_START       1
#define ANT_EXT_FINISH      1
#define ANT_EXT_EMITTER     0
#define ANT_EXT_RELAY       1

#define AP_SSID_START       "CHRONO-DEPART"
#define AP_SSID_FINISH      "CHRONO-ARRIVEE"
#define AP_PASSWORD         "chronodh2026"   // mot de passe des Wi-Fi des boitiers, 8 caracteres minimum
#define ADMIN_USER          "admin"           // identifiant de la page /update
#define ADMIN_PASSWORD      "chronoota"       // mot de passe ArduinoOTA (IDE) et page /update

#define HOSTNAME_START      "chrono-depart"   // nom du port reseau dans l'IDE
#define HOSTNAME_FINISH     "chrono-arrivee"
#define HOSTNAME_EMITTER    "chrono-emetteur"
#define HOSTNAME_RELAY      "chrono-relais"

// ============================================================================
// 3. BROCHES (PCB chronodh, XIAO ESP32-C6)
// ============================================================================
#define PIN_BATT        D0    // pont diviseur 100k/100k
#define PIN_BUTTON      D1    // bouton externe (echelle de resistances, lu en analogique)
#define PIN_BUZZER      D2    // AO3400A + MLT-8530 passif
#define PIN_HALL        D3    // DRV5033 (depart) et header TRIG_IN (mode banc)
#define PIN_IR_RX       D4    // TSSP4038 (arrivee)
#define PIN_IR_TX       D9    // grille de l'etage IR (emetteur), maintenue basse sur les autres
#define PIN_RF_ENABLE   3     // interne XIAO : bas = commutateur RF actif
#define PIN_RF_ANT      14    // interne XIAO : bas = ceramique, haut = U.FL

// ============================================================================
// 3b. LORA : module RFM95W (SX1276) 868 MHz sur le connecteur prevu pour l'ecran
//     Cablage : VCC 3V3, GND, D10 MOSI, D8 SCK, D9 MISO, D5 NSS, D6 RESET, D7 DIO0.
//     IMPORTANT : D9 commande aussi l'etage IR. Sur les cartes equipees du module,
//     retirer la resistance de grille (ou le MOSFET) de l'etage IR.
//     Si le module est absent, le boitier le detecte et reste en ESP-NOW seul.
//     Reglementation (ETSI EN 300 220) : sous-bande 869,4 - 869,65 MHz, 500 mW PAR max,
//     10 % du temps en emission max. Le firmware tient ce budget tout seul.
// ============================================================================
#define LORA_START          1         // 1 = le boitier depart a un module LoRa
#define LORA_FINISH         1         // 1 = le boitier arrivee a un module LoRa
#define LORA_FREQ_HZ        869525000UL   // centre de la sous-bande 869,4 - 869,65 MHz
#define LORA_SF             7         // 7 a 12 : +2,5 dB de portee par cran, temps d'emission x2
#define LORA_TX_DBM         14        // 2 a 17, ou 20 (20 dBm : environ 120 mA en emission)
#define LORA_DUTY_PCT       10        // part maximale du temps passee a emettre
#define LORA_SYNC_WORD      0x12      // mot de synchro prive (identique sur les deux boitiers)
#define PIN_LORA_NSS        D5
#define PIN_LORA_RST        D6
#define PIN_LORA_DIO0       D7
#define PIN_SPI_SCK         D8
#define PIN_SPI_MISO        D9
#define PIN_SPI_MOSI        D10

// ============================================================================
// 4. DEPART (baguette)
// ============================================================================
#define HALL_REST_STABLE_MS   500     // sortie haute stable avant de rearmer
#define HALL_LOCKOUT_MS       3000    // ignore le retour de la tige et les rebonds du ressort
#define CANCEL_WINDOW_S       120     // un depart peut etre annule pendant 2 min

// ============================================================================
// 5. ARRIVEE (barriere IR)
// ============================================================================
#define IR_BURST_CYCLES       10      // periodes de porteuse 38 kHz par salve (6 a 40)
#define IR_GAP_US             400     // pause entre deux salves
#define IR_LOST_PERIODS       4       // salves manquees d'affilee = faisceau coupe
#define FINISH_LOCKOUT_MS     1500    // apres un passage : ignore roue arriere et pilote
#define BEAM_REARM_MS         200     // faisceau retabli et stable avant rearmement
#define BEAM_FAULT_MS         3000    // coupure plus longue = alarme faisceau bloque

// Puissance de l'emetteur, niveau 1 (faible) a 10 (pleine). Se regle ensuite depuis la page
// de l'arrivee (Reglages > Puissance) et reste en memoire dans l'emetteur.
// Trop fort = l'IR contourne le pilote par reflexion et la coupure n'est pas vue.
#define IR_POWER_DEFAULT      10      // valeur au tout premier demarrage de l'emetteur
#define IR_CAL_TARGET         95      // reglage auto : % de salves recues pour un faisceau "propre"
#define IR_CAL_MARGIN         2       // reglage auto : niveaux ajoutes au minimum trouve (environ x2)

// Valeurs par defaut, modifiables ensuite dans la page web de l'arrivee
#define DEFAULT_RUNS          2       // manches comptees dans le cumul
#define DEFAULT_MIN_RUN_S     60      // temps mini plausible (sinon passage orphelin)
#define DEFAULT_MAX_RUN_S     900     // au-dela : DNF automatique

// ============================================================================
// 6. CALIBRATION (mesuree avec le mode banc)
// ============================================================================
#define CAL_START_US          0       // retard propre de la detection au depart
#define CAL_FINISH_US         0       // retard propre de la detection a l'arrivee
#define SYNC_OK_UNCERT_US     1000    // au-dela, le temps est marque "synchro degradee"

// ============================================================================
// 7. BATTERIE
// ============================================================================
#define BATT_DIVIDER          2.0f
#define BATT_CAL              1.000f  // correction fine apres mesure au multimetre
#define BATT_WARN_MV          3500
#define BATT_CUTOFF_MV        3300
#define BATT_CUTOFF_ENABLE    1       // deep sleep apres 30 s sous le seuil
#define BATT_ABSENT_MV        2500    // en dessous : pas de batterie (USB seul)

// ============================================================================
// 8. BOUTON ET BUZZER
// ============================================================================
#define BTN_PRESSED_MV        2200    // couvre les 3 positions de l'echelle (0 V, 1,05 V, 1,65 V)
#define BTN_DEBOUNCE_MS       30
#define BTN_LONG_MS           2000
#define BTN_BOOT_HOLD_MS      1500    // maintenu a l'allumage = maintenance
#define BUZ_HI                2700    // resonance du MLT-8530
#define BUZ_LO                1800

// ============================================================================
// 9. DIVERS
// ============================================================================
#define LINK_TIMEOUT_MS       5000
#define LINK_LOST_BEEP        1       // rappel sonore toutes les 30 s si l'arrivee est perdue
#define MAINT_TIMEOUT_MIN     20
#define ALIGN_TIMEOUT_MIN     5
#define DEBUG_SERIAL          1
