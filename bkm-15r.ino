#include <SPI.h>
#include <EthernetENC.h>
#include <WiFi.h>

#define LED (2)

#define RECV_TIMEOUT (1000)

// status word 1
#define POWER_ON_STATUS     (0x8000)
#define SCANMODE_STATUS     (0x0400)
#define HDELAY_STATUS       (0x0200)
#define VDELAY_STATUS       (0x0100)
#define MONOCHROME_STATUS   (0x0080)
#define CHAR_MUTE_STATUS    (0x0040)
#define MARKER_MODE_STATUS  (0x0020)
#define EXTSYNC_STATUS      (0x0010)
#define APT_STATUS          (0x0008)
#define CHROMA_UP_STATUS    (0x0004)
#define ASPECT_STATUS       (0x0002)

// status word 2
// Unknown / unused

// status word 3
#define COL_TEMP_STATUS     (0x0040)
#define COMB_STATUS         (0x0020)
#define BLUE_ONLY_STATUS    (0x0010)
#define R_CUTOFF_STATUS     (0x0004)
#define G_CUTOFF_STATUS     (0x0002)
#define B_CUTOFF_STATUS     (0x0001)

// status word 4
#define MAN_PHASE_STATUS    (0x0080)
#define MAN_CHROMA_STATUS   (0x0040)
#define MAN_BRIGHT_STATUS   (0x0020)
#define MAN_CONTRAST_STATUS (0x0010)

// status word 5
// Unknown / unused

// Status buttons
#define POWER_BUTTON        "POWER"
#define DEGAUSS_BUTTON     "DEGAUSS"

#define SCANMODE_BUTTON     "SCANMODE"
#define HDELAY_BUTTON       "HDELAY"
#define VDELAY_BUTTON       "VDELAY"
#define MONOCHROME_BUTTON   "MONOCHR"
#define APERTURE_BUTTON     "APERTURE"
#define COMB_BUTTON         "COMB"
#define CHAR_OFF_BUTTON     "CHARMUTE"
#define COL_TEMP_BUTTON     "COLADJ"

#define ASPECT_BUTTON       "ASPECT"
#define EXTSYNC_BUTTON      "EXTSYNC"
#define BLUE_ONLY_BUTTON    "BLUEONLY"
#define R_CUTOFF_BUTTON     "RCUTOFF"
#define G_CUTOFF_BUTTON     "GCUTOFF"
#define B_CUTOFF_BUTTON     "BCUTOFF"
#define MARKER_BUTTON       "MARKER"
#define CHROMA_UP_BUTTON    "CHROMAUP"

#define MAN_PHASE_BUTTON    "MANPHASE"
#define MAN_CHROMA_BUTTON   "MANCHR"
#define MAN_BRIGHT_BUTTON   "MANBRT"
#define MAN_CONTRAST_BUTTON "MANCONT"

#define TOGGLE              "TOGGLE"
#define CURRENT             "CURRENT"
#define STATUS_GET          "STATget"
#define STATUS_SET          "STATset"

// Info buttons/knobs
#define INFO_INP_ENTER      "ENTER"
#define INFO_INP_DELETE     "DELETE"
#define INFO_NAV_MENU       "MENU"
#define INFO_NAV_MENUENT    "MENUENT"
#define INFO_NAV_MENUUP     "MENUUP"
#define INFO_NAV_MENUDOWN   "MENUDOWN"

#define INFO_BUTTON         "INFObutton"

#define INFO_KNOB_PHASE         "R PHASE"
#define INFO_KNOB_CHROMA        "R CHROMA"
#define INFO_KNOB_BRIGHTNESS    "R BRIGHTNESS"
#define INFO_KNOB_CONTRAST      "R CONTRAST"

#define INFO_KNOB           "INFOknob"

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

const char* ssid = "<ssid>";
const char* password = "<psk>";

#define MONITOR_PORT (53484)

IPAddress ip(192,168,0,100);
IPAddress monitorIP(192,168,0,1);

const uint8_t get_status[31] = { 0x03,0x0b,0x53,0x4f,0x4e,0x59,0x00,0x00,0x00,0xb0,0x00,0x00,0x12,0x53,0x54,0x41,0x54,0x67,0x65,0x74,0x20,0x43,0x55,0x52,0x52,0x45,0x4e,0x54,0x20,0x35,0x00 };
uint8_t status_response[53];

EthernetClient monitorClient;

WiFiServer webServer(80);

enum Knobs {
    KNOB_PHASE,
    KNOB_CHROMA,
    KNOB_BRIGHT,
    KNOB_CONTRAST
};

typedef struct {
  Knobs m_knob;
  bool m_positive;
  uint8_t m_factor;  
  uint8_t m_value;
} KnobAction_t;

#define MAX_KNOBACTIONS (4)

typedef struct {
  KnobAction_t m_actions[MAX_KNOBACTIONS];  
  uint8_t m_actionCnt;
} KnobActions_t;

KnobActions_t knobActions;

char knobStatus[20];

const char header [13] = { 0x03, 0x0B, 'S', 'O', 'N', 'Y', 0x00, 0x00, 0x00, 0xB0, 0x00, 0x00, 0x00 };

uint8_t button_response[13];
uint8_t packetBuf[40];
int wait_ms = 0;

uint8_t sendInfoButtonPacket(const char* button) {
    uint8_t dataLength = sizeof(header)-1;
    packetBuf[dataLength++] = strlen(INFO_BUTTON) + strlen(button) + 2;
    memcpy(packetBuf+dataLength, INFO_BUTTON, strlen(INFO_BUTTON));
    dataLength += strlen(INFO_BUTTON);
    packetBuf[dataLength++] = 0x20;
    memcpy(packetBuf+dataLength, button, strlen(button));
    dataLength += strlen(button);
    packetBuf[dataLength++] = 0x20;
    if(monitorClient.write(packetBuf, dataLength) == dataLength) {
      wait_ms = 0;
      while(monitorClient.available() < sizeof(button_response)) {
        delay(1);
        ++wait_ms;
        if(wait_ms >= RECV_TIMEOUT) break;
      }
      if(wait_ms >= RECV_TIMEOUT) return 1;

      uint8_t bytesRead = 0;
      while(bytesRead < sizeof(button_response))
        button_response[bytesRead++] = monitorClient.read();
      return 0;
    }
    return 1;
}

uint8_t sendStatusButtonTogglePacket(const char* button) {
    uint8_t dataLength = sizeof(header)-1;
    packetBuf[dataLength++] = strlen(STATUS_SET) + strlen(button) + strlen(TOGGLE) + 2;
    memcpy(packetBuf+dataLength, STATUS_SET, strlen(STATUS_SET));
    dataLength += strlen(STATUS_SET);
    packetBuf[dataLength++] = 0x20;
    memcpy(packetBuf+dataLength, button, strlen(button));
    dataLength += strlen(button);
    packetBuf[dataLength++] = 0x20;
    memcpy(packetBuf+dataLength, TOGGLE, strlen(TOGGLE));
    dataLength += strlen(TOGGLE);
    if(monitorClient.write(packetBuf, dataLength) == dataLength) {
      wait_ms = 0;
      while(monitorClient.available() < sizeof(button_response)) {
        delay(1);
        ++wait_ms;
        if(wait_ms >= RECV_TIMEOUT) break;
      }
      if(wait_ms >= RECV_TIMEOUT) return 1;
      
      uint8_t bytesRead = 0;
      while(bytesRead < sizeof(button_response))
        button_response[bytesRead++] = monitorClient.read();
      return 0;
    }
    return 1;
}

void turnKnob(Knobs knob, int8_t dir, uint8_t ticks) {
    uint8_t dataLength = sizeof(header)-1;
    char *knobstr = NULL;

    snprintf(knobStatus,20,"96/%d/%d",dir,ticks);

    switch(knob) {
        case KNOB_PHASE:
            knobstr = INFO_KNOB_PHASE;
        break;
        case KNOB_CHROMA:
            knobstr = INFO_KNOB_CHROMA;
        break;
        case KNOB_BRIGHT:
            knobstr = INFO_KNOB_BRIGHTNESS;
        break;
        case KNOB_CONTRAST:
        default:
            knobstr = INFO_KNOB_CONTRAST;
        break;
    }

    packetBuf[dataLength++] = strlen(INFO_KNOB) + strlen(knobstr) + strlen(knobStatus) + 2;
    memcpy(packetBuf+dataLength, INFO_KNOB, strlen(INFO_KNOB));
    dataLength += strlen(INFO_KNOB);
    packetBuf[dataLength++] = 0x20;
    memcpy(packetBuf+dataLength, knobstr, strlen(knobstr));
    dataLength += strlen(knobstr);
    packetBuf[dataLength++] = 0x20;
    memcpy(packetBuf+dataLength, knobStatus, strlen(knobStatus));
    dataLength += strlen(knobStatus);
    if(monitorClient.write(packetBuf, dataLength) == dataLength) {
      wait_ms = 0;
      while(monitorClient.available() < sizeof(button_response)) {
        delay(1);
        ++wait_ms;
        if(wait_ms >= RECV_TIMEOUT) break;
      }
      if(wait_ms >= RECV_TIMEOUT) return;
      uint8_t bytesRead = 0;
      while(bytesRead < sizeof(button_response))
        button_response[bytesRead++] = monitorClient.read();
      return;
    }
    return;
}

TaskHandle_t webServerTask;
TaskHandle_t ledTask;

SemaphoreHandle_t state_sem;

typedef struct {
  bool m_linkUp;
  bool m_connected;
  bool m_isValid;
  bool m_isPowered;
  bool m_scanmode;
  bool m_hdelay;
  bool m_vdelay;
  bool m_charmute;
  bool m_monochrome;
  bool m_marker;
  bool m_extsync;
  bool m_apt;
  bool m_chromaup;
  bool m_aspect;

  bool m_coltemp;
  bool m_comb;
  bool m_blueonly;
  bool m_rcutoff;
  bool m_gcutoff;
  bool m_bcutoff;
  bool m_man_phase;
  bool m_man_chroma;
  bool m_man_bright;
  bool m_man_contrast;
} State_t;

typedef struct {
  String m_URL;
  String m_content;
} WebRequest_t;

#define MAX_QUEUELEN (10)
enum CommandType {
  CT_STATUS,
  CT_INFO  
};

typedef struct {
  CommandType m_commandType;
  char m_command[20];
} Command_t;

typedef struct {
  Command_t m_commands[MAX_QUEUELEN];
  uint8_t m_queueLen;  
} CommandQueue_t;

bool statusUpdated = false;

uint16_t statusw1 = 0xFFFF,_statusw1 = 0xFFFF;
uint16_t statusw2 = 0xFFFF,_statusw2 = 0xFFFF;
uint16_t statusw3 = 0xFFFF,_statusw3 = 0xFFFF;
uint16_t statusw4 = 0xFFFF,_statusw4 = 0xFFFF;
uint16_t statusw5 = 0xFFFF,_statusw5 = 0xFFFF;

typedef struct {
  // when was the waiter added
  unsigned long m_addTick_ms;
  // the associated connection
  WiFiClient m_client;  
} StatusWaiter_t;

#define MAX_WAITERS (4)

typedef struct {
  StatusWaiter_t m_waiters[MAX_WAITERS];
  uint8_t m_waiterCnt;  
} StatusWaiters_t;

StatusWaiters_t statusWaiters;

const char toggleURL[]      = "/toggle/";
const char knobURL[]        = "/turnknob/";
const char infoPushURL[]    = "/infopush/";
const char statusGetURL[]   = "/statusget";
const char statusWaitURL[] = "/statuswait";

State_t currentState = { .m_linkUp = false, .m_connected = false, .m_isValid = false, .m_isPowered = false };
State_t stateCopy;
CommandQueue_t commandQueue;

void addToggleButton(String& content, const char* command, const char* name, bool large = false) {
  if(large) {
    content += "<button class='largeButton'";
  } else {    
    content += "<button class='smallButton'";
  }
  content += " id='";
  content += command;
  content += "' onclick='toggle(\"";
  content += command;
  content += "\")'>";
  content += name;
  content += "</button>";
}

void addInfoButton(String& content, const char* command, const char* name, bool large = false) {
  if(large) {
    content += "<button class='largeButton' onclick='infoPush(\"";
  } else {
    content += "<button class='smallButton' onclick='infoPush(\"";    
  }
  content += command;
  content += "\")'>";
  content += name;
  content += "</button>";
}

void addBool(String& content, char* key, bool val, bool last = false) {
    content += "\"";
    content += key;
    content += "\":";
    content += val ? "true" : "false";
    if(!last) content += ",";
    content += "\n";
}

void addStatus(String& content) {
  xSemaphoreTake(state_sem, portMAX_DELAY);
  State_t state = currentState;
  xSemaphoreGive(state_sem);
  content += "{\n";
  content += "\"m_linkUp\":";
  content += state.m_linkUp ? "true" : "false";
  content += ",\n";
  content += "\"m_connected\":";
  content += state.m_connected ? "true" : "false";
  content += ",\n";
  content += "\"m_isValid\":";
  content += state.m_isValid ? "true" : "false";
  content += ",\n";
  content += "\"buttonStates\":{";
  addBool(content,POWER_BUTTON,state.m_isPowered);
  addBool(content,SCANMODE_BUTTON,state.m_scanmode);
  addBool(content,HDELAY_BUTTON,state.m_hdelay);
  addBool(content,VDELAY_BUTTON,state.m_vdelay);
  addBool(content,CHAR_OFF_BUTTON,state.m_charmute);
  addBool(content,MONOCHROME_BUTTON,state.m_monochrome);
  addBool(content,MARKER_BUTTON,state.m_marker);
  addBool(content,EXTSYNC_BUTTON,state.m_extsync);
  addBool(content,APERTURE_BUTTON,state.m_apt);
  addBool(content,CHROMA_UP_BUTTON,state.m_chromaup);
  addBool(content,ASPECT_BUTTON,state.m_aspect);
  addBool(content,COL_TEMP_BUTTON,state.m_coltemp);
  addBool(content,COMB_BUTTON,state.m_comb);
  addBool(content,BLUE_ONLY_BUTTON,state.m_blueonly);
  addBool(content,R_CUTOFF_BUTTON,state.m_rcutoff);
  addBool(content,G_CUTOFF_BUTTON,state.m_gcutoff);
  addBool(content,B_CUTOFF_BUTTON,state.m_bcutoff);
  addBool(content,MAN_PHASE_BUTTON,state.m_man_phase);
  addBool(content,MAN_CHROMA_BUTTON,state.m_man_chroma);
  addBool(content,MAN_BRIGHT_BUTTON,state.m_man_bright);
  addBool(content,MAN_CONTRAST_BUTTON,state.m_man_contrast,true);
  content += "\n}\n";  
  content += "\n}\n";  
}

void addKnob(String& content,char* name) {
  content += "<div class='knobdiv'>\n";
  content += "<input id='";
  content += name;
  content += "-factor";
  content += "' class='knobinput' value='1'/>";
  content += "<input id='";
  content += name;
  content += "' class='knobinput' value='1'/>";
  content += "<button class='knobmod' onclick='turnKnob(\"";
  content += name;
  content += "\",false)'>-</button>\n";
  content += "<button class='knobmod' onclick='turnKnob(\"";
  content += name;
  content += "\",true)'>+</button>" \    
    "</div>\n";
}

uint8_t handleReq(WiFiClient& client, String& url) {
  uint8_t rv = 0;
  if(url == "/") {
    String content;
    content += "<!DOCTYPE HTML>\n";
    content += "<html>\n";
    content += "<head>\n";
    content += "<title>BKM-15R</title>\n";
    content += "<script>\n" \
      "function toggle(button) {\n" \
      "  var xhttp = new XMLHttpRequest();\n" \
      "  xhttp.open('GET', '";
    content += toggleURL;
    content += "'+button, true);\n" \
      "  xhttp.send();\n" \
      "};\n" \
      "function infoPush(button) {\n" \
      "  var xhttp = new XMLHttpRequest();\n" \
      "  xhttp.open('GET', '";
    content += infoPushURL;
    content += "'+button, true);\n" \
      "  xhttp.send();\n" \
      "};\n";
    content += \
      "function setState(currentState) {\n" \
      " let link = document.getElementById('link');\n" \
      " link.innerHTML = currentState.m_linkUp ? 'up' : 'down';\n" \
      " let conn = document.getElementById('connected');\n" \
      " conn.innerHTML = currentState.m_connected ? 'yes' : 'no';\n" \
      " let remotediv = document.getElementById('remotediv');\n" \
      " if(currentState.m_connected) {\n" \
      "   remotediv.style.visibility='visible';\n" \
      " } else {\n" \
      "   remotediv.style.visibility='hidden';\n" \
      " }\n" \          
      " for(const key in currentState.buttonStates) {\n" \
      "   let elem = document.getElementById(key);\n" \
      "   if(elem !== undefined && elem != null) {\n" \
      "     if(currentState.buttonStates[key]) {\n" \
      "       elem.classList.add('active');\n" \
      "     } else {\n" \
      "       elem.classList.remove('active');\n" \
      "     }\n" \
      "   }\n" \
      " }\n" \
      "};\n";
    content += \
      "function updateState() {\n" \
      "  var xhttp = new XMLHttpRequest();\n" \
      "  xhttp.open('GET', '";
    content += statusGetURL;
    content += "', true);\n" \
      "  xhttp.onreadystatechange = function() {\n" \
      "    if (this.readyState == 4 && this.status == 200) {\n" \
      "       let status = JSON.parse(xhttp.response);\n" \
      "       setState(status);\n" \
      "       waitStateUpdate();\n" \
      "    }\n" \
      "  };\n" \
      "  xhttp.send();\n" \
      "};\n";
    content += \
      "function waitStateUpdate() {\n" \
      "  var xhttp = new XMLHttpRequest();\n" \
      "  xhttp.open('GET', '";
    content += statusWaitURL;
    content += "', true);\n" \
      "  xhttp.onreadystatechange = function() {\n" \
      "    if (this.readyState == 4 && this.status == 200) {\n" \
      "       let status = JSON.parse(xhttp.response);\n" \
      "       setState(status);\n" \
      "       waitStateUpdate();\n" \
      "    }\n" \
      "  };\n" \
      "  xhttp.send();\n" \
      "};\n";
    content += \
      "function turnKnob(knob,positive) {\n" \
      "  let val = document.getElementById(knob);\n" \
      "  let factor = document.getElementById(knob+'-factor');\n" \ 
      "  var xhttp = new XMLHttpRequest();\n" \
      "  xhttp.open('GET', '";
    content += knobURL;
    content += "'+knob+'/";
    content +="'+(positive?'p/':'n/')";
    content +="+factor.value+'/'+val.value";
    content += ", true);\n" \
      "  xhttp.send();\n" \
      "};\n";
      content += "window.onload = updateState();\n";
      content +=  "</script>\n";
    content += "<style>\n" \
      ".knobdiv { font-size: 10px; font-weight: bold; width: 50px; vertical-align: top; display: inline-block; }\n" \
      ".knobinput { width: 50px; box-sizing: border-box; }\n" \
      ".knobmod { width: 23px; height: 23px; }\n" \
      ".smallButton { font-size: 10px; font-weight: bold; width: 50px; height: 50px; margin: 2px; vertical-align: top; }\n" \
      ".largeButton { font-size: 10px; font-weight: bold; width: 60px; height: 60px; margin: 5px; vertical-align: top; }\n" \
      ".active { color: white; background-color: limegreen; }\n" \
      ".buttondiv { display: inline-block; margin: 5px; vertical-align: top; }\n" \
      "</style>\n";
    content += "</html>\n";
    content += "<body>\n";
    content += "<div>\n";
    content += "BKM-15R remote\n";
    content += "<br/>";
    content += "Link: <span id='link'></span> ";
    content += "Connected: <span id='connected'></span>";
    content += "<br>";
    content += "<div id='remotediv'>";
    content += "<div class='buttondiv'>";
    content += "<div>";
    addToggleButton(content,SCANMODE_BUTTON,"SCANMODE");
    addToggleButton(content,HDELAY_BUTTON,"H-DLY");
    addToggleButton(content,VDELAY_BUTTON,"V-DLY");
    addToggleButton(content,MONOCHROME_BUTTON,"MONO");
    addToggleButton(content,APERTURE_BUTTON,"APT");
    addToggleButton(content,COMB_BUTTON,"COMB");
    addToggleButton(content,CHAR_OFF_BUTTON,"CHAR OFF");
    addToggleButton(content,COL_TEMP_BUTTON,"COL TEMP");
    content += "</div>";
    content += "<br>";
    content += "<div>";
    addToggleButton(content,ASPECT_BUTTON,"ASPECT");
    addToggleButton(content,EXTSYNC_BUTTON,"EXT SYNC");
    addToggleButton(content,BLUE_ONLY_BUTTON,"BLUE ONLY");
    addToggleButton(content,R_CUTOFF_BUTTON,"R");
    addToggleButton(content,G_CUTOFF_BUTTON,"G");
    addToggleButton(content,B_CUTOFF_BUTTON,"B");
    addToggleButton(content,MARKER_BUTTON,"MARKER");
    addToggleButton(content,CHROMA_UP_BUTTON,"CHROMA UP");
    content += "</div>";
    content += "</div>";
    content += "<div class='buttondiv'>";
    addInfoButton(content,INFO_NAV_MENUUP,"UP",true);
    addInfoButton(content,INFO_NAV_MENU,"MENU",true);
    content += "<br>";
    addInfoButton(content,INFO_NAV_MENUDOWN,"DOWN",true);
    addInfoButton(content,INFO_NAV_MENUENT,"ENTER",true);
    content += "</div>";

    content += "<div class='buttondiv'>";
    addKnob(content,"PHASE");
    addKnob(content,"CHROMA");
    addKnob(content,"BRIGHTNESS");
    addKnob(content,"CONTRAST");
    content += "<br>";
    addToggleButton(content,MAN_PHASE_BUTTON,"MANUAL PHASE");
    addToggleButton(content,MAN_CHROMA_BUTTON,"MANUAL CHROMA");
    addToggleButton(content,MAN_BRIGHT_BUTTON,"MANUAL BRIGHTNESS");
    addToggleButton(content,MAN_CONTRAST_BUTTON,"MANUAL CONTRAST");
    content += "</div>";

    content += "<div class='buttondiv'>";
    addInfoButton(content,"1","1");
    addInfoButton(content,"2","2");
    addInfoButton(content,"3","3");
    addInfoButton(content,INFO_INP_DELETE,"DEL");
    content += "<br>";
    addInfoButton(content,"4","4");
    addInfoButton(content,"5","5");
    addInfoButton(content,"6","6");
    addInfoButton(content,"0","0");
    content += "<br>";
    addInfoButton(content,"7","7");
    addInfoButton(content,"8","8");
    addInfoButton(content,"9","9");
    addInfoButton(content,INFO_INP_ENTER,"ENT");
    content += "</div>";

    content += "<div class='buttondiv'>";
    addToggleButton(content,DEGAUSS_BUTTON,"DEGAUSS");
    content += "<br>";
    addToggleButton(content,POWER_BUTTON,"POWER",true);
    content += "</div>";

    content += "</div>\n";
    content += "</body>\n";
    content += "</html>\n";

    // send a standard http response header
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.print("Content-Length: ");
    client.println(content.length());
    client.println("Connection: close");  // the connection will be closed after completion of the response
    client.println();
    client.println(content);
  } else if(url.startsWith(toggleURL)) {
    xSemaphoreTake(state_sem, portMAX_DELAY);    
    if(commandQueue.m_queueLen+1 < MAX_QUEUELEN) {
      strcpy(commandQueue.m_commands[commandQueue.m_queueLen].m_command,url.substring(strlen(toggleURL)).c_str());
      commandQueue.m_commands[commandQueue.m_queueLen++].m_commandType = CT_STATUS;
    }
    xSemaphoreGive(state_sem);
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: none");
    client.println("Connection: close");
  } else if(url.startsWith(infoPushURL)) {
    xSemaphoreTake(state_sem, portMAX_DELAY);    
    if(commandQueue.m_queueLen+1 < MAX_QUEUELEN) {
      strcpy(commandQueue.m_commands[commandQueue.m_queueLen].m_command,url.substring(strlen(infoPushURL)).c_str());
      commandQueue.m_commands[commandQueue.m_queueLen++].m_commandType = CT_INFO;
    }
    xSemaphoreGive(state_sem);
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: none");
    client.println("Connection: close");
  } else if(url.startsWith(statusGetURL)) {
    String content;
    addStatus(content);
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.print("Content-Length: ");
    client.println(content.length());
    client.println("Connection: close");
    client.println();
    client.println(content);
  } else if(url.startsWith(statusWaitURL)) {
    if(statusWaiters.m_waiterCnt < MAX_WAITERS) {
      StatusWaiter_t& waiter = statusWaiters.m_waiters[statusWaiters.m_waiterCnt];
      waiter.m_client = client;
      waiter.m_addTick_ms = millis();
      statusWaiters.m_waiterCnt++;
      rv = 1;
    } else {
      client.println("HTTP/1.1 503 Service Unavailable");
      client.println("Content-Type: none");
      client.println("Connection: close");
    }
  } else if(url.startsWith(knobURL)) {
    if(knobActions.m_actionCnt < MAX_KNOBACTIONS) {
      try {
        Knobs knob = (Knobs)-1;
        int p = url.indexOf('/',sizeof(knobURL)+1);
        int p2 = url.indexOf('/',p+1);
        int p3 = url.indexOf('/',p2+1);
        if(p != -1 && p2 != -1 && p3 != -1) {
          String knobstr = url.substring(strlen(knobURL),p);
          String posstr = url.substring(p+1,p2);
          String factorstr = url.substring(p2+1,p3);
          String valuestr = url.substring(p3+1);
          bool pos = true;
          
          if(knobstr == "PHASE") {
            knob = KNOB_PHASE;
          } else if (knobstr == "CHROMA") {
            knob = KNOB_CHROMA;
          } else if (knobstr == "BRIGHTNESS") {        
            knob = KNOB_BRIGHT;
          } else if (knobstr == "CONTRAST") {
            knob = KNOB_CONTRAST;
          } else {
            throw false;
          }

          if(posstr == "n") {
            pos = false;
          } else if(posstr == "p") {
            pos = true;
          } else {
            throw false;
          }
          int factor = factorstr.toInt();
          int value = valuestr.toInt();
          if(factor == 0 || value == 0) throw false;
 
          xSemaphoreTake(state_sem, portMAX_DELAY);    
          KnobAction_t& action = knobActions.m_actions[knobActions.m_actionCnt];
          action.m_knob = knob;
          action.m_positive = pos;
          action.m_factor = factor;
          action.m_value = value;
          knobActions.m_actionCnt++;
          xSemaphoreGive(state_sem);
    
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: none");
          client.println("Connection: close");
        } else {
          throw false;
        }
      } catch(const bool ex) {
        Serial.print("Failed deciphering knob request '");
        Serial.println(url);
        client.println("HTTP/1.1 404 Not Found");
        client.println("Connection: close");  // the connection will be closed after completion of the response    
      }      
    } else {
      client.println("HTTP/1.1 503 Service Unavailable");
      client.println("Content-Type: none");
      client.println("Connection: close");
    }      
  } else {
    client.println("HTTP/1.1 404 Not Found");
    client.println("Connection: close");  // the connection will be closed after completion of the response    
  }
  return rv;
}

void webserverHandler( void * pvParameters ){
  bool stopClient = true;
  bool updateWaiters = false;
  while(true) {
    WiFiClient client = webServer.available();
    if (client) {
      // a http request ends with a blank line
      String str;
      WebRequest_t req;
      bool done = false;
      while (client.connected()) {
        while (client.available()) {
          char c = client.read();
          str += c;
          if(c == '\n') {
            if(str == "\r\n") {
              stopClient = (handleReq(client,req.m_URL) == 0);
              done = true;
              break;
            } else {
              if(str.startsWith("GET")) {
                str = str.substring(4,str.indexOf(' ',4));
                req.m_URL = str;
              }
            }
            str = "";
          }
        }
        if(done) break;
        delay(1);
      }
      // close the connection:
      if(stopClient) {
        client.flush();
        client.stop();
      }
    }
    xSemaphoreTake(state_sem, portMAX_DELAY);    
    if(statusUpdated) {
      stateCopy = currentState;
      statusUpdated = false;
      updateWaiters = true;
    }
    xSemaphoreGive(state_sem);
    if(updateWaiters && statusWaiters.m_waiterCnt > 0) {
      String content;
      for(uint8_t w = 0; w < statusWaiters.m_waiterCnt; ++w) {
        WiFiClient waitClient = statusWaiters.m_waiters[w].m_client;
        content = "";
        addStatus(content);
        waitClient.println("HTTP/1.1 200 OK");
        waitClient.println("Content-Type: application/json");
        waitClient.print("Content-Length: ");
        waitClient.println(content.length());
        waitClient.println("Connection: close");
        waitClient.println();
        waitClient.println(content);
        waitClient.flush();
        waitClient.stop();
        delay(10);
      }
      statusWaiters.m_waiterCnt = 0;
    } 
    updateWaiters = false;
    delay(10);
  }
}

void statusLEDBlinker( void * pvParameters ){
  while(true) {
    if(!currentState.m_linkUp) {
      digitalWrite(LED,HIGH);
      delay(100);
      digitalWrite(LED,LOW);
      delay(100);
      digitalWrite(LED,HIGH);
      delay(100);
      digitalWrite(LED,LOW);
      delay(1000);
    } else if(!currentState.m_connected) {
      digitalWrite(LED,HIGH);
      delay(100);
      digitalWrite(LED,LOW);             
      delay(100);
      digitalWrite(LED,HIGH);
      delay(100);
      digitalWrite(LED,LOW);
      delay(100);
      digitalWrite(LED,HIGH);
      delay(100);
      digitalWrite(LED,LOW);
      delay(1000);
    } else {
      digitalWrite(LED,HIGH);
      delay(200);
      digitalWrite(LED,LOW);      
      delay(200);
      digitalWrite(LED,LOW);      
    }
  }
}

void setup() {
  pinMode(LED,OUTPUT);
  Serial.begin(115200);
  Serial.println("Initializing ethernet");
  Ethernet.init(5);
  Ethernet.begin(mac, ip);

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    while (true) {
      digitalWrite(LED,HIGH);
      delay(100);
    }
  }

  state_sem = xSemaphoreCreateMutex();

  WiFi.setHostname("BKM-15R");

  WiFi.begin(ssid, password);

  Serial.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
      digitalWrite(LED,HIGH);
      delay(100);
      digitalWrite(LED,LOW);
      delay(500);
  }
  memcpy(packetBuf,header,sizeof(header));

  currentState.m_linkUp = (Ethernet.linkStatus() == LinkON);
  commandQueue.m_queueLen = 0;
  statusWaiters.m_waiterCnt = 0;
  knobActions.m_actionCnt = 0;

  Serial.println("Starting WebServer");
  webServer.begin();

  Serial.println(WiFi.localIP());
  
  //create a task to run the webserver stuff on core 0, as core 1 is default
  xTaskCreatePinnedToCore(webserverHandler, "Webserver", 10000, NULL, 1, &webServerTask, 0);
  xTaskCreatePinnedToCore(statusLEDBlinker, "LED Blinker", 1000, NULL, 2, &ledTask, 0);
}

unsigned long lastStatusUpdate_ms = 0;

void updateStatus() {
  monitorClient.write(get_status,sizeof(get_status));
  int wait_ms = 0;
  while(monitorClient.available() < sizeof(status_response)) {
    delay(1);
    ++wait_ms;
    if(wait_ms >= RECV_TIMEOUT) break;
  }
  if(wait_ms >= RECV_TIMEOUT) return;
  
  uint8_t bytesRead = 0;
  while(bytesRead < sizeof(status_response))
    status_response[bytesRead++] = monitorClient.read();

  statusw1 =  ((status_response[29] - 0x30) << 12) +
              ((status_response[30] - 0x30) << 8) +
              ((status_response[31] - 0x30) << 4) +
              ((status_response[32] - 0x30));

  statusw2 =  ((status_response[34] - 0x30) << 12) +
              ((status_response[35] - 0x30) << 8) +
              ((status_response[36] - 0x30) << 4) +
              ((status_response[37] - 0x30));

  statusw3 =  ((status_response[39] - 0x30) << 12) +
              ((status_response[40] - 0x30) << 8) +
              ((status_response[41] - 0x30) << 4) +
              ((status_response[42] - 0x30));

  statusw4 =  ((status_response[44] - 0x30) << 12) +
              ((status_response[45] - 0x30) << 8) +
              ((status_response[46] - 0x30) << 4) +
              ((status_response[47] - 0x30));

  statusw5 =  ((status_response[49] - 0x30) << 12) +
              ((status_response[50] - 0x30) << 8) +
              ((status_response[51] - 0x30) << 4) +
              ((status_response[52] - 0x30));

  if(statusw1 != _statusw1 || statusw2 != _statusw2 ||
      statusw3 != _statusw3 || statusw4 != _statusw4 ||
      statusw5 != _statusw5)
  {
      xSemaphoreTake(state_sem, portMAX_DELAY);
      currentState.m_isPowered =  (statusw1 & POWER_ON_STATUS);
      currentState.m_scanmode =   (statusw1 & SCANMODE_STATUS);
      currentState.m_hdelay =     (statusw1 & HDELAY_STATUS);
      currentState.m_vdelay =     (statusw1 & VDELAY_STATUS);
      currentState.m_monochrome = (statusw1 & MONOCHROME_STATUS);
      currentState.m_charmute =   (statusw1 & CHAR_MUTE_STATUS);
      currentState.m_marker =     (statusw1 & MARKER_MODE_STATUS);
      currentState.m_extsync =    (statusw1 & EXTSYNC_STATUS);
      currentState.m_apt =        (statusw1 & APT_STATUS);
      currentState.m_chromaup =   (statusw1 & CHROMA_UP_STATUS);
      currentState.m_aspect =     (statusw1 & ASPECT_STATUS);

      currentState.m_coltemp =    (statusw3 & COL_TEMP_STATUS);
      currentState.m_comb =       (statusw3 & COMB_STATUS);
      currentState.m_blueonly =   (statusw3 & BLUE_ONLY_STATUS);
      currentState.m_rcutoff =    (statusw3 & R_CUTOFF_STATUS);
      currentState.m_gcutoff =    (statusw3 & G_CUTOFF_STATUS);
      currentState.m_bcutoff =    (statusw3 & B_CUTOFF_STATUS);

      currentState.m_man_phase =    (statusw4 & MAN_PHASE_STATUS);
      currentState.m_man_chroma =   (statusw4 & MAN_CHROMA_STATUS);
      currentState.m_man_bright =   (statusw4 & MAN_BRIGHT_STATUS);
      currentState.m_man_contrast = (statusw4 & MAN_CONTRAST_STATUS);
      
      currentState.m_isValid = true;
      statusUpdated = true;
      xSemaphoreGive(state_sem);
      _statusw1 = statusw1;
      _statusw2 = statusw2;
      _statusw3 = statusw3;
      _statusw4 = statusw4;
      _statusw5 = statusw5;
  }  
}

void loop() {
  if(Ethernet.linkStatus() == LinkOFF) {
    if(monitorClient.connected()) monitorClient.stop();
    xSemaphoreTake(state_sem, portMAX_DELAY);
    currentState.m_linkUp = false;
    currentState.m_connected = false;
    currentState.m_isValid = false;
        statusUpdated = true;
    xSemaphoreGive(state_sem);

    Serial.println("Link is down, waiting for it to come up...");
    while (Ethernet.linkStatus() == LinkOFF) {
      delay(500);
    }
    delay(1000); // dunno, apparently give the ethernet a chance to live...

    Serial.println("Link is now up!");

    xSemaphoreTake(state_sem, portMAX_DELAY);
    currentState.m_linkUp = true;
    statusUpdated = true;
    xSemaphoreGive(state_sem);
  }

  if(!monitorClient.connected()) {
    Serial.println("Connecting to monitor...");
    statusw1 = 0xFFFF;
    statusw2 = 0xFFFF;
    statusw3 = 0xFFFF;
    statusw4 = 0xFFFF;
    statusw5 = 0xFFFF;
    _statusw1 = 0xFFFF;
    _statusw2 = 0xFFFF;
    _statusw3 = 0xFFFF;
    _statusw4 = 0xFFFF;
    _statusw5 = 0xFFFF;
    xSemaphoreTake(state_sem, portMAX_DELAY);
    if(currentState.m_connected) {
      currentState.m_connected = false;
      currentState.m_isValid = false;
          statusUpdated = true;
    }
    xSemaphoreGive(state_sem);
    int rv = 0;
    while(rv != 1) {
      rv = monitorClient.connect(monitorIP,MONITOR_PORT);
      if(rv != 1) delay(1000);
    }
    xSemaphoreTake(state_sem, portMAX_DELAY);
    currentState.m_connected = true;
    statusUpdated = true;
    xSemaphoreGive(state_sem);
    Serial.println("Connected, flushing...");
    monitorClient.flush();
    Serial.println("Starting communication...");
    updateStatus();
    lastStatusUpdate_ms = millis();
  } else {
    Command_t* cmd = NULL;
    KnobAction_t* knobAction = NULL;
    xSemaphoreTake(state_sem, portMAX_DELAY);
    if(commandQueue.m_queueLen > 0) {
      cmd = &commandQueue.m_commands[commandQueue.m_queueLen-1];
      commandQueue.m_queueLen--;
    }
    if(knobActions.m_actionCnt > 0) {
      knobAction = &(knobActions.m_actions[knobActions.m_actionCnt-1]);
      knobActions.m_actionCnt--;
    }    
    xSemaphoreGive(state_sem);        

    if(NULL != cmd) {
      switch(cmd->m_commandType) {
        case CT_INFO:
          sendInfoButtonPacket(cmd->m_command);
        break;
        case CT_STATUS:
          sendStatusButtonTogglePacket(cmd->m_command);
        break;
      }
    }

    if(NULL != knobAction) {
      turnKnob(knobAction->m_knob,knobAction->m_factor * (knobAction->m_positive ? 1 : -1),knobAction->m_value);
    }

    if(millis() >= lastStatusUpdate_ms + 150) {
      updateStatus();
      lastStatusUpdate_ms = millis();
    }
  }
  delay(10);
}
