#include <SPI.h>
#include <EthernetENC.h>
#include <WiFi.h>
#include <Preferences.h>

#define LED                 (2)   // Status LED
#define FORCE_SETUP         (27)  // If low during boot, will make the board enter setup mode

#define RECV_TIMEOUT        (1000)

// Keys for preferences
#define KEY_SSID      "ssid"
#define KEY_PASSWORD  "password"

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
#define DEGAUSS_BUTTON      "DEGAUSS"

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
#define INFO_KNOB           "INFOknob"

#define INFO_KNOB_PHASE         "R PHASE"
#define INFO_KNOB_CHROMA        "R CHROMA"
#define INFO_KNOB_BRIGHTNESS    "R BRIGHTNESS"
#define INFO_KNOB_CONTRAST      "R CONTRAST"

// Global variables
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

bool settingUp = false;

const char contentlengthstr[] = "Content-Length: ";

Preferences preferences;

#define MONITOR_PORT (53484)

IPAddress ip(192,168,0,100);
IPAddress monitorIP(192,168,0,1);

const uint8_t header [13] =     { 0x03, 0x0B, 'S', 'O', 'N', 'Y', 0x00, 0x00, 0x00, 0xB0, 0x00, 0x00, 0x00 };
const uint8_t get_status[31] =  { 0x03, 0x0b, 0x53, 0x4f, 0x4e, 0x59, 0x00, 0x00, 0x00, 0xb0, 0x00, 0x00, 0x12, 0x53, 0x54, 0x41,
                                  0x54, 0x67, 0x65, 0x74, 0x20, 0x43, 0x55, 0x52, 0x52, 0x45, 0x4e, 0x54, 0x20, 0x35, 0x00 };

uint8_t status_response[53];

EthernetClient monitorClient;

WiFiServer webServer(80);

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
    // FIXME no toggle for DEGAUSSS
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

uint8_t sendInfoKnobPacket(const char* knobcmd) {
    uint8_t dataLength = sizeof(header)-1;

    packetBuf[dataLength++] = strlen(INFO_KNOB) + strlen(knobcmd) + 1;
    memcpy(packetBuf+dataLength, INFO_KNOB, strlen(INFO_KNOB));
    dataLength += strlen(INFO_KNOB);
    packetBuf[dataLength++] = 0x20;
    memcpy(packetBuf+dataLength, knobcmd, strlen(knobcmd));
    dataLength += strlen(knobcmd);
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

template <typename T>
class Queue {
public:
  Queue(const uint8_t queueLen) : m_maxCount(queueLen), m_count(0)
  {
    m_queueData = malloc(queueLen*sizeof(T));
    m_head = 0;
    m_tail = 0;
  };

  T* peek() {
    if(m_count == 0) return NULL;    
    return &((T*)m_queueData)[m_tail];;
  };

  T* pop(T* t) {
    if(m_count == 0) return NULL;
    memcpy(t,&((T*)m_queueData)[m_tail],sizeof(T));
    
    ++m_tail;
    --m_count;
    if(m_tail >= m_maxCount) m_tail = 0;
    return t;
  };

  void push(T* t) {
    if(m_count + 1 > m_maxCount) return;
    memcpy(&((T*)m_queueData)[m_head],t,sizeof(T));
    ++m_head;
    ++m_count;
    if(m_head >= m_maxCount) m_head = 0;
  };

  uint8_t getCount() {
     return m_count;
  };
 
private:
  uint8_t m_maxCount;
  uint8_t m_tail;
  uint8_t m_head;
  uint8_t m_count;
  void* m_queueData; 
};

#define MAX_QUEUELEN (8)
enum CommandType {
  CT_STATUS,
  CT_INFO,
  CT_INFOKNOB
};

typedef struct {
  CommandType m_commandType;
  char m_command[25];
} Command_t;

bool statusUpdated = false;

uint16_t statusw1 = 0xFFFF,_statusw1 = 0xFFFF;
uint16_t statusw2 = 0xFFFF,_statusw2 = 0xFFFF;
uint16_t statusw3 = 0xFFFF,_statusw3 = 0xFFFF;
uint16_t statusw4 = 0xFFFF,_statusw4 = 0xFFFF;
uint16_t statusw5 = 0xFFFF,_statusw5 = 0xFFFF;

typedef struct {
  // the associated connection
  WiFiClient m_client;  
  // when was the waiter added
  unsigned long m_added_ms;
} StatusWaiter_t;

#define MAX_WAITERS (2)
// due to smart pointers in wificlient
class WaiterQueue {
public:
  WaiterQueue() : m_maxCount(MAX_WAITERS), m_count(0)
  {
    m_head = 0;
    m_tail = 0;
  };

  StatusWaiter_t* peek() {
    if(m_count == 0) return NULL;    
    return &(m_waiters[m_tail]);
  };

  StatusWaiter_t* pop(StatusWaiter_t& t) {
    if(m_count == 0) return NULL;
    t = m_waiters[m_tail];
    m_waiters[m_tail].m_client.stop();
    ++m_tail;
    --m_count;
    if(m_tail >= m_maxCount) m_tail = 0;
    return &t;
  };

  void push(StatusWaiter_t& t) {
    if(m_count + 1 > m_maxCount) return;
    m_waiters[m_tail] = t;
    ++m_head;
    ++m_count;
    if(m_head >= m_maxCount) m_head = 0;
  };

  uint8_t getCount() {
     return m_count;
  };
 
private:
  uint8_t m_maxCount;
  uint8_t m_tail;
  uint8_t m_head;
  uint8_t m_count;
  StatusWaiter_t m_waiters[MAX_WAITERS];
};

WaiterQueue statusWaiters;

const char toggleURL[]      = "/toggle/";
const char knobURL[]        = "/turnknob/";
const char infoPushURL[]    = "/infopush/";
const char statusGetURL[]   = "/statusget";
const char statusWaitURL[] = "/statuswait";

State_t currentState = { .m_linkUp = false, .m_connected = false, .m_isValid = false, .m_isPowered = false };
State_t stateCopy;

Queue<Command_t> commandQueue(MAX_QUEUELEN);

void addToggleButton(String& content, const char* command, const char* name, bool large = false, bool hasIndicator = true) {
  content += "<div class='togglediv'>\n" \
    "<div class='togglelabel'>";
  content += name;
  content += "\n</div>\n";
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
  if(hasIndicator) {
    content += "<div id='";
    content += command;
    content += "-ind' class='indicator'/>\n";
  }
  content += "</button>" \
    "</div>\n";
}

void addInfoButton(String& content, const char* command, const char* name, bool large = false) {
  content += "<div class='togglediv'>\n" \
    "<div class='togglelabel'>";
  content += name;
  content += "\n</div>\n";
  if(large) {
    content += "<button class='largeButton'";
  } else {    
    content += "<button class='smallButton'";
  }
  content += " id='";
  content += command;
  content += "' onclick='infoPush(\"";
  content += command;
  content += "\")'>";
  content += "</button>" \
    "</div>\n";
}

void addBool(String& content, const char* key, bool val, bool last = false) {
    content += "\"";
    content += key;
    content += "\":";
    content += val ? "true" : "false";
    if(!last) content += ",";
    content += "\n";
}

int statusPacketNo = 0;

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
  content += "\"m_packetNo\":";
  content += statusPacketNo;
  content += ",\n";
  content += "\"buttonStates\":{\n";
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
  content += "}\n";  
  content += "}\n";  
}

void addKnob(String& content, const char* name, const char* command) {
  content += "<div class='knobdiv'>\n";
  
  content += "<div class='togglelabel'>";
  content += name;
  content += "\n</div>\n";

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
  content += "\",true)'>+</button>";

  content += "<br>\n";
  content += "<div class='togglelabel'>";
  content += "MANUAL";
  content += "\n</div>\n";

  content += "<button class='smallButton'";
  content += " id='";
  content += command;
  content += "' onclick='toggle(\"";
  content += command;
  content += "\")'>";
  content += "<div id='";
  content += command;
  content += "-ind' class='indicator'/>\n";
  content += "</button>\n";
  
  content += "\n</div>\n";
}

void handleReq(WiFiClient& wlclient, String& url) {
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
      "   let elem = document.getElementById(key+'-ind');\n" \
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
      "                   xhttp.timeout = 60000;\n" \
      "  xhttp.onreadystatechange = function() {\n" \
      "    if (this.readyState == 4) {\n" \
      "       if(this.status == 200) {\n" \
      "         let status = JSON.parse(xhttp.response);\n" \
      "         setState(status);\n" \
      "         waitStateUpdate();\n" \
      "       } else if (this.status == 503) {\n" \
      "         waitStateUpdate();\n" \
      "       } else if (this.status == 503) {\n" \
      "         setTimeout(waitStateUpdate,500);\n" \
      "       }\n" \
      "       console.log(xhttp.response)\n" \
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
      "body { font-family: sans-serif; }\n" \
      ".knobdiv { font-size: 10px; font-weight: bold; width: 50px; color: white; vertical-align: top; text-align: center;display: inline-block; margin: 10px; }\n" \
      ".knobinput { width: 50px; box-sizing: border-box; }\n" \
      ".knobmod { width: 23px; height: 23px; }\n" \
      "#remotediv { border: solid 2px black; background-color: gray; padding: 10px; width: max-content; }\n" \
      ".togglediv { font-size: 10px;font-weight: bold; color: white; vertical-align: top; text-align: center; display: inline-block; }\n" \
      ".smallButton { background-color: lightgray; width: 50px; height: 50px; margin: 2px; vertical-align: top; border-width: 1px; border-radius: 5px; }\n" \
      ".largeButton { background-color: lightgray; width: 60px; height: 60px; margin: 5px; vertical-align: top; border-width: 1px; border-radius: 5px; }\n" \
      ".powerdiv { text-align: center; }\n" \
      ".indicator { height: 10px; width: 10px; border-radius: 5px; background-color: black; margin: auto; }\n" \
      ".active { background-color: limegreen; }\n" \
      ".buttondiv { display: inline-block; margin: 5px; vertical-align: top; }\n" \
      "</style>\n";
    content += "</head>\n";
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
    addKnob(content,"PHASE",MAN_PHASE_BUTTON);
    addKnob(content,"CHROMA",MAN_CHROMA_BUTTON);
    addKnob(content,"BRIGHTNESS",MAN_BRIGHT_BUTTON);
    addKnob(content,"CONTRAST",MAN_CONTRAST_BUTTON);
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

    content += "<div class='buttondiv powerdiv'>";
    addToggleButton(content,DEGAUSS_BUTTON,"DEGAUSS",false,false);
    content += "<br>";
    addToggleButton(content,POWER_BUTTON,"POWER",true);
    content += "</div>";

    content += "</div>\n";
    content += "</body>\n";
    content += "</html>\n";

    // send a standard http response header
    wlclient.println("HTTP/1.1 200 OK");
    wlclient.println("Content-Type: text/html");
    wlclient.print("Content-Length: ");
    wlclient.println(content.length());
    wlclient.println("Connection: close");
    wlclient.println();
    wlclient.println(content);
  } else if(url.startsWith(toggleURL)) {
    xSemaphoreTake(state_sem, portMAX_DELAY);
    if(commandQueue.getCount() < MAX_QUEUELEN) {
      Command_t cmd;
      strcpy(cmd.m_command,url.substring(strlen(toggleURL)).c_str());
      cmd.m_commandType = CT_STATUS;
      commandQueue.push(&cmd);
    }
    xSemaphoreGive(state_sem);
    wlclient.println("HTTP/1.1 200 OK");
    wlclient.println("Content-Type: none");
    wlclient.println("Connection: close");
    wlclient.println();
  } else if(url.startsWith(infoPushURL)) {
    xSemaphoreTake(state_sem, portMAX_DELAY);    
    if(commandQueue.getCount() < MAX_QUEUELEN) {
      Command_t cmd;
      strcpy(cmd.m_command,url.substring(strlen(infoPushURL)).c_str());
      cmd.m_commandType = CT_INFO;
      commandQueue.push(&cmd);
    }
    xSemaphoreGive(state_sem);
    wlclient.println("HTTP/1.1 200 OK");
    wlclient.println("Content-Type: none");
    wlclient.println("Connection: close");
    wlclient.println();
  } else if(url.startsWith(statusGetURL)) {
    String content;
    addStatus(content);
    statusPacketNo++;
    wlclient.println("HTTP/1.1 200 OK");
    wlclient.println("Content-Type: application/json");
    wlclient.print("Content-Length: ");
    wlclient.println(content.length());
    wlclient.println("Connection: close");
    wlclient.println();
    wlclient.println(content);
  } else if(url.startsWith(statusWaitURL)) {
    if(statusWaiters.getCount() < MAX_WAITERS) {
      StatusWaiter_t waiter;
      waiter.m_added_ms = millis();
      waiter.m_client = wlclient;
      statusWaiters.push(waiter);    
    } else {
      wlclient.println("HTTP/1.1 503 Service Unavailable");
      wlclient.println("Content-Type: none");
      wlclient.println("Connection: close");
      wlclient.println();
    }
  } else if(url.startsWith(knobURL)) {
    try {
      int p = url.indexOf('/',sizeof(knobURL)+1);
      int p2 = url.indexOf('/',p+1);
      int p3 = url.indexOf('/',p2+1);
      if(p != -1 && p2 != -1 && p3 != -1) {
        String knobstr = url.substring(strlen(knobURL),p);
        String posstr = url.substring(p+1,p2);
        String factorstr = url.substring(p2+1,p3);
        String valuestr = url.substring(p3+1);
        bool pos = true;
        
        if(knobstr != "PHASE" &&
           knobstr != "CHROMA" &&
           knobstr != "BRIGHTNESS" &&
           knobstr != "CONTRAST")
        {
          Serial.print("Unknown knob: '");
          Serial.print(knobstr);
          Serial.println("'");
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

        Command_t cmd;
        snprintf(cmd.m_command,20,"R %s 96/%s%d/%d",knobstr,pos ? "" : "-" , factor,value);
        cmd.m_commandType = CT_INFOKNOB;
     
        bool full = false;
        xSemaphoreTake(state_sem, portMAX_DELAY);    
        if(commandQueue.getCount() < MAX_QUEUELEN) {
          commandQueue.push(&cmd);
        } else {
          full = true;
        }
        xSemaphoreGive(state_sem);
        if(!full) {
          wlclient.println("HTTP/1.1 200 OK");
          wlclient.println("Content-Type: none");
          wlclient.println("Connection: close");
          wlclient.println();
        } else {
          wlclient.println("HTTP/1.1 503 Service Unavailable");
          wlclient.println("Content-Type: none");
          wlclient.println("Connection: close");
          wlclient.println();
        }
      } else {
        throw false;
      }
    } catch(const bool ex) {
      Serial.print("Failed deciphering knob request '");
      Serial.println(url);
      wlclient.println("HTTP/1.1 404 Not Found");
      wlclient.println("Connection: close");
      wlclient.println();    
    }            
  } else {
    wlclient.println("HTTP/1.1 404 Not Found");
    wlclient.println("Connection: close");
    wlclient.println();    
  }
}

bool updateWaiters = false;

void checkStatusNotification(){
  xSemaphoreTake(state_sem, portMAX_DELAY);    
  if(statusUpdated) {
    stateCopy = currentState;
    statusUpdated = false;
    updateWaiters = true;
  }
  xSemaphoreGive(state_sem);

  if(statusWaiters.getCount()) {
    StatusWaiter_t w;
    if(updateWaiters) {
      String content;
      content = "";
      addStatus(content);
      statusPacketNo++;
      while(statusWaiters.getCount() > 0) {
        if(statusWaiters.pop(w) != NULL) {
          w.m_client.println("HTTP/1.1 200 OK");
          w.m_client.println("Content-Type: application/json");
          w.m_client.print("Content-Length: ");
          w.m_client.println(content.length());
          w.m_client.println("Connection: close");
          w.m_client.println();
          w.m_client.println(content);
          w.m_client.stop();
        }
      }
      updateWaiters = false;
    } else {
      // go through waiters reset
      unsigned long ms = millis();
      while(statusWaiters.peek() != NULL) {
          if((ms - statusWaiters.peek()->m_added_ms) < 60000) {
            break;
          }
          StatusWaiter_t w;
          if(statusWaiters.pop(w) != NULL) {
            Serial.println("Killing waiter");
            w.m_client.println("HTTP/1.1 204 No Content");
            w.m_client.println("Content-Type: none");
            w.m_client.println("Connection: close");
            w.m_client.println();
            w.m_client.stop();
            Serial.println("Killed waiter");
          }
       }
    }
  }
}

void webserverHandler( void * pvParameters ) {

  int contentLength = 0;
  while(true) {
    if(webServer.hasClient()) {
      WiFiClient cl = webServer.available();
      if (cl) {
        bool done = false;
        contentLength = 0;
        String str;
        WebRequest_t req;
        unsigned long start_ms;
        while (!done) {
          while (cl.available()) {
            char c = cl.read();
            str += c;
            if(c == '\n') {
              if(str == "\r\n") {
                if(contentLength != 0) {
                  int bytesRead = 0;
                  while (cl.available() < contentLength) delay(1);
                  while(bytesRead < contentLength) {
                    c = cl.read();
                    req.m_content += c;
                    ++bytesRead;
                  }
                }
                handleReq(cl,req.m_URL);
                done = true;
                break;
              } else {
                if(str.startsWith("GET")) {
                  str = str.substring(4,str.indexOf(' ',4));
                  req.m_URL = str;
                } else if(str.startsWith(contentlengthstr)) {
                  req.m_content = "";
                  String cl = str.substring(strlen(contentlengthstr),str.indexOf('\r'));
                  contentLength = cl.toInt(); 
                }
              }
              str = "";
            }
          }
          if(done) break;
            delay(1);
        }
        cl.stop();
      }
    }
    checkStatusNotification();
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

IPAddress ap_ip(192,168,4,1);
IPAddress ap_gw(192,168,4,1);
IPAddress ap_subnet(255,255,255,0);
 
void setup() {
  pinMode(LED,OUTPUT);
  pinMode(FORCE_SETUP,INPUT_PULLUP);
 
  Serial.begin(115200);

  Serial.println("\n\nBKM-15Rduino");
  Serial.println("(2022) Martin Hejnfelt (martin@hejnfelt.com)");
  Serial.println("www.immerhax.com\n\n");

  String ssid = "";
  String password = "";

  if(digitalRead(FORCE_SETUP) == LOW) {
    Serial.println("Forcing setup");
    settingUp = true;
  } else {
    preferences.begin("credentials",true);
    if(!preferences.isKey(KEY_SSID)) {
      Serial.println("No credentials exist");
      settingUp = true;
    } else {
      ssid = preferences.getString(KEY_SSID,"");
      if(ssid == "") settingUp = true;
      password = preferences.getString(KEY_PASSWORD,"");
    }
    preferences.end();
  }

  if(settingUp) {
    Serial.println("Starting Setup Access Point:");
    Serial.print("Config: ");    
    Serial.println(WiFi.softAPConfig(ap_ip, ap_gw, ap_subnet) ? "OK" : "Failed!");
    Serial.print("Startup: ");    
    Serial.println(WiFi.softAP("BKM-15R-Setup", "adminadmin", 6, false) ? "OK" : "Failed!");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("MAC: ");
    Serial.println(WiFi.softAPmacAddress());
    
    webServer.begin();
  } else {
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

    WiFi.begin(ssid.c_str(), password.c_str());
    
    Serial.print("Connecting to SSID '");
    Serial.print(ssid);
    Serial.println("'");
    while (WiFi.status() != WL_CONNECTED) {
        digitalWrite(LED,HIGH);
        delay(100);
        digitalWrite(LED,LOW);
        delay(500);
    }
    memcpy(packetBuf,header,sizeof(header));
    
    currentState.m_linkUp = (Ethernet.linkStatus() == LinkON);
    
    Serial.println("Starting WebServer");
    webServer.begin();
    
    Serial.println(WiFi.localIP());

    //create a task to run the webserver stuff on core 0, as core 1 is default
    xTaskCreatePinnedToCore(webserverHandler, "Webserver", 40000, NULL, 1, &webServerTask, 0);
    xTaskCreatePinnedToCore(statusLEDBlinker, "LED Blinker", 1000, NULL, 2, &ledTask, 0);
  }
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

void connectMonitor() {
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
}

void waitLink() {
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

void addWifiNetworks() {
  int ssidsFound = WiFi.scanNetworks();
  if (ssidsFound == -1) {
    Serial.println("Failure scanning for WiFi networks");
  } else {
    // print the list of networks seen:
    Serial.print("Found ");
    Serial.print(ssidsFound);
    Serial.println(" SSID(s)");
  
    // print the network number and name for each network found:
    for (int ssidNo = 0; ssidNo < ssidsFound; ssidNo++) {
      Serial.print(ssidNo);
      Serial.print(") ");
      Serial.print(WiFi.SSID(ssidNo));
      Serial.print("\tSignal: ");
      Serial.print(WiFi.RSSI(ssidNo));
      Serial.print(" dBm");
    }
  }
}

void setupSystem() {
  WiFiClient setupClient = webServer.available();
  if (setupClient) {
    // a http request ends with a blank line
    String str;
    int contentLength = 0;
    WebRequest_t req;
    bool done = false;
    Serial.println("Setup client connected\n");
    while (setupClient.connected()) {
      while (setupClient.available()) {
        char c = setupClient.read();
        str += c;
        if(c == '\n') {
          if(str == "\r\n") {
            if(contentLength != 0) {
              int bytesRead = 0;
              while (setupClient.available() < contentLength) delay(1);
              while(bytesRead < contentLength) {
                c = setupClient.read();
                req.m_content += c;
                ++bytesRead;
              }
            }
            if(req.m_URL == "/") {
              String content;
              content += "<!DOCTYPE HTML>\n";
              content += "<html>\n";
              content += "<head>\n";
              content += "<title>BKM-15R Setup</title>\n";
              content += "<script>\n";
              content += \
                "function sendSetup() {\n" \
                "  let msg = '';\n" \
                "  let ssidelem = document.getElementById('wifissid');\n" \
                "  msg = 'ssid='+ssidelem.value+'\\r\\n'\n" \
                "  let pskelem = document.getElementById('wifipsk');\n" \
                "  msg += 'psk='+pskelem.value+'\\r\\n'\n" \
                "  var xhttp = new XMLHttpRequest();\n" \
                "  xhttp.onreadystatechange = function() {\n" \
                "    if (this.readyState == 4) {\n" \
                "      if(this.status == 200) { \n" \
                "        alert('OK');\n" \
                "      } else if (this.status == 400) {\n" \
                "        alert('Not accepted, try again...');\n" \
                "      }\n" \
                "    }\n" \
                "  };\n" \
                "  xhttp.open('POST', '";
              content += "/setup";
              content += "', true);\n" \
                "  xhttp.setRequestHeader('Content-type', 'application/json');\n" \
                "  xhttp.send(msg);\n" \
                "};\n";
              content += "</script>\n";
              content += "<body>\n";
              content += "BKM-15R remote setup<br>\n";
              content += "<span>SSID:</span><input id='wifissid'><br>\n";
              content += "<span>PSK:</span><input id='wifipsk' type='password'><br>\n";
              content += "<button onclick='sendSetup()'>Send</button><br>\n";
              content += "<p>Pressing 'Send' will save credentials, and restart...</p>\n";
              content += "</body>\n";
              content += "</html>\n";
              // send a standard http response header
              setupClient.println("HTTP/1.1 200 OK");
              setupClient.println("Content-Type: text/html");
              setupClient.print(contentlengthstr);
              setupClient.println(content.length());
              setupClient.println("Connection: close");
              setupClient.println();
              setupClient.println(content);
            } else if (req.m_URL == "/setup") {
              int b = req.m_content.indexOf('=');
              int p = req.m_content.indexOf("\r\n",b+1);
              String ssid = req.m_content.substring(b+1,p); 
              b = req.m_content.indexOf('=',p+1);
              p = req.m_content.indexOf("\r\n",b+1);
              String psk = req.m_content.substring(b+1,p); 
              if(ssid != "") {
                setupClient.println("HTTP/1.1 200 OK");
                setupClient.println("Content-Type: none");
                setupClient.println("Connection: close");
                preferences.begin("credentials",false);
                preferences.putString(KEY_SSID,ssid);
                preferences.putString(KEY_PASSWORD,psk);
                preferences.end();                
                delay(1000);
                ESP.restart();
              } else {
                setupClient.println("HTTP/1.1 400 Bad Request");
                setupClient.println("Content-Type: none");
                setupClient.println("Connection: close");                
              }
            } else {
              setupClient.println("HTTP/1.1 404 Not Found");
              setupClient.println("Connection: close");   
            }
            done = true;
            break;
          } else {
            if(str.startsWith(contentlengthstr)) {
              req.m_content = "";
              String cl = str.substring(strlen(contentlengthstr),str.indexOf('\r'));
              contentLength = cl.toInt();
            }
            if(str.startsWith("GET")) {
              str = str.substring(4,str.indexOf(' ',4));
              req.m_URL = str;
            }
            if(str.startsWith("POST")) {
              str = str.substring(5,str.indexOf(' ',5));
              req.m_URL = str;
            }
          }
          str = "";
        }
      }
      if(done) break;
      delay(1);
    }
    Serial.println("Disconnecting setup client");
    setupClient.flush();
    setupClient.stop();
  }
}

void loop() {
  if(!settingUp) {
    if(Ethernet.linkStatus() == LinkOFF) waitLink();
    
    if(!monitorClient.connected()) {
      connectMonitor();
    } else {
      Command_t c;
      Command_t* cmd = NULL;
      
      xSemaphoreTake(state_sem, portMAX_DELAY);
      cmd = commandQueue.pop(&c);
      xSemaphoreGive(state_sem);        
    
      if(NULL != cmd) {
        switch(cmd->m_commandType) {
          case CT_INFO:
            sendInfoButtonPacket(cmd->m_command);
          break;
          case CT_STATUS:
            sendStatusButtonTogglePacket(cmd->m_command);
          break;
          case CT_INFOKNOB:
            sendInfoKnobPacket(cmd->m_command);
          break;
        }
      }
    
      if(millis() - lastStatusUpdate_ms >= 150) {
        updateStatus();
        lastStatusUpdate_ms = millis();
      }
    }
    delay(10);
  } else {
    setupSystem();
  }
}
