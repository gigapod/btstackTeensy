// A2DP Sink + HFP Hands-Free Example for the BTstackTeensy library.
//
// Plays A2DP audio out over Teensy Audio Library I2S1 (wire an I2S DAC, e.g.
// an SGTL5000 Audio Shield, to hear it -- see README.md). HFP voice audio is
// bridged directly between the module and Teensy over the PCM/I2S2 pins;
// this sketch only handles call control over USB Serial:
//   'a'          answer an incoming call
//   'h'          hang up / reject
//   'd<number>'  dial a number, e.g. "d18005551234"
#include <BTstackTeensy.h>

void setup(void){
    Serial.begin(115200);

    BTstackTeensy.setup();
    BTstackTeensy.enableA2DPSink();
    BTstackTeensy.enableHFPHandsFree();
    BTstackTeensy.setA2DPStreamStateCallback(a2dpStreamStateCallback);
    BTstackTeensy.setHFPEventCallback(hfpEventCallback);

    // Service classes: Audio + Rendering; Major: Audio/Video; Minor: Wearable Headset
    BTstackTeensy.enableClassicDiscoverable("Teensy BT Audio", 0x240404);
}

void loop(void){
    BTstackTeensy.loop();
    pollSerialCommands();
}

void a2dpStreamStateCallback(A2DPStreamState state){
    switch (state){
        case A2DP_STREAM_ESTABLISHED:
            Serial.println("A2DP: stream established");
            break;
        case A2DP_STREAM_STARTED:
            Serial.println("A2DP: stream started");
            break;
        case A2DP_STREAM_SUSPENDED:
            Serial.println("A2DP: stream suspended");
            break;
        case A2DP_STREAM_RELEASED:
            Serial.println("A2DP: stream released");
            break;
    }
}

void hfpEventCallback(HFPEvent event, hci_con_handle_t handle){
    switch (event){
        case HFP_SLC_CONNECTED:
            Serial.println("HFP: connected");
            break;
        case HFP_SLC_DISCONNECTED:
            Serial.println("HFP: disconnected");
            break;
        case HFP_AUDIO_CONNECTED:
            Serial.println("HFP: audio connected");
            break;
        case HFP_AUDIO_DISCONNECTED:
            Serial.println("HFP: audio disconnected");
            break;
        case HFP_START_RINGING:
            Serial.println("HFP: incoming call -- ringing ('a' to answer, 'h' to reject)");
            break;
        case HFP_STOP_RINGING:
            Serial.println("HFP: stopped ringing");
            break;
        case HFP_CALL_ANSWERED:
            Serial.println("HFP: call answered");
            break;
        case HFP_CALL_TERMINATED:
            Serial.println("HFP: call terminated");
            break;
    }
}

void handleSerialLine(const char * line){
    if (line[0] == '\0') return;
    switch (line[0]){
        case 'a':
            Serial.println("> answer");
            BTstackTeensy.hfpAnswer();
            break;
        case 'h':
            Serial.println("> hang up / reject");
            BTstackTeensy.hfpHangup();
            break;
        case 'd':
            Serial.print("> dial ");
            Serial.println(&line[1]);
            BTstackTeensy.hfpDial(&line[1]);
            break;
        default:
            Serial.println("Commands: a=answer, h=hangup/reject, d<number>=dial");
            break;
    }
}

void pollSerialCommands(void){
    static char line_buffer[32];
    static size_t line_length = 0;

    while (Serial.available() > 0){
        char c = (char) Serial.read();
        if (c == '\n' || c == '\r'){
            if (line_length > 0){
                line_buffer[line_length] = '\0';
                handleSerialLine(line_buffer);
                line_length = 0;
            }
        } else if (line_length + 1 < sizeof(line_buffer)){
            line_buffer[line_length++] = c;
        }
    }
}
