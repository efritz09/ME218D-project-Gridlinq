/****************************************************************************
 
  Header file for the WTV020SD Sparkfun audio breakout framework service

 ****************************************************************************/

#ifndef WTV020SDService_H
#define WTV020SDService_H

// WTV State Machine
typedef enum {
        WTVReady,
        WTVResetting,
        WTVTransferring
} WTVState_t;

// Resetting State Machine
typedef enum {
        ResetPulse,
        ResetDelay
} ResettingState_t;

// Transferring State Machine
typedef enum {
        ClockHigh,
                SendData,
        ClockLow
} TransferringState_t;

// Symbolic defines for playback
#define PLAY_PAUSE          0xFFFE
#define STOP                0xFFFF
#define VOLUME_MIN          0xFFF0
#define VOLUME_MAX          0xFFF7

// Symbolic defines for audio clips
#define THEME_SONG          0x0000  
#define BLOW_US_ALL_TO_SHIT 0x0001  
#define DANGER_ZONE         0x0002  
#define HUGE_FAN_OF_COCK    0x0003  
#define IT_COULD_GO_OFF     0x0004  
#define THE_HELIUM          0x0005  
#define YOULL_KILL_US_ALL   0x0006  
#define RAMPAGE             0x0007  
#define THEME_SONG_ABRIDGED 0x0008  

/*----------------------- Public Function Prototypes ----------------------*/
bool InitWTV( uint8_t Priority );
bool PostWTV( ES_Event ThisEvent );
ES_Event RunWTV( ES_Event );
void PlaySound( uint16_t );

#endif /* WtV020SD_H */
