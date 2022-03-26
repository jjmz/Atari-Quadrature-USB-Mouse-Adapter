/********************************** (C) COPYRIGHT *******************************
* File Name          : Derived from USBHostHUB_KM.C
* Author             : JJM/WCH
* Version            : V2.3 (Added Hardware v2)
* Date               : 2018/07/24 (WCH) / 2021/02/06 (JJM)
*******************************************************************************/

#include <ch554.h>
#include <debug.h>
#include "usbhost.h"
#include <ch554_usb.h>
#include <stdio.h>
#include <string.h>
#include "small_print.h"

__code uint8_t  SetupGetDevDescr[] = { USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_DEVICE, 0x00, 0x00, sizeof( USB_DEV_DESCR ), 0x00 };
__code uint8_t  SetupGetCfgDescr[] = { USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00 };
__code uint8_t  SetupSetUsbAddr[] = { USB_REQ_TYP_OUT, USB_SET_ADDRESS, USB_DEVICE_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00 };
__code uint8_t  SetupSetUsbConfig[] = { USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
__code uint8_t  SetupSetUsbInterface[] = { USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
__code uint8_t  SetupClrEndpStall[] = { USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP, USB_CLEAR_FEATURE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
__code uint8_t  SetupSetHIDIdle[]= { 0x21,HID_SET_IDLE,0x00,0x00,0x00,0x00,0x00,0x00 };
__code uint8_t  SetupGetHIDDevReport[] = { 0x81, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_REPORT, 0x00, 0x00, 0xFF, 0x00 };
__code uint8_t  GetProtocol[] = { 0xc0,0x33,0x00,0x00,0x00,0x00,0x02,0x00 };

__xdata uint8_t  UsbDevEndp0Size;                                                      
__xdata __at (0x0380) uint8_t  RxBuffer[ MAX_PACKET_SIZE ];                            // IN, must even address
__xdata __at (0x03C0) uint8_t  TxBuffer[ MAX_PACKET_SIZE ];                            // OUT, must even address

__code __at (0x3FF8) uint8_t DevInfo[8];

#define IDX_JOYSTICK 16
#define INC_JOYSTICK 20

#include "joystick-table.h"

__code __at (0x3600) uint8_t FirmwareID[]="JJM Release 01.10 (" __DATE__ ")";


__code __at (0x3700) uint8_t DevTable[256]={  0x02,0x04, 0x08,0x20,			//	Mouse Params (16 bytes)
/*
__code __at (0x3700) uint8_t DevTable[256]={  0x81,0x01, 0x08,0x20,			//	Mouse Params (16 bytes) / AMIGA
*/
                                              0x00,0x00, 0x00,0x00,			// Spare - Future Button 1/2 ?
											  0x00,0x00, 0x00,0x00,			// Spare - Button 3 ?
											  0x00,0x00, 0x00,0x00,			// Spare
// Joystick table - 12 entries, first one is default											  
DEFAULT_JOYSTICK	\
TWINSHOCK			\
EIGHTBITDO_SN30		\
THRUSTMASTER		\
EMPTY				\
EMPTY				\
EMPTY				\
EMPTY				\
EMPTY				\
EMPTY				\
EMPTY				\
EMPTY				
};


uint8_t Set_Port = 0;
__xdata _RootHubDev ThisUsbDev;                                                   //ROOT
__bit FoundNewDev;

typedef struct  {
	uint8_t  xcnt,ycnt;		//2 - pulse counter
	int16_t  xdelta,ydelta;	//4 - increment value for accumulator
	uint16_t xval,yval;		//4 - accumulator value
	uint8_t xph,yph;		//2 - phase
	uint8_t sdiv,smul;		//2 - config : DPI div, counter mult.
	uint8_t minf,maxf;		//2          : min increment, max increment
} mouse_params;             // -> 16b

typedef struct  {
	uint8_t pUidx,pUval;	//2 - Up   [condition+index,value]
	uint8_t pDidx,pDval;	//2 - Down
	uint8_t pLidx,pLval;	//2 - Left
	uint8_t pRidx,pRval;	//2 - Right
	uint8_t p1idx,p1mask;	//2 - Button 1 (L)
	uint8_t p2idx,p2mask;	//2 - Button 2 (R)
	uint8_t p3idx,p3mask;	//2 - button 3 (M)
	uint8_t dummy1,dummy2;  // unused positions
} joystick_params;			// -> 2*8 -> 16b => same as mouse_params

__bit qmouse_mode;
__bit qmouse_amiga;
__bit neg_mvmt,prev_neg_mvmt;

union { mouse_params m;
        joystick_params j; } p;

uint8_t timer=0;

// UP   -> P1_4
// DOWN -> P1_5
// LEFT -> P1_6
// RGHT -> P1_7

SBIT(DirU,0x90,4);
SBIT(DirD,0x90,5);
SBIT(DirL,0x90,6);
SBIT(DirR,0x90,7);


#if HARDWARE==1
#define HARD_V1
SBIT(BUTT_L,0xB0,1);		// P3_1
SBIT(BUTT_R,0xB0,0);		// P3_0
#define INIT_P3 P3        |= 0x03; \
    		    P3_MOD_OC |= 0x03; /* P3.1,0 Open Collector Output */ \
    		    P3_DIR_PU |= 0x03;
#define PWMLED(a)
#else
#define HARD_V2
SBIT(BUTT_R,0xB0,3);		// P3_3 (Right button  - DB9-9)
SBIT(BUTT_L,0xB0,0);		// P3_0 (Left button   - DB9-6) = Fire 
SBIT(BUTT_M,0xB0,4);		// P3_4 (middle button - DB9-5)
SBIT(LED,0xB0,2);		    // P3_2 (LED, active low)
						    // P3_1 available, as UART TX, non-alternate setting
#define INIT_P3 P3        |= 0x19;  \
    		    P3_MOD_OC |= 0x1D;  \
    		    P3_DIR_PU |= 0x19; LED=0;

uint8_t ledcnt=10,ledfsm=0;
uint8_t ledpwm=0;

__code	uint8_t ledstatus[] ={  2,0x80,          // On
                                16, 16+1,0x82,   // On-Off 50% Fast / Restart
                                16,126+1,0x85,	 // 1-Pulse
								16,16+1,16,96+1,0x88 }; // 2-Pulses
#define FSM_NODEV     0
#define FSM_MOUSE     2
#define FSM_JOYSTICK  5
#define FSM_JOYSTICK2 8

#define PWMLED(a) {a}
#endif

//              P1.  4   5   6   7
// Atari ST mouse 	X1 	X0 	Y0 	Y1 	-> Tested OK
// Amiga mouse 	    Y0 	X0 	Y1 	X1 	-> TBC

// If needed - logic analyser - time spent in Timer Interrupt
// SBIT(DEBUGP,0x90,0);

void Timer0_ISR(void) __interrupt (INT_NO_TMR0) __using(1) {
// DEBUGP=1;

	timer++;

#ifdef HARD_V2
	__asm	
		mov a,_ledpwm
		rr a
		mov _ledpwm,a
		rrc a
		cpl C
		mov _LED,C
	__endasm;
	// i.e. rotate ledpwm right &
	// (ledpwm&1)->C , if C=1 {LED=0;} else {LED=1;}
#endif

	if (qmouse_mode)
	{
	 if (p.m.xcnt) {
		p.m.xval+=p.m.xdelta;
		if (((p.m.xval>>8)&0xff)!=p.m.xph) {
		  p.m.xph=p.m.xval>>8;
		  p.m.xcnt--;
		  if (qmouse_amiga)
		  {
		  ACC=p.m.xph;
		  __asm 
			jnb	ACC.1,00000$
			cpl	ACC.0
00000$:
			rrc	A
			mov	0x95,c
			rrc	A
			mov	0x97,c
		  __endasm;
		  }
		  else
		  {
		  ACC=p.m.xph;
		  __asm 
			jnb	ACC.1,00001$
			cpl	ACC.0
00001$:
			rrc	A
			mov	0x95,c
			rrc	A
			mov	0x94,c
		  __endasm;
		  }
		}
	 } else {p.m.xdelta=0;}

	 if (p.m.ycnt) {
		p.m.yval+=p.m.ydelta;
		if (((p.m.yval>>8)&0xff)!=p.m.yph) {
		  p.m.yph=p.m.yval>>8;
		  p.m.ycnt--;
		  if (qmouse_amiga)
		  {
		  ACC=p.m.yph;
		  __asm 
			jnb	ACC.1,00002$
			cpl	ACC.0
00002$:
			rrc	A
			mov	0x94,c
			rrc	A
			mov	0x96,c
		  __endasm;
		  }
		  else
		  {
		  ACC=p.m.yph;
		  __asm 
			jnb	ACC.1,00003$
			cpl	ACC.0
00003$:
			rrc	A
			mov	0x97,c
			rrc	A
			mov	0x96,c
		  __endasm;
		  }
		}
	 } else {p.m.ydelta=0;}
	}

// DEBUGP=0;
}

//if ((v+i)>=0x100) return 255;
#define add_sat255(v,i) { uint8_t s=v+i; if (CY) s=255; v=s; }

inline void xdatacpy(__code uint8_t *src, __data uint8_t *dest,uint8_t nb)
{
	for (uint8_t i=nb;i!=0;i--) *dest++=*src++;
}

#if DE_PRINTF
#define PRINT(a) a
#else
#define PRINT(a)
#endif

void main( )
{
    uint8_t   s, len, endp;
    uint16_t  loc;
    uint8_t i;

    CfgFsys( );	

	// Port 3 initialisation - depends on Hardware Version
	INIT_P3;

    P1 &= 0x0D;			// Was 0x0F - added P1.1 low for Reset Kdb
    P1_MOD_OC &= 0x0D; // P1.4,5,6,7 Push-Pull
    P1_DIR_PU |= 0xF2;

    mDelaymS(50);

#if DE_PRINTF
    mInitSTDIO( );              
#ifdef HARD_V1                                            
    CH554UART0Alter();
#endif	
    printstr( "Start @ChipID=");printhex2(CHIP_ID);printlf();
#endif

    T2MOD &= 0xEF;				// BT0_CLK = Fsys/12 (1333333Hz)
    TMOD = TMOD & 0xF0 | 0x02; 	// Timer 0 mode 1
    TH0 = 0 - 84; TR0 = 1;		// 1333333/84 -> 15873 Hz
    ET0=1;
	P1 |= 0x02;					// Reset KBD
    EA=1;

    InitUSB_Host( );
    FoundNewDev = 0;

#if DE_PRINTF
again:
    printstr( "Wait Device In\n" );

	printstr("Config Data [0..7]: ");
	for ( i = 0; i != 8; i++ ){
		printx2(DevInfo[i]);
	}
	printlf();
#endif

    while ( 1 )
    {

// checkRootHubConnections ?
        s = ERR_SUCCESS;
        if ( UIF_DETECT ){                                                      
            UIF_DETECT = 0;                                                 
            s = AnalyzeRootHub( );                                              
            if ( s == ERR_USB_CONNECT ) FoundNewDev = 1;						
			if ( s == ERR_USB_DISCON ) 	{
                PRINT(printstr( "Disconnect\n");)
				PWMLED(ledfsm=FSM_NODEV;)
				qmouse_mode=0;
			}
        }
        if ( FoundNewDev ){
            FoundNewDev = 0;
//          mDelaymS( 200 );
            s = EnumAllRootDevice( );                                       
            if ( s != ERR_SUCCESS ){						
            PRINT({printstr( "EnumAllRootDev err = ");printhex2(s);printlf();})
            }
			else
			{
				if (ThisUsbDev.DeviceType == DEV_TYPE_MOUSE)
				{
					// SetBootProto();
					
					memset(&p,0,sizeof(p));
					if (DevTable[0]!=0xFF)
	 					{qmouse_amiga=DevTable[0]&0x80?1:0;
					     p.m.sdiv=DevTable[0]&0x7F; p.m.smul=DevTable[1];
	  					 p.m.minf=DevTable[2]; p.m.maxf=DevTable[3];}
					else 
	 					{p.m.sdiv=1; p.m.smul=2; p.m.minf=0; p.m.maxf=250;}

					PRINT({printstr("Mouse Params : "); printx2(p.m.sdiv); printx2(p.m.smul);})
	                PRINT({printx2(p.m.minf); printx2(p.m.maxf); printlf();})

					PWMLED(ledfsm=FSM_MOUSE;)
					qmouse_mode=1;
				}

				if (ThisUsbDev.DeviceType == DEV_TYPE_JOYSTICK)
				{
					qmouse_mode=0;		// Stops Mouse Quadrature in Int
					PWMLED(ledfsm=FSM_JOYSTICK;)

					xdatacpy(&DevTable[IDX_JOYSTICK+4],&p.j.pUidx,16);			// Copy the default entry (first)

					for(i=(IDX_JOYSTICK+INC_JOYSTICK);i!=0;i+=INC_JOYSTICK)		// !!! i from 16+20 to 16+240=>0 (i is uint_8)
					{
					 if (*(uint16_t *)&DevTable[i]==ThisUsbDev.DeviceVID)
					  if (*(uint16_t *)&DevTable[(uint8_t)(i+2)]==ThisUsbDev.DevicePID)
						{								
							PWMLED(ledfsm=FSM_JOYSTICK2;)
							xdatacpy(&DevTable[(uint8_t)(i+4)],&p.j.pUidx,16);
						}					 
					}
					PRINT({printstr("Joystick Params : "); printlf();})
					PRINT({for(i=0;i!=12;i++) printx2((&p.j.pUidx)[i]); printlf();})
				}

			}
        }


// pollHIDdevice ?
		if (timer&0x80)		// 15874/128 -> ~125hz (8ms)
		{
		  timer=0;

#ifdef HARD_V2
 		  ledcnt--;
		  if (!ledcnt)
			 {
			  ledcnt=(ledstatus[ledfsm]&0x7E)*2+2;
			  //if (ledstatus[ledfsm]&1) LED=1; else LED=0;
			  if (ledstatus[ledfsm]&1) ledpwm=0; else ledpwm=0x01;
			  ledfsm++; if (ledstatus[ledfsm]&0x80) ledfsm=ledstatus[ledfsm]&0x7F;
			 };
#endif

		  loc = SearchTypeDevice( DEV_TYPE_MOUSE );                        
		  if ( loc != 0xFFFF ){                                        

			//printstr( "Query Mouse @");printhex4(loc);printlf();

			i = (uint8_t)( loc >> 8 );
			len = (uint8_t)loc;
			SelectHubPort(); 
			endp = /*len ? DevOnHubPort[len-1].GpVar[0] : */ ThisUsbDev.GpVar[0];     
			if ( endp & USB_ENDP_ADDR_MASK ){                               
				s = USBHostTransact( USB_PID_IN << 4 | endp & 0x7F, endp & 0x80 ? bUH_R_TOG | bUH_T_TOG : 0, 0 );

				if ( s == ERR_SUCCESS ){
					endp ^= 0x80;                                         
					// if ( len ) DevOnHubPort[len-1].GpVar[0] = endp;            
					/*else*/ ThisUsbDev.GpVar[0] = endp;
					len = USB_RX_LEN;                                 
					if ( len ) {
						PRINT(printstr("RX: ");)
						PRINT({for ( i = 0; i != len; i ++ ) printx2(RxBuffer[i]); printlf();})

						// TODO : modif depending on mouse button table...
						i=RxBuffer[0];		// 0->Left,1->Right,2->Middle
						if (i&1) BUTT_L=0; else BUTT_L=1;
						if (i&2) BUTT_R=0; else BUTT_R=1;
#ifdef HARD_V2						
						if (i&4) BUTT_M=0; else BUTT_M=1;
#else
						if (i&4) BUTT_L=0; else BUTT_L=1;		// Stephane's request
#endif						
						EA=0;
						int8_t sval;		// Temporary signed value (mouse mvt from -127 to + 127)

						sval=RxBuffer[1];
						if (sval!=0)
						{
						 neg_mvmt=(sval<0); i=!neg_mvmt?sval:-sval;
						 i/=p.m.sdiv; prev_neg_mvmt=(p.m.xdelta<0);
						 if ( p.m.xcnt && ( neg_mvmt^prev_neg_mvmt ) ) // remaining mvt, but dir changed
							{ if (p.m.xcnt>=i) {p.m.xcnt-=i;neg_mvmt=!neg_mvmt;} //more remaining
							              else {p.m.xcnt=(i-p.m.xcnt); }
							}
						 else
						    add_sat255(p.m.xcnt,i);
						  						    
						 uint16_t delta=p.m.minf+p.m.smul*p.m.xcnt;
						 if (delta>=p.m.maxf) delta=p.m.maxf;
						 if (!neg_mvmt) p.m.xdelta=delta; else p.m.xdelta=-delta;						 
						}

						sval=RxBuffer[2];
						if (sval!=0)
						{
						 neg_mvmt=(sval<0); i=!neg_mvmt?sval:-sval;
						 i/=p.m.sdiv; prev_neg_mvmt=(p.m.ydelta<0);
						 if ( p.m.ycnt && ( neg_mvmt^prev_neg_mvmt ) ) // remaining mvt, but dir changed
							{ if (p.m.ycnt>=i) {p.m.ycnt-=i;neg_mvmt=!neg_mvmt;}
							              else {p.m.ycnt=(i-p.m.ycnt); }
							}
						 else						  
						    add_sat255(p.m.ycnt,i);						  
						    
						 uint16_t delta=p.m.minf+p.m.smul*p.m.ycnt;
						 if (delta>=p.m.maxf) delta=p.m.maxf;
						 if (!neg_mvmt) p.m.ydelta=delta; else p.m.ydelta=-delta;						 						 
						}

						EA=1;
						PRINT({for(i=0;i!=16;i++) printx2((&p.m.xcnt)[i]); printlf();})
#if !DE_PRINTF
						mDelayuS(900);
#endif						
					}
				}
				else if ( s != ( USB_PID_NAK | ERR_USB_TRANSFER ) ) {
					PRINT({printstr("Err ");printhex2(s);})
				}
			}
			else {
				PRINT(printstr("Mouse no interrupt endpoint\n");)
			}
			SetUsbSpeed( 1 );                                                 // The default is full speed
		  }

		  loc = SearchTypeDevice( DEV_TYPE_JOYSTICK );                        
		  if ( loc != 0xFFFF ){                                        

			i = (uint8_t)( loc >> 8 );
			len = (uint8_t)loc;
			SelectHubPort(); 
			endp = /* len ? DevOnHubPort[len-1].GpVar[0] : */ ThisUsbDev.GpVar[0];     
			if ( endp & USB_ENDP_ADDR_MASK ){                               
				s = USBHostTransact( USB_PID_IN << 4 | endp & 0x7F, endp & 0x80 ? bUH_R_TOG | bUH_T_TOG : 0, 0 );

				if ( s == ERR_SUCCESS ){
					endp ^= 0x80;                                         
					//if ( len ) DevOnHubPort[len-1].GpVar[0] = endp;            
					/*else*/ ThisUsbDev.GpVar[0] = endp;
					len = USB_RX_LEN;                                 
					if ( len ) {
						PRINT({printstr("RX: "); for ( i = 0; i != len; i ++ ) { printx2(RxBuffer[i]); } printlf();})

// Ugly...

#if DE_PRINTF
 #define CH(b) putch(b);
#else
 #define CH(b)
#endif
#define COND(idx,val,io,c) { cpval=val; i=cond_set_io(idx); if(i==0) {io=0;CH(c);}; if (i==1) io=1; }

// Up/Down/Left/Right
						 COND(p.j.pUidx,p.j.pUval,DirU,'U')
						 COND(p.j.pDidx,p.j.pDval,DirD,'D')
						 COND(p.j.pLidx,p.j.pLval,DirL,'L')
						 COND(p.j.pRidx,p.j.pRval,DirR,'R')
// Buttons
						 COND(p.j.p1idx,p.j.p1mask,BUTT_L,'l')
						 COND(p.j.p2idx,p.j.p2mask,BUTT_R,'r')
#ifdef HARD_V2						 
						 COND(p.j.p3idx,p.j.p3mask,BUTT_M,'m')
#endif

#if !DE_PRINTF
						 mDelayuS(900);
#endif						 
					}
				}
				else if ( s != ( USB_PID_NAK | ERR_USB_TRANSFER ) ) {
					PRINT({printstr("Err ");printhex2(s);})
				}
			}
			else {
				PRINT(printstr("Joystick no interrupt endpoint\n");)
			}
			SetUsbSpeed( 1 );                                                 // The default is full speed
		  }

		}	//End test if timer

    }	// End While
}

