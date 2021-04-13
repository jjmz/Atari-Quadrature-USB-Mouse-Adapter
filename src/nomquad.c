/********************************** (C) COPYRIGHT *******************************
* File Name          : Derived from USBHostHUB_KM.C
* Author             : JJM/WCH
* Version            : V2.0
* Date               : 2018/07/24 / 2020/11/02
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
__code uint8_t  SetupGetHubDescr[] = { HUB_GET_HUB_DESCRIPTOR, HUB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_HUB, 0x00, 0x00, sizeof( USB_HUB_DESCR ), 0x00 };
__code uint8_t  SetupSetHIDIdle[]= { 0x21,HID_SET_IDLE,0x00,0x00,0x00,0x00,0x00,0x00 };
__code uint8_t  SetupGetHIDDevReport[] = { 0x81, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_REPORT, 0x00, 0x00, 0xFF, 0x00 };
__code uint8_t  GetProtocol[] = { 0xc0,0x33,0x00,0x00,0x00,0x00,0x02,0x00 };

__xdata uint8_t  UsbDevEndp0Size;                                                      
__xdata __at (0x0380) uint8_t  RxBuffer[ MAX_PACKET_SIZE ];                            // IN, must even address
__xdata __at (0x03C0) uint8_t  TxBuffer[ MAX_PACKET_SIZE ];                            // OUT, must even address

__code __at (0x3FF8) uint8_t DevInfo[8];

uint8_t Set_Port = 0;

__xdata _RootHubDev ThisUsbDev;                                                   //ROOT
__xdata _DevOnHubPort DevOnHubPort[HUB_MAX_PORTS];                               

__bit FoundNewDev;

typedef struct  {
	uint8_t  xcnt,ycnt;		//2
	int16_t  xdelta,ydelta;	//4
	uint16_t xval,yval;		//4
	uint8_t xph,yph;		//2
	uint8_t sdiv,smul;		//2
	uint8_t minf,maxf;		//2 -> 16b
} mouse_params;

typedef struct  {
	uint8_t pUidx,pUval;	//2
	uint8_t pDidx,pDval;
	uint8_t pLidx,pLval;
	uint8_t pRidx,pRval;
	uint8_t p1idx,p1mask;
	uint8_t p2idx,p2mask;	// -> 2*6 -> 12b
} joystick_params;

__bit qmouse_mode;

union { mouse_params m;
        joystick_params j; } p;

uint8_t timer=0;


SBIT(BUTT0,0xB0,1);		// P3_1
SBIT(BUTT1,0xB0,0);		// P3_0

// UP   -> P1_4
// DOWN -> P1_5
// LEFT -> P1_6
// RGHT -> P1_7
// FIRE -> P3_0

SBIT(DirU,0x90,4);
SBIT(DirD,0x90,5);
SBIT(DirL,0x90,6);
SBIT(DirR,0x90,7);
SBIT(Fire,0xB0,0);

void Timer0_ISR(void) __interrupt (INT_NO_TMR0) __using(1) {
	timer++;

	if (qmouse_mode)
	{
	 if (p.m.xcnt) {
		p.m.xval+=p.m.xdelta;
		if (((p.m.xval>>8)&0xff)!=p.m.xph) {
		  p.m.xph=p.m.xval>>8;
		  p.m.xcnt--;
		  ACC=p.m.xph;
		  __asm 
			jnb	ACC.1,00000$
			cpl	ACC.0
00000$:
			rrc	A
			mov	0x95,c
			rrc	A
			mov	0x94,c
		  __endasm;
		}
	 } else {p.m.xdelta=0;}

	 if (p.m.ycnt) {
		p.m.yval+=p.m.ydelta;
		if (((p.m.yval>>8)&0xff)!=p.m.yph) {
		  p.m.yph=p.m.yval>>8;
		  p.m.ycnt--;
		  ACC=p.m.yph;
		  __asm 
			jnb	ACC.1,00001$
			cpl	ACC.0
00001$:
			rrc	A
			mov	0x97,c
			rrc	A
			mov	0x96,c
		  __endasm;
		}
	 } else {p.m.ydelta=0;}
	}
}

uint8_t ReadData(uint8_t addr)
{
	ROM_ADDR_H = DATA_FLASH_ADDR >> 8;
	ROM_ADDR_L = 2*addr;
	ROM_CTRL = ROM_CMD_READ;
	return ROM_DATA_L;
}

//#define DEFAULT_CONFIG

#ifdef DEFAULT_CONFIG
__code uint8_t  DefaultConfig[]= { 0x02,0x04,0x08,0x20,			//Mouse Params
                                   0x0F,0x00,0x00,0x00,0x00,	//Default Joystick
								        0x11,0x40,0x21,0xC0,
										0x10,0x40,0x20,0xC0,
										0x33,0xFF,
								   0x0F,0x00,0x79,0x00,0x06,	//DragonRise Inc. - PC TWIN SHOCK Gamepad
								        0x11,0x40,0x21,0xC0,	// > <
										0x10,0x40,0x20,0xC0,	// > <
										0x36,0xFF,				// &
								   0x0F,0x2D,0xC8,0x60,0x01,	// 8bitdo / SN30 Pro
								        0x14,0x40,0x24,0xC0,	// > <
										0x13,0x40,0x23,0xC0,	// > <
										0x30,0xFF,				// &
								   0xFF};						// End

void WriteDataDef(void)
{
	uint8_t i;

	SAFE_MOD=0x55; SAFE_MOD=0xAA; GLOBAL_CFG |= bDATA_WE; SAFE_MOD=0;

	for(i=0;i!=sizeof(DefaultConfig);i++)
	{
			ROM_ADDR_H = DATA_FLASH_ADDR >> 8;
			ROM_ADDR_L = i*2;
			ROM_DATA_L = DefaultConfig[i];
			ROM_CTRL = ROM_CMD_WRITE;			
	}

	SAFE_MOD=0x55; SAFE_MOD=0xAA; GLOBAL_CFG &= ~bDATA_WE; SAFE_MOD=0;

}
#endif

void main( )
{
    uint8_t   s, len, endp;
    uint16_t  loc;
    uint8_t i;

    CfgFsys( );	

    P3 |= 0x03;
    P3_MOD_OC |= 0x03; // P3.1,0 Open Collector Output
    P3_DIR_PU |= 0x03;

    P1 &= 0x0F;
    P1_MOD_OC &= 0x0F; // P1.4,5,6,7 Push-Pull
    P1_DIR_PU |= 0xF0;

    mDelaymS(50);

#if DE_PRINTF
    mInitSTDIO( );                                                          
    CH554UART0Alter();
    printstr( "Start @ChipID=");printhex2(CHIP_ID);printlf();
#endif

    T2MOD &= 0xEF;			// BT0_CLK = Fsys/12 (1333333Hz)
    TMOD = TMOD & 0xF0 | 0x02; 		// Timer 0 mode 1
    TH0 = 0 - 84; TR0 = 1;		// 1333333/84 -> 15873 Hz
    ET0=1;
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

	printstr("Data Flash [0/2/4/..14]: ");
	for ( i = 0; i != 16; i++ ){
			s=ReadData(i);
			printx2(s);
	}
	printlf();
#endif

#ifdef DEFAULT_CONFIG
	for( i=0 ; i != 4 ; i++)
	  if ( ReadData(i)!=DefaultConfig[i] )
		{ WriteDataDef(); break; }
#endif

    while ( 1 )
    {

// checkRootHubConnections ?
        s = ERR_SUCCESS;
        if ( UIF_DETECT ){                                                      
            UIF_DETECT = 0;                                                 
            s = AnalyzeRootHub( );                                              
            if ( s == ERR_USB_CONNECT ) FoundNewDev = 1;						
        }
        if ( FoundNewDev ){
            FoundNewDev = 0;
//          mDelaymS( 200 );
            s = EnumAllRootDevice( );                                       
            if ( s != ERR_SUCCESS ){						
#if DE_PRINTF
                printstr( "EnumAllRootDev err = ");printhex2(s);printlf();					
#endif
            }
			else
			{
				if (ThisUsbDev.DeviceType == DEV_TYPE_MOUSE)
				{
					memset(&p,0,sizeof(p));
					if (ReadData(0)!=0xFF)
	 					{p.m.sdiv=ReadData(0); p.m.smul=ReadData(1);
	  					 p.m.minf=ReadData(2); p.m.maxf=ReadData(3);}
					else 
	 					{p.m.sdiv=1; p.m.smul=2; p.m.minf=0; p.m.maxf=250;}
#if DE_PRINTF
					printstr("Mouse Params : "); printx2(p.m.sdiv); printx2(p.m.smul); 
	                printx2(p.m.minf); printx2(p.m.maxf); printlf();
#endif
					qmouse_mode=1;
				}

				if (ThisUsbDev.DeviceType == DEV_TYPE_JOYSTICK)
				{
					qmouse_mode=0;		// Stops Mouse Quadrature in Int
					i=4;
					while ((s=ReadData(i))!=0xFF)
					{
						uint16_t vid,pid;
						vid=ReadData(i+1)<<8|ReadData(i+2);
						pid=ReadData(i+3)<<8|ReadData(i+4);

						if ( ((vid==0)&&(pid==0)) || 
						     ((vid==ThisUsbDev.DeviceVID)&&(pid==ThisUsbDev.DevicePID)) )
							{
								p.j.pUidx = ReadData(i+5); p.j.pUval = ReadData(i+6);
								p.j.pDidx = ReadData(i+7); p.j.pDval = ReadData(i+8);
								p.j.pLidx = ReadData(i+9); p.j.pLval = ReadData(i+10);
								p.j.pRidx = ReadData(i+11); p.j.pRval = ReadData(i+12);
								p.j.p1idx = ReadData(i+13); p.j.p1mask = ReadData(i+14);
							}
						i+=s;
					}
#if DE_PRINTF					
					printstr("Joystick Params : "); printlf();
					for(i=0;i!=12;i++) printx2((&p.j.pUidx)[i]); printlf();
#endif					
				}

			}
        }

		s = EnumAllHubPort( );                                             
		if ( s != ERR_SUCCESS ){                                             
#if DE_PRINTF
			printstr( "EnumAllHubPort err = ");printhex2(s);printlf();
#endif
		}

// pollHIDdevice ?
		if (timer&0x40)		// 15874/128 -> ~250hz (4ms)
		{
		  timer=0;

		  loc = SearchTypeDevice( DEV_TYPE_MOUSE );                        
		  if ( loc != 0xFFFF ){                                        

			//printstr( "Query Mouse @");printhex4(loc);printlf();

			i = (uint8_t)( loc >> 8 );
			len = (uint8_t)loc;
			SelectHubPort( len ); 
			endp = len ? DevOnHubPort[len-1].GpVar[0] : ThisUsbDev.GpVar[0];     
			if ( endp & USB_ENDP_ADDR_MASK ){                               
				s = USBHostTransact( USB_PID_IN << 4 | endp & 0x7F, endp & 0x80 ? bUH_R_TOG | bUH_T_TOG : 0, 0 );

				if ( s == ERR_SUCCESS ){
					endp ^= 0x80;                                         
					if ( len ) DevOnHubPort[len-1].GpVar[0] = endp;            
					else ThisUsbDev.GpVar[0] = endp;
					len = USB_RX_LEN;                                 
					if ( len ) {
#if DE_PRINTF
						printstr("RX: ");
						for ( i = 0; i != len; i ++ ){
							printx2(RxBuffer[i]);
						}
						printlf();
#endif
						i=RxBuffer[0];

						if (i&1) BUTT1=0; else BUTT1=1;
						if (i&2) BUTT0=0; else BUTT0=1;
						
						EA=0;
						int8_t sval;		// Temporary signed value (mouse mvt from -127 to + 127)

						sval=RxBuffer[1];

						if (sval!=0)
						{
						 i=(sval>=0)?sval:-sval; i/=p.m.sdiv;
						 if ( p.m.xcnt && ( 
							  ((sval>=0)&&(p.m.xdelta<0)) ||
						      ((sval<0)&&(p.m.xdelta>=0)) ) )
							{ if (p.m.xcnt>i) p.m.xcnt-=i; else p.m.xcnt=0; }	// Dir changed
						 else
						  {if (((int16_t)p.m.xcnt+i)<0x100) p.m.xcnt+=i; else p.m.xcnt=255;}
						    
						 uint16_t delta;

						 delta=p.m.minf+p.m.smul*p.m.xcnt;
						 if (delta>=p.m.maxf) delta=p.m.maxf;
						 if (sval>=0) p.m.xdelta=delta;
						         else p.m.xdelta=-delta;						 
						}

						sval=RxBuffer[2];

						if (sval!=0)
						{
						 i=(sval>=0)?sval:-sval; i/=p.m.sdiv;
						 if (sval>=0)
							{if (p.m.ydelta<0) p.m.ycnt=0;}
						 else
							{if (p.m.ydelta>=0) p.m.ycnt=0;}							
						 p.m.ycnt+=i;
						 { uint16_t delta;
						   delta=p.m.minf+p.m.smul*p.m.ycnt;
						   if (delta>=p.m.maxf) delta=p.m.maxf;
						   if (sval>=0) p.m.ydelta=delta;
						 		   else p.m.ydelta=-delta;
						 }
						}

						EA=1;
#if DE_PRINTF
						for(i=0;i!=16;i++) printx2((&p.m.xcnt)[i]); printlf();
#else
						mDelayuS(900);
#endif						 
					}
				}
				else if ( s != ( USB_PID_NAK | ERR_USB_TRANSFER ) ) {
#if DE_PRINTF
					printstr("Err ");printhex2(s);
#endif
				}
			}
			else {
#if DE_PRINTF
				printstr("Mouse no interrupt endpoint\n");
#endif
			}
			SetUsbSpeed( 1 );                                                 // The default is full speed
		  }

		  loc = SearchTypeDevice( DEV_TYPE_JOYSTICK );                        
		  if ( loc != 0xFFFF ){                                        

			i = (uint8_t)( loc >> 8 );
			len = (uint8_t)loc;
			SelectHubPort( len ); 
			endp = len ? DevOnHubPort[len-1].GpVar[0] : ThisUsbDev.GpVar[0];     
			if ( endp & USB_ENDP_ADDR_MASK ){                               
				s = USBHostTransact( USB_PID_IN << 4 | endp & 0x7F, endp & 0x80 ? bUH_R_TOG | bUH_T_TOG : 0, 0 );

				if ( s == ERR_SUCCESS ){
					endp ^= 0x80;                                         
					if ( len ) DevOnHubPort[len-1].GpVar[0] = endp;            
					else ThisUsbDev.GpVar[0] = endp;
					len = USB_RX_LEN;                                 
					if ( len ) {
								// UP/DOWN/LEFT/RIGHT -> P1_4/5/6/7
								// FIRE -> P3_0
#if DE_PRINTF
	#define CH(a) putch(a);
#else
	#define CH(a)
#endif

#define COND(idx,val,io,c) { switch (idx&0xF0) \
                             {  case 0x10: \
							      if (RxBuffer[idx&0xF]<val) {io=0;CH(c);} else {io=1;} \
								  break; \
							    case 0x20: \
								  if (RxBuffer[idx&0xF]>val) {io=0;CH(c);} else {io=1;} \
                                  break; \
								case 0x30: \
								  if (RxBuffer[idx&0xF]&val) {io=0;CH(c);} else {io=1;} \
								  break; \
							 } \
						 }

						 COND(p.j.pUidx,p.j.pUval,DirU,'U')
						 COND(p.j.pDidx,p.j.pDval,DirD,'D')
						 COND(p.j.pLidx,p.j.pLval,DirL,'L')
						 COND(p.j.pRidx,p.j.pRval,DirR,'R')

						 COND(p.j.p1idx,p.j.p1mask,Fire,'*')
#if DE_PRINTF
#else
						 mDelayuS(900);
#endif						 
					}
				}
				else if ( s != ( USB_PID_NAK | ERR_USB_TRANSFER ) ) {
#if DE_PRINTF
					printstr("Err ");printhex2(s);                
#endif
				}
			}
			else {
#if DE_PRINTF
				printstr("Joystick no interrupt endpoint\n");
#endif
			}
			SetUsbSpeed( 1 );                                                 // The default is full speed
		  }

		}	//End test if timer

    }	// End While
}
