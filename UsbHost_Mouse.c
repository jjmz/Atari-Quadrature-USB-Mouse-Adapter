
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

__code uint8_t  SetupGetDevDescr[] = { USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_DEVICE, 0x00, 0x00, sizeof( USB_DEV_DESCR ), 0x00 };
__code uint8_t  SetupGetCfgDescr[] = { USB_REQ_TYP_IN, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_CONFIG, 0x00, 0x00, 0x04, 0x00 };
__code uint8_t  SetupSetUsbAddr[] = { USB_REQ_TYP_OUT, USB_SET_ADDRESS, USB_DEVICE_ADDR, 0x00, 0x00, 0x00, 0x00, 0x00 };
__code uint8_t  SetupSetUsbConfig[] = { USB_REQ_TYP_OUT, USB_SET_CONFIGURATION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
__code uint8_t  SetupSetUsbInterface[] = { USB_REQ_RECIP_INTERF, USB_SET_INTERFACE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
__code uint8_t  SetupClrEndpStall[] = { USB_REQ_TYP_OUT | USB_REQ_RECIP_ENDP, USB_CLEAR_FEATURE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
__code uint8_t  SetupGetHubDescr[] = { HUB_GET_HUB_DESCRIPTOR, HUB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_HUB, 0x00, 0x00, sizeof( USB_HUB_DESCR ), 0x00 };
__code uint8_t  SetupSetHIDIdle[]= { 0x21,HID_SET_IDLE,0x00,0x00,0x00,0x00,0x00,0x00 };
__code uint8_t  SetupGetHIDDevReport[] = { 0x81, USB_GET_DESCRIPTOR, 0x00, USB_DESCR_TYP_REPORT, 0x00, 0x00, 0xFF, 0x00 };
// __code uint8_t  XPrinterReport[] = { 0xA1, 0, 0x00, 0, 0x00, 0x00, 0xF1, 0x03 };
__code uint8_t  GetProtocol[] = { 0xc0,0x33,0x00,0x00,0x00,0x00,0x02,0x00 };
// __code uint8_t  TouchAOAMode[] = { 0x40,0x35,0x00,0x00,0x00,0x00,0x00,0x00 };

__code uint8_t  Sendlen[]= {0,4,16,35,39,53,67};
__code uint8_t  StringID[] = {'W','C','H',0x00,                                                           //manufacturer name
                      'W','C','H','U','A','R','T','D','e','m','o',0x00,                                   //model name
                      0x57,0x43,0x48,0x20,0x41,0x63,0x63,0x65,0x73,0x73,0x6f,0x72,0x79,0x20,0x54,0x65,0x73,0x74,0x00,     //description
                      '1','.','0',0x00 ,                                                                  //version
                      0x68,0x74,0x74,0x70,0x3a,0x2f,0x2f,0x77,0x63,0x68,0x2e,0x63,0x6e,0,                 //URI
                      0x57,0x43,0x48,0x41,0x63,0x63,0x65,0x73,0x73,0x6f,0x72,0x79,0x31,0x00               //serial number
                     };  
__code uint8_t  SetStringID[]= {0x40,0x34,0x00,0x00,0x00,0x00,0x04,0x00,
                        0x40,0x34,0x00,0x00,0x01,0x00,12,0x00,
                        0x40,0x34,0x00,0x00,0x02,0x00,19,0x00,
                        0x40,0x34,0x00,0x00,0x03,0x00,4,0x00,
                        0x40,0x34,0x00,0x00,0x04,0x00,0x0E,0x00,
                        0x40,0x34,0x00,0x00,0x05,0x00,0x0E,0x00
                       };

__xdata uint8_t  UsbDevEndp0Size;                                                      
__xdata __at (0x0000) uint8_t  RxBuffer[ MAX_PACKET_SIZE ];                            // IN, must even address
__xdata __at (0x0040) uint8_t  TxBuffer[ MAX_PACKET_SIZE ];                            // OUT, must even address

uint8_t Set_Port = 0;

__xdata _RootHubDev ThisUsbDev;                                                   //ROOT
__xdata _DevOnHubPort DevOnHubPort[HUB_MAX_PORTS];                               

__bit FoundNewDev;

int16_t xdelta=0,ydelta=0;
uint16_t xval=0,yval=0;
uint8_t xcnt=0,ycnt=0;
uint8_t xph=0,yph=0;

SBIT(BUTT0,0xB0,1);		// P3_1
SBIT(BUTT1,0xB0,0);		// P3_0

void Timer0_ISR(void) __interrupt (INT_NO_TMR0) __using(1) {
	if (xcnt) {
		xval+=xdelta;
		if (((xval>>8)&0xff)!=xph) {
		  xph=xval>>8;
		  xcnt--;
		  ACC=xph;
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
	} else {xdelta=0;}

	if (ycnt) {
		yval+=ydelta;
		if (((yval>>8)&0xff)!=yph) {
		  yph=yval>>8;
		  ycnt--;
		  ACC=yph;
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
	} else {ydelta=0;}
}

void main( )
{
    uint8_t   i, s,k, len, endp;
    uint16_t  loc;
    int8_t val;

    CfgFsys( );	

    P3 |= 0x03;
    P3_MOD_OC |= 0x03; // P3.1,0 Open Collector Output
    P3_DIR_PU |= 0x03;

    P1 &= 0x0F;
    P1_MOD_OC &= 0x0F; // P1.4,5,6,7 Push-Pull
    P1_DIR_PU |= 0xF0;

    mDelaymS(50);

#ifdef DE_PRINTF
    mInitSTDIO( );                                                          
    printf( "Start @ChipID=%02X\n", (uint16_t)CHIP_ID );
#endif

    T2MOD &= 0xEF;	// BT0_CLK = Fsys/12 (1333333Hz)
    TMOD = TMOD & 0xF0 | 0x02; // Timer 0 mode 1
    TH0 = 0 - 84; TR0 = 1;
    ET0=1;
    EA=1;

    InitUSB_Host( );
    FoundNewDev = 0;
#ifdef DE_PRINTF
    printf( "Wait Device In\n" );
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
#ifdef DE_PRINTF
                printf( "EnumAllRootDev err = %02X\n", (uint16_t)s );					
#endif
            }
        }

		s = EnumAllHubPort( );                                             
		if ( s != ERR_SUCCESS ){                                             
#ifdef DE_PRINTF
			printf( "EnumAllHubPort err = %02X\n", (uint16_t)s );
#endif
		}

// pollHIDdevice ?

		loc = SearchTypeDevice( DEV_TYPE_MOUSE );                        
		if ( loc != 0xFFFF ){                                        
#if 0
			printf( "Query Mouse @%04X\r\n", loc );
#endif
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
#if 0
						printf("Mouse data: ");
						for ( i = 0; i < len; i ++ ){
							printf("x%02X ",(uint16_t)(RxBuffer[i]) );
						}
						printf("\r");
#else
						i=RxBuffer[0];

						if (i&1) BUTT1=0; else BUTT1=1;
						if (i&2) BUTT0=0; else BUTT0=1;
						
						//printf("Bx %02X %02X %02X\n",xdelta,xcnt,xph);
						//printf("By %02X %02X %02X\n",ydelta,ycnt,yph);
						val=RxBuffer[1]; xcnt+=(val<0?-val:val); xdelta+=2*(int16_t)val;
						val=RxBuffer[2]; ycnt+=(val<0?-val:val); ydelta+=2*(int16_t)val;
						//printf("Ax %02X %02X %02X\n",xdelta,xcnt,xph);
						//printf("Ay %02X %02X %02X\n\n",ydelta,ycnt,yph);
						mDelayuS(950);
#endif
					}
				}
				else if ( s != ( USB_PID_NAK | ERR_USB_TRANSFER ) ) {
#ifdef DE_PRINTF
					printf("Err %02x-",(uint16_t)s);                
#endif
				}
			}
			else {
#ifdef DE_PRINTF
				printf("Mouse no interrupt endpoint\n");
#endif
			}
			SetUsbSpeed( 1 );                                                 // 默认为全速
		}


    }
}
