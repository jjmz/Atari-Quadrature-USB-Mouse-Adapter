/********************************** (C) COPYRIGHT *******************************
* File Name          : USBHOST.C
* Author             : WCH
* Version            : V1.1
* Date               : 2018/02/28
* Description        : CH554 USB Host interface function
*******************************************************************************/

#include <ch554.h>
#include <debug.h>
#include "usbhost.h"
#include "small_print.h"
#include <string.h>

extern __xdata __at (0x0380) uint8_t  RxBuffer[ MAX_PACKET_SIZE ];
extern __xdata __at (0x03C0) uint8_t  TxBuffer[ MAX_PACKET_SIZE ];

#include <ch554_usb.h>

__xdata uint8_t  Com_Buffer[ COM_BUF_SIZE ];      //Define a user temporary buffer, which is used 
                                                  // to process descriptors during enumeration, and 
                                                  // can also be used as a normal temporary buffer 
                                                  // at the end of enumeration
/*******************************************************************************
* Function Name  : DisableRootHubPort( )
* Description    : Close the HUB port
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  DisableRootHubPort( )          
{
    ThisUsbDev.DeviceStatus = ROOT_DEV_DISCONNECT;
    ThisUsbDev.DeviceAddress = 0x00;
}
/*******************************************************************************
* Function Name  : AnalyzeRootHub(void)
* Description    : Analyze the ROOT-HUB status and handle the plugging and unplugging events of the ROOT-HUB port
                   If the device is unplugged, call the DisableRootHubPort() function in the function to close the port, insert the event, and set the status bit of the corresponding port
* Input          : None
* Output         : None
* Return         : Return ERR_SUCCESS for no condition, return ERR_USB_CONNECT for new connection detected, and return ERR_USB_DISCON for disconnection detected
*******************************************************************************/
uint8_t   AnalyzeRootHub( void )
{ 
	uint8_t	s;
	s = ERR_SUCCESS;
	if ( USB_MIS_ST & bUMS_DEV_ATTACH ) {                                        // Device exists
		if ( ThisUsbDev.DeviceStatus == ROOT_DEV_DISCONNECT                        // Device plugged in
			|| ( UHOST_CTRL & bUH_PORT_EN ) == 0x00 ) {                              //A device is detected to be plugged in, but it has not been allowed, indicating that it has just been plugged in
			DisableRootHubPort( );                                                   // Close the port
//		ThisUsbDev.DeviceSpeed = USB_HUB_ST & bUHS_DM_LEVEL ? 0 : 1;
			ThisUsbDev.DeviceStatus = ROOT_DEV_CONNECTED;                            //Set the connection flag
#if DE_PRINTF
			printstr( "USB dev in\n" );
#endif
			s = ERR_USB_CONNECT;
		}
	}
	else if ( ThisUsbDev.DeviceStatus >= ROOT_DEV_CONNECTED ) {                  //Device unplug detected
		DisableRootHubPort( );                                                     // Close the port
#if DE_PRINTF		
		printstr( "USB dev out\n" );
#endif
		if ( s == ERR_SUCCESS ) s = ERR_USB_DISCON;
	}
//	UIF_DETECT = 0;                                                            // Clear interrupt flag
	return( s );
}
/*******************************************************************************
* Function Name  : SetHostUsbAddr
* Description    : Set the USB device address currently operated by the USB host
* Input          : uint8_t addr
* Output         : None
* Return         : None
*******************************************************************************/
void    SetHostUsbAddr( uint8_t addr )
{
    USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | addr & 0x7F;
}

/*******************************************************************************
* Function Name  : SetUsbSpeed
* Description    : Set current USB speed
* Input          : uint8_t FullSpeed
* Output         : None
* Return         : None
*******************************************************************************/
void    SetUsbSpeed( uint8_t FullSpeed )  
{
    if ( FullSpeed )                                                           // full speed
    {
        USB_CTRL &= ~ bUC_LOW_SPEED;                                           // full speed
        UH_SETUP &= ~ bUH_PRE_PID_EN;                                          // Prohibit PRE PID
    }
    else
    {
        USB_CTRL |= bUC_LOW_SPEED;                                             // Low speed	
    }
}

/*******************************************************************************
* Function Name  : ResetRootHubPort( )
* Description    : After the device is detected, reset the bus to prepare for enumerating the device, and set it to default to full speed
* Input          : None   
* Output         : None
* Return         : None
*******************************************************************************/
void  ResetRootHubPort( )
{
    UsbDevEndp0Size = DEFAULT_ENDP0_SIZE;                                      //Maximum packet size of endpoint 0 of the USB device
    memset( &ThisUsbDev,0,sizeof(ThisUsbDev));                                 //Empty structure
	SetHostUsbAddr( 0x00 );
    UHOST_CTRL &= ~bUH_PORT_EN;                                                // Turn off the port
	SetUsbSpeed( 1 );                                                          // The default is full speed
	UHOST_CTRL = UHOST_CTRL & ~ bUH_LOW_SPEED | bUH_BUS_RESET;                 // The default is full speed, start to reset
    mDelaymS( 20 );                                                            // Reset time 10mS to 20mS
    UHOST_CTRL = UHOST_CTRL & ~ bUH_BUS_RESET;                                 // End reset
    mDelayuS( 250 );
    UIF_DETECT = 0;                                                            // Clear interrupt flag
}
/*******************************************************************************
* Function Name  : EnableRootHubPort( )
* Description    : Enable the ROOT-HUB port, and the corresponding bUH_PORT_EN is set to 1 to enable the port. The device disconnection may cause the return failure
* Input          : None
* Output         : None
* Return         : Return ERR_SUCCESS to detect a new connection, return ERR_USB_DISCON to indicate no connection
*******************************************************************************/
uint8_t   EnableRootHubPort( )
{
	if ( ThisUsbDev.DeviceStatus < ROOT_DEV_CONNECTED ) ThisUsbDev.DeviceStatus = ROOT_DEV_CONNECTED;
	if ( USB_MIS_ST & bUMS_DEV_ATTACH ) {                                        // Have equipment
		if ( ( UHOST_CTRL & bUH_PORT_EN ) == 0x00 ) {                              // Not yet enabled
			ThisUsbDev.DeviceSpeed = USB_MIS_ST & bUMS_DM_LEVEL ? 0 : 1;
			if ( ThisUsbDev.DeviceSpeed == 0 ) UHOST_CTRL |= bUH_LOW_SPEED;          // Low speed
		}
		USB_CTRL |= bUC_DMA_EN;                                                    // Start the USB host and DMA, and automatically pause before the interrupt flag is cleared
		UH_SETUP = bUH_SOF_EN;		
		UHOST_CTRL |= bUH_PORT_EN;                                                 //Enable HUB port
		return( ERR_SUCCESS );
	}
	return( ERR_USB_DISCON );
}

/*******************************************************************************
* Function Name  : SelectHubPort( uint8_t HubPortIndex )
* Description    : Select the HUB port to be operated
* Input          : uint8_t HubPortIndex Select the designated port of the external HUB to operate the designated ROOT-HUB port
* Output         : None
* Return         : None
*******************************************************************************/
void    SelectHubPort( )  
{ 
    
        //HubLowSpeed = 0;        			
        SetHostUsbAddr( ThisUsbDev.DeviceAddress );                            // 设置USB主机当前操作的USB设备地址
        SetUsbSpeed( ThisUsbDev.DeviceSpeed );                                 // 设置USB设备的速度
    
}

/*******************************************************************************
* Function Name  : WaitUSB_Interrupt
* Description    : 等待USB中断
* Input          : None
* Output         : None
* Return         : 返回ERR_SUCCESS 数据接收或者发送成功
                   ERR_USB_UNKNOWN 数据接收或者发送失败
*******************************************************************************/
uint8_t WaitUSB_Interrupt( void )
{
    uint16_t  i;
    for ( i = WAIT_USB_TOUT_200US; i != 0 && UIF_TRANSFER == 0; i -- ){;}
    return( UIF_TRANSFER ? ERR_SUCCESS : ERR_USB_UNKNOWN );
}
/*******************************************************************************
* Function Name  : USBHostTransact
* Description    : CH554传输事务,输入目的端点地址/PID令牌,同步标志,以20uS为单位的NAK重试总时间(0则不重试,0xFFFF无限重试),返回0成功,超时/出错重试
                   本子程序着重于易理解,而在实际应用中,为了提供运行速度,应该对本子程序代码进行优化
* Input          : uint8_t endp_pid 令牌和地址  endp_pid: 高4位是token_pid令牌, 低4位是端点地址
                   uint8_t tog      同步标志
                   uint16_t timeout 超时时间
* Output         : None
* Return         : ERR_USB_UNKNOWN 超时，可能硬件异常
                   ERR_USB_DISCON  设备断开
                   ERR_USB_CONNECT 设备连接
                   ERR_SUCCESS     传输完成
*******************************************************************************/
uint8_t   USBHostTransact( uint8_t endp_pid, uint8_t tog, uint16_t timeout )
{
//	uint8_t	TransRetry;
#define	TransRetry	UEP0_T_LEN	                                               // Save memory
	uint8_t	s, r;
	uint16_t	i;
	UH_RX_CTRL = UH_TX_CTRL = tog;
	TransRetry = 0;

	do {
		UH_EP_PID = endp_pid;                                                      // Specify the token PID and destination endpoint number
		UIF_TRANSFER = 0;                                                          // Allow transfer
//  s = WaitUSB_Interrupt( );
		for ( i = WAIT_USB_TOUT_200US; i != 0 && UIF_TRANSFER == 0; i -- );
		UH_EP_PID = 0x00;                                                          // Stop USB transfer
//	if ( s != ERR_SUCCESS ) return( s );  // Interrupt timeout, may be a hardware abnormality
		if ( UIF_TRANSFER == 0 ) return( ERR_USB_UNKNOWN );
		if ( UIF_DETECT ) {                                                        // USB设备插拔事件
//			mDelayuS( 200 );                                                       // 等待传输完成
			UIF_DETECT = 0;                                                          // 清中断标志
			s = AnalyzeRootHub( );                                                   // 分析ROOT-HUB状态
			if ( s == ERR_USB_CONNECT ) FoundNewDev = 1;
			if ( ThisUsbDev.DeviceStatus == ROOT_DEV_DISCONNECT ) return( ERR_USB_DISCON );// USB设备断开事件
			if ( ThisUsbDev.DeviceStatus == ROOT_DEV_CONNECTED ) return( ERR_USB_CONNECT );// USB设备连接事件
			mDelayuS( 200 );  // 等待传输完成
		}
		if ( UIF_TRANSFER ) {  // 传输完成
			if ( U_TOG_OK ) return( ERR_SUCCESS );
			r = USB_INT_ST & MASK_UIS_H_RES;  // USB设备应答状态
			if ( r == USB_PID_STALL ) return( r | ERR_USB_TRANSFER );
			if ( r == USB_PID_NAK ) {
				if ( timeout == 0 ) return( r | ERR_USB_TRANSFER );
				if ( timeout < 0xFFFF ) timeout --;
				-- TransRetry;
			}
			else switch ( endp_pid >> 4 ) {
				case USB_PID_SETUP:
				case USB_PID_OUT:
//					if ( U_TOG_OK ) return( ERR_SUCCESS );
//					if ( r == USB_PID_ACK ) return( ERR_SUCCESS );
//					if ( r == USB_PID_STALL || r == USB_PID_NAK ) return( r | ERR_USB_TRANSFER );
					if ( r ) return( r | ERR_USB_TRANSFER );  // 不是超时/出错,意外应答
					break;  // 超时重试
				case USB_PID_IN:
//					if ( U_TOG_OK ) return( ERR_SUCCESS );
//					if ( tog ? r == USB_PID_DATA1 : r == USB_PID_DATA0 ) return( ERR_SUCCESS );
//					if ( r == USB_PID_STALL || r == USB_PID_NAK ) return( r | ERR_USB_TRANSFER );
					if ( r == USB_PID_DATA0 || r == USB_PID_DATA1 ) {  // 不同步则需丢弃后重试
					}  // 不同步重试
					else if ( r ) return( r | ERR_USB_TRANSFER );  // 不是超时/出错,意外应答
					break;  // 超时重试
				default:
					return( ERR_USB_UNKNOWN );  // 不可能的情况
					break;
			}
		}
		else {  // 其它中断,不应该发生的情况
			USB_INT_FG = 0xFF;  /* 清中断标志 */
		}
		mDelayuS( 15 );	
	} while ( ++ TransRetry < 3 );
	return( ERR_USB_TRANSFER );  // 应答超时
}
/*******************************************************************************
* Function Name  : HostCtrlTransfer
* Description    : 执行控制传输,8字节请求码在pSetupReq中,DataBuf为可选的收发缓冲区
* Input          : P__xdata uint8_t DataBuf 如果需要接收和发送数据,那么DataBuf需指向有效缓冲区用于存放后续数据
                   Puint8_t RetLen  实际成功收发的总长度保存在RetLen指向的字节变量中
* Output         : None
* Return         : ERR_USB_BUF_OVER IN状态阶段出错
                   ERR_SUCCESS     数据交换成功
                   其他错误状态
*******************************************************************************/
uint8_t HostCtrlTransfer( __xdata uint8_t *DataBuf, uint8_t *RetLen )  
{
    uint16_t  RemLen  = 0;
    uint8_t   s, RxLen, RxCnt, TxCnt;
    __xdata uint8_t  *pBuf;
    uint8_t  *pLen;
    pBuf = DataBuf;
    pLen = RetLen;
    mDelayuS( 200 );
    if ( pLen )
    {
        *pLen = 0;                                                              // 实际成功收发的总长度
    }
    UH_TX_LEN = sizeof( USB_SETUP_REQ );
    s = USBHostTransact( (uint8_t)(USB_PID_SETUP << 4 | 0x00), 0x00, 10000 );          // SETUP阶段,200mS超时
    if ( s != ERR_SUCCESS )
    {
        return( s );
    }
    UH_RX_CTRL = UH_TX_CTRL = bUH_R_TOG | bUH_R_AUTO_TOG | bUH_T_TOG | bUH_T_AUTO_TOG;// 默认DATA1
    UH_TX_LEN = 0x01;                                                           // 默认无数据故状态阶段为IN
    RemLen = (pSetupReq -> wLengthH << 8)|( pSetupReq -> wLengthL);
    if ( RemLen && pBuf )                                                       // 需要收发数据
    {
        if ( pSetupReq -> bRequestType & USB_REQ_TYP_IN )                       // 收
        {
            while ( RemLen )
            {
                mDelayuS( 200 );
                s = USBHostTransact( (uint8_t)(USB_PID_IN << 4 | 0x00), UH_RX_CTRL, 200000/20 );// IN数据
                if ( s != ERR_SUCCESS )
                {
                    return( s );
                }
                RxLen = USB_RX_LEN < RemLen ? USB_RX_LEN : RemLen;
                RemLen -= RxLen;
                if ( pLen )
                {
                    *pLen += RxLen;                                              // 实际成功收发的总长度
                }
//              memcpy( pBuf, RxBuffer, RxLen );
//              pBuf += RxLen;
                for ( RxCnt = 0; RxCnt != RxLen; RxCnt ++ )
                {
                    *pBuf = RxBuffer[ RxCnt ];
                    pBuf ++;
                }
                if ( USB_RX_LEN == 0 || ( USB_RX_LEN & ( UsbDevEndp0Size - 1 ) ) )
                {
                    break;                                                       // 短包
                }
            }
            UH_TX_LEN = 0x00;                                                    // 状态阶段为OUT
        }
        else                                                                     // 发
        {
            while ( RemLen )
            {
                mDelayuS( 200 );
                UH_TX_LEN = RemLen >= UsbDevEndp0Size ? UsbDevEndp0Size : RemLen;
//              memcpy( TxBuffer, pBuf, UH_TX_LEN );
//              pBuf += UH_TX_LEN;
                if(pBuf[1] == 0x09)                                              //HID类命令处理
                {
                    Set_Port = Set_Port^1;
                    *pBuf = Set_Port;
#if DE_PRINTF									
                    printstr("SET_PORT  ");printx2(*pBuf);printx2(Set_Port);printlf();
#endif									
                }
                for ( TxCnt = 0; TxCnt != UH_TX_LEN; TxCnt ++ )
                {
                    TxBuffer[ TxCnt ] = *pBuf;
                    pBuf ++;
                }
                s = USBHostTransact( USB_PID_OUT << 4 | 0x00, UH_TX_CTRL, 200000/20 );// OUT数据
                if ( s != ERR_SUCCESS )
                {
                    return( s );
                }
                RemLen -= UH_TX_LEN;
                if ( pLen )
                {
                    *pLen += UH_TX_LEN;                                           // 实际成功收发的总长度
                }
            }
//          UH_TX_LEN = 0x01;                                                     // 状态阶段为IN
        }
    }
    mDelayuS( 200 );
    s = USBHostTransact( ( UH_TX_LEN ? USB_PID_IN << 4 | 0x00: USB_PID_OUT << 4 | 0x00 ), bUH_R_TOG | bUH_T_TOG, 200000/20 );  // STATUS阶段
    if ( s != ERR_SUCCESS )
    {
        return( s );
    }
    if ( UH_TX_LEN == 0 )
    {
        return( ERR_SUCCESS );                                                    // 状态OUT
    }
    if ( USB_RX_LEN == 0 )
    {
        return( ERR_SUCCESS );                                                    // 状态IN,检查IN状态返回数据长度
    }
    return( ERR_USB_BUF_OVER );                                                   // IN状态阶段错误
}
/*******************************************************************************
* Function Name  : CopySetupReqPkg
* Description    : 复制控制传输的请求包
* Input          : P__code uint8_t pReqPkt 控制请求包地址
* Output         : None
* Return         : None
*******************************************************************************/
void CopySetupReqPkg( __code uint8_t *pReqPkt )                                        // 复制控制传输的请求包
{
    uint8_t   i;
		for ( i = 0; i != sizeof( USB_SETUP_REQ ); i ++ )
		{
			((__xdata uint8_t *)pSetupReq)[ i ] = *pReqPkt;
			pReqPkt++;
		}			
}
/*******************************************************************************
* Function Name  : CtrlGetDeviceDescr
* Description    : 获取设备描述符,返回在TxBuffer中
* Input          : None
* Output         : None
* Return         : ERR_USB_BUF_OVER 描述符长度错误
                   ERR_SUCCESS      成功
                   其他
*******************************************************************************/
uint8_t   CtrlGetDeviceDescr( void )  
{
    uint8_t   s;
    uint8_t   len;
    UsbDevEndp0Size = DEFAULT_ENDP0_SIZE;
    CopySetupReqPkg( SetupGetDevDescr );
    s = HostCtrlTransfer( Com_Buffer, (uint8_t *)&len );                                      // 执行控制传输
    if ( s != ERR_SUCCESS )
    {
        return( s );
    }
    UsbDevEndp0Size = ( (PXUSB_DEV_DESCR)Com_Buffer ) -> bMaxPacketSize0;          // 端点0最大包长度,这是简化处理,正常应该先获取前8字节后立即更新UsbDevEndp0Size再继续
    if ( len < ( (PUSB_SETUP_REQ)SetupGetDevDescr ) -> wLengthL )
    {
        return( ERR_USB_BUF_OVER );                                              // 描述符长度错误
    }
    return( ERR_SUCCESS );
}
/*******************************************************************************
* Function Name  : CtrlGetConfigDescr
* Description    : 获取配置描述符,返回在TxBuffer中
* Input          : None
* Output         : None
* Return         : ERR_USB_BUF_OVER 描述符长度错误
                   ERR_SUCCESS      成功
                   其他
*******************************************************************************/
uint8_t CtrlGetConfigDescr( void )
{
    uint8_t   s,len;
    CopySetupReqPkg( SetupGetCfgDescr );
    s = HostCtrlTransfer( Com_Buffer, (uint8_t *)&len );                                      // 执行控制传输
    if ( s != ERR_SUCCESS )
    {
        return( s );
    }

    len = ( (PXUSB_CFG_DESCR)Com_Buffer ) -> wTotalLengthL;
    CopySetupReqPkg( SetupGetCfgDescr );
    pSetupReq -> wLengthL = len;                                                 // 完整配置描述符的总长度
    s = HostCtrlTransfer( Com_Buffer, (uint8_t *)&len );                                // 执行控制传输
    if ( s != ERR_SUCCESS )
    {
        return( s );
    }
    return( ERR_SUCCESS );
}
/*******************************************************************************
* Function Name  : CtrlSetUsbAddress
* Description    : 设置USB设备地址
* Input          : uint8_t addr 设备地址
* Output         : None
* Return         : ERR_SUCCESS      成功
                   其他
*******************************************************************************/
uint8_t CtrlSetUsbAddress( uint8_t addr ) 
{
    uint8_t   s;
    CopySetupReqPkg( SetupSetUsbAddr );
    pSetupReq -> wValueL = addr;                                                // USB设备地址
    s = HostCtrlTransfer( NULL, NULL );                                         // 执行控制传输
    if ( s != ERR_SUCCESS )
    {
        return( s );
    }
    SetHostUsbAddr( addr );                                                     // 设置USB主机当前操作的USB设备地址
    mDelaymS( 10 );                                                             // 等待USB设备完成操作
    return( ERR_SUCCESS );
}
/*******************************************************************************
* Function Name  : CtrlSetUsbConfig
* Description    : 设置USB设备配置
* Input          : uint8_t cfg       配置值
* Output         : None
* Return         : ERR_SUCCESS      成功
                   其他
*******************************************************************************/
uint8_t   CtrlSetUsbConfig( uint8_t cfg )                   
{
    CopySetupReqPkg( SetupSetUsbConfig );
    pSetupReq -> wValueL = cfg;                                                // USB设备配置
    return( HostCtrlTransfer( NULL, NULL ) );                                  // 执行控制传输
}
/*******************************************************************************
* Function Name  : CtrlClearEndpStall
* Description    : 清除端点STALL
* Input          : uint8_t endp       端点地址
* Output         : None
* Return         : ERR_SUCCESS      成功
                   其他
*******************************************************************************/
uint8_t   CtrlClearEndpStall( uint8_t endp )  
{
    CopySetupReqPkg( SetupClrEndpStall );                                      // 清除端点的错误
    pSetupReq -> wIndexL = endp;                                               // 端点地址
    return( HostCtrlTransfer( NULL, NULL ) );                                  // 执行控制传输
}

/*******************************************************************************
* Function Name  : CtrlSetUsbInterface
* Description    : 设置USB设备接口
* Input          : uint8_t cfg       配置值
* Output         : None
* Return         : ERR_SUCCESS      成功
                   其他
*******************************************************************************/
uint8_t   CtrlSetUsbInterface( uint8_t cfg )                   
{
    CopySetupReqPkg( SetupSetUsbInterface );
    pSetupReq -> wValueL = cfg;                                                 // USB设备配置
    return( HostCtrlTransfer( NULL, NULL ) );                             // 执行控制传输
}

/*******************************************************************************
* Function Name  : CtrlGetHIDDeviceReport
* Description    : 获取HID设备报表描述符,返回在TxBuffer中
* Input          : None
* Output         : None
* Return         : ERR_SUCCESS 成功
                   其他        错误
*******************************************************************************/
uint8_t   CtrlGetHIDDeviceReport( uint8_t infc )  
{
    uint8_t   s;
    uint8_t   len;

	CopySetupReqPkg( SetupSetHIDIdle );
    TxBuffer[4] = infc;
    s = HostCtrlTransfer( Com_Buffer, (uint8_t *)&len );                                    // 执行控制传输
    if ( s != ERR_SUCCESS )
    {
        return( s );
    }	

	CopySetupReqPkg( SetupGetHIDDevReport );
	TxBuffer[4] = infc;
    s = HostCtrlTransfer( Com_Buffer, (uint8_t *)&len );                                    // 执行控制传输
    if ( s != ERR_SUCCESS )
    {
        return( s );
    }

    return( ERR_SUCCESS );
}

/*******************************************************************************
* Function Name  : AnalyzeHidIntEndp
* Description    : 从描述符中分析出HID中断端点的地址,如果HubPortIndex是0保存到ROOTHUB，如果是非零值则保存到HUB下结构体
* Input          : P__xdata uint8_t buf ： 待分析数据缓冲区地址 HubPortIndex：0表示根HUB，非0表示外部HUB下的端口号
* Output         : None
* Return         : 端点数
*******************************************************************************/
uint8_t   AnalyzeHidIntEndp( __xdata uint8_t *buf) 
{
    uint8_t   i, s, l;
    s = 0;

	memset( ThisUsbDev.GpVar,0,sizeof(ThisUsbDev.GpVar) );                     //清空数组

    for ( i = 0; i < ( (PXUSB_CFG_DESCR)buf ) -> wTotalLengthL; i += l )       // 搜索中断端点描述符,跳过配置描述符和接口描述符
    {
        if ( ( (PXUSB_ENDP_DESCR)(buf+i) ) -> bDescriptorType == USB_DESCR_TYP_ENDP  // 是端点描述符
                && ( ( (PXUSB_ENDP_DESCR)(buf+i) ) -> bmAttributes & USB_ENDP_TYPE_MASK ) == USB_ENDP_TYPE_INTER// 是中断端点
                && ( ( (PXUSB_ENDP_DESCR)(buf+i) ) -> bEndpointAddress & USB_ENDP_DIR_MASK ) )// 是IN端点
        {           // 保存中断端点的地址,位7用于同步标志位,清0
		ThisUsbDev.GpVar[s] = ( (PXUSB_ENDP_DESCR)(buf+i) ) -> bEndpointAddress & USB_ENDP_ADDR_MASK;// 中断端点的地址，可以根据需要保存wMaxPacketSize和bInterval                                                          
#if DE_PRINTF			
			printhex2(ThisUsbDev.GpVar[s]);
#endif
			s++;
			if(s >= 4) break;	//Only analyze 4 endpoints

		}
        l = ( (PXUSB_ENDP_DESCR)(buf+i) ) -> bLength;                          // 当前描述符长度,跳过
        if ( l > 16 )
        {
            break;
        }
    }
#if DE_PRINTF
	printlf();
#endif	
    return( s );
}

/*******************************************************************************
* Function Name  : AnalyzeBulkEndp
* Description    : 分析出批量端点,GpVar[0]、GpVar[1]存放上传端点。GpVar[2]、GpVar[3]存放下传端点
* Input          : buf：待分析数据缓冲区地址   HubPortIndex：0表示根HUB，非0表示外部HUB下的端口号
* Output         : None
* Return         : 0
*******************************************************************************/
uint8_t   AnalyzeBulkEndp( __xdata uint8_t *buf) 
{
    uint8_t   i, s1,s2, l;
    s1 = 0;s2 = 2;

	memset( ThisUsbDev.GpVar,0,sizeof(ThisUsbDev.GpVar) );                     //清空数组

    for ( i = 0; i < ( (PXUSB_CFG_DESCR)buf ) -> wTotalLengthL; i += l )       // 搜索中断端点描述符,跳过配置描述符和接口描述符
    {
        if ( (( (PXUSB_ENDP_DESCR)(buf+i) ) -> bDescriptorType == USB_DESCR_TYP_ENDP)     // 是端点描述符
                && ((( (PXUSB_ENDP_DESCR)(buf+i) ) -> bmAttributes & USB_ENDP_TYPE_MASK ) == USB_ENDP_TYPE_BULK))  // 是中断端点

        {
			if(( (PXUSB_ENDP_DESCR)(buf+i) ) -> bEndpointAddress & USB_ENDP_DIR_MASK )
				ThisUsbDev.GpVar[s1++] = ( (PXUSB_ENDP_DESCR)(buf+i) ) -> bEndpointAddress & USB_ENDP_ADDR_MASK;
			else
				ThisUsbDev.GpVar[s2++] = ( (PXUSB_ENDP_DESCR)(buf+i) ) -> bEndpointAddress & USB_ENDP_ADDR_MASK;
			
			if(s1 == 2) s1 = 1;
			if(s2 == 4) s2 = 3;			
		}
        l = ( (PXUSB_ENDP_DESCR)(buf+i) ) -> bLength;                          // 当前描述符长度,跳过
        if ( l > 16 )
        {
            break;
        }
    }
    return( 0 );
}

/*******************************************************************************
* Function Name  : InitRootDevice
* Description    : 初始化指定ROOT-HUB端口的USB设备
* Input          : uint8_t RootHubIndex 指定端口，内置HUB端口号0/1
* Output         : None
* Return         :
*******************************************************************************/
uint8_t InitRootDevice( void ) 
{
    uint8_t   t,i, s, cfg, dv_cls, if_cls,ifc, if_cls2;
	uint8_t touchaoatm = 0;
    t = 0;
#if DE_PRINTF	
    printstr( "Reset USB Port\n");
#endif
USBDevEnum:
    for(i=0;i<t;i++)
    {
        mDelaymS( 100 );	
        if(t>10) return( s );			
    }
    ResetRootHubPort( );                                                    // 检测到设备后,复位相应端口的USB总线
    for ( i = 0, s = 0; i < 100; i ++ )                                     // 等待USB设备复位后重新连接,100mS超时
    {
        mDelaymS( 1 );
        if ( EnableRootHubPort( ) == ERR_SUCCESS )                          // 使能ROOT-HUB端口
        {
            i = 0;
            s ++;                                                           // 计时等待USB设备连接后稳定
            if ( s > (20+t) )
            {
                break;                                                      // 已经稳定连接15mS
            }
        }
    }	
    if ( i )                                                                 // The device is not connected after reset
    {
        DisableRootHubPort( );
#if DE_PRINTF
        printstr( "Disable usb port because of disconnect\n" );
#endif	
//         return( ERR_USB_DISCON );
    }
    SelectHubPort( );
#if DE_PRINTF		
    printstr( "GetDevDescr: " );
#endif
    s = CtrlGetDeviceDescr( );                                               // Get device descriptor
    if ( s == ERR_SUCCESS )
    {
#if DE_PRINTF	
        for ( i = 0; i < ( (PUSB_SETUP_REQ)SetupGetDevDescr ) -> wLengthL; i ++ )
        {
            printx2(Com_Buffer[i]);				
        }
        printlf();                                                       // Show descriptor
#endif	
		ThisUsbDev.DeviceVID = (((uint16_t)((PXUSB_DEV_DESCR)Com_Buffer)->idVendorH)<<8 ) + ((PXUSB_DEV_DESCR)Com_Buffer)->idVendorL; //保存VID PID信息
		ThisUsbDev.DevicePID = (((uint16_t)((PXUSB_DEV_DESCR)Com_Buffer)->idProductH)<<8 ) + ((PXUSB_DEV_DESCR)Com_Buffer)->idProductL;
        dv_cls = ( (PXUSB_DEV_DESCR)Com_Buffer ) -> bDeviceClass;               // 设备类代码			
        s = CtrlSetUsbAddress( ( (PUSB_SETUP_REQ)SetupSetUsbAddr ) -> wValueL );// 设置USB设备地址,加上RootHubIndex可以保证2个HUB端口分配不同的地址
        if ( s == ERR_SUCCESS )
        {
            ThisUsbDev.DeviceAddress = ( (PUSB_SETUP_REQ)SetupSetUsbAddr ) -> wValueL;  // 保存USB地址
#if DE_PRINTF						
            printstr( "GetCfgDescr: " );
#endif					
            s = CtrlGetConfigDescr( );                                        // 获取配置描述符
            if ( s == ERR_SUCCESS )
            {
                cfg = ( (PXUSB_CFG_DESCR)Com_Buffer ) -> bConfigurationValue;
                ifc = ( (PXUSB_CFG_DESCR)Com_Buffer ) -> bNumInterfaces;					
#if DE_PRINTF							
                for ( i = 0; i < ( (PXUSB_CFG_DESCR)Com_Buffer ) -> wTotalLengthL; i ++ )
                {
                    printx2(Com_Buffer[i]);
                }
                printlf();
#endif								
                                                                              //分析配置描述符,获取端点数据/各端点地址/各端点大小等,更新变量endp_addr和endp_size等
                if_cls = ( (PXUSB_CFG_DESCR_LONG)Com_Buffer ) -> itf_descr.bInterfaceClass;  // 接口类代码								
                if_cls2 = Com_Buffer[41];

                if ( (dv_cls == 0x00) && (if_cls == USB_DEV_CLASS_HID) && (( (PXUSB_CFG_DESCR_LONG)Com_Buffer ) -> itf_descr.bInterfaceSubClass <= 0x01) )// 是HID类设备,键盘/鼠标等
                { 									
                    s = AnalyzeHidIntEndp( Com_Buffer);                    // 从描述符中分析出HID中断端点的地址								
#if DE_PRINTF														 
                    printstr( "AnalyzeHidIntEndp ");printhex2(s);printlf();
#endif		                    
					if_cls = ( (PXUSB_CFG_DESCR_LONG)Com_Buffer ) -> itf_descr.bInterfaceProtocol;
#if DE_PRINTF														 
                    printstr( "CtrlSetUsbConfig ");printhex2(cfg);
                               printstr(" class ");printhex2(if_cls);printlf();
#endif		
                    s = CtrlSetUsbConfig( cfg );                          // 设置USB设备配置								
                    if ( s == ERR_SUCCESS )
                    {
#if DE_PRINTF												
                        printstr( "GetHIDReport: " );
#endif			
                        for(dv_cls=0;dv_cls<ifc;dv_cls++)
                        {											
							s = CtrlGetHIDDeviceReport(dv_cls);                    //获取报表描述符
							if(s == ERR_SUCCESS)
							{
#if DE_PRINTF														
								for ( i = 0; i < 64; i++ )
								{
									printx2(Com_Buffer[i]);
								}
								printlf();
#endif														
							}
						}
                        //Set_Idle( );
                                                                         //需保存端点信息以便主程序进行USB传输
                        ThisUsbDev.DeviceStatus = ROOT_DEV_SUCCESS;
                        if ( if_cls == 1 )
                        {
                            ThisUsbDev.DeviceType = DEV_TYPE_KEYBOARD;
                                                                         //进一步初始化,例如设备键盘指示灯LED等
                            if(ifc > 1)
                            {
#if DE_PRINTF														
								printstr( "USB_DEV_CLASS_HID Ready\n" );
#endif																
								ThisUsbDev.DeviceType = USB_DEV_CLASS_HID;//复合HID设备		
                                if ( if_cls2 == 2 )
                                    { ThisUsbDev.DeviceType = DEV_TYPE_MOUSE;
                                      ThisUsbDev.GpVar[0]=ThisUsbDev.GpVar[1];
                                      SetBootProto(1);
                                      SetBootProto(0);							// Dell BIOS calls this with ifc set to 0...
#if DE_PRINTF						  
								      printstr( "MOUSE Interface : 2\n" );
#endif                                      
                                    }													
                                      
                            }																												
#if DE_PRINTF														
                            printstr( "USB-Keyboard Ready\n" );
#endif													
                            SetUsbSpeed( 1 );                            // 默认为全速

                            return( ERR_SUCCESS );
                        }
                        else if ( if_cls == 2 )
                        {
                            ThisUsbDev.DeviceType = DEV_TYPE_MOUSE;
                                                                         //为了以后查询鼠标状态,应该分析描述符,取得中断端口的地址,长度等信息
                            if(ifc > 1)
                            {
#if DE_PRINTF														
								printstr( "USB_DEV_CLASS_HID Ready\n" );
#endif																
								ThisUsbDev.DeviceType = USB_DEV_CLASS_HID;//复合HID设备															
                            }
                            SetBootProto(0);															
#if DE_PRINTF													
                            printstr( "USB-Mouse Ready\n" );
#endif													
                            SetUsbSpeed( 1 );                            // The default is full speed

                            return( ERR_SUCCESS );
                        }
                        else if ( if_cls == 0 )
                        {
                            ThisUsbDev.DeviceType = DEV_TYPE_JOYSTICK;
                                                                         //In order to query the mouse status later, the descriptor should be analyzed to obtain the address, length and other information of the interrupt port
                            if(ifc > 1)
                            {
#if DE_PRINTF														
								printstr( "USB_DEV_CLASS_HID Ready\n" );
#endif																
								ThisUsbDev.DeviceType = USB_DEV_CLASS_HID;//Composite HID equipment															
                            }															
#if DE_PRINTF													
                            printstr( "USB-Joy Ready\n" );
#endif													
                            SetUsbSpeed( 1 );                            // The default is full speed

                            return( ERR_SUCCESS );
                        }
                        s = ERR_USB_UNSUPPORT;
                    }
                }
                else                                                                 //其他设备
                {			
#if DE_PRINTF														 
                    printstr( "dv_cls ");printhex2(dv_cls); printlf();
                    printstr( "if_cls ");printhex2(if_cls ); printlf();
                    printstr( "if_subcls ");printhex2( ( (PXUSB_CFG_DESCR_LONG)Com_Buffer ) -> itf_descr.bInterfaceSubClass );	printlf();
#endif				
					AnalyzeBulkEndp(Com_Buffer);                                  //分析出批量端点
#if DE_PRINTF
					for(i=0;i!=4;i++)
					{
						printx2(ThisUsbDev.GpVar[i]);
					}
					printlf();
#endif
                    s = CtrlSetUsbConfig( cfg );                                     // 设置USB设备配置
                    if ( s == ERR_SUCCESS ) 
                    {
#if DE_PRINTF						
						printx2(ThisUsbDev.DeviceVID); printx2(ThisUsbDev.DevicePID); printlf();
#endif						
                    }
                }
            }
        }
    }
#if DE_PRINTF			
    printstr( "InitRootDev Err = ");printhex2(s);printlf();
#endif		
    ThisUsbDev.DeviceStatus = ROOT_DEV_FAILED;
    SetUsbSpeed( 1 );                                                                 // 默认为全速
    t++;
    goto USBDevEnum;		
}
/*******************************************************************************
* Function Name  : EnumAllRootDevice
* Description    : Enumerate USB devices of all ROOT-HUB ports
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t   EnumAllRootDevice( void )   
{
    __idata uint8_t   s;
#if DE_PRINTF	
    printstr( "EnumUSBDev\n" );
#endif
	if ( ThisUsbDev.DeviceStatus == ROOT_DEV_CONNECTED )            // The device has just been plugged in and has not been initialized
	{
		s = InitRootDevice( );                                      // Initialize/enumerate the USB devices of the specified HUB port
		if ( s != ERR_SUCCESS )
		{
			return( s );
		}
	}
    return( ERR_SUCCESS );
}

/*******************************************************************************
*Function Name: SearchTypeDevice
*Description: Search for the port number where the specified type of device is located on each port of ROOT-HUB and external HUB. If the output port number is 0xFFFF, it will not be found
*Input: uint8_t type search device type
*Output: None
*Return: The high 8 bits of the output are the ROOT-HUB port number, the low 8 bits are the port number of the external HUB, and the low 8 bits are 0, the device is directly on the ROOT-HUB port
                   Of course, you can also search according to the PID of the USB manufacturer's VID product (record the VID and PID of each device in advance), and specify the search serial number
*******************************************************************************/
uint16_t  SearchTypeDevice( uint8_t type )   
{
	uint8_t  RootHubIndex;                                                          //CH554只有一个USB口,RootHubIndex = 0,只需看返回值的低八位即可

	RootHubIndex = 0;

	if ( (ThisUsbDev.DeviceType == type) && (ThisUsbDev.DeviceStatus >= ROOT_DEV_SUCCESS) )
	{
		return( (uint16_t)RootHubIndex << 8 );                                      // 类型匹配且枚举成功,在ROOT-HUB端口上
	} 

    return( 0xFFFF );
}

uint8_t SetBootProto(uint8_t intf)
{
    uint8_t tmp[]= {0x21,0x0b,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t len,s;

	for ( s = 0; s != sizeof( tmp ); s ++ )
	{
		((__xdata uint8_t *)pSetupReq)[ s ] = tmp[s];
	}
    ((__xdata uint8_t *)pSetupReq)[ 4 ]=intf;
    s = HostCtrlTransfer( Com_Buffer, &len );                                     // 执行控制传输
    if ( s != ERR_SUCCESS )
    {
        return( s );
    }
    return( ERR_SUCCESS );
}

/*******************************************************************************
* Function Name  : InitUSB_Host
* Description    : 初始化USB主机
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  InitUSB_Host( void )
{
    uint8_t   i;
    IE_USB = 0;
//  LED_CFG = 1;
//  LED_RUN = 0;
    USB_CTRL = bUC_HOST_MODE;                                                    // 先设定模式
    UHOST_CTRL &= ~bUH_PD_DIS;                                                   //启用主机下拉
    USB_DEV_AD = 0x00;
    UH_EP_MOD = bUH_EP_TX_EN | bUH_EP_RX_EN ;
    UH_RX_DMA = (uint16_t)RxBuffer;
    UH_TX_DMA = (uint16_t)TxBuffer;
    UH_RX_CTRL = 0x00;
    UH_TX_CTRL = 0x00;
    USB_CTRL = bUC_HOST_MODE | bUC_INT_BUSY;// | bUC_DMA_EN;                     // 启动USB主机及DMA,在中断标志未清除前自动暂停
//  UHUB0_CTRL = 0x00;
//  UHUB1_CTRL = 0x00;
//  UH_SETUP = bUH_SOF_EN;
    USB_INT_FG = 0xFF;                                                           // 清中断标志
    for ( i = 0; i != 2; i ++ )
    {
        DisableRootHubPort( );                                                   // 清空
    }
    USB_INT_EN = bUIE_TRANSFER | bUIE_DETECT;
//  IE_USB = 1;                                                                  // 查询方式
}

