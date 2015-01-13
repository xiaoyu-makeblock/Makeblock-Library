#include "ch375.h"

#define p_dev_descr ((PUSB_DEV_DESCR)RECV_BUFFER)
#define p_cfg_descr ((PUSB_CFG_DESCR_LONG)RECV_BUFFER)

unsigned char endp_out_addr;
unsigned char endp_out_size;
unsigned char endp_in_addr;
unsigned char endp6_mode, endp7_mode;

unsigned char *cmd_buf;
unsigned char *ret_buf;
PUSB_ENDP_DESCR tmpEp;
// softserial delays too much, only hardware serial now
#if defined(__AVR_ATmega32U4__)
  #define HSerial Serial1
#else
  #define HSerial Serial
#endif

//#define CH375_DBG

CH375::CH375()
{
}

unsigned char CH375::CH375_RD()
{
  delay(2); // stupid delay, the chip don't got any buffer
  if(HSerial.available()){
    unsigned char c = HSerial.read();
#ifdef CH375_DBG   
    Serial.printf("<<%x\r\n",c);
#endif
    return c;
  }
  return 0;
}

void CH375::CH375_WR(unsigned char c)
{
  HSerial.write(c);
  delay(2);
#ifdef CH375_DBG   
  Serial.printf(">>%x\r\n",c);
#endif
}

int CH375::set_usb_mode(int mode)
{
  CH375_WR(CMD_SET_USB_MODE);
  CH375_WR(mode);
  endp6_mode=endp7_mode=0x80;
  return CH375_RD();
}

unsigned char CH375::getIrq()
{
  CH375_WR(CMD_GET_STATUS);
  delay(20);
  return CH375_RD();
}

void CH375::toggle_send()
{
#ifdef CH375_DBG   
  Serial.printf("toggle send %x\r\n",endp7_mode);
#endif
  CH375_WR(CMD_SET_ENDP7);
  CH375_WR( endp7_mode );
  endp7_mode^=0x40;
}

void CH375::toggle_recv()
{
  CH375_WR( CMD_SET_ENDP6 );
  CH375_WR( endp6_mode );
#ifdef CH375_DBG   
  Serial.printf("toggle recv:%x\r\n", endp6_mode);
#endif
  endp6_mode^=0x40;
}


unsigned char CH375::issue_token( unsigned char endp_and_pid )
{
  CH375_WR( CMD_ISSUE_TOKEN );
  CH375_WR( endp_and_pid );  /* 高4位目的端点号, 低4位令牌PID */
#ifdef CH375_DBG
  Serial.printf("issue token %x\r\n",endp_and_pid);
#endif
  delay(2);
  return getIrq();
}

void CH375::wr_usb_data( unsigned char len, unsigned char *buf )
{
#ifdef CH375_DBG   
  Serial.printf("usb wr %d\r\n",len);
#endif
  CH375_WR( CMD_WR_USB_DATA7 );
  CH375_WR( len );
  while( len-- ){
    CH375_WR( *buf++ );
  }
}

unsigned char CH375::rd_usb_data( unsigned char *buf )
{
  unsigned char i, len;
  CH375_WR( CMD_RD_USB_DATA );
  len=CH375_RD();
#ifdef CH375_DBG
  Serial.printf("usb rd %d\r\n",len);
#endif
  for ( i=0; i!=len; i++ ) *buf++=CH375_RD();
  return( len );
}

int CH375::get_version(){
  CH375_WR(CMD_GET_IC_VER);
  return CH375_RD();
}

void CH375::set_freq(void)
{
  CH375_WR(0x0b);
  CH375_WR(0x17);
  CH375_WR(0xd8);
}

unsigned char CH375::set_addr( unsigned char addr )
{
  unsigned char irq;
  CH375_WR(CMD_SET_ADDRESS);
  CH375_WR(addr);
  irq = getIrq();
  if(irq==USB_INT_SUCCESS){
    CH375_WR(CMD_SET_USB_ADDR);
    CH375_WR(addr);
  }
  return irq;
}

unsigned char CH375::set_config(unsigned char cfg){
  endp6_mode=endp7_mode=0x80; // reset the sync flags
  CH375_WR(CMD_SET_CONFIG);
  CH375_WR(cfg);
  return getIrq();
}

unsigned char CH375::clr_stall6(void)
{
  CH375_WR( CMD_CLR_STALL );
  CH375_WR( endp_out_addr | 0x80 );
  endp6_mode=0x80;
  return getIrq();
}

unsigned char CH375::get_desr(unsigned char type)
{
  CH375_WR( CMD_GET_DESCR );
  CH375_WR( type );  /* 描述符类型, 只支持1(设备)或者2(配置) */
  return getIrq();
}

unsigned char CH375::host_recv()
{
  unsigned char irq;
  toggle_recv();
  irq = issue_token( ( endp_in_addr << 4 ) | DEF_USB_PID_IN );
  if(irq==USB_INT_SUCCESS){
    int len = rd_usb_data(RECV_BUFFER);
#ifdef CH375_DBG
    for(int i=0;i<len;i++){
      // point hid device
      Serial.printf(" 0x%x",(int)RECV_BUFFER[i]);
    }
    Serial.println();
#endif
    stallCount = 0;
    return len;
  }else if(irq==USB_INT_DISCONNECT){
    device_online = false;
    device_ready = false;
#ifdef CH375_DBG   
    Serial.println("##### disconn #####");
#endif
    return 0;
  }else{
    clr_stall6();
#ifdef CH375_DBG   
    Serial.println("##### stall #####");
#endif
    delay(10);
    /*
    stallCount++;
    if(stallCount>10){
      device_online = false;
      device_ready = false;
      resetBus();
    }
    */
    return 0;
  }
}

void CH375::resetBus()
{
  int c;
  c = set_usb_mode(7);
#ifdef CH375_DBG   
  Serial.printf("set mode 7: %x\n",c);
#endif
  delay(10);
  c = set_usb_mode(6);
#ifdef CH375_DBG   
  Serial.printf("set mode 6: %x\n",c);
#endif
  delay(10);
}

void CH375::init(char type)
{
  ch375_online = false;
  device_online = false;
  device_ready = false;
  usbtype = type;
  HSerial.begin(9600);
}

int CH375::initHIDDevice()
{
  int irq, len, address;
  if(usbtype==USB1_0) set_freq(); //work on a lower freq, necessary for ch375
  irq = get_desr(1);
#ifdef CH375_DBG   
  Serial.printf("get des irq:%x\n",irq);
#endif
  if(irq==USB_INT_SUCCESS){
      len = rd_usb_data( RECV_BUFFER );
#ifdef CH375_DBG   
      Serial.printf("descr1 len %d type %x\r\n",len,p_dev_descr->bDescriptorType);
#endif
      irq = set_addr(2);
      if(irq==USB_INT_SUCCESS){
        irq = get_desr(2); // max buf 64byte, todo:config descr overflow
        if(irq==USB_INT_SUCCESS){
          len = rd_usb_data( RECV_BUFFER );
#ifdef CH375_DBG   
          Serial.printf("descr2 len %d class %x subclass %x\r\n",len,p_cfg_descr->itf_descr.bInterfaceClass, p_cfg_descr->itf_descr.bInterfaceSubClass); // interface class should be 0x03 for HID
          Serial.printf("num of ep %d\r\n",p_cfg_descr->itf_descr.bNumEndpoints);
          Serial.printf("ep0 %x %x\r\n",p_cfg_descr->endp_descr[0].bLength, p_cfg_descr->endp_descr[0].bDescriptorType);
#endif
          if(p_cfg_descr->endp_descr[0].bDescriptorType==0x21){ // skip hid des
            tmpEp = (PUSB_ENDP_DESCR)((char*)(&(p_cfg_descr->endp_descr[0]))+p_cfg_descr->endp_descr[0].bLength); // get the real ep position
          }
#ifdef CH375_DBG   
          Serial.printf("endpoint %x %x\r\n",tmpEp->bEndpointAddress,tmpEp->bDescriptorType);
#endif
          endp_out_addr=endp_in_addr=0;
          address =tmpEp->bEndpointAddress;  /* 第一个端点的地址 */
          // actually we only care about the input end points
          if( address&0x80 ){
            endp_in_addr = address&0x0f;  /* IN端点的地址 */
          }else{  /* OUT端点 */
            endp_out_addr = address&0x0f;
            endp_out_size = p_cfg_descr->endp_descr[0].wMaxPacketSize;  /* 
            数据接收端点的最大包长度 */
            if( endp_out_size == 0 || endp_out_size > 64 )
              endp_out_size = 64;
          }
          // todo: some joystick with more than 2 node
          // just assume every thing is fine, bring the device up
          irq = set_config(p_cfg_descr->cfg_descr.bConfigurationvalue);
          if(irq==USB_INT_SUCCESS){
            CH375_WR( CMD_SET_RETRY );  // set the retry times
            CH375_WR( 0x25 );
            CH375_WR( 0x85 );
            device_ready = true;
            return 1;
          }
        }
        
      }
  }
  return 0;
}

int CH375::probeDevice()
{
  int c;
  if(!ch375_online){
    CH375_WR( CMD_CHECK_EXIST );
    CH375_WR( 0x5A);
    c = CH375_RD(); // should return 0xA5
    if(c!=0xA5) return 0;
    ch375_online = true;
    resetBus();
  }
  
  c = getIrq();
  if(c!=USB_INT_CONNECT) return 0;
  resetBus(); // reset bus and wait the device online again
  c=0;
  while(c!=USB_INT_CONNECT){
    delay(500); // some device may need long time to get ready
    c = getIrq();
  }
  if( initHIDDevice()==1)
    device_online=true;
}


