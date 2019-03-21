#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include <string.h>
#include "stm32f10x.h"
#include "wifi.h"
#define RESV_CLIENT_CHANNEL_OFFSET     11
#define WIFI_RX_BUF_LENGTH             124
#define BUFSZ                          512

typedef struct wifi_tx_msg
{
	rt_size_t   channel;
	rt_uint8_t  data[WIFI_RX_BUF_LENGTH];
}wifi_tx_msg;

typedef struct wifi_rx_msg
{
	rt_size_t len;
	rt_uint8_t cmd[WIFI_RX_BUF_LENGTH];
}wifi_rx_msg;

ALIGN(RT_ALIGN_SIZE)
wifi_rx_msg   rx_msg;
wifi_tx_msg   tx_msg;
static int 		rx_msg_offset;
static rt_device_t  wifi_dev;
static rt_uint8_t  uart2_buf[BUFSZ];
/* rx message buffer */
static rt_mq_t rx_mq_t;
static rt_mq_t tx_mq_t;
static struct rt_semaphore wifi_rx_sem;

const char *wifi_start_cmd[7] = {
						"AT+RST\r\n",       	//复位
						"AT+CWMODE=2\r\n",  	//AP模式
						"AT+CWSAP=\"SnakeRobot\",\"smielab510\",1,3\r\n",//设置wifi 用户名：通道号: 1 加密模式 WPA2_PSK
						"AT+RST\r\n",					//复位
						"AT+CIPMUX=1\r\n",				//多路复用复位清零  server必须为1 
						"AT+CIPSERVER=1,8080\r\n",//设置server和端口
						"AT+CIPSTO=2880\r\n"			//设置tcp超时时间 ms
};

char wifi_send_cmd[32];//"AT+CIPSEND=%d,%d";
char wifi_test[32] = "Hello Wi-Fi!";

static rt_err_t wifi_rx_ind(rt_device_t dev, rt_size_t size)
{
	char ch;
	RT_ASSERT(wifi_dev != RT_NULL);
	rt_sem_release(&wifi_rx_sem);
//rt_kprintf("rx_in\n");
//	/* release semaphore to let finsh thread rx data */
//	for(int i = 0;i < size;i++)
//	{
//		rt_device_read(wifi_dev,0,&ch,1);
//		rt_kprintf("a\n");
//		if(ch == '\r'|| ch == '\n')
//		{
//			rt_kprintf("b\n");
//			if(ch != '\n') rt_device_read(wifi_dev,0,&ch,1);//abandon \r\n
//			else break;
//			rx_msg.len = rx_msg_offset;
//			rt_mq_send(rx_mq_t, (void *)&rx_msg,sizeof(wifi_rx_msg));//send msg
//			rx_msg_offset = 0;// reset offset
//			rt_kprintf("c\n");
//			break;		
//		} else {
//			rt_kprintf("d\n");
//				if (rx_msg_offset < WIFI_RX_BUF_LENGTH) {
//						rx_msg.cmd[rx_msg_offset++] = ch;
//						rt_kprintf("e\n");
//				} else
//				{
//					rx_msg.len = rx_msg_offset;
//					rt_mq_send(rx_mq_t, (void *)&rx_msg,sizeof(wifi_rx_msg));// send msg
//					rx_msg_offset = 0;// reset system offset
//					rt_kprintf("f\n");
//					break;
//				}
//		}
//	}
//	rt_kprintf("rx_exit\n");
	return RT_EOK;
}

/* rx message buffer */

void wifi_rx_entry(void *parameter)
{	
	rt_err_t err;
	int com_numb = 0,send_numb = 0;
	int size;
	char *buf;
	wifi_rx_msg msg;
	buf = (char *)uart2_buf;
	rt_device_set_rx_indicate(wifi_dev, wifi_rx_ind);//最后设置回调函数
	while(1)
	{
		//rt_mq_recv(rx_mq_t, &msg, sizeof(wifi_rx_msg), RT_WAITING_FOREVER);
		err = rt_sem_take(&wifi_rx_sem, RT_WAITING_FOREVER);
		if(err == RT_EOK )
		{
			size = rt_device_read(wifi_dev, 0, buf, BUFSZ);
			if(0 == size) continue;
			for(int u = 0;u < size;u++)
				rt_kprintf("%c",buf[u]);//这部分需要对数据包做解析处理 如："0,CONNECT" 获取其通道号和连接状态
		
		}
	}
}

/* 创建can_rx子线程 */
void thread_wifi_rx_init(void)
{
	rt_thread_t tid;
	rt_mq_create("rx_mq",sizeof(wifi_rx_msg),5*sizeof(wifi_rx_msg),	RT_IPC_FLAG_FIFO);
	
	tid = rt_thread_create( "wifi_rx",
													wifi_rx_entry,
													RT_NULL,
													512,
													8,10);
  if (tid != RT_EOK)
				rt_thread_startup(tid);
}

//////////////////     TX      ////////////////////

void wifi_tx_entry(void *parameter)
{
//	int length;
	int com_numb = 0,send_numb = 0;
	char buf[30];
	while(1)
	{
		//wait for new message
		rt_mq_recv(tx_mq_t, &tx_msg, sizeof(wifi_tx_msg), RT_WAITING_FOREVER);
#if 1
		rt_sprintf(wifi_send_cmd,"AT+CIPSEND=%d,%d\r\n",com_numb,strlen(wifi_test));//发数据的格式 真正数据来自tx_msg
		/////////////just for test
		rt_device_write(wifi_dev,0,wifi_send_cmd,strlen(wifi_send_cmd));
		rt_device_read(wifi_dev,0,buf,10);
		rt_thread_delay(10);//TODO:  wait for ">" 
		rt_device_write(wifi_dev,0,wifi_test,strlen(wifi_test));
#endif	
		
//if all mailbox are no availablex
//		switch(tx_msg.cmd)
//		{
//			default: break;
//		}
//		length = rt_sprintf(wifi_send_cmd,"AT+CIPSEND=%d,%d\r\n",com_numb,send_numb=strlen(wifi_test));
//		rt_device_write(wifi_dev,0,wifi_send_cmd,strlen(wifi_send_cmd));
//		rt_device_read(wifi_dev,0,buf,10);
//		rt_kprintf("recieve: %s\n",buf);
//		//rt_thread_delay(10);//TODO:  wait for ">" 
//		strcat(wifi_test,"\r\n");
//		rt_device_write(wifi_dev,0,wifi_test,strlen(wifi_test));  
		  rt_thread_delay(100);
	}
}


/* 创建wifi tx 子线程 */
void thread_wifi_tx_init(void)
{
  rt_thread_t tid;
	rt_mq_create( "tx_mq", 
								sizeof(wifi_tx_msg), 
								5*sizeof(wifi_tx_msg), 
								RT_IPC_FLAG_FIFO);
	
	tid = rt_thread_create( "wifi_tx",
													 wifi_tx_entry,
													 RT_NULL,
													 512,
													 8,10);
  if (tid != RT_NULL)
			rt_thread_startup(tid);
}


void rx_tx_thread_init(void)
{
	thread_wifi_rx_init();
	thread_wifi_tx_init();
}
void wifi_thread_entry(void *parameter)
{
		rt_uint8_t i;
	  char buf[100];
	  const char *s="OK";
		//init wifi
		wifi_dev = rt_device_find("uart2");// PA2-->TX  PA3-->RX
		rt_sem_init(&wifi_rx_sem, "wifi_sem", 0, RT_IPC_FLAG_FIFO);
	  if(wifi_dev == RT_NULL) {
			rt_kprintf("error\n");
		} else
		{
			rt_device_open(wifi_dev,RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
		}
		rt_device_write(wifi_dev,0,"x",1);
		rt_device_read(wifi_dev,0,buf,1);// 先读避免第一个字节丢失
		for(i = 0;i < sizeof(wifi_start_cmd)/sizeof(wifi_start_cmd[0]);i++)
		{
			rt_device_write(wifi_dev,0,wifi_start_cmd[i],strlen(wifi_start_cmd[i]));
			rt_thread_delay(500);
			memset(buf,0,sizeof buf);
			rt_device_read(wifi_dev,0,buf,sizeof buf);
			rt_kprintf("%s\n",buf);
		  
			for(int j = 0;j < rt_strlen(s);j++)
				if (s[j] !=  buf[j]) {
						//rt_kprintf("error \n");break;
				}
		}
		rx_tx_thread_init();//thread init
		
		while(1)
		{
				rt_thread_delay(100);
			// rt_uint8_t ch;
        /* wait receive */
//				while (rt_device_read(wifi_dev, 0, &ch, 1) == 1)
//				{
//					//rt_device_write();
//				}
		}
}
