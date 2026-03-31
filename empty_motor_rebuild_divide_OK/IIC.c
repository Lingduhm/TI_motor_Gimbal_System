#include "IIC.h"

#define delay_us(X)		delay_cycles((CPUCLK_FREQ/1000000)*(X))

/******************************************************************
 * 函 数 名 称：IIC_Start
 * 函 数 说 明：IIC起始信号
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void IIC_Start(void)
{
        SDA_OUT(); // 包含SDA(1)
        SCL(1);
        delay_us(5);
        SDA(0); // 下降沿
        delay_us(5);
        SCL(0);
        delay_us(5);
}

/******************************************************************
 * 函 数 名 称：IIC_Stop
 * 函 数 说 明：IIC停止信号
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void IIC_Stop(void)
{
        SDA_OUT();
        SCL(0);
        SDA(0);
        SCL(1);
        delay_us(5);
        SDA(1); // 上升沿，此时SDA与SCL均为高电平，处于空闲状态
        delay_us(5);
}

/******************************************************************
 * 函 数 名 称：IIC_Send_Ack
 * 函 数 说 明：主机发送应答或者非应答
 * 函 数 形 参：0应答  1非应答
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void IIC_Send_Ack(uint8_t ack)
{
        SDA_OUT(); // 包含SDA(1)
        SCL(0);
        SDA(0);
        delay_us(5);
        if(!ack) SDA(0); // 0 = 应答
        else     SDA(1); // 1 = 非应答
        delay_us(5);
        SCL(1);
        delay_us(5);
        SCL(0);
        SDA(1); // 释放信号线
}

/******************************************************************
 * 函 数 名 称：IIC_Wait_Ack
 * 函 数 说 明：等待从机应答
 * 函 数 形 参：无
 * 函 数 返 回：1=无应答   0=有应答
 * 作       者：LC
 * 备       注：无
******************************************************************/
uint8_t IIC_Wait_Ack(void)
{
        uint8_t ack = 0;
        uint8_t ack_flag = 10;
        SDA_IN();
	    SCL(0);
        delay_us(5);
        SCL(1);
        delay_us(5);
        while( (SDA_GET() == 1) && ( ack_flag ) )
        {
                ack_flag--;
                delay_us(5);
        }

        if( ack_flag <= 0 )
        {
                IIC_Stop();
                return 1;
        }
        else
        {
                SCL(0);
                SDA_OUT();
        }
        return ack;
}
/******************************************************************
 * 函 数 名 称：IIC_Send_Byte
 * 函 数 说 明：IIC写一个字节
 * 函 数 形 参：dat写入的数据
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void IIC_Send_Byte(uint8_t dat)
{
    int i = 0;
    SDA_OUT();
    SCL(0);
    for(i = 0; i < 8; i++)
    {
        SDA( (dat & 0x80) >> 7 );
        delay_us(1);
        SCL(1);
        delay_us(5);
        SCL(0);
        delay_us(5);
        dat <<= 1;
    }
}

/******************************************************************
 * 函 数 名 称：IIC_Read_Byte
 * 函 数 说 明：IIC读1个字节
 * 函 数 形 参：无
 * 函 数 返 回：读出的1个字节数据
 * 作       者：LC
 * 备       注：无
******************************************************************/
unsigned char IIC_Read_Byte(void)
{
    unsigned char i = 0, receive = 0;
    SDA_IN();//SDA设置为输入
    for(i = 0; i < 8; i++)
    {
        SCL(0);
        delay_us(5);
        SCL(1);
        delay_us(5);
        receive <<= 1;
        if( SDA_GET() ) // 主机接收，如果电平读为高，receive最低位置1
        {
            receive |= 1;
        }
    }
    SCL(0);
    return receive;
}

/******************************************************************
 * 函 数 名 称：SHT20_Read
 * 函 数 说 明：测量温湿度（传感器从机模式下）
 * 函 数 形 参：regaddr=0xf3测量温度 =0xf5测量湿度
 * 函 数 返 回：regaddr=0xf3时返回温度，regaddr=0xf5时返回湿度 0=错误
 * 作       者：LC
 * 备       注：无
******************************************************************/
float SHT20_Read(uint8_t regaddr)
{
    //局部变量定义
    unsigned char data_H = 0;
    unsigned char data_L = 0;
    unsigned char check = 0;
    float temp = 0;

    //开启通信
    IIC_Start();
    //发送从机地址‘1000 000’ + 写位0
    IIC_Send_Byte(0x80|0);
    //等待从机应答
    if( IIC_Wait_Ack() == 1 )
		{
			printf("error -1\r\n");
			return 0;
		}

    //发送读写命令，读湿度（传感器非主机模式）0b1111_0101（0xf5） 温度（传感器非主机模式）0b1111_0011（0xf3）
    IIC_Send_Byte(regaddr);
    //等待从机应答
    if( IIC_Wait_Ack() == 1 )
		{
			printf("error -2\r\n");
			return 0;
		}

    //轮询发送起始信号与器件地址加读 0X80 ｜ 1 = 0x81，直到从机成功应答
    do{
        delay_us(10);
        IIC_Start();
        IIC_Send_Byte(0x80|1);
    }while( IIC_Wait_Ack() == 1 );

    delay_us(20);

    //存储从机发送来的数据，高位在前，低位在后，根据厂家协议，第二字节最低两位需在计算时置0
    data_H = IIC_Read_Byte();
    IIC_Send_Ack(0);
    data_L = IIC_Read_Byte();
    IIC_Send_Ack(0);
    check = IIC_Read_Byte();   //读取第三个字节（校验字节）
    IIC_Send_Ack(1);         //发送非应答
    //停止发送信号
    IIC_Stop();

    if( regaddr == 0xf3 )
    {
        temp = (((data_H<<8)|data_L) & ~(0x03)) / 65536.0 * 175.72 - 46.85;
    }
    if( regaddr == 0xf5 )
    {
        temp = (((data_H<<8)|data_L) & ~(0x03)) / 65536.0 * 125.0 - 6;
    }

   return temp;
}