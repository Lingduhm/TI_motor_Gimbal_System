#include "SPI.h"

/**
 * @brief  SPI单字节全双工收发（修正时序，删除冗余等待）
 * @param  dat: 发送的字节
 * @return 接收到的字节
 */
uint8_t spi_read_write_byte(uint8_t dat)
{
        uint8_t data = 0;

        //发送数据
        DL_SPI_transmitData8(SPI_INST,dat);
        //等待SPI总线空闲
        while(DL_SPI_isBusy(SPI_INST));
        //接收数据
        data = DL_SPI_receiveData8(SPI_INST);

        // SPI 全双工 = 发 8 位的同时，收 8 位，同一个时钟同步完成，收发是同一时刻结束，对于DI和DO，主从机的采样沿都是同一个，也就是有效电平对应的沿；而另一个沿则是放上下一帧要传数据的，也就是切换电平的时候
        // 主机有硬件接收缓存（你说的完全对！），数据自动存好
        // receiveData8 只是读缓存，不占用 SPI 总线，所以不需要第二次等待
        // //等待SPI总线空闲（冗余）
        // while(DL_SPI_isBusy(SPI_INST));

        return data;
}

// W25Q128 16MB
// 1 字节：最小读写单位（8 位）
// 1 扇区 = 4096 字节（4KB）：最小擦除单位
// 1 块 = 16 个扇区 = 65536 字节（64KB）：大块擦除单位
// W25Q128 硬件规则
// 芯片必须先收完完整的【指令 + 24 位地址】，才会开始输出有效数据；
// 收指令 / 地址的阶段，芯片只收不发有效数据，默认返回 0xFF（MISO主机输入从机输出线 总线空闲高电平）

// W25Q128 核心参数定义
#define W25Q128_MAX_ADDR     0xFFFFFF    // 16MB 最大字节地址
#define W25Q128_SECTOR_SIZE  4096        // 1扇区=4096字节
#define W25Q128_PAGE_SIZE    256         // 1页=256字节（写入最小限制）
#define W25Q128_MAX_SECTOR   4095        // 最大扇区号

//读取芯片ID
//返回值如下:
//0XEF13,表示芯片型号为W25Q80
//0XEF14,表示芯片型号为W25Q16
//0XEF15,表示芯片型号为W25Q32
//0XEF16,表示芯片型号为W25Q64
//0XEF17,表示芯片型号为W25Q128
//读取制造商ID + 设备ID（共2字节）
/**
 * @brief  读取芯片ID
 * @return 0xEF17 = W25Q128
 */
uint16_t W25Q128_readID(void)
{
    uint16_t  temp = 0;
    //将CS端拉低为低电平
    SPI_CS(0);
    //发送指令90h
    spi_read_write_byte(0x90);//发送读取ID命令
    //发送地址  000000H
    spi_read_write_byte(0x00);
    spi_read_write_byte(0x00);
    spi_read_write_byte(0x00);

    //接收数据
    //接收制造商ID
    temp |= spi_read_write_byte(0xFF)<<8;
    //接收设备ID
    temp |= spi_read_write_byte(0xFF);
    //恢复CS端为高电平 结束访问
    SPI_CS(1);
    //返回ID
    return temp;
}

// 发送写使能, 否则芯片无法写入数据
void W25Q128_write_enable(void)
{
    //拉低CS端为低电平
    SPI_CS(0);
    //发送指令06h
    spi_read_write_byte(0x06);
    //拉高CS端为高电平
    SPI_CS(1);
}

// W25Q128 的 BUSY 位（BIT0）硬件规则：
// 仅在扇区擦除、块擦除、页写入为忙1（这三个是芯片内部的物理存储操作，耗时毫秒级）
// 读取操作 只是 SPI 实时通信，芯片不需要耗时处理，所以硬件 BUSY 位置 0
// 通过状态寄存器1的S0位进行判断，读取状态寄存器1的指令为0X05。
void W25Q128_wait_busy(void)
{
	unsigned char byte = 0; //1字节
	do
	 {
			//拉低CS端为低电平
			SPI_CS(0);
			//发送指令05h
			spi_read_write_byte(0x05);
			//接收状态寄存器值
			byte = spi_read_write_byte(0xFF);
			//恢复CS端为高电平
			SPI_CS(1);
	 //判断BUSY位是否为1 如果为1说明在忙，重新读写BUSY位直到为0
	 }while( ( byte & 0x01 ) == 1 );
}

/**
 * @brief  扇区擦除（4KB）
 * @param  addr: 扇区号(0~4095)
 */
void W25Q128_erase_sector(uint32_t addr)
{
    // 扇区号范围校验（新增保护）
    if(addr > W25Q128_MAX_SECTOR) return;
	//计算扇区地址，地址 = 扇区号 * 4096
	addr *= 4096;
	W25Q128_write_enable();  //写使能
	W25Q128_wait_busy();     //判断忙，如果忙则一直等待
	//拉低CS端为低电平
	SPI_CS(0);
	//发送指令20h
	spi_read_write_byte(0x20);
	//发送24位扇区地址的高8位
	spi_read_write_byte((uint8_t)((addr)>>16));
	//发送24位扇区地址的中8位
	spi_read_write_byte((uint8_t)((addr)>>8));
	//发送24位扇区地址的低8位
	spi_read_write_byte((uint8_t)addr);
	//恢复CS端为高电平
	SPI_CS(1);
	//等待擦除完成
	W25Q128_wait_busy();
}

/**
 * @brief  分页写入数据（核心修正：处理256字节页限制）
 * @param  buffer: 数据指针
 * @param  addr: 字节地址
 * @param  numbyte: 数据长度
 */
void W25Q128_write(uint8_t* buffer, uint32_t addr, uint16_t numbyte)
{
    uint16_t i;
    uint16_t page_remain;
    uint16_t write_size;

    // 地址越界保护
    if(addr + numbyte > W25Q128_MAX_ADDR + 1) return;// 同为地址 从0开始，需要对addr本身的那一位进行额外处理
    if(buffer == NULL) return;

    // 擦除所在扇区（注：会清空整个扇区数据）
    W25Q128_erase_sector(addr / W25Q128_SECTOR_SIZE);

    while(numbyte > 0)
    {
        // 计算当前页剩余可写入字节数 addr从0开始，对于第一个页0～255，若为254，余数2即为实际能写入的字节数，无需用1凑
        page_remain = W25Q128_PAGE_SIZE - (addr % W25Q128_PAGE_SIZE);
        // 本次实际写入长度（不跨页）
        write_size = (numbyte < page_remain) ? numbyte : page_remain;

        W25Q128_write_enable();
        W25Q128_wait_busy();

        SPI_CS(0);
        spi_read_write_byte(0x02);      // 页写入指令 高位在前
        spi_read_write_byte((uint8_t)(addr >> 16));
        spi_read_write_byte((uint8_t)(addr >> 8));
        spi_read_write_byte((uint8_t)addr);

        for(i = 0; i < write_size; i++)
        {
            spi_read_write_byte(buffer[i]);
        }
        SPI_CS(1);
        W25Q128_wait_busy();

        // 更新指针和长度
        buffer += write_size;
        addr += write_size;
        numbyte -= write_size;
    }
}

/**
 * @brief  读取数据
 * @param  buffer: 接收缓存
 * @param  read_addr: 字节地址
 * @param  read_length: 读取长度
 */
void W25Q128_read(uint8_t* buffer,uint32_t read_addr,uint16_t read_length)
{
	uint16_t i;
	//拉低CS端为低电平
	SPI_CS(0);
	//发送指令03h
	spi_read_write_byte(0x03);
	//发送24位读取数据地址的高8位，字节地址 
	spi_read_write_byte((uint8_t)((read_addr)>>16));
	//发送24位读取数据地址的中8位
	spi_read_write_byte((uint8_t)((read_addr)>>8));
	//发送24位读取数据地址的低8位
	spi_read_write_byte((uint8_t)read_addr);
	//根据读取长度读取出地址保存到buffer中
	for(i=0;i<read_length;i++)
	{
		buffer[i]= spi_read_write_byte(0XFF);
	}
	//恢复CS端为高电平
	SPI_CS(1);
}


