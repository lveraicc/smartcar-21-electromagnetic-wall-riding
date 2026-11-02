#include "TOF.h"

// ================= I2C ?? =================
static void I2C_Delay(void)
{
    uint8 i;
    for(i=0; i<40; i++);
}

// ================= SDA ?? =================
static void SDA_IN(void)
{
    P6M1 &= ~(1<<5);
    P6M0 &= ~(1<<5);
}

static void SDA_OUT(void)
{
    P6M1 &= ~(1<<5);
    P6M0 |=  (1<<5);
}

// ================= I2C ?? =================
static void I2C_Start(void)
{
    SDA_OUT();
    TOF_SDA = 1;
    TOF_SCL = 1;
    I2C_Delay();
    TOF_SDA = 0;
    I2C_Delay();
    TOF_SCL = 0;
}

// ================= I2C ?? =================
static void I2C_Stop(void)
{
    SDA_OUT();
    TOF_SCL = 0;
    TOF_SDA = 0;
    I2C_Delay();
    TOF_SCL = 1;
    I2C_Delay();
    TOF_SDA = 1;
}

// ================= ?? ACK =================
static uint8 I2C_Wait_Ack(void)
{
    uint16 timeout = 0;
    SDA_IN();
    TOF_SCL = 1;
    I2C_Delay();

    while(TOF_SDA)
    {
        timeout++;
        if(timeout > 500)
        {
            I2C_Stop();
            return 1;
        }
    }
    TOF_SCL = 0;
    return 0;
}

// ================= ???? =================
static void I2C_Send_Byte(uint8 dat)
{
    uint8 i;
    SDA_OUT();
    for(i=0; i<8; i++)
    {
        TOF_SDA = (dat & 0x80) >> 7;
        dat <<= 1;
        TOF_SCL = 1;
        I2C_Delay();
        TOF_SCL = 0;
        I2C_Delay();
    }
}

// ================= ???? =================
static uint8 I2C_Read_Byte(uint8 ack)
{
    uint8 i, dat=0;
    SDA_IN();
    for(i=0; i<8; i++)
    {
        TOF_SCL = 1;
        dat <<= 1;
        if(TOF_SDA) dat |= 1;
        I2C_Delay();
        TOF_SCL = 0;
        I2C_Delay();
    }

    SDA_OUT();
    TOF_SDA = ack;  // 0=ACK, 1=NACK
    TOF_SCL = 1;
    I2C_Delay();
    TOF_SCL = 0;
    TOF_SDA = 1;
    return dat;
}

// ================= ?16???? =================
static void TOF_Write(uint16 reg, uint8 val)
{
    I2C_Start();
    I2C_Send_Byte(TOF_ADDR << 1);
    I2C_Wait_Ack();

    I2C_Send_Byte(reg >> 8);
    I2C_Wait_Ack();
    I2C_Send_Byte(reg & 0xFF);
    I2C_Wait_Ack();

    I2C_Send_Byte(val);
    I2C_Wait_Ack();
    I2C_Stop();
}

// ================= ?16???? =================
 uint8 TOF_Read(uint16 reg)
{
    uint8 val;
    I2C_Start();
    I2C_Send_Byte(TOF_ADDR << 1);
    if(I2C_Wait_Ack()) return 0xFF;

    I2C_Send_Byte(reg >> 8);
    I2C_Wait_Ack();
    I2C_Send_Byte(reg & 0xFF);
    I2C_Wait_Ack();

    I2C_Start();
    I2C_Send_Byte((TOF_ADDR << 1) | 1);
    if(I2C_Wait_Ack()) return 0xFF;

    val = I2C_Read_Byte(1); // ?1???NACK
    I2C_Stop();
    return val;
}

// ===================== TOF ???(?????)=====================
void TOF_Init(void)
{
    system_delay_ms(20);

    // ????????
    TOF_Write(0x0207, 0x01);
    TOF_Write(0x0208, 0x01);
    TOF_Write(0x0096, 0x00);
    TOF_Write(0x0097, 0xfd);
    TOF_Write(0x00e3, 0x00);
    TOF_Write(0x00e4, 0x04);
    TOF_Write(0x00e5, 0x02);
    TOF_Write(0x00e6, 0x01);
    TOF_Write(0x00e7, 0x03);
    TOF_Write(0x00f5, 0x02);
    TOF_Write(0x00d9, 0x05);
    TOF_Write(0x00db, 0xce);
    TOF_Write(0x00dc, 0x03);
    TOF_Write(0x00dd, 0xf8);
    TOF_Write(0x009f, 0x00);
    TOF_Write(0x00a3, 0x3c);
    TOF_Write(0x00b7, 0x00);
    TOF_Write(0x00bb, 0x3c);
    TOF_Write(0x00b2, 0x09);
    TOF_Write(0x00ca, 0x09);
    TOF_Write(0x0198, 0x01);
    TOF_Write(0x01b0, 0x17);
    TOF_Write(0x01ad, 0x00);
    TOF_Write(0x00ff, 0x05);
    TOF_Write(0x0100, 0x05);
    TOF_Write(0x0199, 0x05);
    TOF_Write(0x01a6, 0x1b);
    TOF_Write(0x01ac, 0x3e);
    TOF_Write(0x01a7, 0x1f);
    TOF_Write(0x0030, 0x00);

    // ??:??????
    TOF_Write(0x0011, 0x00); // ????
    system_delay_ms(5);
}

// ================= ? ID =================
uint8 TOF_Read_ID(void)
{
    return TOF_Read(0x0000);
}

// ================= ???(?????)=================
uint8 TOF_Read_Distance(void)
{
    uint8 status;
    uint16 timeout = 500;

    // 1. ??????
    TOF_Write(0x0015, 0x07);
    system_delay_ms(1);

    // 2. ??????
    TOF_Write(0x0018, 0x01);

    // 3. ????
    while(timeout--)
    {
        status = TOF_Read(0x004F);
        if((status & 0x04) != 0) //  bit2 ????
        {
            break;
        }
        system_delay_ms(1);
    }

    if(timeout == 0)
    {
        return 255;
    }

    // 4. ???
    return TOF_Read(0x0062);
}
