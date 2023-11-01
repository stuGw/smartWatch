#include "CST816S.h"
#include "driver/gpio.h"

uint8_t Touch_Data[10];
//initialize device
esp_err_t cst816S_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    //CHECK_ARG(dev);

    dev->port = port;
    dev->addr = CTP_SLAVER_ADDR;
    dev->sda_io_num = sda_gpio;
    dev->scl_io_num = scl_gpio;
    dev->clk_speed = I2C_FREQ_HZ;
   // return 0;
    return i2c_master_init(port, sda_gpio, scl_gpio);
}

// Write register values to chip
void CST816SWriteReg(i2c_dev_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
  i2c_dev_write_reg(dev, reg, data, len);
}

// read register values to chip
uint8_t CST816SReadReg(i2c_dev_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
  return i2c_dev_read(dev, &reg, 1, data, len);

}

bool CST816SStart(i2c_dev_t *dev,  uint8_t res, uint8_t INT, uint8_t addr)
{

	/*_i2cPort = &port;
	_address = addr;
	_res = res;
	_INT = INT;*/
	//pinMode(_INT, INPUT_PULLUP);
	/* attachInterrupt(
		_INT, []
		{ isTouch = true },
		LOW); */
	//setReset();

	//reset touch
	    gpio_reset_pin(res);
	    /* Set the GPIO as a push/pull output */
	    gpio_set_direction(res, GPIO_MODE_OUTPUT);

	    gpio_set_level(res,0);
	    vTaskDelay(10);
	    gpio_set_level(res, 1);
	    vTaskDelay(50);



	/*_i2cPort->beginTransmission(_address);

	if (_i2cPort->endTransmission() != 0)
	{
		return false;
	}
*/
	uint8_t temp[1];

	//temp[0] = 0x00;
	//CST816SWriteReg(dev, DisAutoSleep, temp, 1); //默认为0，使能自动进入低功耗模式

	//temp[0] = 0x01;
	//CST816SWriteReg(dev, NorScanPer, temp, 1); //设置报点率

	//const uint8_t TOUCH_IRQ_EN_TOUCH		= 0x40;	//gives a lot of events, in itself only provide touch (TOUCH_CONTACT) info. also include gesture info
	//const uint8_t TOUCH_IRQ_EN_CHANGE		= 0x20;	//gives or adds the release (TOUCH_UP) info
	//const uint8_t TOUCH_IRQ_EN_MOTION		= 0x10;	//seems to add the GESTURE_TOUCH_BUTTON events, add long press-while-still-touched gestures
	//const uint8_t TOUCH_IRQ_EN_LONGPRESS	= 0x01;	//seems to do nothing..?

	//const uint8_t MOTION_MASK_CONTINUOUS_LEFT_RIGHT	= 0x04;
	//const uint8_t MOTION_MASK_CONTINUOUS_UP_DOWN	= 0x02;
	//const uint8_t MOTION_MASK_DOUBLE_CLICK			= 0x01;	//add hardware based double click




	temp[0] = 0x11;				//报点：0x60  手势：0X11  报点加手势：0X71
	CST816SWriteReg(dev, IrqCtl, temp, 1); //设置模式 报点/手势

	temp[0] = 0x0f;				//报点：0x60  手势：0X11  报点加手势：0X71
	CST816SWriteReg(dev, MotionMask, temp, 1); //设置模式 报点/手势

	uint8_t dis_auto_sleep = 0xFF;
	CST816SWriteReg(dev, DisAutoSleep, &dis_auto_sleep, 1);

	/*temp[0] = 0x05;				   //单位1S  为0时不启用功能  默认5
	CST816SWriteReg(dev, AutoReset, temp, 1); //设置自动复位时间  X秒内有触摸但无手势时，自动复位

	temp[0] = 0x10;					   //单位1S  为0时不启用功能  默认10
	CST816SWriteReg(dev, LongPressTime, temp, 1); //设置自动复位时间  长按X秒自动复位

	temp[0] = 0xff;					   //单位0.1mS
	CST816SWriteReg(dev, IrqPluseWidth, temp, 1); //设置中断低脉冲输出宽度

// 	temp[0] = 0x30;
//	_writeReg(LpScanTH, temp, 1); //设置低功耗扫描唤醒门限
//	temp[0] = 0x01;
//	_writeReg(LpScanWin, temp, 1); //设置低功耗扫描量程
//	temp[0] = 0x50;
//	_writeReg(LpScanFreq, temp, 1); //设置低功耗扫描频率
	
	temp[0] = 0x80;
	CST816SWriteReg(dev, LpScanIdac, temp, 1); //设置低功耗扫描电流

	temp[0] = 0x01;
	CST816SWriteReg(dev, AutoSleepTime, temp, 1); //设置1S进入低功耗
*/
	//CST816SReadReg(dev, 0x00, Touch_Data, 7);

	return true;
}
/*
// Reset the chip
void CST816S::setReset()
{
	pinMode(_res, OUTPUT);
	digitalWrite(_res, LOW);
	delay(10);
	digitalWrite(_res, HIGH);
	delay(50);
}

// Set I2C Address if different then default.
void CST816S::setADDR(uint8_t b)
{
	_address = b;
}

void CST816S::cst816s_deep_sleep(void)
{
}

bool CST816S::ReadTouch(void)
{
	if (!digitalRead(_INT))
	{
		_readReg(0x00, Touch_Data, 7);
		return true;
	}
	else
		return false;
}
*/
void CST816SReadTouch(i2c_dev_t* dev)
{
	if(CST816SReadReg(dev, 0x01, Touch_Data, 6) !=ESP_OK) printf("Reading touch error!\n");
}

uint8_t CST816SGetToucData(uint8_t index)
{
  return Touch_Data[index];
}
uint8_t CST816SGetTouchType(void)
{
	return Touch_Data[1];
}

uint16_t CST816SGetX(void)
{
	return ((uint16_t)(Touch_Data[2] & 0x0F) << 8) + (uint16_t)Touch_Data[3];
}

uint16_t CST816SGetY(void)
{
	return ((uint16_t)(Touch_Data[4] & 0x0F) << 8) + (uint16_t)Touch_Data[5];
}
