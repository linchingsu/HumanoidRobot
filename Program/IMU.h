

// 定义ITG3200陀螺仪常量
#define GYRO 0xD0 // 设置IIC地址,AD0与GND相接,二进制数值为11101000.
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E
 
#define G_TO_READ 8 // x, y, z 每个轴输出为2个字节
 
 
//  XYZ三轴偏移量修正
int g_offx = 1;  // 修正X轴误差
int g_offy = -33;   // 修正Y轴误差
int g_offz = 12;   // 修正Z轴误差
int hx, hy, hz, turetemp;


 
// 初始化陀螺仪
void initGyro()
{
  
  uint8 ini_data[2];
  /*****************************************
  * ITG 3200
  * 电源管理设置:
  * 时钟选择为内部振荡器
  * 无复位、无睡眠模式
  * 无待机模式
  * 采样率 = 1KHz
  * 参数为+ / - 2000度/秒
  * 低通滤波 = 5Hz
  * 没有中断
  ******************************************/
  ini_data[0]=0x00;  I2C_WriteNByte (GYRO, 1, G_PWR_MGM, ini_data, 1); //writeTo(GYRO, G_PWR_MGM, 0x00);
  ini_data[0]=0x07;  I2C_WriteNByte (GYRO, 1, G_SMPLRT_DIV, ini_data, 1);//writeTo(GYRO, G_SMPLRT_DIV, 0x07); // EB, 50, 80, 7F, DE, 23, 20, FF
  ini_data[0]=0x1E;  I2C_WriteNByte (GYRO, 1, G_DLPF_FS, ini_data, 1);//writeTo(GYRO, G_DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
  ini_data[0]=0x00;  I2C_WriteNByte (GYRO, 1, G_INT_CFG, ini_data, 1);//writeTo(GYRO, G_INT_CFG, 0x00);
}
 
 
void getGyroscopeData(int * result)
{
  /**************************************
  Gyro ITG-3200 I2C
  注册:
  temp MSB = 1B, temp LSB = 1C
  x axis MSB = 1D, x axis LSB = 1E
  y axis MSB = 1F, y axis LSB = 20
  z axis MSB = 21, z axis LSB = 22
  *************************************/
 
  int regAddress = 0x1B;
 
  //readFrom(GYRO, regAddress, G_TO_READ, buff); // 从ITG3200读取数据
  uint8 data_buf[8];
  I2C_ReadNByte (0xD0, 1, regAddress, data_buf, 8);
  result[0] = ((data_buf[2] << 8) | data_buf[3]);
  if( ( 0x80&data_buf[2]) )result[0]=result[0]-65535;
  result[0]=result[0]+g_offx; 

   //+ g_offx;
  result[1] = ((data_buf[4] << 8) | data_buf[5]); //+ g_offy;
  if( ( 0x80&data_buf[4]) )result[1]=result[1]-65535;
  result[1]=result[1]+g_offy;
  
  result[2] = ((data_buf[6] << 8) | data_buf[7]);// + g_offz;
  if( ( 0x80&data_buf[6]) )result[2]=result[2]-65535;
  result[2]=result[2]+g_offz;  

  result[3] = (data_buf[0] << 8) | data_buf[1]; // 温度
  if( ( 0x80&data_buf[0]) )result[3]=result[3]-65535;
}





void ADXL345_Init()
{
   uint8 ini_data[2];
   ini_data[0]=0x0B;  I2C_WriteNByte (0xA6, 1, 0x31, ini_data, 1);	//测量范围,正负16g，13位模式
   ini_data[0]=0x08;  I2C_WriteNByte (0xA6, 1, 0x2C, ini_data, 1);	//速率设定为12.5 参考pdf13页
   ini_data[0]=0x08;  I2C_WriteNByte (0xA6, 1, 0x2D, ini_data, 1);	 //选择电源模式   参考pdf24页
   ini_data[0]=0x80;  I2C_WriteNByte (0xA6, 1, 0x2E, ini_data, 1);	 //使能 DATA_READY 中断
   ini_data[0]=0x00;  I2C_WriteNByte (0xA6, 1, 0x1E, ini_data, 1);	 //X 偏移量 根据测试传感器的状态写入pdf29页
   ini_data[0]=0x00;  I2C_WriteNByte (0xA6, 1, 0x1F, ini_data, 1);	 //Y 偏移量 根据测试传感器的状态写入pdf29页
   ini_data[0]=0x00;  I2C_WriteNByte (0xA6, 1, 0x20, ini_data, 1);	 //Z 偏移量 根据测试传感器的状态写入pdf29页
}

int ADXL345_Test()
{
  	  uint8 data_buf[2]	;
      if(I2C_ReadNByte (0xA6, 1, 0x00, data_buf, 1))
	  {
	     return -1;  //	Fail to connect with I2c
	  }
	  else if(data_buf[0]==0xE5)
	  {
	      return 1; //Success
	  }
	  else 
	  {
	      return 0;	// Fail to Read ADXL345
	  } 

}
 
void get_accdata(double* Raccdata)
{
	int x_a;
	int y_a;
	int z_a;
	double x;
	double y;
	double z;

	double R;
	uint8 data_buf[6];
	I2C_ReadNByte (0xA6, 1, 0x32, data_buf, 6);
	x_a=(  (0x0F & data_buf[1]) <<8) |data_buf[0];
	if((0x80 & data_buf[1]))x_a=x_a-4095;


    y_a=(  (0x0F & data_buf[3]) <<8) |data_buf[2];
	if((0x80 & data_buf[3]))y_a=y_a-4095;


    z_a=(  (0x0F & data_buf[5]) <<8) |data_buf[4];
	if((0x80 & data_buf[5]))z_a=z_a-4095;


	R=sqrt(pow(x_a,2)+pow(y_a,2)+pow(z_a,2));
    x=x_a/R;
	y=0-y_a/R;
	z=0-z_a/R;
	if(R==0)
	{
        Raccdata[0]=0;		
		Raccdata[1]=0;
		Raccdata[2]=0;
	}
	else{
		Raccdata[0]=x;//atan2(x,z);  //Axz
		Raccdata[1]=y;//atan2(y,z);  //Ayz
   		Raccdata[2]=z;//atan2(x,y);  //Axy
	}
	/*
	Racc[0]=atan2(x,z)*180/3.14;  //Axz
	Racc[1]=atan2(y,z)*180/3.14;  //Ayz
   	Racc[2]=atan2(x,y)*180/3.14;  //Axy
	Racc[0]=acos(x_a/R)*180/3.14;
	Racc[1]=acos(y_a/R)*180/3.14;
 	Racc[2]=acos(z_a/R)*180/3.14;
	*/  
}




