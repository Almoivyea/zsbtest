#ifndef __WITIMU_REG_H
#define __WITIMU_REG_H

/*寄存器表>>>>>>>>>>>>>>>>*/
#define REGSIZE 0x90

#define SAVE 		0x00
#define CALSW 		0x01
#define RSW 		0x02
#define RRATE		0x03
#define BAUD 		0x04
#define AXOFFSET	0x05
#define AYOFFSET	0x06
#define AZOFFSET	0x07
#define GXOFFSET	0x08
#define GYOFFSET	0x09
#define GZOFFSET	0x0a
#define HXOFFSET	0x0b`
#define HYOFFSET	0x0c
#define HZOFFSET	0x0d
#define D0MODE		0x0e
#define D1MODE		0x0f
#define D2MODE		0x10
#define D3MODE		0x11
#define D0PWMH		0x12
#define D1PWMH		0x13
#define D2PWMH		0x14
#define D3PWMH		0x15
#define D0PWMT		0x16
#define D1PWMT		0x17
#define D2PWMT		0x18
#define D3PWMT		0x19
#define IICADDR		0x1a
#define LEDOFF 		0x1b
#define MAGRANGX	0x1c
#define MAGRANGY	0x1d
#define MAGRANGZ	0x1e
#define BANDWIDTH	0x1f
#define GYRORANGE	0x20
#define ACCRANGE	0x21
#define SLEEP       0x22
#define ORIENT		0x23
#define AXIS6       0x24
#define FILTK       0x25
#define GPSBAUD		0x26
#define READADDR	0x27
#define BWSCALE		0x28
#define MOVETHR		0x28
#define MOVESTA		0x29
#define ACCFILT		0x2A
#define GYROFILT	0x2b
#define MAGFILT		0x2c
#define POWONSEND	0x2d
#define VERSION		0x2e
#define CCBW			  0x2f
#define YYMM				0x30
#define DDHH				0x31
#define MMSS				0x32
#define MS					0x33
#define AX					0x34
#define AY					0x35
#define AZ					0x36
#define GX					0x37
#define GY					0x38
#define GZ					0x39
#define HX					0x3a
#define HY					0x3b
#define HZ					0x3c
#define Roll				0x3d
#define Pitch				0x3e
#define Yaw					0x3f
#define TEMP				0x40
#define D0Status		0x41
#define D1Status		0x42
#define D2Status		0x43
#define D3Status		0x44
#define PressureL		0x45
#define PressureH		0x46
#define HeightL			0x47
#define HeightH			0x48
#define LonL				0x49
#define LonH				0x4a
#define LatL				0x4b
#define LatH				0x4c
#define GPSHeight       0x4d
#define GPSYAW          0x4e
#define GPSVL				0x4f
#define GPSVH				0x50
#define q0					0x51
#define q1					0x52
#define q2					0x53
#define q3					0x54
#define SVNUM				0x55
#define PDOP				0x56
#define HDOP				0x57
#define VDOP				0x58
#define DELAYT			0x59
#define XMIN            0x5a
#define XMAX            0x5b
#define BATVAL          0x5c
#define ALARMPIN        0x5d
#define YMIN            0x5e
#define YMAX            0x5f
#define GYROZSCALE		0x60
#define GYROCALITHR     0x61
#define ALARMLEVEL      0x62
#define GYROCALTIME		0x63
#define REFROLL			0x64
#define REFPITCH		0x65
#define REFYAW			0x66
#define GPSTYPE     0x67
#define TRIGTIME    0x68
#define KEY         0x69
#define WERROR      0x6a
#define TIMEZONE    0x6b
#define CALICNT     0x6c
#define WZCNT       0x6d
#define WZTIME      0x6e
#define WZSTATIC    0x6f
#define ACCSENSOR 	0x70
#define GYROSENSOR 	0x71
#define MAGSENSOR 	0x72
#define PRESSENSOR 	0x73
#define MODDELAY    0x74

#define ANGLEAXIS   0x75
#define XRSCALE			0x76    
#define YRSCALE			0x77
#define ZRSCALE			0x78

#define XREFROLL		0x79    
#define YREFPITCH		0x7a
#define ZREFYAW			0x7b

#define ANGXOFFSET		0x7c    
#define ANGYOFFSET		0x7d
#define ANGZOFFSET		0x7e

#define NUMBERID1    0x7f
#define NUMBERID2    0x80
#define NUMBERID3    0x81
#define NUMBERID4    0x82
#define NUMBERID5    0x83
#define NUMBERID6    0x84

#define XA85PSCALE       0x85
#define XA85NSCALE       0x86
#define YA85PSCALE       0x87
#define YA85NSCALE       0x88
#define XA30PSCALE       0x89
#define XA30NSCALE       0x8a
#define YA30PSCALE       0x8b
#define YA30NSCALE       0x8c

#define CHIPIDL     0x8D
#define CHIPIDH     0x8E
#define REGINITFLAG       REGSIZE-1 

/*<<<<<<<<<<<<<<<<<<<<<<<寄存器表*/


// 协议: 帧头(0x55) + 输出帧头(数据类型) + 数据 + 和校验位
/*OUTPUTHEAD 输出帧头>>>>>>>>>>*/
#define WIT_TIME        0x50
#define WIT_ACC         0x51
#define WIT_GYRO        0x52
#define WIT_ANGLE       0x53
#define WIT_MAGNETIC    0x54
#define WIT_DPORT       0x55
#define WIT_PRESS       0x56
#define WIT_GPS         0x57
#define WIT_VELOCITY    0x58
#define WIT_QUATER      0x59
#define WIT_GSA         0x5A
#define WIT_REGVALUE    0x5F
/*<<<<<<<<<<<<<<<<<<<<<输出帧头*/





#endif
