/*
 * This file is part of libphidget21
 *
 * Copyright 2006-2015 Phidgets Inc <patrick@phidgets.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see 
 * <http://www.gnu.org/licenses/>
 */

#include "stdafx.h"
#include "cphidgettextled.h"
#include "cusb.h"
#include "csocket.h"
#include "cthread.h"

// === Internal Functions === //

//clearVars - sets all device variables to unknown state
CPHIDGETCLEARVARS(TextLED)

	phid->brightness = PUNK_INT;

	return EPHIDGET_OK;
}

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
CPHIDGETINIT(TextLED)
	TESTPTR(phid);

	//Make sure no old writes are still pending
	phid->outputPacketLen = 0;

	//set variables to unknown
	phid->brightness = PUNK_INT;

	return EPHIDGET_OK;
}

//dataInput - parses device packets - not used
CPHIDGETDATA(TextLED)
	phid = 0;
	return EPHIDGET_OK;
}

//eventsAfterOpen - sends out an event for all valid data, used during attach initialization - not used
CPHIDGETINITEVENTS(TextLED)
	phid = 0;
	return EPHIDGET_OK;
}

//getPacket - used by write thread to get the next packet to send to device
CGETPACKET_BUF(TextLED)

//sendpacket - sends a packet to the device asynchronously, blocking if the 1-packet queue is full
CSENDPACKET_BUF(TextLED)

const unsigned long LED_Brightness[33] = {
	0x00000000,
	0x00000001,
	0x00010001,
	0x01010001,

	0x01010101,
	0x01110101,
	0x01110111,
	0x11110111,

	0x11111111,
	0x11111115,
	0x11151115,
	0x15151115,

	0x15151515,
	0x55151515,
	0x55155515,
	0x55555515,

	0x55555555,
	0x55555557,
	0x55575557,
	0x57575557,

	0x57575757,
	0x77575757,
	0x77775757,
	0x77777757,

	0x77777777,
	0x7777777F,
	0x777F777F,
	0x7F7F777F,
	0x7F7F7F7F,

	0x7FFF7F7F,
	0x7FFF7FFF,
	0x7FFFFFFF,
	0xFFFFFFFF};

//BL: Is this the same mapping as the LCD custom chars?
const unsigned char LED_Translate[256] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00,
	0x7E, 0x42, 0x3B, 0x6B, 0x47, 0x6D, 0x7D, 0x4A,
	0x7F, 0x6F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	0x00, 0x5F, 0x7F, 0x3C, 0x73, 0x3D, 0x1D, 0x7C, /* A,B,C,D,E,F,G */
	0x57, 0x42, 0x72, 0x57, 0x34, 0x5E, 0x5E, 0x7E, /* H, I, J, K, L, M, N, O */
	0x1F, 0x7E, 0x5F, 0x6D, 0x1C, 0x76, 0x76, 0x76, /* P, Q, R, S, T, U, V, W */
	0x57, 0x67, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, /* X, Y, Z */

	0x00, 0x5F, 0x7F, 0x3C, 0x73, 0x3D, 0x1D, 0x7C, /* A,B,C,D,E,F,G */
	0x57, 0x42, 0x72, 0x57, 0x34, 0x5E, 0x5E, 0x7E, /* H, I, J, K, L, M, N, O */
	0x1F, 0x7E, 0x5F, 0x6D, 0x1C, 0x76, 0x76, 0x76, /* P, Q, R, S, T, U, V, W */
	0x57, 0x67, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, /* X, Y, Z */

	0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 
	0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 
	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 

	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 
	0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 
	0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 

	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 
	0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 
	0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 

	0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 
	0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 
	0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 
	0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f};


const unsigned char	LEDV2_Table[512] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0 - 0x0F

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x10 - 0x1F

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x20 - 0x2F

	46, 177, 18, 48, 98, 131, 98, 178, 
	70, 34, 108, 178, 108, 179, 48, 8, 
	110, 179, 110, 178, 1, 4, 1, 8, 
	16, 64, 0, 0, 128, 8, 110, 50,				   // 0x30 - 0x3F 

	62, 147, 110, 35, 110, 179, 44, 129,
	46, 177, 108, 131, 108, 3, 44,179,
	78, 51, 33, 132, 34, 177, 20, 73,
	12, 129, 47, 53, 142, 113, 46, 177,			  // 0x40 - 0x4F 

	110, 3, 110, 82, 110, 67, 108, 178,
	41,4, 70, 179, 20, 9, 6, 181,
	144, 72, 144, 4, 48, 136, 36, 129, 
	128, 64, 34, 160, 0, 72, 0, 128,			// 0x50 - 0x5F 

	128, 0, 64, 179, 68, 179, 64, 131,
	2, 186, 64, 139, 108, 1, 18, 178,
	68, 35, 0, 4, 2, 112, 17, 68,
	4, 1, 64,55, 64,5, 64, 163, 204, 1,			// 0x60 - 0x6F 

	204, 8, 64, 1, 0, 0, 65, 6, 
	0, 161, 0, 9, 0, 165, 144, 72,
	144, 8, 64, 136, 0, 0, 0, 0,				// 0x70 - 0x7F 

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x80 - 0x8F 

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x90 - 0x9F 

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0xA0 - 0xAF 

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0xB0 - 0xBF 

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0xC0 - 0xCF 

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0xD0 - 0xDF 

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0xE0 - 0xEF 

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0xF0 - 0xFF 


/*
A = 102, 35
B = 110, 79
C = 44, 129
D = 46, 177
E = 108,131
F = 108, 3
G = 44,179
H = 78, 51
I = 33, 132
J = 34, 177
K = 20, 73
L = 12, 129
M = 47, 53
N = 142, 113
O = 46, 177
P = 110, 3
Q = 110, 82
R = 110, 67
S = 108,178
T = 41,4
U = 66, 177
V = 20, 9
W = 6, 165
X = 144, 72
Y = 144, 4
Z = 48, 136

0 = 46,177
1 = 18,48
2 = 98, 131
3 = 98,178
4 = 70, 34
5 = 108, 178
6 = 108, 179
7 = 48,136
8 = 110, 179
9 = 110,178

a = 64, 179
b = 68, 179
c = 64, 131
d = 2, 186
e = 64, 139
f = 108, 129
g = 18, 178
h = 68, 35
i = 0, 04
j = 2, 112
k = 17, 68
l = 4, 1
m = 64,55
n = 64,5
o = 64, 163
p = 204, 1
q = 204, 8
r = 64, 1
s = 
t = 65, 6
u = 0, 161
v = 0, 9
w = 0, 165
x = 144, 72
y = 144, 8
z = 64, 136		
		
" = 3, 0 
] = 34, 160
[ = 36, 129
` = 128, 0
' = 16, 0
% = 24, 24

*/

//makePacket - constructs a packet using current device state
CMAKEPACKETINDEXED(TextLED)
	int brightness = 0;
	char displayString[TEXTLED_MAXROWS][TEXTLED_MAXCOLS];
	size_t length = 0;
	int j, k, correction, result;
	unsigned int ui;

	correction = -1;

	TESTPTRS(phid, buffer)

	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_TEXTLED_1x8:
			if ((phid->phid.deviceVersion >= 111) && (phid->phid.deviceVersion < 114))
			{
				switch(Index)
				{
					case TEXTLED_BRIGHTNESS_PACKET:
						brightness = round((phid->brightness * 31) / 100.0);

						buffer[0] = 0x1C | (0x04 << 5);
						buffer[1] = (unsigned char)(LED_Brightness[brightness] & 0xff);
						buffer[2] = (unsigned char)((LED_Brightness[brightness] >> 8) & 0xff);
						buffer[3] = (unsigned char)((LED_Brightness[brightness] >> 16) & 0xff);
						buffer[4] = (unsigned char)((LED_Brightness[brightness] >> 24) & 0xff);

						break;
					case TEXTLED_DISPLAYSTRING_PACKET + 0:
						length = strlen(phid->displayStringPtr[0]);
						correction = 0;

						for (ui = 0; ui<=length; ui++)
						{
							displayString[0][ui] = LED_Translate[phid->displayStringPtr[0][ui + correction] & 0xff];
							if (((phid->displayStringPtr[0][ui + correction] & 0xff) == 0x2E) && (ui > 0)) 
							{
								displayString[0][ui-1] |= 0x80;
								correction++;
								displayString[0][ui] = LED_Translate[phid->displayStringPtr[0][ui + correction] & 0xff];
							}
						}

						//Takes 2 packets to set string, so we have to send the first one here manually

						//3-bit character count, 5-bit position
						buffer[0] = ((0x07 << 5) | 0x00); //7 characters at posn 0
						memcpy(buffer+1, displayString[0], 7); //the first 7 characters
						//send the first 7 characters
						if ((result = CPhidgetTextLED_sendpacket(phid, buffer)) != EPHIDGET_OK)
						{
							return result;
						}

						//3-bit character count, 5-bit position
						ZEROMEM(buffer, phid->phid.outputReportByteLength);
						buffer[0] = ((0x01 << 5) | 0x07); //1 character at posn 7
						memcpy(buffer+1, displayString[0]+7, 1); //the 8th character

						break;
					default:
						return EPHIDGET_UNEXPECTED;
				}
			}
			else
				return EPHIDGET_UNEXPECTED;
			break;
		case PHIDID_TEXTLED_4x8:
			if ((phid->phid.deviceVersion >= 200) && (phid->phid.deviceVersion < 300))
			{
				switch(Index)
				{
					case TEXTLED_BRIGHTNESS_PACKET:
						brightness = round((phid->brightness * 63) / 100.0);

						buffer[0] = 0x80;
						buffer[1] = brightness;

						break;
					case TEXTLED_DISPLAYSTRING_PACKET + 0:
					case TEXTLED_DISPLAYSTRING_PACKET + 1:
					case TEXTLED_DISPLAYSTRING_PACKET + 2:
					case TEXTLED_DISPLAYSTRING_PACKET + 3:
						length = strlen(phid->displayStringPtr[Index]);
						buffer[0] = (unsigned char)Index;

						for (ui = 0, j = 1, k=8; ui<length; ui++)
						{
							/* Collapse decimal point into previous digit if possible */
							if (phid->displayStringPtr[Index][ui] == '.')
							{
								if(buffer[7] & j)
								{
									k+=2;
									j=j<<1;
								}
								buffer[7] |= j;
								if (k == 8) k+=2;
							} 
							else
							{
								buffer[k] = LEDV2_Table[((unsigned short)phid->displayStringPtr[Index][ui] << 1)];
								buffer[k+1] = LEDV2_Table[(((unsigned short)phid->displayStringPtr[Index][ui] << 1) + 1)];
								if (k != 8) j=j<<1;
								k+=2;
							}
						}
						break;
					default:
						return EPHIDGET_UNEXPECTED;
				}
			}
			else
				return EPHIDGET_UNEXPECTED;
			break;
		default:
			return EPHIDGET_UNEXPECTED;
	}

	return EPHIDGET_OK;
}

// === Exported Functions === //

//create and initialize a device structure
CCREATE(TextLED, PHIDCLASS_TEXTLED)

CGET(TextLED,RowCount,int)
	TESTPTRS(phid,pVal) 
	TESTDEVICETYPE(PHIDCLASS_TEXTLED)
	TESTATTACHED

	MASGN(phid.attr.textled.numRows)
}
CGET(TextLED,ColumnCount,int)
	TESTPTRS(phid,pVal) 
	TESTDEVICETYPE(PHIDCLASS_TEXTLED)
	TESTATTACHED

	MASGN(phid.attr.textled.numColumns)
}

CGET(TextLED,Brightness,int)
	TESTPTRS(phid,pVal) 
	TESTDEVICETYPE(PHIDCLASS_TEXTLED)
	TESTATTACHED
	TESTMASGN(brightness, PUNK_INT)

	MASGN(brightness)
}
CSET(TextLED,Brightness,int)
	TESTPTR(phid) 
	TESTDEVICETYPE(PHIDCLASS_TEXTLED)
	TESTATTACHED
	TESTRANGE(0, 100)	

	if(CPhidget_statusFlagIsSet(phid->phid.status, PHIDGET_REMOTE_FLAG))
		ADDNETWORKKEY(Brightness, "%d", brightness);
	else
		SENDPACKETINDEXED(TextLED, brightness, TEXTLED_BRIGHTNESS_PACKET);

	return EPHIDGET_OK;
}

CSETINDEX(TextLED, DisplayString, char *)
	size_t length = strlen(newVal);
	int i = 0, length_temp = 0;
	TESTPTR(phid)
	TESTDEVICETYPE(PHIDCLASS_TEXTLED)
	TESTATTACHED
	TESTINDEX(phid.attr.textled.numRows)

	//for the 4 row textled, periods don't count in the length 
	// unless the first char is a period, or there are two in a row
	switch(phid->phid.deviceIDSpec)
	{
		case PHIDID_TEXTLED_4x8:
			length_temp = (int)length;
			for(i=1;i<length_temp;i++)
				if(newVal[i] == '.' && newVal[i-1] != '.') length--;
			break;
		default:
			break;
	}
	if (length > (size_t)phid->phid.attr.textled.numColumns) return EPHIDGET_INVALIDARG;

	if(CPhidget_statusFlagIsSet(phid->phid.status, PHIDGET_REMOTE_FLAG))
		ADDNETWORKKEYINDEXED(DisplayString, "%s", strings);
	else
		SENDPACKETINDEXED(TextLED, displayStringPtr[Index], TEXTLED_DISPLAYSTRING_PACKET + Index);

	return EPHIDGET_OK;
}

// === Deprecated Functions === //

CGET(TextLED,NumRows,int)
	return CPhidgetTextLED_getRowCount(phid, pVal);
}
CGET(TextLED,NumColumns,int)
	return CPhidgetTextLED_getColumnCount(phid, pVal);
}
