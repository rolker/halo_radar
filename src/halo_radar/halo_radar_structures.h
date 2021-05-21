#ifndef HALO_RADAR_HALO_RADAR_STRUCTURES_H
#define HALO_RADAR_HALO_RADAR_STRUCTURES_H

// Adapted by Roland Arsenault for ROS

 /******************************************************************************
 *
 * Project:  OpenCPN
 * Purpose:  Radar Plugin
 * Author:   David Register
 *           Dave Cowell
 *           Kees Verruijt
 *           Hakan Svensson
 *           Douwe Fokkema
 *           Sean D'Epagnier
 ***************************************************************************
 *   Copyright (C) 2010 by David S. Register              bdbcat@yahoo.com *
 *   Copyright (C) 2012-2013 by Dave Cowell                                *
 *   Copyright (C) 2012-2016 by Kees Verruijt         canboat@verruijt.net *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************
 */


namespace halo_radar
{

#pragma pack(push, 1)

struct RawScanline
{
  uint8_t headerLen;       // 1 bytes
  uint8_t status;          // 1 bytes
  uint16_t scan_number;    // 2 bytes, 0-4095
  uint16_t u00;            // Always 0x4400 (integer)
  uint16_t large_range;     // 2 bytes or -1
  uint16_t angle;          // 2 bytes
  uint16_t heading;        // 2 bytes heading with RI-10/11 or -1. See bitmask explanation above.
  uint16_t small_range;     // 2 bytes or -1
  uint16_t rotation;       // 2 bytes, rotation/angle
  uint32_t u02;            // 4 bytes signed integer, always -1
  uint32_t u03;            // 4 bytes signed integer, mostly -1 (0x80 in last byte) or 0xa0 in last byte
  uint8_t data[1024 / 2];
};                         

// notes 
// velocity track off: u00: 17408 u02: 2147483648 u03: 2684354560
//             normal: u00: 17408 u02: 2147483648 u03: 2684354560
//      approach only: u00: 17408 u02: 2147483648 u03: 2684354560

struct RawSector {
  uint8_t stuff[5];
  uint8_t scanline_count;
  uint16_t scanline_size;
  RawScanline lines[120];  //  scan lines, or spokes
};


struct IPAddress
{
    uint32_t address;
    uint16_t port;
};

    
    
struct RadarReport_b201
{
  uint16_t id;
  char serialno[16];          // ASCII serial number, zero terminated
  IPAddress addr0;        // 0A 00 43 D9 01 01
  uint8_t u1[12];             // 11000000
  IPAddress addr1;        // EC0608201970
  uint8_t u2[4];              // 11000000
  IPAddress addr2;        // EC0607161A26
  uint8_t u3[10];             // 1F002001020010000000
  IPAddress addr3;        // EC0608211971
  uint8_t u4[4];              // 11000000
  IPAddress addr4;        // EC0608221972
  uint8_t u5[10];             // 10002001030010000000
  IPAddress addrDataA;    // EC0608231973
  uint8_t u6[4];              // 11000000
  IPAddress addrSendA;    // EC0608241974
  uint8_t u7[4];              // 12000000
  IPAddress addrReportA;  // EC0608231975
  uint8_t u8[10];             // 10002002030010000000
  IPAddress addrDataB;    // EC0608251976
  uint8_t u9[4];              // 11000000
  IPAddress addrSendB;    // EC0608261977
  uint8_t u10[4];             // 12000000
  IPAddress addrReportB;  // EC0608251978
  uint8_t u11[10];            // 12002001030010000000
  IPAddress addr11;       // EC0608231979
  uint8_t u12[4];             // 11000000
  IPAddress addr12;       // EC060827197A
  uint8_t u13[4];             // 12000000
  IPAddress addr13;       // EC060823197B
  uint8_t u14[10];            // 12002002030010000000
  IPAddress addr14;       // EC060825197C
  uint8_t u15[4];             // 11000000
  IPAddress addr15;       // EC060828197D
  uint8_t u16[4];             // 12000000
  IPAddress addr16;       // EC060825197E
};

struct RadarReport_c402
{
  uint16_t id;                     // 0
  uint32_t range;                  // 2
  uint8_t skip1;                   // 6
  uint8_t mode;                    // 7
  uint8_t gain_auto;               // 8
  uint8_t skip2[3];                // 9
  uint8_t gain;                    // 12
  uint8_t sea_clutter_auto;        // 13
  uint8_t skip3[3];                // 14
  uint8_t sea_clutter;             // 17
  uint32_t skip4;                  // 18
  uint8_t rain_clutter;            // 22
  uint8_t skip5[11];               // 33
  uint8_t interference_rejection;  // 34
  uint8_t skip6[3];                // 35
  uint8_t target_expansion;        // 38
};

struct RadarReport_c404
{
  uint8_t what;                // 0   0x04
  uint8_t command;             // 1   0xC4
  uint32_t field2;             // 2-5
  uint16_t bearing_alignment;  // 6-7
  uint16_t field8;             // 8-9
  uint16_t antenna_height;     // 10-11
  uint8_t unknown[7];
  uint8_t lights;
};

struct RadarReport_c408
{
  uint8_t what;                          // 0  0x08
  uint8_t command;                       // 1  0xC4
  uint8_t sea_state;                     // 2
  uint8_t local_interference_rejection;  // 3
  uint8_t scan_speed;                    // 4
  uint8_t sls_auto;                      // 5 installation: sidelobe suppression auto
  uint8_t field6;                        // 6
  uint8_t field7;                        // 7
  uint8_t field8;                        // 8
  uint8_t side_lobe_suppression;         // 9 installation: sidelobe suppression
  uint16_t field10;                      // 10-11
  uint8_t noise_rejection;               // 12    noise rejection
  uint8_t target_separation;             // 13
  uint8_t field11;                       // 14
  int8_t  auto_sea_clutter_nudge;        // 15
  uint8_t field13;                       // 16
  uint8_t field14;                       // 17
  uint8_t doppler_state;
  uint16_t doppler_speed;
};

struct RangeCmd
{
    uint16_t cmd;
    uint32_t range;
    RangeCmd():cmd(0xc103){}
};

struct BearingAlignmentCmd
{
    uint16_t cmd;
    uint16_t bearing_alignment;
    BearingAlignmentCmd():cmd(0xc105){}
};

struct GainCmd
{
    uint16_t cmd;
    uint32_t sub_cmd;
    uint32_t gain_auto;
    uint8_t  gain;
    GainCmd():cmd(0xc106),sub_cmd(0),gain_auto(0),gain(0){}
};

struct SeaClutterCmd
{
    uint16_t cmd;
    uint32_t sub_cmd;
    uint32_t sea_clutter_auto;
    uint8_t  sea_clutter;
    SeaClutterCmd():cmd(0xc106),sub_cmd(0x02),sea_clutter_auto(0),sea_clutter(0){}
};

struct RainClutterCmd
{
    uint16_t cmd;
    uint32_t sub_cmd;
    uint32_t blank;
    uint8_t  rain_clutter;
    RainClutterCmd():cmd(0xc106),sub_cmd(0x04),blank(0),rain_clutter(0){}
};

struct SidelobeSuppressionCmd
{
    uint16_t cmd;
    uint32_t sub_cmd;
    uint32_t sls_auto;
    uint8_t  sidelobe_suppression;
    SidelobeSuppressionCmd():cmd(0xc106),sub_cmd(0x05),sls_auto(0),sidelobe_suppression(0){}
};

struct EnumCmd
{
    uint16_t cmd;
    uint8_t value;
    EnumCmd(uint16_t c, uint8_t v):cmd(c),value(v){}
};

struct AutoSeaClutterNudgeCmd
{
    uint16_t cmd;
    uint8_t sub_cmd;
    int8_t nudge1;
    int8_t nudge2;
    uint8_t tail;
    AutoSeaClutterNudgeCmd(int8_t nudge):cmd(0xc111),sub_cmd(0x01),nudge1(nudge),nudge2(nudge),tail(0x04){}
};

struct DopplerSpeedCmd
{
    uint16_t cmd;
    uint16_t speed;
    DopplerSpeedCmd(uint16_t s):cmd(0xc124),speed(s){}
};

struct AntennaHeightCmd
{
    uint16_t cmd;
    uint32_t one;
    uint32_t height_mm;
    AntennaHeightCmd(uint32_t h):cmd(0xc130),one(1),height_mm(h){}
};

struct HaloHeadingPacket
{
  char marker[4];    // 4 bytes containing 'NKOE'
  uint8_t u00[4];    // 4 bytes containing '00 01 90 02'
  uint16_t counter;  // 2 byte counter incrementing by 1 every transmission, in BigEndian
  // 10
  uint8_t u01[26];  // 25 bytes of unknown stuff that doesn't seem to vary
  // 36
  uint8_t u02[2];  // 2 bytes containing '12 f1'
  uint8_t u03[2];  // 2 bytes containing '01 00'
  // 40
  uint64_t epoch;  // 8 bytes containing millis since 1970
  // 48
  uint64_t u04;  // 8 bytes containing 2
  // 56
  uint32_t u05a;  // 4 bytes containing some fixed data, could be position?
  // 60
  uint32_t u05b;  // 4 bytes containing some fixed data, could be position?
  // 64
  uint8_t u06[1];  // 1 byte containing counter or 0xff
  // 65
  uint16_t heading;  // 2 bytes containing heading
  // 67
  uint8_t u07[5];  // 5 bytes containing varying unknown data
  // 72
};

struct HaloMysteryPacket
{
  char marker[4];    // 4 bytes containing 'NKOE'
  uint8_t u00[4];    // 4 bytes containing '00 01 90 02'
  uint16_t counter;  // 2 byte counter incrementing by 1 every transmission, in BigEndian
  // 10
  uint8_t u01[26];  // 25 bytes of unknown stuff that doesn't seem to vary
  // 36
  uint8_t u02[2];  // 2 bytes containing '02 f8'...
  uint8_t u03[2];  // 2 bytes containing '01 00'
  // 40
  uint64_t epoch;  // 8 bytes containing millis since 1970
  // 48
  uint64_t u04;  // 8 bytes containing 2
  // 56
  uint32_t u05a;  // 4 bytes containing some fixed data, could be position?
  // 60
  uint32_t u05b;  // 4 bytes containing some fixed data, could be position?
  // 64
  uint8_t u06[1];  // 1 byte containing counter or 0xff
  // 65
  uint8_t u07[1];  // 1 byte containing 0xfc
  // 66
  uint16_t mystery1;  // 2 bytes containing some varying field
  // 68
  uint16_t mystery2;  // 2 bytes containing some varying field
  // 70
  uint8_t u08[2];  // 2 bytes containg 0xff 0xff
  // 72
};


#pragma pack(pop)
    
} // namespace

#endif

