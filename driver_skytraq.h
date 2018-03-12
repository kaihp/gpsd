/*
 * This file is Copyright (c) 2018 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 */
#ifndef _GPSD_SKY_H_
#define _GPSD_SKY_H_

/*
 * Binary commands accepted by the Skytraq Venus638 & Venus838 Modules
 * References:
 * Venus6: https://cdn.sparkfun.com/datasheets/Sensors/GPS/AN0003_v1.4.19.pdf
 * Venus8: https://navspark.mybigcommerce.com/content/AN0028_1.4.42.pdf
 * S1216F: http://www.skytraq.com.tw/datasheet/S1216V8_v0.9.pdf
 * 
 * Message Format:
 * <Header><PL><Payload><CS><End of Sequence>
 * Header: <0xA0, 0xA1>
 * PL: Payload length in bytes (len = (PL[0] << 8) + PL[1])
 * CS: Checksum is XOR of all payload bytes
 * End of Sequence: "\n\r" <0x0D, 0x0A>
 *
 * Minimum packet length is 7 + payload.
 *
 * The below data is as per AN0003_v1.4.19 & AN0028_1.4.42
 * Legend:
 * 6/8 : Accepted/used by Venus638/Venus838 firmware (? = unknown support)
 * Id  : MessageID in first payload byte
 * SID : Sub-Message ID (optional) in second payload byte
 * Len : Payload length (not include header, checksum and End of Sequence)
 * Dir : Direction (input/output). Outputs have bit 7 set on Id/SID.
 * Name: Name of request/response
 * | 6 | 8 | Id   | SID  | Dir | Len | Name                               |
 * |---|---|------|------|-----|-----|------------------------------------|
 * |--- Input System Messages --------------------------------------------|
 * | 6 | 8 | 0x01 |      | In  |  15 | System restart 
 * | 6 | 8 | 0x02 |      | In  |   2 | Query software version 
 * | 6 | 8 | 0x03 |      | In  |   2 | Query software CRC
 * | 6 | 8 | 0x04 |      | In  |   2 | Reset to factory defaults
 * | 6 | 8 | 0x05 |      | In  |   4 | Configure serial port
 * |   |   | 0x06 |      | In  |     | Reserved
 * |   |   | 0x07 |      | In  |     | Reserved
 * | 6 | 8 | 0x08 |      | In  |   9 | Configure NMEA
 * | 6 | 8 | 0x09 |      | In  | 2/3 | Configure message type (V6: 2B)
 * |   | 8 | 0x0B |      | In  |   6 | Software image download
 * | 6 | 8 | 0x0C |      | In  |   3 | Configure power mode
 * | 6 | 8 | 0x0E |      | In  |   3 | Configure position update rate
 * | 6 | 8 | 0x10 |      | In  |   1 | Query position update rate
 * | ? | 8 | 0x11 |      | In  |   3 | Configure navigation data message interval
 * |   | 8 | 0x15 |      | In  |   1 | Query power mode
 * |--- Input GNSS Messages ----------------------------------------------|
 * | 6 | 8 | 0x29 |      | In  |  19 | Configure datum
 * | ? | 8 | 0x2A |      | In  |   9 | Configure DOP mask
 * | ? | 8 | 0x2B |      | In  |   5 | Configure elevation and CNR mask
 * | 6 | 8 | 0x2D |      | In  |   1 | Query datum
 * | ? | 8 | 0x2E |      | In  |   1 | Query DOP mask
 * | ? | 8 | 0x2F |      | In  |   1 | Query elevation and CNR mask
 * | 6 | 8 | 0x30 |      | In  |   2 | Get GPS ephemeris
 * | 6 |   | 0x31 |      | In  |  87 | Set GPS ephemeris (see 0x41 for Venus838 cmd)
 * | 6 |   | 0x37 |      | In  |   3 | Configure WAAS
 * | 6 |   | 0x38 |      | In  |   1 | Query WAAS status
 * | 6 | 8 | 0x39 |      | In  | 2/3 | Configure position pinning (V6: 2B)
 * | 6 | 8 | 0x3A |      | In  |   1 | Query position pinning
 * | 6 | 8 | 0x3B |      | In  |  11 | Configure position pinning parameters
 * | 6 |   | 0x3C |      | In  |   3 | Configuration navigation mode
 * | 6 |   | 0x3D |      | In  |   1 | Query navigation mode
 * | 6 |   | 0x3E |      | In  |   3 | Configure 1PPS mode
 * | 6 |   | 0x3F |      | In  |   1 | Query 1PPS mode
 * |   | 8 | 0x41 |      | In  |  87 | Set GPS ephemeris (see 0x31 for Venus638 cmd)
 * |   | 8 | 0x44 |      | In  |   2 | Query 1PPS timing
 * |   | 8 | 0x45 |      | In  |   6 | Configure 1PPS cable delay
 * |   | 8 | 0x46 |      | In  |   1 | Query 1PPS cable delay
 * |   | 8 | 0x4B |      | In  |   3 | Configure NMEA TalkerID
 * |   | 8 | 0x4F |      | In  |   1 | Query NMEA TalkerID
 * |   | 8 | 0x50 |      | In  |   2 | Get GPS Almanac
 * |   | 8 | 0x51 |      | In  |  52 | Set GPS Almanac
 * |   | 8 | 0x54 |      | In  |  31 | Configure 1PPS timing
 * |   | 8 | 0x5B |      | In  |   2 | Get GLONASS Ephemeris
 * |   | 8 | 0x5C |      | In  |  43 | Set GLONASS Ephemeris
 * |   | 8 | 0x5D |      | In  |   2 | Get GLONASS Almanac
 * |   | 8 | 0x5E |      | In  |  26 | Set GLONASS Almanac
 * |   | 8 | 0x5F |      | In  |   2 | Get GLONASS Time Correction Parameters
 * |   | 8 | 0x60 |      | In  |  10 | Set GLONASS Time Correction Parameters
 * |--- Messages with Sub-IDs -------------------------------------------|
 * |   | 8 | 0x62 | 0x01 | In  |   9 | Configure SBAS
 * |   | 8 | 0x62 | 0x02 | In  |   2 | Query SBAS status
 * |   | 8 | 0x62 | 0x03 | In  |   5 | Configure QZSS
 * |   | 8 | 0x62 | 0x04 | In  |   2 | Query QZSS status
 * |   | 8 | 0x62 | 0x80 | Out |   8 | SBAS status
 * |   | 8 | 0x62 | 0x81 | Out |   4 | QZSS status
 * |   | 8 | 0x63 | 0x01 | In  |   4 | Configure SAEE
 * |   | 8 | 0x63 | 0x02 | In  |   2 | Query SAEE
 * |   | 8 | 0x63 | 0x80 | Out |   3 | SAEE status
 * |   | 8 | 0x64 | 0x01 | In  |   2 | Query boot status
 * |   | 8 | 0x64 | 0x02 | In  |  15 | Configure extended NMEA message interval
 * |   | 8 | 0x64 | 0x03 | In  |   2 | Query extended NMEA message interval
 * |   | 8 | 0x64 | 0x06 | In  |   4 | Configure interference detection
 * |   | 8 | 0x64 | 0x07 | In  |   2 | Query interference detection status
 * |   | 8 | 0x64 | 0x0A | In  |   4 | Configure GPS parameter search engine number
 * |   | 8 | 0x64 | 0x0B | In  |   2 | Query GPS parameter search engine number
 * |   | 8 | 0x64 | 0x11 | In  |   5 | Configure Position Fix Navigation Mask
 * |   | 8 | 0x64 | 0x12 | In  |   2 | Query Position Fix Navigation Mask
 * |   | 8 | 0x64 | 0x15 | In  |   8 | Configure UTC Reference Time Sync to GPS Time
 * |   | 8 | 0x64 | 0x16 | In  |   2 | Query UTC Reference Time Sync to GPS Time
 * |   | 8 | 0x64 | 0x17 | In  |   4 | Configure GNSS navigation mode
 * |   | 8 | 0x64 | 0x18 | In  |   2 | Query GNSS navigation mode
 * |   | 8 | 0x64 | 0x19 | In  |   5 | Configure GNSS constellation type for navigation solution
 * |   | 8 | 0x64 | 0x1A | In  |   2 | Query GNSS constellation type for navigation solution
 * |   | 8 | 0x64 | 0x1F | In  |   4 | Configure GPS/UTC leap seconds
 * |   | 8 | 0x64 | 0x20 | In  |   2 | Query GPS time
 * |   | 8 | 0x64 | 0x21 | In  |   5 | Configure PSTI Message Interval
 * |   | 8 | 0x64 | 0x22 | In  |   3 | Query PTSI Message Interval
 * |   | 8 | 0x64 | 0x27 | In  |   5 | Configure GNSS datum index
 * |   | 8 | 0x64 | 0x28 | In  |   2 | Query GNSS datum index
 * |   | 8 | 0x64 | 0x2F | In  | var | Configure GNSS Geo-Fencing Data PL=4+n*16 (n=1..10)
 * |   | 8 | 0x64 | 0x30 | In  |   2 | Query GNSS Geo-Fencing Data
 * |   | 8 | 0x64 | 0x31 | In  |   2 | Query GNSS Geo-Fencing Result
 * |   | 8 | 0x64 | 0x7D | In  |   2 | Query Version Extension String
 * |   | 8 | 0x64 | 0x80 | Out |   4 | GNSS boot status
 * |   | 8 | 0x64 | 0x81 | Out |  14 | Extended NMEA message interval
 * |   | 8 | 0x64 | 0x83 | Out |   4 | Interference detection status
 * |   | 8 | 0x64 | 0x85 | Out |   3 | GPS parameter search engine number
 * |   | 8 | 0x64 | 0x88 | Out |   5 | Position Fix Navigation
 * |   | 8 | 0x64 | 0x8A | Out |   7 | GPS UTC Reference Time
 * |   | 8 | 0x64 | 0x8B | Out |   3 | GNSS navigation mode
 * |   | 8 | 0x64 | 0x8C | Out |   4 | GNSS constellation type for navigation solution
 * |   | 8 | 0x64 | 0x8E | Out |  15 | GPS time
 * |   | 8 | 0x64 | 0x8F | Out |   3 | PSTI Message Interval
 * |   | 8 | 0x64 | 0x92 | Out |   5 | GNSS datum index
 * |   | 8 | 0x64 | 0x96 | Out | var | GNSS Geo-Fencing Data PL=3+n*16 (n=1..10)
 * |   | 8 | 0x64 | 0x97 | Out |  19 | GNSS Geo-Fencing Result
 * |   | 8 | 0x64 | 0xFE | Out |  34 | Version Extension String
 * |   | 8 | 0x65 | 0x01 | In  |   7 | Configure 1PPS pulse width
 * |   | 8 | 0x65 | 0x02 | In  |   3 | Query 1PPS pulse width
 * |   | 8 | 0x65 | 0x03 | In  |   7 | Configure 1PPS frequency output
 * |   | 8 | 0x65 | 0x04 | In  |   2 | Query 1PPS frequency output
 * |   | 8 | 0x65 | 0x80 | Out |   6 | 1PPS pulse width
 * |   | 8 | 0x65 | 0x81 | Out |   6 | 1PPS frequency output
 * |   | 8 | 0x67 | 0x01 | In  | var | Set Beidou Ephemeris Data PL=126/87
 * |   | 8 | 0x67 | 0x02 | In  |   3 | Get Beidou Ephemeris Data
 * |   | 8 | 0x67 | 0x03 | In  |  53 | Set Beidou Almanac Data
 * |   | 8 | 0x67 | 0x04 | In  |   3 | Get Beidou Almanac Data
 * |   | 8 | 0x67 | 0x80 | Out | var | Beidou Ephemeris Data PL=126/87
 * |   | 8 | 0x67 | 0x81 | Out |  53 | Beidou Almanac Data
 * |   | 8 | 0x6A | 0x01 | In  |   4 | Configure RTK Mode
 * |   | 8 | 0x6A | 0x02 | In  |   3 | Query RTK Mode
 * |   | 8 | 0x6A | 0x80 | Out |   3 | RTK Mode
 * |--- Output System Messages -------------------------------------------|
 * | 6 | 8 | 0x80 |      | Out |  14 | Software version
 * | 6 | 8 | 0x81 |      | Out |   4 | Software CRC
 * |   |   | 0x82 |      | Out |     | Reserved
 * | 6 | 8 | 0x83 |      | Out |   2 | ACK
 * | 6 | 8 | 0x84 |      | Out |   2 | NACK
 * |   | 8 | 0x85 |      | Out |   5 | <undocumented message from FW v2.2.4>
 * | 6 | 8 | 0x86 |      | Out |   2 | Position update rate
 * |   | 8 | 0x90 |      | Out |  43 | GLONASS Ephemeris Data
 * |   | 8 | 0x91 |      | Out |  25 | GLONASS Almanac Data
 * |   | 8 | 0x92 |      | Out |   9 | Beidou Time Correction Parameters
 * | ? | 8 | 0x93 |      | Out |   2 | GNSS NMEA TalkerID
 * |--- Output GNSS Messages ---------------------------------------------|
 * | ? | 8 | 0xA8 |      | Out |  59 | Navigation data message
 * | 6 | 8 | 0xAE |      | Out |   3 | GNSS datum
 * |   | 8 | 0xAF |      | Out |   8 | GNSS DOP mask
 * |   | 8 | 0xB0 |      | Out |   4 | Elevation and CNR mask
 * | 6 | 8 | 0xB1 |      | Out |  87 | GPS Ephemeris Data
 * | 6 |   | 0xB3 |      | Out |   2 | GNSS WAAS status
 * | 6 | 8 | 0xB4 |      | Out |2/12 | GNSS position pinning status V6:2B/V8:12B
 * | 6 |   | 0xB5 |      | Out |   2 | GPS navigation mode
 * | 6 |   | 0xB6 |      | Out |   2 | GPS 1PPS mode
 * |   | 8 | 0xB9 |      | Out |   2 | GNSS power mode
 * |   | 8 | 0xBB |      | Out |   5 | GNSS 1PPS cable delay
 * |   | 8 | 0xBE |      | Out |  52 | GPS Almanac Data
 * |   | 8 | 0xC2 |      | Out |  35 | GNSS 1PPS timing (only flash-based receivers)
 */

typedef enum {
	/* Input System Messages */
	SKY_RESTART              = 0x01,
	SKY_QUERY_SW_VER         = 0x02,
	SKY_QUERY_SW_CRC         = 0x03,
	SKY_FACTORY_RESET        = 0x04,
	SKY_CONFIG_SERIAL        = 0x05,
	SKY_CONFIG_NMEA          = 0x08,
	SKY_CONFIG_MSG_TYPE      = 0x09,
	SKY_DOWNLOAD_SW_IMG      = 0x0B,
	SKY_CONFIG_POWER_MODE    = 0x0C,
	SKY_CONFIG_POS_UPD_RATE  = 0x0E,
	SKY_QUERY_POS_UPD_RATE   = 0x10,
	SKY_CONFIG_NAV_MSG_INTVL = 0x11,
	SKY_QUERY_POWER_MODE     = 0x15,
	/* Input GNSS Messages */
	SKY_CONFIG_DATUM         = 0x29,
	SKY_CONFIG_DOP           = 0x2A,
	SKY_CONFIG_ELEV_CNR      = 0x2B,
	SKY_QUERY_DATUM          = 0x2D,
	SKY_QUERY_DOP            = 0x2E,
	SKY_QUERY_ELEV_CNR       = 0x2F,
	SKY_GET_EPHEMERIS        = 0x30,
	SKY_SET_EPHEMERIS_V6     = 0x31,
	SKY_CONFIG_WAAS          = 0x37,
	SKY_QUERY_WAAS           = 0x38,
	SKY_CONFIG_POSITION_PIN  = 0x39,
	SKY_QUERY_POSITION_PIN   = 0x3A,
	SKY_CONFIG_POS_PIN_PARAM = 0x3B,
	SKY_CONFIG_NAV_MODE      = 0x3C,
	SKY_QUERY_NAV_MODE       = 0x3D,
	SKY_CONFIG_PPS_MODE      = 0x3E,
	SKY_QUERY_PPS_MODE       = 0x3F,
	SKY_SET_EPHEMERIS_V8     = 0x41,
	SKY_QUERY_PPS_TIMING     = 0x44,
	SKY_CONFIG_PPS_CABLE_DEL = 0x45,
	SKY_QUERY_PPS_CABLE_DEL  = 0x46,
	SKY_CONFIG_NMEA_TALKER   = 0x4B,
	SKY_QUERY_NMEA_TALKER    = 0x4F,
	SKY_CONFIG_PPS_TIMING    = 0x54,
	/* Messages with Sub-IDs */
	SKY_MSGID_62             = 0x62,
	SKY_MSGID_63             = 0x63,
	SKY_MSGID_64             = 0x64,
	SKY_MSGID_65             = 0x65,
	SKY_MSGID_67             = 0x67,
	SKY_MSGID_6A             = 0x6A,
	/* Output System Messages */
	SKY_RESP_SW_VER          = 0x80,
	SKY_RESP_SW_CRC          = 0x81,
	SKY_RESP_ACK             = 0x83,
	SKY_RESP_NACK            = 0x84,
	SKY_RESP_85              = 0x85,
	SKY_RESP_POS_UPD_RATE    = 0x86,
	SKY_RESP_NMEA_TALKER     = 0x93,
	/* Output GNSS Messages */
	SKY_RESP_NAV_DATA_MSG    = 0xA8,
	SKY_RESP_DATUM           = 0xAE,
	SKY_RESP_DOP             = 0xAF,
	SKY_RESP_ELEV_CNR        = 0xB0,
	SKY_RESP_WAAS            = 0xB3,
	SKY_RESP_POS_PIN_STATUS  = 0xB4,
	SKY_RESP_NAV_MODE        = 0xB5,
	SKY_RESP_PPS_MODE        = 0xB6,
	SKY_RESP_POWER_MODE      = 0xB9,
	SKY_RESP_PPS_CABLE_DEL   = 0xBB,
	SKY_RESP_PPS_TIMING      = 0xC2,
	/* The following messages come from the original Skytraq driver
		 I haven't found them in the Skytraq documentation */
	SKY_MSG_MEAS_TIME        = 0xDC,
	SKY_MSG_MEAS_RAW         = 0xDD,
	SKY_MSG_SV_CHAN_STATUS   = 0xDE,
	SKY_MSG_NAV_PVT          = 0xDF,
	SKY_MSG_SUBFRAME_DATA    = 0xE0,
	SKY_MSG_BD_SUBFRAME_D1   = 0xE2,
	SKY_MSG_BD_SUBFRAME_D2   = 0xE3,
} sky_msg_id_t;

typedef enum {
	SKY_SID_NONE                 = 0x00,
	SKY_62_CONFIG_SBAS           = 0x01,
	SKY_62_QUERY_SBAS            = 0x02,
	SKY_62_CONFIG_QZSS           = 0x03,
	SKY_62_QUERY_QZSS            = 0x04,
	SKY_62_RESP_SBAS             = 0x80,
	SKY_62_RESP_QZSS             = 0x81,
	SKY_63_CONFIG_SAEE           = 0x01,
	SKY_63_QUERY_SAEE            = 0x02,
	SKY_63_RESP_SAEE             = 0x80,
	SKY_64_QUERY_BOOT_STATUS     = 0x01,
	SKY_64_CONFIG_EXT_NMEA_INTVL = 0x02,
	SKY_64_QUERY_EXT_NMEA_INTVL  = 0x03,
	SKY_64_CONFIG_INTERFER_DET   = 0x06,
	SKY_64_QUERY_INTERFER_DET    = 0x07,
	SKY_64_CONFIG_PARAM_ENG_NUM  = 0x0A,
	SKY_64_QUERY_PARAM_ENG_NUM   = 0x0B,
	SKY_64_CONFIG_GPS_UTC_TIME   = 0x15,
	SKY_64_QUERY_GPS_UTC_TIME    = 0x16,
	SKY_64_CONFIG_NAV_MODE       = 0x17,
	SKY_64_QUERY_NAV_MODE        = 0x18,
	SKY_64_CONFIG_CONSTEL_TYPE   = 0x19,
	SKY_64_QUERY_CONSTEL_TYPE    = 0x1A,
	SKY_64_CONFIG_LEAP_SEC       = 0x1F,
	SKY_64_QUERY_GNSS_TIME       = 0x20,
	SKY_64_CONFIG_PSTI_INTVL     = 0x21,
	SKY_64_QUERY_PSTI_INTVL      = 0x22,
	SKY_64_CONFIG_DATUM_INDEX    = 0x27,
	SKY_64_QUERY_DATUM_INDEX     = 0x28,
	SKY_64_QUERY_VERSION_EXT     = 0x7D,
	SKY_64_RESP_BOOT_STATUS      = 0x80,
	SKY_64_RESP_EXT_NMEA_INTVL   = 0x81,
	SKY_64_RESP_INTERFER_DET     = 0x83,
	SKY_64_RESP_PARAM_ENG_NUM    = 0x85,
	SKY_64_RESP_GPS_UTC_REF_TIME = 0x8A,
	SKY_64_RESP_NAV_TYPE         = 0x8B,
	SKY_64_RESP_CONSTEL_TYPE     = 0x8C,
	SKY_64_RESP_GNSS_TIME        = 0x8E,
	SKY_64_RESP_PSTI_INTVL       = 0x8F,
	SKY_64_RESP_DATUM_INDEX      = 0x92,
	SKY_64_RESP_GEOFENCE_DATA    = 0x96,
	SKY_64_RESP_GEOFENCE_RESULT  = 0x97,
	SKY_64_RESP_VERSION_EXT      = 0xFE,
	SKY_65_CONFIG_PPS_WIDTH      = 0x01,
	SKY_65_QUERY_PPS_WIDTH       = 0x02,
	SKY_65_CONFIG_PPS_FREQ       = 0x03,
	SKY_65_QUERY_PPS_FREQ        = 0x04,
	SKY_65_RESP_PPS_WIDTH        = 0x80,
	SKY_65_RESP_PPS_FREQ         = 0x81,
	SKY_67_SET_BD_EPHEMERIS      = 0x01,
	SKY_67_GET_BD_EPHEMERIS      = 0x02,
	SKY_67_SET_BD_ALMANAC        = 0x03,
	SKY_67_GET_BD_ALMANAC        = 0x04,
	SKY_67_RESP_BD_EPHEMERIS     = 0x80,
	SKY_67_RESP_BD_ALMANAC       = 0x81,
	SKY_6A_CONFIG_RTK_MODE       = 0x01,
	SKY_6A_QUERY_RTK_MODE        = 0x02,
	SKY_6A_RESP_RTK_MODE         = 0x80
} sky_sid_t;

typedef enum {
	VENUS6 = 1,
	VENUS8 = 2,
	V_ALL  = 0xFF,
} sky_supp_t;

typedef struct {
	sky_supp_t chip;
	sky_msg_id_t msg_id;
	sky_sid_t sub_id;
} sky_cmd_supp_t;

#define SKY_START_1 0xA0
#define SKY_START_2 0xA1
#define SKY_END_1 0x0D
#define SKY_END_2 0x0A

#define SKY_MODE_NONE 0
#define SKY_MODE_2D   1
#define SKY_MODE_3D   2
#define SKY_MODE_DGPS 3

#endif /* _GPSD_SKY_H_ */
