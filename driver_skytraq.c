/*
 * This is the gpsd driver for Skytraq GPSes operating in binary mode.
 *
 * This file is Copyright (c) 2016, 2018 by the GPSD project
 * BSD terms apply: see the file COPYING in the distribution root for details.
 *
 * Handles both Skytraq 638 & 838 chipsets
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include <ctype.h>
#include <unistd.h>

#include "gpsd.h"
#include "bits.h"
#include "strfuncs.h"
#include "driver_skytraq.h"
#if defined(SKYTRAQ_ENABLE)

#define HI(n)		((n) >> 8)
#define LO(n)		((n) & 0xff)

/*
 * No ACK/NAK?  Just rety after 6 seconds
 */
#define SKY_RETRY_TIME	6
#define SKY_CHANNELS	48	/* max channels allowed in format */

static gps_mask_t sky_parse(struct gps_device_t *, unsigned char *, size_t);

static gps_mask_t sky_msg_sw_ver(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_sw_crc(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_nav_data_msg(struct gps_device_t *, unsigned char *, size_t);
#if 0 /* NOT IMPLEMENTED YET */
static gps_mask_t sky_msg_pos_upd_rate(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_nmea_talkerid(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_datum(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_dop(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_elev_cnr(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_waas(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_pos_pin_status(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_nav_mode(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_pps_mode(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_power_mode(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_pps_cable_mode(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_pps_timing(struct gps_device_t *, unsigned char *, size_t);
#endif
static gps_mask_t sky_msg_DC(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_DD(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_DE(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_DF(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_E0(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_E2(struct gps_device_t *, unsigned char *, size_t);
static gps_mask_t sky_msg_E3(struct gps_device_t *, unsigned char *, size_t);

static bool sky_write(struct gps_device_t *session, unsigned char *msg);
static bool sky_speed(struct gps_device_t *session, speed_t speed, char parity, int stopbits);
static void sky_mode(struct gps_device_t *session, int mode);
static bool sky_rate(struct gps_device_t *session, double cycletime);

#ifdef CONTROLSEND_ENABLE
/* not used by gpsd, it's for gpsctl and friends */
static ssize_t sky_control_send(struct gps_device_t *session, char *msg, size_t len UNUSED)
{
  return sky_write(session, (unsigned char *)msg);
}
#endif /* CONTROLSEND_ENABLE */

/*
 * Function writes the Skytraq start header (0xA0 0xA1) and trailer (0x0D 0x0A)
 * and computes the correct Checksum (the byte before the trailer).
 */
static bool sky_write(struct gps_device_t *session, unsigned char *msg)
{
    unsigned int csum;
    size_t i, len;
    bool ok;
    unsigned int mid = (unsigned int)msg[4];
    unsigned int sid = (unsigned int)msg[5];
    len = (size_t) ((msg[2] << 8) | msg[3]);
    msg[0] = SKY_START_1;
    msg[1] = SKY_START_2;
    msg[len+5] = SKY_END_1;
    msg[len+6] = SKY_END_2;
    /* calculate Checksum */
    csum = 0;
    /* coverity_submit[tainted_data] */
    for (i = 0; i < len; i++)
	csum ^= (int)msg[4 + i];

    /* enter CSUM after payload */
    msg[len + 4] = (unsigned char)(csum & 0x00ff);

    if ((mid==SKY_MSGID_62) || (mid==SKY_MSGID_63) ||
	(mid==SKY_MSGID_64) || (mid==SKY_MSGID_65))
	gpsd_log(&session->context->errout, LOG_PROG,
		 "=> GPS: Skytraq writing control MsgID/SubID %02x/%02x:\n", mid, sid);
    else
	gpsd_log(&session->context->errout, LOG_PROG,
		 "=> GPS: Skytraq writing control MsgID %02x:\n", mid);
    ok = (gpsd_write(session, (const char *)msg, len+7) == (ssize_t) (len+7));

    return (ok);
}

static void sky_init_query(struct gps_device_t *session)
{
    unsigned char msg[] = {
	SKY_START_1, SKY_START_2,
	0x00, 0x02,
	SKY_QUERY_SW_VER, 0x01,
	0x00, SKY_END_1, SKY_END_2
    };
    /* Query for version information */
    (void)sky_write(session,msg);
}

static void sky_event_hook(struct gps_device_t *session, event_t event)
{
    unsigned char msg[80];
    msg[0] = SKY_START_1;
    msg[1] = SKY_START_2;
    msg[2] = 0x00;
    if (session->context->readonly)
	return;
    else if (event == event_identified) {
	gpsd_log(&session->context->errout, LOG_DATA,
		 "Skytraq configuration\n");
#if 0
	/* Query boot status */
	msg[3] = 0x02;
	msg[4] = SKY_MSGID_64;
	msg[5] = SKY_64_QUERY_BOOT_STATUS;
	(void)sky_write(session, msg);
	/* Enable power-saving mode */
	msg[3] = 0x03;
	msg[4] = SKY_CONFIG_POWER_MODE;
	msg[5] = 0x01;
	msg[6] = 0x00;
	(void)sky_write(session, msg);
	/* System position rate: 1Hz */
	msg[3] = 0x03;
	msg[4] = SKY_CONFIG_POS_UPD_RATE;
	msg[5] = 0x01;
	msg[6] = 0x00;
	(void)sky_write(session, msg);
	/* Binary Navigation Data message interval: 1Hz */
	msg[3] = 0x03;
	msg[4] = SKY_CONFIG_NAV_MSG_INTVL;
	msg[5] = 0x01;
	msg[6] = 0x00;
	(void)sky_write(session, msg);
	/* Enable SBAS mode (not sure what it does, but UBX driver enables it) */
	msg[ 3] = 0x09;
	msg[ 4] = SKY_MSGID_62;
	msg[ 5] = SKY_62_CONFIG_SBAS;
	msg[ 6] = 0x01;
	msg[ 7] = 0x02;
	msg[ 8] = 0x08;
	msg[ 9] = 0x01;
	msg[10] = 0x03;
	msg[11] = 0x07;
	msg[12] = 0x00;
	(void)sky_write(session, msg);
	/* Enable QZSS, 1 ch */
	msg[ 3] = 0x05;
	msg[ 4] = SKY_MSGID_62;
	msg[ 5] = SKY_62_CONFIG_QZSS;
	msg[ 6] = 0x01;
	msg[ 7] = 0x01;
	msg[ 8] = 0x00;
	(void)sky_write(session, msg);
	/* Enable SAEE */
	msg[ 3] = 0x04;
	msg[ 4] = SKY_MSGID_63;
	msg[ 5] = SKY_63_CONFIG_SAEE;
	msg[ 6] = 0x01;
	msg[ 7] = 0x00;
	(void)sky_write(session, msg);
	/* Enable interference detection */
	msg[ 3] = 0x04;
	msg[ 4] = SKY_MSGID_64;
	msg[ 5] = SKY_64_CONFIG_INTERFER_DET;
	msg[ 6] = 0x01;
	msg[ 7] = 0x00;
	(void)sky_write(session, msg);
	/* GNSS Navigation mode = automatic */
	msg[ 3] = 0x04;
	msg[ 4] = SKY_MSGID_64;
	msg[ 5] = SKY_64_CONFIG_NAV_MODE;
	msg[ 6] = 0x00;
	msg[ 7] = 0x00;
	(void)sky_write(session, msg);
#endif

#ifdef RECONFIGURE_ENABLE
	/*
	 * Turn off NMEA output, turn on Skytraq on this port.
	 */
	if (session->mode == O_OPTIMIZE) {
	    sky_mode(session, MODE_BINARY);
	} else {
	    sky_mode(session, MODE_NMEA);
	}
#endif /* RECONFIGURE_ENABLE */
    } else if (event == event_deactivate) {
	unsigned char hh, mm, ss;
	unsigned int year;
	unsigned char month, day;
	gpsd_log(&session->context->errout, LOG_DATA, "Skytraq revert\n");
	/* Do a hot restart. Faking 2018/01/01 00:00:00 at 0.0N 0.0E alt=0m */
	msg[ 3] = 0x0F;
	msg[ 4] = SKY_RESTART;
	msg[ 5] = 0x01; /* Hot restart */
	year = 2018; month = 3; day = 5;
	msg[ 6] = 0x00FF & (year >> 8);
	msg[ 7] = 0x00FF & (year >> 0);
	msg[ 8] = 0x00FF & month;
	msg[ 9] = 0x00FF & day;
	ss = 0; mm = 0; hh = 0;
	msg[10] = hh; msg[11] = mm; msg[12] = ss;
	if (session->gpsdata->fix.mode >= MODE_2D) {
	    int lat, lon;
	    lat = (int) session->gpsdata->fix.latitude;
	    lon = (int) session->gpsdata->fix.longitude;
	    msg[ 7] = 0x00ff & (lat >> 8);
	    msg[ 8] = 0x00ff & (lat >> 0);
	    msg[ 9] = 0x00ff & (lon >> 8);
	    msg[10] = 0x00ff & (lon >> 0);
	}
	if (session->gpsdata->fix.mode == MODE_3D) {
	    unsigned int alt = (unsigned int) session->gpsdata->fix.altitude;
	}	    
	(void)sky_write(session, msg);
    }
}

static void sky_mode(struct gps_device_t *session, int mode)
{
    unsigned char msg[] = {
	SKY_START_1, SKY_START_2, 0x00, 0x03,
	SKY_CONFIG_MSG_TYPE, 0x00, 0x00,
	0x00, SKY_END_1, SKY_END_2};
    if (mode == MODE_NMEA) msg[5] = 0x01;
    if (mode == MODE_BINARY) msg[5] = 0x02;
    (void)sky_write(session, msg);
    return;
}

/* Note:
 * Can only change the baud rate; rest is locked at 8N1
 */
static bool sky_speed(struct gps_device_t *session,
		      speed_t speed, char parity UNUSED, int stopbits UNUSED)
{
    unsigned char msg[] = {
	SKY_START_1, SKY_START_2,
	0x00, 0x04,
	SKY_CONFIG_SERIAL, 0x00, 0x00, 0x00,
	0x00, SKY_END_1, SKY_END_2};
    unsigned spd = 5; /* default to 115200 baud */
    switch(speed) {
    case 4800:
	spd = 0;
	break;
    case 9600:
	spd = 1;
	break;
    case 19200:
	spd = 2;
	break;
    case 38400:
	spd = 3;
	break;
    case 57600:
	spd = 4;
	break;
    case 115200:
	spd = 5;
	break;
    case 230400:
	spd = 6;
	break;
    case 460800:
	spd = 7;
	break;
    case 921600:
	spd = 8;
	break;
    default:
	gpsd_log(&session->context->errout, LOG_ERROR,
		 "Skytraq: unknown baud rate (%d), defaulting to %d\n",
		 speed, 115200);
    }
    msg[6] = spd;
    return sky_write(session,msg);
}

static bool sky_rate(struct gps_device_t *session, double cycletime)
/* change the sample rate of the GPS */
{
    unsigned char msg[] = {
	SKY_START_1, SKY_START_2,
	0x00, 0x00,
	SKY_CONFIG_POS_UPD_RATE, 0x00, 0x00,
	0x00, SKY_END_1, SKY_END_2};
    /* gpsd like to have a cycletime rather than an update rate
     * so flip it and ensure we dont get rounding errors */
#define ACC 0.05
#define SKY_RATES 9
    unsigned rates[SKY_RATES] = {1, 2, 4, 5, 8, 10, 20, 40, 50};
    unsigned rate = (unsigned) (100/cycletime);
    int i;
    for(i=0;i<SKY_RATES;i++) {
      if (rate>=(rates[i]*100*(1-ACC)) &&
	  rate<=(rates[i]*100*(1+ACC))) {
	msg[5] = rates[i];
	break;
      }
    }
    if(0 == msg[5]) {
      /* We didn't find a matching rate: complain and don't try to update */
      gpsd_log(&session->context->errout, LOG_ERROR,
	       "Skytraq: cycletime doesn't match any valid rate (%f %d)\n",
	       cycletime, rate);
      return false;
    }
    return sky_write(session, msg);
}

/*
 * decode MID 0x80, Software Version
 *
 * 14 bytes
 */
static gps_mask_t sky_msg_sw_ver(struct gps_device_t *session,
				 unsigned char *buf, size_t len)
{
    unsigned int kver_x;  /* kernel version */
    unsigned int kver_y;  /* kernel version */
    unsigned int kver_z;  /* kernel version */
    unsigned int over_x;  /* ODM version */
    unsigned int over_y;  /* ODM version */
    unsigned int over_z;  /* ODM version */
    unsigned int rev_yy;   /* revision */
    unsigned int rev_mm;   /* revision */
    unsigned int rev_dd;   /* revision */

    if ( 14 != len)
	return 0;

    kver_x  = getbeu16(buf, 2);
    kver_y  = getub(buf, 4);
    kver_z  = getub(buf, 5);
    over_x  = getbeu16(buf, 6);
    over_y  = getub(buf, 8);
    over_z  = getub(buf, 9);
    rev_yy  = getbeu16(buf, 10);
    rev_mm  = getub(buf, 12);
    rev_dd  = getub(buf, 13);

    (void)snprintf(session->subtype, sizeof(session->subtype) - 1,
	     "kver=%3u.%2u,%2u, over=%3u.%2u,%2u, rev=%02u.%02u.%02u",
	    kver_x, kver_y, kver_z,
	    over_x, over_y, over_z,
	    rev_yy, rev_mm, rev_dd);

    gpsd_log(&session->context->errout, LOG_DATA,
	     "Skytraq: Software version: kver=%u.%u,%u, over=%u.%u,%u, rev=%u.%u.%u\n",
	    kver_x, kver_y, kver_z,
	    over_x, over_y, over_z,
	    rev_yy, rev_mm, rev_dd);
    return 0;
}

/*
 * decode MsgID 0x81, Software CRC
 *   This message is a response to "Query Software CRC", MsgID 0x03
 * 11 bytes
 */
static gps_mask_t sky_msg_sw_crc(struct gps_device_t *session,
		      unsigned char *buf, size_t len)
{
    unsigned int crc; /* CRC value */
    if ( 11 != len)
	return 0;

    crc = getbeu16(buf,2);
    (void)snprintf(session->subtype, sizeof(session->subtype) - 1,
		   "CRC=0x%04x", crc);
    return 0;
}

int fix_to_mode[] = {MODE_NO_FIX, MODE_2D, MODE_3D, MODE_3D};

static gps_mask_t sky_msg_boot_status(struct gps_device_t *session, unsigned char *buf, size_t len UNUSED)
{
  int status;
  int flash;
  status = getub(buf, 2);
  flash  = getub(buf, 3);
  gpsd_log(&session->context->errout, LOG_PROG,
	   "Skytraq: boot status%s ok, flash type %02x\n",status? "" : " not",
	   flash);
  return 0;
}

static gps_mask_t sky_msg_nav_data_msg(struct gps_device_t *session, unsigned char *buf, size_t len)
{
    gps_mask_t mask;
    unsigned char  fix;		/* Fix mode (0: No fix, 1: 2D, 2: 3D, 3: 3D+DGNSS */
#if 0
    unsigned char  sv;		/* Number of SV in fix */
    unsigned short week;	/* GNSS week number */
    unsigned int   tow;		/* Time of Week */
    int lat;			/* Latitude  [1E-7 degrees] N>0>S */
    int lon;			/* Longitude [1E-7 degrees] W>0>E */
    unsigned int ealt;		/* Height above ellipsoid [0.01m] */
    unsigned int salt;		/* Height above mean sea level [0.01m] */
    unsigned short gdop;	/* Geometric dilution of precision  [0.01] */
    unsigned short pdop;	/* Position dilution of precision   [0.01] */
    unsigned short hdop;	/* Horizontal dilution of precision [0.01] */
    unsigned short vdop;	/* Vertical dilution of precision   [0.01] */
    unsigned short tdop;	/* Time dilution of precision       [0.01] */
    int ecef_x;			/* ECEF X coordinate [0.01m] */
    int ecef_y;			/* ECEF Y coordinate [0.01m] */
    int ecef_z;			/* ECEF Z coordinate [0.01m] */
    int ecef_vx;		/* ECEF X velocity [0.01m] */
    int ecef_vy;		/* ECEF X velocity [0.01m] */
    int ecef_vz;		/* ECEF X velocity [0.01m] */
#endif

    if (59 != len) {
	gpsd_log(&session->context->errout, LOG_ERR, "Skytraq: "
		 "Navigation Data Message has incorrect length %d\n", len);
	return 0;
    }

    mask = 0;
    fix = getub(buf, 1);
    if (fix<4) {
	session->newdata.mode = fix_to_mode[fix];
	mask |= MODE_SET;
    }
    // sv  =  getub(buf,  2);
    // week = getbeu16(buf,  3);
    // tow  = getbeu32(buf,  5);
    session->newdata.latitude = 1E-7 * getbes32(buf,  9);
    session->newdata.longitude = 1E-7 * getbes32(buf, 13);
    mask |= LATLON_SET;
    // ealt = getbeu32(buf, 17);
    // salt = getbeu32(buf, 21);
#if 0
    session->gpsdata.dop.gdop = getbeu16(buf, 25);
    session->gpsdata.dop.pdop = getbeu16(buf, 27);
    session->gpsdata.dop.hdop = getbeu16(buf, 29);
    session->gpsdata.dop.vdop = getbeu16(buf, 31);
    session->gpsdata.dop.tdop = getbeu16(buf, 33);
    mask |= DOP_SET;
#endif
    session->newdata.ecef.x = 0.01 * getbes32(buf, 35);
    session->newdata.ecef.y = 0.01 * getbes32(buf, 39);
    session->newdata.ecef.z = 0.01 * getbes32(buf, 43);
    session->newdata.ecef.vx = 0.01 * getbes32(buf, 47);
    session->newdata.ecef.vy = 0.01 * getbes32(buf, 51);
    session->newdata.ecef.vz = 0.01 * getbes32(buf, 55);
    mask |= ECEF_SET | VECEF_SET;
    gpsd_log(&session->context->errout, LOG_DATA,
	     "Skytraq: NAVDATA mode=%d lat=%f lon=%f ECEF =%f/%f/%f\n",
	     fix, session->newdata.latitude, session->newdata.longitude,
	     session->newdata.ecef.x, session->newdata.ecef.y, session->newdata.ecef.z);
    return 0;
}

/*
 * decode MID 0xDC, Measurement Time
 *
 * 10 bytes
 */
static gps_mask_t sky_msg_DC(struct gps_device_t *session,
				  unsigned char *buf, size_t len)
{
    unsigned int iod;   /* Issue of data 0 - 255 */
    unsigned int wn;    /* week number 0 - 65535 */
    unsigned int tow;   /* receiver tow 0 - 604799999 in mS */
    unsigned int mp;    /* measurement period 1 - 1000 ms */
    /* calculated */
    double	f_tow;  /* tow in seconds */
    unsigned int msec;  /* mSec part of tow */

    if ( 10 != len)
	return 0;

    iod = (unsigned int)getub(buf, 1);
    wn = getbeu16(buf, 2);
    tow = getbeu32(buf, 4);
    f_tow = (double)(tow / 1000);
    msec = tow % 1000;
    mp = getbeu16(buf, 8);

    /* should this be newdata.skyview_time? */
    session->gpsdata.skyview_time = gpsd_gpstime_resolve(session, wn, f_tow );

    gpsd_log(&session->context->errout, LOG_DATA,
	     "Skytraq: MID 0xDC: iod=%u, wn=%u, tow=%u, mp=%u, t=%lld.%03u\n",
	     iod, wn, tow, mp,
	     (long long)session->gpsdata.skyview_time, msec);
    return 0;
}

/*
 * decode MID 0xDD, Raw Measurements
 *
 */
static gps_mask_t sky_msg_DD(struct gps_device_t *session,
				  unsigned char *buf, size_t len UNUSED)
{
    unsigned int iod;   /* Issue of data 0 - 255 */
    unsigned int nmeas; /* number of measurements */

    iod = (unsigned int)getub(buf, 1);
    nmeas = (unsigned int)getub(buf, 2);

    gpsd_log(&session->context->errout, LOG_DATA,
	     "Skytraq: MID 0xDD: iod=%u, nmeas=%u\n",
	     iod, nmeas);
    return 0;
}

/*
 * decode MID 0xDE, SV and channel status
 *
 * max payload: 3 + (Num_sats * 10) = 483 bytes
 */
static gps_mask_t sky_msg_DE(struct gps_device_t *session,
				  unsigned char *buf, size_t len UNUSED)
{
    int st, nsv;
    unsigned int i;
    unsigned int iod;   /* Issue of data 0 - 255 */
    unsigned int nsvs;  /* number of SVs in this packet */

    iod = (unsigned int)getub(buf, 1);
    nsvs = (unsigned int)getub(buf, 2);
    /* too many sats? */
    if ( SKY_CHANNELS < nsvs )
	return 0;

    gpsd_zero_satellites(&session->gpsdata);
    for (i = st = nsv =  0; i < nsvs; i++) {
	int off = 3 + (10 * i); /* offset into buffer of start of this sat */
	bool good;	      /* do we have a good record ? */
	unsigned short sv_stat;
	unsigned short chan_stat;
	unsigned short ura;

	session->gpsdata.skyview[st].PRN = (short)getub(buf, off + 1);
	sv_stat = (unsigned short)getub(buf, off + 2);
	ura = (unsigned short)getub(buf, off + 3);
	session->gpsdata.skyview[st].ss = (float)getub(buf, off + 4);
	session->gpsdata.skyview[st].elevation =
	    (short)getbes16(buf, off + 5);
	session->gpsdata.skyview[st].azimuth =
	    (short)getbes16(buf, off + 7);
	chan_stat = (unsigned short)getub(buf, off + 9);

	session->gpsdata.skyview[st].used = (bool)(chan_stat & 0x30);
	good = session->gpsdata.skyview[st].PRN != 0 &&
	    session->gpsdata.skyview[st].azimuth != 0 &&
	    session->gpsdata.skyview[st].elevation != 0;

	gpsd_log(&session->context->errout, LOG_DATA,
		 "Skytraq: PRN=%2d El=%d Az=%d ss=%3.2f stat=%02x,%02x ura=%d %c\n",
		session->gpsdata.skyview[st].PRN,
		session->gpsdata.skyview[st].elevation,
		session->gpsdata.skyview[st].azimuth,
		session->gpsdata.skyview[st].ss,
		chan_stat, sv_stat, ura,
		good ? '*' : ' ');

	if ( good ) {
	    st += 1;
	    if (session->gpsdata.skyview[st].used)
		nsv++;
	}
    }

    session->gpsdata.satellites_visible = st;
    session->gpsdata.satellites_used = nsv;

    gpsd_log(&session->context->errout, LOG_DATA,
	     "Skytraq: MID 0xDE: nsvs=%u visible=%u iod=%u\n", nsvs,
	     session->gpsdata.satellites_visible, iod);
    return SATELLITE_SET | USED_IS;
}

/*
 * decode MID 0xDF, Nav status (PVT)
 *
 * 81 bytes
 */
static gps_mask_t sky_msg_DF(struct gps_device_t *session,
				  unsigned char *buf, size_t len)
{
    unsigned int iod;   /* Issue of data 0 - 255 */
    unsigned short navstat;
    unsigned int wn;    /* week number 0 - 65535 */
    double f_tow;         /* receiver tow Sec */
    double clock_bias;
    double clock_drift;
    gps_mask_t mask = 0;

    if ( 81 != len)
	return 0;

    iod = (unsigned int)getub(buf, 1);

    /* fix status is byte 2 */
    navstat = (unsigned short)getub(buf, 2);
    session->gpsdata.status = STATUS_NO_FIX;
    session->newdata.mode = MODE_NO_FIX;
    switch ( navstat ) {
    case 1:
	/* fix prediction, ignore */
	break;
    case 2:
	session->gpsdata.status = STATUS_FIX;
	session->newdata.mode = MODE_2D;
	break;
    case 3:
	session->gpsdata.status = STATUS_FIX;
	session->newdata.mode = MODE_3D;
	mask |= ALTITUDE_SET | CLIMB_SET;
	break;
    case 4:
	session->gpsdata.status = STATUS_DGPS_FIX;
	session->newdata.mode = MODE_3D;
	mask |= ALTITUDE_SET | CLIMB_SET;
	break;
    default:
	break;
    }

    wn = getbeu16(buf, 3);
    f_tow = getbed64((const char *)buf, 5);

    /* position/velocity is bytes 13-48, meters and m/s */
    ecef_to_wgs84fix(&session->newdata, &session->gpsdata.separation,
		     (double)getbed64((const char *)buf, 13),
		     (double)getbed64((const char *)buf, 21),
		     (double)getbed64((const char *)buf, 29),
		     (double)getbef32((const char *)buf, 37),
		     (double)getbef32((const char *)buf, 41),
		     (double)getbef32((const char *)buf, 46));

    clock_bias = getbed64((const char *)buf, 49);
    clock_drift = getbes32(buf, 57);

    session->gpsdata.dop.gdop = getbef32((const char *)buf, 61);
    session->gpsdata.dop.pdop = getbef32((const char *)buf, 65);
    session->gpsdata.dop.hdop = getbef32((const char *)buf, 69);
    session->gpsdata.dop.vdop = getbef32((const char *)buf, 73);
    session->gpsdata.dop.tdop = getbef32((const char *)buf, 77);

    session->newdata.time = gpsd_gpstime_resolve(session, wn, f_tow );

    gpsd_log(&session->context->errout, LOG_DATA,
	    "Skytraq: MID 0xDF: iod=%u, stat=%u, wn=%u, tow=%f, t=%.6f "
	    "cb: %f, cd: %f "
	    "gdop: %.2f, pdop: %.2f, hdop: %.2f, vdop: %.2f, tdop: %.2f\n",
	    iod, navstat, wn, f_tow,
	    session->newdata.time,
	    clock_bias, clock_drift,
	    session->gpsdata.dop.gdop,
	    session->gpsdata.dop.pdop,
	    session->gpsdata.dop.hdop,
	    session->gpsdata.dop.vdop,
	    session->gpsdata.dop.tdop);

    mask |= TIME_SET | LATLON_SET | TRACK_SET |
	SPEED_SET | STATUS_SET | MODE_SET | DOP_SET | CLEAR_IS | REPORT_IS;
    return mask;
}

/*
 * decode MID 0xE0, GPS Subframe data
 *
 * len 33 bytes
 *
 */
static gps_mask_t sky_msg_E0(struct gps_device_t *session,
				  unsigned char *buf, size_t len UNUSED)
{
    int i;
    unsigned int prn;   /* GPS sat PRN */
    unsigned int subf;  /* subframe 1-5 */
    /* the words are preprocessed, not raw, just the 24bits of data */
    uint32_t words[10];  /* subframe 1-5 */

    if ( 33 != len)
	return 0;

    prn = (unsigned int)getub(buf, 1);
    subf = (unsigned int)getub(buf, 2);
    for ( i = 0; i < 10; i++ ) {
	words[i] = (uint32_t)getbeu24(buf, 3 + (i * 3));
    }

    gpsd_log(&session->context->errout, LOG_DATA,
	     "Skytraq: 50B MID 0xE0: prn=%u, subf=%u\n",
	     prn, subf);

    return gpsd_interpret_subframe(session, prn, words);
}

/*
 * pretend to decode MID 0xE2, Beidou D1 Subframe data
 *
 * from Beidou Standard BDS-SIS-ICD-2.0
 * D1, with the data rate of 50 bps, is broadcasted by the MEO/IGSO satellites
 *
 * len 31 bytes
 *
 */
static gps_mask_t sky_msg_E2(struct gps_device_t *session,
				  unsigned char *buf, size_t len)
{
    int i;
    unsigned int prn;   /* BeidouPS sat PRN 206-214 */
    unsigned int subf;  /* subframe 1-5 */
    /* the words are preprocessed, not raw, just the 28 bytes of data */
    uint8_t bytes[28];  /* raw data */

    if ( 31 != len)
	return 0;

    prn = (unsigned int)getub(buf, 1);
    subf = (unsigned int)getub(buf, 2);
    for ( i = 0; i < 28; i++ ) {
	bytes[i] = getub(buf, 3 + i);
    }

    /* extra guard prevents expensive hexdump calls */
    if (session->context->errout.debug >= LOG_PROG) {
	gpsd_log(&session->context->errout, LOG_PROG,
		 "Skytraq: Beidou D1 subframe PRN %d Subframe %d "
	         "length %zd byte:%s\n",
		 prn, subf,
		 len,
		 gpsd_hexdump(session->msgbuf, sizeof(session->msgbuf),
				 (char *)bytes, 28));
    }

    return ONLINE_SET;
}

/*
 * pretend to decode MID 0xE3, Beidou D2 Subframe data
 *
 * from Beidou Standard BDS-SIS-ICD-2.0
 * D2, with the data rate of 500 bps, is broadcasted by the GEO satellites.
 *
 * len 31 bytes
 *
 */
static gps_mask_t sky_msg_E3(struct gps_device_t *session,
				  unsigned char *buf, size_t len)
{
    int i;
    unsigned int prn;   /* BeidouPS sat PRN 201-205 */
    unsigned int subf;  /* subframe 1-5 */
    /* the words are preprocessed, not raw, just the 28 bytes of data */
    uint8_t bytes[28];  /* raw data */

    if ( 31 != len)
	return 0;

    prn = (unsigned int)getub(buf, 1);
    subf = (unsigned int)getub(buf, 2);
    for ( i = 0; i < 28; i++ ) {
	bytes[i] = getub(buf, 3 + i);
    }

    /* extra guard prevents expensive hexdump calls */
    if (session->context->errout.debug >= LOG_PROG) {
	gpsd_log(&session->context->errout, LOG_PROG,
		 "Skytraq: Beidou D2 subframe PRN %d Subframe %d "
	         "length %zd byte:%s\n",
		 prn, subf,
		 len,
		 gpsd_hexdump(session->msgbuf, sizeof(session->msgbuf),
				 (char *)bytes, 28));
    }


    return ONLINE_SET;
}


static gps_mask_t sky_parse(struct gps_device_t * session, unsigned char *buf,
			    size_t len)
{
    gps_mask_t mask = 0;
    unsigned int csum = 0;
    int pl, i;
    unsigned char mid, sid;

    if (len == 0)
	return mask;

    /* check the checksum */
    pl = getbeu16(buf, 2);
    for(i=0; i<pl; i++)
	csum ^= buf[4+i];
    csum = csum & 0x00FF;
    if (buf[4+i] != csum) {
	gpsd_log(&session->context->errout, LOG_PROG,
		 "Skytraq: invalid csum in MID 0x%02x (expected %02x, got %02x)\n",
		 buf[4],buf[4+i],csum);
	return 0;
    }
    buf += 4;   /* skip the leaders and length */
    len -= 7;   /* don't count the leaders, length, csum and terminators */
    // session->driver.sirf.lastid = buf[0];

    /* could change if the set of messages we enable does */
    /* session->cycle_end_reliable = true; */

    mid = buf[0];
    sid = buf[1];
    switch (mid) {
    case SKY_RESP_SW_VER:
	gpsd_log(&session->context->errout, LOG_DATA, "SKY_RESP_SW_VER\n");
	return sky_msg_sw_ver(session, buf, len);
    case SKY_RESP_SW_CRC:
	gpsd_log(&session->context->errout, LOG_DATA, "SKY_RESP_SW_CRC\n");
	return sky_msg_sw_crc(session, buf, len);
    case SKY_RESP_ACK:
	gpsd_log(&session->context->errout, LOG_PROG,
		 "Skytraq: ACK to MID 0x%02x\n", buf[1]);
	break;
    case SKY_RESP_NACK:
	gpsd_log(&session->context->errout, LOG_INF,
		 "Skytraq: NACK to MID 0x%02x\n", buf[1]);
	break;
   case SKY_RESP_NAV_DATA_MSG:
	// return sky_msg_nav_data_msg(session, buf, len);
	return sky_msg_DC(session, buf, len);
    case SKY_MSGID_62:
      /* FALL-THROUGH */
    case SKY_MSGID_63:
      /* FALL-THROUGH */
    case SKY_MSGID_65:
        gpsd_log(&session->context->errout, LOG_ERROR,
		 "Skytraq: Not handling Mid/Sid: %02x/%02x yet\n",mid,sid);
	break;
    case SKY_MSGID_64:
	switch(sid) {
	case SKY_64_RESP_BOOT_STATUS:
	    return sky_msg_boot_status(session, buf, len);
	default:
	    gpsd_log(&session->context->errout, LOG_ERROR,
		     "Skytraq: Unable to handle Mid/Sid: %02x/%02x\n",mid,sid);
	    break;
	}
 #if 0 /* NOT IMPLEMENTED YET */
    case SKY_RESP_POS_UPD_RATE:
	return sky_msg_pos_upd_rate(session, buf, len);
    case SKY_RESP_NMEA_TALKER:
	return sky_msg_nmea_talkerid(session, buf, len);
    case SKY_RESP_DATUM:
	return sky_msg_datum(session, buf, len);
    case SKY_RESP_DOP:
	return sky_msg_dop(session, buf, len);
    case SKY_RESP_ELEV_CNR:
	return sky_msg_elev_cnr(session, buf, len);
    case SKY_RESP_WAAS:
	return sky_msg_waas(session, buf, len);
    case SKY_RESP_POS_PIN_STATUS:
	return sky_msg_pos_pin_status(session, buf, len);
    case SKY_RESP_NAV_MODE:
	return sky_msg_nav_mode(session, buf, len);
    case SKY_RESP_PPS_MODE:
	return sky_msg_pps_mode(session, buf, len);
    case SKY_RESP_POWER_MODE:
	return sky_msg_power_mode(session, buf, len);
    case SKY_RESP_PPS_CABLE_MODE:
	return sky_msg_pps_cable_mode(session, buf, len);
    case SKY_RESP_PPS_TIMING:
	return sky_msg_pps_timing(session, buf, len);
#endif
    case SKY_MSG_MEAS_TIME:
	/* 220 */
	return sky_msg_DC(session, buf, len);
    case SKY_MSG_MEAS_RAW:
	/* 221 */
	return sky_msg_DD(session, buf, len);
    case SKY_MSG_SV_CHAN_STATUS:
	/* 222 */
	return sky_msg_DE(session, buf, len);
    case SKY_MSG_NAV_PVT:
	/* 223 - Navigation status (PVT)  */
	return sky_msg_DF(session, buf, len);
    case SKY_MSG_SUBFRAME_DATA:
	/* 224 */
	return sky_msg_E0(session, buf, len);
    case SKY_MSG_BD_SUBFRAME_D1:
	/* 226 - Beidou2 D1 Subframe data */
	return sky_msg_E2(session, buf, len);
    case SKY_MSG_BD_SUBFRAME_D2:
	/* 227 - Beidou2 D2 Subframe data */
	return sky_msg_E3(session, buf, len);

    default:
	gpsd_log(&session->context->errout, LOG_PROG,
		 "Skytraq: Unknown MessageID 0x%02x length %zd\n",
		 buf[0], len);
    }
    return mask;
}

static gps_mask_t skybin_parse_input(struct gps_device_t *session)
{
    if (session->lexer.type == SKY_PACKET) {
	return sky_parse(session, session->lexer.outbuffer,
			 session->lexer.outbuflen);
    } else
	return generic_parse_input(session);
#if 0
# ifdef NMEA0183_ENABLE
    } else if (session->lexer.type == NMEA_PACKET) {
	return nmea_parse((char *)session->lexer.outbuffer, session);
# endif /* NMEA0183_ENABLE */
    } else
	return 0;
#endif
}

/* this is everything we export */
/* *INDENT-OFF* */
const struct gps_type_t driver_skytraq =
{
    .type_name      = "Skytraq",	/* full name of type */
    .packet_type    = SKY_PACKET,	/* associated lexer packet type */
    .flags	    = DRIVER_STICKY,	/* remember this */
    .trigger	    = NULL,		/* no trigger */
    .channels       =  SKY_CHANNELS,	/* consumer-grade GPS */
    .probe_detect   = NULL,		/* no probe */
    .get_packet     = generic_get,	/* be prepared for Skytraq or NMEA */
    .parse_packet   = skybin_parse_input,/* parse message packets */
    .rtcm_writer    = gpsd_write,	/* send RTCM data straight */
    .init_query     = sky_init_query,	/* non-perturbing initial query */
    .event_hook     = sky_event_hook,	/* lifetime event handler */
#ifdef RECONFIGURE_ENABLE
    .speed_switcher   = sky_speed,	/* Speed (baudrate) switch */
    .mode_switcher    = sky_mode,	/* Mode switcher */
    .rate_switcher    = sky_rate,	/* Message delivery rate switcher */
#endif /* RECONFIGURE_ENABLE */
#ifdef CONTROLSEND_ENABLE
    .control_send     = sky_control_send,/* how to send a control string */
#endif /* CONTROLSEND_ENABLE */
};
/* *INDENT-ON* */
#endif /* defined( SKYTRAQ_ENABLE) && defined(BINARY_ENABLE) */

