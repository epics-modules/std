/* Includes Pete's modification for new format of ID string (XIA dropped
the leading zero. */

program xia_slit ("name=hsc1, S=ito:serial:b0, P=ito:, HSC=hsc1:")

option +r;  /* re-usable code, can load more than once */
option +d;  /* turn ON run-time debugging messages */

/*
 * usage:
 ld < xia_slit.o
 seq &xia_slit "name=hsc1, S=ito:serial:b0, P=ito:, HSC=hsc1:"
 
 * to kill the running sequence program:
 td "hsc1"

       WISH LIST for XIAHSC
       ====================
   * support device aliases
   * motor velocity control
   * gear backlash setting
   * status bits
   * read limits from controller
   * progress indication (not provided on device)
   * support for INQUIRY command
   * user coordinates
   * user can change the serial port to be used

 */

/*------------------------------------------------ 
  ------------------------------------------------   assignments
  ------------------------------------------------ */

#define ASSIGN(SIZE, LOCAL_VAR, PV) \
        SIZE LOCAL_VAR;             \
        assign LOCAL_VAR to PV;

#define ASSIGN_MONITOR(SIZE, LOCAL_VAR, PV) \
        ASSIGN(SIZE, LOCAL_VAR, PV)         \
        monitor LOCAL_VAR;

#define ASSIGN_MON_FLAG(SIZE, LOCAL_VAR, PV, LOCAL_FLAG) \
        ASSIGN_MONITOR(SIZE, LOCAL_VAR, PV)              \
        evflag LOCAL_FLAG;                               \
        sync   LOCAL_VAR LOCAL_FLAG;


    /**** 
     **** generic serial record 
     ****/
ASSIGN_MON_FLAG(string, s_ainp, "{S}.AINP", s_ainp_mon)
ASSIGN        (string, s_aout,  "{S}.AOUT")
ASSIGN        (short,  s_baud,  "{S}.BAUD")
ASSIGN        (short,  s_dbit,  "{S}.DBIT")
ASSIGN        (short,  s_fctl,  "{S}.FCTL")
ASSIGN        (short,  s_idel,  "{S}.IDEL")
ASSIGN        (short,  s_ifmt,  "{S}.IFMT")
ASSIGN        (short,  s_nrrd,  "{S}.NRRD")
ASSIGN        (short,  s_odel,  "{S}.ODEL")
ASSIGN        (short,  s_ofmt,  "{S}.OFMT")
ASSIGN        (short,  s_proc,  "{S}.PROC")
ASSIGN        (short,  s_prty,  "{S}.PRTY")
ASSIGN        (short,  s_sbit,  "{S}.SBIT")
ASSIGN        (short,  s_scan,  "{S}.SCAN")
ASSIGN_MONITOR(int,    s_sevr,  "{S}.SEVR")
ASSIGN        (short,  s_tmod,  "{S}.TMOD")
ASSIGN_MONITOR(long,   s_tmot,  "{S}.TMOT")


    /**** 
     **** XIAHSC database 
     ****/
ASSIGN         (string, port,      "{P}{HSC}port")
ASSIGN_MON_FLAG(string, hID,       "{P}{HSC}hID",     hID_mon)
ASSIGN_MON_FLAG(string, vID,       "{P}{HSC}vID",     vID_mon)
ASSIGN_MON_FLAG(short,  hOrient,   "{P}{HSC}hOrient", hOrient_mon)
ASSIGN_MON_FLAG(short,  vOrient,   "{P}{HSC}vOrient", vOrient_mon)
ASSIGN         (short,  hBusy,     "{P}{HSC}hBusy")
ASSIGN         (short,  vBusy,     "{P}{HSC}vBusy")
ASSIGN         (short,  Busy,      "{P}{HSC}Busy")
ASSIGN         (short,  BusyOff,   "{P}{HSC}BusyOff.PROC")
ASSIGN         (short,  BusyDisable,   "{P}{HSC}BusyDisable")
ASSIGN_MON_FLAG(short,  stop,      "{P}{HSC}stop",    stop_mon)
ASSIGN_MONITOR (short,  locate,    "{P}{HSC}locate")
ASSIGN_MONITOR (short,  init,      "{P}{HSC}init")
ASSIGN_MONITOR (short,  enable,    "{P}{HSC}enable")
ASSIGN_MONITOR (short,  calibrate, "{P}{HSC}calib")
ASSIGN_MON_FLAG(double, t,         "{P}{HSC}t",       t_mon)
ASSIGN_MONITOR (double, t_hi,      "{P}{HSC}t.HOPR")
ASSIGN_MONITOR (double, t_lo,      "{P}{HSC}t.LOPR")
ASSIGN_MONITOR (double, t_RB,      "{P}{HSC}tRB")
ASSIGN_MON_FLAG(double, l,         "{P}{HSC}l",       l_mon)
ASSIGN_MONITOR (double, l_hi,      "{P}{HSC}l.HOPR")
ASSIGN_MONITOR (double, l_lo,      "{P}{HSC}l.LOPR")
ASSIGN_MONITOR (double, l_RB,      "{P}{HSC}lRB")
ASSIGN_MON_FLAG(double, b,         "{P}{HSC}b",       b_mon)
ASSIGN_MONITOR (double, b_hi,      "{P}{HSC}b.HOPR")
ASSIGN_MONITOR (double, b_lo,      "{P}{HSC}b.LOPR")
ASSIGN_MONITOR (double, b_RB,      "{P}{HSC}bRB")
ASSIGN_MON_FLAG(double, r,         "{P}{HSC}r",       r_mon)
ASSIGN_MONITOR (double, r_hi,      "{P}{HSC}r.HOPR")
ASSIGN_MONITOR (double, r_lo,      "{P}{HSC}r.LOPR")
ASSIGN_MONITOR (double, r_RB,      "{P}{HSC}rRB")
ASSIGN_MON_FLAG(double, h0,        "{P}{HSC}h0",      h0_mon)
ASSIGN_MONITOR (double, h0_hi,     "{P}{HSC}h0.HOPR")
ASSIGN_MONITOR (double, h0_lo,     "{P}{HSC}h0.LOPR")
ASSIGN_MONITOR (double, h0_RB,     "{P}{HSC}h0RB")
ASSIGN_MON_FLAG(double, v0,        "{P}{HSC}v0",      v0_mon)
ASSIGN_MONITOR (double, v0_hi,     "{P}{HSC}v0.HOPR")
ASSIGN_MONITOR (double, v0_lo,     "{P}{HSC}v0.LOPR")
ASSIGN_MONITOR (double, v0_RB,     "{P}{HSC}v0RB")
ASSIGN_MON_FLAG(double, width,     "{P}{HSC}width",   width_mon)
ASSIGN_MONITOR (double, width_hi,  "{P}{HSC}width.HOPR")
ASSIGN_MONITOR (double, width_lo,  "{P}{HSC}width.LOPR")
ASSIGN_MONITOR (double, width_RB,  "{P}{HSC}widthRB")
ASSIGN_MON_FLAG(double, height,    "{P}{HSC}height",  height_mon)
ASSIGN_MONITOR (double, height_hi, "{P}{HSC}height.HOPR")
ASSIGN_MONITOR (double, height_lo, "{P}{HSC}height.LOPR")
ASSIGN_MONITOR (double, height_RB, "{P}{HSC}heightRB")
ASSIGN         (int,    error,     "{P}{HSC}error")
ASSIGN         (string, errMsg,    "{P}{HSC}errMsg")

    /****  
     **** local variables 
     ****/
char   scratch[256];
long   numWords;
string word1;         string word2; string word3; 
string word4;         string word5; string word6;
string movingMotorID;
long   pos_a;         long   pos_b;
double pos_ag;        double pos_bg;
long l_raw; long r_raw; long t_raw; long b_raw;
string id;            short  hasID;
string hscErrors[14];
evflag start_H_move;  evflag start_V_move;
evflag wait_H_input;  evflag wait_V_input;
evflag new_H_RB;      evflag new_V_RB;
evflag move_h;        evflag move_v;
double t_old;         double l_old;
double b_old;         double r_old;
double width_old;     double height_old;
double h0_old;        double v0_old;
int    init_delay;
#define  AINP_FIFO_SIZE  20
string ainp_fifo[AINP_FIFO_SIZE];
int    ainp_fifo_write;
int    ainp_fifo_read;
int    i;

%% #include <math.h>
%% #include <string.h>
/*
%% static int i;
*/

/*------------------------------------------------ 
  ------------------------------------------------   macros
  ------------------------------------------------ */

#define SCAN_PASSIVE    0
#define SCAN_RAPID      7
#define TMOD_WRITE_READ 0
#define TMOD_WRITE      1
#define TMOD_READ       2
#define LINE_FEED       10
#define CARRIAGE_RETURN 13
#define ORIENT_LEFT_RIGHT      0
#define ORIENT_RIGHT_LEFT      1
#define ORIENT_TOP_BOTTOM      0
#define ORIENT_BOTTOM_TOP      1
#define MOTOR_IDLE             0
#define MOTOR_BUSY             1
#define STOP_IDLE              0
#define STOP_STOP              1
#define CALIBRATE_DONE         0
#define CALIBRATE_START        1

#define BAUD_9600	5
#define DBIT_7		3
#define SBIT_2		0
#define PRTY_EVEN	0
#define FCTL_NONE	0

#define MOVING_POLL_INTERVAL	0.2
#define IDLE_POLL_INTERVAL	30.0
#define MOVE_TIMEOUT		300.0
#define MOVE_HESITATION		0.0

/*
 * there are 14 error messages from the HSC
 * (numbered in HSC from 0-13, I move them to 1-14)
 * error 0 means no error
 */
#define ERROR_SOFT_LIMITS       15
#define ERROR_UNKNOWN		16
#define ERROR_BAD_ID		17

#define PVPUT(PV,VAL)  {PV = (VAL);  pvPut(PV);}

#define PVPUTSTR(MSG,TEXT)  {strcpy(MSG, TEXT); pvPut(MSG);}

#define SEND_HSC(HSC_COMMAND)  {	\
   PVPUT(s_scan,  SCAN_PASSIVE);	\
   PVPUT(s_tmod,  TMOD_WRITE);		\
   PVPUT(s_idel,  LINE_FEED);		\
   PVPUT(s_odel,  CARRIAGE_RETURN);	\
   PVPUTSTR(s_aout,  HSC_COMMAND);	\
   delay (.02); }

#define WAIT_AINP()   {		\
   efClear (s_ainp_mon); /* will wait for the record to process */	\
   for (i = 0; i <= 15; i++) { /* wait no more than 0.25 seconds */	\
     if (efTest (s_ainp_mon)) break;	/* input is ready */		\
     delay (.02);	/* wait 1 clock tick before retrying */		\
   } } \

#define READ_HSC()   {		\
   PVPUT(s_idel,  LINE_FEED);		\
   PVPUT(s_tmod,  TMOD_READ);		\
   PVPUT(s_scan,  SCAN_PASSIVE);	\
   PVPUT (s_proc, 1);			\
   WAIT_AINP(); \
}

#define FIFO_PTR_CHECK(PTR) {\
   if (PTR >= AINP_FIFO_SIZE) PTR = 0; }

#define INPUT_TO_FIFO(INPUT)   {		\
  strcpy(ainp_fifo[ainp_fifo_read++], INPUT);	\
  FIFO_PTR_CHECK(ainp_fifo_read); }

#define FIFO_TO_OUTPUT(OUTPUT)   {			\
  strcpy (OUTPUT, ainp_fifo[ainp_fifo_write++]);	\
  FIFO_PTR_CHECK(ainp_fifo_write); }

#define S_PORT_CONFIG(BAUD, DBIT, SBIT, PRTY, FCTL)   { \
      PVPUT (s_baud,  BAUD);	\
      PVPUT (s_dbit,  DBIT);	\
      PVPUT (s_sbit,  SBIT);	\
      PVPUT (s_prty,  PRTY);	\
      PVPUT (s_fctl,  FCTL); }

#define RAW_TO_DIAL(A) ( (A)/400.0 - 1.0 )
#define DIAL_TO_RAW(A) ( (A + 1.0) * 400 + 0.5 )

#define LIMIT_TEST(LO,VAL,HI)	( (LO <= VAL) && (VAL <= HI) )

/* 
 * slit state:  xiahsc
 */
ss xiahsc {

  state startup {
    when () {
      pvGet (s_ainp);
      pvGet (s_aout);
      PVPUT (init, 1);		/* need to initialize */
      init_delay = 1.0;		/* time to delay before re-init */
      /*
       * users have requested some mention of the associated serial port name
       * for the future:  Can use change the serial port via this PV?
       */
      strcpy (port, macValueGet("S"));
      pvPut (port);

      strcpy (hscErrors[0],  "Missing Command");
      strcpy (hscErrors[1],  "Unrecognized Command");
      strcpy (hscErrors[2],  "Input Buffer Overflow");
      strcpy (hscErrors[3],  "No new Alias Given");
      strcpy (hscErrors[4],  "Alias too long");
      strcpy (hscErrors[5],  "Invalid Field Parameter");
      strcpy (hscErrors[6],  "Value Out of Range");
      strcpy (hscErrors[7],  "Parameter is read-only");
      strcpy (hscErrors[8],  "Invalid/Missing Argument");
      strcpy (hscErrors[9],  "No Movement Required");
      strcpy (hscErrors[10], "Uncalibrated: no motion allowed");
      strcpy (hscErrors[11], "Motion out of range");
      strcpy (hscErrors[12], "Invalid/missing direction character");
      strcpy (hscErrors[13], "Invalid Motor Specified");

      /*
       * keep track of last known good value
       * to impose motor limits
       */
      t_old = t;	b_old = b;
      l_old = l;	r_old = r;
      h0_old = h0;	width_old = width;
      v0_old = v0;	height_old = height;

      PVPUT (enable,       1);
      PVPUT (s_tmot,    250);

    } state init
  }

  /* 
   */
  state disable { 
    when (enable) { 
      init_delay = 1.0;
      PVPUT (init, 1);
    } state init
  }

 /* 
 */
  state init {
    when (init && (efTest (hID_mon) || efTest (vID_mon))) {
      /*
       * configure the serial port parameters
       */
      /*
       * users have requested some mention of the associated serial port name
       * reset it here in case some curious user changed it
       */
      strcpy (port, macValueGet("S"));
      pvPut (port);
      /*
       * make a quick check after user has changed an ID
       */
      init_delay = 0;
      efClear (hID_mon); 
      efClear (vID_mon); 
    } state init
    when (init && delay(init_delay)) { 
      /*
       * verify that the ID strings look OK
       */
      error = 0;
      if (!strcmp (hID, vID)) { 
        PVPUTSTR (errMsg, "H & V IDs must be different");
        error++;
      }
      if (sscanf (hID, "XIAHSC-%*c-%ld", &numWords) != 1) { 
        PVPUTSTR (errMsg, "H ID not a valid HSC ID");
        error++;
      }
      if (sscanf (vID, "XIAHSC-%*c-%ld", &numWords) != 1) { 
        PVPUTSTR (errMsg, "V ID not a valid HSC ID");
        error++;
      }
      if (!strlen (hID)) { 
        PVPUTSTR (errMsg, "define H ID string");
        error++;
      }
      if (!strlen (vID)) { 
        PVPUTSTR (errMsg, "define V ID string");
        error++;
      }
      /*
       * could also try to use each ID string
       */
      if (error) { 
        PVPUT (error, ERROR_BAD_ID);
      } else { 
        PVPUT (init, 0);
        PVPUTSTR (errMsg, "no error");
      }
      pvPut (error);
      /* recheck every 30 seconds for appropriate ID strings */
      init_delay = 30;
    } state init
    when (!init && delay(0.1)) {
      PVPUT(hBusy,   MOTOR_IDLE);
      PVPUT(vBusy,   MOTOR_IDLE);
/* printf("state init: when(!init):  BusyOff=0\n"); */
      PVPUT(BusyOff,   0);
      /*
       * serial port configuration
       */
      S_PORT_CONFIG(BAUD_9600, DBIT_7, SBIT_2, PRTY_EVEN, FCTL_NONE);
      /*
       * generic serial record configuration
       */
      PVPUT (s_idel, LINE_FEED);	/* input delimiter is <LF> */
      PVPUT (s_ifmt,  0);	/* ASCII input format */
      PVPUT (s_nrrd,  0);	/* number of characters to read */
      PVPUT (s_odel, CARRIAGE_RETURN);	/* output delim is <CR> */
      PVPUT (s_ofmt, 0);	/* ASCII output format */
      PVPUT (s_tmod,  0);	/* write/read transfer mode */
      PVPUT (s_scan,  SCAN_PASSIVE);	/* serial port is idle */
      PVPUT (s_tmot, 250);	/* timeout in 1/4 second */

      efClear (s_ainp_mon);	/* watch serial record process */

      efClear (wait_H_input);   /* flag to watch for H axis input */
      efClear (wait_V_input);   /* flag to watch for V axis input */

      efClear (move_h);		/* flag H axis to be moved */
      efClear (move_v);		/* flag V axis to be moved */
      efClear (new_H_RB);	/* new readback data available */
      efClear (new_V_RB);	/* new readback data available */

      PVPUT (stop, STOP_IDLE);	/* presume the motors are stopped */
      delay(.02);		/* allow the command to process */
      efClear (stop_mon);

      efClear (t_mon);  efClear (l_mon);  efClear (b_mon);  efClear (r_mon);
      efClear (h0_mon); efClear (width_mon);
      efClear (v0_mon); efClear (height_mon);
      PVPUT (calibrate, CALIBRATE_DONE);	/* don't reset pos. to zero */

      /*
       * Now, everything is initialized
       * stop all the motors and ask where are we
       */
      WAIT_AINP();		/* wait for the record to process or timeout */
      SEND_HSC ("!ALL K");	/* stop all motors */
      delay(.1);		/* wait for record to process */
      SEND_HSC ("!ALL P");	/* request all motor positions */
      delay(.1);		/* wait for record to process */
      ainp_fifo_read = 0;	/* next FIFO position to store input */
      ainp_fifo_write = 0;	/* next FIFO position to process */
      while (!s_sevr) {		/* read until no more input data */
        READ_HSC();		/* process generic serial port */
        INPUT_TO_FIFO(s_ainp);	/* put input string into FIFO */
      }
    } state idle
  }

/* 
 */
  state idle {

    /*
     * user can disable this program
     */
    when (!enable) { } state disable

    /*
     * user can re-initialize this program
     */
    when (init) {
		init_delay = 1; PVPUT (init, 1); PVPUT (BusyOff, 0);
	} state init

    /*
     * user can stop the motors
     */
    when (efTest (stop_mon) && stop) { 
      WAIT_AINP();		/* wait for the record to process or timeout */
      SEND_HSC ("!ALL K");
      delay(.1);
      efClear (stop_mon);
    } state idle

    /*
     * target value has changed
     */
    when ( efTest (l_mon)     || efTest (r_mon) 
    ||     efTest (t_mon)     || efTest (b_mon) 
    ||     efTest (h0_mon)    || efTest (height_mon) 
    ||     efTest (v0_mon)    || efTest (width_mon)   ) {
    } state premove

    /*
     * send a MOVE command to HSC
     *  wait just a skosh before processing
     */
    when ( (efTest (start_H_move) || efTest (start_V_move)) 
    &&     delay (MOVE_HESITATION)) {
      if (efTest (start_H_move)) { 
        if (hBusy) { 		/* allow an active move to be interrupted */
          WAIT_AINP();		/* wait for the record to process or timeout */
          sprintf (scratch, "!%s K", hID);	/* stop this axis only */
          SEND_HSC (scratch);		/* do it */
          delay(.5);		/* wait for it to happen */
        } else {
          PVPUT (hBusy, MOTOR_BUSY);	/* tell the world we are busy */
          if (vBusy) delay(.25);	/* port may be busy with a READ */
        }
        if (hOrient == ORIENT_LEFT_RIGHT) { 
          pos_ag = l;  pos_bg = r;
        } else { 
          pos_ag = r;  pos_bg = l;
        }
        strcpy (movingMotorID, hID);		/* horizontal axis */
        efSet (wait_H_input);		/* could enable a timeout on the move */
        efClear (start_H_move);		/* OK, we told it to move now */
      }
      if (efTest (start_V_move)) { 
        if (vBusy) {  		/* allow an active move to be interrupted */
          WAIT_AINP();		/* wait for the record to process or timeout */
          sprintf (scratch, "!%s K", vID);	/* stop this axis only */
          SEND_HSC (scratch);		/* do it */
          delay(.5);		/* wait for it to happen */
        } else {
          PVPUT (vBusy, MOTOR_BUSY);	/* tell the world we are busy */
          if (hBusy) delay(.25);	/* port may be busy with a READ */
        }
        if (vOrient == ORIENT_TOP_BOTTOM) { 
          pos_ag = t;  pos_bg = b;
        } else { 
          pos_ag = b;  pos_bg = t;
        }
        strcpy (movingMotorID, vID);		/* vertical axis */
        efSet (wait_V_input);		/* could enable a timeout on the move */
        efClear (start_V_move);
      }
      pos_a = DIAL_TO_RAW (pos_ag);	/* convert to raw (motor) coordinates */
      pos_b = DIAL_TO_RAW (pos_bg);	/* convert to raw (motor) coordinates */
      sprintf (scratch, "!%s M %ld %ld", movingMotorID, pos_a, pos_b);
      SEND_HSC (scratch);		/* send the command */
      PVPUT (locate, 1);		/* expect a readback */
    } state idle

    /*
     * user can request device to resend coordinates
     */
    when (locate) { 	/* poll the generic serial port for input */
      /*
       * users have requested some mention of the associated serial port name
       * reset here in case some curious user changed it
       */
      strcpy (port, macValueGet("S"));
      pvPut (port);
      PVPUT (locate, 0);
      SEND_HSC ("!ALL P");
      delay(.1);
      while (!s_sevr) {		/* read until no more input data */
        READ_HSC();		/* process generic serial port */
        INPUT_TO_FIFO(s_ainp);	/* put input string into FIFO */
      }
    } state idle

    /*
     * target value has changed
     */
    when ( calibrate ) {
      SEND_HSC ("!ALL 0 I");	/* reset all axes to position=400 steps */
      delay(.1);		/* wait for record to process */
      PVPUT (calibrate, CALIBRATE_DONE);	/* OK, we're done here */
      PVPUT (locate, 1);	/* update with new controller positions */
    } state idle


    /*
     * process the input data
     */
    when (ainp_fifo_read != ainp_fifo_write) { 
      FIFO_TO_OUTPUT(scratch);
      id[0] = word1[0] = word2[0] = word3[0] = word4[0] = 
        word5[0] = word6[0] = 0;
      numWords = sscanf (scratch, "%s %s %s %s %s %s", 
		word1, word2, word3, word4, word5, word6);
      hasID = sscanf (word1, "%%%s", id);
    } state process_response

    /*
     * new H axis readback data is available
     */
    when (efTest (new_H_RB)) { 
      efClear (new_H_RB);
      /*
       * update the target positions
       * but only if we have not interrupted a move-in-progress
       */
      if (!hBusy) {
        if (l     != l_RB) {
			PVPUT(BusyDisable,1); PVPUT (l, l_RB); PVPUT(BusyDisable,0);
		}
        if (r     != r_RB) {
			PVPUT(BusyDisable,1); PVPUT (r, r_RB); PVPUT(BusyDisable,0)
		}
        if (h0    != h0_RB) {
			PVPUT(BusyDisable,1); PVPUT (h0, h0_RB); PVPUT(BusyDisable,0)
		}
        if (width != width_RB) {
			PVPUT(BusyDisable,1); PVPUT (width, width_RB); PVPUT(BusyDisable,0)
		}
/*
		if ((l==l_RB) && (r == r_RB) && (h0 == h0_RB) && (width == width_RB) &&
			(t==t_RB) && (b == b_RB) && (v0 == v0_RB) && (height == height_RB)) {
			PVPUT(BusyOff, 0);
		}
*/
        delay (.02);
        efClear (l_mon);    efClear (r_mon);
        efClear (h0_mon);   efClear (width_mon);
        efClear (wait_H_input);
      }
    } state idle

    /*
     * new V axis readback data is available
     */
    when (efTest (new_V_RB)) { 
      efClear (new_V_RB);
      /*
       * update the target positions
       * but only if we have not interrupted a move-in-progress
       */
      if (!vBusy) {
        if (t     != t_RB) {
			PVPUT(BusyDisable,1); PVPUT (t, t_RB); PVPUT(BusyDisable,0);
		}
        if (b     != b_RB) {
			PVPUT(BusyDisable,1); PVPUT (b, b_RB); PVPUT(BusyDisable,0)
		}
        if (v0    != v0_RB) {
			PVPUT(BusyDisable,1); PVPUT (v0, v0_RB); PVPUT(BusyDisable,0)
		}
        if (height != height_RB) {
			PVPUT(BusyDisable,1); PVPUT (height, height_RB); PVPUT(BusyDisable,0)
		}
/*
		if ((l==l_RB) && (r == r_RB) && (h0 == h0_RB) && (width == width_RB) &&
			(t==t_RB) && (b == b_RB) && (v0 == v0_RB) && (height == height_RB)) {
			PVPUT(BusyOff, 0);
		}
*/
        delay (.02);
        efClear (t_mon);    efClear (b_mon);
        efClear (v0_mon);   efClear (height_mon);
        efClear (wait_V_input);
      }
    } state idle

    /*
     * signal that the KILL command has finished
     */
    when (stop && !efTest (wait_H_input) && !efTest (wait_V_input)) { 
      PVPUT (stop, STOP_IDLE);
      efClear (stop_mon);
    } state idle

    /*
     * things to do in the background ...
     */
    when ((hBusy || vBusy) && delay (MOVING_POLL_INTERVAL)) { 
      PVPUT (locate, 1);		/* signal for a position update */
    } state idle
    when (delay (IDLE_POLL_INTERVAL)) { 
      PVPUT (locate, 1);		/* signal for a position update */
    } state idle
    
  }

 /* 
 */
  state premove { 
    /*
     * ignore any non-movement monitors
     */
    when (efTest (l_mon) && (l == l_old))
		{efClear (l_mon);} state idle
    when (efTest (r_mon) && (r == r_old))
		{efClear (r_mon);} state idle
    when (efTest (t_mon) && (t == t_old))
		{efClear (t_mon);} state idle
    when (efTest (b_mon) && (b == b_old))
		{efClear (b_mon);} state idle
    when (efTest (h0_mon) && (h0 == h0_old))
		{efClear (h0_mon);} state idle
    when (efTest (v0_mon) && (v0 == v0_old))
		{efClear (v0_mon);} state idle
    when (efTest (width_mon) && (width == width_old))
		{efClear (width_mon);} state idle
    when (efTest (height_mon) && (height == height_old))
		{efClear (height_mon);} state idle
    when () { 
      /*
       * horizontal axis
       */
      if (efTest (l_mon) || efTest (r_mon) ||
          efTest (h0_mon) || efTest (width_mon)) { 
        /* find out which coordinate scheme changed, calc the other */
        if (efTest (l_mon) || efTest (r_mon)) {
          width = r + l;
          h0 = (r - l) / 2;
        } else {
          l = width / 2 - h0;
          r = width / 2 + h0;
        }
        /* check the limits */
        error = 0;
        if (!LIMIT_TEST (l_lo, l, l_hi)) { l = l_old; error++; }
        if (!LIMIT_TEST (r_lo, r, r_hi)) { r = r_old; error++; }
        if (!LIMIT_TEST (h0_lo, h0, h0_hi)) { h0 = h0_old; error++; }
        if (!LIMIT_TEST (width_lo, width, width_hi)) { 
           width = width_old; error++; }
        if (error) { 
          /* reset values if there was an error */
          if (efTest (l_mon)) pvPut (l);
          if (efTest (r_mon)) pvPut (r);
          if (efTest (h0_mon)) pvPut (h0);
          if (efTest (width_mon)) pvPut (width);
          error = ERROR_SOFT_LIMITS;
          PVPUTSTR (errMsg, "H soft limits exceeded");
          PVPUT(Busy, 0);
          delay (.02);
          efClear(l_mon);   efClear(r_mon);
          efClear(h0_mon);  efClear(width_mon);
        } else { 
          PVPUTSTR (errMsg, "no error");
          /* update the new coordinates */
          efSet (start_H_move);
          if (efTest (l_mon) || efTest (r_mon)) {
            pvPut (h0);  pvPut (width);
          } else {
            pvPut (l);  pvPut (r);
          }
          delay (.02);
          efClear(l_mon);   efClear(r_mon);
          efClear(h0_mon);  efClear(width_mon);
        }
        pvPut (error);
      }
      /*
       * vertical axis
       */
      if (efTest (t_mon) || efTest (b_mon) 
      ||  efTest (v0_mon) || efTest (height_mon)) { 
        /* find out which coordinate scheme changed, calc the other */
        if (efTest (t_mon) || efTest (b_mon)) {
          height = t + b;
          v0 = (t - b) / 2;
        } else {
          t = height / 2 + v0;
          b = height / 2 - v0;
        }
        /* check the limits */
        error = 0;
        if (!LIMIT_TEST (t_lo, t, t_hi)) { t = t_old; error++; }
        if (!LIMIT_TEST (b_lo, b, b_hi)) { b = b_old; error++; }
        if (!LIMIT_TEST (v0_lo, v0, v0_hi)) { v0 = v0_old; error++; }
        if (!LIMIT_TEST (height_lo, height, height_hi)) { 
           height = height_old; error++; }
        if (error) { 
          /* reset values if there was an error */
          if (efTest (t_mon)) pvPut (t);
          if (efTest (b_mon)) pvPut (b);
          if (efTest (v0_mon)) pvPut (v0);
          if (efTest (height_mon)) pvPut (height);
          error = ERROR_SOFT_LIMITS;
          PVPUTSTR (errMsg, "V soft limits exceeded");
          PVPUT(Busy, 0);
          delay (.02);
          efClear(t_mon);   efClear(b_mon);
          efClear(v0_mon);  efClear(height_mon);
        } else { 
          /* update the new coordinates */
          efSet (start_V_move);
          if (efTest (t_mon) || efTest (b_mon)) {
            pvPut (v0);  pvPut (height);
          } else {
            pvPut (t);  pvPut (b);
          }
          delay (.02);
          efClear(t_mon);   efClear(b_mon);
          efClear(v0_mon);  efClear(height_mon);
        }
        pvPut (error);
      }
    } state idle
  }

 /* 
 */
  state process_response {
    /*
     * various error messages
     */
    when ( (numWords == 2) && (!strcmp(word2, "ERROR;")) ) { 
      PVPUT (error, ERROR_UNKNOWN);
      sprintf (errMsg, "%s: unknown error", id);
      pvPut (errMsg);
      if (!strcmp(word1, movingMotorID)) PVPUT(BusyOff, 0);
    } state idle
    when ( (numWords > 2) && (!strcmp(word2, "ERROR")) ) { 
      sscanf (word3, "%d", &error);
      pvPut (error);
      PVPUTSTR (errMsg, hscErrors[error]);
      if (!strcmp(word1, movingMotorID)) PVPUT(BusyOff, 0);
    } state idle

    /*
     * controller is busy
     */
    when ( (numWords == 2) && (!strcmp(word2, "BUSY;")) ) { 
      /* axes are moving */
      if (!strcmp(id, hID)) { 
        PVPUT(hBusy,   MOTOR_BUSY);
      } else {
        PVPUT(vBusy,   MOTOR_BUSY);
      }
      if (error) { /* don't update unless previous error exists */
        PVPUT (error, 0);
        PVPUTSTR (errMsg, "no error");
      }
    } state idle

    /*
     * position update
     */
    when ( (numWords == 4) && (!strcmp(word4, "DONE;")) ) { 
      /* position info received */
      sscanf (word2, "%ld", &pos_a);   pos_ag = RAW_TO_DIAL(pos_a);
      sscanf (word3, "%ld", &pos_b);   pos_bg = RAW_TO_DIAL(pos_b);
      if (error) { /* don't update unless previous error exists */
        PVPUT (error, 0);
        PVPUTSTR (errMsg, "no error");
      }
	  if (!strcmp(id, movingMotorID)) {
		/* did we get where we want to go? */
/*printf("pos_a = %d, pos_b = %d\n", pos_a, pos_b);*/
		if (!strcmp(id, hID)) {
			l_raw = DIAL_TO_RAW(l);
			r_raw = DIAL_TO_RAW(r);
/*printf("horizontal; l=%d, r=%d\n", l_raw, r_raw);*/
			if (hOrient == ORIENT_LEFT_RIGHT) { 
				if ((pos_a == l_raw) && (pos_b == r_raw)) PVPUT(BusyOff, 0);
			} else { 
				if ((pos_a == r_raw) && (pos_b == l_raw)) PVPUT(BusyOff, 0);
			}
		}
		else {
			t_raw = DIAL_TO_RAW(t);
			b_raw = DIAL_TO_RAW(b);
			if (vOrient == ORIENT_TOP_BOTTOM) { 
				if ((pos_a == t_raw) && (pos_b == b_raw)) PVPUT(BusyOff, 0);
			} else { 
				if ((pos_a == b_raw) && (pos_b == t_raw)) PVPUT(BusyOff, 0);
			}
		}
	  }
    } state update_positions
    when ( (numWords == 5) && (!strcmp(word2, "OK")) ) { 
      /* position info received */
      sscanf (word3, "%ld", &pos_a);   pos_ag = RAW_TO_DIAL(pos_a);
      sscanf (word4, "%ld", &pos_b);   pos_bg = RAW_TO_DIAL(pos_b);
      if (error) { /* don't update unless previous error exists */
        PVPUT (error, 0);
        PVPUTSTR (errMsg, "no error");
      }
    } state update_positions

    when () {/* keep us from getting stuck here */} state idle
  }

 /* 
 */
  state update_positions {
    when () {
      /* horizontal */
      if (!strcmp(id, hID)) { 
        if (hOrient == ORIENT_LEFT_RIGHT) { 
          PVPUT (l_RB, pos_ag);
          PVPUT (r_RB, pos_bg);
        } else { 
          PVPUT (r_RB, pos_ag);
          PVPUT (l_RB, pos_bg);
        }
        PVPUT (width_RB, l_RB+r_RB);
        PVPUT (h0_RB, (r_RB-l_RB)/2);
        efSet (new_H_RB);	/* new readback data available */
        PVPUT(hBusy,   MOTOR_IDLE);
        /*
         * keep track of last known good values
         * to impose motor limits
         */
        l_old = l_RB;	r_old = r_RB;
        h0_old = h0_RB;	width_old = width_RB;
      }
      /* vertical */
      if (!strcmp(id, vID)) { 
        if (vOrient == ORIENT_TOP_BOTTOM) { 
          PVPUT (t_RB, pos_ag);
          PVPUT (b_RB, pos_bg);
        } else { 
          PVPUT (b_RB, pos_ag);
          PVPUT (t_RB, pos_bg);
        }
        PVPUT (height_RB, t_RB+b_RB);
        PVPUT (v0_RB, (t_RB-b_RB)/2);
        efSet (new_V_RB);	/* new readback data available */
        PVPUT(vBusy,   MOTOR_IDLE);
        /*
         * keep track of last known good values
         * to impose motor limits
         */
        t_old = t_RB;	b_old = b_RB;
        v0_old = v0_RB;	height_old = height_RB;
      }
    } state idle
  }

}
