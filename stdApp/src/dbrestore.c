/*
 * 10/29/96  tmm  v2.0 conversion to EPICS 3.13
 * 01/03/97  tmm  v2.1 use backup save file if save file can't be opened
 * 04/26/99  tmm  v2.2 check first character of restore file.  If not '#',
 *                then file is not trusted.
 * 11/24/99  tmm  v2.3 file-ok marker is now <END> and is placed at end of file.
 *                allow caller to choose whether boot-backup file is written,
 *                provide option of dated boot-backup files.
 * 02/27/02  tmm  v2.4 Added some features from Frank Lenkszus's (FRL) code:
 *                added path to request files
 *                added set_pass0_restoreFile( char *filename)
 *                added set_pass1_restoreFile( char *filename)
 *                a few more tweaks: 
 *                changed date-time stamp suffix to use FRL's fGetDateStr()
 *                don't write redundant backup files
 * 03/15/02  tmm  v2.5 check saveRestoreFilePath before using it.
 * 03/19/02  tmm  v2.6 initialize fname before using it.
 * 04/05/02  tmm  v2.7 Don't use copy for backup file.  It uses mode 640.
 * 07/14/03  tmm  v3.0 In addition to .sav and .savB, can save/restore <= 10
 *                sequenced files .sav0 -.sav9, which are written at preset
 *                intervals independent of the channel-list settings.
 * 08/13/03  tmm  v3.2 Merge bug fixes from 3.13 and 3.14 versions into
 *                something that will work under 3.13.
 * 08/19/03  tmm  v3.3 More error checking
 */
#define VERSION "3.3"

#include	<stdio.h>
#include	<stdlib.h>
#include	<sys/stat.h>
#include	<string.h>
#include	<ctype.h>
#include	<time.h>
#include	<usrLib.h>

#include	<dbDefs.h>
#include	<dbStaticLib.h>
#include	<initHooks.h>
#include 	"fGetDateStr.h"
#include	"save_restore.h"

#ifndef vxWorks
#define OK 0
#define ERROR -1
#endif

extern	DBBASE *pdbbase;

#ifdef NODEBUG
#define Debug(l,FMT,V) ;
#else
#define Debug0(l,FMT) {  if(l <= save_restoreDebug) \
			{ errlogPrintf("%s(%d):",__FILE__,__LINE__); \
			  errlogPrintf(FMT); } }
#define Debug(l,FMT,V) {  if(l <= save_restoreDebug) \
			{ errlogPrintf("%s(%d):",__FILE__,__LINE__); \
			  errlogPrintf(FMT,V); } }
#define Debug2(l,FMT,V,W) {  if(l <= save_restoreDebug) \
			{ errlogPrintf("%s(%d):",__FILE__,__LINE__); \
			  errlogPrintf(FMT,V,W); } }
#endif

#define myPrintErrno(errNo) {errlogPrintf("%s(%d): [0x%x]=",__FILE__,__LINE__,errNo); printErrno(errNo);}

struct restoreList restoreFileList = {0, 0, 
			{NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
			{0,0,0,0,0,0,0,0},
			{NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
			{NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL},
			{0,0,0,0,0,0,0,0},
			{NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL}
};

void dbrestoreShow()
{
	int i;
	printf("  '     filename     ' -  status  - 'message'\n");
	printf("  pass 0:\n");
	for (i=0; i<MAXRESTOREFILES; i++) {
		if (restoreFileList.pass0files[i]) {
			printf("  '%s' - %s - '%s'\n", restoreFileList.pass0files[i],
				SR_STATUS_STR[restoreFileList.pass0Status[i]],
				restoreFileList.pass0StatusStr[i]);
		}
	}
	printf("  pass 1:\n");
	for (i=0; i<MAXRESTOREFILES; i++) {
		if (restoreFileList.pass1files[i]) {
			printf("  '%s' - %s - '%s'\n", restoreFileList.pass1files[i],
				SR_STATUS_STR[restoreFileList.pass1Status[i]],
				restoreFileList.pass1StatusStr[i]);
		}
	}
}

int myFileCopy(char *source, char *dest)
{
	FILE 	*source_fd, *dest_fd;
	char	buffer[120], *bp;
	struct stat fileStat;
	int		chars_printed, size=0;

	Debug2(5, "myFileCopy: copying '%s' to '%s'\n", source, dest);

	if (stat(source, &fileStat) == 0) size = (int)fileStat.st_size;
	errno = 0;
	if ((source_fd = fopen(source,"r")) == NULL) {
		errlogPrintf("save_restore:myFileCopy: Can't open file '%s'\n", source);
		if (errno) myPrintErrno(errno);
		if (++save_restoreIoErrors > save_restoreRemountThreshold) 
			save_restoreNFSOK = 0;
		return(ERROR);
	}
	errno = 0;
	/* Note: frequently, the following fopen() will set errno to 
	 * S_nfsLib_NFSERR_NOENT even though it succeeds.  Probably this means
	 * a failed attempt was retried. (System calls never set errno to zero.)
	 */
	if ((dest_fd = fopen(dest,"w")) == NULL) {
		errlogPrintf("save_restore:myFileCopy: Can't open file '%s'\n", dest);
		if (errno) myPrintErrno(errno);
		fclose(source_fd);
		return(ERROR);
	}
	chars_printed = 0;
	while ((bp=fgets(buffer, 120, source_fd))) {
		errno = 0;
		chars_printed += fprintf(dest_fd, "%s", bp);
		if (errno) {myPrintErrno(errno); errno = 0;}
	}
	errno = 0;
	fclose(source_fd);
	if (errno) {myPrintErrno(errno); errno = 0;}
	fclose(dest_fd);
	if (errno) myPrintErrno(errno);
	if (size && (chars_printed != size)) {
		errlogPrintf("myFileCopy: size=%d, chars_printed=%d\n",
			size, chars_printed);
		return(ERROR);
	}
	return(OK);
}

/*
 * file_restore
 *
 * Read a list of channel names and values from an ASCII file,
 * and update database values before ioc_init is invoked.
 * Must use static database access routines.
 *
 */
int reboot_restore(char *filename, initHookState init_state)
{
	char		channel[80];
	char		bu_filename[259];
	char		fname[256] = "";
	char		buffer[120], *bp;
	char		input_line[120];
	char		datetime[32];
	char		*s;
	FILE		*inp_fd;
	char		c;
	unsigned short	i,j;
	DBENTRY		dbentry;
	DBENTRY		*pdbentry= &dbentry;
	long		status;
	char		*endp;
	int			n, write_backup, num_errors;
	long		*pStatusVal = 0;
	char		*statusStr = 0;

	printf("reboot_restore (v%s): entry\n", VERSION);
	/* initialize database access routines */
	if (!pdbbase) {
		errlogPrintf("No Database Loaded\n");
		return(OK);
	}

	dbInitEntry(pdbbase,pdbentry);

	/* what are we supposed to do here? */
	if (init_state >= INITHOOKafterInitDatabase) {
		for (i = 0; i < restoreFileList.pass1cnt; i++) {
			if (strcmp(filename, restoreFileList.pass1files[i]) == 0) {
				pStatusVal = &(restoreFileList.pass1Status[i]);
				statusStr = restoreFileList.pass1StatusStr[i];
			}
		}
	} else {
		for (i = 0; i < restoreFileList.pass0cnt; i++) {
			if (strcmp(filename, restoreFileList.pass0files[i]) == 0) {
				pStatusVal = &(restoreFileList.pass0Status[i]);
				statusStr = restoreFileList.pass0StatusStr[i];
			}
		}
	}

	if ((pStatusVal == 0) || (statusStr == 0)) {
		errlogPrintf("reboot_restore: Can't find filename '%s' in list.\n",
			filename);
	}

	/* open file */
	strncpy(fname, saveRestoreFilePath, sizeof(fname) -1);
	strncat(fname, filename, MAX(sizeof(fname) -1 - strlen(fname),0));
	printf("*** restoring from '%s' at initHookState %d ***\n",
		fname, (int)init_state);
	if ((inp_fd = fopen_and_check(fname, "r", &status)) == NULL) {
		errlogPrintf("save_restore: Can't open save file.");
		if (pStatusVal) *pStatusVal = SR_STATUS_FAIL;
		if (statusStr) strcpy(statusStr, "Can't open save file.");
		return(ERROR);
	}
	if (status) {
		if (pStatusVal) *pStatusVal = SR_STATUS_WARN;
		if (statusStr) strcpy(statusStr, "Bad .sav(B) files; used seq. backup");
	}

	(void)fgets(buffer, 120, inp_fd); /* discard header line */
	Debug(1, "reboot_restore: header line '%s'\n", buffer);
	/* restore from data file */
	num_errors = 0;
	while ((bp=fgets(buffer, 120, inp_fd))) {
		/* get PV_name, one space character, value */
		/* (value may be a string with leading whitespace; it may be */
		/* entirely whitespace; the number of spaces may be crucial; */
		/* it might also consist of zero characters) */
		n = sscanf(bp,"%s%c%[^\n]", channel, &c, input_line);
		if (n<3) *input_line = 0;
		if (isalpha(channel[0]) || isdigit(channel[0])) {
			/* add default field name */
			if (strchr(channel,'.') == 0)
				strcat(channel,".VAL");

			Debug2(10,"attempting to put '%s' to '%s'\n", input_line, channel);
			status = dbFindRecord(pdbentry,channel);
			if (status != 0) {
				errlogPrintf("dbFindRecord for '%s' failed\n", channel);
				errMessage(status,"");
				num_errors++;
			} else {
				if (!dbFoundField(pdbentry)) {
					errlogPrintf("save_restore: dbFindRecord did not find field '%s'\n",
						channel);
					num_errors++;
				}
				Debug(15,"field type '%s'\n",
					pamapdbfType[pdbentry->pflddes->field_type].strvalue);
				switch (pdbentry->pflddes->field_type) {
				case DBF_STRING:
				case DBF_CHAR:
				case DBF_UCHAR:
				case DBF_SHORT:
				case DBF_USHORT:
				case DBF_LONG:
				case DBF_ULONG:
				case DBF_FLOAT:
				case DBF_DOUBLE:
				case DBF_ENUM:
					status = dbPutString(pdbentry, input_line);
					if (status) num_errors++;
					Debug(15,"dbPutString() returns %d:", status);
					if (save_restoreDebug >= 15) errMessage(status, "");
					if ((s = dbVerify(pdbentry, input_line))) {
						errlogPrintf("save_restore: for '%s', dbVerify() says '%s'\n", channel, s);
						num_errors++;
					}
					break;

				case DBF_INLINK:
				case DBF_OUTLINK:
				case DBF_FWDLINK:
					/* Can't restore links after InitDatabase */
					if (init_state < INITHOOKafterInitDatabase) {
						status = dbPutString(pdbentry,input_line);
						if (status) num_errors++;
						Debug(15,"dbPutString() returns %d:",status);
						if (save_restoreDebug >= 15) errMessage(status,"");
						if ((s = dbVerify(pdbentry,input_line))) {
							errlogPrintf("save_restore: for '%s', dbVerify() says '%s'\n", channel, s);
							num_errors++;
						}
					}
					break;

				case DBF_MENU:
					n = (int)strtol(input_line,&endp,0);
					status = dbPutMenuIndex(pdbentry, n);
					if (status) num_errors++;
					Debug(15,"dbPutMenuIndex() returns %d:",status);
					if (save_restoreDebug >= 15) errMessage(status,"");
					break;

				default:
					status = -1;
					Debug(10,"field type not handled\n", 0);
					num_errors++;
					break;
				}
				if (status != 0) {
					errlogPrintf("dbPutString/dbPutMenuIndex of '%s' for '%s' failed\n",
					  input_line,channel);
					errMessage(status,"");
				}
				Debug(15,"dbGetString() returns '%s'\n",dbGetString(pdbentry));
			}
		} else if (channel[0] == '!') {
			for (i = 0; input_line[i] == ' '; i++);	/*skip blanks */
			for (j = 0; (input_line[i] != ' ') && (input_line[i] != 0); i++,j++)
				channel[j] = input_line[i];
			channel[j] = 0;
			errlogPrintf("%s channel(s) not connected / fetch failed\n",channel);
			if (pStatusVal) *pStatusVal = SR_STATUS_WARN;
			if (statusStr) strcpy(statusStr, ".sav file contained an error message");
			if (!save_restoreIncompleteSetsOk) {
				errlogPrintf("aborting restore\n");
				fclose(inp_fd);
				dbFinishEntry(pdbentry);
				if (pStatusVal) *pStatusVal = SR_STATUS_FAIL;
				if (statusStr) strcpy(statusStr, "restore aborted");
				return(ERROR);
			}
		} else if (channel[0] == '<') {
			/* end of file */
			break;
		}
	}
	fclose(inp_fd);
	dbFinishEntry(pdbentry);

	/* If this is the second pass for a restore file, don't write backup file again.*/
	write_backup = 1;
	if (init_state >= INITHOOKafterInitDatabase) {
		for(i = 0; i < restoreFileList.pass0cnt; i++) {
			if (strcmp(filename, restoreFileList.pass0files[i]) == 0) {
				write_backup = 0;
				break;
			}
		}
	}

	if (write_backup) {
		/* write  backup file*/
		strcpy(bu_filename,fname);
		if (save_restoreDatedBackupFiles && (fGetDateStr(datetime) == 0)) {
			strcat(bu_filename, "_");
			strcat(bu_filename, datetime);
		} else {
			strcat(bu_filename, ".bu");
		}
		Debug(1, "save_restore: writing boot-backup file '%s'.\n", bu_filename);
		status = (long)myFileCopy(fname,bu_filename);
		if (status) {
			errlogPrintf("save_restore: Can't write backup file.\n");
			if (pStatusVal) *pStatusVal = SR_STATUS_WARN;
			if (statusStr) strcpy(statusStr, "Can't write backup file");
			return(OK);
		}
	}

	/* Record status */
	if (pStatusVal && statusStr) {
		if (*pStatusVal != 0) {
			/* Status and message have already been recorded */
			;
		} else if (num_errors != 0) {
			sprintf(statusStr, "%d %s", num_errors, num_errors==1?"PV error":"PV errors");
			*pStatusVal = SR_STATUS_WARN;
		} else {
			strcpy(statusStr, "No errors");
			*pStatusVal = SR_STATUS_OK;
		}
	}

	return(OK);
}

int set_pass0_restoreFile( char *filename)
{
	char *cp;

	if (restoreFileList.pass0cnt >= MAXRESTOREFILES) {
		errlogPrintf("set_pass0_restoreFile: MAXFILE count exceeded\n");
		return(ERROR);
	}
	cp = (char *)calloc(strlen(filename) + 4,sizeof(char));
	restoreFileList.pass0files[restoreFileList.pass0cnt] = cp;
	if (cp == NULL) {
		errlogPrintf("set_pass0_restoreFile: calloc failed\n");
		restoreFileList.pass0StatusStr[restoreFileList.pass0cnt] = (char *)0;
		return(ERROR);
	}
	strcpy(cp, filename);
	cp = (char *)calloc(40, 1);
	restoreFileList.pass0StatusStr[restoreFileList.pass0cnt] = cp;
	strcpy(cp, "Unknown, probably failed");
	restoreFileList.pass0cnt++;
	return(OK);
}

int set_pass1_restoreFile( char *filename)
{
	char *cp;

	if (restoreFileList.pass1cnt >= MAXRESTOREFILES) {
		errlogPrintf("set_pass1_restoreFile: MAXFILE count exceeded\n");
		return(ERROR);
	}
	cp = (char *)calloc(strlen(filename) + 4,sizeof(char));
	restoreFileList.pass1files[restoreFileList.pass1cnt] = cp;
	if (cp == NULL) {
		errlogPrintf("set_pass1_restoreFile: calloc failed\n");
		restoreFileList.pass1StatusStr[restoreFileList.pass1cnt] = (char *)0;
		return(ERROR);
	}
	strcpy(cp, filename);
	cp = (char *)calloc(40, 1);
	restoreFileList.pass1StatusStr[restoreFileList.pass1cnt] = cp;
	strcpy(cp, "Unknown, probably failed");
	restoreFileList.pass1cnt++;
	return(OK);
}

FILE *fopen_and_check(const char *fname, const char *mode, long *status)
{
	FILE *inp_fd = NULL;
	char tmpstr[PATH_SIZE+50];
	char file[256];
	char datetime[32];
	int i, backup_sequence_num;
	struct stat fileStat;
	char *p;
	time_t currTime;
	double dTime, min_dTime;

	*status = 0;	/* presume success */
	strncpy(file, fname, 255);
	if ((inp_fd = fopen(file, "r")) == NULL) {
		errlogPrintf("save_restore: Can't open file '%s'.\n", file);
	} else {
		/* check out "successfully written" marker */
		if ((fseek(inp_fd, -6, SEEK_END)) ||
				(fgets(tmpstr, 6, inp_fd) == 0) ||
				(strncmp(tmpstr, "<END>", 5) != 0)) {
			fclose(inp_fd);
			/* File doesn't look complete, make a copy of it */
			errlogPrintf("save_restore: File '%s' is not trusted.\n",
					file);
			strcpy(tmpstr, file);
			strcat(tmpstr, "_RBAD_");
			if (save_restoreDatedBackupFiles) {
				fGetDateStr(datetime);
				strcat(tmpstr, datetime);
			}
			(void)myFileCopy(file, tmpstr);
		} else {
			fseek(inp_fd, 0, SEEK_SET); /* file is ok.  go to beginning */
			return(inp_fd);
		}
	}

	/* Still here?  Try the backup file. */
	strncat(file, "B", 1);
	errlogPrintf("save_restore: Trying backup file '%s'\n", file);
	if ((inp_fd = fopen(file, "r")) == NULL) {
		errlogPrintf("save_restore: Can't open file '%s'\n", file);
	} else {
		if ((fseek(inp_fd, -6, SEEK_END)) ||
				(fgets(tmpstr, 6, inp_fd) == 0) ||
				(strncmp(tmpstr, "<END>", 5) != 0)) {
			fclose(inp_fd);
			errlogPrintf("save_restore: File '%s' is not trusted.\n",
				file);
			fGetDateStr(datetime);
			strcpy(tmpstr, file);
			strcat(tmpstr, "_RBAD_");
			strcat(tmpstr, datetime);
			(void)myFileCopy(file, tmpstr);
		} else {
			fseek(inp_fd, 0, SEEK_SET); /* file is ok.  go to beginning */
			return(inp_fd);
		}
	}

	/* Still haven't found a good file?  Try the sequenced backups */
	*status = 1;
	strcpy(file, fname);
	backup_sequence_num = -1;
	p = &file[strlen(file)];
	currTime = time(NULL);
	min_dTime = 1.e9;
	for (i=0; i<save_restoreNumSeqFiles; i++) {
		sprintf(p, "%1d", i);
		if (stat(file, &fileStat) == 0) {
			dTime = difftime(currTime, fileStat.st_mtime);
			if (save_restoreDebug >= 5) {
				errlogPrintf("'%s' modified at %s\n", file,
					ctime(&fileStat.st_mtime));
				errlogPrintf("'%s' is %lf seconds old\n", file, dTime);
			}
			if (dTime < min_dTime) {
				min_dTime = dTime;
				backup_sequence_num = i;
			}
		}
	}

	/* try the backup file */
	for (i=0; i<save_restoreNumSeqFiles; i++) {
		sprintf(p, "%1d", backup_sequence_num);
		errlogPrintf("save_restore: Trying backup file '%s'\n", file);
		if ((inp_fd = fopen(file, "r")) == NULL) {
			errlogPrintf("save_restore: Can't open file '%s'\n", file);
		} else {
			if ((fseek(inp_fd, -6, SEEK_END)) ||
					(fgets(tmpstr, 6, inp_fd) == 0) ||
					(strncmp(tmpstr, "<END>", 5) != 0)) {
				fclose(inp_fd);
				errlogPrintf("save_restore: File '%s' is not trusted.\n",
					file);
				fGetDateStr(datetime);
				strcpy(tmpstr, file);
				strcat(tmpstr, "_RBAD_");
				strcat(tmpstr, datetime);
				(void)myFileCopy(file, tmpstr);
			} else {
				fseek(inp_fd, 0, SEEK_SET); /* file is ok.  go to beginning */
				return(inp_fd);
			}
		}
		if (++backup_sequence_num >= save_restoreNumSeqFiles)
			backup_sequence_num = 0;
	}

	errlogPrintf("save_restore: Can't find a file to restore from...");
	errlogPrintf("save_restore: ...last tried '%s'. I quit.\n", file);
	errlogPrintf("save_restore: **********************************\n\n");
	return(0);
}
