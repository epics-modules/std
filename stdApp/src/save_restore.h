/* save_restore.h */

/*	
 * 	Original Author: Frank Lenkszus
 *	Current Author:  Frank Lenkszus
 *	Date:		7/10/97
 *
 */

#define         MAX(a,b)   ((a)>(b)?(a):(b))
#define         MIN(a,b)   ((a)<(b)?(a):(b))
#define         MAXRESTOREFILES 8

#define SR_STATUS_OK		3
#define SR_STATUS_SEQ_WARN	2
#define SR_STATUS_WARN		1
#define SR_STATUS_FAIL		0

static char SR_STATUS_STR[4][8] =
	{"Failure ", "Warning", "Warning", "   Ok   "};

struct restoreList {
        int pass0cnt;
        int pass1cnt;
        char *pass0files[MAXRESTOREFILES];
		long pass0Status[MAXRESTOREFILES];
		char *pass0StatusStr[MAXRESTOREFILES];
        char *pass1files[MAXRESTOREFILES];
		long pass1Status[MAXRESTOREFILES];
		char *pass1StatusStr[MAXRESTOREFILES];
};

FILE *fopen_and_check(const char *file, const char *mode, long *status);

#define PATH_SIZE 255		/* max size of the complete path to one file */

extern int save_restoreIncompleteSetsOk;
extern char saveRestoreFilePath[];              /* path to save files */
extern int save_restoreNumSeqFiles;
extern int save_restoreDebug;
extern int save_restoreDatedBackupFiles;
extern struct restoreList restoreFileList;
int myFileCopy(char *source, char *dest);
extern void dbrestoreShow(void);

extern int	save_restoreNFSOK;
extern int	save_restoreIoErrors;
extern int	save_restoreRemountThreshold;
