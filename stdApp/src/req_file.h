/************************************************************************/
/*
 *      Original Author: Eric Boucher
 *
 * Modification Log:
 * .01 01-09-901 tmm  Included Ron Sluiter's change for ppc
 */

#ifndef __REQ_FILE_H__
#define __REQ_FILE_H__

#include <stdioLib.h>
#include <ioLib.h>


/* #include <stdio.h> */
/* #define MAX_FILENAME_LENGTH  160 */


/************************************************************************/
/* TYPES								*/

/*----------------------------------------------------------------------*/
/* macro structure							*/
typedef struct macro {
  char  name[21];
  char  value[41];
  struct macro* nxt;
} MACRO;

/*----------------------------------------------------------------------*/
/* req file structure							*/
typedef struct req_file {
  FILE*  fd;
  MACRO* list_macros;
  char   file_name[MAX_FILENAME_LENGTH];
  char   sect_name[41];
  long   sect_start;
  long   sect_end;
  signed char cur;	/* Must be signed for PPC compatibility and EOF comparisons. */
} REQ_FILE;


/************************************************************************/
/* MACROS								*/

/*----------------------------------------------------------------------*/
/* TRUE if the req file reaches the end of a section or the end of file */
#define eos(rf)	       (rf->cur==EOF)

/*----------------------------------------------------------------------*/
/* TRUE if c is a valid identifier char.                                */
#define isidchar(c) (isalnum(c) || (c=='_') || (c=='.') || (c==':'))

/*----------------------------------------------------------------------*/
/* The current char of the req file.                                    */
#define current(rf)    (rf->cur)


/************************************************************************/
/* PROTOTYPES								*/


/*----------------------------------------------------------------------*/
/* Open a req file.                                                     */
/* The current section is set to ALL, ending at the end of the file.	*/
/* name: the name of the file.						*/
/* macro: a string containing the macros associated with the file.	*/
/* return a pointer to a REQ_FILE structure if successful.		*/
/*	  NULL otherwise.						*/
REQ_FILE* req_open_file(char* name, char* macro);

/*----------------------------------------------------------------------*/
/* Close a req file and free the macros associated with it.             */
/* rf: the req file.		        				*/
void req_close_file(REQ_FILE* rf);

/*----------------------------------------------------------------------*/
/* Print all macros associated with a req file.                         */
/* rf: the req file.		        				*/
void req_infoMacro(REQ_FILE* rf);

/*----------------------------------------------------------------------*/
/* Print info about the current section of a req file.			*/
/* rf: the req file.		        				*/
void req_infoSection(REQ_FILE* rf);

/*----------------------------------------------------------------------*/
/* Locate the file cursor to a  pecifique section of the req file.      */
/* rf: the req file.							*/
/* name: the name of the section					*/
/* return 0 if successful.						*/
/*       -1 otherwise.							*/
int req_gotoSection(REQ_FILE* rf, char* section);

/*----------------------------------------------------------------------*/
/* Go to the next section in the req file.                              */
/* rf: the req file.          						*/
/* return a pointer to the name of the new section if successful	*/
/*	  NULL otherwise.						*/
const char* req_nextSection(REQ_FILE* rf);

/*----------------------------------------------------------------------*/
/* Get the name of the current section of a req file.                   */
/* rf: the req file.          						*/
/* return a pointer to the name of the section.               		*/
const char* req_getSectName(REQ_FILE* rf);

/*----------------------------------------------------------------------*/
/* Rewind a req file, ei set the current section to ALL and set the     */
/* current char to the first char of the file.				*/
/* rf: the req file.          						*/
void req_rewindFile(REQ_FILE* rf);

/*----------------------------------------------------------------------*/
/* Rewind a section, ei set the current char to the first char of the   */
/* current section.							*/
/* rf: the req file.          						*/
void req_rewindSect(REQ_FILE* rf);


/*----------------------------------------------------------------------*/
/* Skip a line.                                                         */
/* rf: the req file.          						*/
/* return the value of the first char of the new line or EOS if the	*/
/* end of section or end of file is reached.				*/
int req_skipLine(REQ_FILE* rf);

/*----------------------------------------------------------------------*/
/* Skip space char as defined by isspace()				*/
/* rf: the req file.          						*/
/* return the value of the first char not a space or EOS if the		*/
/* end of section or end of file is reached.				*/
int req_skipSpace(REQ_FILE* rf);

/*----------------------------------------------------------------------*/
/* Skip comment, ie all the lines that follow a # sign.			*/
/* rf: the req file.          						*/
/* return the value of the first char after the comments or EOS if the	*/
/* end of section or end of file is reached.				*/
int req_skipComment(REQ_FILE* rf);

/*----------------------------------------------------------------------*/
/* Read the next char of a req file.                    		*/
/* rf: the req file.          						*/
/* return the value of the next char or EOS if the			*/
/* end of section or end of file is reached.				*/
int req_readChar(REQ_FILE* rf);

/*----------------------------------------------------------------------*/
/* Read an identifier from a req file.                  		*/
/* an identifier is composed of [a-zA-Z0-9_.:]				*/
/* rf: the req file.          						*/
/* dest: a pointer to where the id has to be copied.			*/
/* max: the max number of char to be copied.				*/
/* return the number of char copied to dest.				*/
int req_readId(REQ_FILE*rf, char* dest, int max);

/*----------------------------------------------------------------------*/
/* Read a "macroed" identifier from a req file.                  	*/
/* a "macroed" identifier is an identifier in which $(xx) are subtituted*/
/* by the value of the corresponding macro value.			*/
/* rf: the req file.          						*/
/* dest: a pointer to where the id has to be copied.			*/
/* max: the max number of char to be copied.				*/
/* return the number of char copied to dest.				*/
int req_readMacId(REQ_FILE* rf, char* dest, int max);

/*----------------------------------------------------------------------*/
/* Read a quoted string from a req file.                  		*/
/* rf: the req file.          						*/
/* dest: a pointer to where the string has to be copied.		*/
/* max: the max number of char to be copied.				*/
/* return the number of char copied to dest.				*/
int req_readString(REQ_FILE*rf, char* dest, int max);


#endif

