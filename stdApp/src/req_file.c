/************************************************************************/
/*
 *      Original Author: Eric Boucher
 *      Date:            04-09-98
 *
 *	Experimental Physics and Industrial Control System (EPICS)
 *
 *      Beamline Controls & Data Acquisition Group
 *      Experimental Facilities Division
 *      Advanced Photon Source
 *      Argonne National Laboratory
*
 * Modification Log:
 * .01 04-15-98  erb  Initial development
 */


#include <stdlib.h>
#include <ctype.h>

#include "req_file.h"


#define LOCAL static

LOCAL MACRO* initMacros(char* macro);
LOCAL MACRO* searchMacro(REQ_FILE* rf, char* name);
LOCAL MACRO* readMacro(REQ_FILE* rf);
#define END(m) (*m=='\0')

LOCAL MACRO* initMacros(char* macro)
{
  MACRO* head;
  MACRO* cur;
  int    i;
  char   mc_name[9];
  char   mc_value[20];

  head= NULL;

  if(!macro) return NULL;
  if(END(macro)) return NULL;

  do {
    while(!END(macro) && (*macro==' ')) macro++;
    if(END(macro)) return head;

    i= 0;
    while(!END(macro) && (*macro!=' ') && (*macro!='=') && (i<8))
      mc_name[i++]= *(macro++);
    if(END(macro)) return head;
    while(!END(macro) && (*macro!='=')) macro++;
    mc_name[i]= '\0';
    macro++;

    while(!END(macro) && (*macro==' ')) macro++;
    if(END(macro)) return head;
    
    i= 0;
    while(!END(macro) && (*macro!=' ')  && (*macro!=',') && (i<19))
      mc_value[i++]= *(macro++);
    mc_value[i]= '\0';
    
    cur= (MACRO*) malloc(sizeof(MACRO));
    if(!cur) return head;
    
    strcpy(cur->name, mc_name);
    strcpy(cur->value, mc_value);
    cur->nxt= head;
    head= cur;

    while(!END(macro) && (*macro!=',')) macro++; 
  } while(!END(macro) && (*(macro++)==','));
  return head;
}

LOCAL MACRO* searchMacro(REQ_FILE* rf, char* name)
{
  MACRO* cur;

  cur= rf->list_macros;
  while(cur) {
    if(strcmp(cur->name, name)==0) return cur;
    cur= cur->nxt;
  }
  return NULL;
}

LOCAL MACRO* readMacro(REQ_FILE* rf)
{
  char macName[9];
  int i;
  
  req_skipSpace(rf);

  if(current(rf)!='$') return NULL;
  req_readChar(rf);
  if(current(rf)!='(') return NULL;
  req_readChar(rf);

  i= 0;
  while(!eos(rf) && (current(rf)!=')') && (i<8)) {
    macName[i++]= current(rf);
    req_readChar(rf);
  }
  macName[i]= '\0';
  if(current(rf)!=')') return NULL;
  req_readChar(rf);

  return searchMacro(rf, macName);
}


/*----------------------------------------------------------------------*/
/* Open a req file.                                                     */
/* The current section is set to ALL, ending at the end of the file.	*/
/* name: the name of the file.						*/
/* macro: a string containing the macros associated with the file.	*/
/* return a pointer to a REQ_FILE structure if successful.		*/
/*	  NULL otherwise.						*/
REQ_FILE* req_open_file(char* name, char* macro)
{
  FILE*     fd;
  REQ_FILE* rf;

  fd= fopen(name, "r");
  if(!fd) return NULL;

  rf= (REQ_FILE*) malloc(sizeof(REQ_FILE));
  if(!rf) return NULL;

  rf->fd= fd;
  rf->list_macros= initMacros(macro);
  strcpy(rf->file_name, name);
  req_rewindFile(rf);

  return rf;
}

/*----------------------------------------------------------------------*/
/* Close a req file and free the macros associated with it.             */
/* rf: the req file.		        				*/
void req_close_file(REQ_FILE* rf)
{
  MACRO* mac;

  fclose(rf->fd);
  
  while(rf->list_macros){
    mac= rf->list_macros;
    rf->list_macros= mac->nxt;
    free(mac);
  }
  free(rf);
}

/*----------------------------------------------------------------------*/
/* Print all macros associated with a req file.                         */
/* rf: the req file.		        				*/
void req_infoMacro(REQ_FILE* rf)
{
  MACRO* cur;

  printf("file name: %s\n", rf->file_name);
  printf("macros:\n");
  cur= rf->list_macros;
  while(cur) {
    printf(" $(%s)= %s\n", cur->name, cur->value);
    cur= cur->nxt;
  }
}

/*----------------------------------------------------------------------*/
/* Print info about the current section of a req file.			*/
/* rf: the req file.		        				*/
void req_infoSection(REQ_FILE* rf)
{
  printf("sect_name   = %s\n", rf->sect_name);
  printf("sect_start  = %d\n", rf->sect_start);
  printf("sect_end    = %d\n", rf->sect_end);
  printf("current char= %c\n", rf->cur);
  printf("current pos = %d\n\n", ftell(rf->fd));
}


/*----------------------------------------------------------------------*/
/* Locate the file cursor to a  pecifique section of the req file.      */
/* rf: the req file.							*/
/* name: the name of the section					*/
/* return 0 if successful.						*/
/*       -1 otherwise.							*/
int req_gotoSection(REQ_FILE* rf, char* section)
{
  char buff[21];
  int i;

  req_rewindFile(rf);

  while(1) {
    while(!eos(rf) && (current(rf)!='[')) req_skipLine(rf);
    if(eos(rf)) return -1;

    /* skip '[' */
    req_readChar(rf);
    i= 0;
    while(!eos(rf) && (current(rf)!=']') && (i<20)) {
      buff[i++]= current(rf);
      req_readChar(rf);
    }
    buff[i]= '\0';
    /* we've not find a closing sq bracket */
    if(current(rf)!=']') return -1;

    /* skip the rest of the line */
    req_skipLine(rf);

    /* have we find the correct section */
    if(strcmp(buff, section)==0) break;

    if(eos(rf)) return -1;
  }
    
  strcpy(rf->sect_name, section);
  rf->sect_start= ftell(rf->fd)-1;
  
  /* search the end of the section */
  while(!eos(rf) && (current(rf)!='[')) req_skipLine(rf);

  rf->sect_end= ftell(rf->fd)-1;

  fseek(rf->fd, rf->sect_start, SEEK_SET);
  req_readChar(rf);
  req_skipComment(rf);

  return 0;
}

/*----------------------------------------------------------------------*/
/* Go to the next section in the req file.                              */
/* rf: the req file.          						*/
/* return a pointer to the name of the new section if successful	*/
/*	  NULL otherwise.						*/
const char* req_nextSection(REQ_FILE* rf)
{
  int i;

  if(rf->sect_end==-1) return NULL;

  fseek(rf->fd, rf->sect_end, SEEK_SET);
  rf->sect_end= -1;

  req_readChar(rf);
  if(current(rf)!='[') return NULL;

  req_readChar(rf);
  i= 0;
  while(!eos(rf) && (current(rf)!=']') && (i<20)) {
    rf->sect_name[i++]= current(rf);
    req_readChar(rf);
  }
  rf->sect_name[i]='\0';
  if(current(rf)!=']') return NULL;

  req_skipLine(rf);

  rf->sect_start= ftell(rf->fd)-1;

  while(!eos(rf) && (current(rf)!='[')) req_skipLine(rf);
  rf->sect_end= ftell(rf->fd)-1;

  fseek(rf->fd, rf->sect_start, SEEK_SET);
  req_readChar(rf);
  req_skipComment(rf);

  return rf->sect_name;
}

/*----------------------------------------------------------------------*/
/* Get the name of the current section of a req file.                   */
/* rf: the req file.          						*/
/* return a pointer to the name of the section.               		*/
const char* req_getSectName(REQ_FILE* rf)
{
  return rf->sect_name;
}


/*----------------------------------------------------------------------*/
/* Rewind a req file, ei set the current section to ALL and set the     */
/* current char to the first char of the file.				*/
/* rf: the req file.          						*/
void req_rewindFile(REQ_FILE* rf)
{
  fseek(rf->fd, 0, SEEK_SET); 
  rf->sect_start= 0;
  rf->sect_end= -1;

  strcpy(rf->sect_name, "ALL");
 
  req_readChar(rf);
  req_skipComment(rf);
}

/*----------------------------------------------------------------------*/
/* Rewind a section, ei set the current char to the first char of the   */
/* current section.							*/
/* rf: the req file.          						*/
void req_rewindSect(REQ_FILE* rf)
{
  fseek(rf->fd, rf->sect_start, SEEK_SET);
  req_readChar(rf);
  req_skipComment(rf);
}
    

/*----------------------------------------------------------------------*/
/* Skip a line.                                                         */
/* rf: the req file.          						*/
/* return the value of the first char of the new line or EOS if the	*/
/* end of section or end of file is reached.				*/
int req_skipLine(REQ_FILE* rf)
{
  while(!eos(rf) && (current(rf)!='\n')) req_readChar(rf);
  return req_skipComment(rf);
}

/*----------------------------------------------------------------------*/
/* Skip space char as defined by isspace()				*/
/* rf: the req file.          						*/
/* return the value of the first char not a space or EOS if the		*/
/* end of section or end of file is reached.				*/
int req_skipSpace(REQ_FILE* rf)
{
  while(isspace(current(rf))) req_readChar(rf);
  return req_skipComment(rf);
}

/*----------------------------------------------------------------------*/
/* Skip comment, ie all the lines that follow a # sign.			*/
/* rf: the req file.          						*/
/* return the value of the first char after the comments or EOS if the	*/
/* end of section or end of file is reached.				*/
int req_skipComment(REQ_FILE* rf)
{
  while(isspace(current(rf))) req_readChar(rf);
  while(current(rf)=='#') {
    while(!eos(rf) && (current(rf)!='\n')) req_readChar(rf);
    while(isspace(current(rf))) req_readChar(rf);
  } 
  return current(rf);
}  


/*----------------------------------------------------------------------*/
/* Read the next char of a req file.                    		*/
/* rf: the req file.          						*/
/* return the value of the next char or EOS if the			*/
/* end of section or end of file is reached.				*/
int req_readChar(REQ_FILE* rf)
{
  if((rf->sect_end!=-1) && (ftell(rf->fd)==rf->sect_end)) rf->cur=EOF;
  else rf->cur= fgetc(rf->fd);
  return rf->cur;
}

/*----------------------------------------------------------------------*/
/* Read an identifier from a req file.                  		*/
/* an identifier is composed of [a-zA-Z0-9_.:]				*/
/* rf: the req file.          						*/
/* dest: a pointer to where the id has to be copied.			*/
/* max: the max number of char to be copied.				*/
/* return the number of char copied to dest.				*/
int req_readId(REQ_FILE*rf, char* dest, int max)
{
  int    i;

  *dest= '\0';
  max= max-1;
  i= 0;

  req_skipComment(rf);
  while(!eos(rf) && isidchar(current(rf)) && (i<max)) {
    *(dest++)= current(rf);
    i++;
    req_readChar(rf);
  }
  *dest= '\0';

  req_skipComment(rf);
  return i;
}



/*----------------------------------------------------------------------*/
/* Read a "macroed" identifier from a req file.                  	*/
/* a "macroed" identifier is an identifier in which $(xx) are subtituted*/
/* by the value of the corresponding macro value.			*/
/* rf: the req file.          						*/
/* dest: a pointer to where the id has to be copied.			*/
/* max: the max number of char to be copied.				*/
/* return the number of char copied to dest.				*/
int req_readMacId(REQ_FILE* rf, char* dest, int max)
{
  MACRO* mac;
  char*  mc_value;
  int    i;

  *dest= '\0';
  max= max-1;
  i= 0;

  req_skipComment(rf);
  while(!eos(rf) && (current(rf)=='$' || isidchar(current(rf))) && (i<max)) {
    
    if(current(rf)=='$') {
      mac= readMacro(rf);
      if(mac==NULL) {
	*dest= '\0';
	return -1;
      }
      mc_value= mac->value;
      while((i<max) && !END(mc_value)) {
	*(dest++)= *(mc_value++);
	i++;
      }
    } else {
      *(dest++)= current(rf);
      i++;
      req_readChar(rf);
    }
  }
  *dest= '\0';
  req_skipComment(rf);
  return i;
}

/*----------------------------------------------------------------------*/
/* Read a quoted string from a req file.                  		*/
/* rf: the req file.          						*/
/* dest: a pointer to where the string has to be copied.		*/
/* max: the max number of char to be copied.				*/
/* return the number of char copied to dest.				*/
int req_readString(REQ_FILE*rf, char* dest, int max)
{
  int    i;

  *dest= '\0';
  max= max-1;
  i= 0;

  req_skipComment(rf);
  if(current(rf)!='"') return 0;

  req_readChar(rf);
  while(!eos(rf) && (current(rf)!='"') && (i<max)) {
    *(dest++)=current(rf);
    i++;
    req_readChar(rf);
  }
  *dest='\0';
  if(current(rf)=='"') req_readChar(rf);

  req_skipComment(rf);
  return i;
}



/* int main() { */
/*   REQ_FILE* rf; */
/*   char      buff1[40], buff2[40]; */
  
/*   rf= req_open_file("saveData.req", "P=rix:"); */
  
/* get the IOC prefix							*/
/*   if(req_gotoSection(rf, "prefix")==0) { */
/*     req_readMacId(rf, buff1, 9); */
/*   } */
/*   printf("prefix: %s\n", buff1); */

/*   Connect to saveData_active						*/
/*   if(req_gotoSection(rf, "active")!=0) { */
/*     printf("section [active] not found\n"); */
/*     return -1; */
/*   } */
/*   if(req_readMacId(rf, buff1, 40)==0) { */
/*     printf("active pv name not defined\n"); */
/*     return -1; */
/*   } */
/*   printf("active: %s\n", buff1); */

/*   Connect to saveData_message					*/
/*   if(req_gotoSection(rf, "message")!=0) { */
/*     printf("section [message] not found\n"); */
/*   } else { */
/*     if(req_readMacId(rf, buff1, 40)==0) { */
/*       printf("message pv name not defined\n"); */
/*     } else { */
/*       printf("message: %s\n", buff1); */
/*     } */
/*   } */

/*   Connect to saveData_fullPathName					*/
/*   if(req_gotoSection(rf, "fullPathName")!=0) { */
/*     printf("section [fullPathName] not found\n"); */
/*   } else { */
/*     if(req_readMacId(rf, buff1, 40)==0) { */
/*       printf("fullPathName pv name not defined\n"); */
/*     } else { */
/*       printf("fullpath: %s\n", buff1); */
/*     } */
/*   } */

/*   Connect to saveData_scanNumber					*/
/*   if(req_gotoSection(rf, "counter")!=0) { */
/*     printf("section [counter] not found\n"); */
/*     return -1; */
/*   } */
/*   if(req_readMacId(rf, buff1, 40)==0) { */
/*     printf("counter pv name not defined\n"); */
/*     return -1; */
/*   } */
/*   printf("counter: %s\n", buff1); */
  
/*   Connect to saveData_fileSystem					*/
/*   if(req_gotoSection(rf, "fileSystem")!=0) { */
/*     printf("section [fileSystem] not found\n"); */
/*     return -1; */
/*   } */
/*   if(req_readMacId(rf, buff1, 40)==0) { */
/*     printf("fileSystem pv name not defined\n"); */
/*     return -1; */
/*   } */
/*   printf("filesystem: %s\n", buff1); */

/*   Connect to saveData_baseName					*/
/*   if(req_gotoSection(rf, "baseName")!=0) { */
/*     printf("section [baseName] not found\n"); */
/*     return -1; */
/*   } */
/*   if(req_readMacId(rf, buff1, 40)==0) { */
/*     printf("baseName pv name not defined\n"); */
/*     return -1; */
/*   } */
/*   printf("baseName: %s\n", buff1); */

/*   Connect all scan records.      					*/
/*   if(req_gotoSection(rf, "scanRecord")==0) { */
/*     while(!eos(rf)) { */
/*       req_readMacId(rf, buff1, 40); */
/*       if(current(rf)==',') { */
/* 	req_readChar(rf); */
/* 	req_readMacId(rf, buff2, 40); */
/*       } else { */
/* 	buff2[0]= '\0'; */
/*       } */
/*         printf("new scan: %s->%s\n", buff1, buff2); */
/*     } */
/*   } */

/*   Connect all extra pvnames      					*/
/*   if(req_gotoSection(rf, "extraPV")==0) { */
/*     while(!eos(rf)) { */
/*       req_readMacId(rf, buff1, 40); */
/*       if(current(rf)=='"') { */
/* 	req_readString(rf, buff2, 40); */
/*       } else { */
/* 	buff2[0]= '\0'; */
/*       } */
/*       printf("extraPV: %s[%s]\n",buff1, buff2); */
/*     } */
/*   } */


/*   Connect to saveData_realTime1D					*/
/*   if(req_gotoSection(rf, "realTime1D")!=0) { */
/*     printf("section [realTime1D] not found\n"); */
/*   } else { */
/*     if(req_readMacId(rf, buff1, 40)==0) { */
/*       printf("realTime1D pv name not defined\n"); */
/*     } else { */
/*       printf("realTime1D: %s\n", buff1); */
/*     } */
/*   } */

/*   req_close_file(rf); */

/*   return 0; */
/* } */
