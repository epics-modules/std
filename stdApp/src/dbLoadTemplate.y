%{

/**************************************************************************
 *
 *     Author:	Jim Kowalkowski
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contracts:
 *      (W-7405-ENG-36) at the Los Alamos National Laboratory,
 *      and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *      Initial development by:
 *              The Controls and Automation Group (AT-8)
 *              Ground Test Accelerator
 *              Accelerator Technology Division
 *              Los Alamos National Laboratory
 *
 *      Co-developed with
 *              The Controls and Computing Group
 *              Accelerator Systems Division
 *              Advanced Photon Source
 *              Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01	10-29-93	jbk	initial version
 *
 ***********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

#ifdef vxWorks
#include <symLib.h>
#include <sysSymTbl.h>
#endif

#include "macLib.h"
#include "dbmf.h"
#include "epicsVersion.h"
#ifdef _WIN32
#include "getopt.h"
#endif

#define MAXPATHLEN 300
static char db_path[MAXPATHLEN] = "";

static int line_num;
static int yyerror();
int dbLoadTemplate(char* sub_file, char* template_path);

int dbLoadRecords(char* pfilename, char* pattern, char* ppath);

#ifdef vxWorks
#define VAR_MAX_VAR_STRING 5000
#define VAR_MAX_VARS 100
#else
#define VAR_MAX_VAR_STRING 50000
#define VAR_MAX_VARS 700
#endif

static char *sub_collect = NULL;
static MAC_HANDLE *macHandle = NULL;
static char** vars = NULL;
static char* db_file_name = NULL;
static int var_count,sub_count;
static SYM_TYPE symTyp;
static char **symVal;

%}

%start template

%token <Str> WORD QUOTE
%token DBFILE
%token PATTERN
%token EQUALS
%left O_PAREN C_PAREN
%left O_BRACE C_BRACE

%union
{
    int	Int;
    char Char;
    char *Str;
    double Real;
}

%%

template: templs
	| subst
	;

templs: templs templ
	| templ
	;

templ: templ_head O_BRACE subst C_BRACE
	| templ_head
	{
		if(db_file_name)
			dbLoadRecords(db_file_name,NULL,db_path);
		else
			fprintf(stderr,"Error: no db file name given\n");
	}
	;

templ_head: DBFILE WORD
	{
		var_count=0;
		if(db_file_name) dbmfFree(db_file_name);
		db_file_name = dbmfMalloc(strlen($2)+1);
		strcpy(db_file_name,$2);
		dbmfFree($2);
	}
        | DBFILE WORD WORD
	{
#ifdef vxWorks
		var_count=0;
		if(db_file_name) dbmfFree(db_file_name);
		db_file_name = dbmfMalloc(strlen($2)+1);
		strcpy(db_file_name,$2);
		dbmfFree($2);
                /* First assume that the string is a vxWorks symbol, if not then
                   assume it is a literal path */
                if (symFindByName(sysSymTbl,$3,(char **)&symVal,&symTyp)==OK)
                        strcpy(db_path, *symVal);
                else
                        strcpy(db_path,$3);
                dbmfFree($3);
#endif
	}
	;

subst: PATTERN pattern subs
    | var_subs
    ;

pattern: O_BRACE vars C_BRACE
	{ 
#ifdef ERROR_STUFF
		int i;
		for(i=0;i<var_count;i++) fprintf(stderr,"variable=(%s)\n",vars[i]);
		fprintf(stderr,"var_count=%d\n",var_count);
#endif
	}
    ;

vars: vars var
	| var
	;

var: WORD
	{
	    vars[var_count] = dbmfMalloc(strlen($1)+1);
	    strcpy(vars[var_count],$1);
	    var_count++;
	    dbmfFree($1);
	}
	;

subs: subs sub
	| sub
	;

sub: WORD O_BRACE vals C_BRACE
	{
		sub_collect[strlen(sub_collect)-1]='\0';
#ifdef ERROR_STUFF
		fprintf(stderr,"dbLoadRecords(%s)\n",sub_collect);
#endif
		if(db_file_name)
			dbLoadRecords(db_file_name,sub_collect,db_path);
		else
			fprintf(stderr,"Error: no db file name given\n");
		dbmfFree($1);
		sub_collect[0]='\0';
		sub_count=0;
	}
	| O_BRACE vals C_BRACE
	{
		sub_collect[strlen(sub_collect)-1]='\0';
#ifdef ERROR_STUFF
		fprintf(stderr,"dbLoadRecords(%s)\n",sub_collect);
#endif
		if(db_file_name)
			dbLoadRecords(db_file_name,sub_collect,db_path);
		else
			fprintf(stderr,"Error: no db file name given\n");
		sub_collect[0]='\0';
		sub_count=0;
	}
	;

vals: vals val
	| val
	;

val: QUOTE
	{
		if(sub_count<=var_count)
		{
			strcat(sub_collect,vars[sub_count]);
			strcat(sub_collect,"=\"");
			strcat(sub_collect,$1);
			strcat(sub_collect,"\",");
			sub_count++;
		}
		dbmfFree($1);
	}
	| WORD
	{
		if(sub_count<=var_count)
		{
			strcat(sub_collect,vars[sub_count]);
			strcat(sub_collect,"=");
			strcat(sub_collect,$1);
			strcat(sub_collect,",");
			sub_count++;
		}
		dbmfFree($1);
	}
	;

var_subs: var_subs var_sub
	| var_sub
	;

var_sub: WORD O_BRACE sub_pats C_BRACE
	{
		sub_collect[strlen(sub_collect)-1]='\0';
#ifdef ERROR_STUFF
		fprintf(stderr,"dbLoadRecords(%s)\n",sub_collect);
#endif
		if(db_file_name)
			dbLoadRecords(db_file_name,sub_collect,db_path);
		else
			fprintf(stderr,"Error: no db file name given\n");
		dbmfFree($1);
		sub_collect[0]='\0';
		sub_count=0;
	}
	| O_BRACE sub_pats C_BRACE
	{
		sub_collect[strlen(sub_collect)-1]='\0';
#ifdef ERROR_STUFF
		fprintf(stderr,"dbLoadRecords(%s)\n",sub_collect);
#endif
		if(db_file_name)
			dbLoadRecords(db_file_name,sub_collect,db_path);
		else
			fprintf(stderr,"Error: no db file name given\n");
		sub_collect[0]='\0';
		sub_count=0;
	}
	;

sub_pats: sub_pats sub_pat
	| sub_pat
	;

sub_pat: WORD EQUALS WORD
	{
		strcat(sub_collect,$1);
		strcat(sub_collect,"=");
		strcat(sub_collect,$3);
		strcat(sub_collect,",");
		dbmfFree($1); dbmfFree($3);
		sub_count++;
	}
	| WORD EQUALS QUOTE
	{
		strcat(sub_collect,$1);
		strcat(sub_collect,"=\"");
		strcat(sub_collect,$3);
		strcat(sub_collect,"\",");
		dbmfFree($1); dbmfFree($3);
		sub_count++;
	}
	;

%%
 
#include "dbLoadTemplate_lex.c"
 
static int yyerror(char* str)
{
	fprintf(stderr,"Substitution file parse error\n");
	fprintf(stderr,"line %d:%s\n",line_num,yytext);
	return(0);
}

static int is_not_inited = 1;
 
int dbLoadTemplate(char* sub_file, char* template_path)
{
	FILE *fp;
	int ind;

	line_num=0;

	if( !sub_file || !*sub_file)
	{
		fprintf(stderr,"must specify variable substitution file\n");
		return -1;
	}

	if( !(fp=fopen(sub_file,"r")) )
	{
		fprintf(stderr,"dbLoadTemplate: error opening sub file %s\n",sub_file);
		return -1;
	}

	if (template_path && strlen(template_path)) {
		strcpy(db_path, template_path);
	} else {
		db_path[0] = '\0';
	}

	vars = (char**)malloc(VAR_MAX_VARS * sizeof(char*));
	sub_collect = malloc(VAR_MAX_VAR_STRING);
	sub_collect[0]='\0';
	var_count=0;
	sub_count=0;

	if(is_not_inited)
	{
		yyin=fp;
		is_not_inited=0;
	}
	else
	{
		yyrestart(fp);
	}

	yyparse();
	for(ind=0;ind<var_count;ind++) dbmfFree(vars[ind]);
	free(vars);
	free(sub_collect);
	vars = NULL;
	fclose(fp);
	if(db_file_name){
	    dbmfFree((void *)db_file_name);
	    db_file_name = NULL;
	}
	return 0;
}

#ifndef vxWorks
/* this is template loader similar to vxWorks one for .db files */
int main(int argc, char** argv)
{
	extern char* optarg;
	extern int optind;
	char* name = (char*)NULL;
	char* path = (char*)NULL;
	int no_error = 1;
	int c;

	optind = 1;
	while(no_error && (c=getopt(argc,argv,"s:p:"))!=-1)
	{
		switch(c)
		{
		case 's':
			if(name) dbmfFree(name);
			name = dbmfMalloc(strlen(optarg));
			strcpy(name,optarg);
			break;
		case 'p':
			if (path) dbmfFree(path);
			path = dbmfMalloc(strlen(optarg));
			strcpy(path,optarg);
			break;
		default: no_error=0; break;
		}
	}

	if(!no_error || optind>=argc)
	{
		fprintf(stderr,"Usage: %s <-s name> <-p path> sub_file\n",argv[0]);
		fprintf(stderr,"\n\twhere name is the output database name,\n");
		fprintf(stderr,"\tsub_file is the variable substitution file,\n");
		fprintf(stderr,"\tand path is to be prepended to the file name\n");
		fprintf(stderr,"\tfound in that substitution file.\n");
		fprintf(stderr,"\n\tThis program uses the sub_file to produce a\n");
		fprintf(stderr,"\tdatabase of name name to standard out.\n");
		exit(1);
	}

	if(!name) {
	    name = dbmfMalloc(strlen("Composite") + 1);
	    strcpy(name,"Composite");
	}
	dbLoadTemplate(argv[optind], path);
	dbmfFree((void *)name);
	if (path) dbmfFree((void *)path);
	return(0);
}
#endif
