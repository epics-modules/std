/* genSubRecord.c - Record Support Routines for
 *                  General Subroutine Records
 *
 *      Author: Andy Foster
 *
 *	History
 *	-------
 *	Version 1.0  12/11/96  ajf  Created.
 *	Version 1.1  18/04/97  ajf  Fixed get_value, pvdes->pvalue, field.
 *      Version 1.2  18/06/97  ajf  Changed number of fields from 10 to 21.
 *      Version 1.3  05/04/00  cjm  Fixed bug where fldnames was defined as
 *                                  a static array. Cannot do this since there
 *                                  are many instances of the genSub record.
 *	Version 1.4  15/03/01  ajf  Changes for 3.13.
 *                                  Set precision of VERS field to 1.
 *                                  Initialise "string" constants to 0.
 *                                  Replace "symFindbyName" by "symFindbyNameEPICS"
 *                                  for architectures which do not prepend an
 *                                  "_" (i.e. PPC).
 *
 *	Version 1.5  02/10/01  ajf/jfm  Changes for 3.14.3.
 *                                  Removed resriction on array sizes.
 *                                  vxWorks specific things removed.
 *                                  Error checking added during allocation.
 *                                  Record support routines registered 
 *                                  with epicsExportAddress call
 */


#define DEBUG   0
#define VERSION 1.5

#include	<stdlib.h>
#include	<stdio.h>
#include	<string.h>

#include	<alarm.h>
#include	<dbDefs.h>
#include	<registryFunction.h>
#include	<dbEvent.h>
#include	<dbAccess.h>
#include	<dbFldTypes.h>
#include	<errMdef.h>
#include	<recSup.h>
#include	<devSup.h>
#include 	<special.h>
#include	<recGbl.h>
#include    <epicsExport.h>
#include    <cantProceed.h>

#define GEN_SIZE_OFFSET
#include	<genSubRecord.h>
#undef  GEN_SIZE_OFFSET

/* Create RSET - Record Support Entry Table*/

static long init_record();
static long process();
static long get_value();
static long get_precision();
static long cvt_dbaddr();
static long get_array_info();
static long put_array_info();
static long special();
#define report             NULL
#define initialize         NULL
#define get_units          NULL
#define get_graphic_double NULL
#define get_control_double NULL
#define get_alarm_double   NULL
#define get_enum_str       NULL
#define get_enum_strs      NULL
#define put_enum_str       NULL

struct rset genSubRSET={
	RSETNUMBER,
	report,
	initialize,
	init_record,
	process,
	special,
	get_value,
	cvt_dbaddr,
	get_array_info,
	put_array_info,
	get_units,
	get_precision,
	get_enum_str,
	get_enum_strs,
	put_enum_str,
	get_graphic_double,
	get_control_double,
	get_alarm_double };

epicsExportAddress(rset,genSubRSET);

static void monitor( genSubRecord *, int );
static long do_sub( genSubRecord * );
static long findField( int, struct dbAddr *, long *, long );

#define ARG_MAX        21
#define FLDNAME_SZ	   4		/* This went away in 3.14 */
typedef long (*SUBFUNCPTR)();

/* These are the names of the Input fields */
static char Ifldnames[ARG_MAX][FLDNAME_SZ+1] =
  { "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K",
    "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U" };

/* These are the names of the Output fields */
static char Ofldnames[ARG_MAX][FLDNAME_SZ+1] =
  { "VALA", "VALB", "VALC", "VALD", "VALE", "VALF", "VALG", 
    "VALH", "VALI", "VALJ", "VALK", "VALL", "VALM", "VALN",
    "VALO", "VALP", "VALQ", "VALR", "VALS", "VALT", "VALU" };

/* Sizes of field types */
static int sizeofTypes[] = {0, 1, 1, 2, 2, 4, 4, 4, 8, 2};


static long init_record( genSubRecord *pgsub, int pass )
{
  SUBFUNCPTR     psubroutine;
  void           *sub_addr;
  long	         status;
  long           error;
  int            i;
  int            j;
  char           *ufunct;
  unsigned short *typptr;
  void           **valptr;
  void           **ovlptr;
  unsigned long  *nelptr;
  unsigned long  *totptr;
  unsigned long  num;
  struct link    *plinkin;
  struct link    *plinkout;
  char           fldnames[ARG_MAX][FLDNAME_SZ+1];

  status = 0;
  if( pass == 0 )
  {
    pgsub->vers = VERSION;
    for( j=0; j<2; j++ )
    {
      if( j == 0 )    /* Input fields */
      {
        ufunct = pgsub->ufa;
        typptr = &pgsub->fta;
        valptr = &pgsub->a;
        ovlptr = NULL;
        nelptr = &pgsub->noa;
        memcpy( fldnames, Ifldnames, ARG_MAX*(FLDNAME_SZ+1) );
      }
      else           /* Output fields */
      {
        ufunct = pgsub->ufva;
        typptr = &pgsub->ftva;
        valptr = &pgsub->vala;
        ovlptr = &pgsub->ovla;
        nelptr = &pgsub->nova;
        memcpy( fldnames, Ofldnames, ARG_MAX*(FLDNAME_SZ+1) );
      }
      totptr = &pgsub->tova;

      for( i=0; i<ARG_MAX; i++, ufunct+=40, typptr++, valptr++, nelptr++, totptr++ )
      {
        if( *nelptr <= 0 )
          *nelptr = 1;

        if( strlen(ufunct) != 0 ) 
        {
          /* convert the user function subroutine name  */
          sub_addr = (void *)registryFunctionFind(ufunct);
          if( sub_addr == 0 )
          {
            recGblRecordError(S_db_BadSub,(void *)pgsub,"genSubRecord(init_record)");
            status = S_db_BadSub;
          }
          else
          {
            psubroutine = (SUBFUNCPTR)(sub_addr);
            *totptr     = psubroutine(pgsub);

            if( *typptr != DBF_CHAR ) 
              *typptr = DBF_CHAR;
			
            num = (*nelptr)*(*totptr);
            *valptr   = (char *)callocMustSucceed( *nelptr, *totptr, "genSub init_record: insufficent memory" );
            if( j == 1 )
              *ovlptr = (char *)callocMustSucceed( *nelptr, *totptr, "genSub init_record: insufficent memory" );
            *nelptr = num;
            *totptr = num;
#if DEBUG
            printf("Link(%s): Address = 0x%x, Bytes = %d\n", fldnames[i], (unsigned int)(*valptr), *totptr );
#endif
          }
        }
        else
        {
          if( *typptr == DBF_STRING )
          {
            num = (*nelptr)*MAX_STRING_SIZE;
            *valptr   = (char *)callocMustSucceed( *nelptr, MAX_STRING_SIZE, "genSub init_record: insufficent memory" );
            if( j == 1 )
              *ovlptr = (char *)callocMustSucceed( *nelptr, MAX_STRING_SIZE, "genSub init_record: insufficent memory" );
            *totptr = num;
#if DEBUG
            printf("Link(%s): Address = 0x%x, Bytes = %d\n", fldnames[i], (unsigned int)(*valptr), *totptr);
#endif
            
          }
          else
          {
            if( *typptr > DBF_ENUM )
              *typptr = 2;
            num = (*nelptr)*sizeofTypes[*typptr];
            *valptr   = (char *)callocMustSucceed( *nelptr, sizeofTypes[*typptr], "genSub init_record: insufficent memory" );
            if( j == 1 )
              *ovlptr = (char *)callocMustSucceed( *nelptr, sizeofTypes[*typptr], "genSub init_record: insufficent memory" );
            *totptr = num;
#if DEBUG
            printf("Link(%s): Address = 0x%x, Bytes = %d\n", fldnames[i], (unsigned int)(*valptr), *totptr);
#endif
          }
        }
        if( j == 1 )
          ovlptr++;
      }
      if( status )
        return(status);
    }
    return(status);
  }
  else if( pass == 1 )
  {
    /* Deal with the Subroutine Input Link */

    switch( pgsub->subl.type )
    {
      case (CONSTANT):
        if( pgsub->lflg == genSubLFLG_READ )
        {
          recGblInitConstantLink( &pgsub->subl, DBF_STRING, pgsub->snam );
          if( !strncmp(pgsub->snam, "0.0", 3) )
            strcpy(pgsub->snam, " ");
        }
        break;

      case (PV_LINK):
      case (DB_LINK):
      case (CA_LINK):
        break;

      default:
        recGblRecordError( S_db_badField, (void *)pgsub, 
                           "genSubRecord(init_record) Illegal SUBROUTINE LINK" );
        status = S_db_badField;
        break;
    }

    if( !status )
    {
      /* Initialise Input Links */

      plinkin = &pgsub->inpa;
      typptr  = &pgsub->fta;
      valptr  = &pgsub->a;
      nelptr  = &pgsub->noa;
      for( i=0; i<ARG_MAX; i++, plinkin++, typptr++, valptr++, nelptr++ )
      {
        switch( (*plinkin).type ) 
        {
          case (CONSTANT):
#if DEBUG
            printf("Input Link %s is a CONSTANT\n", Ifldnames[i] );
#endif
            if( *nelptr < 2 )
            {
              if( recGblInitConstantLink(plinkin, *typptr, *valptr) )
              {
                if( (*typptr == DBF_STRING) && (!strncmp((char *)(*valptr), "0.0", 3)) )
                  strcpy((char *)(*valptr), " ");
                pgsub->udf = FALSE;
              }
              else
                pgsub->udf = TRUE;
            }
            break;

          case (PV_LINK):
#if DEBUG
            printf("Input Link %s is a PV_LINK\n", Ifldnames[i] );
#endif
            break;

          case (CA_LINK):
#if DEBUG
            printf("Input Link %s is a CA_LINK\n", Ifldnames[i] );
#endif
            break;

          case (DB_LINK):
#if DEBUG
            printf("Input Link %s is a DB_LINK\n", Ifldnames[i] );
#endif
            break;

          default:
            recGblRecordError( S_db_badField, (void *)pgsub, 
                               "genSubRecord(init_record) Illegal INPUT LINK" );
            status = S_db_badField;
            break;
        }
      }

      if( status )
        return(status);

      /* Initialise Output Links */

      plinkout = &pgsub->outa;
      typptr   = &pgsub->ftva;
      valptr   = &pgsub->vala;
      for( i=0; i<ARG_MAX; i++, plinkout++, typptr++, valptr++ )
      {
        switch( (*plinkout).type ) 
        {
          case (CONSTANT):
#if DEBUG
            printf("Output Link %s is a CONSTANT\n", Ofldnames[i] );
#endif
            break;

          case (PV_LINK):
#if DEBUG
            printf("Output Link %s is a PV_LINK\n", Ofldnames[i] );
#endif
            break;

          case (CA_LINK):
#if DEBUG
            printf("Output Link %s is a CA_LINK\n", Ofldnames[i] );
#endif
            break;

          case (DB_LINK):
#if DEBUG
            printf("Output Link %s is a DB_LINK\n", Ofldnames[i] );
#endif
            break;

          default:
            recGblRecordError( S_db_badField, (void *)pgsub, 
                               "genSubRecord(init_record) Illegal OUTPUT LINK" );
            status = S_db_badField;
            break;
        }
      }
    
      if( !status )
      {
        if(strlen(pgsub->inam)!=0) 
        { 
          /* convert the initialization subroutine name  */

          sub_addr = (void *)registryFunctionFind(pgsub->inam);
          if( sub_addr == 0 )
          {
	    recGblRecordError(S_db_BadSub,(void *)pgsub,"genSubRecord(init_record)");
	    status = S_db_BadSub;
          }
          else
          {
            psubroutine = (SUBFUNCPTR)(sub_addr);
            error       = psubroutine(pgsub);
          }
        }
      }

      if( !status )
      {
        if( pgsub->lflg == genSubLFLG_IGNORE )
        {
          if(strlen(pgsub->snam)!=0) 
          { 
            /* convert the process subroutine name  */

            sub_addr = (void *)registryFunctionFind(pgsub->snam);
            if( sub_addr == 0 )
            {
              recGblRecordError(S_db_BadSub,(void *)pgsub,"genSubRecord(init_record)");
              status = S_db_BadSub;
            }
            else
              pgsub->sadr = (long)sub_addr;
          }
        }
      }
    }
  }
  return( status );
}


static long process( genSubRecord *pgsub )
{
  int            i;
  void           *sub_addr;
  long           status;
  struct link    *plinkin;
  struct link    *plinkout;
  unsigned short *typptr;
  unsigned long  *nelptr;
  long           nRequest;
  long           options;
  void           **valptr;

  pgsub->pact = TRUE;
  status      = 0;
  options     = 0;

  if( pgsub->lflg == genSubLFLG_READ )
  {
    /* Get the Subroutine Name from the Link and look it up */

    nRequest = 1;
    status   = dbGetLink( &(pgsub->subl), DBR_STRING, pgsub->snam, &options, &nRequest );
    if( !status )
    {
      if( pgsub->snam[0] != '\0' )
      {
        if( strcmp(pgsub->snam, pgsub->onam) )  
        {
          /* To save time, only look up the routine if it has a different name */

          strcpy(pgsub->onam, pgsub->snam);

          sub_addr = (void *)registryFunctionFind(pgsub->snam);
          if( sub_addr == 0 )
          {
            recGblRecordError(S_db_BadSub,(void *)pgsub,"genSubRecord(process) registryFunctionFind failed");
            status = S_db_BadSub;
          }
          else
            pgsub->sadr = (long)sub_addr;
        }
      }
    }
  }

  /* Get the Values from the input links */

  if( !status )
  {
    plinkin  = &pgsub->inpa;
    valptr   = &pgsub->a;
    nelptr   = &pgsub->noa;
    typptr   = &pgsub->fta;
    for( i=0; i<ARG_MAX; i++, plinkin++, valptr++, nelptr++, typptr++ )
    {
      nRequest = *nelptr;
      status   = dbGetLink( plinkin, *typptr, *valptr, &options, &nRequest );
      if( status )
      {
        printf("Status %ld from dbGetLink (%s)\n", status, Ifldnames[i]);
        break;
      }
    }
  }

  /* Call the user routine */

  if( !status )
    pgsub->val = do_sub(pgsub);

  /* Put the values on the output links */

  if( !status )
  {
    plinkout = &pgsub->outa;
    valptr   = &pgsub->vala;
    nelptr   = &pgsub->nova;
    typptr   = &pgsub->ftva;
    for( i=0; i<ARG_MAX; i++, plinkout++, valptr++, nelptr++, typptr++ )
    {
      nRequest = *nelptr;
      status   = dbPutLink( plinkout, *typptr, *valptr, nRequest );
      status   = 0;

/*
      Do not check this because dbPutLink returns -1 if channel not connected!
      if( status )
      {
        printf("Status %d from recGblPutLinkValue (%s)\n", status, Ofldnames[i]);
        break;
      }
*/
    }
  }

  if( !status )
  {
    recGblGetTimeStamp(pgsub);
    monitor(pgsub, 1);
    recGblFwdLink(pgsub);
  } 

  pgsub->pact = FALSE;
  return(status);
}


static long get_precision( struct dbAddr *paddr, long *precision )
{
    genSubRecord *pgsub;
    int          fieldIndex;

    fieldIndex = dbGetFieldIndex(paddr);
    if( fieldIndex == genSubRecordVERS )
    {
      *precision = 1;
      return 0;
    }

    pgsub      = (genSubRecord *)paddr->precord;
    *precision = pgsub->prec;
    if( paddr->pfield == (void *)&pgsub->val ) 
      return(0);
    recGblGetPrec(paddr,precision);
    return(0);
}


static long get_value( genSubRecord *pgsub, struct valueDes *pvdes )
{
#if DEBUG
    printf("Calling get_value...\n");
#endif
    pvdes->no_elements = 1;
    pvdes->pvalue      = (void *)(&pgsub->val);
    pvdes->field_type  = DBF_LONG;
    return(0);
}


static void monitor( genSubRecord *pgsub, int reset )
{
  int            i;
  unsigned short monitor_mask;
  unsigned long  *totptr;
  void           **valptr;
  void           **ovlptr;

  if( reset )
    monitor_mask = recGblResetAlarms(pgsub);
  else
    monitor_mask = 0;

  monitor_mask |= DBE_VALUE | DBE_LOG;

  /* Post events for SADR (subroutine address) whenever this routine is called */

  if( monitor_mask )
  {
    if( pgsub->sadr != pgsub->osad )
    {
      db_post_events(pgsub, &pgsub->sadr, monitor_mask);
      pgsub->osad = pgsub->sadr;
    }
  }

  /* Post events for VAL and output fields when this routine called from process */
  /* Event posting on output arrays depends on the setting of pgsub->eflg        */

  if( reset )
  {
    if( monitor_mask )
    {
      if( pgsub->val != pgsub->oval )
      {
        db_post_events(pgsub, &pgsub->val, monitor_mask);
        pgsub->oval = pgsub->val;
      }
    }
    if( pgsub->eflg == genSubEFLG_NEVER )
      return;
    else if( pgsub->eflg == genSubEFLG_ON_CHANGE )
    {
      if(monitor_mask)
      {
        valptr = &pgsub->vala;
        ovlptr = &pgsub->ovla;
        totptr = &pgsub->tova;
        for( i=0; i<ARG_MAX; i++, valptr++, ovlptr++, totptr++ )
        {
          if( memcmp(*ovlptr, *valptr, *totptr) )
          {
#if DEBUG
            printf("Differences in field %s were found. Total Bytes = %d\n", Ofldnames[i], *totptr);
#endif
            memcpy( *ovlptr, *valptr, *totptr );
            db_post_events(pgsub, *valptr, monitor_mask);
          }
        }
      }
    }
    else if( pgsub->eflg == genSubEFLG_ALWAYS )
    {
      if(monitor_mask)
      {
        valptr = &pgsub->vala;
        for( i=0; i<ARG_MAX; i++, valptr++ )
          db_post_events(pgsub, *valptr, monitor_mask);
      }
    }
  }
  return;
}


static long do_sub( genSubRecord *pgsub )
{
  long       status;
  SUBFUNCPTR psubroutine;

  /* If there is a routine, call it */

  if( pgsub->snam[0] != '\0' )
  {
    psubroutine = (SUBFUNCPTR)((void *)pgsub->sadr);
    if( psubroutine == NULL)
    {
      recGblRecordError(S_db_BadSub,(void *)pgsub,"genSubRecord(process) NO SUBROUTINE");
      status = S_db_BadSub;
    }
    else
    {
      status = psubroutine(pgsub);
      if( status < 0 )
        recGblSetSevr(pgsub, SOFT_ALARM, pgsub->brsv);
      else
        pgsub->udf = FALSE;
    }
  }
  else
    status = 0;

  return( status );
}


static long cvt_dbaddr( struct dbAddr *paddr )
{
  int  error;
  int  flag;
  long no_elements;
  long nNew;

#if DEBUG
  printf("Calling cvt_dbaddr...\n");
#endif

  flag  = 1;
  nNew  = 0;
  error = findField( flag, paddr, &no_elements, nNew );
  if( error )
    printf("cvt_dbaddr: Could not find field\n");
  return(0);
}


static long get_array_info( struct dbAddr *paddr, long *no_elements, long *offset )
{
  int  error;
  int  flag;
  long nNew;

#if DEBUG
  printf("Calling get_array_info...\n");
#endif
  *offset = 0;
  nNew    = 0;
  flag    = 2;
  error   = findField( flag, paddr, no_elements, nNew );
  if( error )
    printf("get_array_info: Could not find field\n");
  return(0);
}


static long put_array_info( struct dbAddr *paddr, long nNew )
{
  int  error;
  int  flag;
  long no_elements;

#if DEBUG
  printf("Calling put_array_info...\n");
#endif
  flag  = 3;
  error = findField( flag, paddr, &no_elements, nNew );
  if( error )
    printf("put_array_info: Could not find field\n");
  return(0);
}


static long special( struct dbAddr *paddr, int after )
{
  genSubRecord *pgsub;
  void         *sub_addr;

  pgsub = (genSubRecord *)paddr->precord;
  if( after )
  {
    if( pgsub->lflg == genSubLFLG_IGNORE )
    {
      if( pgsub->snam[0] != '\0' )
      {
        sub_addr = (void *)registryFunctionFind(pgsub->snam);
        if( sub_addr == 0 )
        {
          recGblRecordError(S_db_BadSub,(void *)pgsub,"genSubRecord(special) registryFunctionFind failed");
          return(S_db_BadSub);
        }
        else
        {
          pgsub->sadr = (long)sub_addr;
          monitor(pgsub, 0);
        }
      }
    }
  }
  return(0);
}


static long findField( int flag, struct dbAddr *paddr, long *no_elements, long nNew )
{
  long         error;
  int          fieldIndex;
  genSubRecord *pgsub;

  pgsub      = (genSubRecord *)paddr->precord;
  error      = 0;
  fieldIndex = dbGetFieldIndex(paddr);
  switch( fieldIndex )
  {
    case genSubRecordA:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->a;
        paddr->no_elements = pgsub->noa;
        paddr->field_type  = pgsub->fta;
      }
      else if( flag == 2 )
        *no_elements = pgsub->noa;
      else if( flag == 3 )
        pgsub->noa = nNew;
      break;

    case genSubRecordB:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->b;
        paddr->no_elements = pgsub->nob;
        paddr->field_type  = pgsub->ftb;
      }
      else if( flag == 2 )
        *no_elements = pgsub->nob;
      else if( flag == 3 )
        pgsub->nob = nNew;
      break;

    case genSubRecordC:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->c;
        paddr->no_elements = pgsub->noc;
        paddr->field_type  = pgsub->ftc;
      }
      else if( flag == 2 )
        *no_elements = pgsub->noc;
      else if( flag == 3 )
        pgsub->noc = nNew;
      break;

    case genSubRecordD:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->d;
        paddr->no_elements = pgsub->nod;
        paddr->field_type  = pgsub->ftd;
      }
      else if( flag == 2 )
        *no_elements = pgsub->nod;
      else if( flag == 3 )
        pgsub->nod = nNew;
      break;

    case genSubRecordE:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->e;
        paddr->no_elements = pgsub->noe;
        paddr->field_type  = pgsub->fte;
      }
      else if( flag == 2 )
        *no_elements = pgsub->noe;
      else if( flag == 3 )
        pgsub->noe = nNew;
      break;

    case genSubRecordF:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->f;
        paddr->no_elements = pgsub->nof;
        paddr->field_type  = pgsub->ftf;
      }
      else if( flag == 2 )
        *no_elements = pgsub->nof;
      else if( flag == 3 )
        pgsub->nof = nNew;
      break;

    case genSubRecordG:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->g;
        paddr->no_elements = pgsub->nog;
        paddr->field_type  = pgsub->ftg;
      }
      else if( flag == 2 )
        *no_elements = pgsub->nog;
      else if( flag == 3 )
        pgsub->nog = nNew;
      break;

    case genSubRecordH:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->h;
        paddr->no_elements = pgsub->noh;
        paddr->field_type  = pgsub->fth;
      }
      else if( flag == 2 )
        *no_elements = pgsub->noh;
      else if( flag == 3 )
        pgsub->noh = nNew;
      break;

    case genSubRecordI:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->i;
        paddr->no_elements = pgsub->noi;
        paddr->field_type  = pgsub->fti;
      }
      else if( flag == 2 )
        *no_elements = pgsub->noi;
      else if( flag == 3 )
        pgsub->noi = nNew;
      break;

    case genSubRecordJ:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->j;
        paddr->no_elements = pgsub->noj;
        paddr->field_type  = pgsub->ftj;
      }
      else if( flag == 2 )
        *no_elements = pgsub->noj;
      else if( flag == 3 )
        pgsub->noj = nNew;
      break;

    case genSubRecordK:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->k;
        paddr->no_elements = pgsub->nok;
        paddr->field_type  = pgsub->ftk;
      }
      else if( flag == 2 )
        *no_elements = pgsub->nok;
      else if( flag == 3 )
        pgsub->nok = nNew;
      break;

    case genSubRecordL:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->l;
        paddr->no_elements = pgsub->nol;
        paddr->field_type  = pgsub->ftl;
      }
      else if( flag == 2 )
        *no_elements = pgsub->nol;
      else if( flag == 3 )
        pgsub->nol = nNew;
      break;

    case genSubRecordM:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->m;
        paddr->no_elements = pgsub->nom;
        paddr->field_type  = pgsub->ftm;
      }
      else if( flag == 2 )
        *no_elements = pgsub->nom;
      else if( flag == 3 )
        pgsub->nom = nNew;
      break;

    case genSubRecordN:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->n;
        paddr->no_elements = pgsub->non;
        paddr->field_type  = pgsub->ftn;
      }
      else if( flag == 2 )
        *no_elements = pgsub->non;
      else if( flag == 3 )
        pgsub->non = nNew;
      break;

    case genSubRecordO:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->o;
        paddr->no_elements = pgsub->noo;
        paddr->field_type  = pgsub->fto;
      }
      else if( flag == 2 )
        *no_elements = pgsub->noo;
      else if( flag == 3 )
        pgsub->noo = nNew;
      break;

    case genSubRecordP:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->p;
        paddr->no_elements = pgsub->nop;
        paddr->field_type  = pgsub->ftp;
      }
      else if( flag == 2 )
        *no_elements = pgsub->nop;
      else if( flag == 3 )
        pgsub->nop = nNew;
      break;

    case genSubRecordQ:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->q;
        paddr->no_elements = pgsub->noq;
        paddr->field_type  = pgsub->ftq;
      }
      else if( flag == 2 )
        *no_elements = pgsub->noq;
      else if( flag == 3 )
        pgsub->noq = nNew;
      break;

    case genSubRecordR:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->r;
        paddr->no_elements = pgsub->nor;
        paddr->field_type  = pgsub->ftr;
      }
      else if( flag == 2 )
        *no_elements = pgsub->nor;
      else if( flag == 3 )
        pgsub->nor = nNew;
      break;

    case genSubRecordS:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->s;
        paddr->no_elements = pgsub->nos;
        paddr->field_type  = pgsub->fts;
      }
      else if( flag == 2 )
        *no_elements = pgsub->nos;
      else if( flag == 3 )
        pgsub->nos = nNew;
      break;

    case genSubRecordT:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->t;
        paddr->no_elements = pgsub->not;
        paddr->field_type  = pgsub->ftt;
      }
      else if( flag == 2 )
        *no_elements = pgsub->not;
      else if( flag == 3 )
        pgsub->not = nNew;
      break;

    case genSubRecordU:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->u;
        paddr->no_elements = pgsub->nou;
        paddr->field_type  = pgsub->ftu;
      }
      else if( flag == 2 )
        *no_elements = pgsub->nou;
      else if( flag == 3 )
        pgsub->nou = nNew;
      break;

    case genSubRecordVALA:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->vala;
        paddr->no_elements = pgsub->nova;
        paddr->field_type  = pgsub->ftva;
      }
      else if( flag == 2 )
        *no_elements = pgsub->nova;
      else if( flag == 3 )
        pgsub->nova = nNew;
      break;

    case genSubRecordVALB:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->valb;
        paddr->no_elements = pgsub->novb;
        paddr->field_type  = pgsub->ftvb;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novb;
      else if( flag == 3 )
        pgsub->novb = nNew;
      break;

    case genSubRecordVALC:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->valc;
        paddr->no_elements = pgsub->novc;
        paddr->field_type  = pgsub->ftvc;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novc;
      else if( flag == 3 )
        pgsub->novc = nNew;
      break;

    case genSubRecordVALD:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->vald;
        paddr->no_elements = pgsub->novd;
        paddr->field_type  = pgsub->ftvd;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novd;
      else if( flag == 3 )
        pgsub->novd = nNew;
      break;

    case genSubRecordVALE:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->vale;
        paddr->no_elements = pgsub->nove;
        paddr->field_type  = pgsub->ftve;
      }
      else if( flag == 2 )
        *no_elements = pgsub->nove;
      else if( flag == 3 )
        pgsub->nove = nNew;
      break;

    case genSubRecordVALF:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->valf;
        paddr->no_elements = pgsub->novf;
        paddr->field_type  = pgsub->ftvf;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novf;
      else if( flag == 3 )
        pgsub->novf = nNew;
      break;

    case genSubRecordVALG:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->valg;
        paddr->no_elements = pgsub->novg;
        paddr->field_type  = pgsub->ftvg;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novg;
      else if( flag == 3 )
        pgsub->novg = nNew;
      break;

    case genSubRecordVALH:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->valh;
        paddr->no_elements = pgsub->novh;
        paddr->field_type  = pgsub->ftvh;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novh;
      else if( flag == 3 )
        pgsub->novh = nNew;
      break;

    case genSubRecordVALI:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->vali;
        paddr->no_elements = pgsub->novi;
        paddr->field_type  = pgsub->ftvi;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novi;
      else if( flag == 3 )
        pgsub->novi = nNew;
      break;

    case genSubRecordVALJ:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->valj;
        paddr->no_elements = pgsub->novj;
        paddr->field_type  = pgsub->ftvj;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novj;
      else if( flag == 3 )
        pgsub->novj = nNew;
      break;

    case genSubRecordVALK:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->valk;
        paddr->no_elements = pgsub->novk;
        paddr->field_type  = pgsub->ftvk;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novk;
      else if( flag == 3 )
        pgsub->novk = nNew;
      break;

    case genSubRecordVALL:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->vall;
        paddr->no_elements = pgsub->novl;
        paddr->field_type  = pgsub->ftvl;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novl;
      else if( flag == 3 )
        pgsub->novl = nNew;
      break;

    case genSubRecordVALM:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->valm;
        paddr->no_elements = pgsub->novm;
        paddr->field_type  = pgsub->ftvm;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novm;
      else if( flag == 3 )
        pgsub->novm = nNew;
      break;

    case genSubRecordVALN:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->valn;
        paddr->no_elements = pgsub->novn;
        paddr->field_type  = pgsub->ftvn;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novn;
      else if( flag == 3 )
        pgsub->novn = nNew;
      break;

    case genSubRecordVALO:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->valo;
        paddr->no_elements = pgsub->novo;
        paddr->field_type  = pgsub->ftvo;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novo;
      else if( flag == 3 )
        pgsub->novo = nNew;
      break;

    case genSubRecordVALP:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->valp;
        paddr->no_elements = pgsub->novp;
        paddr->field_type  = pgsub->ftvp;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novp;
      else if( flag == 3 )
        pgsub->novp = nNew;
      break;

    case genSubRecordVALQ:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->valq;
        paddr->no_elements = pgsub->novq;
        paddr->field_type  = pgsub->ftvq;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novq;
      else if( flag == 3 )
        pgsub->novq = nNew;
      break;

    case genSubRecordVALR:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->valr;
        paddr->no_elements = pgsub->novr;
        paddr->field_type  = pgsub->ftvr;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novr;
      else if( flag == 3 )
        pgsub->novr = nNew;
      break;

    case genSubRecordVALS:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->vals;
        paddr->no_elements = pgsub->novs;
        paddr->field_type  = pgsub->ftvs;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novs;
      else if( flag == 3 )
        pgsub->novs = nNew;
      break;

    case genSubRecordVALT:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->valt;
        paddr->no_elements = pgsub->novt;
        paddr->field_type  = pgsub->ftvt;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novt;
      else if( flag == 3 )
        pgsub->novt = nNew;
      break;

    case genSubRecordVALU:
      if( flag == 1 )
      {
        paddr->pfield      = pgsub->valu;
        paddr->no_elements = pgsub->novu;
        paddr->field_type  = pgsub->ftvu;
      }
      else if( flag == 2 )
        *no_elements = pgsub->novu;
      else if( flag == 3 )
        pgsub->novu = nNew;
      break;

    default:
      error = 1;
      break;
  }

  if( !error && flag == 1 )
  {
    paddr->dbr_field_type = paddr->field_type;
    if( paddr->field_type == DBF_STRING )
      paddr->field_size = MAX_STRING_SIZE;
    else
      paddr->field_size = sizeofTypes[paddr->field_type];
  }

  return(error);
}
