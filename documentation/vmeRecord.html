<HTML>
<HEAD>
<TITLE>vme - Generic VME Record</TITLE>
</HEAD>
<BODY>
<H1>vme - Generic VME Record</H1>

<ADDRESS>Mark Rivers</ADDRESS>

<HR>
<H2>Contents</H2>
<UL>
<LI><A HREF="#Overview">Overview</A>
<LI><A HREF="#Fields">Field Descriptions</A>
<LI><A HREF="#Files">Files</A>
<LI><A HREF="#Release">Release Notes</A>
<LI><A HREF="#Example">Example</A>
</UL>

<A NAME="Overview">
<H2>Overview</H2></A>

<P>
The VME record is designed to perform generic VME I/O.  The VME base address,
addressing mode (A16, A24, A32), address increment, data size (D8, D16, D32),
and I/O direction are all controlled by record fields which can be modified at
run time.  Applications for this record include accessing VME modules for which
no device support exists, and performing IOC diagnostics.
<P>
The VME record is intended only for this specialized VME I/O function, and thus
does not have a separate device support layer.  The VME record itself performs
the VME I/O. All VME I/O is done with the vxWorks function vxMemProbe so that
bus errors are handled harmlessly.
<P>
The VME record supports array operations, i.e. reading or writing continguous
blocks of VME addresses in a single record processing operation.  The VAL field
contains the data to be written or the data read. The SARR (Status Array) field
contains the status of each VME I/O operation, i.e. whether the operation
succeeded or generated a VME bus error. 
<P>
The VME record should be used with care, since it is possible to write to any
location in the VME address space.  It is definitely possible to affect VME
modules in unintended ways!  Even read-only operations can have significant
side-effects.
<P>

<HR>
<TABLE BORDER CELLPADDING=5>
<A NAME="Fields"><CAPTION>
<H2>Record Field Descriptions</H2></CAPTION></A>
<TR>
<TH>Name</TH>
<TH>Access</TH>
<TH>Prompt</TH><TH>Data type</TH> 
<TH>Description</TH>
</TR>

<TR VALIGN=TOP>
<TD>VAL</TD>  <TD>R/W*</TD>   <TD>"Current value"</TD>   
<TD>DBF_LONG (array)</TD>
<TD>The data to be written to VME or the data read from VME.  This is
always an array of LONGS, regardless of whether DSIZ is D8, D16 or D32. 
The maximum array length is determined by NMAX at IOC initialization. 
The actual array length used is equal to NUSE.</TD>
</TR>

<TR VALIGN=TOP>
<TD>SARR</TD>    <TD>R</TD>   <TD>"Status array"</TD>   
<TD>DBF_UCHAR (array)</TD>
<TD>The status of each VME I/O operation. This is an array of UCHAR. Each
element is the status returned by vmMemProbe, i.e. 0 for success, 0xFF
for failure. The maximum array length is determined by NMAX at IOC
initialization.  The actual array length used is equal to NUSE.</TD>
</TR>

<TR VALIGN=TOP>
<TD>ADDR</TD>  <TD>R/W</TD>  <TD>"VME address (hex)"</TD>  <TD>DBF_LONG</TD>
<TD> The starting address of the VME I/O operation. When NUSE is greater
than 1 then the address of each succeeding VME I/O operation will be
incremented by AINC.  The value of ADDR is not modified during this
address autoincrementing.</TD>
</TR>

<TR VALIGN=TOP>
<TD>AMOD</TD>    <TD>R/W</TD>    <TD>"VME address mode"</TD>
<TD>DBF_RECCHOICE</TD>
<TD>The VME address mode. The choices are "A16" (default), "A24" 
and "A32".</TD>
</TR>

<TR VALIGN=TOP>
<TD>DSIZ</TD>  <TD>R/W</TD>    <TD>"VME data size"</TD>    
<TD>DBF_RECCHOICE</TD>
<TD>The VME data transfer size.  The choices are "D8", 
"D16" (default) and "D32".  Note
that the AINC field is not automatically changed to 1, 2, or 4 for
these addressing modes.  The reason for this is that there are some VME boards
which require, for example, D8 transfers but only respond to
odd addresses, so DSIZ must be D8 but AINC must be 2. There are also
some devices which want AINC of 0, because successive data are read
from the same VME address.</TD>
</TR>

<TR VALIGN=TOP>
<TD>RDWT</TD>  <TD>R/W</TD>  <TD>"Read/write"</TD>  <TD>DBF_RECCHOICE</TD>
<TD>The data transfer direction. The choices are "Read" 
(VME bus to VAL field, the default)
and "Write" (VAL field to VME bus).</TD>
</TR>

<TR VALIGN=TOP>
<TD>NMAX</TD>  <TD>R</TD>  <TD>"Max. number of values"</TD>   <TD>DBF_LONG</TD>
<TD>The maximum length of the VAL and SARR arrays. It cannot be modified
after the IOC is initialized. Default=32.</TD>
</TR>

<TR VALIGN=TOP>
<TD>NUSE</TD>  <TD>R/W</TD>  <TD>"Number of values to R/W"</TD> 
<TD>DBF_LONG</TD>
<TD>The actual number of values to be transferred when the record
processes.  It must be less than or equal to NMAX. Default=1.</TD>
</TR>

<TR VALIGN=TOP>
<TD>AINC</TD>  <TD>R/W</TD> <TD>"Address increment (0-4)"</TD> 
<TD>DBF_LONG</TD>
<TD>The address increment which is added to the previous VME address on
each I/O transfer.  It is typically (but not always) 1 for D8
transfers, 2 for D16 transfers, and 4 for D32 transfers. Default=2.</TD>

<TR VALIGN=TOP><TH COLSPAN=5> Private Fields</TH>
</TR>

<TR VALIGN=TOP>
<TD>BPTR</TD>  <TD>N</TD> <TD>"Buffer Pointer"</TD> <TD>DBF_NOACCESS</TD>
<TD>The pointer to the buffer for the VAL field.</TD>
</TR>

<TR VALIGN=TOP>
<TD>SPTR</TD>  <TD>N</TD> <TD>"Status Pointer"</TD> <TD>DBF_NOACCESS</TD>
<TD>The pointer to the buffer for the SARR field.</TD>
</TR>

<TR VALIGN=TOP>
<TD COLSPAN=5, ALIGN=LEFT>
<TABLE>
<TD COLSPAN=3> Note: In the Access column above: </TD>
<TR VALIGN=TOP>
<TD>R</TD>    <TD>Read only<TD>
</TR>
<TR VALIGN=TOP>
<TD>R/W</TD>  <TD>Read and write are allowed</TD>
</TR>
<TR VALIGN=TOP>
<TD>R/W*</TD> <TD>Read and write are allowed; write triggers record 
processing if the record's SCAN field is set to "Passive".</TD>
</TR>
<TR VALIGN=TOP>
<TD>N</TD>    <TD>No access allowed</TD>
</TR>
</TABLE>
</TD>
</TR>
</TABLE>

<HR>
<P>
<A NAME="Files"><H2>Files</H2></A>
The following table briefly describes all of the files required to implement
the VME record.  The reader is assumed to be familiar with the 
<A HREF="http://www.aps.anl.gov/asd/controls/epics/EpicsDocumentation/
AppDevManuals/iocAppBuildSRcontrol.html">
EPICS Application Source/Release Control document</A>
 which describes how to build an
EPICS application tree into which these files are to be placed, and how to run
"gnumake" to build the record support. These files can all be
obtained in a 
<A HREF="pub/vme_record.tar.Z">
compressed tar file</A>.  This file should be untarred in a 
<CODE>&lt;top&gt;/xxxApp/</CODE> directory.


<P>
<TABLE BORDER CELLPADDING=5>
<TR><TH COLSPAN=2>Files to be placed in 
<CODE>&lt;top&gt;/xxxApp/src/</CODE>
</TH> </TR>

<TR VALIGN=TOP>
<TD>vmeRecord.c</TD>
<TD>The source file for the record</TD>
</TR>
<TD>vmeRecord.dbd</TD>
<TD>The database definition file for the record</TD>

<TR VALIGN=TOP>
<TD>Makefile.Vx</TD>
<TD>This file is not included in the distribution.  However, the user must edit
this file and add the following lines:
<PRE>
RECTYPES += vmeRecord.h
SRCS.c   += ../vmeRecord.c
LIBOBJS  += vmeRecord.o

<TR VALIGN=TOP>
<TD>xxxApp.dbd</TD>
<TD>This file is not included in the distribution.  However, the user must edit
this file and add the following line:
<PRE>
include "vmeRecord.dbd"
</PRE></TD>
</TR>

<TR><TH COLSPAN=2>Files to be placed in 
<CODE>&lt;top&gt;/xxxApp/op/adl/</CODE>
</TH> </TR>

<TR VALIGN=TOP>
<TD>VME_IO.adl</TD>
<TD>This file builds an <CODE>medm</CODE> screen to access the VME record.
The <CODE>medm</CODE> screen can be used to probe single memory locations only,
since <CODE>medm</CODE> does not have a widget to display array data as text.
To use it from the command line, type the following:
<PRE>
cars&gt; medm -x -macro REC=my_vme_record VME_IO.adl
</PRE>
<P>
where <CODE>my_vme_record</CODE> is the name of a VME record in an IOC.
<P>
This file can also be used as a related display from other <CODE>medm</CODE>
screens by passing the argument <CODE>REC=my_vme_record</CODE>
</TD>
</TR>
</TABLE>

<HR>
<A NAME="Release"><H2>Release Notes</H2></A>
<UL>
<LI> Version 1.0, March 1996.  Initial release for R3.12 by Mark Rivers.
<LI> Version 1.1, December 1997.  Conversion to EPICS R3.13 by Tim Mooney.
</UL>

<HR>
<A NAME="Example"><H2>Example</H2></A>
The following is an IDL program which uses the VME record to determine and
print out a complete map of all VME bus A16 addresses which respond to D16
read bus cycles.  It is useful for determining what VME addresses are currently
in use.

<PRE>pro vme_mem_map, record

; This procedure prints a table of the VME A16 address space using the
; VME record.
;
; It requires as input the name of a VME record which must exist in the IOC to
; be tested.  For efficiency this record should have a reasonably large value
; of NMAX, but one which is smaller than the channel access transfer limit.
; 2048 is a good value to use.  The name of the record must be passed without 
; a trailing period or field name, e.g. 'test_vme1'.

; Determine maximum number of VME cycles which can be done in a single record
; processing operation
t = caget(record+'.NMAX', n)

; Set the value of NUSE (number to actually use) to this value
t = caput(record+'.NUSE', n)

; Set the address mode to A16
t = caput(record+'.AMOD', 'A16')

; Set the data size to D16
t = caput(record+'.DSIZ', 'D16')

; Set the address increment to 2
t = caput(record+'.AINC', 2)

; Make arrays to hold data and status return info
ntot = 2L^16/2
data = lonarr(ntot)
status = bytarr(ntot)

; Compute addresses of each point
address = 2*lindgen(ntot)

for i=0L, ntot-1, n do begin
   ; Set the base address
   t = caput(record+'.ADDR', address(i))
   ; Process the record
   t = caput(record+'.PROC', 1)
   t = caget(record+'.VAL', d)    ; This copies n values into data()
   data(i)=d
   t = caget(record+'.SARR', d) ; This copies n values into status()
   status(i) = d
endfor

; Print addresses which responded
valid = 0
for i=0L, ntot-1 do begin
   if (valid) then begin
      if (status(i) ne 0) then begin
        ; We have a transition from valid address to invalid.
        print, address(start), data(start), address(i-1), data(i-1), $
            format="(z4, '( ', z8,') --- ', z4, ' (', z8, ')')"
        valid = 0
      endif else if (i eq ntot-1) then begin
        ; We have valid addresses to the end
        print, address(start), data(start), address(i), data(i), $
            format="(z4, '( ', z8,') --- ', z4, ' (', z8, ')')"
      endif
   endif else begin
      if (status(i) eq 0) then begin
        ; We have a transition from invalid address to valid address
        start = i
        valid = 1
      endif
   endelse
endfor
end
</PRE>

<HR>
<ADDRESS>
Suggestions and comments to: 
<A HREF="mailto:rivers@cars.uchicago.edu">
Mark Rivers </A> : (rivers@cars.uchicago.edu)
<BR>
Last modified: December 12, 1996
</ADDRESS>

</BODY>
</HTML>
