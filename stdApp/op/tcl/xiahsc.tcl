#! /bin/sh
#
# 
#
# the next line restarts using tclsh \
exec et_wish $0 ${1+"$@"}

#  hscE
#     communicate with an EPICS database for a Huber Slit Controller

#  set argv jfk:hsc1:  ; set argc [llength $argv]

if {$argc != 1} { 
  puts "usage: hscE PVprefix"
  exit
}
set PVprefix $argv

proc HSCisBusy axisText { 
  switch -- $axisText { 
    idle {set busy 0}
    busy {set busy 1}
    default {set busy "ERROR: Could not tell."}
  }
  return $busy
}

proc HSCdrawPicture {} {
  global slit
  #
  # redraw the new picture
  #
  set slit(status) "new picture"
  update idletasks
  set l [expr -10*$slit(left,RB)]
  set r [expr  10*$slit(right,RB)]
  set t [expr -10*$slit(top,RB)]
  set b [expr  10*$slit(bottom,RB)]
  $slit(widget) coord slit $l $t $r $b
  set movingStr ""
  if [HSCisBusy $slit(hBusy)] { 
    set color green
    lappend movingStr horizontal
  } else { 
    set color bisque
  }
  foreach blade "left right width h0" {
    .$blade.val  config -bg $color
  }
  if [HSCisBusy $slit(vBusy)] { 
    set color green
    lappend movingStr vertical
  } else { 
    set color bisque
  }
  foreach blade "top bottom height v0" {
    .$blade.val  config -bg $color
  }
  if {[HSCisBusy $slit(hBusy)] || [HSCisBusy $slit(vBusy)]} { 
    set slit(status) "$movingStr moving"
  } else {
    set slit(status) "done"
  }
}

proc HSCsend var { 
  global slit
  switch -- $var { 
    stop   {set slit($var) 1; pv put slit($var) }
    locate {set slit($var) 1; pv put slit($var) }
    init   {set slit($var) 1; pv put slit($var) }
    default {pv put slit($var)}
  }
}

proc HSCmonitorReceiver args { 
  HSCdrawPicture
}

proc HSCerrorDialog args { 
  global slit PVprefix
  if $slit(error) {
    set title [format "Error %.0f from %s" $slit(error) $PVprefix]
    set msg $slit(errMsg)
    set bitmap error
    . config -bg yellow
    set slit(status) [format "error #%.0f" $slit(error)]
    tk_dialog .error $title $msg $bitmap 0 Ok
  } else { 
    . config -bg [lindex [. config -bg] 3]
    set slit(status) $slit(errMsg)
  }
}

proc HSCtweak {axis dir} { 
  global slit
  pv get slit($axis)
  set slit($axis) [expr $slit($axis) $dir $slit($axis,twv)]
  HSCsend $axis
}

########################################################################

#define the GUI

set bitmap(left) {
  #define left_width 16
  #define left_height 16
  static char left_bits[] = {
     0x00, 0x00, 0x00, 0x60, 0x00, 0x78, 0x00, 0x7e, 0x80, 0x7f, 0xe0, 0x7f,
     0xf8, 0x7f, 0xfe, 0x7f, 0xf8, 0x7f, 0xe0, 0x7f, 0x80, 0x7f, 0x00, 0x7e,
     0x00, 0x78, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00};
}
set bitmap(right) {
  #define right_width 16
  #define right_height 16
  static char right_bits[] = {
     0x00, 0x00, 0x06, 0x00, 0x1e, 0x00, 0x7e, 0x00, 0xfe, 0x01, 0xfe, 0x07,
     0xfe, 0x1f, 0xfe, 0x7f, 0xfe, 0x1f, 0xfe, 0x07, 0xfe, 0x01, 0x7e, 0x00,
     0x1e, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00};
}

foreach dir "left right" { 
  image create bitmap $dir -data $bitmap($dir)
}

label .title -text "Huber Slit Controller $PVprefix"

#
# motion controls: label, readback, target, and tweaks
#
foreach blade "top left bottom right width height h0 v0" {
  set stub $blade
    set w [frame .$stub -relief groove -bd 2]
    label  $w.lbl -text $blade -width 10 -justify center
    label  $w.rbv -textvariable slit($blade,RB) -width 10 -justify center
    entry  $w.val -textvariable slit($blade) \
                  -bg bisque -justify center -width 10
    entry  $w.twv -textvariable slit($blade,twv) \
                  -bg bisque -justify center -width 10
    set slit($blade,twv) 0.01
    button $w.twr -image left  -command "HSCtweak $blade -"
    button $w.twf -image right -command "HSCtweak $blade +"
    grid $w.lbl -columnspan 3 -sticky EW
    grid $w.rbv -columnspan 3 -sticky EW
    grid $w.val -columnspan 3 -sticky EW
    grid $w.twr $w.twv $w.twf
    grid configure $w.twv -sticky EW
}

# draw a 202x202 rectangle on the picture
set slit(widget) [canvas .picture -width 202 -height 202 \
                                  -bg darkgray -bd 2 -relief sunken]

  button .locate -text Locate -command "HSCsend locate"
  button .init   -text Init   -command "HSCsend init"
  label  .status -textvariable slit(status) -relief groove
  label  .errMsg -textvariable slit(errMsg) -relief groove
  button .stop -text Stop! \
             -bg red2 -activebackground red \
             -fg white -activeforeground yellow \
             -command "HSCsend stop"

  label .statLbl -text status: -justify right
  label .erMsLbl -text error: -justify right
  label .hIDLbl  -text "hor. ID" -justify right
  label .vIDLbl  -text "ver. ID" -justify right
  entry .hID     -textvariable slit(hID) -bg bisque -justify center -width 10
  entry .vID     -textvariable slit(vID) -bg bisque -justify center -width 10

  #
  # radiobuttons: enable/disable
  #
  set w [frame .enable -bd 2 -relief groove]
    radiobutton $w.e -variable slit(enable) -text enable  -value enable
    radiobutton $w.d -variable slit(enable) -text disable -value disable
    $w.e config -command "HSCsend enable"   ;# "enable" is the variable name
    $w.d config -command "HSCsend enable"   ;# "enable" is the variable name
    grid $w.e $w.d

  #
  # radiobuttons: horizontal orientation
  #
  set w [frame .hOrient -bd 2 -relief groove]
    radiobutton $w.lr -variable slit(hOrient) -text LR  -value LR
    radiobutton $w.rl -variable slit(hOrient) -text RL  -value RL
    $w.lr config -command "HSCsend hOrient"
    $w.rl config -command "HSCsend hOrient"
    grid $w.lr $w.rl

  #
  # radiobuttons: vertical orientation
  #
  set w [frame .vOrient -bd 2 -relief groove]
    radiobutton $w.tb -variable slit(vOrient) -text TB  -value TB
    radiobutton $w.bt -variable slit(vOrient) -text BT  -value BT
    $w.tb config -command "HSCsend vOrient"
    $w.bt config -command "HSCsend vOrient"
    grid $w.tb $w.bt

  grid  .title    -columnspan 3    -sticky NEW
  grid  .width .top     .height
  grid  .left  .picture .right
  grid  .h0    .bottom  .v0
  grid  .stop -columnspan 3 -sticky EW
  grid  .statLbl -row 5 -column 0 -sticky E
  grid  .status  -row 5 -column 1 -columnspan 2 -sticky EW
  grid  .erMsLbl -row 6 -column 0 -sticky E
  grid  .errMsg  -row 6 -column 1 -columnspan 2 -sticky EW
  grid  .init    -row 7 -column 0 -sticky EW
  grid  .enable  -row 7 -column 1 -sticky EW
  grid  .locate  -row 7 -column 2 -sticky EW
  grid  .hIDLbl .hID .hOrient  -sticky EW
  grid  .vIDLbl .vID .vOrient  -sticky EW
  grid  configure .hIDLbl -sticky E
  grid  configure .vIDLbl -sticky E

update idletasks
# center the coordinate 0,0
$slit(widget) config -xscrollincrement 1
$slit(widget) config -yscrollincrement 1
$slit(widget) xview scroll -104 units
$slit(widget) yview scroll -104 units
# draw axis lines
$slit(widget) create line -100 0 100 0 ;#-fill gray
$slit(widget) create line 0 -100 0 100 ;#-fill gray
for {set i -10} {$i <= 10} {incr i} { 
  set pos [set i]0
  $slit(widget) create line $pos -2 $pos 3 ;#-fill gray
  $slit(widget) create line -2 $pos 3 $pos ;#-fill gray
}
# draw an image of the slit
# start with a default setting and let drawPicture resize it
$slit(widget) create rect -5 -5 5 5 -tag slit -fill ivory

foreach item "top left bottom right width height h0 v0" { 
  bind  .$item.val  <Return> "HSCsend $item"
}
foreach item "hID vID" { 
  bind  .$item  <Return> "HSCsend $item"
}

# cannot resize this window
wm resizable . 0 0

########################################################################

set tclList ""
set pvList ""
foreach pair {{t top} {l left} {r right} {b bottom}} { 
  scan $pair %s%s epics tcl
  lappend pvList  $PVprefix$epics
  lappend tclList slit($tcl)
  lappend pvList  $PVprefix[set epics]RB
  lappend tclList slit($tcl,RB)
}
foreach item {width height h0 v0} { 
  lappend pvList  $PVprefix$item
  lappend tclList slit($item)
  lappend pvList  $PVprefix[set item]RB
  lappend tclList slit($item,RB)
}
foreach item {Busy ID Orient} { 
  foreach axis "h v" {
    lappend pvList  $PVprefix$axis$item
    lappend tclList slit($axis$item)
  }
}
foreach item {stop enable init locate error errMsg} { 
   lappend pvList  $PVprefix$item
   lappend tclList slit($item)
}
pv linkw $tclList $pvList
pv umon $tclList HSCmonitorReceiver

trace variable slit(error) w HSCerrorDialog

HSCmonitorReceiver

########################################################################

