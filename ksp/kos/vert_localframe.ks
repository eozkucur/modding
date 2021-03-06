print "Start...".

function gethorzvelangle {

  //local pointing is ves:facing:forevector.
  local pointing is velocity:surface.

  local trig_x is vdot(-1*(ship:up*R(0,0,ship:facing:roll)):topvector, pointing).
  local trig_y is vdot((ship:up*R(0,0,ship:facing:roll)):starvector, pointing).

  local result is arctan2(trig_y, trig_x).

  if result < 0 { 
    return 360 + result.
  } else {
    return result.
  }
}

function PID_init {
  parameter
    Kp,      // gain of position
    Ki,      // gain of integral
    Kd,      // gain of derivative
    cMin,  // the bottom limit of the control range (to protect against integral windup)
    cMax.  // the the upper limit of the control range (to protect against integral windup)

  local SeekP is 0. // desired value for P (will get set later).
  local P is 0.     // phenomenon P being affected.
  local I is 0.     // crude approximation of Integral of P.
  local D is 0.     // crude approximation of Derivative of P.
  local oldT is -1. // (old time) start value flags the fact that it hasn't been calculated
  local oldInput is 0. // previous return value of PID controller.

  // Because we don't have proper user structures in kOS (yet?)
  // I'll store the PID tracking values in a list like so:
  //
  local PID_array is list(Kp, Ki, Kd, cMin, cMax, SeekP, P, I, D, oldT, oldInput).

  return PID_array.
}.

function PID_seek {
  parameter
    PID_array, // array built with PID_init.
    seekVal,   // value we want.
    curVal.    // value we currently have.

  // Using LIST() as a poor-man's struct.

  local Kp   is PID_array[0].
  local Ki   is PID_array[1].
  local Kd   is PID_array[2].
  local cMin is PID_array[3].
  local cMax is PID_array[4].
  local oldS   is PID_array[5].
  local oldP   is PID_array[6].
  local oldI   is PID_array[7].
  local oldD   is PID_array[8].
  local oldT   is PID_array[9]. // Old Time
  local oldInput is PID_array[10]. // prev return value, just in case we have to do nothing and return it again.

  local P is seekVal - curVal.
  local D is oldD. // default if we do no work this time.
  local I is oldI. // default if we do no work this time.
  local newInput is oldInput. // default if we do no work this time.

  local t is time:seconds.
  local dT is t - oldT.

  if oldT < 0 {
    // I have never been called yet - so don't trust any
    // of the settings yet.
  } else {
    if dT > 0 { // Do nothing if no physics tick has passed from prev call to now.
     set D to (P - oldP)/dT. // crude fake derivative of P
     local onlyPD is Kp*P + Kd*D.
     if (oldI > 0 or onlyPD > cMin) and (oldI < 0 or onlyPD < cMax) { // only do the I turm when within the control range
      set I to oldI + P*dT. // crude fake integral of P
     }.
     set newInput to onlyPD + Ki*I.
    }.
  }.

  set newInput to max(cMin,min(cMax,newInput)).

  // remember old values for next time.
  set PID_array[5] to seekVal.
  set PID_array[6] to P.
  set PID_array[7] to I.
  set PID_array[8] to D.
  set PID_array[9] to t.
  set PID_array[10] to newInput.

  return newInput.
}.

set collectlog to false.

SET thrott TO 0.

SET t0 TO TIME:SECONDS.
SET start_time TO t0.
LOCK logline TO (TIME:SECONDS - start_time) + " " + thrott.

set logenabed to false.
set execute to false.
set finished to false.

lock input_ik to SHIP:CONTROL:PILOTTOP.
set input_ikreset to true.

when input_ik = 0 then {
    set input_ikreset to true.
    preserve.
}

when input_ikreset and input_ik > 0 then {
    print "reset targets".
    set target_verts to 0.
    set forw_speed_target to 0.
    set side_speed_target to 0.
    set input_ikreset to false.
    preserve.
}

when input_ikreset and input_ik < 0 then {
    print "take control".
   
    if not execute {
        print "taking control".
        set execute to true.
        lock throttle to thrott.
        lock steering to target_steer.
        
        if collectlog {
            set logenabed to true.
        }
    }else{
        print "releasing control".
        set finished to true.
        set execute to false.
        if collectlog {
            set logenabed to false.
            copy throttle_log TO 0.
        }
    }
    
    set input_ikreset to false.
    preserve.
}

lock input_hn to SHIP:CONTROL:PILOTFORE.
set input_hnreset to true.

when input_hn = 0 then {
    set input_hnreset to true.
    preserve.
}

when  input_hn < 0 then {
    print "lower".
    set target_verts to target_verts-0.3.
    print "new target verts: "+target_verts.
    set input_hnreset to false.
    preserve.
}

when  input_hn > 0 then {
    print "higher".
    set target_verts to target_verts+0.3.
    print "new target verts: "+target_verts.
    set input_hnreset to false.
    preserve.
}

lock input_ws to SHIP:CONTROL:PILOTPITCH.
set input_wsreset to true.

when input_ws = 0 then {
    set input_wsreset to true.
    preserve.
}

when  input_ws < 0 then {
    print "forw".
    set forw_speed_target to forw_speed_target+0.3.
    print "new forw_speed_target: "+forw_speed_target.
    set input_wsreset to false.
    preserve.
}

when  input_ws > 0 then {
    print "back".
    set forw_speed_target to forw_speed_target-0.3.
    print "new forw_speed_target: "+forw_speed_target.
    set input_wsreset to false.
    preserve.
}

lock input_ad to SHIP:CONTROL:PILOTYAW.
set input_adreset to true.

when input_ad = 0 then {
    set input_adreset to true.
    preserve.
}

when  input_ad < 0 then {
    print "left".
    set side_speed_target to side_speed_target-0.3.
    print "new side_speed_target: "+side_speed_target.
    set input_adreset to false.
    preserve.
}

when  input_ad > 0 then {
    print "right".
    set side_speed_target to side_speed_target+0.3.
    print "new side_speed_target: "+side_speed_target.
    set input_adreset to false.
    preserve.
}

set vertPID to PID_init( 0.096, 0.5999996775, 0.0102400055, -1, 1 ).
set forwPID to PID_init( 0.07, 0.2, 0.001, -1, 1 ).
set sidePID to PID_init( 0.07, 0.2, 0.001, -1, 1 ).
SET target_verts TO 0.
SET forw_speed_target TO 0.
SET side_speed_target TO 0.

lock horz_spd to ship:surfacespeed.

until finished {
    set thrott to PID_seek( vertPID, target_verts, ship:verticalspeed ).
    
    set horz_vel_heading to gethorzvelangle().
    set forw_speed to horz_spd*cos(horz_vel_heading).
    set side_speed to horz_spd*sin(horz_vel_heading).
    set forw_speed_cmd to PID_seek( forwPID, forw_speed_target, forw_speed ).
    set side_speed_cmd to PID_seek( sidePID, side_speed_target, side_speed ).
    set target_steer to ship:up*R(0,0,ship:facing:roll)*r(forw_speed_cmd*10,0,0)*r(0,side_speed_cmd*10,0).
    //print "intendedspd: "+(forw_speed_cmd*5).
    if collectlog and logenabed {
        LOG logline TO throttle_log.
    }
    WAIT 0.001.
}

set ship:control:pilotmainthrottle to 0.
print "Finish...".