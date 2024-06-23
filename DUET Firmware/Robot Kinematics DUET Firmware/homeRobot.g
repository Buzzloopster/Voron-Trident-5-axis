; homeRobot.g
; called to home all axes

M669 K1
; HOME Z AXIS
G91                 ; relative positioning
G1 H1 Z200 F1800	; move Z up until the endstop is triggered 
G90

G91 ; relative mode
G1 H1 X240 Y240 F3000 ; coarse home X or Y
G1 H1 X240 ; coarse home X
G1 H1 Y240 ; coarse home Y
G1 X-3 Y-3 F600 ; move away from the endstops
G1 H1 X7 ; fine home X
G1 H1 Y7 ; fine home Y
G90

; HOME A AXIS
G91
G1 H2 A10 F6000   ; lift U relative to current position
G1 H1 A-120 F1800 ; move U up until the endstop is triggered
G90
;G1 A0 F300

; HOME C AXIS
G91              ; relative positioning
G1 H1 C0 F6000   ; liftC relative to current position
G1 H1 C10 F1800  ; move C up until the endstop is triggered
G1 H1 C-10 F1800
G90

M669 K13 B"CoreXY5AC"
M669 A"X=0:120:0"
M669 A"Y=0:120:0"
M669 A"Z=0:120:0"
M669 A"A=-90:90:0"
M669 A"C=0"
M669 C"X=1:0:0:0:0:0"
M669 C"Y=0:1:0:0:0:0"
M669 C"Z=0:0:1:0:0:0"
M669 C"A=1:0:0:0:0:0"
M669 C"C=0:0:1:0:0:0"
M669 C"Mnoap=1:0:0:0:1:0:0:0:1:0:0:0"
M669 C"Mreference=0:0:0:0:0"
M669 S100
M669 T0.05

G91                 ; relative positioning
G1 H1 Z100 F1800	; move Z up until the endstop is triggered 
G92 Z130.0         ; set Z position to axis maximum (you may want to adjust this)
G90
G1 Z117 F1500

G91 ; relative mode
G1 H1 X240 Y240 F3000 ; coarse home X or Y
G1 H1 X240 ; coarse home X
G1 H1 Y240 ; coarse home Y
G1 X-2 Y-2 F600 ; move away from the endstops
G1 H1 X5 ; fine home X
G1 H1 Y5 ; fine home Y
G92 X120.6         ; set x position to the center of bed (you may want to adjust this)
G92 Y105.9        ; set Y position to the center of bed (you may want to adjust this)
G90
;M669 K13 B"CoreXY5AC"

; HOME A AXIS
G91
G1 H2 A5 F6000   ; lift U relative to current position
G1 H1 A-120 F1800 ; move U up until the endstop is triggered
G92 A-0.672
G90
G1 A0 F300

; HOME C AXIS
G91              ; relative positioning
G1 H1 C0 F6000   ; lift C relative to current position
G1 H1 C10 F1800  ; move C up until the endstop is triggered
G1 H1 C-10 F1800
G92 C0           ; set C position to axis maximum (you may want to adjust this)
G90


