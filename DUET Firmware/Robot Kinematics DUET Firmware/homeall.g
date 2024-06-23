; homeall.g
; called to home all axes
;
;M98 P"homez.g"
;M98 P"homex.g"
;M98 P"homey.g"
;M98 P"homea.g"
;M98 P"homec.g"

;M669 K1

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

; HOME Z AXIS
G91                 ; relative positioning
G1 H1 Z100 F1800	; move Z up until the endstop is triggered 
G92 Z128.94         ; set Z position to axis maximum (you may want to adjust this)
G90
G1 Z116.4 F1500

; HOME A AXIS
G91
G1 H2 A10 F6000   ; lift U relative to current position
G1 H1 A-120 F1800 ; move U up until the endstop is triggered
G92 A-0.55
G90
G1 A0 F300

; HOME C AXIS
G91
G91              ; relative positioning
G1 H1 C0 F6000   ; lift V relative to current position
G1 H1 C60 F1800  ; move V up until the endstop is triggered
G92 C0           ; set V position to axis maximum (you may want to adjust this)
G90

G91 ; relative mode
G1 H1 X240 Y240 F3000 ; coarse home X or Y
G1 H1 X240 ; coarse home X
G1 H1 Y240 ; coarse home Y
G1 X-4 Y-4 F600 ; move away from the endstops
G1 H1 X10 ; fine home X
G1 H1 Y10 ; fine home Y


;G91               ; relative positioning
;G1 H1 X255 Y255 F1800  ; move quickly to X axis endstop and stop there (first pass)
;G1 X-5 Y-5 F1800     ; go back a few mm
;G1 H1 Y255 X255 F360   ; move slowly to X axis endstop once more (second pass)
;G92 X120.94         ; set x position to the center of bed (you may want to adjust this)
G90               ; absolute positioning

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
