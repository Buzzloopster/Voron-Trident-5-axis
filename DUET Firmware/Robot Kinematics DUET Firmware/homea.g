
G91                     ; relative positioning
;G1 Z8 F800 H2          ; lift Z relative to current position
G1 H2 A10 F6000   ; lift U relative to current position
G1 H1 A-120 F1800 ; move U up until the endstop is triggered
G92 A-0.55
G90
G1 A0 F300