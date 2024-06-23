; Default config.g template for DuetPi
; Replace this with a proper configuration file (e.g from https://configtool.reprapfirmware.org)

; Display initial welcome message
;M291 P"Please go to <a href=""https://www.duet3d.com/StartHere"" target=""_blank"">this</a> page for further instructions on how to set it up." R"Welcome to your new Duet 3!" S1 T0

; Enable network
if {network.interfaces[0].type = "ethernet"}
    M552 P192.168.50.50 S1 ; P169.254.115.150 S1
else
    M552 S1
    
	
; Enable PanelDue
 M575 P1 S1 B57600


; General preferences
G90                                         ; Send absolute coordinates...
M83                                         ; ...but relative extruder moves
;M555 P2 									; Set firmware compatibility to look like Marlin

; set printer name
M550 P"5 Axis Trident"    

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

; Heaters BED
M308 S0 P"temp0" Y"thermistor" T100000 B3950 		; configure sensor 0 as thermistor on pin temp0
M950 H0 C"out0" T0                           		; create bed heater output on out0 and map it to sensor 0
M307 H0 R0.380 K0.410:0.000 D7.86 E1.35 S1.00 B0	; disable bang-bang mode for the bed heater and set PWM limit
M140 H0                                      		; map heated bed to heater 0
M143 H0 S120                                 		; set temperature limit for heater 0 to 120C


; Heaters HOTEND
M308 S1 P"temp1" Y"thermistor" T100000 B4725 C7.06e-8 ; configure sensor 1 as thermistor on pin temp1
M950 H1 C"out1" T1   						 ; create nozzle heater output on out1 and map it to sensor 1
M307 H1 R4.250 K1.828:0.000 D4.91 E1.35 S1.00 B0 V24.1  ; disable bang-bang mode for heater  and set PWM limit
M143 H1 S280                                 ; set temperature limit for heater 1 to 280C


; Drive Mappings
M569 P0 S1                                  ; Drive 1 goes forwards: X Axis
M569 P1 S1                                  ; Drive 1 goes forwards: Y Axis
M569 P2 S0                                  ; Drive 2 goes backwards: Z Axis
M569 P3 S0                                  ; Drive 3 goes backwards: E0
M569 P4 S0                                  ; Drive 4 goes anti-clockwise: A Axis
M569 P5 S1				    				; Drive 5 goes anti-clockwise: C Axis


M584 X0 Y1 Z2 E3 A4 C5 ; My Driver Mapping


;Microstepping and Speed

M350 X32 Y32 Z16 E16 A16 C16 I1                 			; Configure microstepping with interpolation

M92 X320.00 Y320.00 Z80.00 E720.0 A90.0 C68.885 		    ; Set steps per mm and steps per degree OLD VALUE E654.5
M566 X1200.00 Y1200.00 Z240.00 E270.00 A480.00 C480.00 P1     	    ; Set maximum instantaneous speed changes (mm/min)
M203 X3000.00 Y3000.00 Z2000.00 E2500.00 A2000.00 C6000.00  	    ; Set maximum speeds (mm/min)
M201 X1000.00 Y1000.00 Z1000.00 E1000.00 A1000.00 C1500.00   	    ; Set accelerations (mm/s^2)
M906 X650.00 Y650.00 Z650.00 E600.00 A2000.00 C600.00 I10            ; Set motor currents (mA) and motor idle factor in percent
M84 S3000                                    			    ; Set idle timeout


;Axis Limit
;Centre of the rotating bed is set to origin 0,0
;M208 X-110 Y-120 Z0 S1		;A-90 C-12000 S1 	; set axis minima
;M208 X125 Y109.5 Z127 S0 	;A90 C12000 S0 		; set axis maxima

;M564 H0								; allow unhomed movement


; Endstops
M574 X2 S1 P"!io1.in"
M574 Y2 S1 P"!io2.in"
M574 Z2 S1 P"io3.in"
M574 A1 S1 P"io4.in"
M574 C1 S3 

; Stallgaurd Sensitivy
M915 C S3 F0 H500 R1		                ; Set C axis Sensitivity


; Fans
M950 F0 C"out5" Q500                ; create fan 0 on pin out7 and set its frequency
M106 P0 S0 H-1                      ; set fan 0 value. Thermostatic control is
M950 F1 C"out7" Q500				; Creates HOTEND Fan
M106 P1 T45 S255 H1                 ; HOTEND Fan Settings
M950 F2 C"out8" Q500				; Creates CASE Fan 2
M106 P2  S55 H-1                 	; HOTEND Fan Settings
M950 F3 C"out9" Q500                ; create case fan 3 on pin out9 and set its frequency
M106 P3  S0 H-1                     ; set fan 3 value. Thermostatic control is turned ON

; Tools
M563 P0 D0 H1 F0                                      ; define tool 0
G10 P0 X0 Y0 Z0                                       ; set tool 0 axis offsets
G10 P0 R0 S0                                          ; set initial tool 0 active and standby temperatures to 0C
