;
; m328-GPS-clock.asm
;
; Created: 7/16/2016 12:31:18 PM
; Author : lynf
;
;
;######################################################################################
; This software is Copyright by Francis Lyn and is issued under the following license:
;
; Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License
;
;######################################################################################
;
;
; This file is used as a folder for all versions of m328-GPS-clock.asm to
; assemble, link and test the program.
;
; Notes:
; ======
;
; Must have TWI pull-up termination resistors installed else
; interface will not work!
;
.include	"m328-GPS-clock(testing).asm"
;
;
.exit
;
