;############################################################################
;
; FILE:   DSP28xxx_SectionCopy_nonBIOS.asm
;
; DESCRIPTION:  Provides functionality for copying intialized sections from 
;				flash to ram at runtime before entering the _c_int00 startup
;				routine
;############################################################################
; Author: Tim Love
; Release Date: March 2008	
;############################################################################


	.ref _c_int00
;	.global copy_sections
;	.global _cinit_loadstart, _cinit_runstart, _cinit_size
;	.global _const_loadstart, _const_runstart, _const_size
;	.global _econst_loadstart, _econst_runstart, _econst_size
;	.global _pinit_loadstart, _pinit_runstart, _pinit_size
;	.global _switch_loadstart, _switch_runstart, _switch_size
;	.global _text_loadstart, _text_runstart, _text_size
	.global copy_sections
	.global _text_LoadStart, _text_RunStart, _text_Size
	.global _cinit_LoadStart, _cinit_RunStart, _cinit_Size
	.global _pinit_LoadStart, _pinit_RunStart, _pinit_Size
	.global _switch_LoadStart, _switch_RunStart, _switch_Size
	.global _econst_LoadStart, _econst_RunStart, _econst_Size
	.global _const_LoadStart, _const_RunStart, _const_Size
	
***********************************************************************
* Function: copy_sections
*
* Description: Copies initialized sections from flash to ram
***********************************************************************

	.sect "copysections"

copy_sections:
    MOVL XAR5, #_cinit_Size			; Store Section Size in XAR5
    MOVL ACC, @XAR5					; Move Section Size to ACC
    MOVL XAR6, #_cinit_LoadStart	; Store Load Starting Address in XAR6
    MOVL XAR7, #_cinit_RunStart		; Store Run Address in XAR7
	LCR  copy						; Branch to Copy

    MOVL XAR5, #_text_Size			; Store Section Size in XAR5
    MOVL ACC, @XAR5					; Move Section Size to ACC
    MOVL XAR6, #_text_LoadStart	; Store Load Starting Address in XAR6
    MOVL XAR7, #_text_RunStart		; Store Run Address in XAR7
	LCR  copy						; Branch to Copy

    MOVL XAR5, #_pinit_Size			; Store Section Size in XAR5
    MOVL ACC, @XAR5					; Move Section Size to ACC
    MOVL XAR6, #_pinit_LoadStart	; Store Load Starting Address in XAR6
    MOVL XAR7, #_pinit_RunStart		; Store Run Address in XAR7
	LCR  copy
							; Branch to Copy
    MOVL XAR5, #_switch_Size			; Store Section Size in XAR5
    MOVL ACC, @XAR5					; Move Section Size to ACC
    MOVL XAR6, #_switch_LoadStart	; Store Load Starting Address in XAR6
    MOVL XAR7, #_switch_RunStart		; Store Run Address in XAR7
	LCR  copy						; Branch to Copy

    MOVL XAR5, #_econst_Size			; Store Section Size in XAR5
    MOVL ACC, @XAR5					; Move Section Size to ACC
    MOVL XAR6, #_econst_LoadStart	; Store Load Starting Address in XAR6
    MOVL XAR7, #_econst_RunStart		; Store Run Address in XAR7
	LCR  copy						; Branch to Copy

    MOVL XAR5, #_const_Size			; Store Section Size in XAR5
    MOVL ACC, @XAR5					; Move Section Size to ACC
    MOVL XAR6, #_const_LoadStart	; Store Load Starting Address in XAR6
    MOVL XAR7, #_const_RunStart		; Store Run Address in XAR7
	LCR  copy						; Branch to Copy

    LB _c_int00         			; Branch to start of boot.asm in RTS library

;
;	MOVL XAR5,#_const_size				; Store Section Size in XAR5
;	MOVL ACC,@XAR5						; Move Section Size to ACC
;	MOVL XAR6,#_const_loadstart			; Store Load Starting Address in XAR6
;   MOVL XAR7,#_const_runstart			; Store Run Address in XAR7
;   LCR  copy							; Branch to Copy
; 
;	MOVL XAR5,#_econst_size				; Store Section Size in XAR5
;	MOVL ACC,@XAR5						; Move Section Size to ACC
;	MOVL XAR6,#_econst_loadstart		; Store Load Starting Address in XAR6
;   MOVL XAR7,#_econst_runstart			; Store Run Address in XAR7
;   LCR  copy							; Branch to Copy
;
;	MOVL XAR5,#_pinit_size				; Store Section Size in XAR5
;	MOVL ACC,@XAR5						; Move Section Size to ACC
;	MOVL XAR6,#_pinit_loadstart			; Store Load Starting Address in XAR6
;   MOVL XAR7,#_pinit_runstart			; Store Run Address in XAR7
;   LCR  copy							; Branch to Copy 
;
;	MOVL XAR5,#_switch_size				; Store Section Size in XAR5
;	MOVL ACC,@XAR5						; Move Section Size to ACC
;	MOVL XAR6,#_switch_loadstart		; Store Load Starting Address in XAR6
;   MOVL XAR7,#_switch_runstart			; Store Run Address in XAR7
;   LCR  copy							; Branch to Copy

;	MOVL XAR5,#_text_size				; Store Section Size in XAR5
;	MOVL ACC,@XAR5						; Move Section Size to ACC
;	MOVL XAR6,#_text_loadstart			; Store Load Starting Address in XAR6
;   MOVL XAR7,#_text_runstart			; Store Run Address in XAR7
;    LCR  copy							; Branch to Copy
    
;  	MOVL XAR5,#_cinit_size				; Store Section Size in XAR5
;	MOVL ACC,@XAR5						; Move Section Size to ACC
;	MOVL XAR6,#_cinit_loadstart			; Store Load Starting Address in XAR6
;   MOVL XAR7,#_cinit_runstart			; Store Run Address in XAR7
;   LCR  copy							; Branch to Copy 
;
;   LB _c_int00				 			; Branch to start of boot.asm in RTS library



copy:	
	B return,EQ							; Return if ACC is Zero (No section to copy)

    RPT AL								; Copy Section From Load Address to
    || PWRITE  *XAR7, *XAR6++			; Run Address

return:
	LRETR								; Return

	.end
	
;//===========================================================================
;// End of file.
;//===========================================================================