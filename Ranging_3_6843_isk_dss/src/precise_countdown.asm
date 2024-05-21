    .global precision_countdown

    .text
    .align  4
precision_countdown:

    ; Function body
    MV      A4, A0          	; Operate on A0

    ; Fine tuning value, determined by testing and analysis
    SUB A0, 30, A0
    SUB A0, 11, A0

; Loop until A0 is below 42.
; Each loop decrements by 8, which is the number of clock cycles
    MVK     41, A3
start:
    SUB     A0, 8, A0          ; Decrement the counter (A0 = A0 - 8)
    CMPGT   A0, A3, A1         ; Compare A0 with 41, set A1 if A0 > 41
    [A1]    B start            ; If A0 > 41, branch to start

; At this point A0 holds a value between 34 and 40, inclusive


; Handle A0 starting at 34
    NOP     5                  ; Delay slot fill (5 cycles)
    MVK     35, A2			   ; 1 Cycle
    CMPLT   A0, A2, A1         ; Compare A0 with 35, set A1 if A0 < 35
    [A1]    B exit_delay_34    ; If A0 < 35, branch to exit_delay_35

; Handle A0 starting at 35
    NOP     5                  ; Delay slot fill (5 cycles)
    MVK     36, A2			   ; 1 Cycle
    CMPLT   A0, A2, A1         ; Compare A0 with 31, set A1 if A0 < 31
    [A1]    B exit_delay_35    ; If A0 < 31, branch to exit_delay_31

; Handle A0 starting at 36
    NOP     5                  ; Delay slot fill (5 cycles)
    MVK     37, A2
    CMPLT   A0, A2, A1         ; Compare A0 with 27, set A1 if A0 < 27
    [A1]    B exit_delay_36    ; If A0 < 27, branch to exit_delay_27

; Handle A0 starting at 37
    NOP     5                  ; Delay slot fill (5 cycles)
    MVK     38, A2
    CMPLT   A0, A2, A1         ; Compare A0 with 23, set A1 if A0 < 23
    [A1]    B exit_delay_37    ; If A0 < 23, branch to exit_delay_23

; Handle A0 starting at 38
    NOP     5                  ; Delay slot fill (5 cycles)
    MVK     39, A2
    CMPLT   A0, A2, A1         ; Compare A0 with 19, set A1 if A0 < 19
    [A1]    B exit_delay_38    ; If A0 < 19, branch to exit_delay_19

; Handle A0 starting at 39
    NOP     5                  ; Delay slot fill (5 cycles)
    MVK     40, A2
    CMPLT   A0, A2, A1         ; Compare A0 with 15, set A1 if A0 < 15
    [A1]    B exit_delay_39    ; If A0 < 15, branch to exit_delay_15

; Handle A0 starting at 40
    NOP     5                  ; Delay slot fill (5 cycles)
    MVK     41, A2
    CMPLT   A0, A2, A1         ; Compare A0 with 11, set A1 if A0 < 11
    [A1]    B exit_delay_40    ; If A0 < 11, branch to exit_delay_11

; Handle A0 starting at 41
    NOP     5                  ; Delay slot fill (5 cycles)
    MVK     42, A2
    CMPLT   A0, A2, A1         ; Compare A0 with 11, set A1 if A0 < 11
    [A1]    B exit_delay_41    ; If A0 < 11, branch to exit_delay_11
    NOP     5                  ; Delay slot fill (5 cycles)

exit_delay_34:
	NOP		9		; total delay 50
	NOP		9
	NOP		9
	NOP		9
	NOP		8
	NOP		6
    B       B3      ; Return from subroutine
    NOP		5

exit_delay_35:
	NOP		9		; total delay 43
	NOP		9
	NOP		9
	NOP		9
	NOP		1
	NOP		6
    B       B3    	; Return from subroutine
    NOP		5

exit_delay_36:
	NOP		9		; total delay 36
	NOP		9
	NOP		9
	NOP		3
	NOP		6
    B       B3     	; Return from subroutine
    NOP		5

exit_delay_37:
	NOP		9		; total delay 29
	NOP		9
	NOP		5
	NOP		6
    B       B3    	; Return from subroutine
    NOP		5

exit_delay_38:
	NOP		9		; total delay 22
	NOP		7
	NOP		6
    B       B3     	; Return from subroutine
    NOP		5

exit_delay_39:
	NOP		9		; total delay 15
	NOP		6
    B       B3 		; Return from subroutine
    NOP		5

exit_delay_40:
	NOP		8		; total delay 8
    B       B3    	; Return from subroutine
    NOP		5

exit_delay_41:
	NOP		1		; total delay 1
    B       B3    	; Return from subroutine
    NOP		5

; Loop until A0 is below 42.
; Each loop decrements by 8, which is the number of clock cycles if the loop is taken.
;start:
;    SUB     A0, 8, A0          ; Decrement the counter (A0 = A0 - 8)
;    CMPGT   A0, A3, A1         ; Compare A0 with 41, set A1 if A0 > 41
;    [A1]    B start            ; If A0 > 41, branch to start

; At this point A0 holds a value between 34 and 40, inclusive


; Handle A0 starting at 34
;    NOP     4                  ; Delay slot fill (4 cycles)
;    MVK     35, A2			   ; 1 Cycle
;    CMPLT   A0, A2, A1         ; Compare A0 with 35, set A1 if A0 < 35
;    [A1]    B exit_delay_34    ; If A0 < 35, branch to exit_delay_35

; Handle A0 starting at 35
;    NOP     4                  ; Delay slot fill (3 cycles)
;    MVK     36, A2			   ; 1 Cycle
;    CMPLT   A0, A2, A1         ; Compare A0 with 31, set A1 if A0 < 31
;    [A1]    B exit_delay_35    ; If A0 < 31, branch to exit_delay_31

; Handle A0 starting at 36
;    NOP     4                  ; Delay slot fill (5 cycles)
;    MVK     37, A2
;    CMPLT   A0, A2, A1         ; Compare A0 with 27, set A1 if A0 < 27
;    [A1]    B exit_delay_36    ; If A0 < 27, branch to exit_delay_27

; Handle A0 starting at 37
;    NOP     4                  ; Delay slot fill (5 cycles)
;    MVK     38, A2
;    CMPLT   A0, A2, A1         ; Compare A0 with 23, set A1 if A0 < 23
;    [A1]    B exit_delay_37    ; If A0 < 23, branch to exit_delay_23

; Handle A0 starting at 38
;    NOP     4                  ; Delay slot fill (5 cycles)
;    MVK     39, A2
;    CMPLT   A0, A2, A1         ; Compare A0 with 19, set A1 if A0 < 19
;    [A1]    B exit_delay_38    ; If A0 < 19, branch to exit_delay_19

; Handle A0 starting at 39
;    NOP     4                  ; Delay slot fill (5 cycles)
;    MVK     40, A2
;    CMPLT   A0, A2, A1         ; Compare A0 with 15, set A1 if A0 < 15
;    [A1]    B exit_delay_39    ; If A0 < 15, branch to exit_delay_15

; Handle A0 starting at 40
;    NOP     4                  ; Delay slot fill (5 cycles)
;    MVK     41, A2
;    CMPLT   A0, A2, A1         ; Compare A0 with 11, set A1 if A0 < 11
;    [A1]    B exit_delay_40    ; If A0 < 11, branch to exit_delay_11

; Handle A0 starting at 41
;    NOP     4                  ; Delay slot fill (5 cycles)
;    MVK     42, A2
;    CMPLT   A0, A2, A1         ; Compare A0 with 11, set A1 if A0 < 11
;    [A1]    B exit_delay_41    ; If A0 < 11, branch to exit_delay_11

;exit_delay_34:
;	NOP		9
;	NOP		9
;	NOP		9
;	NOP		9
;	NOP		9		; total delay 45
;   B       B3      ; Return from subroutine
;    NOP		5

;exit_delay_35:
;	NOP		9
;	NOP		9
;	NOP		9
;	NOP		9
;	NOP		3		; total delay 39
;    B       B3    	; Return from subroutine
;    NOP		5

;exit_delay_36:
;	NOP		9
;	NOP		9
;	NOP		9
;	NOP		6		; total delay 33
;    B       B3     	; Return from subroutine
;    NOP		5

;exit_delay_37:
;	NOP		9
;	NOP		9
;	NOP		9		; total delay 27
 ;   B       B3    	; Return from subroutine
 ;   NOP		5

;exit_delay_38:
;	NOP		9
;	NOP		9
;	NOP		3		; total delay 21
 ;   B       B3     	; Return from subroutine
  ;  NOP		5

;exit_delay_39:
;	NOP		9
;	NOP		6		; total delay 15
 ;   B       B3 		; Return from subroutine
  ;  NOP		5

;exit_delay_40:
;	NOP		9		; total delay 9
 ;   B       B3    	; Return from subroutine
  ;  NOP		5

;exit_delay_41:
;	NOP		9		; total delay 3
 ;   B       B3    	; Return from subroutine
  ;  NOP		5

