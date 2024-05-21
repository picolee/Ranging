
    .global precision_count_until

precision_count_until:
    ; Function body
    MV      A4, A0          	; A4 contains the target value of TSCL we want to count to
	MVC 	TSCL, B0         	; Move the current value of TSCL into register B0

    ; Compute the # cycles between A0 (target TSCL value) and B0 (current TSCL value)
    ; Handle the case where the target value (A0) is smaller than the current value (A1)
    SUB     A0, B0, A2      ; A2 = target - current
    CMPGT   B0, A0, A1      ; A1 = (current > target)

	; If the current value (B0) is greater than the target (A0), it means rollover
    [A1]    ADD A2, 0xFFFFFFFF, A2 ; Adjust for rollover (A2 = A2 + 0xFFFFFFFF + 1)
    [A1]    ADD A2, 1, A2

    ; Subtract the ten cycles that elapse between loading the TSCL register and starting the countdown
    SUB A2, 10, A2

    ; Fine tuning value, determined by testing and analysis
    SUB A2, 30, A2

    ; Here A2 contains the difference considering possible rollover, and compensated for elapsed cycles
    MV      A2, A0          ; Move the result back to A0

; Loop until A0 is below 42.
; Each loop decrements by 8, which is the number of clock cycles if the loop is taken.
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
	NOP		7		; total delay 7 - testing shows we need 7
    B       B3    	; Return from subroutine
    NOP		5

exit_delay_41:
;	NOP		1		; Testing shows we need 0 delay here
    B       B3    	; Return from subroutine
    NOP		5

