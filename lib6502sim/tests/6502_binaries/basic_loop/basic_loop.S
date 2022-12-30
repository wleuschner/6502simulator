*=0

start:
NOP
LDA #$FF
CMP #$FF
BEQ eq
ASL
eq:
ROR
ROR
ADC #$FF
ADC #$01
CLC
ADC #$01
SBC #$01
SBC #$01
CLC
BCC no_carry
LDA #$08
no_carry:
JSR sub
LDA #$02
PHA
LDA #$00
PLA
INC data
DEC data
NOP
DEX
INY
INX
DEY
LDA start
STA data
LDA #$00
TAX
TAY
SEC
SEI
SED
CLC
CLI
CLD
CLV
JMP start

sub:
NOP
NOP
RTS

data:
!byte 0
