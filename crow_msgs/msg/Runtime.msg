string section
builtin_interfaces/Time stamp
uint8 action
int8 severity

uint8 A_NOP=0
uint8 A_ENTER=1  # enter section (overwrite as per set default behaviour)
uint8 A_TRY_ENTER=2  # if section already entered, do nothing
uint8 A_REENTER=4  # overwrites current section enter time
uint8 A_EXIT=8  # exits section, error if wasn't entered
uint8 A_EXIT_IF_IN=16  # only exit if already in (no error if section was not entered)
uint8 A_CANCEL=128  # erases section entering time

int8 S_MAIN=100
int8 S_IMPORTANT=80
int8 S_MAJOR=50
int8 S_NORMAL=0
int8 S_MINOR=-50
int8 S_DETAIL=-80
int8 S_SINGLE_LINE=-90
int8 S_DEBUG=-100