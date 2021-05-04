string section
builtin_interfaces/Time stamp
uint8 action

uint8 A_NOP=0
uint8 A_ENTER=1  # enter section (overwrite as per set default behaviour)
uint8 A_TRY_ENTER=2  # if section already entered, do nothing
uint8 A_REENTER=4  # overwrites current section enter time
uint8 A_EXIT=8  # exits section, error if wasn't entered
uint8 A_EXIT_IF_IN=16  # only exit if already in (no error if section was not entered)
uint8 A_CANCEL=128  # erases section entering time