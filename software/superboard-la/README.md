# sb_la — host for the Superboard logic analyser

Windows only (WinUSB). MIT.

```
make
```

```
sb_la --list
sb_la --info
sb_la -r 2000000 -n 1M -o cap.bin
sb_la -t 2 -f csv -o cap.csv
```

```
include/la_protocol.h   wire protocol
src/sb_la.c             host
```
