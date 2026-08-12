# Superboard logic analyser — device firmware

AT32F405. MIT.

```
src/la_protocol.h          wire protocol
src/la_device.c/.h         protocol state machine
src/la_hardware.c          timer + DMA capture
src/composite_desc.c/.h    USB descriptors
src/composite_class.h      endpoint layout
```

Not a complete image: no `main.c`. Builds against Artery's AT32F402/405 SDK,
not redistributed here.

Host: [`../software/superboard-la`](../software/superboard-la)
