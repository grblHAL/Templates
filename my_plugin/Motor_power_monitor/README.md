## Stepper motor power monitor

On power loss alarm 17 is raised (Motor fault).
On alarm cleared or soft reset a `M122I` command is issued to reinit Trinamic drivers if power is back on.

Setting `$450` is for configuring which aux input port to assign for monitoring.

> [!NOTE]
> If the driver does not support mapping of port number settings `$450` will not be available.
The mapped pin has to be interrupt capable and support change (falling and rising) interrupt mode.

Tip: use the `$pins` command to check the port mapping.

---
2026-02-16
