## Modbus command

** EXPERIMENTAL **

This plugin implements the `$MODBUSCMD` and `$MODBUSDBG` system commands.

`$MODBUSCMD` is for sending messages and the following Modbus functions are supported:

Functions 1-4, read many:  
`$MODBUSCMD=<server address>,<function>,<register address base>{,<number of registers>}`  
`<number of registers>` defaults to 1 if not specified, max 3.  
Returns number of registers read along with their values.  

Functions 5-6, write:  
`$MODBUSCMD=<server address>,<function>,<register address>,<value>}`  
`<number of registers>` defaults to one if not specified, max 3.  
Returns number of registers written.  

Functions 7, get exception status:  
`$MODBUSCMD=<server address>,7`  
Returns exception status  

Functions 15-16, write many:  
`$MODBUSCMD=<server address>,<function>,<register address base>,<value>{,<value>{,<value>}}`  

Both decimal and hexadecimal arguments can be used. Some examples:
```
$MODBUSCMD=1,4,0,2          // Read status register from a H100 VFD
$MODBUSCMD=1,3,0x200B       // Read status register from a YL620 VFD
$MODBUSCMD=1,6,0x0201,1000  // Set frequency register on a H100 VFD
```

---

`$MODBUSDBG` - enable debug output, this outputs messages containing the transmitted and received data, an example:
```
[MSG:TX: 01 03 21 03 00 01 7E 36]
[MSG:RX: 01 03 02 00 00 B8 44]
```

`$MODBUSDBG=0` - disable debug output.

---
2026-02-16
