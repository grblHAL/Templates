## Modbus command

** EXPERIMENTAL **

This plugin implements the `$MODBUSCMD` system command. 
The following Modbus functions are supported with the following syntax:

Functions 1-4, read many:  
`$MODBUSCMD=<modbus address>,<function>,<register address base>{,<number of registers>}`  
`<number of registers>` defaults to 1 if not specified, max 3.  
Returns number of registers read along with their values.  

Functions 5-6, write:  
`$MODBUSCMD=<modbus address>,<function>,<register address>,<value>}`  
`<number of registers>` defaults to one if not specified, max 3.  
Returns number of registers written.  

Functions 7, get exception status:  
`$MODBUSCMD=<modbus address>,7`  
Returns exception status  

Functions 15-16, write many:  
`$MODBUSCMD=<modbus address>,<function>,<register address base>,<value>{,<value>{,<value>}}`  

Both decimal and hexadecimal arguments can be used. Some examples:
```
$MODBUSCMD=1,4,0,2          // Read status register from a H100 VFD
$MODBUSCMD=1,3,0x200B       // Read status register from a YL620 VFD
$MODBUSCMD=1,6,0x0201,1000  // Set frequency register on a H100 VFD
```
---
2026-02-16
