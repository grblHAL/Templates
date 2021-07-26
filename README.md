## grblHAL templates

Template code to aid writing custom code for extensions such as additional IO-pins or M-codes by publishing entry points in the HAL function pointer structs.

* [ioports.c](./ioports.c) - for additional digital and/or analog outputs. For enabling M62 - M68 M-codes.

* [mcodes.c](./mcodes.c) - for additional M-codes, includes example code.

* [my_plugin](my_plugin) folder - contains some plugin examples.

The HAL supports a wide range of extension possibilities, this without touching the core grbl codebase. Some examples can be found in the [plugins](../plugins) folder.

---
2021-07-26
