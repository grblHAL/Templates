## Mot&ouml;ri the Plotter as a grblHAL plugin

Original code by [Viacheslav Slavinsky](http://sensi.org/~svo/motori/), modified and refactored for grblHAL.

This plugin adds a HPGL interpreter to grblHAL making grblHAL multilingual.  
No changes to the grblHAL code was needed to add this plugin!

The `$HPGL` command disables the grblHAL gcode interpreter and activates the HPGL interpreter lifted from Mot&ouml;ri.
Use `<CTRL>+<X>` to exit back to normal operation.

I made the plugin for my [C.ITOH CX-600 plotter](https://hackaday.io/project/183600-citoh-cx-6000-plotter-upgrade) and as an example for how the grblHAL APIs can be used.

---
2022-01-15
