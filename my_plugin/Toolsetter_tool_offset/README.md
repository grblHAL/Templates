## Toolsetter tool offset

For probing odd shaped large radius tools @ G59.3. The radius offset is used to move the controlled point in the X direction over the toolsetter.

Adds `M102 P- T- R-` command for binding tool radius to tool number for up to five tools:

`P` - tool radius table entry, range 0 - 4.  
`T` - tool number, use tool number 0 to disable an entry.  
`R` - radius.

Adds `[TRA:P-,T-,R-]` element to the `$#` report for each valid tool table entry.

The tool radius table is persistent.

---
2026-02-16
