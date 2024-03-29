/*
   Copyright (c) 1992 - 1994 Heinz W. Werntges.  All rights reserved.
   Parts Copyright (c) 1999  Martin Kroeker  All rights reserved.
   
   Distributed by Free Software Foundation, Inc.

This file is part of HP2xx.

HP2xx is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY.  No author or distributor accepts responsibility
to anyone for the consequences of using it or for whether it serves any
particular purpose or works at all, unless he says so in writing.  Refer
to the GNU General Public License, Version 2 or later, for full details.

Everyone is granted permission to copy, modify and redistribute
HP2xx, but only under the conditions described in the GNU General Public
License.  A copy of this license is supposed to have been
given to you along with HP2xx so you can know your rights and
responsibilities.  It should be in a file named COPYING.  Among other
things, the copyright notice and this notice must be preserved on all
copies.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
*/

/**
 ** This file defines a standard character set by elementary
 ** "draw" & "move" commands. The format is a very compact one from
 ** the old days where every byte was still appreciated.
 **
 ** A font or character set is an array of strings. Each character
 ** corresponds to one of these strings, which is addressed by its ASCII code.
 **
 ** A character is a (NULL-terminated) string of bytes. Each byte
 ** codes for a draw or move action according to the code below:
 **
 ** Bit:    7 6 5 4 3 2 1 0
 **     p x x x y y y y
 **
 ** p:  Plot flag. If set, "draw to" new point, else "move to" it.
 ** xxx:    3-bit unsigned integer  (0...7). X coordinate of new point.
 ** yyyy:   4-bit unsigned integer (0..15). Y coordinate of new point.
 **
 ** The baseline is y = 4 instead of y = 0, so characters with parts
 ** below it can be drawn properly without a need for sign bits.
 ** Function "code_to_ucoord()" transforms these coordinates into
 ** actual user coordinates.
 **
 ** Example:    code for character 'L': "\032\224\324" translates to:
 **     moveto(1,10); drawto(1,4); drawto(5,4);
 **
 ** From the example you can conclude that the font below essentially is
 ** defined on a 5x7 grid:
 **
 **     0 1 2 3 4 5 6 7
 ** 15  . . . . . . . .     . : unused
 ** 14  . . . . . . . .     * : always used
 ** 13  . . . . . . . .     o : sometimes used
 ** 12  . . . . . . . .
 ** 11  . . . . . . . .
 ** 10  o * * * * * . .
 **  9  o * * * * * . .
 **  8  o * * * * * . .
 **  7  o * * * * * . .
 **  6  o * * * * * . .
 **  5  o * * * * * . .
 **  4  o * * * * * . .
 **  3  o o o o o o . .
 **  2  o o o o o o . .
 **  1  o o o o o o . .
 **  0  o o o o o o . .
 **/


/**
 ** The following array of strings contains the basic character set (set 0).
 **
 ** NOTE: A nice way to add a new charset would be, e. g., to introduce a
 ** ``charset1[]'' as the "alternate" charset and implement the HP-GL
 ** commands needed for switching from one to the other.
 **/

const char *const charset0[256] = {
    /* 0x00 ... 0x1f        */

/**
 ** Some control codes are valid in HPGL. These are handled elsewhere
 ** in a font-independent manner, so following codes are dummies:
 **/
    "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
    "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",

/**
 ** Unfortunately, some compilers do not process \xNN properly,
 ** so I changed all hex codes (\xNN) into octal codes (\NNN),
 ** thereby losing readability but gaining portability.
 **/

    /* 0x20 ... 0x2f        */
    "",
    "\064\265\066\272",
    "\051\252\111\312",
    "\044\252\104\312\026\326\030\330",
    "\064\272\131\251\230\247\307\326\305\225",
    "\024\332\051\250\270\271\251\066\265\305\306\266",
    "\124\230\231\252\271\270\226\225\244\264\326",
    "\071\312",
    "\132\270\266\324",
    "\024\266\270\232",
    "\005\351\145\211\072\264",
    "\065\271\027\327",
    "\064\244\245\265\263\242",
    "\027\327",
    "\064\244\245\265\264",
    "\352",

    /* 0x30 ... 0x3f        */
/*
"\025\244\304\325\331\312\252\231\225\331", ** Zero including `/' **
*/
    "\025\244\304\325\331\312\252\231\225",
    "\044\304\064\272\251",
    "\031\252\312\331\330\225\224\324",
    "\025\244\304\325\326\307\267\332\232",
    "\112\227\226\326\107\304",
    "\132\232\230\310\327\325\304\244\225",
    "\132\272\230\225\244\304\325\326\307\227",
    "\032\332\331\226\224",
    "\107\330\331\312\252\231\230\247\307\326\325\304\244\225\226\247",
    "\044\264\326\331\312\252\231\230\247\327",
    "\047\250\270\267\247\045\265\264\244\245",
    "\046\247\267\266\246\064\244\245\265\263\242",
    "\112\227\304",
    "\030\330\026\326",
    "\032\307\224",
    "\031\252\312\331\330\307\267\266\065\264",

    /* 0x40 ... 0x4f        */
    "\103\243\224\230\252\312\331\326\305\266\267\310\330",
    "\024\231\252\312\331\324\026\326",
    "\024\232\312\331\330\307\227\024\304\325\326\307",
    "\125\304\244\225\231\252\312\331",
    "\024\232\312\331\325\304\224",
    "\124\224\232\332\027\307",
    "\024\232\332\027\307",
    "\131\312\252\231\225\244\304\325\327\247",
    "\024\232\124\332\027\327",
    "\024\324\064\272\032\332",
    "\025\244\304\325\332\232",
    "\024\232\027\247\324\047\332",
    "\032\224\324",
    "\024\232\270\332\324",
    "\024\232\324\332",
    "\044\225\231\252\312\331\325\304\244",

    /* 0x50 ... 0x5f        */
    "\024\232\312\331\330\307\227",
    "\044\225\231\252\312\331\326\264\244\066\324",
    "\024\232\312\331\330\307\227\247\324",
    "\025\244\304\325\326\307\247\230\231\252\312\331",
    "\064\272\232\332",
    "\032\225\244\304\325\332",
    "\032\230\264\330\332",
    "\032\224\267\324\332",
    "\024\332\124\232",
    "\032\231\266\264\066\331\332",
    "\032\332\224\324",
    "\124\264\272\332",
    "\032\324",
    "\024\264\272\232",
    "\030\272\330",
    "\023\323",

    /* 0x60 ... 0x6f        */
    "\053\310",
    "\124\244\225\227\250\310\304",
    "\024\304\325\327\310\250\052\244",
    "\125\304\264\245\247\270\310\327",
    "\112\304\244\225\227\250\310\104\324",
    "\026\306\327\310\250\227\225\244\324",
    "\064\271\312\332\047\307",
    "\022\262\303\310\250\227\225\244\304",
    "\032\224\030\270\307\304",
    "\072\271\050\270\264\044\304",
    "\072\271\050\270\263\242\222",
    "\024\232\104\226\310",
    "\052\272\264\044\304",
    "\024\230\027\250\267\264\067\310\327\324",
    "\024\230\027\250\270\307\304",
    "\044\225\227\250\270\307\305\264\244",

    /* 0x70 ... 0x7f        */
    "\022\230\270\307\305\264\224",
    "\104\244\225\227\250\310\302",
    "\030\224\026\270\310",
    "\110\250\227\246\266\305\264\224",
    "\052\244\304\030\310",
    "\030\225\244\304\310",
    "\030\226\264\326\330",
    "\030\225\244\265\267\065\304\325\330",
    "\030\324\024\330",
    "\022\326\330\030\226\264",
    "\030\310\224\304",
    "\113\273\252\250\227\246\244\263\303",
    "\073\263",
    "\053\273\312\310\327\306\304\263\243",
    "\031\252\310\331",
    ""
};
