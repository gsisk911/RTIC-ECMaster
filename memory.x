/* Teensy 4.1 (i.MX RT1062) memory layout
 *
 * Default FlexRAM fuse configuration:
 *   ITCM  = 512 KB  (instruction tightly-coupled memory)
 *   DTCM  = 512 KB  (data tightly-coupled memory)
 *   OCRAM = 0 KB     (not banked by default)
 *
 * Flash = 8 MB (external QSPI via FlexSPI)
 *
 * teensy4-bsp / imxrt-rt linker scripts handle most details;
 * this file provides the MEMORY regions they expect.
 */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x60000000, LENGTH = 1984K
    DTCM  (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
    ITCM  (rwx) : ORIGIN = 0x00000000, LENGTH = 512K
    OCRAM (rwx) : ORIGIN = 0x20200000, LENGTH = 512K
}
