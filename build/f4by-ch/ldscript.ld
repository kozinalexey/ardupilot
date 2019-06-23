/* generated ldscript.ld */
MEMORY
{
    flash : org = 0x08004000, len = 1008K
    ram0  : org = 0x20000000, len = 128k
}

INCLUDE common.ld
