/*----------------------------------------------------------------------------*/
/* r4f_linker.cmd                                                                 */
/*                                                                            */
/* (c) Texas Instruments 2016, All rights reserved.                           */
/*                                                                            */

/* USER CODE BEGIN (0) */
/* USER CODE END */


/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
--retain="*(.intvecs)"

/*----------------------------------------------------------------------------*/
/* Memory Map                                                                 */
#define MMWAVE_L3RAM_SIZE (MMWAVE_L3RAM_NUM_BANK*MMWAVE_SHMEM_BANK_SIZE)
MEMORY{
PAGE 0:
    VECTORS  (X)  : origin=0x00000000 length=0x00000100
    PROG_RAM (RX) : origin=0x00000100 length=0x0007FF00+(MMWAVE_SHMEM_TCMA_NUM_BANK*MMWAVE_SHMEM_BANK_SIZE)
    DATA_RAM (RW) : origin=0x08000000 length=0x00030000+(MMWAVE_SHMEM_TCMB_NUM_BANK*MMWAVE_SHMEM_BANK_SIZE)
    L3_RAM (RW)   : origin=0x51000000 length=MMWAVE_L3RAM_SIZE
    HWA_RAM (RW)  : origin=0x52030000 length=0x00010000
    HS_RAM (RW)   : origin=0x52080000 length=0x8000
PAGE 1:
    L3_RAM (RW)   : origin=0x51000000 length=MMWAVE_L3RAM_SIZE
}

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */
SECTIONS{
    .intvecs : {} > VECTORS
    .text    : {} > PROG_RAM
    .const   : {} > PROG_RAM
    .cinit   : {} > PROG_RAM
    .pinit   : {} > PROG_RAM
    .bss     : {} > DATA_RAM
    .data    : {} > DATA_RAM
    .stack   : {} > DATA_RAM
}
/*----------------------------------------------------------------------------*/

