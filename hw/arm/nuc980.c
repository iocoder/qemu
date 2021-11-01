/*
 * ARM Versatile Platform/Application Baseboard System emulation.
 *
 * Copyright (c) 2005-2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "qemu/error-report.h"
#include "qemu/datadir.h"
#include "qom/object.h"
#include "cpu.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/arm/boot.h"
#include "hw/char/serial.h"
#include "hw/block/flash.h"
#include "hw/ssi/ssi.h"

/*****************************************************************************/
/*                               BOOT FLASH                                  */
/*****************************************************************************/

static uint8_t nuc980_boot_flash[32*1024*1024];

/*****************************************************************************/
/*                             SYSTEM CONTROLLER                             */
/*****************************************************************************/

/*------------------------------- SYS MACROS --------------------------------*/

#define TYPE_NUC980_SYS "nuc980-sys"

#define REG_SYS_PDID       0x000
#define REG_SYS_PWRON      0x004
#define REG_SYS_ARBCON     0x008
#define REG_SYS_LVRDCR     0x020
#define REG_SYS_MISCFCR    0x030
#define REG_SYS_MISCIER    0x040
#define REG_SYS_MISCISR    0x044
#define REG_SYS_ROMSUM0    0x048
#define REG_SYS_ROMSUM1    0x04C
#define REG_SYS_WKUPSER    0x058
#define REG_SYS_WKUPSSR    0x05C
#define REG_SYS_AHBIPRST   0x060
#define REG_SYS_APBIPRST0  0x064
#define REG_SYS_APBIPRST1  0x068
#define REG_SYS_RSTSTS     0x06C
#define REG_SYS_MFP_GPA_L  0x070
#define REG_SYS_MFP_GPA_H  0x074
#define REG_SYS_MFP_GPB_L  0x078
#define REG_SYS_MFP_GPB_H  0x07C
#define REG_SYS_MFP_GPC_L  0x080
#define REG_SYS_MFP_GPC_H  0x084
#define REG_SYS_MFP_GPD_L  0x088
#define REG_SYS_MFP_GPD_H  0x08C
#define REG_SYS_MFP_GPE_L  0x090
#define REG_SYS_MFP_GPE_H  0x094
#define REG_SYS_MFP_GPF_L  0x098
#define REG_SYS_MFP_GPF_H  0x09C
#define REG_SYS_MFP_GPG_L  0x0A0
#define REG_SYS_MFP_GPG_H  0x0A4
#define REG_SYS_MFP_GPH_L  0x0A8
#define REG_SYS_MFP_GPH_H  0x0AC
#define REG_SYS_MFP_GPI_L  0x0B0
#define REG_SYS_MFP_GPI_H  0x0B4
#define REG_SYS_DDR_DS_CR  0x0E0
#define REG_SYS_WPCTL      0x1FC

/*---------------------------- SYS DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980SYSState, NUC980_SYS)

struct NUC980SYSState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    uint32_t      regs[256];
};

/*----------------------------- SYS FUNCTIONS -------------------------------*/

static uint64_t nuc980_sys_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980SYSState *sys = opaque;
    uint64_t        ret = 0;

    ret = sys->regs[(addr>>2)&0xFF];

    switch(addr) {
      case REG_SYS_PWRON:
        /* boot source:          0b10  (NAND flash) 
         * QSPI clock:           0b0   (37.5MHz)
         * Watchdog:             0b0   (disabled)
         * JTAG:                 0b1   (PinG)
         * UART debug:           0b1   (OFF)
         * NAND flash page size: 0b11  (ignore)
         * MISC config:          0b11  (ignore)
         * USB ID:               0b0   (USB device)
         * TIC Mode:             0b0   (Disabled)
         * DRAM Size:            0b110 (64MB)
         */
        ret = 0x004003F2;
        break;

      case REG_SYS_WPCTL:
        ret = 1;
        break;
        
      default:
        error_report("SYS RD: 0x%08lX --> 0x%08lX", sys->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_sys_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980SYSState *sys = opaque;

    sys->regs[(addr>>2)&0xFF] = value;

    switch(addr) {
      default:
        error_report("SYS WR: 0x%08lX <-- 0x%08lX", sys->iomem.addr + addr, value);
        break;
    }
}

static void nuc980_sys_instance_init(Object *obj)
{
    NUC980SYSState *sys = NUC980_SYS(obj);
    
    static const MemoryRegionOps sys_ops = {
      .read       = nuc980_sys_read,
      .write      = nuc980_sys_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&sys->iomem, obj, &sys_ops, sys, "sys", 0x1000);

    sysbus_init_mmio(SYS_BUS_DEVICE(sys), &sys->iomem);
}

static void nuc980_sys_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 SYS Controller";
    dev_class->reset = NULL;
}

/*------------------------------ SYS TYPE -----------------------------------*/

static const TypeInfo nuc980_sys_type = {
    .name          = TYPE_NUC980_SYS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980SYSState),
    .instance_init = nuc980_sys_instance_init,
    .class_init    = nuc980_sys_class_init,
};

/*****************************************************************************/
/*                             CLOCK CONTROLLER                              */
/*****************************************************************************/

/*------------------------------- CLK MACROS --------------------------------*/

#define TYPE_NUC980_CLK "nuc980-clk"

/*---------------------------- CLK DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980CLKState, NUC980_CLK)

struct NUC980CLKState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    uint32_t      regs[32];
};

/*----------------------------- CLK FUNCTIONS -------------------------------*/

static uint64_t nuc980_clk_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980CLKState *clk = opaque;
    uint64_t        ret = 0;

    ret = clk->regs[(addr>>2)&31];
    error_report("CLK RD: 0x%08lX --> 0x%08lX", clk->iomem.addr + addr, ret);

    return ret;
}

static void nuc980_clk_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980CLKState *clk = opaque;

    clk->regs[(addr>>2)&31] = value;
    error_report("CLK WR: 0x%08lX <-- 0x%08lX", clk->iomem.addr + addr, value);
}

static void nuc980_clk_instance_init(Object *obj)
{
    NUC980CLKState *clk = NUC980_CLK(obj);
    
    static const MemoryRegionOps clk_ops = {
      .read       = nuc980_clk_read,
      .write      = nuc980_clk_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&clk->iomem, obj, &clk_ops, clk, "clk", 0x1000);

    sysbus_init_mmio(SYS_BUS_DEVICE(clk), &clk->iomem);
}

static void nuc980_clk_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 CLK Controller";
    dev_class->reset = NULL;
}

/*------------------------------ CLK TYPE -----------------------------------*/

static const TypeInfo nuc980_clk_type = {
    .name          = TYPE_NUC980_CLK,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980CLKState),
    .instance_init = nuc980_clk_instance_init,
    .class_init    = nuc980_clk_class_init,
};

/*****************************************************************************/
/*                          SDRAM INTERFACE CONTROLLER                       */
/*****************************************************************************/

/*------------------------------- SDR MACROS --------------------------------*/

#define TYPE_NUC980_SDR "nuc980-sdr"

#define REG_SDR_OPMCTL     0x000
#define REG_SDR_CMD        0x004
#define REG_SDR_REFCTL     0x008
#define REG_SDR_SIZE0      0x010
#define REG_SDR_SIZE1      0x014
#define REG_SDR_MR         0x018
#define REG_SDR_EMR        0x01C
#define REG_SDR_EMR2       0x020
#define REG_SDR_EMR3       0x024
#define REG_SDR_TIME       0x028
#define REG_SDR_DQSODS     0x030
#define REG_SDR_CKDQSDS    0x034
#define REG_SDR_DAENSEL    0x038

/*---------------------------- SDR DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980SDRState, NUC980_SDR)

struct NUC980SDRState {
    SysBusDevice  parent_obj;
    MemoryRegion  sdram;
    MemoryRegion  bootram;
    MemoryRegion  iomem;
    SSIBus       *bus;
    uint32_t      lol_reg;
};

/*----------------------------- SDR FUNCTIONS -------------------------------*/

static uint64_t nuc980_sdr_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980SDRState *sdr = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      case REG_SDR_SIZE0:
        ret = 0x0000000E; // SDRAM0: 64MB
        break;

      case REG_SDR_SIZE1:
        ret = 0x00000000; // SDRAM1: 0MB
        break;

      default:
        error_report("SDR RD: 0x%08lX --> 0x%08lX", sdr->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_sdr_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980SDRState *sdr = opaque;

    switch(addr) {
      default:
        error_report("SDR WR: 0x%08lX <-- 0x%08lX", sdr->iomem.addr + addr, value);
        break;
    }    
}

static void nuc980_sdr_instance_init(Object *obj)
{
    NUC980SDRState *sdr     = NUC980_SDR(obj);

    static const MemoryRegionOps sdr_ops = {
      .read       = nuc980_sdr_read,
      .write      = nuc980_sdr_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_ram(&sdr->sdram, NULL, "sdram", 64*1024*1024, &error_fatal);
    memory_region_init_ram(&sdr->bootram, NULL, "bootram", 0x4000, &error_fatal);
    memory_region_init_io(&sdr->iomem, obj, &sdr_ops, sdr, "sdr", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(sdr), &sdr->iomem);
}

static void nuc980_sdr_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 SDR Controller";
    dev_class->reset = NULL;
}

/*------------------------------ SDR TYPE -----------------------------------*/

static const TypeInfo nuc980_sdr_type = {
    .name          = TYPE_NUC980_SDR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980SDRState),
    .instance_init = nuc980_sdr_instance_init,
    .class_init    = nuc980_sdr_class_init,
};

/*****************************************************************************/
/*                         INTERNAL NAND FLASH DRIVER                        */
/*****************************************************************************/

/*------------------------------- FMI MACROS --------------------------------*/

#define TYPE_NUC980_FMI "nuc980-fmi"

#define FMI_DMACTL       0x400
#define FMI_DMASA        0x408
#define FMI_DMAABCNT     0x40C
#define FMI_DMAINTEN     0x410
#define FMI_DMAINTSTS    0x414
#define FMI_GCTL         0x800
#define FMI_NANDCTL      0x8A0
#define FMI_NANDTMCTL    0x8A4
#define FMI_NANDINTEN    0x8A8
#define FMI_NANDINTSTS   0x8AC
#define FMI_NANDCMD      0x8B0
#define FMI_NANDADDR     0x8B4
#define FMI_NANDDATA     0x8B8
#define FMI_NANDRACTL    0x8BC
#define FMI_NANDECTL     0x8C0
#define FMI_NANDRA0      0xA00
#define FMI_NANDRA1      0xA04
#define FMI_NANDRA2      0xA08
#define FMI_NANDRA3      0xA0C
#define FMI_NANDRA4      0xA10
#define FMI_NANDRA5      0xA14
#define FMI_NANDRA6      0xA18
#define FMI_NANDRA7      0xA1C
#define FMI_NANDRA8      0xA20
#define FMI_NANDRA9      0xA24
#define FMI_NANDRA10     0xA28
#define FMI_NANDRA11     0xA2C
#define FMI_NANDRA12     0xA30
#define FMI_NANDRA13     0xA34
#define FMI_NANDRA14     0xA38
#define FMI_NANDRA15     0xA3C

/*---------------------------- FMI DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980FMIState, NUC980_FMI)

struct NUC980FMIState {
    SysBusDevice    parent_obj;
    MemoryRegion    iomem;
    uint32_t        dma_enable;
    uint32_t        dma_addr;
    uint32_t        nand_ctrl;
    uint32_t        nand_tmctl;
    uint32_t        int_en;
    uint32_t        int_st;
    uint8_t         cmd;
    uint8_t         addr[4];
    uint8_t         addr_idx;
    uint8_t         data_idx;
    uint32_t        redu_area[16];
};

/*----------------------------- FMI FUNCTIONS -------------------------------*/

static void nuc980_fmi_dma(NUC980FMIState *fmi)
{
    uint32_t sec_size;
    uint32_t src_addr;
    uint32_t dst_addr;

    sec_size = 0x800;
    src_addr = ((fmi->addr[3]<<8) | (fmi->addr[2]<<0))*sec_size;
    dst_addr = fmi->dma_addr;

    cpu_physical_memory_write(dst_addr, nuc980_boot_flash+src_addr, sec_size);
}

static uint64_t nuc980_fmi_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980FMIState *fmi = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      case FMI_DMACTL:
        ret = fmi->dma_enable;
        break;

      case FMI_DMASA:
        ret = fmi->dma_addr;
        break;

      case FMI_DMAABCNT:
        break;

      case FMI_DMAINTEN:
        break;

      case FMI_DMAINTSTS:
        break;

      case FMI_GCTL:
        break;

      case FMI_NANDCTL:
        ret = fmi->nand_ctrl;
        break;

      case FMI_NANDTMCTL:
        ret = fmi->nand_tmctl;
        break;

      case FMI_NANDINTEN:
        ret = fmi->int_en;
        break;

      case FMI_NANDINTSTS:
        ret = fmi->int_st;
        break;

      case FMI_NANDCMD:
        break;

      case FMI_NANDADDR:
        break;

      case FMI_NANDDATA:
        /* process command here */
        if (fmi->cmd == 0x90) {
          /* READ ID (MT29F2G08AAD) */
          if (fmi->data_idx == 0) {
            ret = 0x2C;
          } else if (fmi->data_idx == 1) {
            ret = 0xDA;
          } else if (fmi->data_idx == 2) {
            ret = 0x80;
          } else if (fmi->data_idx == 3) {
            ret = 0x95;
          } else if (fmi->data_idx == 4) {
            ret = 0x50;
          } else {
            ret = 0xFF;
          }
        } else if (fmi->cmd == 0x30) {
          /* START READ */
          if (fmi->addr[0] != 0 || fmi->addr[1] != 0) {
            ret = 0xFF;
          }
        }
        fmi->data_idx++;
        break;

      case FMI_NANDRA0:
      case FMI_NANDRA1:
      case FMI_NANDRA2:
      case FMI_NANDRA3:
      case FMI_NANDRA4:
      case FMI_NANDRA5:
      case FMI_NANDRA6:
      case FMI_NANDRA7:
      case FMI_NANDRA8:
      case FMI_NANDRA9:
      case FMI_NANDRA10:
      case FMI_NANDRA11:
      case FMI_NANDRA12:
      case FMI_NANDRA13:
      case FMI_NANDRA14:
      case FMI_NANDRA15:
        ret = fmi->redu_area[(addr&0xFF)>>2];
        break;

      default:
        error_report("FMI RD: 0x%08lX --> 0x%08lX", fmi->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_fmi_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980FMIState *fmi = opaque;

    switch(addr) {
      case FMI_DMACTL:
        fmi->dma_enable = value & 1;
        break;

      case FMI_DMASA:
        fmi->dma_addr = value;
        break;

      case FMI_DMAABCNT:
        break;

      case FMI_DMAINTEN:
        break;

      case FMI_DMAINTSTS:
        break;

      case FMI_GCTL:
        break;

      case FMI_NANDCTL:
        if (value & 0) {
          /* software reset */
        }
        if (value & 2) {
          /* DMA read */
          nuc980_fmi_dma(fmi);
          fmi->int_st |= (1<<0);
        }
        if (value & 4) {
          /* DMA write */
        }
        if (value & 8) {
          /* redundant area read */
        }
        fmi->nand_ctrl = value & 0xFFFFFFF0;
        break;

      case FMI_NANDTMCTL:
        fmi->nand_tmctl = value;
        break;

      case FMI_NANDINTEN:
        fmi->int_en = value;
        break;

      case FMI_NANDINTSTS:
        if (value & (1<<0)) {
          fmi->int_st &= ~(1<<0);
        }
        if (value & (1<<2)) {
          fmi->int_st &= ~(1<<2);
        }
        if (value & (1<<10)) {
          fmi->int_st &= ~(1<<10);
        }
        break;

      case FMI_NANDCMD:
        fmi->cmd = value;
        fmi->addr_idx = 0;
        fmi->data_idx = 0;
        break;

      case FMI_NANDADDR:
        if (fmi->addr_idx < 4) {
          fmi->addr[fmi->addr_idx] = value;
        }    
        fmi->addr_idx++;
        break;

      case FMI_NANDDATA:
        fmi->data_idx++;
        break;

      case FMI_NANDRACTL:
        break;

      case FMI_NANDECTL:
        break;

      case FMI_NANDRA0:
      case FMI_NANDRA1:
      case FMI_NANDRA2:
      case FMI_NANDRA3:
      case FMI_NANDRA4:
      case FMI_NANDRA5:
      case FMI_NANDRA6:
      case FMI_NANDRA7:
      case FMI_NANDRA8:
      case FMI_NANDRA9:
      case FMI_NANDRA10:
      case FMI_NANDRA11:
      case FMI_NANDRA12:
      case FMI_NANDRA13:
      case FMI_NANDRA14:
      case FMI_NANDRA15:
        fmi->redu_area[(addr&0xFF)>>2] = value;
        break;

      default:
        error_report("FMI WR: 0x%08lX <-- 0x%08lX", fmi->iomem.addr + addr, value);
        break;
    }
}

static void nuc980_fmi_instance_init(Object *obj)
{
    NUC980FMIState *fmi = NUC980_FMI(obj);
    
    static const MemoryRegionOps fmi_ops = {
      .read       = nuc980_fmi_read,
      .write      = nuc980_fmi_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&fmi->iomem, obj, &fmi_ops, fmi, "fmi", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(fmi), &fmi->iomem);
    
    fmi->nand_ctrl = 0x02860090;
    fmi->int_st    = (1<<18);
}

static void nuc980_fmi_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 FMI Controller";
    dev_class->reset = NULL;
}

/*------------------------------ FMI TYPE -----------------------------------*/

static const TypeInfo nuc980_fmi_type = {
    .name          = TYPE_NUC980_FMI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980FMIState),
    .instance_init = nuc980_fmi_instance_init,
    .class_init    = nuc980_fmi_class_init,
};

/*****************************************************************************/
/*                              IRQ CONTROLLER                               */
/*****************************************************************************/

/*------------------------------- AIC MACROS --------------------------------*/

#define TYPE_NUC980_AIC "nuc980-aic"

#define REG_AIC_SRC00    0x000
#define REG_AIC_SRC01    0x004
#define REG_AIC_SRC02    0x008
#define REG_AIC_SRC03    0x00C
#define REG_AIC_SRC04    0x010
#define REG_AIC_SRC05    0x014
#define REG_AIC_SRC06    0x018
#define REG_AIC_SRC07    0x01C
#define REG_AIC_SRC08    0x020
#define REG_AIC_SRC09    0x024
#define REG_AIC_SRC10    0x028
#define REG_AIC_SRC11    0x02C
#define REG_AIC_SRC12    0x030
#define REG_AIC_SRC13    0x034
#define REG_AIC_SRC14    0x038
#define REG_AIC_SRC15    0x03C
#define REG_AIC_RAW0     0x100
#define REG_AIC_RAW1     0x104
#define REG_AIC_IS0      0x110
#define REG_AIC_IS1      0x114
#define REG_AIC_IRQ      0x120
#define REG_AIC_FIQ      0x124
#define REG_AIC_IE0      0x128
#define REG_AIC_IE1      0x12C
#define REG_AIC_IEN0     0x130
#define REG_AIC_IEN1     0x134
#define REG_AIC_IDIS0    0x138
#define REG_AIC_IDIS1    0x13C
#define REG_AIC_IRQRST   0x150
#define REG_AIC_FIQRST   0x154

/*---------------------------- AIC DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980AICState, NUC980_AIC)

struct NUC980AICState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    qemu_irq      irq[2];
    uint64_t      int_ien;
    uint64_t      int_sts;
    uint8_t       irq_src;
    uint8_t       fiq_src;
    uint8_t       irq_pin;
    uint8_t       fiq_pin;
};

/*----------------------------- AIC FUNCTIONS -------------------------------*/

static void nuc980_aic_update(void *opaque)
{
    NUC980AICState *aic = opaque;  
    int             i   = 0; 

    /* should there be any IRQ to CPU? */
    if (aic->int_sts) {
      for (i = 0; i < 64; i++) {
        if (aic->int_sts & (1UL<<i)) {
          aic->irq_src = i;
        }
      }
      if (aic->irq_pin == 0) {
        qemu_set_irq(aic->irq[0], 1);
        aic->irq_pin = 1;
        //printf("IRQ SOURCE: %d\n", aic->irq_src);
      }
    } else {
      aic->irq_src = 0;
      if (aic->irq_pin == 1) {
        qemu_set_irq(aic->irq[0], 0);
        aic->irq_pin = 0;
        //printf("IRQ CLEARED\n");
      }    
    }
}

static void nuc980_aic_irq(void *opaque, int irq, int level)
{
    NUC980AICState *aic = opaque;

    if (level == 0) {
      aic->int_sts &= ~(1UL<<irq);
    } else {
      aic->int_sts |= (1UL<<irq) & aic->int_ien;
    }
    //printf("IRQ: %d %d 0x%016lX 0x%016lX\n", irq, level, aic->int_sts, aic->int_ien);

    nuc980_aic_update(aic);
}

static uint64_t nuc980_aic_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980AICState *aic = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      case REG_AIC_RAW0:
        ret = (aic->int_sts>>0) & 0xFFFFFFFF;
        break;

      case REG_AIC_RAW1:
        ret = (aic->int_sts>>32) & 0xFFFFFFFF;
        break;

      case REG_AIC_IS0:
        ret = (aic->int_sts>>0) & 0xFFFFFFFF;
        break;

      case REG_AIC_IS1:
        ret = (aic->int_sts>>32) & 0xFFFFFFFF;
        break;

      case REG_AIC_IRQ:
        ret = aic->irq_src;
        break;

      case REG_AIC_FIQ:
        ret = aic->fiq_src;
        break;

      case REG_AIC_IE0:
        ret = (aic->int_ien>>0) & 0xFFFFFFFF;
        break;

      case REG_AIC_IE1:
        ret = (aic->int_ien>>32) & 0xFFFFFFFF;
        break;

      default:
        error_report("AIC RD: 0x%08lX --> 0x%08lX", aic->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_aic_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980AICState *aic = opaque;

    switch(addr) {
      case REG_AIC_IE0:
        aic->int_ien = (aic->int_ien & 0xFFFFFFFF00000000ULL) | (value<< 0);
        break;

      case REG_AIC_IE1:
        aic->int_ien = (aic->int_ien & 0x00000000FFFFFFFFULL) | (value<<32);
        break;

      case REG_AIC_IEN0:
        aic->int_ien |= (value<< 0);
        break;

      case REG_AIC_IEN1:
        aic->int_ien |= (value<<32);
        break;

      case REG_AIC_IDIS0:
        aic->int_ien &= ~(value<< 0);
        break;

      case REG_AIC_IDIS1:
        aic->int_ien &= ~(value<<32);
        break;

      case REG_AIC_IRQRST:  
        nuc980_aic_update(aic);
        break;

      case REG_AIC_FIQRST:
        break;

      default:
        error_report("AIC WR: 0x%08lX <-- 0x%08lX", aic->iomem.addr + addr, value);
        break;
    }
}

static void nuc980_aic_instance_init(Object *obj)
{
    NUC980AICState *aic = NUC980_AIC(obj);
    
    static const MemoryRegionOps aic_ops = {
      .read       = nuc980_aic_read,
      .write      = nuc980_aic_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&aic->iomem, obj, &aic_ops, aic, "aic", 0x1000);

    sysbus_init_mmio(SYS_BUS_DEVICE(aic), &aic->iomem);

    qdev_init_gpio_in (DEVICE(aic), nuc980_aic_irq, 64);
    qdev_init_gpio_out(DEVICE(aic), aic->irq,       2);
}

static void nuc980_aic_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 AIC Controller";
    dev_class->reset = NULL;
}

/*------------------------------ AIC TYPE -----------------------------------*/

static const TypeInfo nuc980_aic_type = {
    .name          = TYPE_NUC980_AIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980AICState),
    .instance_init = nuc980_aic_instance_init,
    .class_init    = nuc980_aic_class_init,
};

/*****************************************************************************/
/*                              RTC CONTROLLER                               */
/*****************************************************************************/

/*------------------------------- RTC MACROS --------------------------------*/

#define TYPE_NUC980_RTC "nuc980-rtc"

#define REG_RTC_INIT       0x000
#define REG_RTC_RWEN       0x004
#define REG_RTC_FREQADJ    0x008
#define REG_RTC_TIME       0x00C
#define REG_RTC_CAL        0x010
#define REG_RTC_TIMEFMT    0x014
#define REG_RTC_WEEKDAY    0x018
#define REG_RTC_TALM       0x01C
#define REG_RTC_CALM       0x020
#define REG_RTC_LEAPYEAR   0x024
#define REG_RTC_INTEN      0x028
#define REG_RTC_INTSTS     0x02C
#define REG_RTC_TICK       0x030
#define REG_RTC_PWRCTL     0x034
#define REG_RTC_PWRCNT     0x038
#define REG_RTC_CLKCTL     0x03C
#define REG_RTC_SPR0       0x040 
#define REG_RTC_SPR1       0x044
#define REG_RTC_SPR2       0x048
#define REG_RTC_SPR3       0x04C
#define REG_RTC_SPR4       0x050
#define REG_RTC_SPR5       0x054
#define REG_RTC_SPR6       0x058
#define REG_RTC_SPR7       0x05C
#define REG_RTC_SPR8       0x060 
#define REG_RTC_SPR9       0x064
#define REG_RTC_SPR10      0x068
#define REG_RTC_SPR11      0x06C
#define REG_RTC_SPR12      0x070
#define REG_RTC_SPR13      0x074
#define REG_RTC_SPR14      0x078
#define REG_RTC_SPR15      0x07C

/*---------------------------- RTC DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980RTCState, NUC980_RTC)

struct NUC980RTCState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    QEMUTimer    *ts;
    qemu_irq      irq;
};

/*----------------------------- RTC FUNCTIONS -------------------------------*/

static void nuc980_rtc_cb(void *opaque)
{
    NUC980RTCState *rtc = opaque;

    /* RTC interrupt */
    //qemu_set_irq(rtc->irq, 1);

    timer_mod_ns(rtc->ts, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)+10000);
}


static uint64_t nuc980_rtc_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980RTCState *rtc = opaque;
    uint64_t        ret = 0;

    switch (addr) {
      default:
        error_report("RTC RD: 0x%08lX --> 0x%08lX", rtc->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_rtc_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980RTCState *rtc = opaque;

    switch (addr) {
      default:
        error_report("RTC WR: 0x%08lX <-- 0x%08lX", rtc->iomem.addr + addr, value);
        break;
    }
}

static void nuc980_rtc_instance_init(Object *obj)
{
    NUC980RTCState *rtc = NUC980_RTC(obj);
    
    static const MemoryRegionOps rtc_ops = {
      .read       = nuc980_rtc_read,
      .write      = nuc980_rtc_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&rtc->iomem, obj, &rtc_ops, rtc, "rtc", 0x1000);

    sysbus_init_mmio(SYS_BUS_DEVICE(rtc), &rtc->iomem);

    rtc->ts = timer_new_ns(QEMU_CLOCK_VIRTUAL, nuc980_rtc_cb, rtc);
    timer_mod_anticipate_ns(rtc->ts, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

static void nuc980_rtc_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 RTC Controller";
    dev_class->reset = NULL;
}

/*------------------------------ RTC TYPE -----------------------------------*/

static const TypeInfo nuc980_rtc_type = {
    .name          = TYPE_NUC980_RTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980RTCState),
    .instance_init = nuc980_rtc_instance_init,
    .class_init    = nuc980_rtc_class_init,
};

/*****************************************************************************/
/*                               TIMER DRIVER                                */
/*****************************************************************************/

/*------------------------------- TMR MACROS --------------------------------*/

#define TYPE_NUC980_TMR "nuc980-tmr"

#define REG_TMR_CTL     0x000
#define REG_TMR_PRECNT  0x004
#define REG_TMR_CMP     0x008
#define REG_TMR_INTEN   0x00C
#define REG_TMR_INTSTS  0x010
#define REG_TMR_CNT     0x014
#define REG_TMR_CAP     0x018
#define REG_TMR_ECTL    0x020

/*---------------------------- TMR DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980TMRState, NUC980_TMR)

struct NUC980TMRState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    qemu_irq      irq;
    QEMUTimer    *ts;
    uint32_t      ctl;
    uint32_t      sts;
    uint64_t      ctr;
    uint64_t      cmp;
    uint32_t      pre;
    uint32_t      ien;
};

/*----------------------------- TMR FUNCTIONS -------------------------------*/

static void nuc980_tmr_cb(void *opaque)
{
    NUC980TMRState *tmr = opaque;

    /* RESET */
    if (tmr->ctl & 2) {
      tmr->ctr = 0;
      tmr->ctl &= ~2;
      tmr->ctl |= 1;
    }

    /* COUNTING */
    if (tmr->ctl & 1) {
      tmr->ctr+=1;
      if (tmr->ctr >= tmr->cmp) {
        tmr->ctr = tmr->cmp;
      }
      //if (tmr->cmp < 0xFFFFF)
      //  printf("counter: 0x%016lX 0x%016lX 0x%02X 0x%02X\n", tmr->ctr, tmr->cmp, tmr->ctl, tmr->ien);
      if (tmr->ctr == tmr->cmp) {
        tmr->ctr = 0;
        if (((tmr->ctl>>4)&3) == 0) {
          tmr->ctl &= ~3;
        }
        tmr->sts |= 1;
        if (tmr->ien & 1) {
          qemu_set_irq(tmr->irq, 1);
        }
      }
    }

    timer_mod(tmr->ts, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 10000);
}

static uint64_t nuc980_tmr_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980TMRState *tmr = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      case REG_TMR_CTL:
        ret = tmr->ctl;
        break;

      case REG_TMR_CMP:
        ret = tmr->cmp;
        break;

      case REG_TMR_INTSTS:
        ret = tmr->sts;
        break;

      case REG_TMR_CNT:
        ret = tmr->ctr;
        break;

      default:
        error_report("TMR RD: 0x%08lX --> 0x%08lX", tmr->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_tmr_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980TMRState *tmr = opaque;

    switch(addr) {
      case REG_TMR_CTL:
        tmr->ctl = value;
        break;

      case REG_TMR_PRECNT:
        tmr->pre = value;
        break;

      case REG_TMR_INTEN:
        tmr->ien = value;
        break;

      case REG_TMR_CMP:
        //printf("COUNT FOR: %ld\n", value);
        tmr->ctl &= ~3;
        tmr->cmp = value & 0xFFFFFF;
        tmr->ctl |= 2;
        break;

      case REG_TMR_INTSTS:
        if ((value & 1) && (tmr->sts & 1)) {
          tmr->sts &= ~(1);
          qemu_set_irq(tmr->irq, 0);
        }
        break;

      default:
        error_report("TMR WR: 0x%08lX <-- 0x%08lX", tmr->iomem.addr + addr, value);
        break;
    }
}

static void nuc980_tmr_instance_init(Object *obj)
{
    NUC980TMRState *tmr = NUC980_TMR(obj);
    
    static const MemoryRegionOps tmr_ops = {
      .read       = nuc980_tmr_read,
      .write      = nuc980_tmr_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&tmr->iomem, obj, &tmr_ops, tmr, "tmr", 0x100);

    sysbus_init_mmio(SYS_BUS_DEVICE(tmr), &tmr->iomem);
    
    tmr->ts = timer_new_ns(QEMU_CLOCK_REALTIME, nuc980_tmr_cb, tmr);
    timer_mod_anticipate_ns(tmr->ts, qemu_clock_get_ns(QEMU_CLOCK_REALTIME));
}

static void nuc980_tmr_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 TMR Controller";
    dev_class->reset = NULL;
}

/*------------------------------ TMR TYPE -----------------------------------*/

static const TypeInfo nuc980_tmr_type = {
    .name          = TYPE_NUC980_TMR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980TMRState),
    .instance_init = nuc980_tmr_instance_init,
    .class_init    = nuc980_tmr_class_init,
};

/*****************************************************************************/
/*                                 SPI DRIVER                                */
/*****************************************************************************/

/*------------------------------- SPI MACROS --------------------------------*/

#define TYPE_NUC980_SPI "nuc980-spi"

/*---------------------------- SPI DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980SPIState, NUC980_SPI)

struct NUC980SPIState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    SSIBus       *bus;
    uint32_t      lol_reg;
};

/*----------------------------- SPI FUNCTIONS -------------------------------*/

static uint64_t nuc980_spi_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980SPIState *spi = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      default:
        error_report("SPI RD: 0x%08lX --> 0x%08lX", spi->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_spi_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980SPIState *spi = opaque;

    switch(addr) {
      default:
        error_report("SPI WR: 0x%08lX <-- 0x%08lX", spi->iomem.addr + addr, value);
        break;
    }
}

static void nuc980_spi_instance_init(Object *obj)
{
    NUC980SPIState *spi = NUC980_SPI(obj);
    
    static const MemoryRegionOps spi_ops = {
      .read       = nuc980_spi_read,
      .write      = nuc980_spi_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&spi->iomem, obj, &spi_ops, spi, "spi", 0x1000);

    sysbus_init_mmio(SYS_BUS_DEVICE(spi), &spi->iomem);

    spi->bus = ssi_create_bus(DEVICE(spi), "ssi");
}

static void nuc980_spi_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 SPI Controller";
    dev_class->reset = NULL;
}

/*------------------------------ SPI TYPE -----------------------------------*/

static const TypeInfo nuc980_spi_type = {
    .name          = TYPE_NUC980_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980SPIState),
    .instance_init = nuc980_spi_instance_init,
    .class_init    = nuc980_spi_class_init,
};

/*****************************************************************************/
/*                               UART DRIVER                                 */
/*****************************************************************************/

/*------------------------------- UIC MACROS --------------------------------*/

#define TYPE_NUC980_UIC "nuc980-uic"

#define REG_UIC_DAT        0x00
#define REG_UIC_INTEN      0x04
#define REG_UIC_FIFO       0x08
#define REG_UIC_LINE       0x0C
#define REG_UIC_MODEM      0x10
#define REG_UIC_MODEMSTS   0x14
#define REG_UIC_FIFOSTS    0x18
#define REG_UIC_INTSTS     0x1C
#define REG_UIC_TOUT       0x20
#define REG_UIC_BAUD       0x24
#define REG_UIC_IRDA       0x28
#define REG_UIC_ALTCTL     0x2C
#define REG_UIC_FUNCSEL    0x30
#define REG_UIC_LINCTL     0x34
#define REG_UIC_LINSTS     0x38
#define REG_UIC_BRCOMP     0x3C
#define REG_UIC_WKCTL      0x40
#define REG_UIC_WKSTS      0x44
#define REG_UIC_DWKCOMP    0x48

/*---------------------------- UIC DATATYPES --------------------------------*/

OBJECT_DECLARE_SIMPLE_TYPE(NUC980UICState, NUC980_UIC)

struct NUC980UICState {
    SysBusDevice  parent_obj;
    MemoryRegion  iomem;
    CharBackend   be;
    bool          rx_avail;
    uint8_t       rx_data;
    qemu_irq      irq;
    uint32_t      ien;
    uint32_t      fifo_ctl;
    uint32_t      line_ctl;
    uint32_t      modm_ctl;
    uint32_t      fifo_sts;
    uint32_t      modm_sts;
    uint32_t      intr_sts;
    uint32_t      baud;
};

/*----------------------------- UIC FUNCTIONS -------------------------------*/

static int nuc980_uic_chardev_canrd(void *opaque)
{
    NUC980UICState *uic = opaque;

    return (uic->rx_avail == false);
}

static void nuc980_uic_chardev_read(void *opaque, const uint8_t *buf, int size)
{
    NUC980UICState *uic = opaque;

    uic->rx_avail = true;
    uic->rx_data  = buf[0];

    //qemu_set_irq(uic->irq, 1);
}

static void nuc980_uic_chardev_write(void *opaque, const uint8_t *buf, int size)
{
    NUC980UICState *uic = opaque;

    qemu_chr_fe_write(&uic->be, buf, size);
}

static void nuc980_uic_chardev_event(void *opaque, QEMUChrEvent event)
{
}

static int nuc980_uic_chardev_canchg(void *opaque)
{
    return 1;
}

static void nuc980_uic_chardev_attach(NUC980UICState *uic, Chardev *s) {
    qemu_chr_fe_init(&uic->be, s, &error_abort);
    qemu_chr_fe_set_handlers(&uic->be,
                             nuc980_uic_chardev_canrd,
                             nuc980_uic_chardev_read,
                             nuc980_uic_chardev_event,
                             nuc980_uic_chardev_canchg,
                             uic,
                             NULL,
                             false);
}

static uint64_t nuc980_uic_read(void *opaque, hwaddr addr, unsigned size)
{
    NUC980UICState *uic = opaque;
    uint64_t        ret = 0;

    switch(addr) {
      case REG_UIC_DAT:
        ret = uic->rx_data;
        uic->rx_avail = false;
        break;
        
      case REG_UIC_INTEN:
        ret = uic->ien;
        break;
        
      case REG_UIC_FIFO:
        ret = uic->fifo_ctl;
        break;
        
      case REG_UIC_LINE:
        ret = uic->line_ctl;
        break;
        
      case REG_UIC_MODEM:
        ret = uic->modm_ctl;
        break;
        
      case REG_UIC_MODEMSTS:
        ret = uic->modm_sts;
        break;

      case REG_UIC_FIFOSTS:
        ret |= (1<<28); // TXEMPTF
        ret |= (1<<22); // TXEMPTY
        if (!uic->rx_avail) {
          ret |= (1<<14); // RXEMPTY
        }
        break;

      case REG_UIC_INTSTS:
        ret = uic->intr_sts;
        break;

      case REG_UIC_BAUD:
        ret = uic->baud;
        break;

      default:
        error_report("UIC RD: 0x%08lX --> 0x%08lX", uic->iomem.addr + addr, ret);
        break;
    }

    return ret;
}

static void nuc980_uic_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    NUC980UICState *uic = opaque;

    switch(addr) {
      case REG_UIC_DAT:
        nuc980_uic_chardev_write(uic, (const uint8_t *) &value, 1);
        break;
        
      case REG_UIC_INTEN:
        uic->ien = value;
        break;
        
      case REG_UIC_FIFO:
        uic->fifo_ctl = value;
        break;
        
      case REG_UIC_LINE:
        uic->line_ctl = value;
        break;
        
      case REG_UIC_MODEM:
        uic->modm_ctl = value;
        break;
        
      case REG_UIC_MODEMSTS:
        //uic->modm_sts = value;
        break;
        
      case REG_UIC_FIFOSTS:
        //uic->fifo_sts = value;
        break;
        
      case REG_UIC_INTSTS:
        //uic->intr_sts = value;
        qemu_set_irq(uic->irq, 0);
        break;

      case REG_UIC_BAUD:
        uic->baud = value;
        break;

      default:
        error_report("UIC WR: 0x%08lX <-- 0x%08lX", uic->iomem.addr + addr, value);
        break;
    }
}

static void nuc980_uic_instance_init(Object *obj)
{
    NUC980UICState *uic = NUC980_UIC(obj);
    
    static const MemoryRegionOps uic_ops = {
      .read       = nuc980_uic_read,
      .write      = nuc980_uic_write,
      .endianness = DEVICE_NATIVE_ENDIAN,
    };

    memory_region_init_io(&uic->iomem, obj, &uic_ops, uic, "uic", 0x1000);

    sysbus_init_mmio(SYS_BUS_DEVICE(uic), &uic->iomem);
}

static void nuc980_uic_class_init(ObjectClass *obj_class, void *data)
{
    DeviceClass *dev_class = DEVICE_CLASS(obj_class);

    dev_class->desc  = "NUC980 UIC Controller";
    dev_class->reset = NULL;
}

/*------------------------------ UIC TYPE -----------------------------------*/

static const TypeInfo nuc980_uic_type = {
    .name          = TYPE_NUC980_UIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980UICState),
    .instance_init = nuc980_uic_instance_init,
    .class_init    = nuc980_uic_class_init,
};

/*****************************************************************************/
/*                                BOARD DRIVER                               */
/*****************************************************************************/

/*------------------------------- SOC MACROS --------------------------------*/

#define TYPE_NUC980_SOC      MACHINE_TYPE_NAME("nuc980-soc")

/*------------------------------ SOC FUNCTIONS ------------------------------*/

static void nuc980_soc_ldflash(MachineState *machine)
{
    FILE *file_desc = NULL;
    char *file_name = NULL;
    char  read_cnt  = 0;

    if (!machine->firmware) {
      error_report("No ROM boot image provided.");
      exit(1);
    }
    
    file_name = qemu_find_file(QEMU_FILE_TYPE_BIOS, machine->firmware);
    if (!file_name) {
      error_report("Could not find ROM image '%s'", machine->firmware);
      exit(1);
    }
    
    file_desc = fopen(file_name, "r");

    read_cnt = fread(nuc980_boot_flash, sizeof(nuc980_boot_flash), 1, file_desc);
    if (read_cnt != 1) {
      error_report("Failed to read from '%s'", machine->firmware);
      exit(1);
    }

    fclose(file_desc);
}

static void nuc980_soc_reset(MachineState *machine)
{
    /* on real board there is a 16.5KB internal boot ROM (IBR)
     * that initializes the 64MB SDRAM, maps it to address 0x00000 
     * and loads early-stage SPL loader from flash into RAM.
     */
    int spl_addr   = 0x00200;
    int spl_size   = 0x10000;

    cpu_physical_memory_write(spl_addr, nuc980_boot_flash, spl_size);

    /* set CPU to jump to SPL code */
    //cpu_set_pc(CPU(cpu), 0x200);
}

static void nuc980_soc_instance_init(MachineState *machine)
{
    Object         *cpu       = NULL;
    MemoryRegion   *mem       = get_system_memory();
    qemu_irq        pic[64]   = {0};
    qemu_irq        irq       = {0};
    qemu_irq        fiq       = {0};
    NUC980SYSState *sys       = NULL;
    NUC980CLKState *clk       = NULL;
    NUC980SDRState *sdr       = NULL;
    NUC980FMIState *fmi       = NULL;
    NUC980AICState *aic       = NULL;
    NUC980RTCState *rtc       = NULL;
    NUC980TMRState *tmr[6]    = {0};
    NUC980SPIState *spi[3]    = {0};
    NUC980UICState *uic[10]   = {0};

    /* allocate new CPU */
    cpu = object_new("arm926-arm-cpu");
    qdev_realize(DEVICE(cpu), NULL, &error_fatal);
    irq = qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ);
    fiq = qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ);

    /* SYS Controller */
    sys = NUC980_SYS(object_new(TYPE_NUC980_SYS));
    memory_region_add_subregion(mem, 0xB0000000, &sys->iomem);

    /* CLK Controller */
    clk = NUC980_CLK(object_new(TYPE_NUC980_CLK));
    memory_region_add_subregion(mem, 0xB0000200, &clk->iomem);

    /* SDR Controller */
    sdr = NUC980_SDR(object_new(TYPE_NUC980_SDR));
    memory_region_add_subregion(mem, 0x00000000, &sdr->sdram);
    memory_region_add_subregion(mem, 0xBC000000, &sdr->bootram);
    memory_region_add_subregion(mem, 0xB0002000, &sdr->iomem);

    /* FMI */
    fmi = NUC980_FMI(object_new(TYPE_NUC980_FMI));
    memory_region_add_subregion(mem, 0xB0019000, &fmi->iomem);

    /* AIC */
    aic = NUC980_AIC(object_new(TYPE_NUC980_AIC));
    memory_region_add_subregion(mem, 0xB0042000, &aic->iomem);
    pic[ 0] = qdev_get_gpio_in(DEVICE(aic),  0);
    pic[ 1] = qdev_get_gpio_in(DEVICE(aic),  1);
    pic[ 2] = qdev_get_gpio_in(DEVICE(aic),  2);
    pic[ 3] = qdev_get_gpio_in(DEVICE(aic),  3);
    pic[ 4] = qdev_get_gpio_in(DEVICE(aic),  4);
    pic[ 5] = qdev_get_gpio_in(DEVICE(aic),  5);
    pic[ 6] = qdev_get_gpio_in(DEVICE(aic),  6);
    pic[ 7] = qdev_get_gpio_in(DEVICE(aic),  7);
    pic[ 8] = qdev_get_gpio_in(DEVICE(aic),  8);
    pic[ 9] = qdev_get_gpio_in(DEVICE(aic),  9);
    pic[10] = qdev_get_gpio_in(DEVICE(aic), 10);
    pic[11] = qdev_get_gpio_in(DEVICE(aic), 11);
    pic[12] = qdev_get_gpio_in(DEVICE(aic), 12);
    pic[13] = qdev_get_gpio_in(DEVICE(aic), 13);
    pic[14] = qdev_get_gpio_in(DEVICE(aic), 14);
    pic[15] = qdev_get_gpio_in(DEVICE(aic), 15);
    pic[16] = qdev_get_gpio_in(DEVICE(aic), 16);
    pic[17] = qdev_get_gpio_in(DEVICE(aic), 17);
    pic[18] = qdev_get_gpio_in(DEVICE(aic), 18);
    pic[19] = qdev_get_gpio_in(DEVICE(aic), 19);
    pic[20] = qdev_get_gpio_in(DEVICE(aic), 20);
    pic[21] = qdev_get_gpio_in(DEVICE(aic), 21);
    pic[22] = qdev_get_gpio_in(DEVICE(aic), 22);
    pic[23] = qdev_get_gpio_in(DEVICE(aic), 23);
    pic[24] = qdev_get_gpio_in(DEVICE(aic), 24);
    pic[25] = qdev_get_gpio_in(DEVICE(aic), 25);
    pic[26] = qdev_get_gpio_in(DEVICE(aic), 26);
    pic[27] = qdev_get_gpio_in(DEVICE(aic), 27);
    pic[28] = qdev_get_gpio_in(DEVICE(aic), 28);
    pic[29] = qdev_get_gpio_in(DEVICE(aic), 29);
    pic[30] = qdev_get_gpio_in(DEVICE(aic), 30);
    pic[31] = qdev_get_gpio_in(DEVICE(aic), 31);
    pic[32] = qdev_get_gpio_in(DEVICE(aic), 32);
    pic[33] = qdev_get_gpio_in(DEVICE(aic), 33);
    pic[34] = qdev_get_gpio_in(DEVICE(aic), 34);
    pic[35] = qdev_get_gpio_in(DEVICE(aic), 35);
    pic[36] = qdev_get_gpio_in(DEVICE(aic), 36);
    pic[37] = qdev_get_gpio_in(DEVICE(aic), 37);
    pic[38] = qdev_get_gpio_in(DEVICE(aic), 38);
    pic[39] = qdev_get_gpio_in(DEVICE(aic), 39);
    pic[40] = qdev_get_gpio_in(DEVICE(aic), 40);
    pic[41] = qdev_get_gpio_in(DEVICE(aic), 41);
    pic[42] = qdev_get_gpio_in(DEVICE(aic), 42);
    pic[43] = qdev_get_gpio_in(DEVICE(aic), 43);
    pic[44] = qdev_get_gpio_in(DEVICE(aic), 44);
    pic[45] = qdev_get_gpio_in(DEVICE(aic), 45);
    pic[46] = qdev_get_gpio_in(DEVICE(aic), 46);
    pic[47] = qdev_get_gpio_in(DEVICE(aic), 47);
    pic[48] = qdev_get_gpio_in(DEVICE(aic), 48);
    pic[49] = qdev_get_gpio_in(DEVICE(aic), 49);
    pic[50] = qdev_get_gpio_in(DEVICE(aic), 50);
    pic[51] = qdev_get_gpio_in(DEVICE(aic), 51);
    pic[52] = qdev_get_gpio_in(DEVICE(aic), 52);
    pic[53] = qdev_get_gpio_in(DEVICE(aic), 53);
    pic[54] = qdev_get_gpio_in(DEVICE(aic), 54);
    pic[55] = qdev_get_gpio_in(DEVICE(aic), 55);
    pic[56] = qdev_get_gpio_in(DEVICE(aic), 56);
    pic[57] = qdev_get_gpio_in(DEVICE(aic), 57);
    pic[58] = qdev_get_gpio_in(DEVICE(aic), 58);
    pic[59] = qdev_get_gpio_in(DEVICE(aic), 59);
    pic[60] = qdev_get_gpio_in(DEVICE(aic), 60);
    pic[61] = qdev_get_gpio_in(DEVICE(aic), 61);
    pic[62] = qdev_get_gpio_in(DEVICE(aic), 62);
    pic[63] = qdev_get_gpio_in(DEVICE(aic), 63);
    qdev_connect_gpio_out(DEVICE(aic), 0, irq);
    qdev_connect_gpio_out(DEVICE(aic), 1, fiq);

    /* RTC Controller */
    rtc = NUC980_RTC(object_new(TYPE_NUC980_RTC));
    memory_region_add_subregion(mem, 0xB0041000, &rtc->iomem);
    rtc->irq = pic[15];

    /* Timers */
    tmr[0] = NUC980_TMR(object_new(TYPE_NUC980_TMR));
    tmr[1] = NUC980_TMR(object_new(TYPE_NUC980_TMR));
    tmr[2] = NUC980_TMR(object_new(TYPE_NUC980_TMR));
    tmr[3] = NUC980_TMR(object_new(TYPE_NUC980_TMR));
    tmr[4] = NUC980_TMR(object_new(TYPE_NUC980_TMR));
    tmr[5] = NUC980_TMR(object_new(TYPE_NUC980_TMR));
    memory_region_add_subregion(mem, 0xB0050000, &tmr[0]->iomem);
    memory_region_add_subregion(mem, 0xB0050100, &tmr[1]->iomem);
    memory_region_add_subregion(mem, 0xB0051000, &tmr[2]->iomem);
    memory_region_add_subregion(mem, 0xB0051100, &tmr[3]->iomem);
    memory_region_add_subregion(mem, 0xB0052000, &tmr[4]->iomem);
    memory_region_add_subregion(mem, 0xB0052100, &tmr[5]->iomem);
    tmr[0]->irq = pic[16];
    tmr[1]->irq = pic[17];
    tmr[2]->irq = pic[30];
    tmr[3]->irq = pic[31];
    tmr[4]->irq = pic[32];
    tmr[5]->irq = pic[34];

    /* SPIs */
    spi[0] = NUC980_SPI(object_new(TYPE_NUC980_SPI));
    spi[1] = NUC980_SPI(object_new(TYPE_NUC980_SPI));
    spi[2] = NUC980_SPI(object_new(TYPE_NUC980_SPI));
    memory_region_add_subregion(mem, 0xB0060000, &spi[0]->iomem);
    memory_region_add_subregion(mem, 0xB0061000, &spi[1]->iomem);
    memory_region_add_subregion(mem, 0xB0062000, &spi[2]->iomem);

    /* UARTs */
    uic[0] = NUC980_UIC(object_new(TYPE_NUC980_UIC));
    uic[1] = NUC980_UIC(object_new(TYPE_NUC980_UIC));
    uic[2] = NUC980_UIC(object_new(TYPE_NUC980_UIC));
    uic[3] = NUC980_UIC(object_new(TYPE_NUC980_UIC));
    uic[4] = NUC980_UIC(object_new(TYPE_NUC980_UIC));
    uic[5] = NUC980_UIC(object_new(TYPE_NUC980_UIC));
    uic[6] = NUC980_UIC(object_new(TYPE_NUC980_UIC));
    uic[7] = NUC980_UIC(object_new(TYPE_NUC980_UIC));
    uic[8] = NUC980_UIC(object_new(TYPE_NUC980_UIC));
    uic[9] = NUC980_UIC(object_new(TYPE_NUC980_UIC));
    nuc980_uic_chardev_attach(uic[0], serial_hd(0));
    nuc980_uic_chardev_attach(uic[1], serial_hd(1));
    nuc980_uic_chardev_attach(uic[2], serial_hd(2));
    nuc980_uic_chardev_attach(uic[3], serial_hd(3));
    nuc980_uic_chardev_attach(uic[4], serial_hd(4));
    nuc980_uic_chardev_attach(uic[5], serial_hd(5));
    nuc980_uic_chardev_attach(uic[6], serial_hd(6));
    nuc980_uic_chardev_attach(uic[7], serial_hd(7));
    nuc980_uic_chardev_attach(uic[8], serial_hd(8));
    nuc980_uic_chardev_attach(uic[9], serial_hd(9));
    memory_region_add_subregion(mem, 0xB0070000, &uic[0]->iomem);
    memory_region_add_subregion(mem, 0xB0071000, &uic[1]->iomem);
    memory_region_add_subregion(mem, 0xB0072000, &uic[2]->iomem);
    memory_region_add_subregion(mem, 0xB0073000, &uic[3]->iomem);
    memory_region_add_subregion(mem, 0xB0074000, &uic[4]->iomem);
    memory_region_add_subregion(mem, 0xB0075000, &uic[5]->iomem);
    memory_region_add_subregion(mem, 0xB0076000, &uic[6]->iomem);
    memory_region_add_subregion(mem, 0xB0077000, &uic[7]->iomem);
    memory_region_add_subregion(mem, 0xB0078000, &uic[8]->iomem);
    memory_region_add_subregion(mem, 0xB0079000, &uic[9]->iomem);
    uic[0]->irq = pic[36];

    /* load boot flash from file */
    nuc980_soc_ldflash(machine);
}

static void nuc980_soc_class_init(ObjectClass *obj_class, void *data)
{
    MachineClass *machine_class = MACHINE_CLASS(obj_class);

    machine_class->desc     = "Nuvoton NUC980 SoC (ARM926EJ-S)";
    machine_class->reset    = nuc980_soc_reset;
    machine_class->init     = nuc980_soc_instance_init;
    
    /* FIXME */
    machine_class->ignore_memory_transaction_failures = true;
}

/*------------------------------ SOC TYPE -----------------------------------*/

static const TypeInfo nuc980_soc_type = {
    .name          = TYPE_NUC980_SOC,
    .parent        = TYPE_MACHINE,
    .class_init    = nuc980_soc_class_init,
};

/*****************************************************************************/
/*                                REGISTERATION                              */
/*****************************************************************************/

static void nuc980_register_types(void)
{
    type_register_static(&nuc980_sys_type);
    type_register_static(&nuc980_clk_type);
    type_register_static(&nuc980_sdr_type);
    type_register_static(&nuc980_fmi_type);
    type_register_static(&nuc980_aic_type);
    type_register_static(&nuc980_rtc_type);
    type_register_static(&nuc980_tmr_type);
    type_register_static(&nuc980_spi_type);
    type_register_static(&nuc980_uic_type);
    type_register_static(&nuc980_soc_type);
}

type_init(nuc980_register_types)

