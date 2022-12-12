/*******************************************************************/
/*                                                                 */
/* This is the C++ source code of the flx-dma-test application     */
/*                                                                 */
/* Author: Markus Joos, CERN                                       */
/*                                                                 */
/**C 2015 Ecosoft - Made from at least 80% recycled source code*****/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>

#include "cmem_rcc/cmem_rcc.h"
#include "flxcard/FlxCard.h"
#include "flxcard/FlxException.h"


#define APPLICATION_NAME    "flx-dma-test"
#define BUFSIZE (1024)
#define DMA_ID (0)

#ifndef BF_GBT_EMU_ENA_TOHOST
#define BF_GBT_EMU_ENA_TOHOST BF_FE_EMU_ENA_EMU_TOHOST
#endif
#ifndef BF_GBT_EMU_ENA_TOFRONTEND
#define BF_GBT_EMU_ENA_TOFRONTEND BF_FE_EMU_ENA_EMU_TOFRONTEND
#endif


//Globals
FlxCard flxCard;


/*****************/
void display_help()
/*****************/
{
  printf("Usage: %s [OPTIONS]\n", APPLICATION_NAME);
  printf("Initializes DMA transfers and the DMA memory on the screen in 1s intervals.\n\n");
  printf("Options:\n");
  printf("  -d NUMBER      Use card indicated by NUMBER. Default: 0.\n");
  printf("  -h             Display help.\n");
}


/********************************/
void dump_buffer(u_long virt_addr)
/********************************/
{
  u_char *buf = (u_char *)virt_addr;
  int i;

  for(i = 0; i < BUFSIZE; i++)
  {
    if(i % 32 == 0)
      printf("\n0x  ");
    printf("%02x ", *buf++);
  }
  printf("\n");
}


/*****************************/
int main(int argc, char **argv)
/*****************************/
{
  int i, loop, ret, device_number = 0, opt, handle;
  u_long baraddr0, vaddr, paddr, board_id, bsize, opt_emu_ena_to_host, opt_emu_ena_to_frontend;
  flxcard_bar0_regs_t *bar0;

  while((opt = getopt(argc, argv, "hd:D:V")) != -1)
  {
    switch (opt)
    {
      case 'd':
        device_number = atoi(optarg);
        break;

      case 'h':
        display_help();
        exit(0);
        break;

      default:
        fprintf(stderr, "Usage: %s COMMAND [OPTIONS]\nTry %s -h for more information.\n", APPLICATION_NAME, APPLICATION_NAME);
        exit(-1);
    }
  }

  try
  {
    flxCard.card_open(device_number, 0);

    // save current state
    opt_emu_ena_to_host     = flxCard.cfg_get_option(BF_GBT_EMU_ENA_TOHOST);
    opt_emu_ena_to_frontend = flxCard.cfg_get_option(BF_GBT_EMU_ENA_TOFRONTEND);

    for(loop = 0; loop < 8; loop++)
      flxCard.dma_stop(loop);

    flxCard.dma_reset();
    flxCard.soft_reset();
    //flxCard.dma_fifo_flush(); MJ: Method disabled (requsted by Frans)
    flxCard.cfg_set_option(BF_GBT_EMU_ENA_TOFRONTEND, 0);
    flxCard.cfg_set_option(BF_GBT_EMU_ENA_TOHOST, 1);

    board_id = flxCard.cfg_get_option(REG_GIT_TAG); 

    u_long loop;
    char git_tag[8];
    for(loop = 0; loop < 8; loop++)
      git_tag[loop] = (board_id >> (8 * loop)) & 0xff;      
    printf("Board ID (GIT): %s\n", git_tag);   

    ret = CMEM_Open();
    bsize = BUFSIZE;
    if (!ret)
      ret = CMEM_SegmentAllocate(bsize, (char *)"FlxThroughput", &handle);

    if (!ret)
      ret = CMEM_SegmentPhysicalAddress(handle, &paddr);

    if (!ret)
      ret = CMEM_SegmentVirtualAddress(handle, &vaddr);

    if (ret)
    {
      rcc_error_print(stdout, ret);
      exit(-1);
    }

    printf("Allocated Memory Segment\n  Phys. Addr: 0x%016lx\n  Virt. Addr: 0x%016lx\n", paddr, vaddr);

    printf("\nBuffer before DMA write:\n");
    dump_buffer(vaddr);

    flxCard.dma_to_host(DMA_ID, paddr, BUFSIZE, FLX_DMA_WRAPAROUND);
    //flxCard.dma_wait(DMA_ID);

    baraddr0 = flxCard.openBackDoor(0);
    bar0 = (flxcard_bar0_regs_t *)baraddr0;

    printf("Start Ptr:   0x%016lx\n", bar0->DMA_DESC[0].start_address);
    printf("End Ptr:     0x%016lx\n", bar0->DMA_DESC[0].end_address);
    printf("Enable:      0x%0x\n", bar0->DMA_DESC_ENABLE);
    printf("Read Ptr:    0x%016lx\n", bar0->DMA_DESC[0].read_ptr);
    printf("Write Ptr:   0x%016lx\n", bar0->DMA_DESC_STATUS[0].current_address);
    printf("Descriptor done DMA0: 0x%lx\n", bar0->DMA_DESC_STATUS[0].descriptor_done);
    printf("Even Addr. DMA  DMA0: 0x%lx\n", bar0->DMA_DESC_STATUS[0].even_addr_dma);
    printf("Even Addr. PC   DMA0: 0x%lx\n", bar0->DMA_DESC_STATUS[0].even_addr_pc);
    printf("Descriptor done DMA1: 0x%lx\n", bar0->DMA_DESC_STATUS[1].descriptor_done);
    printf("Even Addr. DMA  DMA1: 0x%lx\n", bar0->DMA_DESC_STATUS[1].even_addr_dma);
    printf("Even Addr. PC   DMA1: 0x%lx\n", bar0->DMA_DESC_STATUS[1].even_addr_pc);

    printf("Start Addr: %016lx\nEnd Addr:  %016lx\nRead Ptr: %016lx\n", bar0->DMA_DESC[0].start_address, bar0->DMA_DESC[0].end_address, bar0->DMA_DESC[0].read_ptr);

    printf("\nBuffer after DMA write:\n");
    dump_buffer(vaddr);

    for(i = 0; ; i++)
    {
      printf("\n--------------------\n  %d:\n", i);
      flxCard.dma_advance_ptr(DMA_ID, paddr, BUFSIZE, 512);
      //flxCard.dma_wait(DMA_ID);

      printf("Read Ptr:    0x%016lx\n", bar0->DMA_DESC[0].read_ptr);
      printf("Write Ptr:   0x%016lx\n", bar0->DMA_DESC_STATUS[0].current_address);
      printf("Descriptor done DMA0: 0x%lx\n", bar0->DMA_DESC_STATUS[0].descriptor_done);
      printf("Even Addr. DMA  DMA0: 0x%lx\n", bar0->DMA_DESC_STATUS[0].even_addr_dma);
      printf("Even Addr. PC   DMA0: 0x%lx\n", bar0->DMA_DESC_STATUS[0].even_addr_pc);

      dump_buffer(vaddr);
      sleep(1);
    }

    // reset to initial state
    flxCard.cfg_set_option(BF_GBT_EMU_ENA_TOHOST, opt_emu_ena_to_host);
    flxCard.cfg_set_option(BF_GBT_EMU_ENA_TOFRONTEND, opt_emu_ena_to_frontend);

    ret = CMEM_SegmentFree(handle);
    if (!ret)
      ret = CMEM_Close();
    if (ret)
      rcc_error_print(stdout, ret);

    flxCard.card_close();
  }
  catch(FlxException &ex)
  {
    std::cout << "ERROR. Exception thrown: " << ex.what() << std:: endl;
    exit(-1);
  }

  exit(0);
}
