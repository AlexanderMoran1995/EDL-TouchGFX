/*
 * user_diskio_spi.h
 *
 *  Created on: Sep 28, 2022
 *      Author: moran
 */

#ifndef INC_USER_DISKIO_SPI_H_
#define INC_USER_DISKIO_SPI_H_


#include "integer.h" //from FatFs middleware library
#include "diskio.h" //from FatFs middleware library
#include "ff_gen_drv.h" //from FatFs middleware library

//we define these as inline because we don't want them to be actual function calls (they get "called" from the cubemx autogenerated user_diskio file)
//we define them as extern because they are defined in a separate .c file to user_diskio.c (which #includes this .h file)

extern DSTATUS USER_SPI_initialize (BYTE pdrv);
extern DSTATUS USER_SPI_status (BYTE pdrv);
extern DRESULT USER_SPI_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  extern DRESULT USER_SPI_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  extern DRESULT USER_SPI_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */


#endif /* INC_USER_DISKIO_SPI_H_ */
