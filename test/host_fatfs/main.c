#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ff.h"
#include "diskio.h"

static FILE *image;
static unsigned multi_writes;
PARTITION VolToPart[FF_VOLUMES] = {{0, 0}};

DSTATUS disk_initialize(BYTE drive) { return drive ? STA_NOINIT : 0; }
DSTATUS disk_status(BYTE drive) { return disk_initialize(drive); }

DRESULT disk_read(BYTE drive, BYTE *buffer, LBA_t sector, UINT count)
{
    if (drive || fseeko(image, (off_t)sector * 512, SEEK_SET)) return RES_ERROR;
    return fread(buffer, 512, count, image) == count ? RES_OK : RES_ERROR;
}

DRESULT disk_write(BYTE drive, const BYTE *buffer, LBA_t sector, UINT count)
{
    if (drive || fseeko(image, (off_t)sector * 512, SEEK_SET)) return RES_ERROR;
    if (count > 1) ++multi_writes;
    return fwrite(buffer, 512, count, image) == count ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(BYTE drive, BYTE command, void *buffer)
{
    if (drive) return RES_PARERR;
    switch (command) {
    case CTRL_SYNC: return fflush(image) ? RES_ERROR : RES_OK;
    case GET_SECTOR_SIZE: *(WORD *)buffer = 512; return RES_OK;
    case GET_BLOCK_SIZE: *(DWORD *)buffer = 1; return RES_OK;
    case GET_SECTOR_COUNT:
        if (fseeko(image, 0, SEEK_END)) return RES_ERROR;
        *(LBA_t *)buffer = (LBA_t)(ftello(image) / 512);
        return RES_OK;
    case CTRL_TRIM: return RES_OK;
    default: return RES_PARERR;
    }
}

void *ff_memalloc(unsigned size) { return malloc(size); }
void ff_memfree(void *buffer) { free(buffer); }
static DWORD clock_time = (DWORD)(2026 - 1980) << 25 | 10U << 21 | 20U << 16;
DWORD get_fattime(void) { return clock_time; }
int ff_mutex_create(int volume) { (void)volume; return 1; }
void ff_mutex_delete(int volume) { (void)volume; }
int ff_mutex_take(int volume) { (void)volume; return 1; }
void ff_mutex_give(int volume) { (void)volume; }

int main(int argc, char **argv)
{
    assert(argc == 3);
    assert(FF_FS_EXFAT && sizeof(FSIZE_t) == 8);
    image = fopen(argv[1], "r+b");
    assert(image);
    FATFS fs;
    FIL file;
    FILINFO info;
    UINT count;
    static BYTE expected[65536], actual[65536];
    for (size_t i = 0; i < sizeof(expected); ++i) {
        expected[i] = (BYTE)((i * 37U) ^ (i >> 8U));
    }

    assert(f_mount(&fs, "0:", 1) == FR_OK);
    assert(fs.fs_type == atoi(argv[2]));
    assert(f_mkdir("0:/Compatibility test") == FR_OK);
    assert(f_open(&file, "0:/Compatibility test/Long filename.bin", FA_CREATE_NEW | FA_WRITE) == FR_OK);
    assert(f_write(&file, expected, sizeof(expected), &count) == FR_OK);
    assert(count == sizeof(expected));
    assert(f_sync(&file) == FR_OK);
    assert(f_close(&file) == FR_OK);
    assert(f_rename("0:/Compatibility test/Long filename.bin", "0:/Compatibility test/Renamed file.bin") == FR_OK);
    assert(f_mount(NULL, "0:", 0) == FR_OK);

    assert(f_mount(&fs, "0:", 1) == FR_OK);
    assert(f_stat("0:/Compatibility test/Renamed file.bin", &info) == FR_OK);
    assert(info.fsize == sizeof(expected));
    // The stored date survives flush/remount on every supported filesystem.
    assert(info.fdate == ((2026U - 1980U) << 9 | 10U << 5 | 20U));
    assert(info.ftime == 0);
    // A later write/flush records the current RTC time rather than creation time.
    clock_time |= 20U << 11 | 9U << 5 | 29U; /* 20:09:58, two-second FAT resolution */
    assert(f_open(&file, "0:/Compatibility test/Renamed file.bin", FA_WRITE) == FR_OK);
    assert(f_write(&file, expected, 1, &count) == FR_OK && count == 1);
    assert(f_sync(&file) == FR_OK);
    assert(f_close(&file) == FR_OK);
    assert(f_stat("0:/Compatibility test/Renamed file.bin", &info) == FR_OK);
    assert(info.ftime == (20U << 11 | 9U << 5 | 29U));
    assert(f_open(&file, "0:/Compatibility test/Renamed file.bin", FA_READ) == FR_OK);
    assert(f_read(&file, actual, sizeof(actual), &count) == FR_OK);
    assert(count == sizeof(actual) && memcmp(expected, actual, sizeof(actual)) == 0);
    assert(f_lseek(&file, 513) == FR_OK);
    assert(f_read(&file, actual, 777, &count) == FR_OK);
    assert(count == 777 && memcmp(expected + 513, actual, 777) == 0);
    assert(f_close(&file) == FR_OK);
    assert(f_unlink("0:/Compatibility test/Renamed file.bin") == FR_OK);
    assert(f_unlink("0:/Compatibility test") == FR_OK);
    DWORD free_clusters;
    FATFS *mounted;
    assert(f_getfree("0:", &free_clusters, &mounted) == FR_OK && free_clusters > 0);
    assert(f_mount(NULL, "0:", 0) == FR_OK);
    assert(multi_writes > 0);
    assert(fclose(image) == 0);
    printf("PASS: filesystem type %s, read/write/remount/rename/delete, %u multi-block writes\n",
           argv[2], multi_writes);
    return 0;
}
