#include <stdint.h>
#include <functional>
#include <string.h>

#include "chip.h"
#include "adc_11xx.h"
#include "gpio_11xx_1.h"
#include "timer_11xx.h"
#include "gpiogroup_11xx.h"
#include "ssp_11xx.h"
#include "printf.h"
#include "usbd_rom_api.h"

#include "duck_font.h"

//#define ENABLE_USB_MSC

#ifdef ENABLE_USB_MSC

typedef void (*emfat_readcb_t)(uint8_t *dest, int size, uint32_t offset, size_t userdata);
typedef void (*emfat_writecb_t)(const uint8_t *data, int size, uint32_t offset, size_t userdata);

typedef struct emfat_entry emfat_entry_t;

struct emfat_entry
{
	const char     *name;
	bool            dir;
	int             level;
	uint32_t        offset;
	uint32_t        curr_size;
	uint32_t        max_size;
	size_t          user_data;
	uint32_t        cma_time[3]; /**< create/mod/access time in unix format */
	emfat_readcb_t  readcb;
	emfat_writecb_t writecb;
	struct
	{
		uint32_t       first_clust;
		uint32_t       last_clust;
		uint32_t       last_reserved;
		uint32_t       num_subentry;
		emfat_entry_t *top;
		emfat_entry_t *sub;
		emfat_entry_t *next;
	} priv;
};

typedef struct
{
	uint64_t    vol_size;
	uint32_t    disk_sectors;
	const char *vol_label;
	struct
	{
		uint32_t       boot_lba;
		uint32_t       fsinfo_lba;
		uint32_t       fat1_lba;
		uint32_t       fat2_lba;
		uint32_t       root_lba;
		uint32_t       num_clust;
		emfat_entry_t *entries;
		emfat_entry_t *last_entry;
		int            num_entries;
	} priv;
} emfat_t;

static emfat_t Emfat;

#define EMFAT_ENCODE_CMA_TIME(D,M,Y,h,m,s) \
    ((((((Y)-1980) << 9) | ((M) << 5) | (D)) << 16) | \
    (((h) << 11) | ((m) << 5) | (s >> 1)))

static inline uint32_t emfat_encode_cma_time(int D, int M, int Y, int h, int m, int s)
{
    return EMFAT_ENCODE_CMA_TIME(D,M,Y,h,m,s);
}

#define SECT              512
#define CLUST             4096
#define SECT_PER_CLUST    (CLUST / SECT)
#define SIZE_TO_NSECT(s)  ((s) == 0 ? 1 : ((s) + SECT - 1) / SECT)
#define SIZE_TO_NCLUST(s) ((s) == 0 ? 1 : ((s) + CLUST - 1) / CLUST)

#define CLUST_FREE     0x00000000
#define CLUST_RESERVED 0x00000001
#define CLUST_BAD      0x0FFFFFF7
#define CLUST_ROOT_END 0X0FFFFFF8
#define CLUST_EOF      0x0FFFFFFF

#define MAX_DIR_ENTRY_CNT 16
#define FILE_SYS_TYPE_OFF 82
#define BYTES_PER_SEC_OFF 11
#define SEC_PER_CLUS_OFF 13
#define RES_SEC_CNT_OFF 14
#define FAT_CNT_OFF 16
#define TOT_SEC_CNT_OFF 32
#define SEC_PER_FAT 36
#define ROOT_DIR_STRT_CLUS_OFF 44
#define FS_INFOSECTOR_OFF 48
#define BACKUP_BOOT_SEC_OFF 50
#define NXT_FREE_CLUS_OFF 492
#define FILE_SYS_TYPE_LENGTH 8
#define SHRT_FILE_NAME_LEN 11
#define STRT_CLUS_LOW_OFF 26
#define STRT_CLUS_HIGH_OFF 20
#define FILE_SIZE_OFF 28
#define ATTR_OFF 11
#define FILE_STAT_LEN 21
#define CHECK_SUM_OFF 13
#define FILE_NAME_SHRT_LEN 8
#define FILE_NAME_EXTN_LEN 3
#define LONG_FILE_NAME_LEN 255
#define LOW_CLUSWORD_MASK 0x0000FFFF
#define HIGH_CLUSWORD_MASK 0xFFFF0000
#define LONG_FNAME_MASK 0x0F
#define LAST_ORD_FIELD_SEQ 0x40
#define LFN_END_MARK 0xFFFF
#define LFN_TERM_MARK 0x0000
#define LFN_FIRST_OFF 0x01
#define LFN_SIXTH_OFF 0x0E
#define LFN_TWELVETH_OFF 0x1C
#define LFN_FIRST_SET_CNT 5
#define LFN_SEC_SET_CNT 6
#define LFN_THIRD_SET_CNT 2
#define LFN_FIRST_SET_LEN 10
#define LFN_SEC_SET_LEN 12
#define LFN_THIRD_SET_LEN 4
#define LFN_EMPTY_LEN 2
#define LFN_LEN_PER_ENTRY 13
#define FNAME_EXTN_SEP_OFF 6
#define FNAME_SEQ_NUM_OFF 7
#define BYTES_PER_CLUSTER_ENTRY 4
#define DIR_ENTRY_LEN 32
#define VOL_ID_LEN 4
#define VOL_LABEL_LEN 11
#define RESERV_LEN 12
#define FS_VER_LEN 2
#define OEM_NAME_LEN 8
#define JUMP_INS_LEN 3
#define MAX_FAT_CNT 2
#define SPACE_VAL 32
#define FILE_READ 0x01
#define FILE_WRITE 0X02
#define FILE_CREATE_NEW 0x04
#define FILE_CREATE_ALWAYS 0x08
#define FILE_APPEND 0x10
#define ATTR_READ 0x01
#define ATTR_HIDDEN 0x02
#define ATTR_SYSTEM 0x04
#define ATTR_VOL_LABEL 0x08
#define ATTR_DIR 0x10
#define ATTR_ARCHIVE 0x20
#define ATTR_LONG_FNAME 0x0F
#define FREE_DIR_ENTRY 0x00
#define DEL_DIR_ENTRY 0xE5
#define DOT_DIR_ENTRY 0x2E
#define ASCII_DIFF 32
#define FILE_SEEK_SET 0
#define FILE_SEEK_CUR 1
#define FILE_SEEK_END 2
#define DELIMITER '/'
#define EXTN_DELIMITER '.'
#define TILDE '~'
#define FULL_SHRT_NAME_LEN 13

#pragma pack(push, 1)

typedef struct
{
	uint8_t  status;          // 0x80 for bootable, 0x00 for not bootable, anything else for invalid
	uint8_t  start_head;      // The head of the start
	uint8_t  start_sector;    // (S | ((C >> 2) & 0xC0)) where S is the sector of the start and C is the cylinder of the start. Note that S is counted from one.
	uint8_t  start_cylinder;  // (C & 0xFF) where C is the cylinder of the start
	uint8_t  PartType;
	uint8_t  end_head;
	uint8_t  end_sector;
	uint8_t  end_cylinder;
	uint32_t StartLBA;        // linear address of first sector in partition. Multiply by sector size (usually 512) for real offset
	uint32_t EndLBA;          // linear address of last sector in partition. Multiply by sector size (usually 512) for real offset
} mbr_part_t;

typedef struct
{
	uint8_t    Code[440];
	uint32_t   DiskSig;  //This is optional
	uint16_t   Reserved; //Usually 0x0000
	mbr_part_t PartTable[4];
	uint8_t    BootSignature[2]; //0x55 0xAA for bootable
} mbr_t;

typedef struct
{
	uint8_t jump[JUMP_INS_LEN];
	uint8_t OEM_name[OEM_NAME_LEN];
	uint16_t bytes_per_sec;
	uint8_t sec_per_clus;
	uint16_t reserved_sec_cnt;
	uint8_t fat_cnt;
	uint16_t root_dir_max_cnt;
	uint16_t tot_sectors;
	uint8_t media_desc;
	uint16_t sec_per_fat_fat16;
	uint16_t sec_per_track;
	uint16_t number_of_heads;
	uint32_t hidden_sec_cnt;
	uint32_t tol_sector_cnt;
	uint32_t sectors_per_fat;
	uint16_t ext_flags;
	uint8_t fs_version[FS_VER_LEN];
	uint32_t root_dir_strt_cluster;
	uint16_t fs_info_sector;
	uint16_t backup_boot_sector;
	uint8_t reserved[RESERV_LEN];
	uint8_t drive_number;
	uint8_t reserved1;
	uint8_t boot_sig;
	uint8_t volume_id[VOL_ID_LEN];
	uint8_t volume_label[VOL_LABEL_LEN];
	uint8_t file_system_type[FILE_SYS_TYPE_LENGTH];
} boot_sector;

typedef struct
{
	uint32_t signature1;     /* 0x41615252L */
	uint32_t reserved1[120]; /* Nothing as far as I can tell */
	uint32_t signature2;     /* 0x61417272L */
	uint32_t free_clusters;  /* Free cluster count.  -1 if unknown */
	uint32_t next_cluster;   /* Most recently allocated cluster */
	uint32_t reserved2[3];
	uint32_t signature3;
} fsinfo_t;

typedef struct
{
	uint8_t name[FILE_NAME_SHRT_LEN];
	uint8_t extn[FILE_NAME_EXTN_LEN];
	uint8_t attr;
	uint8_t reserved;
	uint8_t crt_time_tenth;
	uint16_t crt_time;
	uint16_t crt_date;
	uint16_t lst_access_date;
	uint16_t strt_clus_hword;
	uint16_t lst_mod_time;
	uint16_t lst_mod_date;
	uint16_t strt_clus_lword;
	uint32_t size;
} dir_entry;

typedef struct
{
	uint8_t ord_field;
	uint8_t fname0_4[LFN_FIRST_SET_LEN];
	uint8_t flag;
	uint8_t reserved;
	uint8_t chksum;
	uint8_t fname6_11[LFN_SEC_SET_LEN];
	uint8_t empty[LFN_EMPTY_LEN];
	uint8_t fname12_13[LFN_THIRD_SET_LEN];
} lfn_entry;

#pragma pack(pop)

static bool emfat_init_entries(emfat_entry_t *entries)
{
	emfat_entry_t *e;
	int i, n;

	e = &entries[0];
	if (e->level != 0 || !e->dir || e->name == NULL) return false;

	e->priv.top = NULL;
	e->priv.next = NULL;
	e->priv.sub = NULL;
	e->priv.num_subentry = 0;

	n = 0;
	for (i = 1; entries[i].name != NULL; i++)
	{
		entries[i].priv.top = NULL;
		entries[i].priv.next = NULL;
		entries[i].priv.sub = NULL;
		entries[i].priv.num_subentry = 0;
		if (entries[i].level == n - 1)
		{
			if (n == 0) return false;
			e = e->priv.top;
			n--;
		}
		if (entries[i].level == n + 1)
		{
			if (!e->dir) return false;
			e->priv.sub = &entries[i];
			entries[i].priv.top = e;
			e = &entries[i];
			n++;
			continue;
		}
		if (entries[i].level == n)
		{
			if (n == 0) return false;
			e->priv.top->priv.num_subentry++;
			entries[i].priv.top = e->priv.top;
			e->priv.next = &entries[i];
			e = &entries[i];
			continue;
		}
		return false;
	}
	return true;
}

static void lba_to_chs(int lba, uint8_t *cl, uint8_t *ch, uint8_t *dh)
{
  int cylinder, head, sector;
  int sectors = 63;
	int heads = 255;
	int cylinders = 1024;
  sector = lba % sectors + 1;
  head = (lba / sectors) % heads;
  cylinder = lba / (sectors * heads);
  if (cylinder >= cylinders)
    {
      *cl = *ch = *dh = 0xff;
      return;
    }
  *cl = sector | ((cylinder & 0x300) >> 2);
  *ch = cylinder & 0xFF;
  *dh = head;
}

static bool emfat_init(emfat_t *emfat, const char *label, emfat_entry_t *entries)
{
	uint32_t sect_per_fat;
	uint32_t clust;
	emfat_entry_t *e;
	int i;

	if (emfat == NULL || label == NULL || entries == NULL)
		return false;

	if (!emfat_init_entries(entries))
		return false;

	clust = 2;
	for (i = 0; entries[i].name != NULL; i++)
	{
		e = &entries[i];
		if (e->dir)
		{
			e->curr_size = 0;
			e->max_size = 0;
			e->priv.first_clust = clust;
			e->priv.last_clust = clust + SIZE_TO_NCLUST(e->priv.num_subentry * sizeof(dir_entry)) - 1;
			e->priv.last_reserved = e->priv.last_clust;
		}
		else
		{
			e->priv.first_clust = clust;
			e->priv.last_clust = e->priv.first_clust + SIZE_TO_NCLUST(entries[i].curr_size) - 1;
			e->priv.last_reserved = e->priv.first_clust + SIZE_TO_NCLUST(entries[i].max_size) - 1;
		}
		clust = e->priv.last_reserved + 1;
	}
	clust -= 2;

	emfat->vol_label = label;
	emfat->priv.num_entries = i;
	emfat->priv.boot_lba = 62;
	emfat->priv.fsinfo_lba = emfat->priv.boot_lba + 1;
	emfat->priv.fat1_lba = emfat->priv.fsinfo_lba + 1;
	emfat->priv.num_clust = clust;
	sect_per_fat = SIZE_TO_NSECT((uint64_t)emfat->priv.num_clust * 4);
	emfat->priv.fat2_lba = emfat->priv.fat1_lba + sect_per_fat;
	emfat->priv.root_lba = emfat->priv.fat2_lba + sect_per_fat;
	emfat->priv.entries = entries;
	emfat->priv.last_entry = entries;
	emfat->disk_sectors = clust * SECT_PER_CLUST + emfat->priv.root_lba;
	emfat->vol_size = (uint64_t)emfat->disk_sectors * SECT;
	return true;
}

static void read_mbr_sector(const emfat_t *emfat, uint8_t *sect)
{
	mbr_t *mbr;
	memset(sect, 0, SECT);
	mbr = (mbr_t *)sect;
	mbr->DiskSig = 0;
	mbr->Reserved = 0;
	mbr->PartTable[0].status = 0x80;
	mbr->PartTable[0].PartType = 0x0C;
	mbr->PartTable[0].StartLBA = emfat->priv.boot_lba;
	mbr->PartTable[0].EndLBA = emfat->disk_sectors;
	lba_to_chs(mbr->PartTable[0].StartLBA, &mbr->PartTable[0].start_sector, &mbr->PartTable[0].start_cylinder, &mbr->PartTable[0].start_head);
	lba_to_chs(emfat->disk_sectors - 1, &mbr->PartTable[0].end_sector, &mbr->PartTable[0].end_cylinder, &mbr->PartTable[0].end_head);
	mbr->BootSignature[0] = 0x55;
	mbr->BootSignature[1] = 0xAA;
}

static void read_boot_sector(const emfat_t *emfat, uint8_t *sect)
{
	boot_sector *bs;
	memset(sect, 0, SECT);
	bs = (boot_sector *)sect;
	bs->jump[0] = 0xEB;
	bs->jump[1] = 0x58;
	bs->jump[2] = 0x90;
	memcpy(bs->OEM_name, "MSDOS5.0", 8);
	bs->bytes_per_sec = SECT;
	bs->sec_per_clus = 8;     /* 4 kb per cluster */
	bs->reserved_sec_cnt = 2; /* boot sector & fsinfo sector */
	bs->fat_cnt = 2;          /* two tables */
	bs->root_dir_max_cnt = 0;
	bs->tot_sectors = 0;
	bs->media_desc = 0xF8;
	bs->sec_per_fat_fat16 = 0;
	bs->sec_per_track = 63;
	bs->number_of_heads = 0xFF;
	bs->hidden_sec_cnt = 62;
	bs->tol_sector_cnt = emfat->disk_sectors - emfat->priv.boot_lba;
	bs->sectors_per_fat = emfat->priv.fat2_lba - emfat->priv.fat1_lba;
	bs->ext_flags = 0;
	bs->fs_version[0] = 0;
	bs->fs_version[1] = 0;
	bs->root_dir_strt_cluster = 2;
	bs->fs_info_sector = 1;
	bs->backup_boot_sector = 0; /* not used */
	bs->drive_number = 128;
	bs->boot_sig = 0x29;
	bs->volume_id[0] = 148;
	bs->volume_id[1] = 14;
	bs->volume_id[2] = 13;
	bs->volume_id[3] = 8;
	memcpy(bs->volume_label, "NO NAME     ", 12);
	memcpy(bs->file_system_type, "FAT32   ", 8);
	sect[SECT - 2] = 0x55;
	sect[SECT - 1] = 0xAA;
}

#define IS_CLUST_OF(clust, entry) ((clust) >= (entry)->priv.first_clust && (clust) <= (entry)->priv.last_reserved)

static emfat_entry_t *find_entry(const emfat_t *emfat, uint32_t clust, emfat_entry_t *nearest)
{
	if (nearest == NULL)
		nearest = emfat->priv.entries;

	if (nearest->priv.first_clust > clust)
		while (nearest >= emfat->priv.entries) // backward finding
		{
			if (IS_CLUST_OF(clust, nearest))
				return nearest;
			nearest--;
		}
	else
		while (nearest->name != NULL) // forward finding
		{
			if (IS_CLUST_OF(clust, nearest))
				return nearest;
			nearest++;
		}
	return NULL;
}

static void read_fsinfo_sector(const emfat_t *emfat, uint8_t *sect)
{
	fsinfo_t *info = (fsinfo_t *)sect;
	info->signature1 = 0x41615252L;
	info->signature2 = 0x61417272L;
	info->free_clusters = 0;
	info->next_cluster = emfat->priv.num_clust + 2;
	memset(info->reserved1, 0, sizeof(info->reserved1));
	memset(info->reserved2, 0, sizeof(info->reserved2));
	info->signature3 = 0xAA550000;
}

static void read_fat_sector(emfat_t *emfat, uint8_t *sect, uint32_t index)
{
	emfat_entry_t *le;
	uint32_t *values;
	uint32_t count;
	uint32_t curr;

	values = (uint32_t *)sect;
	curr = index * 128;
	count = 128;

	if (curr == 0)
	{
		*values++ = CLUST_ROOT_END;
		*values++ = 0xFFFFFFFF;
		count -= 2;
		curr += 2;
	}

	le = emfat->priv.last_entry;
	while (count != 0)
	{
		if (!IS_CLUST_OF(curr, le))
		{
			le = find_entry(emfat, curr, le);
			if (le == NULL)
			{
				le = emfat->priv.last_entry;
				*values = CLUST_RESERVED;
				values++;
				count--;
				curr++;
				continue;
			}
		}
		if (le->dir)
		{
			if (curr == le->priv.last_clust)
				*values = CLUST_EOF; else
				*values = curr + 1;
		}
		else
		{
			if (curr == le->priv.last_clust)
				*values = CLUST_EOF; else
			if (curr > le->priv.last_clust)
				*values = CLUST_FREE; else
				*values = curr + 1;
		}
		values++;
		count--;
		curr++;
	}
	emfat->priv.last_entry = le;
}

static void fill_entry(dir_entry *entry, const char *name, uint8_t attr, uint32_t clust, const uint32_t cma[3], uint32_t size)
{
	int i, l, l1, l2;
	int dot_pos;

	memset(entry, 0, sizeof(dir_entry));

	if (cma)
	{
		entry->crt_date = cma[0] >> 16;
		entry->crt_time = cma[0] & 0xFFFF;
		entry->lst_mod_date = cma[1] >> 16;
		entry->lst_mod_time = cma[1] & 0xFFFF;
		entry->lst_access_date = cma[2] >> 16;
	}

	l = strlen(name);
	dot_pos = -1;
	if ((attr & ATTR_DIR) == 0)
		for (i = l - 1; i >= 0; i--)
			if (name[i] == '.')
			{
				dot_pos = i;
				break;
			}
	if (dot_pos == -1)
	{
		l1 = l > FILE_NAME_SHRT_LEN ? FILE_NAME_SHRT_LEN : l;
		l2 = 0;
	}
	else
	{
		l1 = dot_pos;
		l1 = l1 > FILE_NAME_SHRT_LEN ? FILE_NAME_SHRT_LEN : l1;
		l2 = l - dot_pos - 1;
		l2 = l2 > FILE_NAME_EXTN_LEN ? FILE_NAME_EXTN_LEN : l2;
	}
	memset(entry->name, ' ', FILE_NAME_SHRT_LEN + FILE_NAME_EXTN_LEN);
	memcpy(entry->name, name, l1);
	memcpy(entry->extn, name + dot_pos + 1, l2);
	
	for (i = 0; i < FILE_NAME_SHRT_LEN; i++) {
		if (entry->name[i] >= 'a' && entry->name[i] <= 'z') {
			entry->name[i] -= 0x20;
		}
	}
	for (i = 0; i < FILE_NAME_EXTN_LEN; i++) {
		if (entry->extn[i] >= 'a' && entry->extn[i] <= 'z') {
			entry->extn[i] -= 0x20;
		}
	}

	entry->attr = attr;
	entry->reserved = 24;
	entry->strt_clus_hword = clust >> 16;
	entry->strt_clus_lword = clust;
	entry->size = size;
	return;
}

static void fill_dir_sector(emfat_t *emfat, uint8_t *data, emfat_entry_t *entry, uint32_t rel_sect)
{
	dir_entry *de;
	uint32_t avail;

	memset(data, 0, SECT);
	de = (dir_entry *)data;
	avail = SECT;

	if (rel_sect == 0)
	// 1. first sector of directory
	{
		if (entry->priv.top == NULL)
		{
			fill_entry(de++, emfat->vol_label, ATTR_VOL_LABEL, 0, 0, 0);
			avail -= sizeof(dir_entry);
		}
		else
		{
			fill_entry(de++, ".", ATTR_DIR | ATTR_READ, entry->priv.first_clust, 0, 0);
			fill_entry(de++, "..", ATTR_DIR | ATTR_READ, entry->priv.top->priv.first_clust, 0, 0);
			avail -= sizeof(dir_entry) * 2;
		}
		entry = entry->priv.sub;
	}
	else
	// 2. not a first sector
	{
		int n;
		n = rel_sect * (SECT / sizeof(dir_entry));
		n -= entry->priv.top == NULL ? 1 : 2;
		entry = entry->priv.sub;
		while (n > 0 && entry != NULL)
		{
			entry = entry->priv.next;
			n--;
		}
	}
	while (entry != NULL && avail >= sizeof(dir_entry))
	{
		if (entry->dir)
			fill_entry(de++, entry->name, ATTR_DIR | ATTR_READ, entry->priv.first_clust, entry->cma_time, 0); else
			fill_entry(de++, entry->name, ATTR_ARCHIVE | ATTR_READ, entry->priv.first_clust, entry->cma_time, entry->curr_size);
		entry = entry->priv.next;
		avail -= sizeof(dir_entry);
	}
}

static void read_data_sector(emfat_t *emfat, uint8_t *data, uint32_t rel_sect)
{
	emfat_entry_t *le;
	uint32_t cluster;
	cluster = rel_sect / 8 + 2;
	rel_sect = rel_sect % 8;

	le = emfat->priv.last_entry;
	if (!IS_CLUST_OF(cluster, le))
	{
		le = find_entry(emfat, cluster, le);
		if (le == NULL)
		{
			int i;
			for (i = 0; i < SECT / 4; i++)
				((uint32_t *)data)[i] = 0xEFBEADDE;
			return;
		}
		emfat->priv.last_entry = le;
	}
	if (le->dir)
	{
		fill_dir_sector(emfat, data, le, rel_sect);
		return;
	}
	if (le->readcb == NULL)
		memset(data, 0, SECT);
	else
	{
		uint32_t offset = cluster - le->priv.first_clust;
		offset = offset * CLUST + rel_sect * SECT;
		le->readcb(data, SECT, offset + le->offset, le->user_data);
	}
	return;
}

static void emfat_read(emfat_t *emfat, uint8_t *data, uint32_t sector, int num_sectors)
{
	while (num_sectors > 0)
	{
		if (sector >= emfat->priv.root_lba)
			read_data_sector(emfat, data, sector - emfat->priv.root_lba);
		else
		if (sector == 0)
			read_mbr_sector(emfat, data);
		else
		if (sector == emfat->priv.fsinfo_lba)
			read_fsinfo_sector(emfat, data);
		else
		if (sector == emfat->priv.boot_lba)
			read_boot_sector(emfat, data);
		else
		if (sector >= emfat->priv.fat1_lba && sector < emfat->priv.fat2_lba)
			read_fat_sector(emfat, data, sector - emfat->priv.fat1_lba);
		else
		if (sector >= emfat->priv.fat2_lba && sector < emfat->priv.root_lba)
			read_fat_sector(emfat, data, sector - emfat->priv.fat2_lba);
		else
			memset(data, 0, SECT);
		data += SECT;
		num_sectors--;
		sector++;
	}
}

static void write_data_sector(emfat_t *emfat, const uint8_t *data, uint32_t rel_sect)
{
	emfat_entry_t *le;
	uint32_t cluster;
	cluster = rel_sect / 8 + 2;
	rel_sect = rel_sect % 8;

	le = emfat->priv.last_entry;
	if (!IS_CLUST_OF(cluster, le))
	{
		le = find_entry(emfat, cluster, le);
		if (le == NULL) return;
		emfat->priv.last_entry = le;
	}
	if (le->dir)
	{
		// TODO: handle changing a filesize
		return;
	}
	if (le->writecb != NULL)
		le->writecb(data, SECT, rel_sect * SECT + le->offset, le->user_data);
}

static void write_fat_sector(emfat_t *emfat, const uint8_t *data, uint32_t rel_sect)
{
}

static void emfat_write(emfat_t *emfat, const uint8_t *data, uint32_t sector, int num_sectors)
{
	while (num_sectors > 0)
	{
		if (sector >= emfat->priv.root_lba)
			write_data_sector(emfat, data, sector - emfat->priv.root_lba);
		else
		if (sector >= emfat->priv.fat1_lba && sector < emfat->priv.fat2_lba)
			write_fat_sector(emfat, data, sector - emfat->priv.fat1_lba);
		else
		if (sector >= emfat->priv.fat2_lba && sector < emfat->priv.root_lba)
			write_fat_sector(emfat, data, sector - emfat->priv.fat2_lba);
		data += SECT;
		num_sectors--;
		sector++;
	}
}

static void translate_rd( uint32_t offset, uint8_t** buff_adr, uint32_t length, uint32_t high_offset)
{
	emfat_read(&Emfat, (*buff_adr), offset / SECT, length / SECT);
}

static void translate_wr( uint32_t offset, uint8_t** buff_adr, uint32_t length, uint32_t high_offset)
{
	emfat_write(&Emfat, (*buff_adr), offset / SECT, length / SECT);
}

static ErrorCode_t translate_verify( uint32_t offset, uint8_t* src, uint32_t length, uint32_t high_offset)
{
	uint8_t sector[SECT];
	for (uint32_t c = 0; c < (length/SECT); c++) {
		emfat_read(&Emfat, sector, c + (offset/SECT), SECT);
		if (memcmp(sector, src + (c * SECT), SECT)) {
			return ERR_FAILED;
		}
	}
	return LPC_OK;
}

#endif  // #ifdef ENABLE_USB_MSC

namespace std {
    void __throw_bad_function_call() { for(;;) {} }
}

namespace {
	

template<class T> const T& max(const T &a, const T &b) { return (a>b) ? a : b; }
template<class T> const T& min(const T &a, const T &b) { return (a<b) ? a : b; }
template<class T> const T& abs(const T &a) { return (a<0) ? -a : a; }
template<class T> const T& constrain(const T& x, const T &a, const T &b) { return (x>b)?b:((x<a)?a:x); }

static volatile uint32_t system_clock_ms = 0;

#include "duck_font.h"

struct rgba {
	
	rgba() { rgbp = 0; }

	rgba(const rgba &in) { rgbp = in.rgbp; }
	
	rgba(uint32_t _rgbp) { rgbp = _rgbp; }
	
	rgba(uint8_t _r, 
		 uint8_t _g, 
		 uint8_t _b, 
		 uint8_t _a = 0) { 
		 rgbp = (uint32_t(_r)<<16)|
		  	    (uint32_t(_g)<< 8)|
				(uint32_t(_b)<< 0)|
				(uint32_t(_a)<<24); 
	}
	
	operator uint32_t() const { return rgbp; }
	
	rgba operator/(const uint32_t div) const {
		return rgba(ru()/div, gu()/div, bu()/div);
	}

	rgba operator*(const uint32_t mul) const {
		return rgba(ru()*mul, gu()*mul, bu()*mul);
	}

	uint8_t r() const { return ((rgbp>>16)&0xFF); };
	uint8_t g() const { return ((rgbp>> 8)&0xFF); };
	uint8_t b() const { return ((rgbp>> 0)&0xFF); };

	int32_t ri() const { return int32_t((rgbp>>16)&0xFF); };
	int32_t gi() const { return int32_t((rgbp>> 8)&0xFF); };
	int32_t bi() const { return int32_t((rgbp>> 0)&0xFF); };
	
	uint32_t ru() const { return uint32_t((rgbp>>16)&0xFF); };
	uint32_t gu() const { return uint32_t((rgbp>> 8)&0xFF); };
	uint32_t bu() const { return uint32_t((rgbp>> 0)&0xFF); };

	static rgba hsvToRgb(uint16_t h, uint8_t s, uint8_t v) {
		uint8_t f = (h % 60) * 255 / 60;
		uint8_t p = (255 - s) * (uint16_t)v / 255;
		uint8_t q = (255 - f * (uint16_t)s / 255) * (uint16_t)v / 255;
		uint8_t t = (255 - (255 - f) * (uint16_t)s / 255) * (uint16_t)v / 255;
		uint8_t r = 0, g = 0, b = 0;
		switch ((h / 60) % 6) {
			case 0: r = v; g = t; b = p; break;
			case 1: r = q; g = v; b = p; break;
			case 2: r = p; g = v; b = t; break;
			case 3: r = p; g = q; b = v; break;
			case 4: r = t; g = p; b = v; break;
			case 5: r = v; g = p; b = q; break;
		}
		return rgba(r, g, b);
	}

private:

	uint32_t rgbp;
};

static const uint8_t rev_bits[] = 
{
  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 
  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 
  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 
  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 
  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 
  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 
  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 
  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 
  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 
  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};

static const uint32_t radio_colors[] = {
	0x808080UL,
	0xA00000UL,
	0x00A000UL,
	0x0000A0UL,
	0x909000UL,
	0x009090UL,
	0x900090UL,
	0x906020UL,
	0x206090UL,
	0x602090UL,
	0x902060UL,
};


static void delay(uint32_t ms) {
    for (volatile uint32_t j = 0; j < ms; j++) {
        Chip_WWDT_Feed(LPC_WWDT);
        for (volatile uint32_t i = 0; i < 3000; i++) {
        }
    }
}

class Random {
public:
	Random(uint32_t seed) {
		uint32_t i;
		a = 0xf1ea5eed, b = c = d = seed;
		for (i=0; i<20; ++i) {
		    (void)get();
		}
	}

	#define rot(x,k) (((x)<<(k))|((x)>>(32-(k))))
	uint32_t get() {
		uint32_t e = a - rot(b, 27);
		a = b ^ rot(c, 17);
		b = c + d;
		c = d + e;
		d = e + a;
		return d;
	}

	uint32_t get(uint32_t lower, uint32_t upper) {
		return (get() % (upper-lower)) + lower;
	}

private:
	uint32_t a; 
	uint32_t b; 
	uint32_t c; 
	uint32_t d; 

};

class I2C_Guard {
public:
	static volatile uint32_t i2c_guard;
	
	I2C_Guard() {
		i2c_guard++;
	}
	
	~I2C_Guard() {
		i2c_guard--;
	}
	
	bool Check() {
		if (i2c_guard > 0) {
			return false;
		}
		return false;
	}
};

volatile uint32_t I2C_Guard::i2c_guard = 0;

class FT25H16S {

public:
	const uint32_t FLASH_MOSI0_PIN = 0x0009; // 0_9
	const uint32_t FLASH_MISO0_PIN = 0x0008; // 0_8
	const uint32_t FLASH_SCK0_PIN = 0x000A; // 0_10
	const uint32_t FLASH_CSEL_PIN = 0x011F; // 1_31
	const uint32_t FLASH_HOLD_PIN = 0x0115; // 1_21

	FT25H16S() {
		// CS#
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), IOCON_FUNC0);
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF));
		// MISO0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_MISO0_PIN>>8), (FLASH_MISO0_PIN&0xFF), IOCON_FUNC0);
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, (FLASH_MISO0_PIN>>8), (FLASH_MISO0_PIN&0xFF));
		// SCK0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_SCK0_PIN>>8), (FLASH_SCK0_PIN&0xFF), IOCON_FUNC1);
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, (FLASH_SCK0_PIN>>8), (FLASH_SCK0_PIN&0xFF));
		// MOSI0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_MOSI0_PIN>>8), (FLASH_MOSI0_PIN&0xFF), IOCON_FUNC0);
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, (FLASH_MOSI0_PIN>>8), (FLASH_MOSI0_PIN&0xFF));
		// HOLD#
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_HOLD_PIN>>8), (FLASH_HOLD_PIN&0xFF), IOCON_FUNC0);
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, (FLASH_HOLD_PIN>>8), (FLASH_HOLD_PIN&0xFF));
	}
	
	bool DevicePresent() {
		return read_rdid_id() == 0x000e4015;
	}
	
	uint32_t read_rdid_id() {
		// MOSI0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_MOSI0_PIN>>8), (FLASH_MOSI0_PIN&0xFF), IOCON_FUNC0);

		// HOLD to high
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_HOLD_PIN>>8), (FLASH_HOLD_PIN&0xFF), true);
		// CSEL to low
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), false);

		push_byte(0x9F);
		push_byte(0x00);
		push_byte(0x00);
		push_byte(0x00);
		
		uint32_t ret = 0;
		ret |= read_byte() << 16;
		ret |= read_byte() <<  8;
		ret |= read_byte() <<  0;

		// CSEL to high
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), true);
		
		return ret;
	}
	
	void read_data(uint32_t address, uint8_t *ptr, uint32_t size) {
		// MOSI0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_MOSI0_PIN>>8), (FLASH_MOSI0_PIN&0xFF), IOCON_FUNC0);

		// HOLD to high
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_HOLD_PIN>>8), (FLASH_HOLD_PIN&0xFF), true);
		// CSEL to low
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), false);

		push_byte(0x03);
		push_byte((address>>16)&0xFF);
		push_byte((address>> 8)&0xFF);
		push_byte((address>> 0)&0xFF);
		
		for (uint32_t c=0; c<size; c++) {
			*ptr++ = read_byte();
		}

		// CSEL to high
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), true);
	}
	
	void write_enable() {
		// MOSI0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_MOSI0_PIN>>8), (FLASH_MOSI0_PIN&0xFF), IOCON_FUNC0);

		// CSEL to low
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), false);
		// HOLD to high
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_HOLD_PIN>>8), (FLASH_HOLD_PIN&0xFF), true);

		push_byte(0x06);

		// CSEL to high
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), true);
	}

	void write_data(uint32_t address, uint8_t *ptr, uint32_t size) {
		write_enable();
		
		// MOSI0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_MOSI0_PIN>>8), (FLASH_MOSI0_PIN&0xFF), IOCON_FUNC0);

		// CSEL to low
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), false);
		// HOLD to high
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_HOLD_PIN>>8), (FLASH_HOLD_PIN&0xFF), true);

		push_byte(0x02);
		push_byte((address>>16)&0xFF);
		push_byte((address>> 8)&0xFF);
		push_byte((address>> 0)&0xFF);
		
		for (uint32_t c=0; c<size; c++) {
			push_byte(*ptr++);
		}

		// CSEL to high
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), true);
		
		while (wip()) { };
	}
	
	void chip_erase() {
		write_enable();
		
		// MOSI0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_MOSI0_PIN>>8), (FLASH_MOSI0_PIN&0xFF), IOCON_FUNC0);

		// CSEL to low
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), false);
		// HOLD to high
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_HOLD_PIN>>8), (FLASH_HOLD_PIN&0xFF), true);

		push_byte(0x60);

		// CSEL to high
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), true);
		
		while (wip()) { };
	}

private:

	bool wip() {
		// MOSI0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (FLASH_MOSI0_PIN>>8), (FLASH_MOSI0_PIN&0xFF), IOCON_FUNC0);

		// HOLD to high
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_HOLD_PIN>>8), (FLASH_HOLD_PIN&0xFF), true);
		// CSEL to low
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), false);

		push_byte(0x05);
		bool in_wip = (read_byte() & 1) ? true : false; 
		
		// CSEL to high
		Chip_GPIO_SetPinState(LPC_GPIO, (FLASH_CSEL_PIN>>8), (FLASH_CSEL_PIN&0xFF), true);

        Chip_WWDT_Feed(LPC_WWDT);

		return in_wip;
	}
	

	void push_byte(uint32_t byte) {
		spiwrite(byte);
	}

	uint32_t read_byte() {
		return spiwrite(0);
	}

	uint8_t spiwrite(uint8_t val) {
		Chip_GPIO_SetPinOutLow(LPC_GPIO, (FLASH_SCK0_PIN>>8), (FLASH_SCK0_PIN&0xFF));
		uint8_t read_value = 0;
		for (uint32_t c=0; c<8; c++) {
			read_value <<= 1;
			Chip_GPIO_SetPinOutLow(LPC_GPIO, (FLASH_SCK0_PIN>>8), (FLASH_SCK0_PIN&0xFF));
			if ((val&(1<<(7-c)))) {
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (FLASH_MOSI0_PIN>>8), (FLASH_MOSI0_PIN&0xFF));
			} else {
				Chip_GPIO_SetPinOutLow(LPC_GPIO, (FLASH_MOSI0_PIN>>8), (FLASH_MOSI0_PIN&0xFF));
			}
			read_value |= Chip_GPIO_GetPinState(LPC_GPIO, (FLASH_MISO0_PIN>>8), (FLASH_MISO0_PIN&0xFF));
			Chip_GPIO_SetPinOutHigh(LPC_GPIO, (FLASH_SCK0_PIN>>8),(FLASH_SCK0_PIN&0xFF));
		}
		return read_value;
	}
};

class EEPROM {

public:
	static bool loaded;

	EEPROM() {
		program_count = 27;
		program_curr = 2;
		program_change_count = 0;
		brightness = 1;
		bird_color_index = 0;
		bird_color = rgba(0x404000UL);
		ring_color_index = 5;
		ring_color = rgba(0x001050UL);
		radio_enabled = true;
		radio_message = 0;
		radio_color = 0;

		memcpy(radio_messages[0], " QUACK! ", 8);
		memcpy(radio_messages[1], "  NOW!  ", 8);
		memcpy(radio_messages[2], "!LASERS!", 8);
		memcpy(radio_messages[3], "!SAFETY!", 8);
		memcpy(radio_messages[4], "ASSEMBLE", 8);
		memcpy(radio_messages[5], "IM DRUNK", 8);
		memcpy(radio_messages[6], "IM HIGH!", 8);
		memcpy(radio_messages[7], "IM GONE!", 8);

		memcpy(&radio_name[0], "  DUCK  ", 8);
		
		runtime_time_count = Chip_TIMER_ReadCount(LPC_TIMER32_0);
	}
	
	void Reset(bool deep) {
		
		if (deep) {
			memset(this, 0, sizeof(EEPROM));

			recv_message_count = 0;
			sent_message_count = 0;
			program_change_count = 0;
			brightness_change_count = 0;
			total_runtime = 0;
			recv_buffer_ptr = 0;
			recv_flash_ptr = 0;
		}

		bird_color_index = 0;
		bird_color = rgba(0x404000UL);
		ring_color_index = 5;
		ring_color = rgba(0x001050UL);
		program_count = 27;
		program_curr = 0;
		program_change_count = 0;
		brightness = 1;
		radio_enabled = true;
		radio_message = 0;
		radio_color = 0;
		
		memcpy(radio_messages[0], " QUACK! ", 8);
		memcpy(radio_messages[1], "  NOW!  ", 8);
		memcpy(radio_messages[2], "!LASERS!", 8);
		memcpy(radio_messages[3], "!SAFETY!", 8);
		memcpy(radio_messages[4], "ASSEMBLE", 8);
		memcpy(radio_messages[5], "IM DRUNK", 8);
		memcpy(radio_messages[6], "IM HIGH!", 8);
		memcpy(radio_messages[7], "IM GONE!", 8);
		
		memcpy(&radio_name[0], "  DUCK  ", 8);
		
		Save();
	}

	void Load() {
		unsigned int param[5] = { 0 };
		param[0] = 62; // Read EEPROM
		param[1] = 0; // EEPROM address
		param[2] = (uintptr_t)this; // RAM address
		param[3] = sizeof(EEPROM); // Number of bytes
		param[4] = SystemCoreClock / 1000; // CCLK
		unsigned int result[4] = { 0 };
		iap_entry(param, result);

		if (program_count != 27 ||
			bird_color == 0UL ||
			bird_color_index > 16 ||
			ring_color == 0UL ||
			ring_color_index > 16 ||
			brightness == 0 ||
			brightness >= 8 ||
			radio_messages[0][0] < 0x20 ||
			radio_name[0] < 0x20) {
				
			Reset(true);
		}

		runtime_time_count = Chip_TIMER_ReadCount(LPC_TIMER32_0);
		
		loaded = true;
	}

	void Save() {
		if (loaded) {
			unsigned int param[5] = { 0 };
			param[0] = 61; // Write EEPROM
			param[1] = 0; // EEPROM address
			param[2] = (uintptr_t)this; // RAM address
			param[3] = sizeof(EEPROM); // Number of bytes
			param[4] = SystemCoreClock / 1000; // CCLK
			unsigned int result[4] = { 0 };
			iap_entry(param, result);
		}
	}

	void NextEffect() {
		if (loaded) {
			program_curr++;
			program_change_count++;
			if (program_curr >= program_count) {
				program_curr = 0;
			}
			Save();
		}
	}
	
	void NextBrightness() {
		if (loaded) {
			brightness ++;
			brightness_change_count ++;
			brightness &= 0x7;
			Save();
		}
	}
	
	void UpdateRecvCount() {
		if (loaded) {
			recv_message_count ++;
			Save();
		}
	}
	
	void UpdateSentCount() {
		if (loaded) {
			sent_message_count ++;
			Save();
		}
	}

	void SaveRuntime() {
		if (loaded) {
			uint32_t new_runtime_time_count = Chip_TIMER_ReadCount(LPC_TIMER32_0);
			total_runtime += new_runtime_time_count - runtime_time_count;
			runtime_time_count = new_runtime_time_count;
			Save();
		}
	}
	
	#define BLOCK_SIZE 256
	#define MSG_SIZE 32
	
	void RecordMessage(FT25H16S &ft25h16s, const uint8_t *msg) {
		memcpy(&recv_buffer[recv_buffer_ptr], msg, 24);
		recv_buffer_ptr += 32;
		if (recv_buffer_ptr >= 0x100) {
			// Spill into flash
			ft25h16s.write_data(recv_flash_ptr, recv_buffer, 256);
			recv_buffer_ptr = 0;
			recv_flash_ptr += BLOCK_SIZE;
		}
		Save();
	}
	
	int32_t GetMessageCount() {
		return (recv_buffer_ptr / MSG_SIZE) + (recv_flash_ptr / MSG_SIZE);
	}
	
	bool GetMessage(FT25H16S &ft25h16s, int32_t msg_num, uint8_t *msg) {
		if (msg_num >= GetMessageCount()) {
			return false;
		}
		if ( msg_num < int32_t(recv_buffer_ptr / MSG_SIZE) ) {
			memcpy(msg, &recv_buffer[recv_buffer_ptr - ( ( msg_num + 1 ) * MSG_SIZE)], MSG_SIZE);
		} else {
			msg_num -= int32_t(recv_buffer_ptr / MSG_SIZE);
			ft25h16s.read_data(recv_flash_ptr - ( ( msg_num + 1 ) * MSG_SIZE), msg, MSG_SIZE);
		}
		return true;
	}
	
	uint32_t program_count;
	uint32_t program_curr;
	uint32_t runtime_time_count;
	rgba bird_color;
	uint32_t bird_color_index;
	rgba ring_color;
	uint32_t ring_color_index;
	uint32_t brightness;
	
	uint32_t radio_enabled;
	uint32_t radio_message;
	uint32_t radio_color;
	char 	 radio_name[8];
	char 	 radio_messages[8][8];
	
	char 	 recv_radio_name[8];
	char 	 recv_radio_message[8];
	uint32_t recv_radio_color;
	uint32_t recv_radio_message_pending;

	// Stats
	uint32_t recv_message_count;
	uint32_t sent_message_count;
	uint32_t program_change_count;
	uint32_t brightness_change_count;
	uint64_t total_runtime;
	
	uint32_t recv_buffer_ptr;
	uint8_t  recv_buffer[256];
	uint32_t recv_flash_ptr;

};

bool EEPROM::loaded = 0;
 
class SPI;

class LEDs {

#define HALF_LEDS 			12

	const uint8_t frnt_ring_indecies[8] = { 0x04*3, 0x05*3, 0x06*3, 0x07*3, 0x08*3, 0x09*3, 0x0A*3, 0x0B*3 };
	const uint8_t back_ring_indecies[8] = { 0x04*3, 0x05*3, 0x06*3, 0x07*3, 0x08*3, 0x09*3, 0x0A*3, 0x0B*3 };
	const uint8_t frnt_bird_indecies[4] = { 0x00*3, 0x01*3, 0x02*3, 0x03*3 };
	const uint8_t back_bird_indecies[4] = { 0x00*3, 0x01*3, 0x02*3, 0x03*3 };

public:

	LEDs() {
		memset(led_data, 0, sizeof(led_data));
	}

	void set_ring(uint32_t index, uint32_t r, uint32_t g, uint32_t b) {
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+1] = r;
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+0] = g;
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+2] = b;
		led_data[HALF_LEDS*1*3+back_ring_indecies[index]+1] = r;
		led_data[HALF_LEDS*1*3+back_ring_indecies[index]+0] = g;
		led_data[HALF_LEDS*1*3+back_ring_indecies[index]+2] = b;
	}

	void set_ring(uint32_t index, const rgba &color) {
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+1] = color.r();
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+0] = color.g();
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+2] = color.b();
		led_data[HALF_LEDS*1*3+back_ring_indecies[index]+1] = color.r();
		led_data[HALF_LEDS*1*3+back_ring_indecies[index]+0] = color.g();
		led_data[HALF_LEDS*1*3+back_ring_indecies[index]+2] = color.b();
	}

	void set_ring_synced(uint32_t index, uint32_t r, uint32_t g, uint32_t b) {
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+1] = r;
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+0] = g;
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+2] = b;
		led_data[HALF_LEDS*1*3+back_ring_indecies[(8-index)&7]+1] = r;
		led_data[HALF_LEDS*1*3+back_ring_indecies[(8-index)&7]+0] = g;
		led_data[HALF_LEDS*1*3+back_ring_indecies[(8-index)&7]+2] = b;
	}

	void set_ring_synced(uint32_t index, const rgba &color) {
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+1] = color.r();
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+0] = color.g();
		led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+2] = color.b();
		led_data[HALF_LEDS*1*3+back_ring_indecies[(8-index)&7]+1] = color.r();
		led_data[HALF_LEDS*1*3+back_ring_indecies[(8-index)&7]+0] = color.g();
		led_data[HALF_LEDS*1*3+back_ring_indecies[(8-index)&7]+2] = color.b();
	}

	void set_ring_all(uint32_t index, uint32_t r, uint32_t g, uint32_t b) {
		if(index < 8) { 
			led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+1] = r;
			led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+0] = g;
			led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+2] = b;
		} else if (index < 16) {
			led_data[HALF_LEDS*1*3+back_ring_indecies[index-8]+1] = r;
			led_data[HALF_LEDS*1*3+back_ring_indecies[index-8]+0] = g;
			led_data[HALF_LEDS*1*3+back_ring_indecies[index-8]+2] = b;
		}
	}

	void set_ring_all(uint32_t index, const rgba &color) {
		if(index < 8) { 
			led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+1] = color.r();
			led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+0] = color.g();
			led_data[HALF_LEDS*0*3+frnt_ring_indecies[index]+2] = color.b();
		} else if (index < 16) {
			led_data[HALF_LEDS*1*3+back_ring_indecies[index-8]+1] = color.r();
			led_data[HALF_LEDS*1*3+back_ring_indecies[index-8]+0] = color.g();
			led_data[HALF_LEDS*1*3+back_ring_indecies[index-8]+2] = color.b();
		}
	}

	void set_bird(uint32_t index, uint32_t r, uint32_t g, uint32_t b) {
		led_data[HALF_LEDS*0*3+frnt_bird_indecies[index]+1] = r;
		led_data[HALF_LEDS*0*3+frnt_bird_indecies[index]+0] = g;
		led_data[HALF_LEDS*0*3+frnt_bird_indecies[index]+2] = b;
		led_data[HALF_LEDS*1*3+back_bird_indecies[index]+1] = r;
		led_data[HALF_LEDS*1*3+back_bird_indecies[index]+0] = g;
		led_data[HALF_LEDS*1*3+back_bird_indecies[index]+2] = b;
	}

	void set_bird(uint32_t index, const rgba &color) {
		led_data[HALF_LEDS*0*3+frnt_bird_indecies[index]+1] = color.r();
		led_data[HALF_LEDS*0*3+frnt_bird_indecies[index]+0] = color.g();
		led_data[HALF_LEDS*0*3+frnt_bird_indecies[index]+2] = color.b();
		led_data[HALF_LEDS*1*3+back_bird_indecies[index]+1] = color.r();
		led_data[HALF_LEDS*1*3+back_bird_indecies[index]+0] = color.g();
		led_data[HALF_LEDS*1*3+back_bird_indecies[index]+2] = color.b();
	}
	
private:
	friend class SPI;

	uint8_t led_data[2*HALF_LEDS*3];

};

class SPI {
public:
	const uint32_t BOTTOM_LED_MOSI0_PIN = 0x0009; // 0_9
	const uint32_t BOTTOM_LED_SCK0_PIN = 0x011D; // 1_29

	const uint32_t TOP_LED_MOSI1_PIN = 0x0116; // 1_22
	const uint32_t TOP_LED_SCK1_PIN = 0x010F; // 1_15

	SPI() {
		// MOSI0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (BOTTOM_LED_MOSI0_PIN>>8), (BOTTOM_LED_MOSI0_PIN&0xFF), IOCON_FUNC1);
		// SCK0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (BOTTOM_LED_SCK0_PIN>>8), (BOTTOM_LED_SCK0_PIN&0xFF), IOCON_FUNC1);
		
		// MOSI1
		Chip_IOCON_PinMuxSet(LPC_IOCON, (TOP_LED_MOSI1_PIN>>8), (TOP_LED_MOSI1_PIN&0xFF), IOCON_FUNC2);
		// SCK1
		Chip_IOCON_PinMuxSet(LPC_IOCON, (TOP_LED_SCK1_PIN>>8), (TOP_LED_SCK1_PIN&0xFF), IOCON_FUNC3);

		Chip_SSP_Init(LPC_SSP0);
	    Chip_SSP_SetMaster(LPC_SSP0, 1);
		Chip_SSP_SetClockRate(LPC_SSP0, 0, 2);
		Chip_SSP_SetFormat(LPC_SSP0, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);
		Chip_SSP_Enable(LPC_SSP0);

	    Chip_SSP_SetMaster(LPC_SSP1, 1);
		Chip_SSP_Init(LPC_SSP1);
		Chip_SSP_SetClockRate(LPC_SSP1, 0, 2);
		Chip_SSP_SetFormat(LPC_SSP1, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);
		Chip_SSP_Enable(LPC_SSP1);
	}

	void push_frame(LEDs &leds, int32_t brightness = 0x01)  {
		// MOSI0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (BOTTOM_LED_MOSI0_PIN>>8), (BOTTOM_LED_MOSI0_PIN&0xFF), IOCON_FUNC1);

		// Set format again
		Chip_SSP_SetClockRate(LPC_SSP0, 0, 2);
		Chip_SSP_SetFormat(LPC_SSP0, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);

		// Start frame
		push_byte_top(0);
		push_byte_btm(0);
		push_byte_top(0);
		push_byte_btm(0);
		push_byte_top(0);
		push_byte_btm(0);
		push_byte_top(0);
		push_byte_btm(0);

		// Frame data
		for (int32_t c=0; c<HALF_LEDS; c++) {
			push_byte_top(0xE0 | max(0L, (brightness * 4) - 3) );
			push_byte_btm(0xE0 | max(0L, (brightness * 4) - 3) );
			push_byte_top((leds.led_data[HALF_LEDS*0*3 + c*3+2] & ~(3)) + 4);
			push_byte_btm((leds.led_data[HALF_LEDS*1*3 + c*3+2] & ~(3)) + 4);
			push_byte_top((leds.led_data[HALF_LEDS*0*3 + c*3+0] & ~(3)) + 4);
			push_byte_btm((leds.led_data[HALF_LEDS*1*3 + c*3+0] & ~(3)) + 4);
			push_byte_top((leds.led_data[HALF_LEDS*0*3 + c*3+1] & ~(3)) + 4);
			push_byte_btm((leds.led_data[HALF_LEDS*1*3 + c*3+1] & ~(3)) + 4);
		}

		// End frame
		push_byte_top(0xFF);
		push_byte_btm(0xFF);
		push_byte_top(0xFF);
		push_byte_btm(0xFF);
		push_byte_top(0xFF);
		push_byte_btm(0xFF);
		push_byte_top(0xFF);
		push_byte_btm(0xFF);
	}

	void push_null()  {
		// MOSI0
		Chip_IOCON_PinMuxSet(LPC_IOCON, (BOTTOM_LED_MOSI0_PIN>>8), (BOTTOM_LED_MOSI0_PIN&0xFF), IOCON_FUNC1);

		// Set format again
		Chip_SSP_SetClockRate(LPC_SSP0, 0, 2);
		Chip_SSP_SetFormat(LPC_SSP0, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_MODE0);

		// Start frame
		push_byte_top(0);
		push_byte_btm(0);
		push_byte_top(0);
		push_byte_btm(0);
		push_byte_top(0);
		push_byte_btm(0);
		push_byte_top(0);
		push_byte_btm(0);

		// Frame data
		for (int32_t c=0; c<HALF_LEDS; c++) {
			push_byte_top(0xE0);
			push_byte_btm(0xE0);
			
			push_byte_top(0);
			push_byte_btm(0);
			push_byte_top(0);
			push_byte_btm(0);
			push_byte_top(0);
			push_byte_btm(0);
		}

		// End frame
		push_byte_top(0xFF);
		push_byte_btm(0xFF);
		push_byte_top(0xFF);
		push_byte_btm(0xFF);
		push_byte_top(0xFF);
		push_byte_btm(0xFF);
		push_byte_top(0xFF);
		push_byte_btm(0xFF);
	}

private:
	
	void push_byte_top(uint8_t byte) {
		while(!Chip_SSP_GetStatus(LPC_SSP1, SSP_STAT_TNF));
		Chip_SSP_SendFrame(LPC_SSP1, byte);
	}
	
	void push_byte_btm(uint8_t byte) {
		while(!Chip_SSP_GetStatus(LPC_SSP0, SSP_STAT_TNF));
		Chip_SSP_SendFrame(LPC_SSP0, byte);
	}
};

class BQ24295 {
	
	public:
			static const uint32_t i2caddr = 0x6B;
	
			BQ24295() {
				devicePresent = false;
				fault_state = 0;
			}
			
			void SetBoostVoltage (uint32_t voltageMV) {
				I2C_Guard guard;
				if (guard.Check()) {
					return;
				}
				
				uint8_t reg = getRegister(0x06);
				if ((voltageMV >= 4550) && (voltageMV <= 5510)) {
					uint32_t codedValue = voltageMV;
					codedValue = (codedValue - 4550) / 64;
					if ((voltageMV - 4550) % 64 != 0) {
						codedValue++;
					}
					reg &= ~(0x0f << 4);
					reg |= (uint8_t) ((codedValue & 0x0f) << 4);
					setRegister (0x06, reg);
				}
			}
			
			uint32_t GetBoostVoltage () {
				I2C_Guard guard;
				if (guard.Check()) {
					return 0;
				}

				uint8_t reg = getRegister(0x06);
				reg = (reg >> 4) & 0x0f;
				return 4550 + ((uint32_t) reg) * 64;
			}

			void SetBoostUpperTemperatureLimit (uint32_t temperatureC) {
				I2C_Guard guard;
				if (guard.Check()) {
					return;
				}

				uint8_t reg = getRegister(0x06);
				uint8_t codedValue = 0;
				if (temperatureC < 60) {
					codedValue = 0;
				} else if (temperatureC < 65) {
					codedValue = 1;
				} else {
					codedValue = 2;
				}
				reg &= ~(0x03 << 2);
				reg |= (uint8_t) (codedValue << 2);
				setRegister (0x06, reg);
			}

			uint32_t GetBoostUpperTemperatureLimit () {
				I2C_Guard guard;
				if (guard.Check()) {
					return 0;
				}

				uint8_t reg = getRegister(0x06);
				if (((reg >> 2) & 0x03) != 0x03) {
					switch ((reg >> 2) & 0x03) {
						case 0:
							return 55;
						break;
						case 1:
							return 60;
						break;
						case 2:
							return 65;
						break;
					}
				}
				return 0;
			}

			void SetInputCurrentLimit(uint32_t currentMA) {
				I2C_Guard guard;
				if (guard.Check()) {
					return;
				}

				uint8_t reg = 0;
				if ((reg = getRegister(0x00)) != 0) {
					// Input current limit is in bits 0 to 2, coded
					// such that the smallest limit is applied for
					// a range (e.g. 120 mA ends up as 100 mA rather
					// than 150 mA)
					if ((currentMA >= 100) && (currentMA <= 3000)) {
						uint8_t codedValue = 0;
						if (currentMA < 150) {
							codedValue = 0;
						} else if (currentMA < 500) {
							codedValue = 1;
						} else if (currentMA < 900) {
							codedValue = 2;
						} else if (currentMA < 1000) {
							codedValue = 3;
						} else if (currentMA < 1500) {
							codedValue = 4;
						} else if (currentMA < 2000) {
							codedValue = 5;
						} else if (currentMA < 3000) {
							codedValue = 6;
						} else {
							codedValue = 7;
						}                
						reg &= ~(0x07 << 0);
						reg |= codedValue;
						setRegister (0x00, reg);
					}
				}
			}
			
			uint32_t GetInputCurrentLimit() {
				I2C_Guard guard;
				if (guard.Check()) {
					return 0;
				}

				uint8_t reg = getRegister(0x00);
				switch (reg & 0x07) {
					case 0:
						return 100;
					break;
					case 1:
						return 150;
					break;
					case 2:
						return 500;
					break;
					case 3:
						return 900;
					break;
					case 4:
						return 1000;
					break;
					case 5:
						return 1500;
					break;
					case 6:
						return 2000;
					break;
					case 7:
						return 3000;
					break;
				}
				return 0;
			}

			void EnableInputLimits() {
				I2C_Guard guard;
				if (guard.Check()) {
					return;
				}
				setRegisterBits(0x00, (1 << 7));
			}
			
			void DisableInputLimits() {
				I2C_Guard guard;
				if (guard.Check()) {
					return;
				}
				clearRegisterBits(0x00, (1 << 7));
			}
			
			void DisableWatchdog() {
				clearRegisterBits(0x05, (1 << 4));
				clearRegisterBits(0x05, (1 << 5));
			}
			
			void DisableOTG() { 
				clearRegisterBits(0x01, (1 << 5));
			}

			void SetChipThermalRegulationThreshold(uint32_t temperatureC) {
				I2C_Guard guard;
				if (guard.Check()) {
					return;
				}
				uint8_t reg = getRegister(0x06);
				uint8_t codedValue = 0;
				if (temperatureC < 80) {
					codedValue = 0;
				} else if (temperatureC < 100) {
					codedValue = 1;
				} else if (temperatureC < 120) {
					codedValue = 2;
				} else {
					codedValue = 3;
				}
				reg &= ~0x03;
				reg |= (uint8_t) codedValue;
				setRegister (0x06, reg);
			}
			
			uint32_t GetChipThermalRegulationThreshold() {
				I2C_Guard guard;
				if (guard.Check()) {
					return 0;
				}
				uint8_t reg = getRegister(0x06);
				switch (reg & 0x03) {
					case 0:
						return 60;
					break;
					case 1:
						return 80;
					break;
					case 2:
						return 100;
					break;
					case 3:
						return 120;
					break;
				}
				return 0;
			}
			
			uint8_t GetStatus() {
				I2C_Guard guard;
				if (guard.Check()) {
					return 0;
				}
				return getRegister(0x08);
			}
			
			uint8_t FaultState() {
				return fault_state;
			}
			
			bool IsInFaultState() {
				I2C_Guard guard;
				if (guard.Check()) {
					return 0;
				}
				uint8_t reg = getRegister(0x09);
				fault_state = reg;
				return reg != 0;
			}
			
			bool DevicePresent() const { return devicePresent; }

	private:

			uint8_t getRegister(uint8_t address) {
				uint8_t value = 0;
				Chip_I2C_MasterSend(I2C0, i2caddr, &address, 1);
				Chip_I2C_MasterRead(I2C0, i2caddr, &value, 1);
				return value;
			}

			void setRegister(uint8_t address, uint8_t value) {
				uint8_t set[2];
				set[0] = address;
				set[1] = value;
				Chip_I2C_MasterSend(I2C0, i2caddr, &set[0], 2);
			}

			void setRegisterBits(uint8_t address, uint8_t mask) {
				uint8_t value = getRegister(address);
				value |= mask;
				setRegister(address, value);
			}

			void clearRegisterBits(uint8_t address, uint8_t mask) {
				uint8_t value = getRegister(address);
				value &= ~mask;
				setRegister(address, value);
			}
			
			friend class Setup;

			bool devicePresent;
			uint8_t fault_state;
};


class SDD1306 {

	public:
	
 			static const uint32_t i2caddr = 0x3C;

			SDD1306() {
				devicePresent = false;
				display_scroll_message = false;
				scroll_message_offset = 0;
			}
			
			void Clear() {
				memset(text_buffer_cache, 0, sizeof(text_buffer_cache));
				memset(text_buffer_screen, 0, sizeof(text_buffer_screen));
				memset(text_attr_cache, 0, sizeof(text_attr_cache));
				memset(text_attr_screen, 0, sizeof(text_attr_screen));
				center_flip_screen = 0;
				center_flip_cache = 0;
			}
			
			void ClearAttr() {
				memset(text_attr_cache, 0, sizeof(text_attr_cache));
			}
			
			void DisplayBootScreen() {
				for (uint32_t c=0; c<8*4; c++) {
					text_buffer_cache[c] = 0x80 + c;
				}
			}
			
			void SetCenterFlip(int8_t progression) {
				center_flip_cache = progression;
			}

			void PlaceAsciiStr(uint32_t x, uint32_t y, const char *str) {
				if (y>3 || x>7) return;
				size_t len = strlen(str);
				if (x+len > 8) len = 7-x;
				for (size_t c=0; c<len; c++) {
					uint8_t ch = uint8_t(str[c]);
					if ((ch < 0x20) || (ch >= 0x7D)) {
						text_buffer_cache[y*8+x+c] = 0;
					} else {
						text_buffer_cache[y*8+x+c] = uint8_t(str[c]) - 0x20;
					}
				}
			}

			void PlaceCustomChar(uint32_t x, uint32_t y, uint16_t code) {
				if (y>3 || x>7) return;
				text_buffer_cache[y*8+x] = code;
			}
			
			void Invert() {
				for (uint32_t c=0; c<8*4; c++) {
					text_attr_cache[c] ^= 1;
				}
			}
			
			void SetAttr(uint32_t x, uint32_t y, uint8_t attr) {
				if (y>3 || x>7) return;
				text_attr_cache[y*8+x] = attr;
			}
			
			void SetAsciiScrollMessage(const char *str, int32_t offset) {
				if (str) {
					size_t len = min(strlen(str), size_t(8));
					memset(scroll_message, 0, 9);
					for (int32_t c=0; c<len; c++) {
						uint8_t ch = uint8_t(str[c]);
						if ((ch < 0x20) || (ch >= 0x7D)) {
							scroll_message[c] = 0x20;
						} else {
							scroll_message[c] = ch - 0x20;
						}
					}
					display_scroll_message = true;
					scroll_message_offset = offset;
				} else {
					display_scroll_message = false;
				}
			}

			void Display() {
				bool display_center_flip = false;
				if (center_flip_cache || center_flip_screen) {
					center_flip_screen = center_flip_cache;
					display_center_flip = true;
				}
				for (uint32_t y=0; y<4; y++) {
					if (display_scroll_message && y < 2) {
						I2C_Guard guard;
						if (guard.Check()) {
							return;
						}
						
						{
							WriteCommand(0xB0+0);
							int32_t x=32;
							WriteCommand(0x0f&(x   )); // 0x20 offset
							WriteCommand(0x10|(x>>4)); // 0x20 offset
						}

						uint8_t buf[65];
						buf[0] = 0x40;
						
						// write first line
						for (int32_t x = 0; x < 64; x++) {
							int32_t rx = (scroll_message_offset + x ) % (9 * 16) ;
							int32_t cx = rx >> 4;
							buf[x+1] = duck_font_raw[0x1000 + scroll_message[cx] * 16 + (rx & 0x0F)];
						}

						Chip_I2C_MasterSend(I2C0, i2caddr, buf, 0x41);
						
						{
							WriteCommand(0xB0+1);
							int32_t x=32;
							WriteCommand(0x0f&(x   )); // 0x20 offset
							WriteCommand(0x10|(x>>4)); // 0x20 offset
						}

						// write first line
						for (int32_t x = 0; x < 64; x++) {
							int32_t rx = (scroll_message_offset + x ) % (9 * 16) ;
							int32_t cx = rx >> 4;
							buf[x+1] = duck_font_raw[0x1800 + scroll_message[cx] * 16 + (rx & 0x0F)];
						}
						
						Chip_I2C_MasterSend(I2C0, i2caddr, buf, 0x41);
						
					} else {
						for (uint32_t x=0; x<8; x++) {
							if (text_buffer_cache[y*8+x] != text_buffer_screen[y*8+x] ||
								text_attr_cache[y*8+x] != text_attr_screen[y*8+x]) {
								text_buffer_screen[y*8+x] = text_buffer_cache[y*8+x];
								text_attr_screen[y*8+x] = text_attr_cache[y*8+x];
								if (!display_center_flip) {
									DisplayChar(x,y,text_buffer_screen[y*8+x],text_attr_screen[y*8+x]);
								}
							}
						}
					}
				}
				if (display_center_flip) {
					DisplayCenterFlip();
				}
				Chip_WWDT_Feed(LPC_WWDT);
			}
			
			void SetVerticalShift(int8_t val) {
				WriteCommand(0xD3);
				if (val < 0) {
					val = 64+val;
					WriteCommand(val&0x3F);
				} else {
					WriteCommand(val&0x3F);
				}
			}

			void DisplayOn() {
				WriteCommand(0xAF);
			}

			void DisplayOff() {
				WriteCommand(0xAE);
			}

			void DisplayUID() {
				unsigned int param[1] = { 0 };
				param[0] = 58; // Read UID
				unsigned int result[4] = { 0 };
				iap_entry(param, result);

				char str[32];
				sprintf(str,"%08x",result[0]);
				PlaceAsciiStr(0,0,str);
				
				sprintf(str,"%08x",result[1]);
				PlaceAsciiStr(0,1,str);
				
				sprintf(str,"%08x",result[2]);
				PlaceAsciiStr(0,2,str);
				
				sprintf(str,"%08x",result[3]);
				PlaceAsciiStr(0,3,str);
				
				Display();
			}
			
			void Init() const {

				// Toggle RESET line
				Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 22);
				Chip_GPIO_SetPinState(LPC_GPIO, 0, 22, true);
				delay(1);
				Chip_GPIO_SetPinState(LPC_GPIO, 0, 22, false);
				delay(10);
				Chip_GPIO_SetPinState(LPC_GPIO, 0, 22, true);


				static uint8_t startup_sequence[] = {
					0xAE,			// Display off

					0xD5, 0x80,		// Set Display Clock Divide Ratio
					
					0xA8, 0x1F,		// Set Multiplex Ratio
					
					0xD3, 0x00,		// Set Display Offset

					0x8D, 0x14,		// Enable Charge Pump

					0x40,			// Set Display RAM start

					0xA6,			// Set to normal display (0xA7 == inverse)
					
					0xA4,			// Force Display From RAM On

					0xA1,			// Set Segment Re-map

					0xC8,			// Set COM Output Scan Direction (flipped)
					
					0xDA, 0x12, 	// Set Pins configuration
				
					0x81, 0x00,		// Set Contrast (0x00-0xFF)
					
					0xD9, 0xF1,		// Set Pre-Charge period

					0xDB, 0x40,		// Adjust Vcomm regulator output

					0xAF			// Display on
				};

				for (size_t c = 0; c < sizeof(startup_sequence); c++) {
					WriteCommand(startup_sequence[c]);
				}

			}

			bool DevicePresent() const { return devicePresent; }

	private:

			void DisplayCenterFlip() {
				I2C_Guard guard;
				if (guard.Check()) {
					return;
				}
				
				(void)duck_font_raw_len;
				
				uint8_t buf[65];
				buf[0] = 0x40;
				for (uint32_t y=0; y<4; y++) {
					WriteCommand(0xB0+y);
					uint32_t sx = 32;
					WriteCommand(0x0f&(sx   )); // 0x20 offset
					WriteCommand(0x10|(sx>>4)); // 0x20 offset
					for (uint32_t x = 0; x < 64; x++) {
						if (center_flip_screen == 32) {
							buf[x+1] = 0x00;
						} else {
							int32_t rx = ( ( ( int32_t(x) - 32 ) * 32 ) / int32_t(32 - center_flip_screen) ) + 32;
							if (rx < 0 || rx > 63) { 
								buf[x+1] = 0x00;
							} else {
                                uint8_t a = text_attr_screen[y*8+rx/8];
                                uint8_t r = (a & 4) ? (7-(rx&7)) : (rx&7);
                                uint8_t v = duck_font_raw[text_buffer_screen[y*8+rx/8]*8+r];
								if (a & 1) {
									v = ~v;
								}
                                if (a & 2) {
									v = rev_bits[v];
								}
                                buf[x+1] = v;
							}
						}
					}
					Chip_I2C_MasterSend(I2C0, i2caddr, buf, 0x41);
				} 
			}
			
			void DisplayChar(uint32_t x, uint32_t y, uint16_t ch, uint8_t attr) {
				I2C_Guard guard;
				if (guard.Check()) {
					return;
				}
				WriteCommand(0xB0+y);
				x=x*8+32;
				WriteCommand(0x0f&(x   )); // 0x20 offset
				WriteCommand(0x10|(x>>4)); // 0x20 offset
				
				uint8_t buf[9];
				buf[0] = 0x40;
                if ((attr & 4)) {
                    if ((attr & 1)) {
                        if ((attr & 2)) {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] = ~rev_bits[duck_font_raw[ch*8+7-c]];
                            }
                        } else {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] = ~duck_font_raw[ch*8+7-c];
                            }
                        }
                    } else {
                        if ((attr & 2)) {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] =  rev_bits[duck_font_raw[ch*8+7-c]];
                            }
                        } else {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] =  duck_font_raw[ch*8+7-c];
                            }
                        }
                    }
                } else {
                    if ((attr & 1)) {
                        if ((attr & 2)) {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] = ~rev_bits[duck_font_raw[ch*8+c]];
                            }
                        } else {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] = ~duck_font_raw[ch*8+c];
                            }
                        }
                    } else {
                        if ((attr & 2)) {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] =  rev_bits[duck_font_raw[ch*8+c]];
                            }
                        } else {
                            for (uint32_t c=0; c<8; c++) {
                                buf[c+1] =  duck_font_raw[ch*8+c];
                            }
                        }
                    }
                }
				Chip_I2C_MasterSend(I2C0, i2caddr, buf, 0x9);
			}

			void WriteCommand(uint8_t v) const {
				I2C_Guard guard;
				if (guard.Check()) {
					return;
				}
				uint8_t control[2];
				control[0] = 0;
				control[1] = v;
				Chip_I2C_MasterSend(I2C0, i2caddr, control, 2);
			}


			friend class Setup;

			bool devicePresent;

			int8_t center_flip_screen;
			int8_t center_flip_cache;
			uint16_t text_buffer_cache[8*4];
			uint16_t text_buffer_screen[8*4];
			uint8_t text_attr_cache[8*4];
			uint8_t text_attr_screen[8*4];
			
			bool display_scroll_message;
			uint8_t scroll_message[9];
			int32_t scroll_message_offset;
};  // class SDD1306

class Setup {

	public:
            Setup(SDD1306 &sdd1306, BQ24295 &bq24295) {
                InitWWDT();
				InitGPIO();
				InitPININT();
				InitADC();
				InitI2C(sdd1306, bq24295);
				InitTimer();
			}

			void ProbeI2CSlaves(SDD1306 &sdd1306, BQ24295 &bq24295) {
				int i;
				uint8_t ch[2];

				for (i = 0; i <= 0x7F; i++) {
					if (!(i & 0x0F)) {
					}
					if ((i <= 7) || (i > 0x78)) {
						continue;
					}
					if (Chip_I2C_MasterRead(I2C0, i, ch, 1 + (i == 0x48)) > 0) {
						switch(i) {
							case	sdd1306.i2caddr:
									sdd1306.devicePresent = true;
									break;
							case	bq24295.i2caddr:
									bq24295.devicePresent = true;
									break;
						}
					}
				}
			}

	private:
	
            void InitWWDT() {
                Chip_WWDT_Init(LPC_WWDT);

                Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_WDTOSC_PD);
                Chip_Clock_SetWDTOSC(WDTLFO_OSC_1_05, 20);
                
                // 1s watchdog timer
                Chip_WWDT_SelClockSource(LPC_WWDT, WWDT_CLKSRC_WATCHDOG_WDOSC);
                Chip_WWDT_SetTimeOut(LPC_WWDT, 4 * Chip_Clock_GetWDTOSCRate() / 4);
                Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);
                Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF | WWDT_WDMOD_WDINT);

                NVIC_ClearPendingIRQ(WDT_IRQn);
                NVIC_EnableIRQ(WDT_IRQn);
                
                Chip_WWDT_Start(LPC_WWDT);
            }
    
			void InitGPIO() {
				Chip_GPIO_Init(LPC_GPIO);
			}
			
			void InitADC() {
				ADC_CLOCK_SETUP_T setup;
				Chip_ADC_Init(LPC_ADC, &setup);
				Chip_ADC_EnableChannel(LPC_ADC, ADC_CH5, ENABLE);
				Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 16, IOCON_FUNC1 | IOCON_ADMODE_EN);
			}
			
			void InitPININT() {
				Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_PINT);
				
				Chip_PININT_Init(LPC_PININT);
				Chip_PININT_EnableIntHigh(LPC_PININT, 0);
				Chip_PININT_EnableIntLow(LPC_PININT, 0);
			}
			
			void InitI2C(SDD1306 &sdd1306, BQ24295 &bq24295) {

#define SPEED_100KHZ         100000
#define SPEED_400KHZ         400000
#define I2C_DEFAULT_SPEED    SPEED_400KHZ
#define I2C_FASTPLUS_BIT     0

#if (I2C_DEFAULT_SPEED > SPEED_400KHZ)
#undef  I2C_FASTPLUS_BIT
#define I2C_FASTPLUS_BIT IOCON_FASTI2C_EN
#endif

				Chip_SYSCTL_PeriphReset(RESET_I2C0);

				Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 4, IOCON_FUNC1 | I2C_FASTPLUS_BIT);
				Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 5, IOCON_FUNC1 | I2C_FASTPLUS_BIT);

				Chip_I2C_Init(I2C0);
				Chip_I2C_SetClockRate(I2C0, I2C_DEFAULT_SPEED);

				NVIC_DisableIRQ(I2C0_IRQn);
				Chip_I2C_SetMasterEventHandler(I2C0, Chip_I2C_EventHandlerPolling);

				ProbeI2CSlaves(sdd1306, bq24295);
			}
			
			void InitTimer() {
				Chip_TIMER_Init(LPC_TIMER32_0);
				Chip_TIMER_Reset(LPC_TIMER32_0);
				Chip_TIMER_PrescaleSet(LPC_TIMER32_0, (Chip_Clock_GetSystemClockRate() / 1000) - 1);
				Chip_TIMER_Enable(LPC_TIMER32_0);
			}

};  // class Setup {


class SX1280 {
	public:
			enum {
				REG_LR_FIRMWARE_VERSION_MSB             = 0x0153,
				REG_LR_PACKETPARAMS                     = 0x0903,
				REG_LR_PAYLOADLENGTH                    = 0x0901,
				REG_LR_CRCSEEDBASEADDR                  = 0x09C8,
				REG_LR_CRCPOLYBASEADDR                  = 0x09C6,
				REG_LR_WHITSEEDBASEADDR                 = 0x09C5,
				REG_LR_RANGINGIDCHECKLENGTH             = 0x0931,
				REG_LR_DEVICERANGINGADDR                = 0x0916,
				REG_LR_REQUESTRANGINGADDR               = 0x0912,
				REG_LR_RANGINGRESULTCONFIG              = 0x0924,
				REG_LR_RANGINGRESULTBASEADDR            = 0x0961,
				REG_LR_RANGINGRESULTSFREEZE             = 0x097F,
				REG_LR_RANGINGRERXTXDELAYCAL            = 0x092C,
				REG_LR_RANGINGFILTERWINDOWSIZE          = 0x091E,
				REG_LR_RANGINGRESULTCLEARREG            = 0x0923,
				REG_LR_SYNCWORDBASEADDRESS1             = 0x09CE,
				REG_LR_SYNCWORDBASEADDRESS2             = 0x09D3,
				REG_LR_SYNCWORDBASEADDRESS3             = 0x09D8,
				REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB    = 0x0954,
				REG_LR_SYNCWORDTOLERANCE                = 0x09CD,
				REG_LR_PREAMBLELENGTH                   = 0x09C1,
				REG_LR_BLE_ACCESS_ADDRESS               = 0x09CF,
			};
			
			enum {
				REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK   = 0x0FFFFF,
			};
	
			enum RadioCommand {
				RADIO_GET_STATUS                        = 0xC0,
				RADIO_WRITE_REGISTER                    = 0x18,
				RADIO_READ_REGISTER                     = 0x19,
				RADIO_WRITE_BUFFER                      = 0x1A,
				RADIO_READ_BUFFER                       = 0x1B,
				RADIO_SET_SLEEP                         = 0x84,
				RADIO_SET_STANDBY                       = 0x80,
				RADIO_SET_FS                            = 0xC1,
				RADIO_SET_TX                            = 0x83,
				RADIO_SET_RX                            = 0x82,
				RADIO_SET_RXDUTYCYCLE                   = 0x94,
				RADIO_SET_CAD                           = 0xC5,
				RADIO_SET_TXCONTINUOUSWAVE              = 0xD1,
				RADIO_SET_TXCONTINUOUSPREAMBLE          = 0xD2,
				RADIO_SET_PACKETTYPE                    = 0x8A,
				RADIO_GET_PACKETTYPE                    = 0x03,
				RADIO_SET_RFFREQUENCY                   = 0x86,
				RADIO_SET_TXPARAMS                      = 0x8E,
				RADIO_SET_CADPARAMS                     = 0x88,
				RADIO_SET_BUFFERBASEADDRESS             = 0x8F,
				RADIO_SET_MODULATIONPARAMS              = 0x8B,
				RADIO_SET_PACKETPARAMS                  = 0x8C,
				RADIO_GET_RXBUFFERSTATUS                = 0x17,
				RADIO_GET_PACKETSTATUS                  = 0x1D,
				RADIO_GET_RSSIINST                      = 0x1F,
				RADIO_SET_DIOIRQPARAMS                  = 0x8D,
				RADIO_GET_IRQSTATUS                     = 0x15,
				RADIO_CLR_IRQSTATUS                     = 0x97,
				RADIO_CALIBRATE                         = 0x89,
				RADIO_SET_REGULATORMODE                 = 0x96,
				RADIO_SET_SAVECONTEXT                   = 0xD5,
				RADIO_SET_AUTOTX                        = 0x98,
				RADIO_SET_AUTOFS                        = 0x9E,
				RADIO_SET_LONGPREAMBLE                  = 0x9B,
				RADIO_SET_UARTSPEED                     = 0x9D,
				RADIO_SET_RANGING_ROLE                  = 0xA3,
			};

			enum IrqRangingCode
			{
				IRQ_RANGING_SLAVE_ERROR_CODE            = 0x00,
				IRQ_RANGING_SLAVE_VALID_CODE,
				IRQ_RANGING_MASTER_ERROR_CODE,
				IRQ_RANGING_MASTER_VALID_CODE,
			};

			enum IrqErrorCode
			{
				IRQ_HEADER_ERROR_CODE                   = 0x00,
				IRQ_SYNCWORD_ERROR_CODE,
				IRQ_CRC_ERROR_CODE,
				IRQ_RANGING_ON_LORA_ERROR_CODE,
			};

			enum IrqValidCode
			{
				IRQ_HEADER_VALID_CODE                   = 0x00,
				IRQ_SYNCWORD_VALID_CODE,
			};

			enum RadioStates
			{
				RF_IDLE                                 = 0x00,         //!< The radio is idle
				RF_RX_RUNNING,                                          //!< The radio is in reception state
				RF_TX_RUNNING,                                          //!< The radio is in transmission state
				RF_CAD,                                                 //!< The radio is doing channel activity detection
			};
			
			enum RadioCommandStatus
			{
				CMD_SUCCESS								= 0x01,
				CMD_DATA_AVAILABLE						= 0x02,
				CMD_TIMEOUT								= 0x03,
				CMD_ERROR								= 0x04,
				CMD_FAIL								= 0x05,
				CMD_TX_DONE								= 0x06,
			};

			enum RadioOperatingModes
			{
				MODE_SLEEP                              = 0x00,         //! The radio is in sleep mode
				MODE_CALIBRATION,                                       //! The radio is in calibration mode
				MODE_STDBY_RC,                                          //! The radio is in standby mode with RC oscillator
				MODE_STDBY_XOSC,                                        //! The radio is in standby mode with XOSC oscillator
				MODE_FS,                                                //! The radio is in frequency synthesis mode
				MODE_RX,                                                //! The radio is in receive mode
				MODE_TX,                                                //! The radio is in transmit mode
				MODE_CAD                                                //! The radio is in channel activity detection mode
			};

			enum RadioStandbyModes
			{
				STDBY_RC                                = 0x00,
				STDBY_XOSC                              = 0x01,
			};

			enum RadioRegulatorModes
			{
				USE_LDO                                 = 0x00,         //! Use LDO (default value)
				USE_DCDC                                = 0x01,         //! Use DCDC
			};

			enum  RadioPacketTypes
			{
				PACKET_TYPE_GFSK                        = 0x00,
				PACKET_TYPE_LORA,
				PACKET_TYPE_RANGING,
				PACKET_TYPE_FLRC,
				PACKET_TYPE_BLE,
				PACKET_TYPE_NONE                        = 0x0F,
			};

			enum  RadioRampTimes
			{
				RADIO_RAMP_02_US                        = 0x00,
				RADIO_RAMP_04_US                        = 0x20,
				RADIO_RAMP_06_US                        = 0x40,
				RADIO_RAMP_08_US                        = 0x60,
				RADIO_RAMP_10_US                        = 0x80,
				RADIO_RAMP_12_US                        = 0xA0,
				RADIO_RAMP_16_US                        = 0xC0,
				RADIO_RAMP_20_US                        = 0xE0,
			};

			enum  RadioLoRaCadSymbols
			{
				LORA_CAD_01_SYMBOL                      = 0x00,
				LORA_CAD_02_SYMBOLS                     = 0x20,
				LORA_CAD_04_SYMBOLS                     = 0x40,
				LORA_CAD_08_SYMBOLS                     = 0x60,
				LORA_CAD_16_SYMBOLS                     = 0x80,
			};

			enum RadioGfskBleBitrates
			{
				GFSK_BLE_BR_2_000_BW_2_4                = 0x04,
				GFSK_BLE_BR_1_600_BW_2_4                = 0x28,
				GFSK_BLE_BR_1_000_BW_2_4                = 0x4C,
				GFSK_BLE_BR_1_000_BW_1_2                = 0x45,
				GFSK_BLE_BR_0_800_BW_2_4                = 0x70,
				GFSK_BLE_BR_0_800_BW_1_2                = 0x69,
				GFSK_BLE_BR_0_500_BW_1_2                = 0x8D,
				GFSK_BLE_BR_0_500_BW_0_6                = 0x86,
				GFSK_BLE_BR_0_400_BW_1_2                = 0xB1,
				GFSK_BLE_BR_0_400_BW_0_6                = 0xAA,
				GFSK_BLE_BR_0_250_BW_0_6                = 0xCE,
				GFSK_BLE_BR_0_250_BW_0_3                = 0xC7,
				GFSK_BLE_BR_0_125_BW_0_3                = 0xEF,
			};

			enum RadioGfskBleModIndexes
			{
				GFSK_BLE_MOD_IND_0_35                   =  0,
				GFSK_BLE_MOD_IND_0_50                   =  1,
				GFSK_BLE_MOD_IND_0_75                   =  2,
				GFSK_BLE_MOD_IND_1_00                   =  3,
				GFSK_BLE_MOD_IND_1_25                   =  4,
				GFSK_BLE_MOD_IND_1_50                   =  5,
				GFSK_BLE_MOD_IND_1_75                   =  6,
				GFSK_BLE_MOD_IND_2_00                   =  7,
				GFSK_BLE_MOD_IND_2_25                   =  8,
				GFSK_BLE_MOD_IND_2_50                   =  9,
				GFSK_BLE_MOD_IND_2_75                   = 10,
				GFSK_BLE_MOD_IND_3_00                   = 11,
				GFSK_BLE_MOD_IND_3_25                   = 12,
				GFSK_BLE_MOD_IND_3_50                   = 13,
				GFSK_BLE_MOD_IND_3_75                   = 14,
				GFSK_BLE_MOD_IND_4_00                   = 15,
			};

			enum RadioFlrcBitrates
			{
				FLRC_BR_1_300_BW_1_2                    = 0x45,
				FLRC_BR_1_040_BW_1_2                    = 0x69,
				FLRC_BR_0_650_BW_0_6                    = 0x86,
				FLRC_BR_0_520_BW_0_6                    = 0xAA,
				FLRC_BR_0_325_BW_0_3                    = 0xC7,
				FLRC_BR_0_260_BW_0_3                    = 0xEB,
			};

			enum RadioFlrcCodingRates
			{
				FLRC_CR_1_2                             = 0x00,
				FLRC_CR_3_4                             = 0x02,
				FLRC_CR_1_0                             = 0x04,
			};

			enum RadioModShapings
			{
				RADIO_MOD_SHAPING_BT_OFF                = 0x00,         //! No filtering
				RADIO_MOD_SHAPING_BT_1_0                = 0x10,
				RADIO_MOD_SHAPING_BT_0_5                = 0x20,
			};

			enum RadioLoRaSpreadingFactors
			{
				LORA_SF5                                = 0x50,
				LORA_SF6                                = 0x60,
				LORA_SF7                                = 0x70,
				LORA_SF8                                = 0x80,
				LORA_SF9                                = 0x90,
				LORA_SF10                               = 0xA0,
				LORA_SF11                               = 0xB0,
				LORA_SF12                               = 0xC0,
			};

			enum RadioLoRaBandwidths
			{
				LORA_BW_0200                            = 0x34,
				LORA_BW_0400                            = 0x26,
				LORA_BW_0800                            = 0x18,
				LORA_BW_1600                            = 0x0A,
			};

			enum RadioLoRaCodingRates
			{
				LORA_CR_4_5                             = 0x01,
				LORA_CR_4_6                             = 0x02,
				LORA_CR_4_7                             = 0x03,
				LORA_CR_4_8                             = 0x04,
				LORA_CR_LI_4_5                          = 0x05,
				LORA_CR_LI_4_6                          = 0x06,
				LORA_CR_LI_4_7                          = 0x07,
			};

			enum RadioPreambleLengths
			{
				PREAMBLE_LENGTH_04_BITS                 = 0x00,         //!< Preamble length: 04 bits
				PREAMBLE_LENGTH_08_BITS                 = 0x10,         //!< Preamble length: 08 bits
				PREAMBLE_LENGTH_12_BITS                 = 0x20,         //!< Preamble length: 12 bits
				PREAMBLE_LENGTH_16_BITS                 = 0x30,         //!< Preamble length: 16 bits
				PREAMBLE_LENGTH_20_BITS                 = 0x40,         //!< Preamble length: 20 bits
				PREAMBLE_LENGTH_24_BITS                 = 0x50,         //!< Preamble length: 24 bits
				PREAMBLE_LENGTH_28_BITS                 = 0x60,         //!< Preamble length: 28 bits
				PREAMBLE_LENGTH_32_BITS                 = 0x70,         //!< Preamble length: 32 bits
			};

			enum RadioFlrcSyncWordLengths
			{
				FLRC_NO_SYNCWORD                        = 0x00,
				FLRC_SYNCWORD_LENGTH_4_BYTE             = 0x04,
			};

			enum RadioSyncWordLengths
			{
				GFSK_SYNCWORD_LENGTH_1_BYTE             = 0x00,         //!< Sync word length: 1 byte
				GFSK_SYNCWORD_LENGTH_2_BYTE             = 0x02,         //!< Sync word length: 2 bytes
				GFSK_SYNCWORD_LENGTH_3_BYTE             = 0x04,         //!< Sync word length: 3 bytes
				GFSK_SYNCWORD_LENGTH_4_BYTE             = 0x06,         //!< Sync word length: 4 bytes
				GFSK_SYNCWORD_LENGTH_5_BYTE             = 0x08,         //!< Sync word length: 5 bytes
			};

			enum RadioSyncWordRxMatchs
			{
				RADIO_RX_MATCH_SYNCWORD_OFF             = 0x00,         //!< No correlator turned on, i.e. do not search for SyncWord
				RADIO_RX_MATCH_SYNCWORD_1               = 0x10,
				RADIO_RX_MATCH_SYNCWORD_2               = 0x20,
				RADIO_RX_MATCH_SYNCWORD_1_2             = 0x30,
				RADIO_RX_MATCH_SYNCWORD_3               = 0x40,
				RADIO_RX_MATCH_SYNCWORD_1_3             = 0x50,
				RADIO_RX_MATCH_SYNCWORD_2_3             = 0x60,
				RADIO_RX_MATCH_SYNCWORD_1_2_3           = 0x70,
			};

			enum RadioPacketLengthModes
			{
				RADIO_PACKET_FIXED_LENGTH               = 0x00,         //!< The packet is known on both sides, no header included in the packet
				RADIO_PACKET_VARIABLE_LENGTH            = 0x20,         //!< The packet is on variable size, header included
			};

			enum RadioCrcTypes
			{
				RADIO_CRC_OFF                           = 0x00,         //!< No CRC in use
				RADIO_CRC_1_BYTES                       = 0x10,
				RADIO_CRC_2_BYTES                       = 0x20,
				RADIO_CRC_3_BYTES                       = 0x30,
			};

			enum RadioWhiteningModes
			{
				RADIO_WHITENING_ON                      = 0x00,
				RADIO_WHITENING_OFF                     = 0x08,
			};

			enum RadioLoRaPacketLengthsModes
			{
				LORA_PACKET_VARIABLE_LENGTH             = 0x00,         //!< The packet is on variable size, header included
				LORA_PACKET_FIXED_LENGTH                = 0x80,         //!< The packet is known on both sides, no header included in the packet
				LORA_PACKET_EXPLICIT                    = LORA_PACKET_VARIABLE_LENGTH,
				LORA_PACKET_IMPLICIT                    = LORA_PACKET_FIXED_LENGTH,
			};

			enum RadioLoRaCrcModes
			{
				LORA_CRC_ON                             = 0x20,         //!< CRC activated
				LORA_CRC_OFF                            = 0x00,         //!< CRC not used
			};

			enum RadioLoRaIQModes
			{
				LORA_IQ_NORMAL                          = 0x40,
				LORA_IQ_INVERTED                        = 0x00,
			};

			enum RadioRangingIdCheckLengths
			{
				RANGING_IDCHECK_LENGTH_08_BITS          = 0x00,
				RANGING_IDCHECK_LENGTH_16_BITS,
				RANGING_IDCHECK_LENGTH_24_BITS,
				RANGING_IDCHECK_LENGTH_32_BITS,
			};

			enum RadioRangingResultTypes
			{
				RANGING_RESULT_RAW                      = 0x00,
				RANGING_RESULT_AVERAGED                 = 0x01,
				RANGING_RESULT_DEBIASED                 = 0x02,
				RANGING_RESULT_FILTERED                 = 0x03,
			};

			enum RadioBleConnectionStates
			{
				BLE_MASTER_SLAVE                        = 0x00,
				BLE_ADVERTISER                          = 0x20,
				BLE_TX_TEST_MODE                        = 0x40,
				BLE_RX_TEST_MODE                        = 0x60,
				BLE_RXTX_TEST_MODE                      = 0x80,
			};

			enum RadioBleCrcTypes
			{
				BLE_CRC_OFF                             = 0x00,
				BLE_CRC_3B                              = 0x10,
			};

			enum RadioBleTestPayloads
			{
				BLE_PRBS_9                              = 0x00,         //!< Pseudo Random Binary Sequence based on 9th degree polynomial
				BLE_PRBS_15                             = 0x0C,         //!< Pseudo Random Binary Sequence based on 15th degree polynomial
				BLE_EYELONG_1_0                         = 0x04,         //!< Repeated '11110000' sequence
				BLE_EYELONG_0_1                         = 0x18,         //!< Repeated '00001111' sequence
				BLE_EYESHORT_1_0                        = 0x08,         //!< Repeated '10101010' sequence
				BLE_EYESHORT_0_1                        = 0x1C,         //!< Repeated '01010101' sequence
				BLE_ALL_1                               = 0x10,         //!< Repeated '11111111' sequence
				BLE_ALL_0                               = 0x14,         //!< Repeated '00000000' sequence
			};

			enum RadioIrqMasks
			{
				IRQ_RADIO_NONE                          = 0x0000,
				IRQ_TX_DONE                             = 0x0001,
				IRQ_RX_DONE                             = 0x0002,
				IRQ_SYNCWORD_VALID                      = 0x0004,
				IRQ_SYNCWORD_ERROR                      = 0x0008,
				IRQ_HEADER_VALID                        = 0x0010,
				IRQ_HEADER_ERROR                        = 0x0020,
				IRQ_CRC_ERROR                           = 0x0040,
				IRQ_RANGING_SLAVE_RESPONSE_DONE         = 0x0080,
				IRQ_RANGING_SLAVE_REQUEST_DISCARDED     = 0x0100,
				IRQ_RANGING_MASTER_RESULT_VALID         = 0x0200,
				IRQ_RANGING_MASTER_TIMEOUT              = 0x0400,
				IRQ_RANGING_SLAVE_REQUEST_VALID         = 0x0800,
				IRQ_CAD_DONE                            = 0x1000,
				IRQ_CAD_DETECTED                        = 0x2000,
				IRQ_RX_TX_TIMEOUT                       = 0x4000,
				IRQ_PREAMBLE_DETECTED                   = 0x8000,
				IRQ_RADIO_ALL                           = 0xFFFF,
			};

			enum RadioDios
			{
				RADIO_DIO1                              = 0x02,
				RADIO_DIO2                              = 0x04,
				RADIO_DIO3                              = 0x08,
			};
			
			enum RadioTickSizes
			{
				RADIO_TICK_SIZE_0015_US                 = 0x00,
				RADIO_TICK_SIZE_0062_US                 = 0x01,
				RADIO_TICK_SIZE_1000_US                 = 0x02,
				RADIO_TICK_SIZE_4000_US                 = 0x03,
			};

			enum RadioRangingRoles
			{
				RADIO_RANGING_ROLE_SLAVE                = 0x00,
				RADIO_RANGING_ROLE_MASTER               = 0x01,
			};

			struct RadioStatus
			{
				struct
				{
					uint8_t CpuBusy   : 1;  //!< Flag for CPU radio busy
					uint8_t DmaBusy   : 1;  //!< Flag for DMA busy
					uint8_t CmdStatus : 3;  //!< Command status
					uint8_t ChipMode  : 3;  //!< Chip mode
				} Fields;
				uint8_t Value;
			};

			typedef struct
			{
				uint8_t WakeUpRTC               : 1;                    //!< Get out of sleep mode if wakeup signal received from RTC
				uint8_t InstructionRamRetention : 1;                    //!< InstructionRam is conserved during sleep
				uint8_t DataBufferRetention     : 1;                    //!< Data buffer is conserved during sleep
				uint8_t DataRamRetention        : 1;                    //!< Data ram is conserved during sleep
			} SleepParams;

			typedef struct
			{
				uint8_t RC64KEnable    : 1;                             //!< Calibrate RC64K clock
				uint8_t RC13MEnable    : 1;                             //!< Calibrate RC13M clock
				uint8_t PLLEnable      : 1;                             //!< Calibrate PLL
				uint8_t ADCPulseEnable : 1;                             //!< Calibrate ADC Pulse
				uint8_t ADCBulkNEnable : 1;                             //!< Calibrate ADC bulkN
				uint8_t ADCBulkPEnable : 1;                             //!< Calibrate ADC bulkP
			} CalibrationParams;

			typedef struct
			{
				RadioPacketTypes                    packetType;       //!< Packet to which the packet status are referring to.
				union
				{
					struct
					{
						uint16_t PacketReceived;                        //!< Number of received packets
						uint16_t CrcError;                              //!< Number of CRC errors
						uint16_t LengthError;                           //!< Number of length errors
						uint16_t SyncwordError;                         //!< Number of sync-word errors
					}Gfsk;
					struct
					{
						uint16_t PacketReceived;                        //!< Number of received packets
						uint16_t CrcError;                              //!< Number of CRC errors
						uint16_t HeaderValid;                           //!< Number of valid headers
					}LoRa;
				};
			} RxCounter;


			typedef struct
			{
				RadioPacketTypes                    packetType;        //!< Packet to which the packet status are referring to.
				union
				{
					struct
					{
						int8_t RssiSync;                                //!< The RSSI measured on last packet
						struct
						{
							bool SyncError :1;                          //!< SyncWord error on last packet
							bool LengthError :1;                        //!< Length error on last packet
							bool CrcError :1;                           //!< CRC error on last packet
							bool AbortError :1;                         //!< Abort error on last packet
							bool HeaderReceived :1;                     //!< Header received on last packet
							bool PacketReceived :1;                     //!< Packet received
							bool PacketControlerBusy :1;                //!< Packet controller busy
						}ErrorStatus;                                   //!< The error status Byte
						struct
						{
							bool RxNoAck :1;                            //!< No acknowledgment received for Rx with variable length packets
							bool PacketSent :1;                         //!< Packet sent, only relevant in Tx mode
						}TxRxStatus;                                    //!< The Tx/Rx status Byte
						uint8_t SyncAddrStatus :3;                      //!< The id of the correlator who found the packet
					}Gfsk;
					struct
					{
						int8_t RssiPkt;                                 //!< The RSSI of the last packet
						int8_t SnrPkt;                                  //!< The SNR of the last packet
					}LoRa;
					struct
					{
						int8_t RssiSync;                                //!< The RSSI of the last packet
						struct
						{
							bool SyncError :1;                          //!< SyncWord error on last packet
							bool LengthError :1;                        //!< Length error on last packet
							bool CrcError :1;                           //!< CRC error on last packet
							bool AbortError :1;                         //!< Abort error on last packet
							bool HeaderReceived :1;                     //!< Header received on last packet
							bool PacketReceived :1;                     //!< Packet received
							bool PacketControlerBusy :1;                //!< Packet controller busy
						}ErrorStatus;                                   //!< The error status Byte
						struct
						{
							uint8_t RxPid :2;                           //!< PID of the Rx
							bool RxNoAck :1;                            //!< No acknowledgment received for Rx with variable length packets
							bool RxPidErr :1;                           //!< Received PID error
							bool PacketSent :1;                         //!< Packet sent, only relevant in Tx mode
						}TxRxStatus;                                    //!< The Tx/Rx status Byte
						uint8_t SyncAddrStatus :3;                      //!< The id of the correlator who found the packet
					}Flrc;
					struct
					{
						int8_t RssiSync;                                //!< The RSSI of the last packet
						struct
						{
							bool SyncError :1;                          //!< SyncWord error on last packet
							bool LengthError :1;                        //!< Length error on last packet
							bool CrcError :1;                           //!< CRC error on last packet
							bool AbortError :1;                         //!< Abort error on last packet
							bool HeaderReceived :1;                     //!< Header received on last packet
							bool PacketReceived :1;                     //!< Packet received
							bool PacketControlerBusy :1;                //!< Packet controller busy
						}ErrorStatus;                                   //!< The error status Byte
						struct
						{
							bool PacketSent :1;                         //!< Packet sent, only relevant in Tx mode
						}TxRxStatus;                                    //!< The Tx/Rx status Byte
						uint8_t SyncAddrStatus :3;                      //!< The id of the correlator who found the packet
					}Ble;
				};
			} PacketStatus;

			typedef struct
			{
				RadioPacketTypes                    PacketType;       //!< Packet to which the packet parameters are referring to.
				struct
				{
					/*!
					 * \brief Holds the GFSK packet parameters
					 */
					struct
					{
						RadioPreambleLengths       PreambleLength;    //!< The preamble length for GFSK packet type
						RadioSyncWordLengths       SyncWordLength;    //!< The synchronization word length for GFSK packet type
						RadioSyncWordRxMatchs      SyncWordMatch;     //!< The synchronization correlator to use to check synchronization word
						RadioPacketLengthModes     HeaderType;        //!< If the header is explicit, it will be transmitted in the GFSK packet. If the header is implicit, it will not be transmitted
						uint8_t                      PayloadLength;     //!< Size of the payload in the GFSK packet
						RadioCrcTypes              CrcLength;         //!< Size of the CRC block in the GFSK packet
						RadioWhiteningModes        Whitening;         //!< Usage of whitening on payload and CRC blocks plus header block if header type is variable
					}Gfsk;
					/*!
					 * \brief Holds the LORA packet parameters
					 */
					struct
					{
						uint8_t                       PreambleLength;   //!< The preamble length is the number of LORA symbols in the preamble. To set it, use the following formula @code Number of symbols = PreambleLength[3:0] * ( 2^PreambleLength[7:4] ) @endcode
						RadioLoRaPacketLengthsModes HeaderType;       //!< If the header is explicit, it will be transmitted in the LORA packet. If the header is implicit, it will not be transmitted
						uint8_t                       PayloadLength;    //!< Size of the payload in the LORA packet
						RadioLoRaCrcModes           Crc;              //!< Size of CRC block in LORA packet
						RadioLoRaIQModes            InvertIQ;         //!< Allows to swap IQ for LORA packet
					}LoRa;
					/*!
					 * \brief Holds the FLRC packet parameters
					 */
					struct
					{
						RadioPreambleLengths       PreambleLength;    //!< The preamble length for FLRC packet type
						RadioFlrcSyncWordLengths   SyncWordLength;    //!< The synchronization word length for FLRC packet type
						RadioSyncWordRxMatchs      SyncWordMatch;     //!< The synchronization correlator to use to check synchronization word
						RadioPacketLengthModes     HeaderType;        //!< If the header is explicit, it will be transmitted in the FLRC packet. If the header is implicit, it will not be transmitted.
						uint8_t                      PayloadLength;     //!< Size of the payload in the FLRC packet
						RadioCrcTypes              CrcLength;         //!< Size of the CRC block in the FLRC packet
						RadioWhiteningModes        Whitening;         //!< Usage of whitening on payload and CRC blocks plus header block if header type is variable
					}Flrc;
					/*!
					 * \brief Holds the BLE packet parameters
					 */
					struct
					{
						RadioBleConnectionStates    ConnectionState;   //!< The BLE state
						RadioBleCrcTypes            CrcLength;         //!< Size of the CRC block in the BLE packet
						RadioBleTestPayloads        BleTestPayload;    //!< Special BLE payload for test purpose
						RadioWhiteningModes         Whitening;         //!< Usage of whitening on PDU and CRC blocks of BLE packet
					}Ble;
				}Params;                                                 //!< Holds the packet parameters structure
			} PacketParams;

			typedef struct
			{
				RadioPacketTypes                    PacketType;       //!< Packet to which the modulation parameters are referring to.
				struct
				{
					/*!
					 * \brief Holds the GFSK modulation parameters
					 *
					 * In GFSK modulation, the bit-rate and bandwidth are linked together. In this structure, its values are set using the same token.
					 */
					struct
					{
						RadioGfskBleBitrates    BitrateBandwidth;     //!< The bandwidth and bit-rate values for BLE and GFSK modulations
						RadioGfskBleModIndexes  ModulationIndex;      //!< The coding rate for BLE and GFSK modulations
						RadioModShapings        ModulationShaping;    //!< The modulation shaping for BLE and GFSK modulations
					}Gfsk;
					/*!
					 * \brief Holds the LORA modulation parameters
					 *
					 * LORA modulation is defined by Spreading Factor (SF), Bandwidth and Coding Rate
					 */
					struct
					{
						RadioLoRaSpreadingFactors  SpreadingFactor;   //!< Spreading Factor for the LORA modulation
						RadioLoRaBandwidths        Bandwidth;         //!< Bandwidth for the LORA modulation
						RadioLoRaCodingRates       CodingRate;        //!< Coding rate for the LORA modulation
					}LoRa;
					/*!
					 * \brief Holds the FLRC modulation parameters
					 *
					 * In FLRC modulation, the bit-rate and bandwidth are linked together. In this structure, its values are set using the same token.
					 */
					struct
					{
						RadioFlrcBitrates          BitrateBandwidth;  //!< The bandwidth and bit-rate values for FLRC modulation
						RadioFlrcCodingRates       CodingRate;        //!< The coding rate for FLRC modulation
						RadioModShapings           ModulationShaping; //!< The modulation shaping for FLRC modulation
					}Flrc;
					/*!
					 * \brief Holds the BLE modulation parameters
					 *
					 * In BLE modulation, the bit-rate and bandwidth are linked together. In this structure, its values are set using the same token.
					 */
					struct
					{
						RadioGfskBleBitrates       BitrateBandwidth;  //!< The bandwidth and bit-rate values for BLE and GFSK modulations
						RadioGfskBleModIndexes     ModulationIndex;   //!< The coding rate for BLE and GFSK modulations
						RadioModShapings           ModulationShaping; //!< The modulation shaping for BLE and GFSK modulations
					}Ble;
				}Params;                                                //!< Holds the modulation parameters structure
			} ModulationParams;
			
			typedef struct TickTime_s
			{
				RadioTickSizes PeriodBase;                            //!< The base time of ticktime
				/*!
				 * \brief The number of periodBase for ticktime
				 * Special values are:
				 *     - 0x0000 for single mode
				 *     - 0xFFFF for continuous mode
				 */
				uint16_t PeriodBaseCount;
			} TickTime;

			// Customization
			//#define GFSK_SUPPORT
			//#define FLRC_SUPPORT
			//#define BLE_SUPPORT
			#define RANGING_SUPPORT
			#define LORA_SUPPORT
			
			const uint32_t BUSY_PIN = 0x0118; // 1_24
			const uint32_t DIO1_PIN = 0x0006; // 0_6
			const uint32_t RESET_PIN = 0x011C; // 1_28
			
			const uint32_t SPI_MOSI = 0x010D; // 1_13
			const uint32_t SPI_MISO = 0x010E; // 1_14
			const uint32_t SPI_SCLK = 0x0007; // 0_7
			const uint32_t SPI_CSEL = 0x0011; // 0_17
			
			const uint32_t LORA_BUFFER_SIZE = 24;
			uint8_t txBuffer[24] = { 0 };
			uint8_t rxBuffer[24] = { 0 };
			
			const uint32_t RF_FREQUENCY = 2425000000UL;
			const uint32_t TX_OUTPUT_POWER = 13;

			const uint16_t TX_TIMEOUT_VALUE = 1000; // ms
			const uint16_t RX_TIMEOUT_VALUE = 0xffff; // ms
			const RadioTickSizes RX_TIMEOUT_TICK_SIZE = RADIO_TICK_SIZE_1000_US;

			const uint16_t IrqMask = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;

			// Standard values

			const uint32_t AUTO_TX_OFFSET = 0x21;
			const uint32_t MASK_RANGINGMUXSEL = 0xCF;
			const uint32_t DEFAULT_RANGING_FILTER_SIZE = 0x7F;
			const uint32_t MASK_FORCE_PREAMBLELENGTH = 0x8F;
			const uint32_t BLE_ADVERTIZER_ACCESS_ADDRESS = 0x8E89BED6;
			
			SDD1306 &sdd1306;
			EEPROM &settings;
			FT25H16S &ft25h16s;

			SX1280(SDD1306 &_sdd1306, EEPROM &_settings, FT25H16S &_ft25h16s):
				sdd1306(_sdd1306),
				settings(_settings),
				ft25h16s(_ft25h16s) { 
			}
			
			void Init(bool pollMode) {
				// Configure control pins
				Chip_IOCON_PinMuxSet(LPC_IOCON, (RESET_PIN>>8), (RESET_PIN&0xFF), IOCON_FUNC0 | IOCON_MODE_PULLUP);
				Chip_GPIO_SetPinDIROutput(LPC_GPIO, (RESET_PIN>>8), (RESET_PIN&0xFF));

				Chip_IOCON_PinMuxSet(LPC_IOCON, (BUSY_PIN>>8), (BUSY_PIN&0xFF), IOCON_FUNC0 | IOCON_MODE_INACT);
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, (BUSY_PIN>>8), (BUSY_PIN&0xFF));
				
				Chip_IOCON_PinMuxSet(LPC_IOCON, (DIO1_PIN>>8), (DIO1_PIN&0xFF), IOCON_FUNC0 | IOCON_MODE_INACT);
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, (DIO1_PIN>>8), (DIO1_PIN&0xFF));

				Chip_IOCON_PinMuxSet(LPC_IOCON, (SPI_MISO>>8), (SPI_MISO&0xFF), IOCON_FUNC0 | IOCON_MODE_INACT);
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, (SPI_MISO>>8), (SPI_MISO&0xFF));

				Chip_IOCON_PinMuxSet(LPC_IOCON, (SPI_MOSI>>8), (SPI_MOSI&0xFF), IOCON_FUNC0 | IOCON_MODE_INACT);
				Chip_GPIO_SetPinDIROutput(LPC_GPIO, (SPI_MOSI>>8), (SPI_MOSI&0xFF));

				Chip_IOCON_PinMuxSet(LPC_IOCON, (SPI_CSEL>>8), (SPI_CSEL&0xFF), IOCON_FUNC0 | IOCON_MODE_INACT);
				Chip_GPIO_SetPinDIROutput(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));

				Chip_IOCON_PinMuxSet(LPC_IOCON, (SPI_SCLK>>8), (SPI_SCLK&0xFF), IOCON_FUNC0 | IOCON_MODE_INACT);
				Chip_GPIO_SetPinDIROutput(LPC_GPIO, (SPI_SCLK>>8), (SPI_SCLK&0xFF));

				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_SCLK>>8), (SPI_SCLK&0xFF));

				Reset();

				if (!pollMode) {
					Chip_IOCON_PinMuxSet(LPC_IOCON, uint8_t(DIO1_PIN>>8), uint8_t(DIO1_PIN&0xFF), IOCON_FUNC0 | IOCON_MODE_PULLUP);
					Chip_GPIO_SetPinDIRInput(LPC_GPIO, uint8_t(DIO1_PIN>>8), uint8_t(DIO1_PIN&0xFF));

					Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH0);
					Chip_PININT_EnableIntLow(LPC_PININT, Chip_PININT_GetHighEnabled(LPC_PININT) | PININTCH0);
					Chip_PININT_EnableIntHigh(LPC_PININT, Chip_PININT_GetLowEnabled(LPC_PININT) | PININTCH0);
					Chip_SYSCTL_SetPinInterrupt(0, uint8_t(DIO1_PIN>>8), uint8_t(DIO1_PIN&0xFF));  
					
					NVIC_ClearPendingIRQ(PIN_INT0_IRQn);  
					Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH0);
					NVIC_EnableIRQ(PIN_INT0_IRQn);  
				}

				Wakeup();

				SetStandby( STDBY_RC );

				ModulationParams modulationParams;
				modulationParams.PacketType                  = PACKET_TYPE_LORA;
				modulationParams.Params.LoRa.SpreadingFactor = LORA_SF11;
				modulationParams.Params.LoRa.Bandwidth       = LORA_BW_0200;
				modulationParams.Params.LoRa.CodingRate      = LORA_CR_LI_4_7;
				SetPacketType( modulationParams.PacketType );
				SetModulationParams( &modulationParams );

				PacketParams PacketParams;
				PacketParams.PacketType                 	 = PACKET_TYPE_LORA;
				PacketParams.Params.LoRa.PreambleLength      = 0x0C;
				PacketParams.Params.LoRa.HeaderType          = LORA_PACKET_VARIABLE_LENGTH;
				PacketParams.Params.LoRa.PayloadLength       = LORA_BUFFER_SIZE;
				PacketParams.Params.LoRa.Crc                 = LORA_CRC_ON;
				PacketParams.Params.LoRa.InvertIQ            = LORA_IQ_NORMAL;
				SetPacketParams( &PacketParams );
				
				SetRfFrequency( RF_FREQUENCY );
				SetBufferBaseAddresses( 0x00, 0x00 );
				SetTxParams( TX_OUTPUT_POWER, RADIO_RAMP_20_US );
				SetDioIrqParams( SX1280::IrqMask, SX1280::IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
				
		    	SetRx( TickTime { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
			}
			
			void SendBuffer() {
				SendPayload( txBuffer, LORA_BUFFER_SIZE, TickTime { RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE } ); 
			}
			
			void Reset() {
				disableIRQ();
				
				delay( 50 );
				Chip_GPIO_SetPinDIROutput(LPC_GPIO, (RESET_PIN>>8), (RESET_PIN&0xFF));
				Chip_GPIO_SetPinOutLow(LPC_GPIO, (RESET_PIN>>8), (RESET_PIN&0xFF));
				delay( 50 );
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (RESET_PIN>>8), (RESET_PIN&0xFF));
				delay( 50 );
				enableIRQ();

				WaitOnBusy();
				
				delay( 10 );
			}
			
			void Wakeup() {
				disableIRQ();

				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				spiwrite(RADIO_GET_STATUS);
				spiwrite(0);
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));

				WaitOnBusy( );

				enableIRQ( );
			}
			
			bool DevicePresent()  {
				return GetFirmwareVersion() == 0x0000a9b5;
			}

			uint16_t GetFirmwareVersion( void )
			{
				return( ( ( ReadRegister( REG_LR_FIRMWARE_VERSION_MSB ) ) << 8 ) | ( ReadRegister( REG_LR_FIRMWARE_VERSION_MSB + 1 ) ) );
			}

			RadioStatus GetStatus( void )
			{
				uint8_t stat = 0;
				RadioStatus status;

				ReadCommand( RADIO_GET_STATUS, ( uint8_t * )&stat, 1 );
				status.Value = stat;
				return( status );
			}

			void WaitOnBusy() {
				while ( Chip_GPIO_GetPinState(LPC_GPIO, uint8_t(BUSY_PIN>>8), uint8_t(BUSY_PIN&0xFF)) == 1) { };
			}
			
			void WriteCommand(RadioCommand command, uint8_t *buffer, uint32_t size) {
				WaitOnBusy();

				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				spiwrite( ( uint8_t )command );
				for( uint16_t i = 0; i < size; i++ ) {
					spiwrite( buffer[i] );
				}
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
			
				if( command != RADIO_SET_SLEEP ) {
					WaitOnBusy( );
				}
			}

			void ReadCommand(RadioCommand command, uint8_t *buffer, uint32_t size ) {
				WaitOnBusy();

				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				if( command == RADIO_GET_STATUS ) {
					buffer[0] = spiwrite( ( uint8_t )command );
					spiwrite( 0 );
					spiwrite( 0 );
				} else {
					spiwrite( ( uint8_t )command );
					spiwrite( 0 );
					for( uint16_t i = 0; i < size; i++ )
					{
						 buffer[i] = spiwrite( 0 );
					}
				}
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));

				WaitOnBusy( );
			}
			
			void WriteRegister( uint32_t address, uint8_t *buffer, uint32_t size ) {
				WaitOnBusy( );

				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				spiwrite( RADIO_WRITE_REGISTER );
				spiwrite( ( address & 0xFF00 ) >> 8 );
				spiwrite( address & 0x00FF );
				for( uint16_t i = 0; i < size; i++ ) {
					spiwrite( buffer[i] );
				}
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));

				WaitOnBusy( );
			}

			void WriteRegister( uint32_t address, uint8_t value ) {
				WriteRegister( address, &value, 1 );
			}
			
			void ReadRegister( uint32_t address, uint8_t *buffer, uint32_t size ) {
				WaitOnBusy( );

				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				spiwrite( RADIO_READ_REGISTER );
				spiwrite( ( address & 0xFF00 ) >> 8 );
				spiwrite( address & 0x00FF );
				spiwrite( 0 );
				for( uint16_t i = 0; i < size; i++ ) {
					buffer[i] = spiwrite( 0 );
				}
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));

				WaitOnBusy( );
			}

			uint8_t ReadRegister( uint32_t address ) {
				uint8_t data;
				ReadRegister( address, &data, 1 );
				return data;
			}

			void WriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size ) {
			    WaitOnBusy( );

				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				spiwrite( RADIO_WRITE_BUFFER );
				spiwrite( offset );
				for( uint16_t i = 0; i < size; i++ ) {
					spiwrite( buffer[i] );
				}
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));

				WaitOnBusy( );
			}

			void ReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size ) {
			    WaitOnBusy( );

				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));
				spiwrite( RADIO_READ_BUFFER );
				spiwrite( offset );
				spiwrite( 0 );
				for( uint16_t i = 0; i < size; i++ ) {
					buffer[i] = spiwrite( 0 );
				}
				Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_CSEL>>8), (SPI_CSEL&0xFF));

				WaitOnBusy( );
			}


			void SetSleep( SleepParams sleepConfig ) {
				uint8_t sleep = ( sleepConfig.WakeUpRTC << 3 ) |
								( sleepConfig.InstructionRamRetention << 2 ) |
								( sleepConfig.DataBufferRetention << 1 ) |
								( sleepConfig.DataRamRetention );

				OperatingMode = MODE_SLEEP;
				WriteCommand( RADIO_SET_SLEEP, &sleep, 1 );
			}

			void SetStandby( RadioStandbyModes standbyConfig ) {
				WriteCommand( RADIO_SET_STANDBY, ( uint8_t* )&standbyConfig, 1 );
				if( standbyConfig == STDBY_RC )
				{
					OperatingMode = MODE_STDBY_RC;
				}
				else
				{
					OperatingMode = MODE_STDBY_XOSC;
				}
			}

			void SetFs( void ) {
				WriteCommand( RADIO_SET_FS, 0, 0 );
				OperatingMode = MODE_FS;
			}

			void SetTx( TickTime timeout ) {
				uint8_t buf[3];
				buf[0] = timeout.PeriodBase;
				buf[1] = ( uint8_t )( ( timeout.PeriodBaseCount >> 8 ) & 0x00FF );
				buf[2] = ( uint8_t )( timeout.PeriodBaseCount & 0x00FF );

				ClearIrqStatus( IRQ_RADIO_ALL );

				// If the radio is doing ranging operations, then apply the specific calls
				// prior to SetTx
				#ifdef RANGING_SUPPORT
				if( GetPacketType( true ) == PACKET_TYPE_RANGING )
				{
					SetRangingRole( RADIO_RANGING_ROLE_MASTER );
				}
				#endif  // #ifdef RANGING_SUPPORT
				WriteCommand( RADIO_SET_TX, buf, 3 );
				OperatingMode = MODE_TX;
			}

			void SetRx( TickTime timeout ) {
				uint8_t buf[3];
				buf[0] = timeout.PeriodBase;
				buf[1] = ( uint8_t )( ( timeout.PeriodBaseCount >> 8 ) & 0x00FF );
				buf[2] = ( uint8_t )( timeout.PeriodBaseCount & 0x00FF );

				ClearIrqStatus( IRQ_RADIO_ALL );

				// If the radio is doing ranging operations, then apply the specific calls
				// prior to SetRx
				#ifdef RANGING_SUPPORT
				if( GetPacketType( true ) == PACKET_TYPE_RANGING )
				{
					SetRangingRole( RADIO_RANGING_ROLE_SLAVE );
				}
				#endif  // #ifdef RANGING_SUPPORT
				WriteCommand( RADIO_SET_RX, buf, 3 );
				OperatingMode = MODE_RX;
			}

			void SetRxDutyCycle( RadioTickSizes periodBase, uint16_t periodBaseCountRx, uint16_t periodBaseCountSleep ) {
				uint8_t buf[5];

				buf[0] = periodBase;
				buf[1] = ( uint8_t )( ( periodBaseCountRx >> 8 ) & 0x00FF );
				buf[2] = ( uint8_t )( periodBaseCountRx & 0x00FF );
				buf[3] = ( uint8_t )( ( periodBaseCountSleep >> 8 ) & 0x00FF );
				buf[4] = ( uint8_t )( periodBaseCountSleep & 0x00FF );
				WriteCommand( RADIO_SET_RXDUTYCYCLE, buf, 5 );
				OperatingMode = MODE_RX;
			}

			void SetCad( void ) {
				WriteCommand( RADIO_SET_CAD, 0, 0 );
				OperatingMode = MODE_CAD;
			}

			void SetTxContinuousWave( void ) {
				WriteCommand( RADIO_SET_TXCONTINUOUSWAVE, 0, 0 );
			}

			void SetTxContinuousPreamble( void ) {
				WriteCommand( RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0 );
			}

			void SetPacketType( RadioPacketTypes packetType ) {
				// Save packet type internally to avoid questioning the radio
				PacketType = packetType;

				WriteCommand( RADIO_SET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
			}

			RadioPacketTypes GetPacketType( bool returnLocalCopy ) {
				RadioPacketTypes packetType = PACKET_TYPE_NONE;
				if( returnLocalCopy == false )
				{
					ReadCommand( RADIO_GET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
					if( PacketType != packetType )
					{
						PacketType = packetType;
					}
				}
				else
				{
					packetType = PacketType;
				}
				return packetType;
			}
			
			void SetRfFrequency( uint32_t rfFrequency ) {
				uint8_t buf[3];
				uint32_t freq = 0;

				const uint64_t XTAL_FREQ = 52000000;
				freq = uint32_t( (uint64_t(rfFrequency) * uint64_t(262144)) / uint64_t(XTAL_FREQ) );

				buf[0] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
				buf[1] = ( uint8_t )( ( freq >> 8  ) & 0xFF );
				buf[2] = ( uint8_t )( ( freq       ) & 0xFF );
				WriteCommand( RADIO_SET_RFFREQUENCY, buf, 3 );
			}

			void SetTxParams( int8_t power, RadioRampTimes rampTime ) {
				uint8_t buf[2];

				// The power value to send on SPI/UART is in the range [0..31] and the
				// physical output power is in the range [-18..13]dBm
				buf[0] = power + 18;
				buf[1] = ( uint8_t )rampTime;
				WriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
			}

			void SetCadParams( RadioLoRaCadSymbols cadSymbolNum ) {
				WriteCommand( RADIO_SET_CADPARAMS, ( uint8_t* )&cadSymbolNum, 1 );
				OperatingMode = MODE_CAD;
			}

			void SetBufferBaseAddresses( uint8_t txBaseAddress, uint8_t rxBaseAddress ) {
				uint8_t buf[2];

				buf[0] = txBaseAddress;
				buf[1] = rxBaseAddress;
				WriteCommand( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );
			}

			void SetModulationParams( ModulationParams *modParams ) {
				uint8_t buf[3];

				// Check if required configuration corresponds to the stored packet type
				// If not, silently update radio packet type
				if( PacketType != modParams->PacketType )
				{
					SetPacketType( modParams->PacketType );
				}

				switch( modParams->PacketType )
				{
				#ifdef GFSK_SUPPORT
					case PACKET_TYPE_GFSK:
						buf[0] = modParams->Params.Gfsk.BitrateBandwidth;
						buf[1] = modParams->Params.Gfsk.ModulationIndex;
						buf[2] = modParams->Params.Gfsk.ModulationShaping;
						break;
				#endif  // #ifdef GFSK_SUPPORT
				#if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)
					case PACKET_TYPE_LORA:
					case PACKET_TYPE_RANGING:
						buf[0] = modParams->Params.LoRa.SpreadingFactor;
						buf[1] = modParams->Params.LoRa.Bandwidth;
						buf[2] = modParams->Params.LoRa.CodingRate;
						LoRaBandwidth = modParams->Params.LoRa.Bandwidth;
						break;
				#endif  // #if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)
				#ifdef FLRC_SUPPORT
					case PACKET_TYPE_FLRC:
						buf[0] = modParams->Params.Flrc.BitrateBandwidth;
						buf[1] = modParams->Params.Flrc.CodingRate;
						buf[2] = modParams->Params.Flrc.ModulationShaping;
						break;
				#endif  // #ifdef FLRC_SUPPORT
				#ifdef BLE_SUPPORT
					case PACKET_TYPE_BLE:
						buf[0] = modParams->Params.Ble.BitrateBandwidth;
						buf[1] = modParams->Params.Ble.ModulationIndex;
						buf[2] = modParams->Params.Ble.ModulationShaping;
						break;
				#endif  // #ifdef BLE_SUPPORT
					default:
					case PACKET_TYPE_NONE:
						buf[0] = 0;
						buf[1] = 0;
						buf[2] = 0;
						break;
				}
				WriteCommand( RADIO_SET_MODULATIONPARAMS, buf, 3 );
			}

			void SetPacketParams( PacketParams *packetParams ) {
				uint8_t buf[7];
				// Check if required configuration corresponds to the stored packet type
				// If not, silently update radio packet type
				if( PacketType != packetParams->PacketType )
				{
					SetPacketType( packetParams->PacketType );
				}

				switch( packetParams->PacketType )
				{
				#ifdef GFSK_SUPPORT
					case PACKET_TYPE_GFSK:
						buf[0] = packetParams->Params.Gfsk.PreambleLength;
						buf[1] = packetParams->Params.Gfsk.SyncWordLength;
						buf[2] = packetParams->Params.Gfsk.SyncWordMatch;
						buf[3] = packetParams->Params.Gfsk.HeaderType;
						buf[4] = packetParams->Params.Gfsk.PayloadLength;
						buf[5] = packetParams->Params.Gfsk.CrcLength;
						buf[6] = packetParams->Params.Gfsk.Whitening;
						break;
				#endif  // #ifdef GFSK_SUPPORT
				#if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)
					case PACKET_TYPE_LORA:
					case PACKET_TYPE_RANGING:
						buf[0] = packetParams->Params.LoRa.PreambleLength;
						buf[1] = packetParams->Params.LoRa.HeaderType;
						buf[2] = packetParams->Params.LoRa.PayloadLength;
						buf[3] = packetParams->Params.LoRa.Crc;
						buf[4] = packetParams->Params.LoRa.InvertIQ;
						buf[5] = 0;
						buf[6] = 0;
						break;
				#endif  // #if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)
				#ifdef FLRC_SUPPORT
					case PACKET_TYPE_FLRC:
						buf[0] = packetParams->Params.Flrc.PreambleLength;
						buf[1] = packetParams->Params.Flrc.SyncWordLength;
						buf[2] = packetParams->Params.Flrc.SyncWordMatch;
						buf[3] = packetParams->Params.Flrc.HeaderType;
						buf[4] = packetParams->Params.Flrc.PayloadLength;
						buf[5] = packetParams->Params.Flrc.CrcLength;
						buf[6] = packetParams->Params.Flrc.Whitening;
						break;
				#endif  // #ifdef FLRC_SUPPORT
				#ifdef BLE_SUPPORT
					case PACKET_TYPE_BLE:
						buf[0] = packetParams->Params.Ble.ConnectionState;
						buf[1] = packetParams->Params.Ble.CrcLength;
						buf[2] = packetParams->Params.Ble.BleTestPayload;
						buf[3] = packetParams->Params.Ble.Whitening;
						buf[4] = 0;
						buf[5] = 0;
						buf[6] = 0;
						break;
				#endif  // #ifdef BLE_SUPPORT
					default:
					case PACKET_TYPE_NONE:
						buf[0] = 0;
						buf[1] = 0;
						buf[2] = 0;
						buf[3] = 0;
						buf[4] = 0;
						buf[5] = 0;
						buf[6] = 0;
						break;
				}
				WriteCommand( RADIO_SET_PACKETPARAMS, buf, 7 );
			}

			void ForcePreambleLength( RadioPreambleLengths preambleLength ) {
				WriteRegister( REG_LR_PREAMBLELENGTH, ( ReadRegister( REG_LR_PREAMBLELENGTH ) & MASK_FORCE_PREAMBLELENGTH ) | preambleLength );
			}

			void GetRxBufferStatus( uint8_t *rxPayloadLength, uint8_t *rxStartBufferPointer ) {
				uint8_t status[2];

				ReadCommand( RADIO_GET_RXBUFFERSTATUS, status, 2 );

				// In case of LORA fixed header, the rxPayloadLength is obtained by reading
				// the register REG_LR_PAYLOADLENGTH
				if( ( this -> GetPacketType( true ) == PACKET_TYPE_LORA ) && ( ReadRegister( REG_LR_PACKETPARAMS ) >> 7 == 1 ) )
				{
					*rxPayloadLength = ReadRegister( REG_LR_PAYLOADLENGTH );
				}
				else if( this -> GetPacketType( true ) == PACKET_TYPE_BLE )
				{
					// In the case of BLE, the size returned in status[0] do not include the 2-byte length PDU header
					// so it is added there
					*rxPayloadLength = status[0] + 2;
				}
				else
				{
					*rxPayloadLength = status[0];
				}

				*rxStartBufferPointer = status[1];
			}

			void GetPacketStatus( PacketStatus *packetStatus ) {
				uint8_t status[5];

				ReadCommand( RADIO_GET_PACKETSTATUS, status, 5 );

				packetStatus->packetType = this -> GetPacketType( true );
				switch( packetStatus->packetType )
				{
				#ifdef GFSK_SUPPORT
					case PACKET_TYPE_GFSK:
						packetStatus->Gfsk.RssiSync = -( status[1] / 2 );

						packetStatus->Gfsk.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
						packetStatus->Gfsk.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
						packetStatus->Gfsk.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
						packetStatus->Gfsk.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
						packetStatus->Gfsk.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
						packetStatus->Gfsk.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
						packetStatus->Gfsk.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

						packetStatus->Gfsk.TxRxStatus.RxNoAck = ( status[3] >> 5 ) & 0x01;
						packetStatus->Gfsk.TxRxStatus.PacketSent = status[3] & 0x01;

						packetStatus->Gfsk.SyncAddrStatus = status[4] & 0x07;
						break;
				#endif  // #ifdef GFSK_SUPPORT
				#if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)
					case PACKET_TYPE_LORA:
					case PACKET_TYPE_RANGING:
						packetStatus->LoRa.RssiPkt = -( status[0] / 2 );
						( status[1] < 128 ) ? ( packetStatus->LoRa.SnrPkt = status[1] / 4 ) : ( packetStatus->LoRa.SnrPkt = ( ( status[1] - 256 ) /4 ) );
						break;
				#endif  // #if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)
				#ifdef FLRC_SUPPORT
					case PACKET_TYPE_FLRC:
						packetStatus->Flrc.RssiSync = -( status[1] / 2 );

						packetStatus->Flrc.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
						packetStatus->Flrc.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
						packetStatus->Flrc.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
						packetStatus->Flrc.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
						packetStatus->Flrc.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
						packetStatus->Flrc.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
						packetStatus->Flrc.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

						packetStatus->Flrc.TxRxStatus.RxPid = ( status[3] >> 6 ) & 0x03;
						packetStatus->Flrc.TxRxStatus.RxNoAck = ( status[3] >> 5 ) & 0x01;
						packetStatus->Flrc.TxRxStatus.RxPidErr = ( status[3] >> 4 ) & 0x01;
						packetStatus->Flrc.TxRxStatus.PacketSent = status[3] & 0x01;

						packetStatus->Flrc.SyncAddrStatus = status[4] & 0x07;
						break;
				#endif  // #ifdef FLRC_SUPPORT
				#ifdef BLE_SUPPORT
					case PACKET_TYPE_BLE:
						packetStatus->Ble.RssiSync =  -( status[1] / 2 );

						packetStatus->Ble.ErrorStatus.SyncError = ( status[2] >> 6 ) & 0x01;
						packetStatus->Ble.ErrorStatus.LengthError = ( status[2] >> 5 ) & 0x01;
						packetStatus->Ble.ErrorStatus.CrcError = ( status[2] >> 4 ) & 0x01;
						packetStatus->Ble.ErrorStatus.AbortError = ( status[2] >> 3 ) & 0x01;
						packetStatus->Ble.ErrorStatus.HeaderReceived = ( status[2] >> 2 ) & 0x01;
						packetStatus->Ble.ErrorStatus.PacketReceived = ( status[2] >> 1 ) & 0x01;
						packetStatus->Ble.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

						packetStatus->Ble.TxRxStatus.PacketSent = status[3] & 0x01;

						packetStatus->Ble.SyncAddrStatus = status[4] & 0x07;
						break;
				#endif  // #ifdef BLE_SUPPORT
					default:
					case PACKET_TYPE_NONE:
						// In that specific case, we set everything in the packetStatus to zeros
						// and reset the packet type accordingly
						for (uint32_t c=0; c<sizeof(PacketStatus); c++) { ((uint8_t*)packetStatus)[c] = 0; } 
						packetStatus->packetType = PACKET_TYPE_NONE;
						break;
				}
			}

			int8_t GetRssiInst( void ) {
				uint8_t raw = 0;

				ReadCommand( RADIO_GET_RSSIINST, &raw, 1 );

				return ( int8_t ) ( -raw / 2 );
			}

			void SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask ) {
				uint8_t buf[8];

				buf[0] = ( uint8_t )( ( irqMask >> 8 ) & 0x00FF );
				buf[1] = ( uint8_t )( irqMask & 0x00FF );
				buf[2] = ( uint8_t )( ( dio1Mask >> 8 ) & 0x00FF );
				buf[3] = ( uint8_t )( dio1Mask & 0x00FF );
				buf[4] = ( uint8_t )( ( dio2Mask >> 8 ) & 0x00FF );
				buf[5] = ( uint8_t )( dio2Mask & 0x00FF );
				buf[6] = ( uint8_t )( ( dio3Mask >> 8 ) & 0x00FF );
				buf[7] = ( uint8_t )( dio3Mask & 0x00FF );
				WriteCommand( RADIO_SET_DIOIRQPARAMS, buf, 8 );
			}

			uint16_t GetIrqStatus( void ) {
				uint8_t irqStatus[2];
				ReadCommand( RADIO_GET_IRQSTATUS, irqStatus, 2 );
				return ( irqStatus[0] << 8 ) | irqStatus[1];
			}

			void ClearIrqStatus( uint16_t irqMask ) {
				uint8_t buf[2];

				buf[0] = ( uint8_t )( ( ( uint16_t )irqMask >> 8 ) & 0x00FF );
				buf[1] = ( uint8_t )( ( uint16_t )irqMask & 0x00FF );
				WriteCommand( RADIO_CLR_IRQSTATUS, buf, 2 );
			}

			void Calibrate( CalibrationParams calibParam ) {
				uint8_t cal = ( calibParam.ADCBulkPEnable << 5 ) |
							  ( calibParam.ADCBulkNEnable << 4 ) |
							  ( calibParam.ADCPulseEnable << 3 ) |
							  ( calibParam.PLLEnable << 2 ) |
							  ( calibParam.RC13MEnable << 1 ) |
							  ( calibParam.RC64KEnable );
				WriteCommand( RADIO_CALIBRATE, &cal, 1 );
			}

			void SetRegulatorMode( RadioRegulatorModes mode ) {
				WriteCommand( RADIO_SET_REGULATORMODE, ( uint8_t* )&mode, 1 );
			}

			void SetSaveContext( void ) {
				WriteCommand( RADIO_SET_SAVECONTEXT, 0, 0 );
			}

			void SetAutoTx( uint16_t time ) {
				uint16_t compensatedTime = time - ( uint16_t )AUTO_TX_OFFSET;
				uint8_t buf[2];

				buf[0] = ( uint8_t )( ( compensatedTime >> 8 ) & 0x00FF );
				buf[1] = ( uint8_t )( compensatedTime & 0x00FF );
				WriteCommand( RADIO_SET_AUTOTX, buf, 2 );
			}

			void SetAutoFs( bool enableAutoFs ) {
				WriteCommand( RADIO_SET_AUTOFS, ( uint8_t * )&enableAutoFs, 1 );
			}

			void SetLongPreamble( bool enable ) {
				WriteCommand( RADIO_SET_LONGPREAMBLE, ( uint8_t * )&enable, 1 );
			}

			void SetPayload( uint8_t *buffer, uint8_t size, uint8_t offset ) {
				WriteBuffer( offset, buffer, size );
			}

			uint8_t GetPayload( uint8_t *buffer, uint8_t *size , uint8_t maxSize ) {
				uint8_t offset;

				GetRxBufferStatus( size, &offset );
				if( *size > maxSize )
				{
					return 1;
				}
				ReadBuffer( offset, buffer, *size );
				return 0;
			}

			void SendPayload( uint8_t *payload, uint8_t size, TickTime timeout, uint8_t offset = 0 ) {
				SetPayload( payload, size, offset );
				SetTx( timeout );
			}

			uint8_t SetSyncWord( uint8_t syncWordIdx, uint8_t *syncWord ) {
				uint16_t addr;
				uint8_t syncwordSize = 0;

				switch( GetPacketType( true ) )
				{
				#ifdef GFSK_SUPPORT
					case PACKET_TYPE_GFSK:
						syncwordSize = 5;
						switch( syncWordIdx )
						{
							case 1:
								addr = REG_LR_SYNCWORDBASEADDRESS1;
								break;
							case 2:
								addr = REG_LR_SYNCWORDBASEADDRESS2;
								break;
							case 3:
								addr = REG_LR_SYNCWORDBASEADDRESS3;
								break;
							default:
								return 1;
						}
						break;
				#endif  // #ifdef GFSK_SUPPORT
				#ifdef FLRC_SUPPORT
					case PACKET_TYPE_FLRC:
						// For FLRC packet type, the SyncWord is one byte shorter and
						// the base address is shifted by one byte
						syncwordSize = 4;
						switch( syncWordIdx )
						{
							case 1:
								addr = REG_LR_SYNCWORDBASEADDRESS1 + 1;
								break;
							case 2:
								addr = REG_LR_SYNCWORDBASEADDRESS2 + 1;
								break;
							case 3:
								addr = REG_LR_SYNCWORDBASEADDRESS3 + 1;
								break;
							default:
								return 1;
						}
						break;
				#endif  // #ifdef FLRC_SUPPORT
				#ifdef BLE_SUPPORT
					case PACKET_TYPE_BLE:
						// For Ble packet type, only the first SyncWord is used and its
						// address is shifted by one byte
						syncwordSize = 4;
						switch( syncWordIdx )
						{
							case 1:
								addr = REG_LR_SYNCWORDBASEADDRESS1 + 1;
								break;
							default:
								return 1;
						}
						break;
				#endif  // #ifdef BLE_SUPPORT
					default:
						return 1;
				}
				WriteRegister( addr, syncWord, syncwordSize );
				return 0;
			}

			void SetSyncWordErrorTolerance( uint8_t ErrorBits ) {
				ErrorBits = ( ReadRegister( REG_LR_SYNCWORDTOLERANCE ) & 0xF0 ) | ( ErrorBits & 0x0F );
				WriteRegister( REG_LR_SYNCWORDTOLERANCE, ErrorBits );
			}

			uint8_t SetCrcSeed( uint8_t *seed ) {
				uint8_t updated = 0;
				switch( GetPacketType( true ) )
				{
				#if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT)
					case PACKET_TYPE_GFSK:
					case PACKET_TYPE_FLRC:
						WriteRegister( REG_LR_CRCSEEDBASEADDR, seed, 2 );
						updated = 1;
						break;
				#endif  // #if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT)
				#ifdef BLE_SUPPORT
					case PACKET_TYPE_BLE:
						WriteRegister(0x9c7, seed[2] );
						WriteRegister(0x9c8, seed[1] );
						WriteRegister(0x9c9, seed[0] );
						updated = 1;
						break;
				#endif  // #ifdef BLE_SUPPORT
					default:
						break;
				}
				return updated;
			}

			#ifdef BLE_SUPPORT
			void SetBleAccessAddress( uint32_t accessAddress ) {
				WriteRegister( REG_LR_BLE_ACCESS_ADDRESS, ( accessAddress >> 24 ) & 0x000000FF );
				WriteRegister( REG_LR_BLE_ACCESS_ADDRESS + 1, ( accessAddress >> 16 ) & 0x000000FF );
				WriteRegister( REG_LR_BLE_ACCESS_ADDRESS + 2, ( accessAddress >> 8 ) & 0x000000FF );
				WriteRegister( REG_LR_BLE_ACCESS_ADDRESS + 3, accessAddress & 0x000000FF );
			}

			void SetBleAdvertizerAccessAddress( void ) {
				SetBleAccessAddress( BLE_ADVERTIZER_ACCESS_ADDRESS );
			}
			#endif  // #ifdef BLE_SUPPORT

			void SetCrcPolynomial( uint16_t polynomial ) {
				switch( GetPacketType( true ) )
				{
				#if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT)
					case PACKET_TYPE_GFSK:
					case PACKET_TYPE_FLRC:
						uint8_t val[2];
						val[0] = ( uint8_t )( polynomial >> 8 ) & 0xFF;
						val[1] = ( uint8_t )( polynomial  & 0xFF );
						WriteRegister( REG_LR_CRCPOLYBASEADDR, val, 2 );
						break;
				#endif  // #if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT)
					default:
						break;
				}
			}

			void SetWhiteningSeed( uint8_t seed ) {
				switch( GetPacketType( true ) )
				{
				#if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT) || defined(BLE_SUPPORT)
					case PACKET_TYPE_GFSK:
					case PACKET_TYPE_FLRC:
					case PACKET_TYPE_BLE:
						WriteRegister( REG_LR_WHITSEEDBASEADDR, seed );
						break;
				#endif  // #if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT) || defined(BLE_SUPPORT)
					default:
						break;
				}
			}

			#ifdef RANGING_SUPPORT
			void SetRangingIdLength( RadioRangingIdCheckLengths length ) {
				switch( GetPacketType( true ) )
				{
					case PACKET_TYPE_RANGING:
						WriteRegister( REG_LR_RANGINGIDCHECKLENGTH, ( ( ( ( uint8_t )length ) & 0x03 ) << 6 ) | ( ReadRegister( REG_LR_RANGINGIDCHECKLENGTH ) & 0x3F ) );
						break;
					default:
						break;
				}
			}

			void SetDeviceRangingAddress( uint32_t address ) {
				uint8_t addrArray[] = { uint8_t(address >> 24), uint8_t(address >> 16), uint8_t(address >> 8), uint8_t(address) };

				switch( GetPacketType( true ) )
				{
					case PACKET_TYPE_RANGING:
						WriteRegister( REG_LR_DEVICERANGINGADDR, addrArray, 4 );
						break;
					default:
						break;
				}
			}

			void SetRangingRequestAddress( uint32_t address ) {
				uint8_t addrArray[] = { uint8_t(address >> 24), uint8_t(address >> 16), uint8_t(address >> 8), uint8_t(address) };

				switch( GetPacketType( true ) )
				{
					case PACKET_TYPE_RANGING:
						WriteRegister( REG_LR_REQUESTRANGINGADDR, addrArray, 4 );
						break;
					default:
						break;
				}
			}

			double GetRangingResult( RadioRangingResultTypes resultType ) {
				uint32_t valLsb = 0;
				double val = 0.0;

				switch( GetPacketType( true ) )
				{
					case PACKET_TYPE_RANGING:
						SetStandby( STDBY_XOSC );
						WriteRegister( 0x97F, ReadRegister( 0x97F ) | ( 1 << 1 ) ); // enable LORA modem clock
						WriteRegister( REG_LR_RANGINGRESULTCONFIG, ( ReadRegister( REG_LR_RANGINGRESULTCONFIG ) & MASK_RANGINGMUXSEL ) | ( ( ( ( uint8_t )resultType ) & 0x03 ) << 4 ) );
						valLsb = ( ( ReadRegister( REG_LR_RANGINGRESULTBASEADDR ) << 16 ) | ( ReadRegister( REG_LR_RANGINGRESULTBASEADDR + 1 ) << 8 ) | ( ReadRegister( REG_LR_RANGINGRESULTBASEADDR + 2 ) ) );
						SetStandby( STDBY_RC );

						// Convertion from LSB to distance. For explanation on the formula, refer to Datasheet of SX1280
						switch( resultType )
						{
							case RANGING_RESULT_RAW:
								// Convert the ranging LSB to distance in meter
								// The theoretical conversion from register value to distance [m] is given by:
								// distance [m] = ( complement2( register ) * 150 ) / ( 2^12 * bandwidth[MHz] ) )
								// The API provide BW in [Hz] so the implemented formula is complement2( register ) / bandwidth[Hz] * A,
								// where A = 150 / (2^12 / 1e6) = 36621.09
								val = ( double )complement2( valLsb, 24 ) / ( double )GetLoRaBandwidth( ) * 36621.09375;
								break;

							case RANGING_RESULT_AVERAGED:
							case RANGING_RESULT_DEBIASED:
							case RANGING_RESULT_FILTERED:
								val = ( double )valLsb * 20.0 / 100.0;
								break;
							default:
								val = 0.0;
						}
						break;
					default:
						break;
				}
				return val;
			}

			void SetRangingCalibration( uint16_t cal ) {
				switch( GetPacketType( true ) )
				{
					case PACKET_TYPE_RANGING:
						WriteRegister( REG_LR_RANGINGRERXTXDELAYCAL, ( uint8_t )( ( cal >> 8 ) & 0xFF ) );
						WriteRegister( REG_LR_RANGINGRERXTXDELAYCAL + 1, ( uint8_t )( ( cal ) & 0xFF ) );
						break;
					default:
						break;
				}
			}

			void RangingClearFilterResult( void ) {
				uint8_t regVal = ReadRegister( REG_LR_RANGINGRESULTCLEARREG );

				// To clear result, set bit 5 to 1 then to 0
				WriteRegister( REG_LR_RANGINGRESULTCLEARREG, regVal | ( 1 << 5 ) );
				WriteRegister( REG_LR_RANGINGRESULTCLEARREG, regVal & ( ~( 1 << 5 ) ) );
			}

			void RangingSetFilterNumSamples( uint8_t num ) {
				// Silently set 8 as minimum value
				WriteRegister( REG_LR_RANGINGFILTERWINDOWSIZE, ( num < DEFAULT_RANGING_FILTER_SIZE ) ? DEFAULT_RANGING_FILTER_SIZE : num );
			}

			void SetRangingRole( RadioRangingRoles role ) {
				uint8_t buf[1];

				buf[0] = role;
				WriteCommand( RADIO_SET_RANGING_ROLE, &buf[0], 1 );
			}
			#endif  // #ifdef RANGING_SUPPORT

			double GetFrequencyError( ) {
				uint8_t efeRaw[3] = {0};
				uint32_t efe = 0;
				double efeHz = 0.0;

				switch( GetPacketType( true ) )
				{
				#if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)
					case PACKET_TYPE_LORA:
					case PACKET_TYPE_RANGING:
						efeRaw[0] = ReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB );
						efeRaw[1] = ReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1 );
						efeRaw[2] = ReadRegister( REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2 );
						efe = ( efeRaw[0]<<16 ) | ( efeRaw[1]<<8 ) | efeRaw[2];
						efe &= REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK;

						efeHz = 1.55 * ( double )complement2( efe, 20 ) / ( 1600.0 / ( double )GetLoRaBandwidth( ) * 1000.0 );
						break;
				#endif  // #if defined(LORA_SUPPORT) || defined(RANGING_SUPPORT)

					case PACKET_TYPE_NONE:
					case PACKET_TYPE_BLE:
					case PACKET_TYPE_FLRC:
					case PACKET_TYPE_GFSK:
						break;
				}

				return efeHz;
			}

			void SetPollingMode( void ) {
				PollingMode = true;
			}

			#ifdef LORA_SUPPORT
			int32_t GetLoRaBandwidth( ) {
				int32_t bwValue = 0;

				switch( LoRaBandwidth ) {
					case LORA_BW_0200:
						bwValue = 203125;
						break;
					case LORA_BW_0400:
						bwValue = 406250;
						break;
					case LORA_BW_0800:
						bwValue = 812500;
						break;
					case LORA_BW_1600:
						bwValue = 1625000;
						break;
					default:
						bwValue = 0;
				}
				return bwValue;
			}
			#endif  // #ifdef LORA_SUPPORT

			void SetInterruptMode( void ) {
				PollingMode = false;
			}

			void OnDioIrq( void ) {
				/*
				 * When polling mode is activated, it is up to the application to call
				 * ProcessIrqs( ). Otherwise, the driver automatically calls ProcessIrqs( )
				 * on radio interrupt.
				 */
				if( PollingMode == true ) {
					IrqState = true;
				} else {
					ProcessIrqs( );
				}
			}

			void ProcessIrqs( void ) {
				RadioPacketTypes packetType = PACKET_TYPE_NONE;

				if( PollingMode == true ) {
					if( IrqState == true ) {
			            disableIRQ( );
						IrqState = false;
			            enableIRQ( );
					} else {
						return;
					}
				}

				packetType = GetPacketType( true );
				uint16_t irqRegs = GetIrqStatus( );
				ClearIrqStatus( IRQ_RADIO_ALL );

				switch( packetType )
				{
				#if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT) || defined(BLE_SUPPORT)
					case PACKET_TYPE_GFSK:
					case PACKET_TYPE_FLRC:
					case PACKET_TYPE_BLE:
						switch( OperatingMode )
						{
							case MODE_RX:
								if( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE )
								{
									if( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR )
									{
										rxError( IRQ_CRC_ERROR_CODE );
									}
									else if( ( irqRegs & IRQ_SYNCWORD_ERROR ) == IRQ_SYNCWORD_ERROR )
									{
										rxError( IRQ_SYNCWORD_ERROR_CODE );
									}
									else
									{
										rxDone( );
									}
								}
								if( ( irqRegs & IRQ_SYNCWORD_VALID ) == IRQ_SYNCWORD_VALID )
								{
									rxSyncWordDone( );
								}
								if( ( irqRegs & IRQ_SYNCWORD_ERROR ) == IRQ_SYNCWORD_ERROR )
								{
									rxError( IRQ_SYNCWORD_ERROR_CODE );
								}
								if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
								{
									rxTimeout( );
								}
								break;
							case MODE_TX:
								if( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE )
								{
									txDone( );
								}
								if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
								{
									txTimeout( );
								}
								break;
							default:
								// Unexpected IRQ: silently returns
								break;
						}
						break;
				#endif  // #if defined(GFSK_SUPPORT) || defined(FLRC_SUPPORT) || defined(BLE_SUPPORT)
				#ifdef LORA_SUPPORT
					case PACKET_TYPE_LORA:
						switch( OperatingMode )
						{
							case MODE_RX:
								if( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE )
								{
									if( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR )
									{
										rxError( IRQ_CRC_ERROR_CODE );
									}
									else
									{
										rxDone( );
									}
								}
								if( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
								{
									rxHeaderDone( );
								}
								if( ( irqRegs & IRQ_HEADER_ERROR ) == IRQ_HEADER_ERROR )
								{
									rxError( IRQ_HEADER_ERROR_CODE );
								}
								if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
								{
									rxTimeout( );
								}
								if( ( irqRegs & IRQ_RANGING_SLAVE_REQUEST_DISCARDED ) == IRQ_RANGING_SLAVE_REQUEST_DISCARDED )
								{
									rxError( IRQ_RANGING_ON_LORA_ERROR_CODE );
								}
								break;
							case MODE_TX:
								if( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE )
								{
									txDone( );
								}
								if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
								{
									txTimeout( );
								}
								break;
							case MODE_CAD:
								if( ( irqRegs & IRQ_CAD_DONE ) == IRQ_CAD_DONE )
								{
									if( ( irqRegs & IRQ_CAD_DETECTED ) == IRQ_CAD_DETECTED )
									{
										cadDone( true );
									}
									else
									{
										cadDone( false );
									}
								}
								else if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
								{
									rxTimeout( );
								}
								break;
							default:
								// Unexpected IRQ: silently returns
								break;
						}
						break;
				#endif  // #ifdef LORA_SUPPORT
				#ifdef RANGING_SUPPORT
					case PACKET_TYPE_RANGING:
						switch( OperatingMode )
						{
							// MODE_RX indicates an IRQ on the Slave side
							case MODE_RX:
								if( ( irqRegs & IRQ_RANGING_SLAVE_REQUEST_DISCARDED ) == IRQ_RANGING_SLAVE_REQUEST_DISCARDED )
								{
									rangingDone( IRQ_RANGING_SLAVE_ERROR_CODE );
								}
								if( ( irqRegs & IRQ_RANGING_SLAVE_REQUEST_VALID ) == IRQ_RANGING_SLAVE_REQUEST_VALID )
								{
									rangingDone( IRQ_RANGING_SLAVE_VALID_CODE );
								}
								if( ( irqRegs & IRQ_RANGING_SLAVE_RESPONSE_DONE ) == IRQ_RANGING_SLAVE_RESPONSE_DONE )
								{
									rangingDone( IRQ_RANGING_SLAVE_VALID_CODE );
								}
								if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
								{
									rangingDone( IRQ_RANGING_SLAVE_ERROR_CODE );
								}
								if( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
								{
									rxHeaderDone( );
								}
								if( ( irqRegs & IRQ_HEADER_ERROR ) == IRQ_HEADER_ERROR )
								{
									rxError( IRQ_HEADER_ERROR_CODE );
								}
								break;
							// MODE_TX indicates an IRQ on the Master side
							case MODE_TX:
								if( ( irqRegs & IRQ_RANGING_MASTER_TIMEOUT ) == IRQ_RANGING_MASTER_TIMEOUT )
								{
									rangingDone( IRQ_RANGING_MASTER_ERROR_CODE );
								}
								if( ( irqRegs & IRQ_RANGING_MASTER_RESULT_VALID ) == IRQ_RANGING_MASTER_RESULT_VALID )
								{
									rangingDone( IRQ_RANGING_MASTER_VALID_CODE );
								}
								break;
							default:
								// Unexpected IRQ: silently returns
								break;
						}
						break;
				#endif  // #ifdef RANGING_SUPPORT
					default:
						// Unexpected IRQ: silently returns
						break;
				}
			}

			void txDone() {
				//sdd1306.PlaceAsciiStr(0,0,"TXDONE!!");
				//sdd1306.Display();
				//delay(250);
		    	SetRx( TickTime { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE } );
			}

			void rxDone() {
				//sdd1306.PlaceAsciiStr(0,0,"RXDONE!!");
				//sdd1306.Display();
				//delay(250);
			    PacketStatus packetStatus;
				GetPacketStatus(&packetStatus);
				uint8_t rxBufferSize = 0;
                GetPayload( rxBuffer, &rxBufferSize, LORA_BUFFER_SIZE );
				if (memcmp(rxBuffer,"DUCK!!",6) == 0) {
					memcpy(settings.recv_radio_message, rxBuffer+8, 8);
					memcpy(settings.recv_radio_name, rxBuffer+16, 8);
					settings.recv_radio_color = rxBuffer[7];
					if (settings.radio_enabled) {
						settings.recv_radio_message_pending = true;
						settings.UpdateRecvCount();
						settings.RecordMessage(ft25h16s, rxBuffer);
					}
				}
			}

			void rxSyncWordDone() {
				sdd1306.PlaceAsciiStr(0,0,"RXSYNCDO");
				sdd1306.Display();
				delay(250);
			}

			void rxHeaderDone() {
				sdd1306.PlaceAsciiStr(0,0,"RXHEADER");
				sdd1306.Display();
				delay(250);
			}
			
			void txTimeout() {
				sdd1306.PlaceAsciiStr(0,0,"TXTIMOUT");
				sdd1306.Display();
				delay(250);
			}
			
			void rxTimeout() {
				sdd1306.PlaceAsciiStr(0,0,"RXTIMOUT");
				sdd1306.Display();
				delay(250);
			}
			
			void rxError(IrqErrorCode errCode) {
				sdd1306.PlaceAsciiStr(0,0,"RXERROR!");
				sdd1306.Display();
				delay(250);
			}

			void rangingDone(IrqRangingCode errCode) {
				// TODO
			}

			void cadDone(bool cadFlag) {
				// TODO
			}
			
			void SendMessage() {
				char buf[24];
				memcpy(buf,"DUCK!!",6);
				buf[6] = 0;
				buf[7] = settings.radio_color;
				memcpy(buf+8,settings.radio_messages[settings.radio_message],8);
				memcpy(buf+16,settings.radio_name,8);
				memcpy(txBuffer,buf,24);
				SendBuffer();
				settings.UpdateSentCount();
				settings.RecordMessage(ft25h16s, txBuffer);
			}
			
	private:
	
			void disableIRQ() {
				NVIC_DisableIRQ(PIN_INT0_IRQn);
			}

			void enableIRQ() {
				NVIC_EnableIRQ(PIN_INT0_IRQn);
			}

			uint8_t spiwrite(uint8_t val) {
				Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_SCLK>>8), (SPI_SCLK&0xFF));
				uint8_t read_value = 0;
				for (uint32_t c=0; c<8; c++) {
					read_value <<= 1;
					Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_SCLK>>8), (SPI_SCLK&0xFF));
					if ((val&(1<<(7-c)))) {
						Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_MOSI>>8), (SPI_MOSI&0xFF));
					} else {
						Chip_GPIO_SetPinOutLow(LPC_GPIO, (SPI_MOSI>>8), (SPI_MOSI&0xFF));
					}
					read_value |= Chip_GPIO_GetPinState(LPC_GPIO, (SPI_MISO>>8), (SPI_MISO&0xFF));
					Chip_GPIO_SetPinOutHigh(LPC_GPIO, (SPI_SCLK>>8), (SPI_SCLK&0xFF));
				}
				return read_value;
			}
						
			int32_t complement2( const uint32_t num, const uint8_t bitCnt ) {
				int32_t retVal = int32_t(num);
				if( int32_t(num) >= 2<<( bitCnt - 2 ) ) {
					retVal -= 2<<( bitCnt - 1 );
				}
				return retVal;
			}

    RadioOperatingModes 	OperatingMode = MODE_SLEEP;
	RadioPacketTypes 		PacketType = PACKET_TYPE_NONE;
    RadioLoRaBandwidths 	LoRaBandwidth = LORA_BW_0200;
    bool 					IrqState = false;
    bool 					PollingMode = true;

};

class UI {
	
	EEPROM &settings;
	SDD1306 &sdd1306;
	SX1280 &sx1280;
	Random &random;
	FT25H16S &ft25h16s;
	BQ24295 &bq24295;
	
	uint32_t mode;
	uint32_t mode_start_time;

	uint32_t top_fall_time;
	bool top_button_down;
	bool top_short_press;
	
	uint32_t bottom_fall_time;
	bool bottom_button_down;
	bool bottom_short_press;
	
	int32_t previous_mode;
	int32_t interlude;


	int32_t menu_scroll;
	int32_t max_menu;
	int32_t max_menu_scroll;
	int32_t menu_selection;
	
	bool ring_or_duck;
	int32_t rgb_selection;
	
	int32_t radio_settings_selection;
	
	int32_t radio_selection;

	int32_t radio_name_selection;

	int32_t radio_message_selection;
	int32_t radio_message_current;

	int32_t stats_select_current;
	int32_t stats_menu_selection;

	int32_t history_select_current;
	int32_t history_menu_selection;
	
public:

	#define LONG_PRESS_TIME 750
	
	UI(EEPROM &_settings,
	   SDD1306 &_sdd1306,
	   SX1280 &_sx1280,
	   Random &_random,
	   FT25H16S &_ft25h16s,
	   BQ24295 &_bq24295):
		settings(_settings),
		sdd1306(_sdd1306),
		sx1280(_sx1280),
		random(_random),
		ft25h16s(_ft25h16s),
		bq24295(_bq24295) {
	}

	const uint32_t PRIMARY_BUTTON = 0x0119;
	const uint32_t SECONDARY_BUTTON = 0x0001;

	void Init() {
		mode = 0;
		
		top_fall_time = 0;
		top_button_down = false;
		top_short_press = false;
		bottom_fall_time = 0;
		bottom_button_down = false;
		bottom_short_press = false;
		
		menu_scroll = 0;
		max_menu = 7;
		max_menu_scroll = 4;
		menu_selection = 0;
		
		ring_or_duck = false;
		
		rgb_selection = 0;
		
		radio_settings_selection = 0;
		
		radio_selection = 0;
		
		radio_name_selection = 0;
		
		radio_message_selection = 0;
		radio_message_current = 0;

		mode_start_time = 0;
		previous_mode = 0;
		interlude = 0;
		
		stats_select_current = 0;
		stats_menu_selection = 0;

		history_select_current = 0;
		history_menu_selection = 0;

		Chip_IOCON_PinMuxSet(LPC_IOCON, uint8_t(PRIMARY_BUTTON>>8), uint8_t(PRIMARY_BUTTON&0xFF), IOCON_FUNC0 | IOCON_MODE_PULLUP);
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, uint8_t(PRIMARY_BUTTON>>8), uint8_t(PRIMARY_BUTTON&0xFF));

		Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH1);
		Chip_PININT_EnableIntLow(LPC_PININT, Chip_PININT_GetHighEnabled(LPC_PININT) | PININTCH1);
		Chip_PININT_EnableIntHigh(LPC_PININT, Chip_PININT_GetLowEnabled(LPC_PININT) | PININTCH1);
		Chip_SYSCTL_SetPinInterrupt(1, uint8_t(PRIMARY_BUTTON>>8), uint8_t(PRIMARY_BUTTON&0xFF));  

		NVIC_ClearPendingIRQ(PIN_INT1_IRQn);  
		Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH1);
		NVIC_EnableIRQ(PIN_INT1_IRQn);  
		
		Chip_IOCON_PinMuxSet(LPC_IOCON, uint8_t(SECONDARY_BUTTON>>8), uint8_t(SECONDARY_BUTTON&0xFF), IOCON_FUNC0 | IOCON_MODE_PULLUP);
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, uint8_t(SECONDARY_BUTTON>>8), uint8_t(SECONDARY_BUTTON&0xFF));

		Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH2);
		Chip_PININT_EnableIntLow(LPC_PININT, Chip_PININT_GetHighEnabled(LPC_PININT) | PININTCH2);
		Chip_PININT_EnableIntHigh(LPC_PININT, Chip_PININT_GetLowEnabled(LPC_PININT) | PININTCH2);
		Chip_SYSCTL_SetPinInterrupt(2, uint8_t(SECONDARY_BUTTON>>8), uint8_t(SECONDARY_BUTTON&0xFF));  

		NVIC_ClearPendingIRQ(PIN_INT2_IRQn);  
		Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH2);
		NVIC_EnableIRQ(PIN_INT2_IRQn);  
	}

	void HandleINT1IRQ() {
		uint32_t timer_ms = Chip_TIMER_ReadCount(LPC_TIMER32_0);
		if ( (Chip_PININT_GetFallStates(LPC_PININT) & PININTCH1) ) {
			Chip_PININT_ClearFallStates(LPC_PININT, PININTCH1);
			top_fall_time = timer_ms;
			top_button_down = true;
			Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH1);
		}

		if ( (Chip_PININT_GetRiseStates(LPC_PININT) & PININTCH1) ) {
			Chip_PININT_ClearRiseStates(LPC_PININT, PININTCH1);
			if ( top_button_down && (timer_ms - top_fall_time) > 10 && ((timer_ms - top_fall_time) < LONG_PRESS_TIME)) {
				top_short_press = true;
			}
			top_button_down = false;
			Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH1);
		}
	}
	
	void TopLongPress() {
		uint32_t timer_ms = Chip_TIMER_ReadCount(LPC_TIMER32_0);
		if ( top_button_down && (timer_ms - top_fall_time) > LONG_PRESS_TIME) {
			if (Mode() != 0) {
				SetMode(system_clock_ms, 0);
			} else {
				SetMode(system_clock_ms, 2);
			}
			top_button_down = false;
		}
	}

	void TopShortPress() {
		if (top_short_press) {
			top_short_press = false;
			switch (mode) {
				case 	0: {
					settings.NextEffect();
				} break;
				case	1: {
					// NOP
				} break;
				case	2: {
					SettingsHandler(true, false);
				} break;
				case	3: {
					RGBGHandler(true, false);
				} break;
				case	4: {
					RadioSettingsHandler(true, false);
				} break;
				case	5: {
					RadioHandler(true, false);
				} break;
				case	6: {
					// NOP
				} break;
				case	7: {
					NameSettingsHandler(true, false);
				} break;
				case	8: {
					MessageSettingsHandler(true, false);
				} break;
				case	9: {
					StatsHandler(true, false);
				} break;
				case	10: {
					HistoryHandler(true, false);
				} break;
			}
		}
	}
	
	void HandleINT2IRQ() {
		uint32_t timer_ms = Chip_TIMER_ReadCount(LPC_TIMER32_0);
		if ( (Chip_PININT_GetFallStates(LPC_PININT) & PININTCH2) ) {
			Chip_PININT_ClearFallStates(LPC_PININT, PININTCH2);
			bottom_fall_time = timer_ms;
			bottom_button_down = true;
			Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH2);
		}

		if ( (Chip_PININT_GetRiseStates(LPC_PININT) & PININTCH2) ) {
			Chip_PININT_ClearRiseStates(LPC_PININT, PININTCH2);
			if ( bottom_button_down && (timer_ms - bottom_fall_time) > 10 && ((timer_ms - bottom_fall_time) < LONG_PRESS_TIME)) {
				bottom_short_press = true;
			}
			bottom_button_down = false;
			Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH2);
		}
	}
	
	void BottomLongPress() {
		uint32_t timer_ms = Chip_TIMER_ReadCount(LPC_TIMER32_0);
		if (Mode() == 2 && menu_selection == 5) {
			if ( bottom_button_down && (timer_ms - bottom_fall_time) > 5000) {
				sdd1306.PlaceAsciiStr(0,0,"        ");
				sdd1306.PlaceAsciiStr(0,1,"  HARD  ");
				sdd1306.PlaceAsciiStr(0,2," RESET! ");
				sdd1306.PlaceAsciiStr(0,3,"[00/04] ");
				sdd1306.Display();

				Chip_WWDT_SetTimeOut(LPC_WWDT, 60 * Chip_Clock_GetWDTOSCRate() / 4);
				NVIC_DisableIRQ(I2C0_IRQn);
				NVIC_DisableIRQ(WDT_IRQn);
				NVIC_DisableIRQ(PIN_INT0_IRQn);  
				NVIC_DisableIRQ(PIN_INT1_IRQn);  
				NVIC_DisableIRQ(PIN_INT2_IRQn);  

				sdd1306.PlaceAsciiStr(0,3,"[01/04] ");
				sdd1306.Display();

				settings.Reset(true);

				sdd1306.PlaceAsciiStr(0,3,"[02/04] ");
				sdd1306.Display();
				
				ft25h16s.chip_erase();

				sdd1306.PlaceAsciiStr(0,3,"[03/04] ");
				sdd1306.Display();

				EEPROM::loaded = false;
				
				delay(500);

				sdd1306.PlaceAsciiStr(0,3,"[04/04] ");
				sdd1306.Display();

				delay(500);

				NVIC_SystemReset();
				bottom_button_down = false;
			}
		} else if ( bottom_button_down && (timer_ms - bottom_fall_time) > LONG_PRESS_TIME) {
			if (Mode() != 0) {
				SetMode(system_clock_ms, 0);
			} else {
				SetMode(system_clock_ms, 5);
			}
			bottom_button_down = false;
		}
	}
	
	void BottomShortPress() {
		if (bottom_short_press) {
			bottom_short_press = false;
			switch (mode) {
				case 	0: {
					settings.NextBrightness();
				} break;
				case	1: {
					// NOP
				} break;
				case	2: {
					SettingsHandler(false, true);
				} break;
				case	3: {
					RGBGHandler(false, true);
				} break;
				case	4: {
					RadioSettingsHandler(false, true);
				} break;
				case	5: {
					RadioHandler(false, true);
				} break;
				case	6: {
					// NOP
				} break;
				case	7: {
					NameSettingsHandler(false, true);
				} break;
				case	8: {
					MessageSettingsHandler(false, true);
				} break;
				case	9: {
					StatsHandler(false, true);
				} break;
				case	10: {
					HistoryHandler(false, true);
				} break;
			}
		}
	}
	
	void CheckInput() {
		TopLongPress();
		TopShortPress();
		BottomLongPress();
		BottomShortPress();
	}

	void SetMode(uint32_t current_time, uint32_t _mode) {
		mode_start_time = current_time;
		previous_mode = mode;
		mode = _mode;
		sdd1306.ClearAttr();
		if (mode == 1) {
			interlude = random.get(0,2);
		}
		sdd1306.SetVerticalShift(0);
		sdd1306.SetCenterFlip(0);
		Display();
	}
	
	uint32_t Mode() const { return mode; }
	
	void DisplayDog() {
		if ((system_clock_ms - mode_start_time) < 50) {
			for (int32_t c=0; c<8; c++) {
				sdd1306.PlaceCustomChar(c,0,0x128+c);
			}
			for (int32_t c=0; c<8; c++) {
				sdd1306.PlaceCustomChar(c,1,0x130+c);
			}
			for (int32_t c=0; c<8; c++) {
				sdd1306.PlaceCustomChar(c,2,0x138+c);
			}
			for (int32_t c=0; c<8; c++) {
				sdd1306.PlaceCustomChar(c,3,0x140+c);
			}
			sdd1306.SetVerticalShift(0);
		}
		else if ((system_clock_ms - mode_start_time) < 50 + 640 + 50) {
			uint32_t ltime = (system_clock_ms - mode_start_time) - 50;
			static uint8_t bounce[] = {
				0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1e, 0x1d, 0x1d, 
				0x1c, 0x1b, 0x1a, 0x18, 0x17, 0x16, 0x14, 0x12, 
				0x10, 0x0e, 0x0c, 0x0a, 0x08, 0x05, 0x03, 0x01, 
				0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x07, 0x08, 
				0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x07, 0x07, 
				0x06, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x01, 
				0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 
				0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 
			};
			uint32_t y = ltime/10;
			if (y >= 64) y = 63;
			sdd1306.Display();
			sdd1306.SetVerticalShift(bounce[y]);
		}
		else if ((system_clock_ms - mode_start_time) < 2000 + 50 + 640 + 50) {
		}
		else if ((system_clock_ms - mode_start_time) < 160 + 2000 + 50 + 640 + 50) {
			uint32_t ltime = (system_clock_ms - mode_start_time) - (2000 + 50 + 640 + 50);
			static int8_t ease[] = {
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 
				0x02, 0x02, 0x03, 0x03, 0x04, 0x05, 0x06, 0x07, 
				0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0e, 0x0f, 0x11, 
				0x12, 0x14, 0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f, 
				0x1f,
			};
			sdd1306.SetVerticalShift(-ease[ltime/5]);
			sdd1306.SetCenterFlip(ltime/5);
			sdd1306.Display();
		} else {
			mode = previous_mode;
			sdd1306.ClearAttr();
			DisplayStatus();
			sdd1306.SetVerticalShift(0);
			sdd1306.SetCenterFlip(0);
			sdd1306.Display();
		}
	}

	void DisplayNow() {
		if ((system_clock_ms - mode_start_time) < 50) {
			for (int32_t c=0; c<8; c++) {
				sdd1306.PlaceCustomChar(c,0,0xB8+c);
			}
			for (int32_t c=0; c<8; c++) {
				sdd1306.PlaceCustomChar(c,1,0xC0+c);
			}
			for (int32_t c=0; c<8; c++) {
				sdd1306.PlaceCustomChar(c,2,0xC8+c);
			}
			for (int32_t c=0; c<8; c++) {
				sdd1306.PlaceCustomChar(c,3,0xD0+c);
			}
			sdd1306.SetVerticalShift(0);
		}
		else if ((system_clock_ms - mode_start_time) < 50 + 640 + 50) {
			uint32_t ltime = (system_clock_ms - mode_start_time) - 50;
			static uint8_t bounce[] = {
				0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1e, 0x1d, 0x1d, 
				0x1c, 0x1b, 0x1a, 0x18, 0x17, 0x16, 0x14, 0x12, 
				0x10, 0x0e, 0x0c, 0x0a, 0x08, 0x05, 0x03, 0x01, 
				0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x07, 0x08, 
				0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x07, 0x07, 
				0x06, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x01, 
				0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 
				0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 
			};
			uint32_t y = ltime/10;
			if (y >= 64) y = 63;
			sdd1306.Display();
			sdd1306.SetVerticalShift(bounce[y]);
		}
		else if ((system_clock_ms - mode_start_time) < 1000 + 50 + 640 + 50) {
			uint32_t ltime = (system_clock_ms - mode_start_time) - (50 + 640 + 50);
			if (((ltime / 100)&1) == 0) {
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,0,0);
				}
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,1,0);
				}
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,2,0);
				}
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,3,0xD0+c);
				}
				sdd1306.Display();
			} else if (((ltime / 100)&1) == 1) {
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,0,0xB8+c);
				}
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,1,0xC0+c);
				}
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,2,0xC8+c);
				}
				for (int32_t c=0; c<8; c++) {
					sdd1306.PlaceCustomChar(c,3,0xD0+c);
				}
				sdd1306.Display();
			}
		}
		else if ((system_clock_ms - mode_start_time) < 160 + 1000 + 50 + 640 + 50) {
			uint32_t ltime = (system_clock_ms - mode_start_time) - (1000 + 50 + 640 + 50);
			static int8_t ease[] = {
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 
				0x02, 0x02, 0x03, 0x03, 0x04, 0x05, 0x06, 0x07, 
				0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0e, 0x0f, 0x11, 
				0x12, 0x14, 0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f, 
				0x1f,
			};
			sdd1306.SetVerticalShift(-ease[ltime/5]);
			sdd1306.SetCenterFlip(ltime/5);
			sdd1306.Display();
		} else {
			mode = previous_mode;
			sdd1306.ClearAttr();
			DisplayStatus();
			sdd1306.SetVerticalShift(0);
			sdd1306.SetCenterFlip(0);
			sdd1306.Display();
		}
	}
	
	void DisplayBar(uint8_t x, uint8_t y, uint8_t w, uint8_t val, uint8_t range) {
		if (y >= 4) {
			return;
		}
		if (w == 7 && x <= 1) {
			if (range == 0) {
				const uint8_t val_to_chr[7*8] = {
					0x6B, 0x6E, 0x6E, 0x6E, 0x6E, 0x6E, 0x71,
					0x6D, 0x6E, 0x6E, 0x6E, 0x6E, 0x6E, 0x71,
					0x6D, 0x70, 0x6E, 0x6E, 0x6E, 0x6E, 0x71,
					0x6D, 0x70, 0x70, 0x6E, 0x6E, 0x6E, 0x71,
					0x6D, 0x70, 0x70, 0x70, 0x6E, 0x6E, 0x71,
					0x6D, 0x70, 0x70, 0x70, 0x70, 0x6E, 0x71,
					0x6D, 0x70, 0x70, 0x70, 0x70, 0x70, 0x71,
					0x6D, 0x70, 0x70, 0x70, 0x70, 0x70, 0x73,
				};
				for (uint32_t c=0; c<7; c++) {
					sdd1306.PlaceCustomChar(c+x,y,val_to_chr[val*7+c]);
				}
			} else {
				const uint8_t val_to_chr[7*14] = {
					0x6B, 0x6E, 0x6E, 0x6E, 0x6E, 0x6E, 0x71,
					0x6C, 0x6E, 0x6E, 0x6E, 0x6E, 0x6E, 0x71,
					0x6D, 0x6F, 0x6E, 0x6E, 0x6E, 0x6E, 0x71,
					0x6D, 0x70, 0x6E, 0x6E, 0x6E, 0x6E, 0x71,
					0x6D, 0x70, 0x70, 0x6E, 0x6E, 0x6E, 0x71,
					0x6D, 0x70, 0x70, 0x6E, 0x6E, 0x6E, 0x71,
					0x6D, 0x70, 0x70, 0x6F, 0x6E, 0x6E, 0x71,
					0x6D, 0x70, 0x70, 0x70, 0x6E, 0x6E, 0x71,
					0x6D, 0x70, 0x70, 0x70, 0x6F, 0x6E, 0x71,
					0x6D, 0x70, 0x70, 0x70, 0x70, 0x6E, 0x71,
					0x6D, 0x70, 0x70, 0x70, 0x70, 0x6F, 0x71,
					0x6D, 0x70, 0x70, 0x70, 0x70, 0x70, 0x71,
					0x6D, 0x70, 0x70, 0x70, 0x70, 0x70, 0x72,
					0x6D, 0x70, 0x70, 0x70, 0x70, 0x70, 0x73,
				};
				for (uint32_t c=0; c<7; c++) {
					sdd1306.PlaceCustomChar(c+x,y,val_to_chr[val*7+c]);
				}
			}
		}
	}
	
	uint8_t BadConnection() {
		static int32_t avg_buf[16];
		static int32_t avg_pos = 0;

		uint16_t adc5_value = 0;
		Chip_ADC_SetStartMode(LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
		while (Chip_ADC_ReadStatus(LPC_ADC, ADC_CH5, ADC_DR_DONE_STAT) != SET) {}
		Chip_ADC_ReadValue(LPC_ADC, ADC_CH5, &adc5_value);
		
		avg_buf[(avg_pos++)&0xF] = (uint32_t(adc5_value) * 4800) / 1024;
		int32_t avg = 0;
		
		for (uint32_t c=0; c<16; c++) {
			avg += avg_buf[c];
		}
		
		int32_t charge = ((((avg / 16) - 1500) * 255 ) / (4200 - 1500));
		if (charge < 0) charge = 0;
		if (charge >= 256 ) charge = 255;
		return charge;
	}

	void DisplayStatus() {
		sdd1306.PlaceCustomChar(0,0,0xA9);
		sdd1306.PlaceCustomChar(1,0,0xAA);
		sdd1306.PlaceCustomChar(2,0,0xAB);
		sdd1306.PlaceCustomChar(3,0,0xAC);
		sdd1306.PlaceCustomChar(4,0,0xAD);
		sdd1306.PlaceCustomChar(5,0,0xAE);
		sdd1306.PlaceCustomChar(6,0,0xAF);
		sdd1306.PlaceCustomChar(7,0,0xA0+(system_clock_ms/0x400)%8);
		sdd1306.PlaceCustomChar(0,1,0x65);
		DisplayBar(1,1,7,uint8_t(settings.brightness), 0);
		sdd1306.PlaceCustomChar(0,2,0x67);
		char str[9];
		sprintf(str,"[%02d/%02d]",settings.program_curr + 1,settings.program_count);
		sdd1306.PlaceAsciiStr(1,2,str);


		sdd1306.PlaceCustomChar(0,3,0x66);
		uint8_t bat_stat = bq24295.GetStatus();
		if (BadConnection()) {
			sdd1306.PlaceAsciiStr(1,3,"BATERR!");
		} else {
			sdd1306.PlaceCustomChar(1,3,(bat_stat & 0x01) ? 0x289 : 0x288);
			sdd1306.PlaceCustomChar(2,3,(bat_stat & 0x02) ? 0x289 : 0x288);
			sdd1306.PlaceCustomChar(3,3,(bat_stat & 0x08) ? 0x289 : 0x288);
			sdd1306.PlaceCustomChar(4,3,(bat_stat & 0x10) ? 0x289 : 0x288);
			sdd1306.PlaceCustomChar(5,3,(bat_stat & 0x20) ? 0x289 : 0x288);
			sdd1306.PlaceCustomChar(6,3,(bat_stat & 0x40) ? 0x289 : 0x288);
			sdd1306.PlaceCustomChar(7,3,(bat_stat & 0x80) ? 0x289 : 0x288);
		}

		sdd1306.Display();
	}
	
	void DisplaySettings() {
		sdd1306.PlaceCustomChar(0,0,0x7B);
		sdd1306.PlaceCustomChar(1,0,0x149);
		sdd1306.PlaceCustomChar(2,0,0x14A);
		sdd1306.PlaceCustomChar(3,0,0x14B);
		sdd1306.PlaceCustomChar(4,0,0x14C);
		sdd1306.PlaceCustomChar(5,0,0x14D);
		sdd1306.PlaceCustomChar(6,0,0x14E);
		sdd1306.PlaceCustomChar(7,0,0x14F);
		if (menu_selection == 0) {
			sdd1306.SetAttr(0,0,1);
		} else {
			sdd1306.SetAttr(0,0,0);
		}
		const char *menu[] = {
			"1COLORS ",
			"2RADIO  ",
			"3HISTORY",
			"4STATS  ",
			"5RESET  ",
			"6TEST   ",
			"7VERSION",
		};
		for (int32_t c=0; c<3; c++) {
			sdd1306.PlaceAsciiStr(0,c+1,menu[c+menu_scroll]);
			if (menu_selection == c+1+menu_scroll) {
				sdd1306.SetAttr(0,c+1,1);
			} else {
				sdd1306.SetAttr(0,c+1,0);
			}
		}
		sdd1306.Display();
	}
	
	void SettingsHandler(bool top_pressed, bool bottom_pressed) {
		if (top_pressed) {
			menu_selection++;
			if (menu_selection > max_menu) {
				menu_selection = 0;
				menu_scroll = 0;
			}
			if (menu_selection > 3) {
				menu_scroll = menu_selection - 3;
			}
		}
		if (bottom_pressed) {
			switch (menu_selection) { 
				case	0: {
							menu_scroll = 0;
							menu_selection = 0;
							SetMode(system_clock_ms, 0);
							return;
						} break;
				case	1: {
							menu_scroll = 0;
							menu_selection = 0;
							SetMode(system_clock_ms, 3);
							return;
						} break;
				case	2: {
							menu_scroll = 0;
							menu_selection = 0;
							SetMode(system_clock_ms, 4);
							return;
						} break;
				case	3: {
							menu_scroll = 0;
							menu_selection = 0;
							SetMode(system_clock_ms, 10);
							return;
						} break;
				case	4: {
							menu_scroll = 0;
							menu_selection = 0;
							SetMode(system_clock_ms, 9);
							return;
						} break;
				case	5: {
							settings.Reset(false);
							EEPROM::loaded = false;
							NVIC_SystemReset();
						} break;
				case	6: {
							sdd1306.ClearAttr();
							DisplayTest();
						} break;
				case	7: {
							sdd1306.ClearAttr();
							DisplayVersion();
						} break;
			}
		}
		DisplaySettings();
	}

	void DisplayRGB() {
		
		sdd1306.SetAttr(0,0,0);
		sdd1306.SetAttr(0,1,0);
		sdd1306.SetAttr(0,2,0);
		sdd1306.SetAttr(0,3,0);
		
		switch (rgb_selection) {
			case	0: {
						sdd1306.SetAttr(0,0,1);
					} break;
			case	1: {
					} break;
			case	2: {
						sdd1306.SetAttr(0,1,1);
					} break;
			case	3: {
						sdd1306.SetAttr(0,2,1);
					} break;
			case	4: {
						sdd1306.SetAttr(0,3,1);
					} break;
		}
		
		sdd1306.PlaceCustomChar(0,0,0x7B);
		
		if (rgb_selection == 1) {
			if (ring_or_duck) {
				sdd1306.PlaceCustomChar(1,0,0x157);
				sdd1306.PlaceCustomChar(2,0,0x158);
				sdd1306.PlaceCustomChar(3,0,0x159);
				sdd1306.PlaceCustomChar(4,0,0x15A);
				sdd1306.PlaceCustomChar(5,0,0x15B);
				sdd1306.PlaceCustomChar(6,0,0x15C);
				sdd1306.PlaceCustomChar(7,0,0x15D);
			} else {
				sdd1306.PlaceCustomChar(1,0,0x150);
				sdd1306.PlaceCustomChar(2,0,0x151);
				sdd1306.PlaceCustomChar(3,0,0x152);
				sdd1306.PlaceCustomChar(4,0,0x153);
				sdd1306.PlaceCustomChar(5,0,0x154);
				sdd1306.PlaceCustomChar(6,0,0x155);
				sdd1306.PlaceCustomChar(7,0,0x156);
			}
		} else {
			if (ring_or_duck) {
				sdd1306.PlaceCustomChar(1,0,0x15E);
				sdd1306.PlaceCustomChar(2,0,0x15F);
				sdd1306.PlaceCustomChar(3,0,0x160);
				sdd1306.PlaceCustomChar(4,0,0x161);
				sdd1306.PlaceCustomChar(5,0,0x162);
				sdd1306.PlaceCustomChar(6,0,0x163);
				sdd1306.PlaceCustomChar(7,0,0x164);
			} else {
				sdd1306.PlaceCustomChar(1,0,0x165);
				sdd1306.PlaceCustomChar(2,0,0x166);
				sdd1306.PlaceCustomChar(3,0,0x167);
				sdd1306.PlaceCustomChar(4,0,0x168);
				sdd1306.PlaceCustomChar(5,0,0x169);
				sdd1306.PlaceCustomChar(6,0,0x16A);
				sdd1306.PlaceCustomChar(7,0,0x16B);
			}
		}
		
		sdd1306.PlaceCustomChar(0,1,0x68);
		DisplayBar(1,1,7,uint8_t(ring_or_duck ? settings.ring_color.r()/16 : settings.bird_color.r()/16), 0);

		sdd1306.PlaceCustomChar(0,2,0x69);
		DisplayBar(1,2,7,uint8_t(ring_or_duck ? settings.ring_color.g()/16 : settings.bird_color.g()/16), 0);

		sdd1306.PlaceCustomChar(0,3,0x6A);
		DisplayBar(1,3,7,uint8_t(ring_or_duck ? settings.ring_color.b()/16 : settings.bird_color.b()/16), 0);

		sdd1306.Display();
	}
	
	
	void RGBGHandler(bool top_pressed, bool bottom_pressed) {
		if (top_pressed) {
			rgb_selection ++;
			if (rgb_selection > 4) {
				rgb_selection = 0;
			}
		}
		if (bottom_pressed) {
			switch (rgb_selection) { 
				case	0: {
							rgb_selection = 0;
							SetMode(system_clock_ms, 0);
							settings.Save();
							return;
						} break;
				case	1: {
							if (ring_or_duck) {
								ring_or_duck = false;
							} else {
								ring_or_duck = true;
							}
						} break;
				case	2: {
							if (ring_or_duck) {
								uint8_t r = settings.ring_color.r() + 4; if (r >= 0x80) r = 0;
								settings.ring_color = rgba(r,settings.ring_color.g()+0,settings.ring_color.b()+0);
							} else {
								uint8_t r = settings.bird_color.r() + 4; if (r >= 0x80) r = 0;
								settings.bird_color = rgba(r,settings.bird_color.g()+0,settings.bird_color.b()+0);
							}
						} break;
				case	3: {
							if (ring_or_duck) {
								uint8_t g = settings.ring_color.g() + 4; if (g >= 0x80) g = 0;
								settings.ring_color = rgba(settings.ring_color.r()+0,g,settings.ring_color.b()+0);
							} else {
								uint8_t g = settings.bird_color.g() + 4; if (g >= 0x80) g = 0;
								settings.bird_color = rgba(settings.bird_color.r()+0,g,settings.bird_color.b()+0);
							}
						} break;
				case	4: {
							if (ring_or_duck) {
								uint8_t b = settings.ring_color.b() + 4; if (b >= 0x80) b = 0;
								settings.ring_color = rgba(settings.ring_color.r()+0,settings.ring_color.g()+0,b);
							} else {
								uint8_t b = settings.bird_color.b() + 4; if (b >= 0x80) b = 0;
								settings.bird_color = rgba(settings.bird_color.r()+0,settings.bird_color.g()+0,b);
							}
						} break;
			}
		}
		DisplayRGB();
	}

	void DisplayRadioSettings() {
		sdd1306.SetAttr(0,0,0);
		sdd1306.SetAttr(0,1,0);
		sdd1306.SetAttr(0,2,0);
		sdd1306.SetAttr(0,3,0); 
		sdd1306.SetAttr(0,radio_settings_selection,1);
		
		sdd1306.PlaceCustomChar(0,0,0x7B);
		sdd1306.PlaceCustomChar(1,0,0x17A);
		sdd1306.PlaceCustomChar(2,0,0x17B);
		sdd1306.PlaceCustomChar(3,0,0x17C);
		sdd1306.PlaceCustomChar(4,0,0x17D);
		sdd1306.PlaceCustomChar(5,0,0x17E);
		sdd1306.PlaceCustomChar(6,0,0x17F);
		sdd1306.PlaceCustomChar(7,0,0x180);

		const char *menu[] = {
			"1       ",
			"2NAME   ",
			"3MESSAGE",
		};
		
		for (int32_t c=0; c<3; c++) {
			sdd1306.PlaceAsciiStr(0,c+1,menu[c]);
			if (radio_settings_selection == c+1) {
				sdd1306.SetAttr(0,c+1,1);
			} else {
				sdd1306.SetAttr(0,c+1,0);
			}
		}

		if (settings.radio_enabled) {
			sdd1306.PlaceCustomChar(1,1,0x16C);
			sdd1306.PlaceCustomChar(2,1,0x16D);
			sdd1306.PlaceCustomChar(3,1,0x16E);
			sdd1306.PlaceCustomChar(4,1,0x16F);
			sdd1306.PlaceCustomChar(5,1,0x170);
			sdd1306.PlaceCustomChar(6,1,0x171);
			sdd1306.PlaceCustomChar(7,1,0x172);
		} else {
			sdd1306.PlaceCustomChar(1,1,0x173);
			sdd1306.PlaceCustomChar(2,1,0x174);
			sdd1306.PlaceCustomChar(3,1,0x175);
			sdd1306.PlaceCustomChar(4,1,0x176);
			sdd1306.PlaceCustomChar(5,1,0x177);
			sdd1306.PlaceCustomChar(6,1,0x178);
			sdd1306.PlaceCustomChar(7,1,0x179);
		}

		sdd1306.Display();
	}

	void RadioSettingsHandler(bool top_pressed, bool bottom_pressed) {
		if (top_pressed) {
			radio_settings_selection ++;
			if (radio_settings_selection > 3) {
				radio_settings_selection = 0;
			}
		}
		if (bottom_pressed) {
			switch (radio_settings_selection) { 
				case	0: {
							radio_settings_selection = 0;
							SetMode(system_clock_ms, 0);
							settings.Save();
							return;
						} break;
				case	1: {
							if (settings.radio_enabled) {
								settings.radio_enabled = false;
							} else {
								settings.radio_enabled = true;
							}
						} break;
				case	2: {
							radio_settings_selection = 0;
							SetMode(system_clock_ms, 7);
							return;
						} break;
				case	3: {
							radio_settings_selection = 0;
							SetMode(system_clock_ms, 8);
							return;
						} break;
			}
		}
		DisplayRadioSettings();
	}

	void DisplayRadio() {
		
		sdd1306.SetAttr(0,1,0);
		sdd1306.SetAttr(0,2,0);
		sdd1306.SetAttr(0,3,0); 
		sdd1306.SetAttr(0,radio_selection+1,1);
		
		if (settings.radio_message >= 8) {
			settings.radio_message = 0;
		}

		if (settings.radio_color >= 11) {
			settings.radio_color = 0;
		}
		
		char str[9];
		memset(str,0,sizeof(str));
		memcpy(str,settings.radio_messages[settings.radio_message],8);
		sdd1306.PlaceAsciiStr(0,0,str);

		sdd1306.PlaceCustomChar(0,1,0x193);
		
		if (radio_selection == 0) {
			sdd1306.PlaceCustomChar(1,1,0x19B);
			sdd1306.PlaceCustomChar(2,1,0x19C);
			sdd1306.PlaceCustomChar(3,1,0x19D);
			sdd1306.PlaceCustomChar(4,1,0x19E);
			sdd1306.PlaceCustomChar(5,1,0x19F);
			sdd1306.PlaceCustomChar(6,1,0x1A0);
			sdd1306.PlaceCustomChar(7,1,0x1A1);
		} else {
			sdd1306.PlaceCustomChar(1,1,0x194);
			sdd1306.PlaceCustomChar(2,1,0x195);
			sdd1306.PlaceCustomChar(3,1,0x196);
			sdd1306.PlaceCustomChar(4,1,0x197);
			sdd1306.PlaceCustomChar(5,1,0x198);
			sdd1306.PlaceCustomChar(6,1,0x199);
			sdd1306.PlaceCustomChar(7,1,0x19A);
		}

		sdd1306.PlaceCustomChar(0,2,0x191);
		sprintf(str,"[%02d/%02d]",settings.radio_message + 1,8);
		sdd1306.PlaceAsciiStr(1,2,str);

		sdd1306.PlaceCustomChar(0,3,0x192);
		sprintf(str,"[%02d/%02d]",settings.radio_color + 1,11);
		sdd1306.PlaceAsciiStr(1,3,str);

		sdd1306.Display();
	}
	
	void RadioHandler(bool top_pressed, bool bottom_pressed) {
		if (top_pressed) {
			radio_selection ++;
			if (radio_selection > 2) {
				radio_selection = 0;
			}
		}
		if (bottom_pressed) {
			switch (radio_selection) { 
				case	0: {
							sx1280.SendMessage();
							radio_selection = 0;
							sdd1306.ClearAttr();
							SetMode(system_clock_ms, 0);
							settings.Save();
							return;
						} break;
				case	1: {
							settings.radio_message ++;
							if (settings.radio_message >= 8) {
								settings.radio_message = 0;
							}
						} break;
				case	2: {
							settings.radio_color ++;
							if (settings.radio_color >= 11) {
								settings.radio_color = 0;
							}
						} break;
			}
		}
		DisplayRadio();
	}

	void DisplayMessage() {
		static int32_t scroll_x = 0;
		if ((system_clock_ms - mode_start_time) < 50) {
			for (int32_t c=0; c<8; c++) {
				sdd1306.PlaceCustomChar(c,0,0x181+c);
			}

			for (int32_t c=0; c<8; c++) {
				sdd1306.PlaceCustomChar(c,2,0x280+c);
			}
			
			char str[9];
			memset(str,0,9);
			memcpy(str,settings.recv_radio_name,8);
			sdd1306.PlaceAsciiStr(0,3,str);

			sdd1306.SetAsciiScrollMessage(settings.recv_radio_message, scroll_x++);

			sdd1306.SetVerticalShift(0);
		}
		else if ((system_clock_ms - mode_start_time) < 50 + 640 + 50) {
			uint32_t ltime = (system_clock_ms - mode_start_time) - 50;
			static uint8_t bounce[] = {
				0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1e, 0x1d, 0x1d, 
				0x1c, 0x1b, 0x1a, 0x18, 0x17, 0x16, 0x14, 0x12, 
				0x10, 0x0e, 0x0c, 0x0a, 0x08, 0x05, 0x03, 0x01, 
				0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x07, 0x08, 
				0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x07, 0x07, 
				0x06, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x01, 
				0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 
				0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 
			};
			uint32_t y = ltime/10;
			if (y >= 64) y = 63;

			sdd1306.SetAsciiScrollMessage(settings.recv_radio_message, scroll_x++);

			sdd1306.Display();
			sdd1306.SetVerticalShift(bounce[y]);
		}
		else if ((system_clock_ms - mode_start_time) < 5000 + 50 + 640 + 50) {
			sdd1306.SetAsciiScrollMessage(settings.recv_radio_message, scroll_x++);
			sdd1306.Display();
		}
		else if ((system_clock_ms - mode_start_time) < 160 + 5000 + 50 + 640 + 50) {
			uint32_t ltime = (system_clock_ms - mode_start_time) - (5000 + 50 + 640 + 50);
			static int8_t ease[] = {
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 
				0x02, 0x02, 0x03, 0x03, 0x04, 0x05, 0x06, 0x07, 
				0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0e, 0x0f, 0x11, 
				0x12, 0x14, 0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f, 
				0x1f,
			};
			sdd1306.SetVerticalShift(-ease[ltime/5]);
			sdd1306.SetCenterFlip(ltime/5);
			sdd1306.Display();
		} else {
			sdd1306.SetAsciiScrollMessage(0, 0);
			mode = previous_mode;
			sdd1306.ClearAttr();
			DisplayStatus();
			sdd1306.SetVerticalShift(0);
			sdd1306.SetCenterFlip(0);
			sdd1306.Display();
		}
	}
	
	void DisplayNameSettings() {
		if ( radio_name_selection == 0 ) {
			sdd1306.SetAttr(0,0,1);
		} else {
			sdd1306.SetAttr(0,0,0);
		}
		
		sdd1306.PlaceCustomChar(0,0,0x7B);
		sdd1306.PlaceCustomChar(1,0,0x1A2);
		sdd1306.PlaceCustomChar(2,0,0x1A3);
		sdd1306.PlaceCustomChar(3,0,0x1A4);
		sdd1306.PlaceCustomChar(4,0,0x1A5);
		sdd1306.PlaceCustomChar(5,0,0x1A6);
		sdd1306.PlaceCustomChar(6,0,0x1A7);
		sdd1306.PlaceCustomChar(7,0,0x1A8);
		
		sdd1306.PlaceAsciiStr(0,1,"        ");
		
		char str[9];
		memset(str,0,sizeof(str));
		memcpy(str,settings.radio_name,8);
		sdd1306.PlaceAsciiStr(0,2,str);

		sdd1306.PlaceAsciiStr(0,3,"        ");
		
		if (radio_name_selection > 0) {
			sdd1306.PlaceCustomChar(radio_name_selection - 1,3,0x1B0);
		}

		sdd1306.Display();
	}

	void NameSettingsHandler(bool top_pressed, bool bottom_pressed) {
		if (top_pressed) {
			radio_name_selection ++;
			if (radio_name_selection >= 9) {
				radio_name_selection = 0;
			}
		}
		if (bottom_pressed) {
			switch (radio_name_selection) { 
				case	0: {
							radio_name_selection = 0;
							SetMode(system_clock_ms, 0);
							sdd1306.ClearAttr();
							settings.Save();
							return;
						} break;
				default: {
							if (settings.radio_name[radio_name_selection - 1] < 0x20 ||
								settings.radio_name[radio_name_selection - 1] > 0x60) {
								settings.radio_name[radio_name_selection - 1] = 0x20;
							}
							settings.radio_name[radio_name_selection - 1] ++;
							if (settings.radio_name[radio_name_selection - 1] >= 0x60) {
								settings.radio_name[radio_name_selection - 1] = 0x20;
							}
						} break;
			}
		}
		DisplayNameSettings();
	}

	void DisplayMessageSettings() {
		
		if ( radio_message_selection == 0 ) {
			sdd1306.SetAttr(0,0,1);
		} else {
			sdd1306.SetAttr(0,0,0);
		}

		if ( radio_message_selection == 1 ) {
			sdd1306.SetAttr(0,1,1);
		} else {
			sdd1306.SetAttr(0,1,0);
		}
		
		sdd1306.PlaceCustomChar(0,0,0x7B);
		sdd1306.PlaceCustomChar(1,0,0x1A9);
		sdd1306.PlaceCustomChar(2,0,0x1AA);
		sdd1306.PlaceCustomChar(3,0,0x1AB);
		sdd1306.PlaceCustomChar(4,0,0x1AC);
		sdd1306.PlaceCustomChar(5,0,0x1AD);
		sdd1306.PlaceCustomChar(6,0,0x1AE);
		sdd1306.PlaceCustomChar(7,0,0x1AF);
		
		char str[9];
		sdd1306.PlaceCustomChar(0,1,0x191);
		sprintf(str,"[%02d/%02d]",radio_message_current + 1,8);
		sdd1306.PlaceAsciiStr(1,1,str);
		
		memset(str,0,sizeof(str));
		memcpy(str,settings.radio_messages[radio_message_current],8);
		sdd1306.PlaceAsciiStr(0,2,str);

		sdd1306.PlaceAsciiStr(0,3,"        ");
		
		if (radio_message_selection > 0) {
			sdd1306.PlaceCustomChar(radio_message_selection - 2,3,0x1B0);
		}

		sdd1306.Display();
	}

	void MessageSettingsHandler(bool top_pressed, bool bottom_pressed) {
		if (top_pressed) {
			radio_message_selection ++;
			if (radio_message_selection >= 10) {
				radio_message_selection = 0;
			}
		}
		if (bottom_pressed) {
			switch (radio_message_selection) { 
				case	0: {
							radio_message_selection = 0;
							radio_message_current = 0;
							SetMode(system_clock_ms, 0);
							settings.Save();
							return;
						} break;
				case	1: {
							radio_message_current ++;
							if (radio_message_current >= 8) {
								radio_message_current = 0;
							}
						} break;
				default: {
							if (settings.radio_messages[radio_message_current][radio_message_selection - 1] < 0x20 ||
								settings.radio_messages[radio_message_current][radio_message_selection - 1] > 0x60) {
								settings.radio_messages[radio_message_current][radio_message_selection - 1] = 0x20;
							}
							settings.radio_messages[radio_message_current][radio_message_selection - 2] ++;
							if (settings.radio_messages[radio_message_current][radio_message_selection - 2] >= 0x40) {
								settings.radio_messages[radio_message_current][radio_message_selection - 2] = 0x20;
							}
						} break;
			}
		}
		DisplayMessageSettings();
	}
	
	void DisplayTest() {
		if (sdd1306.DevicePresent()) {
			sdd1306.PlaceAsciiStr(0,0,"        ");
			sdd1306.PlaceAsciiStr(0,1,"        ");
			sdd1306.PlaceAsciiStr(0,2,"        ");
			sdd1306.PlaceAsciiStr(0,3,"        ");
			for (int32_t c=2; c<7; c++) {
				sdd1306.PlaceCustomChar(c,0,0x1B1+c-2);
			}
			for (int32_t c=2; c<7; c++) {
				sdd1306.PlaceCustomChar(c,1,0x1B6+c-2);
			}
			for (int32_t c=1; c<7; c++) {
				sdd1306.PlaceCustomChar(c,2,0x1BB+c-1);
			}
			for (int32_t c=1; c<7; c++) {
				sdd1306.PlaceCustomChar(c,3,0x1C1+c-1);
			}
			if (sdd1306.DevicePresent()) {
				sdd1306.PlaceCustomChar(7,0,0x7C);
			} else {
				sdd1306.PlaceCustomChar(7,0,0x7B);
			}
			if (sx1280.DevicePresent()) {
				sdd1306.PlaceCustomChar(7,1,0x7C);
			} else {
				sdd1306.PlaceCustomChar(7,1,0x7B);
			}
			bool bq24295_fault = false;
			if (bq24295.DevicePresent()) {
				if (bq24295.IsInFaultState()) {
					bq24295_fault = true;
					sdd1306.PlaceCustomChar(7,2,0x01);
				} else {
					sdd1306.PlaceCustomChar(7,2,0x7C);
				}
			} else {
				sdd1306.PlaceCustomChar(7,2,0x7B);
			}
			if (ft25h16s.DevicePresent()) {
				sdd1306.PlaceCustomChar(7,3,0x7C);
			} else {
				sdd1306.PlaceCustomChar(7,3,0x7B);
			}
			
			sdd1306.Display();
			delay(2000);
			if (bq24295_fault) {
				char str[9];
				uint8_t s = bq24295.FaultState();
				sprintf(str,"%c%c%c%c%c%c%c%c",
					(s&0x01)?'1':'0',
					(s&0x02)?'1':'0',
					(s&0x04)?'1':'0',
					(s&0x08)?'1':'0',
					(s&0x10)?'1':'0',
					(s&0x20)?'1':'0',
					(s&0x40)?'1':'0',
					(s&0x80)?'1':'0');
				sdd1306.PlaceAsciiStr(0,2,str);
				sdd1306.Display();
				delay(2000);
			}
		}
	}
	
	void DisplayVersion() {

#include "build_number.h"
		(void) build_number_len;

		#define COMPUTE_BUILD_YEAR \
			( \
				(__DATE__[ 7] - '0') * 1000 + \
				(__DATE__[ 8] - '0') *  100 + \
				(__DATE__[ 9] - '0') *   10 + \
				(__DATE__[10] - '0') \
			)


		#define COMPUTE_BUILD_DAY \
			( \
				((__DATE__[4] >= '0') ? (__DATE__[4] - '0') * 10 : 0) + \
				(__DATE__[5] - '0') \
			)

		#define BUILD_MONTH_IS_JAN (__DATE__[0] == 'J' && __DATE__[1] == 'a' && __DATE__[2] == 'n')
		#define BUILD_MONTH_IS_FEB (__DATE__[0] == 'F')
		#define BUILD_MONTH_IS_MAR (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'r')
		#define BUILD_MONTH_IS_APR (__DATE__[0] == 'A' && __DATE__[1] == 'p')
		#define BUILD_MONTH_IS_MAY (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'y')
		#define BUILD_MONTH_IS_JUN (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'n')
		#define BUILD_MONTH_IS_JUL (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'l')
		#define BUILD_MONTH_IS_AUG (__DATE__[0] == 'A' && __DATE__[1] == 'u')
		#define BUILD_MONTH_IS_SEP (__DATE__[0] == 'S')
		#define BUILD_MONTH_IS_OCT (__DATE__[0] == 'O')
		#define BUILD_MONTH_IS_NOV (__DATE__[0] == 'N')
		#define BUILD_MONTH_IS_DEC (__DATE__[0] == 'D')


		#define COMPUTE_BUILD_MONTH \
			( \
				(BUILD_MONTH_IS_JAN) ?  1 : \
				(BUILD_MONTH_IS_FEB) ?  2 : \
				(BUILD_MONTH_IS_MAR) ?  3 : \
				(BUILD_MONTH_IS_APR) ?  4 : \
				(BUILD_MONTH_IS_MAY) ?  5 : \
				(BUILD_MONTH_IS_JUN) ?  6 : \
				(BUILD_MONTH_IS_JUL) ?  7 : \
				(BUILD_MONTH_IS_AUG) ?  8 : \
				(BUILD_MONTH_IS_SEP) ?  9 : \
				(BUILD_MONTH_IS_OCT) ? 10 : \
				(BUILD_MONTH_IS_NOV) ? 11 : \
				(BUILD_MONTH_IS_DEC) ? 12 : \
				/* error default */  99 \
			)

		#define BUILD_DATE_IS_BAD (__DATE__[0] == '?')

		#define BUILD_YEAR  ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_YEAR)
		#define BUILD_MONTH ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_MONTH)
		#define BUILD_DAY   ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_DAY)
			
		char str[16];
		sdd1306.PlaceAsciiStr(0,0,"        ");
		sprintf(str,"%02d%02d%04d",
			BUILD_MONTH,
			BUILD_DAY,
			BUILD_YEAR);
		sdd1306.PlaceAsciiStr(0,1,str);
		sdd1306.PlaceAsciiStr(0,2,__TIME__);
		
		strcpy(str, "build   ");
		str[6] = build_number[0];
		str[7] = build_number[1];
		sdd1306.PlaceAsciiStr(0,3,str);

		sdd1306.Display();
		delay(2000);
	}
	
	void DisplayStats() {
		if ( stats_menu_selection == 0 ) {
			sdd1306.SetAttr(0,0,1);
		} else {
			sdd1306.SetAttr(0,0,0);
		}

		if ( stats_menu_selection == 1 ) {
			sdd1306.SetAttr(0,1,1);
		} else {
			sdd1306.SetAttr(0,1,0);
		}
		
		sdd1306.PlaceCustomChar(0,0,0x7B);
		sdd1306.PlaceCustomChar(1,0,0x1C7);
		sdd1306.PlaceCustomChar(2,0,0x1C8);
		sdd1306.PlaceCustomChar(3,0,0x1C9);
		sdd1306.PlaceCustomChar(4,0,0x1CA);
		sdd1306.PlaceCustomChar(5,0,0x1CB);
		sdd1306.PlaceCustomChar(6,0,0x1CC);
		sdd1306.PlaceCustomChar(7,0,0x1CD);
		
		
		char str[9];
		sdd1306.PlaceCustomChar(0,1,0x67);
		sprintf(str,"[%02d/%02d]",stats_select_current + 1,8);
		sdd1306.PlaceAsciiStr(1,1,str);

		switch(stats_select_current) {
			case	0: {
						sdd1306.PlaceAsciiStr(0,2,"PRG CHNG");
						if (settings.program_change_count > 99999999) {
							sdd1306.PlaceAsciiStr(0,3,"99999999");
						} else {
							sprintf(str,"%08d",settings.program_change_count);
							sdd1306.PlaceAsciiStr(0,3,str);
						}
					} break;
			case	1: {
						sdd1306.PlaceAsciiStr(0,2,"BRT CHNG");
						if (settings.brightness_change_count > 99999999) {
							sdd1306.PlaceAsciiStr(0,3,"99999999");
						} else {
							sprintf(str,"%08d",settings.brightness_change_count);
							sdd1306.PlaceAsciiStr(0,3,str);
						}
					} break;
			case	2: {
						sdd1306.PlaceAsciiStr(0,2,"RECV MSG");
						if (settings.recv_message_count > 99999999) {
							sdd1306.PlaceAsciiStr(0,3,"99999999");
						} else {
							sprintf(str,"%08d",settings.recv_message_count);
							sdd1306.PlaceAsciiStr(0,3,str);
						}
					} break;
			case	3: {
						sdd1306.PlaceAsciiStr(0,2,"SENT MSG");
						if (settings.sent_message_count > 99999999) {
							sdd1306.PlaceAsciiStr(0,3,"99999999");
						} else {
							sprintf(str,"%08d",settings.sent_message_count);
							sdd1306.PlaceAsciiStr(0,3,str);
						}
					} break;
			case	4: {
						sdd1306.PlaceAsciiStr(0,2,"TIME ON ");
						uint32_t hours = settings.total_runtime / 1000 / 60 / 60;
						uint32_t minutes = (settings.total_runtime / 1000 / 60) % 60;
						uint32_t seconds = (settings.total_runtime / 1000) % 60;
						if (hours > 99) {
							sdd1306.PlaceAsciiStr(0,3,"99:99:99");
						} else {
							sprintf(str,"%02d:%02d:%02d",hours, minutes, seconds);
							sdd1306.PlaceAsciiStr(0,3,str);
						}
					} break;
		}
		sdd1306.Display();
	}

	void StatsHandler(bool top_pressed, bool bottom_pressed) {
		if (top_pressed) {
			stats_menu_selection ++;
			if (stats_menu_selection >= 2) {
				stats_menu_selection = 0;
			}
		}
		if (bottom_pressed) {
			switch (stats_menu_selection) { 
				case	0: {
							stats_select_current = 0;
							stats_menu_selection = 0;
							sdd1306.ClearAttr();
							SetMode(system_clock_ms, 0);
							settings.Save();
							return;
						} break;
				case	1: {
							stats_select_current ++;
							if (stats_select_current >= 5) {
								stats_select_current = 0;
							}
						} break;
			}
		}
		DisplayStats();
	}
	
	void DisplayHistory() {
		if ( history_menu_selection == 0 ) {
			sdd1306.SetAttr(0,0,1);
		} else {
			sdd1306.SetAttr(0,0,0);
		}

		if ( history_menu_selection == 1 ) {
			sdd1306.SetAttr(0,1,1);
		} else {
			sdd1306.SetAttr(0,1,0);
		}
		
		sdd1306.PlaceCustomChar(0,0,0x7B);
		sdd1306.PlaceCustomChar(1,0,0x1CE);
		sdd1306.PlaceCustomChar(2,0,0x1CF);
		sdd1306.PlaceCustomChar(3,0,0x1D0);
		sdd1306.PlaceCustomChar(4,0,0x1D1);
		sdd1306.PlaceCustomChar(5,0,0x1D2);
		sdd1306.PlaceCustomChar(6,0,0x1D3);
		sdd1306.PlaceCustomChar(7,0,0x1D4);

		sdd1306.PlaceCustomChar(0,1,0x191);
		char str[9];
		sprintf(str,"%03d/%03d", history_select_current + 1, settings.GetMessageCount());
		sdd1306.PlaceAsciiStr(1,1,str);
		
		uint8_t msg[32];
		if (settings.GetMessage(ft25h16s, history_select_current, &msg[0])) {
			settings.recv_radio_color = msg[7];
			memset(str,0,9);
			strncpy(str,(const char *)&msg[8],8);
			sdd1306.PlaceAsciiStr(0,2,str);
			memset(str,0,9);
			strncpy(str,(const char *)&msg[16],8);
			sdd1306.PlaceAsciiStr(0,3,str);
		} else {
			sdd1306.PlaceAsciiStr(0,2,"        ");
			sdd1306.PlaceAsciiStr(0,3,"        ");
		}
		sdd1306.Display();
	}

	void HistoryHandler(bool top_pressed, bool bottom_pressed) {
		if (top_pressed && settings.GetMessageCount()) {
			history_menu_selection ++;
			if (history_menu_selection >= 2) {
				history_menu_selection = 0;
			}
		}
		if (bottom_pressed) {
			switch (history_menu_selection) { 
				case	0: {
							history_select_current = 0;
							history_menu_selection = 0;
							sdd1306.ClearAttr();
							SetMode(system_clock_ms, 0);
							settings.Save();
							return;
						} break;
				case	1: {
							history_select_current ++;
							if (history_select_current >= settings.GetMessageCount()) {
								history_select_current = 0;
							}
						} break;
			}
		}
		DisplayHistory();
	}

	void Display() {
		switch (mode) {
			case	0:
					DisplayStatus();
					break;
			case	1:
					switch(interlude) {
						case 	0:
								DisplayNow();
								break;
						case 	1:
								DisplayDog();
								break;
					}
					break;
			case	2:
					DisplaySettings();
					break;
			case	3:
					DisplayRGB();
					break;
			case	4:
					DisplayRadioSettings();
					break;
			case	5:
					DisplayRadio();
					break;
			case	6:
					DisplayMessage();
					break;
			case	7:
					DisplayNameSettings();
					break;
			case	8:
					DisplayMessageSettings();
					break;
			case	9:
					DisplayStats();
					break;
			case	10:
					DisplayHistory();
					break;
		}
	}
};

class Effects {

	EEPROM &settings;
	Random &random;
	LEDs &leds;
	SPI &spi;
	SDD1306 &sdd1306;
	UI &ui;

	uint32_t post_clock_ms;
	bool past_post_time;
	bool break_on_message;
	
public:
	
	Effects(EEPROM &_settings, 
			Random &_random, 
			LEDs &_leds, 
			SPI &_spi, 
			SDD1306 &_sdd1306,
			UI &_ui):

			settings(_settings),
			random(_random),
			leds(_leds),
			spi(_spi),
			sdd1306(_sdd1306),
			ui(_ui)	{
		post_clock_ms = system_clock_ms + 10;
		past_post_time = true;
		break_on_message = false;
	}	
	
	void RunForever() {
		while (1) {
			if (past_post_time) {
				past_post_time = false;
				if (ui.Mode() == 6) {
					message_ring();
				} else if ( ui.Mode() == 3 || ui.Mode() == 5 || ui.Mode() == 10) {
					color_ring();
				} else switch(settings.program_curr) {
					case	0:
							color_ring();
							break;
					case	1:	
							fade_ring();
							break;
					case	2:
							rgb_walker();
							break;
					case	3:
							rgb_glow();
							break;
					case	4:
							rgb_tracer();
							break;
					case	5: 
							ring_tracer();
							break;
					case	6:
							light_tracer();
							break;
					case	7: 
							ring_bar_rotate();
							break;
					case	8: 
							ring_bar_move();
							break;
					case	9:
							sparkle();
							break;
					case	10:
							lightning();
							break;
					case	11:
							lightning_crazy();
							break;
					case 	12:
							rgb_vertical_wall();
							break;
					case 	13:
							rgb_horizontal_wall();
							break;
					case	14:
							shine_vertical();
							break;
					case	15:
							shine_horizontal();
							break;	
					case 	16:
							heartbeat();
							break;
					case 	17:
							brilliance();
							break;
					case    18:
							tingling();
							break;
					case    19:
							twinkle();
							break;
					case	20:
							simple_change_ring();
							break;
					case	21:
							simple_change_bird();
							break;
					case	22:
							simple_random();
							break;
					case	23:
							diagonal_wipe();
							break;
					case	24:
							shimmer_outside();
							break;
					case	25:
							shimmer_inside();
							break;
					case	26:
							red();
							break;
					default:
							color_ring();
							break;
				}
			}
            Chip_WWDT_Feed(LPC_WWDT);
			__WFI();
		}
	}

	void CheckPostTime() {
		if (system_clock_ms > post_clock_ms) {
			past_post_time = true;
			post_clock_ms = system_clock_ms + 250; // at minimum update every 0.25s
		}
	}
	
private:

	bool break_effect() {
		static uint32_t old_program_curr = 0;
		static uint32_t old_mode = 0;
		if (settings.program_curr != old_program_curr) {
			old_program_curr = settings.program_curr;
			return true;
		}
		if ((ui.Mode() != old_mode) && ui.Mode() != 1 && old_mode != 1) {
			old_mode = ui.Mode();
			return true;
		}
		return false;
	}
	
	bool post_frame(uint32_t ms) {
		post_clock_ms = system_clock_ms + ms;

		if (sdd1306.DevicePresent()) {
			ui.Display();
		}

		for (;;) {
			if (past_post_time) {
				past_post_time = false;
				return break_effect();
			}
			if (break_effect()) {
				return true;
			}
            Chip_WWDT_Feed(LPC_WWDT);
			__WFI();
		}
		return false;
	}
	
	void message_ring() {
		int32_t rgb_walk = 0;
		int32_t switch_dir = 4;
		for (; ;) {
			for (uint32_t d = 0; d < 8; d++) {
				if (settings.recv_radio_color <= 10) {
					leds.set_ring(d, ((rgba(radio_colors[settings.recv_radio_color]).r())*rgb_walk)/256,
									 ((rgba(radio_colors[settings.recv_radio_color]).g())*rgb_walk)/256,
									 ((rgba(radio_colors[settings.recv_radio_color]).b())*rgb_walk)/256);
				}
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}
			
			rgb_walk += switch_dir;
			if (rgb_walk >= 256) {
				rgb_walk = 255;
				switch_dir *= -1;
			}
			if (rgb_walk < 0) {
				rgb_walk = 0;
				switch_dir *= -1;
			}

			if (post_frame(4)) {
				return;
			}
		}
	}

	void color_ring() {
		for (;;) {
			if (ui.Mode() == 10) {
				for (uint32_t d = 0; d < 8; d++) {
					if (settings.recv_radio_color <= 10) {
						leds.set_ring(d, rgba(radio_colors[settings.recv_radio_color]));
					}
				}
			} else if (ui.Mode() == 5 ) {
				for (uint32_t d = 0; d < 8; d++) {
					leds.set_ring(d, rgba(radio_colors[settings.radio_color]));
				}
			} else {
				for (uint32_t d = 0; d < 8; d++) {
					leds.set_ring(d, settings.ring_color);
				}
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(5)) {
				return;
			}
		}
	}

	void fade_ring() {
		for (;;) {

			rgba color;
			const rgba &rc = settings.ring_color;
			color = rgba(max(rc.ri()-0x40,0L), max(rc.gi()-0x40,0L), max(rc.bi()-0x40,0L));
			leds.set_ring(0, color);
			color = rgba(max(rc.ri()-0x3A,0L), max(rc.gi()-0x3A,0L), max(rc.bi()-0x3A,0L));
			leds.set_ring(1, color);
			leds.set_ring(7, color);
			color = rgba(max(rc.ri()-0x28,0L), max(rc.gi()-0x28,0L), max(rc.bi()-0x28,0L));
			leds.set_ring(2, color);
			leds.set_ring(6, color);
			color = rgba(max(rc.ri()-0x20,0L), max(rc.gi()-0x20,0L), max(rc.bi()-0x20,0L));
			leds.set_ring(3, color);
			leds.set_ring(5, color);
			color = rgba(max(rc.ri()-0x00,0L), max(rc.gi()-0x00,0L), max(rc.bi()-0x00,0L));
			leds.set_ring(4, color);

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}
			
			if (post_frame(5)) {
				return;
			}
		}
	}

	void rgb_walker() {

		static uint8_t work_buffer[0x80] = { 0 };

		for (uint32_t c = 0; c < 0x40; c++) {
			work_buffer[c] = c;
		}
		for (uint32_t c = 0; c < 0x40; c++) {
			work_buffer[c+0x40] = 0x40 - c;
		}

		uint32_t walk = 0;
		uint32_t rgb_walk = 0;
		for (;;) {
			rgba color = rgba::hsvToRgb(rgb_walk/3, 255, 255);
			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, min(0x80,(work_buffer[(((0x80/8)*d) + walk)&0x7F] * ((color.r())&0xFF)) >> 8),
						 	     min(0x80,(work_buffer[(((0x80/8)*d) + walk)&0x7F] * ((color.g())&0xFF)) >> 8),
						 		 min(0x80,(work_buffer[(((0x80/8)*d) + walk)&0x7F] * ((color.b())&0xFF)) >> 8));
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			walk ++;
			walk &= 0x7F;

			rgb_walk ++;
			if (rgb_walk >= 360*3) {
				rgb_walk = 0;
			}

			if (post_frame(5)) {
				return;
			}
		}
	}

	void rgb_glow() {
		uint32_t rgb_walk = 0;
		for (;;) {

			rgba color = rgba::hsvToRgb(rgb_walk, 255, 255);
			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring_synced(d, (color / 4UL));
			}
			
			rgb_walk ++;
			if (rgb_walk >= 360) {
				rgb_walk = 0;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(50)) {
				return;
			}
		}
	}

	void rgb_tracer() {
		uint32_t rgb_walk = 0;
		uint32_t walk = 0;
		int32_t switch_dir = 1;
		uint32_t switch_counter = 0;
		for (;;) {
			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d,0,0,0);
			}

			rgba color = rgba::hsvToRgb(rgb_walk/3, 255, 255);
			leds.set_ring_synced(walk&0x7, (color / 4UL));

			walk += switch_dir;

			rgb_walk += 7;
			if (rgb_walk >= 360*3) {
				rgb_walk = 0;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			switch_counter ++;
			if (switch_counter > 64 && random.get(0,2)) {
				switch_dir *= -1;
				switch_counter = 0;
				walk += switch_dir;
				walk += switch_dir;
			}

			if (post_frame(50)) {
				return;
			}
		}
	}

	void light_tracer() {
		uint32_t walk = 0;

		rgba gradient[8];
		const rgba &rc = settings.ring_color;
		gradient[7] = rgba(max(rc.r()-0x40,0x00), max(rc.g()-0x40,0x00), max(rc.b()-0x40,0x00));
		gradient[6] = rgba(max(rc.r()-0x40,0x00), max(rc.g()-0x40,0x00), max(rc.b()-0x40,0x00));
		gradient[5] = rgba(max(rc.r()-0x30,0x00), max(rc.g()-0x30,0x00), max(rc.b()-0x30,0x00));
		gradient[4] = rgba(max(rc.r()-0x18,0x00), max(rc.g()-0x18,0x00), max(rc.b()-0x18,0x00));
		gradient[3] = rgba(max(rc.r()-0x00,0x00), max(rc.g()-0x00,0x00), max(rc.b()-0x00,0x00));
		gradient[2] = rgba(max(rc.r()-0x00,0x10), max(rc.g()-0x00,0x00), max(rc.b()-0x00,0x20));
		gradient[1] = rgba(max(rc.r()-0x00,0x30), max(rc.g()-0x00,0x30), max(rc.b()-0x00,0x30));
		gradient[0] = rgba(max(rc.r()-0x00,0x40), max(rc.g()-0x00,0x40), max(rc.b()-0x00,0x40));

		for (;;) {
			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring((walk+d)&0x7, gradient[d]);
			}

			walk--;

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(100)) {
				return;
			}
		}
	}


	void ring_tracer() {
		uint32_t walk = 0;
		int32_t switch_dir = 1;
		uint32_t switch_counter = 0;
		for (;;) {
			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d,0,0,0);
			}

			leds.set_ring_synced((walk+0)&0x7, settings.ring_color);
			leds.set_ring_synced((walk+1)&0x7, settings.ring_color);
			leds.set_ring_synced((walk+2)&0x7, settings.ring_color);

			walk += switch_dir;

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			switch_counter ++;
			if (switch_counter > 64 && random.get(0,2)) {
				switch_dir *= -1;
				switch_counter = 0;
				walk += switch_dir;
				walk += switch_dir;
			}

			if (post_frame(50)) {
				return;
			}
		}
	}

	void ring_bar_rotate() {
		uint32_t rgb_walk = 0;
		uint32_t walk = 0;
		int32_t switch_dir = 1;
		uint32_t switch_counter = 0;
		for (;;) {
			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d,0,0,0);
			}

			leds.set_ring_synced((walk+0)&0x7, settings.ring_color);
			leds.set_ring_synced((walk+4)&0x7, settings.ring_color);

			walk += switch_dir;

			rgb_walk += 7;
			if (rgb_walk >= 360*3) {
				rgb_walk = 0;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			switch_counter ++;
			if (switch_counter > 64 && random.get(0,2)) {
				switch_dir *= -1;
				switch_counter = 0;
				walk += switch_dir;
				walk += switch_dir;
			}

			if (post_frame(75)) {
				return;
			}
		}
	}

	void ring_bar_move() {
		uint32_t rgb_walk = 0;
		uint32_t walk = 0;
		int32_t switch_dir = 1;
		uint32_t switch_counter = 0;

		static const int8_t indecies0[] = {
			-1,
			-1,
			-1,
			-1,
			-1,
			-1,
			0,
			1,
			2,
			3,
			4
			-1,
			-1,
			-1,
			-1,
			-1,
		};

		static const int8_t indecies1[] = {
			-1,
			-1,
			-1,
			-1,
			-1,
			-1,
			0,
			7,
			6,
			5,
			4,
			-1,
			-1,
			-1,
			-1,
			-1,
		};

		for (;;) {
			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d,0,0,0);
			}

			if (indecies0[(walk)%15] >=0 ) {
				leds.set_ring_synced(indecies0[(walk)%15], settings.ring_color);	
			}
			if (indecies1[(walk)%15] >=0 ) {
				leds.set_ring_synced(indecies1[(walk)%15], settings.ring_color);
			}

			walk += switch_dir;

			rgb_walk += 7;
			if (rgb_walk >= 360*3) {
				rgb_walk = 0;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			switch_counter ++;
			if (switch_counter > 64 && random.get(0,2)) {
				switch_dir *= -1;
				switch_counter = 0;
				walk += switch_dir;
				walk += switch_dir;
			}

			if (post_frame(50)) {
				return;
			}
		}
	}

	void rgb_vertical_wall() {
		uint32_t rgb_walk = 0;
		for (;;) {
			rgba color;
			color = rgba::hsvToRgb(((rgb_walk+  0)/3)%360, 255, 255);
			leds.set_ring_synced(0, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+ 30)/3)%360, 255, 255);
			leds.set_ring_synced(1, (color / 4UL));
			leds.set_ring_synced(7, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+120)/3)%360, 255, 255);
			leds.set_ring_synced(2, (color / 4UL));
			leds.set_ring_synced(6, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+210)/3)%360, 255, 255);
			leds.set_ring_synced(3, (color / 4UL));
			leds.set_ring_synced(5, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+230)/3)%360, 255, 255);
			leds.set_ring_synced(4, (color / 4UL));

			rgb_walk += 7;
			if (rgb_walk >= 360*3) {
				rgb_walk = 0;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(40)) {
				return;
			}
		}
	}

	void shine_vertical() {
		uint32_t rgb_walk = 0;
		rgba gradient[256];
		for (uint32_t c = 0; c < 128; c++) {
			uint32_t r = max(settings.ring_color.ru(),c/2);
			uint32_t g = max(settings.ring_color.gu(),c/2);
			uint32_t b = max(settings.ring_color.bu(),c/2);
			gradient[c] = rgba(r, g, b);
		}
		for (uint32_t c = 0; c < 128; c++) {
			uint32_t r = max(settings.ring_color.ru(),(128-c)/2);
			uint32_t g = max(settings.ring_color.gu(),(128-c)/2);
			uint32_t b = max(settings.ring_color.bu(),(128-c)/2);
			gradient[c+128] = rgba(r, g, b);
		}

		for (;;) {
			rgba color;
			color = gradient[((rgb_walk+ 0))%256];
			leds.set_ring_synced(0, color);
			color = gradient[((rgb_walk+10))%256];
			leds.set_ring_synced(1, color);
			leds.set_ring_synced(7, color);
			color = gradient[((rgb_walk+40))%256];
			leds.set_ring_synced(2, color);
			leds.set_ring_synced(6, color);
			color = gradient[((rgb_walk+70))%256];
			leds.set_ring_synced(3, color);
			leds.set_ring_synced(5, color);
			color = gradient[((rgb_walk+80))%256];
			leds.set_ring_synced(4, color);

			rgb_walk += 7;
			if (rgb_walk >= 256) {
				rgb_walk = 0;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(80)) {
				return;
			}
		}
	}

	void shine_horizontal() {
		int32_t rgb_walk = 0;
		int32_t switch_dir = 1;

		rgba gradient[256];
		for (uint32_t c = 0; c < 128; c++) {
			uint32_t r = max(settings.ring_color.ru(),c/2);
			uint32_t g = max(settings.ring_color.gu(),c/2);
			uint32_t b = max(settings.ring_color.bu(),c/2);
			gradient[c] = rgba(r, g, b);
		}
		for (uint32_t c = 0; c < 128; c++) {
			uint32_t r = max(settings.ring_color.ru(),(128-c)/2);
			uint32_t g = max(settings.ring_color.gu(),(128-c)/2);
			uint32_t b = max(settings.ring_color.bu(),(128-c)/2);
			gradient[c+128] = rgba(r, g, b);
		}

		for (;;) {
			rgba color;
			color = gradient[((rgb_walk+ 0))%256];
			leds.set_ring_synced(6, color);
			color = gradient[((rgb_walk+10))%256];
			leds.set_ring_synced(7, color);
			leds.set_ring_synced(5, color);
			color = gradient[((rgb_walk+40))%256];
			leds.set_ring_synced(0, color);
			leds.set_ring_synced(4, color);
			color = gradient[((rgb_walk+70))%256];
			leds.set_ring_synced(1, color);
			leds.set_ring_synced(3, color);
			color = gradient[((rgb_walk+80))%256];
			leds.set_ring_synced(2, color);

			rgb_walk += 7*switch_dir;
			if (rgb_walk >= 256) {
				rgb_walk = 255;
				switch_dir *= -1;
			}
			if (rgb_walk < 0) {
				rgb_walk = 0;
				switch_dir *= -1;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(80)) {
				return;
			}
		}
	}

	void rgb_horizontal_wall() {
		uint32_t rgb_walk = 0;
		for (;;) {

			rgba color;
			color = rgba::hsvToRgb(((rgb_walk+  0)/3)%360, 255, 255);
			leds.set_ring_synced(6, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+ 30)/3)%360, 255, 255);
			leds.set_ring_synced(7, (color / 4UL));
			leds.set_ring_synced(5, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+120)/3)%360, 255, 255);
			leds.set_ring_synced(0, (color / 4UL));
			leds.set_ring_synced(4, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+210)/3)%360, 255, 255);
			leds.set_ring_synced(1, (color / 4UL));
			leds.set_ring_synced(3, (color / 4UL));
			color = rgba::hsvToRgb(((rgb_walk+230)/3)%360, 255, 255);
			leds.set_ring_synced(2, (color / 4UL));

			rgb_walk += 7;
			if (rgb_walk >= 360*3) {
				rgb_walk = 0;
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(40)) {
				return;
			}
		}
	}

	void lightning() {
		for (;;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, 0,0,0);
			}

			int index = random.get(0,128);
			leds.set_ring_all(index,0x40,0x40,0x40);

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(10)) {
				return;
			}
		}
	}

	void sparkle() {
		for (;;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, 0,0,0);
			}

			int index = random.get(0,16);
			leds.set_ring_all(index,random.get(0x00,0x40),random.get(0x00,0x40),random.get(0,0x40));

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(50)) {
				return;
			}
		}
	}

	void lightning_crazy() {
		for (;;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, 0,0,0);
			}

			int index = random.get(0,16);
			leds.set_ring_all(index,0x40,0x40,0x40);

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(10)) {
				return;
			}
		}
	}

	void heartbeat() {
		int32_t rgb_walk = 0;
		int32_t switch_dir = 1;
		for (; ;) {
			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, settings.ring_color);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, ((settings.bird_color.r())*rgb_walk)/256,
						 		 ((settings.bird_color.g())*rgb_walk)/256,
						 		 ((settings.bird_color.b())*rgb_walk)/256);
			}

			rgb_walk += switch_dir;
			if (rgb_walk >= 256) {
				rgb_walk = 255;
				switch_dir *= -1;
			}
			if (rgb_walk < 0) {
				rgb_walk = 0;
				switch_dir *= -1;
			}

			if (post_frame(8)) {
				return;
			}
		}
	}

	void brilliance() {
		int32_t current_wait = 0;
		int32_t wait_time = 0;
		int32_t rgb_walk = 0;
		int32_t switch_dir = 1;
		for (; ;) {
			rgba gradient[256];
			for (int32_t c = 0; c < 112; c++) {
				uint32_t r = settings.bird_color.r();
				uint32_t g = settings.bird_color.g();
				uint32_t b = settings.bird_color.b();
				gradient[c] = rgba(r, g, b);
			}
			for (uint32_t c = 0; c < 16; c++) {
				uint32_t r = max(settings.bird_color.ru(),c*8);
				uint32_t g = max(settings.bird_color.gu(),c*8);
				uint32_t b = max(settings.bird_color.bu(),c*8);
				gradient[c+112] = rgba(r, g, b);
			}
			for (uint32_t c = 0; c < 16; c++) {
				uint32_t r = max(settings.bird_color.ru(),(16-c)*8);
				uint32_t g = max(settings.bird_color.gu(),(16-c)*8);
				uint32_t b = max(settings.bird_color.bu(),(16-c)*8);
				gradient[c+128] = rgba(r, g, b);
			}
			for (int32_t c = 0; c < 112; c++) {
				uint32_t r = settings.bird_color.r();
				uint32_t g = settings.bird_color.g();
				uint32_t b = settings.bird_color.b();
				gradient[c+144] = rgba(r, g, b);
			}


			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, settings.ring_color);
			}

			for (uint32_t d = 0; d < 4; d++) {
				rgba color = gradient[((rgb_walk+ 0))%256];
				leds.set_bird(d, color);
			}

			rgb_walk += switch_dir;
			if (rgb_walk >= 256) {
				current_wait++;
				if (current_wait > wait_time) {
					wait_time = random.get(0,2000);
					rgb_walk = 0;
					current_wait = 0;
				} else {
					rgb_walk = 255;
				}
			}

			if (post_frame(10)) {
				return;
			}
		}
	}

	void tingling() {
		#define NUM_TINGLES 16
		struct tingle {
			bool active;
			int32_t wait;
			int32_t index;
			int32_t progress;
			bool lightordark;
		} tingles[NUM_TINGLES];

		memset(tingles, 0, sizeof(tingles));

		for (; ;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, settings.ring_color);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			for (int32_t c = 0 ; c < NUM_TINGLES; c++) {
				if (tingles[c].active == 0) {
					tingles[c].wait = random.get(0,25);
					for (;;) {
						bool done = true;
						tingles[c].index = random.get(0,16);
						for (int32_t d = 0 ; d < NUM_TINGLES; d++) {
							if( d != c && 
								tingles[c].active && 
								tingles[c].index == tingles[d].index) {
								done = false;
								break;
							}
						}
						if (done) {
							break;
						}
					}
					tingles[c].index = random.get(0,16);
					tingles[c].progress = 0;
					tingles[c].lightordark = random.get(0,2);
					tingles[c].active = 1;
				} else if (tingles[c].progress >= 16) {
					tingles[c].active = 0;
				} else if (tingles[c].wait > 0) {
					tingles[c].wait --;
				} else {
					int32_t r = 0,g = 0,b = 0;
					int32_t progress = tingles[c].progress;
					if (progress > 8) {
						progress -= 8;
						progress = 8 - progress;
					}
					if (tingles[c].lightordark) {
						r = max(settings.ring_color.ri(),progress*int32_t(8));
						g = max(settings.ring_color.gi(),progress*int32_t(8));
						b = max(settings.ring_color.bi(),progress*int32_t(8));					
					} else {
						r = (settings.ring_color.ri())-progress*int32_t(8);
						g = (settings.ring_color.gi())-progress*int32_t(8);
						b = (settings.ring_color.bi())-progress*int32_t(8);
						r = max(r,int32_t(0));
						g = max(g,int32_t(0));
						b = max(b,int32_t(0));					
					}
					leds.set_ring_all(tingles[c].index, r, g, b);
					tingles[c].progress++;
				}
			}

			if (post_frame(20)) {
				return;
			}
		}
	}


	void twinkle() {
		#define NUM_TWINKLE 3

		struct tingle {
			bool active;
			int32_t wait;
			int32_t index;
			uint32_t progress;
		} tingles[NUM_TWINKLE];

		memset(tingles, 0, sizeof(tingles));

		for (; ;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, settings.ring_color);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			for (int32_t c = 0 ; c < NUM_TWINKLE; c++) {
				if (tingles[c].active == 0) {
					tingles[c].wait = random.get(0,50);
					for (;;) {
						bool done = true;
						tingles[c].index = random.get(0,16);
						for (int32_t d = 0 ; d < NUM_TWINKLE; d++) {
							if( d != c && 
								tingles[c].active && 
								tingles[c].index == tingles[d].index) {
								done = false;
								break;
							}
						}
						if (done) {
							break;
						}
					}
					tingles[c].index = random.get(0,16);
					tingles[c].progress = 0;
					tingles[c].active = 1;
				} else if (tingles[c].progress >= 16) {
					tingles[c].active = 0;
				} else if (tingles[c].wait > 0) {
					tingles[c].wait --;
				} else {
					uint32_t r = 0,g = 0,b = 0;
					uint32_t progress = tingles[c].progress;
					if (progress > 8) {
						progress -= 8;
						progress = 8 - progress;
					}

					r = max(settings.ring_color.ru(),progress*16);
					g = max(settings.ring_color.gu(),progress*16);
					b = max(settings.ring_color.bu(),progress*16);					
					leds.set_ring_all(tingles[c].index, r,  g, b);
					tingles[c].progress++;
				}
			}

			if (post_frame(50)) {
				return;
			}
		}
	}

	void simple_change_ring() {
		int32_t index = 0;

		int32_t r = random.get(0x00,0x40);
		int32_t g = random.get(0x00,0x40);
		int32_t b = random.get(0x00,0x40);
		int32_t cr = 0;
		int32_t cg = 0;
		int32_t cb = 0;
		int32_t nr = 0;
		int32_t ng = 0;
		int32_t nb = 0;

		for (; ;) {
			if (index >= 600) {
				if (index == 600) {
					cr = r;
					cg = g;
					cb = b;
					nr = random.get(0x00,0x40);
					ng = random.get(0x00,0x40);
					nb = random.get(0x00,0x40);
				}
				if (index >= 664) {
					index = 0;
				} else {
					int32_t lft = index-600;
					int32_t rgt = 64-lft;
					r = (nr*lft + cr*rgt) / 64;
					g = (ng*lft + cg*rgt) / 64;
					b = (nb*lft + cb*rgt) / 64;
					index++;
				}
			} else {
				index++;
			}

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, r, g, b);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(15)) {
				return;
			}
		}
	}

	void simple_change_bird() {
		int32_t index = 0;

		int32_t r = random.get(0x00,0x40);
		int32_t g = random.get(0x00,0x40);
		int32_t b = random.get(0x00,0x40);
		int32_t cr = 0;
		int32_t cg = 0;
		int32_t cb = 0;
		int32_t nr = 0;
		int32_t ng = 0;
		int32_t nb = 0;

		for (; ;) {

			if (index >= 600) {
				if (index == 600) {
					cr = r;
					cg = g;
					cb = b;
					nr = random.get(0x00,0x40);
					ng = random.get(0x00,0x40);
					nb = random.get(0x00,0x40);
				}
				if (index >= 664) {
					index = 0;
				} else {
					int32_t lft = index-600;
					int32_t rgt = 64-lft;
					r = (nr*lft + cr*rgt) / 64;
					g = (ng*lft + cg*rgt) / 64;
					b = (nb*lft + cb*rgt) / 64;
					index++;
				}
			} else {
				index++;
			}

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, settings.ring_color);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, r,
						 		 g,
						 		 b);
			}

			if (post_frame(15)) {
				return;
			}
		}
	}

	void simple_random() {

		rgba colors[16];
		for (int32_t c = 0; c<16; c++) {
			colors[c] = rgba(random.get(0x00,0x40),random.get(0x00,0x40),random.get(0x00,0x40));
		}

		for (; ;) {

			uint32_t index = random.get(0x00,0x10);
			colors[index] = rgba(random.get(0x00,0x40),random.get(0x00,0x40),random.get(0x00,0x40));

			for (uint32_t d = 0; d < 16; d++) {
				leds.set_ring_all(d, colors[d]);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			if (post_frame(20)) {
				return;
			}
		}
	}

	void diagonal_wipe() {

		int32_t walk = 0;
		int32_t wait = random.get(60,1500);
		int32_t dir = random.get(0,2);

		for (; ;) {
			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, settings.ring_color);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			int32_t i0 = -1;
			int32_t i1 = -1;

			if (dir) {
				if (walk < 10) {
					i0 = 7;
					i1 = 7;
				} else if (walk < 20) {
					i0 = 0;
					i1 = 6;
				} else if (walk < 30) {
					i0 = 1;
					i1 = 5;
				} else if (walk < 40) {
					i0 = 2;
					i1 = 4;
				} else if (walk < 50) {
					i0 = 3;
					i1 = 3;
				} else {
					i0 = -1;
					i1 = -1;
				}	
			} else {
				if (walk < 10) {
					i0 = 1;
					i1 = 1;
				} else if (walk < 20) {
					i0 = 0;
					i1 = 2;
				} else if (walk < 30) {
					i0 = 7;
					i1 = 3;
				} else if (walk < 40) {
					i0 = 6;
					i1 = 4;
				} else if (walk < 50) {
					i0 = 5;
					i1 = 5;
				} else {
					i0 = -1;
					i1 = -1;
				}	
			}
			
			walk ++;
			if (walk > wait) {
				walk = 0;
				wait = random.get(60,1024);
				dir = random.get(0,2);
			}

			if (i0 >= 0) leds.set_ring_synced(i0, 0x40,0x40,0x40);
			if (i1 >= 0) leds.set_ring_synced(i1, 0x40,0x40,0x40);

			if (post_frame(5)) {
				return;
			}
		}
	}

	void shimmer_outside() {
		int32_t walk = 0;
		int32_t wait = random.get(16,64);

		for (; ;) {

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, settings.bird_color);
			}

			int32_t r = settings.ring_color.r();
			int32_t g = settings.ring_color.g();
			int32_t b = settings.ring_color.b();
			if (walk < 8) {
				r = max(int32_t(0),r - walk);
				g = max(int32_t(0),g - walk);
				b = max(int32_t(0),b - walk);
			} else if (walk < 16) {
				r = max(int32_t(0),r - (8-(walk-8)));
				g = max(int32_t(0),g - (8-(walk-8)));
				b = max(int32_t(0),b - (8-(walk-8)));
			}

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, r,
						 		 g,
						 		 b);
			}
			
			walk ++;
			if (walk > wait) {
				walk = 0;
				wait = random.get(16,64);

			}

			if (post_frame(2)) {
				return;
			}
		}
	}

	void shimmer_inside() {
		int32_t walk = 0;
		int32_t wait = random.get(16,64);

		for (; ;) {

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, settings.ring_color);
			}

			int32_t r = settings.ring_color.r();
			int32_t g = settings.ring_color.g();
			int32_t b = settings.ring_color.b();
			if (walk < 8) {
				r = max(int32_t(0),r - walk);
				g = max(int32_t(0),g - walk);
				b = max(int32_t(0),b - walk);
			} else if (walk < 16) {
				r = max(int32_t(0),r - (8-(walk-8)));
				g = max(int32_t(0),g - (8-(walk-8)));
				b = max(int32_t(0),b - (8-(walk-8)));
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, r,
								 g,
								 b);
			}
			
			walk ++;
			if (walk > wait) {
				walk = 0;
				wait = random.get(16,64);

			}

			if (post_frame(10)) {
				return;
			}
		}
	}

	void red() {
		int32_t wait = 1200;

		int32_t index = 0;

		int32_t br = settings.bird_color.r();
		int32_t bg = settings.bird_color.g();
		int32_t bb = settings.bird_color.b();

		int32_t rr = settings.ring_color.r();
		int32_t rg = settings.ring_color.g();
		int32_t rb = settings.ring_color.b();

		int32_t b1r = br;
		int32_t b1g = bg;
		int32_t b1b = bb;

		int32_t r1r = rr;
		int32_t r1g = rg;
		int32_t r1b = rb;

		for (; ;) {

			if (index >= 0) {
				if (index >= wait) {
					wait = random.get(1200,10000);
					index = 0;
				} else if (index >= 0 && index < 64) {
					int32_t rgt = index-0;
					int32_t lft = 64-rgt;
					b1r = (br*lft + 0x40*rgt) / 64;
					b1g = (bg*lft + 0x00*rgt) / 64;
					b1b = (bb*lft + 0x10*rgt) / 64;
					r1r = (rr*lft + 0x40*rgt) / 64;
					r1g = (rg*lft + 0x00*rgt) / 64;
					r1b = (rb*lft + 0x10*rgt) / 64;
					index++;
				} else if (index >= 600 && index < 664) {
					int32_t lft = index-600;
					int32_t rgt = 64-lft;
					b1r = (br*lft + 0x40*rgt) / 64;
					b1g = (bg*lft + 0x00*rgt) / 64;
					b1b = (bb*lft + 0x10*rgt) / 64;
					r1r = (rr*lft + 0x40*rgt) / 64;
					r1g = (rg*lft + 0x00*rgt) / 64;
					r1b = (rb*lft + 0x10*rgt) / 64;
					index++;
				} else {
					index++;
				}
			} else {
				index++;
			}

			for (uint32_t d = 0; d < 8; d++) {
				leds.set_ring(d, r1r, r1g, r1b);
			}

			for (uint32_t d = 0; d < 4; d++) {
				leds.set_bird(d, b1r, b1g, b1b);
			}

			if (post_frame(20)) {
				return;
			}
		}
	}
};  // class Effects

}  // namespace {

static LEDs *g_leds = 0;
static SPI *g_spi = 0;
static UI *g_ui = 0;
static Effects *g_effects = 0;
static SX1280 *g_sx1280 = 0;
static SDD1306 *g_sdd1306 = 0;
static EEPROM *g_settings = 0;
static FT25H16S *g_ft25h16s = 0;

#ifdef ENABLE_USB_MSC
static USBD_HANDLE_T g_hUsb;
#endif  // #ifdef ENABLE_USB_MSC

extern "C" {
	
	void SysTick_Handler(void)
	{	
		system_clock_ms++;
		
		g_ui->CheckInput();

		g_spi->push_frame(*g_leds, g_settings->brightness);

		if ( (system_clock_ms % (1024*256)) == 0) {
			if (g_ui->Mode() == 0) {
				g_ui->SetMode(system_clock_ms, 1);
			}
		}
		
		if ( (system_clock_ms % (256)) == 0) {
			g_sx1280->ProcessIrqs();
			if (g_settings->recv_radio_message_pending && g_ui->Mode() == 0) {
				g_settings->recv_radio_message_pending = false;
				g_ui->SetMode(system_clock_ms, 6);
			}
		}

		if ( (system_clock_ms % (1024*64)) == 0) {
			g_settings->SaveRuntime();
		}
		
#if 0
		if ( (system_clock_ms % (1024*8)) == 0) {
			g_settings->radio_color ++;
			g_settings->radio_color %= 11;
			static int32_t msg_index = 0;
			char str[8];
			sprintf(str,"%08d",msg_index++);
			memcpy(g_settings->radio_messages[g_settings->radio_message],str,8);
			g_sx1280->SendMessage();
		}
#endif  // #if 0
		
		g_effects->CheckPostTime();
	}

	void TIMER32_0_IRQHandler(void)
	{
	}

	void UART_IRQHandler(void)
	{
	}
	
	void FLEX_INT0_IRQHandler(void)
	{
		g_sx1280->OnDioIrq();
		Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH0);
	}

	void FLEX_INT1_IRQHandler(void)
	{	
		if (g_ui) {
			g_ui->HandleINT1IRQ();
		}
	}

	void FLEX_INT2_IRQHandler(void)
	{
		if (g_ui) {
			g_ui->HandleINT2IRQ();
		}
	}
	
	void USB_IRQHandler(void)
	{
#ifdef ENABLE_USB_MSC
		const  USBD_API_T *pUsbApi = (const USBD_API_T *) LPC_ROM_API->usbdApiBase;
		pUsbApi->hw->ISR(g_hUsb);
#endif  // #ifdef ENABLE_USB_MSC
	}

}

#ifdef ENABLE_USB_MSC
const char *autorun_file =
	"[autorun]\r\n"
	"label=emfat test drive\r\n"
	"ICON=icon.ico\r\n";


#define AUTORUN_SIZE 50
static void autorun_read_proc(uint8_t *dest, int size, uint32_t offset, size_t userdata)
{
	int len = 0;
	if (offset > AUTORUN_SIZE) return;
	if (offset + size > AUTORUN_SIZE)
		len = AUTORUN_SIZE - offset; else
		len = size;
	memcpy(dest, &autorun_file[offset], len);
}

#define README_SIZE  21
const char *readme_file =
	"This is readme file\r\n";

static void readme_read_proc(uint8_t *dest, int size, uint32_t offset, size_t userdata)
{
	int len = 0;
	if (offset > README_SIZE) return;
	if (offset + size > README_SIZE)
		len = README_SIZE - offset; else
		len = size;
	memcpy(dest, &readme_file[offset], len);
}

static bool usb_init()
{
	static const uint8_t USB_DeviceDescriptor[] =
	{
		USB_DEVICE_DESC_SIZE,              /* bLength */
		USB_DEVICE_DESCRIPTOR_TYPE,        /* bDescriptorType */
		WBVAL(0x0200), /* 2.00 */          /* bcdUSB */
		0x00,                              /* bDeviceClass */
		0x00,                              /* bDeviceSubClass */
		0x00,                              /* bDeviceProtocol */
		USB_MAX_PACKET0,                   /* bMaxPacketSize0 */
		WBVAL(0x1FC9),                     /* idVendor */
		WBVAL(0x0108),                     /* idProduct */
		WBVAL(0x0100), /* 1.00 */          /* bcdDevice */
		0x01,                              /* iManufacturer */
		0x02,                              /* iProduct */
		0x03,                              /* iSerialNumber */
		0x01                               /* bNumConfigurations */
	};

	static uint8_t USB_FsConfigDescriptor[] = {
		/* Configuration 1 */
		USB_CONFIGURATION_DESC_SIZE,       /* bLength */
		USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType */
		WBVAL(                             /* wTotalLength */
		1*USB_CONFIGURATION_DESC_SIZE +
		1*USB_INTERFACE_DESC_SIZE     +
		2*USB_ENDPOINT_DESC_SIZE
		),
		0x01,                              /* bNumInterfaces */
		0x01,                              /* bConfigurationValue */
		0x00,                              /* iConfiguration */
		USB_CONFIG_SELF_POWERED,           /* bmAttributes */
		USB_CONFIG_POWER_MA(100),          /* bMaxPower */
		
		/* Interface 0, Alternate Setting 0, MSC Class */
		USB_INTERFACE_DESC_SIZE,           /* bLength */
		USB_INTERFACE_DESCRIPTOR_TYPE,     /* bDescriptorType */
		0x00,                              /* bInterfaceNumber */
		0x00,                              /* bAlternateSetting */
		0x02,                              /* bNumEndpoints */
		USB_DEVICE_CLASS_STORAGE,          /* bInterfaceClass */
		MSC_SUBCLASS_SCSI,                 /* bInterfaceSubClass */
		MSC_PROTOCOL_BULK_ONLY,            /* bInterfaceProtocol */
		0x05,                              /* iInterface */
		
		/* Bulk In Endpoint */
		USB_ENDPOINT_DESC_SIZE,            /* bLength */
		USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType */
		MSC_EP_IN,                         /* bEndpointAddress */
		USB_ENDPOINT_TYPE_BULK,            /* bmAttributes */
		WBVAL(USB_FS_MAX_BULK_PACKET),     /* wMaxPacketSize */
		0,                                 /* bInterval */
		
		/* Bulk Out Endpoint */
		USB_ENDPOINT_DESC_SIZE,            /* bLength */
		USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType */
		MSC_EP_OUT,                        /* bEndpointAddress */
		USB_ENDPOINT_TYPE_BULK,            /* bmAttributes */
		WBVAL(USB_FS_MAX_BULK_PACKET),     /* wMaxPacketSize */
		0,                                 /* bInterval */
		/* Terminator */
		0                                  /* bLength */
	};

	static const uint8_t USB_StringDescriptor[] =
	{
		/* Index 0x00: LANGID Codes */
		0x04,                              /* bLength */
		USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
		WBVAL(0x0409), /* US English */    /* wLANGID */
		/* Index 0x01: Manufacturer */
		(18*2 + 2),                        /* bLength (13 Char + Type + lenght) */
		USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
		'N', 0,
		'X', 0,
		'P', 0,
		' ', 0,
		'S', 0,
		'e', 0,
		'm', 0,
		'i', 0,
		'c', 0,
		'o', 0,
		'n', 0,
		'd', 0,
		'u', 0,
		'c', 0,
		't', 0,
		'o', 0,
		'r', 0,
		's', 0,
		/* Index 0x02: Product */
		(14*2 + 2),                        /* bLength (13 Char + Type + lenght) */
		USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
		'L', 0,
		'P', 0,
		'C', 0,
		'1', 0,
		'1', 0,
		'U', 0,
		'x', 0,
		'x', 0,
		'M', 0,
		'e', 0,
		'm', 0,
		'o', 0,
		'r', 0,
		'y', 0,
		/* Index 0x03: Serial Number */
		(13*2 + 2),                        /* bLength (13 Char + Type + lenght) */
		USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
		'A', 0,
		'B', 0,
		'C', 0,
		'D', 0,
		'1', 0,
		'2', 0,
		'3', 0,
		'4', 0,
		'5', 0,
		'6', 0,
		'7', 0,
		'8', 0,
		'9', 0,
		/* Index 0x04: Interface 0, Alternate Setting 0 */
		(3*2 + 2),                        /* bLength (3 Char + Type + lenght) */
		USB_STRING_DESCRIPTOR_TYPE,       /* bDescriptorType */
		'D', 0,
		'F', 0,
		'U', 0,
		/* Index 0x05: Interface 1, Alternate Setting 0 */
		(6*2 + 2),                        /* bLength (13 Char + Type + lenght) */
		USB_STRING_DESCRIPTOR_TYPE,       /* bDescriptorType */
		'M', 0,
		'e', 0,
		'm', 0,
		'o', 0,
		'r', 0,
		'y', 0,
	};
	
	/* initialize USBD ROM API pointer. */
	const  USBD_API_T *pUsbApi = (const USBD_API_T *) LPC_ROM_API->usbdApiBase;

	/* enable USB main clock */
	Chip_Clock_SetUSBClockSource(SYSCTL_USBCLKSRC_PLLOUT, 1);
	/* Enable AHB clock to the USB block and USB RAM. */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_USB);
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_USBRAM);
	/* power UP USB Phy */
	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_USBPAD_PD);
	
	/* initialize call back structures */
	static USBD_API_INIT_PARAM_T usb_param;
	memset((void *) &usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
	usb_param.usb_reg_base = LPC_USB0_BASE;
	usb_param.mem_base = USB_STACK_MEM_BASE;
	usb_param.mem_size = USB_STACK_MEM_SIZE;
	usb_param.max_num_ep = 2;

	/* Set the USB descriptors */
	static USB_CORE_DESCS_T desc;
	memset((void*)&desc, 0, sizeof(USB_CORE_DESCS_T));
	desc.device_desc = (uint8_t *) &USB_DeviceDescriptor[0];
	desc.string_desc = (uint8_t *) &USB_StringDescriptor[0];
	desc.high_speed_desc = (uint8_t *)&USB_FsConfigDescriptor[0];
	desc.full_speed_desc = (uint8_t *)&USB_FsConfigDescriptor[0];

	/* USB Initialization */
	ErrorCode_t ret = pUsbApi->hw->Init(&g_hUsb, &desc, &usb_param);

	if (ret == LPC_OK) {

		static USBD_MSC_INIT_PARAM_T msc_param;
		memset(&msc_param, 0, sizeof(USBD_MSC_INIT_PARAM_T));

		msc_param.mem_base = usb_param.mem_base;
		msc_param.mem_size = usb_param.mem_size;

		static const uint8_t InquiryStr[] = {	'N','X','P',' ',' ',' ',' ',' ',     \
												'L','P','C',' ','M','e','m',' ',     \
												'D','i','s','k',' ',' ',' ',' ',     \
												'1','.','0',' ',};

		msc_param.InquiryStr = (uint8_t*)InquiryStr; 
		msc_param.BlockCount = Emfat.disk_sectors;
		msc_param.BlockSize = SECT;
		msc_param.MemorySize = Emfat.vol_size;

		USB_INTERFACE_DESCRIPTOR* pIntfDesc = (USB_INTERFACE_DESCRIPTOR *)&USB_FsConfigDescriptor[sizeof(USB_CONFIGURATION_DESCRIPTOR)];
		
		if ((pIntfDesc == 0) || (pIntfDesc->bInterfaceClass != USB_DEVICE_CLASS_STORAGE) || (pIntfDesc->bInterfaceSubClass != MSC_SUBCLASS_SCSI) ) {
			return false;
		}

		msc_param.intf_desc = (uint8_t*)pIntfDesc;

		msc_param.MSC_Write = translate_wr; 
		msc_param.MSC_Read = translate_rd;
		msc_param.MSC_Verify = translate_verify;   

		ret = pUsbApi->msc->init(g_hUsb, &msc_param);

		usb_param.mem_base = msc_param.mem_base;
		usb_param.mem_size = msc_param.mem_size;

		if (ret == LPC_OK) {
			/*  enable USB interrupts */
			NVIC_EnableIRQ(USB0_IRQn);
			/* now connect */
			pUsbApi->hw->Connect(g_hUsb, 1);
			return true;
		}
	}
	return false;
}
#endif  // #ifdef ENABLE_USB_MSC

int main(void)
{
	SystemCoreClockUpdate();

	delay(100);
	
	SDD1306 sdd1306; g_sdd1306 = &sdd1306;
	
	BQ24295 bq24295;
	
	Setup setup(sdd1306, bq24295);

	if (bq24295.DevicePresent()) {
		bq24295.SetBoostVoltage(4550);
		bq24295.DisableWatchdog();
		bq24295.DisableOTG();
	}

	EEPROM settings;
	g_settings = &settings;
	
	SPI spi; g_spi = &spi;

	LEDs leds; g_leds = &leds;
	
	spi.push_null();

	if (sdd1306.DevicePresent()) {
		sdd1306.Init(); 
		sdd1306.Clear();
		sdd1306.DisplayBootScreen();
		static uint8_t bounce[] = {
			0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1e, 0x1d, 0x1d, 
			0x1c, 0x1b, 0x1a, 0x18, 0x17, 0x16, 0x14, 0x12, 
			0x10, 0x0e, 0x0c, 0x0a, 0x08, 0x05, 0x03, 0x01, 
			0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x07, 0x08, 
			0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x07, 0x07, 
			0x06, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x01, 
			0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 
			0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 
		};
		sdd1306.SetVerticalShift(bounce[0]);
		sdd1306.Display();
		for (uint32_t y=0; y<64; y++) {
			sdd1306.SetVerticalShift(bounce[y]);
			delay(10);
		}
		delay(500);
		static int8_t ease[] = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 
			0x02, 0x02, 0x03, 0x03, 0x04, 0x05, 0x06, 0x07, 
			0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0e, 0x0f, 0x11, 
			0x12, 0x14, 0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f, 
			0x1f,
		};
		for (uint32_t x=0; x<=32; x++) {
			sdd1306.SetVerticalShift(-ease[x]);
			sdd1306.SetCenterFlip(x);
			sdd1306.Display();
			delay(1);
		}
		sdd1306.SetVerticalShift(0);
		sdd1306.SetCenterFlip(0);
	}
	
	settings.Load();
	
	Random random(0xCAFFE);
	
	FT25H16S ft25h16s; g_ft25h16s = &ft25h16s;

	SX1280 sx1280(sdd1306, settings, ft25h16s); g_sx1280 = &sx1280;
	sx1280.Init(false);

	if (bq24295.DevicePresent()) {
		bq24295.SetBoostVoltage(4550);
		bq24295.DisableWatchdog();
		bq24295.DisableOTG();
	}

#ifdef ENABLE_USB_MSC
	#define CMA_TIME EMFAT_ENCODE_CMA_TIME(2,4,2017, 13,0,0)
	#define CMA { CMA_TIME, CMA_TIME, CMA_TIME }
	static emfat_entry_t emfat_entries[] =
	{
		// name          dir    lvl offset  size             max_size        user  time  read               write
		{ "",            true,  0,  0,      0,               0,              0,    CMA,  NULL,              NULL, { } }, // root
		{ "autorun.inf", false, 1,  0,      AUTORUN_SIZE,    AUTORUN_SIZE,   0,    CMA,  autorun_read_proc, NULL, { } }, // autorun.inf
		{ "drivers",     true,  1,  0,      0,               0,              0,    CMA,  NULL,              NULL, { } }, // drivers/
		{ "readme.txt",  false, 2,  0,      README_SIZE,     1024*1024,      0,    CMA,  readme_read_proc,  NULL, { } }, // drivers/readme.txt
		{ NULL,			 false,	0,	0,		0,				 0,				 0,	   CMA,	 NULL,				NULL, { } }
	};

	emfat_init(&Emfat, "emfat", emfat_entries);
	
	usb_init();
#endif  // #ifdef ENABLE_USB_MSC

	UI ui(settings, sdd1306, sx1280, random, ft25h16s, bq24295);
	// For IRQ handlers only
	g_ui = &ui;
	ui.Init();

	// start 1ms timer
	SysTick_Config(SystemCoreClock / 1000);

	Effects effects(settings, random, leds, spi, sdd1306, ui); g_effects = &effects;

	effects.RunForever();
	
	return 0;
}
