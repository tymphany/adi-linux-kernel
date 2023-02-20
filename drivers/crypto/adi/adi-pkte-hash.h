#ifndef ADI_PKTE_HASH
#define ADI_PKTE_HASH

#define NUM_HASH_CATEGORIES 2
extern ADI_ALGS_INFO adi_algs_info_adi[NUM_HASH_CATEGORIES];

void adi_write_packet(struct adi_dev *pkte_dev, u32 *source);

#endif