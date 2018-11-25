#include "tgsec_ibeacon.h"
#include "sha.h"

// private functions
static void tgsec_sha256_encoding(const uint8_t *in, unsigned int in_count, uint8_t *digest);
static bool tgsec_xors_in_half(const uint8_t *in_data, unsigned int in_count, uint8_t *out);

#define ROT_ORIGIN_DATA_LEN 14

static uint8_t beaconid[4];
static uint8_t data[ROT_ORIGIN_DATA_LEN];
static uint8_t digest[SHA256HashSize];
static uint8_t xors1st[16];
static uint8_t xors2nd[8];
static uint8_t xors3rd[4];

void calc_store_rot_mm(uint8_t *rot_mm)
{
  uint8_t *_beacon_info = ble_bms_get_beacon_info();

  beaconid[0] = _beacon_info[BINFO_MAJOR_VALUE_IDX];
  beaconid[1] = _beacon_info[BINFO_MAJOR_VALUE_IDX+1];
  beaconid[2] = _beacon_info[BINFO_MINOR_VALUE_IDX];
  beaconid[3] = _beacon_info[BINFO_MINOR_VALUE_IDX+1];    

  memcpy(data, beaconid, 4);
  memcpy(data+4, &_beacon_info[BINFO_TGSECB_SEC_KEY_IDX], 6);

  for (int i=0; i<4; i++) {
    data[i+10] = m_tgsec_timestamp.array[3-i];
  }

#ifdef _DEBUG_
  printf("DATA: ");
  for (int i = 0; i < ROT_ORIGIN_DATA_LEN; i++) {
    printf("%02X ", data[i]);
  }
  printf("\n");
#endif
  
  tgsec_sha256_encoding(data, ROT_ORIGIN_DATA_LEN, digest);

#ifdef _DEBUG_ 
  printf("SHA256: ");
  for (int i = 0; i < SHA256HashSize; i++) {
    printf("%02X ", digest[i]);
  }
  printf("\n");
#endif
  
  tgsec_xors_in_half(digest, 32, xors1st);

  tgsec_xors_in_half(xors1st, 16, xors2nd);

  tgsec_xors_in_half(xors2nd, 8, xors3rd);

#ifdef _DEBUG_ 
  printf("FINAL: ");
  for (int i = 0; i < 4; i++) {
    printf("%02X ", xors3rd[i]);
  }
  printf("\n");
#endif
  
  memcpy(rot_mm, xors3rd, 4);

}

bool tgsec_xors_in_half(const uint8_t *in_data, unsigned int in_count, uint8_t *out)
{
  if (in_count % 2 != 0) {
    return false;
  }

  unsigned int half = in_count / 2;

  for (int i = 0; i < half; i++) {
    out[i] = in_data[i] ^ in_data[half+i];
  }

  return true;
}

void tgsec_sha256_encoding(const uint8_t *in_data, unsigned int in_count, uint8_t *digest) 
{
  SHA256Context sha_context;
  SHA256Context *psha_context = &sha_context;
  
  SHA256Reset(psha_context);
  SHA256Input(psha_context, in_data, in_count);
  //SHA256FilnalBits(SHA256, ?, ?);
  SHA256Result(psha_context, digest);
}
