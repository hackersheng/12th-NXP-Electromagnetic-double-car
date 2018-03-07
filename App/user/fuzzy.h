#ifndef __fuzzy_h__
#define __fuzzy_h__

#define s16     int16
#define u16     uint16
#define u8      uint8
#define s8      int8

typedef struct FUZTAB
{
  s16 cntrow;
  s16 cntcolume;
  s16 ptab[10][10];
  s16 itab[10][10];
  s16 dtab[10][10];
  s16 Edot[10];
  s16 ECdot[10];
} FUZTAB;

typedef struct LinearTAB
{
   s16 row;
   s16 colume;
   u8  K1[6][2];
   u8  K2[6][2];
   u8  K3[6][2];
   u8  K4[6][2];
   u8  K5[6][2];
   s16 Sumdot[6]; 
   
}LinearTAB;

void fuzzyout(const FUZTAB *l_tab,s16 l_EE,s16 l_EEC,s16 *pout,s16 *iout,s16 *dout);
int speed_fuzzy(uint8 conponent1,int16 conponent2);
void expspeed_fuzzyout(FUZTAB *l_tab,s16 l_EE,s16 l_EEC,s16 *pout);
void Linear_error_ratio(LinearTAB *Ptab,u16 Psum,s8 Dirflag,u8 *Pk1,u8 *Pk2,u8 *Pk3,u8 *Pk4,u8 *Pk5);
int distance_fuzzy(int32 distance,uint8 position);
#endif