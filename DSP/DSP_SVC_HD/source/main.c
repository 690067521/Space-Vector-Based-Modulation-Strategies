#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Sintable.h"
#include "math.h"

#define tmode	0	// tmode is for runtime test
#define method	0	// 0=xy, 1=HD, 2=BJ120, 3=BJ60, 4=ABC, 5=Six, 6=Shifted HD, 7=Shifted BJ, (iterative) 8=xy, 9=HD, 10=BJ120, 11=BJ60, 12=ABC, 13=Six, 14=original
#define GET_FLOAT_WORD(dst,f) (dst=__f32_bits_as_u32(f))
long b1, b2, b3;
short s1, s2, s3;

#if !tmode
	#define SetLOAD GpioDataRegs.GPBDAT.bit.GPIO55=1;	//将LOAD置高
	#define ClrLOAD GpioDataRegs.GPBDAT.bit.GPIO55=0;	//将LOAD置低
	void WriteDAC(unsigned char add,unsigned char rng,unsigned char vol);
	void delay(unsigned int t);
	void spi_xmit(Uint16 a);
	void spi_fifo_init(void);
	void spi_init(void);
#endif

#define 	N_SM_2		3

float	v1, v2, v3, v4, v5, v6, d1, d2, d3, ff1, ff2, tt, lt, st, dsum, ddif, vabc[] = {0.6108, -1.034, 0.4227}, *c1c2v1vl;
int		i, f1=0, f2=0, f3=0, fsum, vo[2], k, origin[3]={0,0,0}, state[3]={0,0,0}, cnt=0, ph, temp[]={0,0,0,0}, minv, mini, maxv, maxi, samplecnt[1000], samplea[1000], sampleb[1000], samplec[1000], vs[2];
float	samplecom[1000];
#if !tmode
interrupt void cpu_timer0_isr(void)
{
//	if (cnt==555)
//		cnt = 555;
	CpuTimer1Regs.TIM.all = 5999;

#if (method == 0)	// xy, 108-114/123-129
	v1 = sin_a8[cnt]*3 * N_SM_2/8*8.66;	// 11, -20 ~ +20
	v2 = (sin_b8[cnt] - sin_c8[cnt]) * N_SM_2/8*8.66;	// 13, -10 ~ +10
	f1 = (int)(v1>=0 ? v1 : v1-1);	// 15
	f2 = (int)(v2>=0 ? v2 : v2-1);	// 14

	samplea[cnt] = 0;
	if ((f1 + f2) & 1)	// 9, odd
		if ((v2 - f2) * 3 < v1 - f1 + 1.0)
		{
			f1++;
			samplea[cnt] = 1;
		}
		else
			f2++;
	else	// even
		if ((v2 - f2) * 3 >= f1 + 2.0 - v1)	// 17
		{
			f1++;
			f2++;
		}
	v1 = N_SM_2 + 0.5 + f1*0.3333333333333333;	// 8
	state[0] = (int)(v1>=0.5? v1 : v1-0.99);			// 16
	state[1] = state[0] - (f1-f2)/2;		// 11
	state[2] = state[1] - f2;		// 4

#elif (method == 1)	// HD, 119-149/134-164
//  d1 = sin_a8[cnt] - sin_b8[cnt];	// 11
//	f2 = floor(v2);	// 68
	v1 = (sin_a8[cnt] - sin_b8[cnt]) * N_SM_2;		// 15
	v2 = (sin_a8[cnt] - sin_c8[cnt]) * N_SM_2;		// 13
	f1 = (int)(v1>=0 ? v1 : v1-1);
	f2 = (int)(v2>=0 ? v2 : v2-1);
	d1 = v1 - f1;		// 5
	d2 = v2 - f2;		// 5
	if (d1 >= d2*2 && d1*2 >= d2+1.0)	// 12/18, low low
	   f1++;
	else if (d2*2 >= d1+1.0 && d2 >= d1*2)	// 20/24, high high
	   f2++;
	else if (d1 + d2 >= 1)	// 14/16, right
	{
	   f1++;	// 1
	   f2++;
	}
	v1 = N_SM_2 + 0.5 + (f1+f2)*0.3333333333333333;
//	k = (int)(v1>=0? v1+0.5 : v1-0.5);	// 12
//	state[0] = k;			// 1
//	state[1] = k - f1;		// 2
//	state[2] = k - f2;		// 3
	state[0] = (int)(v1>=0.5? v1 : v1-1);			// 1
	state[1] = state[0] - f1;		// 2
	state[2] = state[0] - f2;		// 3


#elif (method == 2)	// BJ120, 130-152
	v1 = sin_a8[cnt]*3 * N_SM_2;	// 21, -20 ~ +20
	v2 = -sin_c8[cnt]*3 * N_SM_2;	// 15, -10 ~ +10
	f1 = (int)(v1>=0 ? v1 : v1-1);
	f2 = (int)(v2>=0 ? v2 : v2-1);
	fsum = f1+f2;
	k = fsum>=0 ? fsum - (int)(fsum*0.3333333333333333) * 3 : fsum - (int)(fsum*0.3333333333333333-0.99) * 3;	// 11/29
//	k = (f1+f2) - (int)((f1+f2)*0.3333333333333333) * 3;	// 21
//	k = k >= 0 ? k : k+3;	// 7/9
//	k = fmodl(f1+f2, 3);	// 365
	if (k == 1)	// 8
	{
		f1++;
		f2++;	// assure f1+f2=3k
	}
	else if (k == 2)	// 5
		if (v1 - f1 >= v2 - f2)	// 15, why fastest?
			f1++;
		else
			f2++;
	v1 = N_SM_2 + 0.5 + f1*0.3333333333333333;
	state[0] = (int)(v1>=0.5 ? v1 : v1-1);			// 1
	state[1] = state[0] - (f1*2-f2)*0.3333333333333333;
	state[2] = state[0]*2 - f1 - state[1];		// 4

#elif (method == 3)	// BJ60, 125-159
	v1 = -sin_b8[cnt]*3 * N_SM_2;	// 11, -20 ~ +20
	v2 = -sin_c8[cnt]*3 * N_SM_2;	// 11, -10 ~ +10
	f1 = (int)(v1>=0 ? v1 : v1-1);
	f2 = (int)(v2>=0 ? v2 : v2-1);
	fsum = f2 - f1;
	k = fsum>=0 ? fsum - (int)(fsum*0.3333333333333333) * 3 : fsum - (int)(fsum*0.3333333333333333-1) * 3;	// 21/24
//	k = (f2-f1) - (int)((f2-f1)*0.3333333333333333) * 3;	// 21
//	k = k >= 0 ? k : k+3;
	if (k == 1)	// 9
		f1++;
	else if (k == 2)	// 8
		f2++;
	else
		if (v1 + v2 >= f1 + f2 + 1.0)	// 18/21
		{
			f1++;
			f2++;
		}
	fsum = f1 + f2;
	v1 = N_SM_2 + 0.5 + fsum*0.3333333333333333;
	state[0] = (int)(v1>=0.5 ? v1 : v1-1);			// 1
	state[1] = state[0]-(f1*2+f2)*0.3333333333333333;
	state[2] = state[0]*2 - fsum - state[1];		// 5

#elif (method == 4)	// ABC, 140-171
	v1 = sin_a8[cnt] * 3 * N_SM_2;	// 11, -20 ~ +20
	v2 = sin_b8[cnt] * 3 * N_SM_2;	// 9
	v3 = -v1 - v2;	// 6
//	v3 = sin_c8[cnt] * 3 * N_SM_2;	// 9

	f1 = (int)(v1>=0 ? v1 : v1-1);
	f2 = (int)(v2>=0 ? v2 : v2-1);
	f3 = (int)(v3>=0 ? v3+1 : v3);
	temp[0] = f1*2 + f3;
	if (temp[0] == (int)(temp[0]*0.3333333333333333) * 3)	// 9, f3++
		f2 = -f1 - f3;
	else
	{
		temp[1] = f2*2 + f1+1;
		if (temp[1] == (int)(temp[1]*0.3333333333333333) * 3)	// 8
			f1++;
		else
		{
			f1 = -f3 - f2;
			f2++;
		}
	}

//	f3 = (int)(v3>=0 ? v3 : v3-1);
//	temp[0] = f1+1 - f2;
//	if (temp[0] == (int)(temp[0]*0.3333333333333333) * 3)	// 9, f3++
//		f1++;
//	else
//	{
//		temp[1] = f2+1 - f3;
//		if (temp[1] == (int)(temp[1]*0.3333333333333333) * 3)	// 8
//		{
//			f2++;
//			f1 = -f2 - f3;
//		}
//		else	// f3+1 - f1 = 3*k
//			f2 = -f1 - f3 - 1;
//	}
	v1 = N_SM_2 + 0.5 + f1*0.3333333333333333;
	state[0] = (int)(v1>=0.5 ? v1 : v1-1);			// 1
	state[1] = state[0] + (f2-f1)*0.3333333333333333;
	state[2] = state[0]*2 - f1 - state[1];		// 5

#elif (method == 6)	// Shifted HD, 110-136
	v1 = (sin_a8[cnt] - sin_b8[cnt]) * N_SM_2 + 0.3333333333333333;		// 17
	v2 = (sin_a8[cnt] - sin_c8[cnt]) * N_SM_2 + 0.6666666666666666;		// 15
//	f1 = v1>=0 ? (int)v1 : (int)v1-1;			// 16
//	f2 = v2>=0 ? (int)v2 : (int)v2-1;			// 16
	f1 = (int)(v1>=0 ? v1 : v1-1);
	f2 = (int)(v2>=0 ? v2 : v2-1);
	d1 = v1 - f1;		// 5
	d2 = v2 - f2;		// 5

//	if (d2*2 <= d1)
//	{
//		if (d1 + d2 <= 1)	// 10/13	right
//			f2--;	// 1
//		else
//			f1++;	// 1
//	}
//	else if (d2 <= d1*2-1)
//	{
//		f1++;	// 1
//	}

	if (d1 >= d2)	// 7/10
		if (d1 + d2 >= 1)	// 10/13	right
		{
			if (d1*2-d2 >= 1)	// 11	low
				f1++;	// 1
		}
		else if (d1 >= d2*2)		// 8/9	left, low
				f2--;	// 1

//	if (d1 + d2 >= 1)	// 10/13	right	max down but ave up
//	{
//		if (d1*2 >= d2 + 1)	f1++;	// 11	low
//	}
//	else if (d1 >= d2*2)	f2--;	// 8/9	left, low

	v1 = N_SM_2 + 0.5 + (f1+f2)*0.3333333333333333;
	state[0] = (int)(v1>=0.5 ? v1 : v1-1);			// 1
	state[1] = state[0] - f1;		// 2
	state[2] = state[0] - f2;		// 3

#elif (method == 7)	// Shifted BJ60, 196-230
	v1 = -sin_b8[cnt]*3 * N_SM_2 * 0.5 + 0.5;	// 11, -20 ~ +20
	v2 = -sin_c8[cnt]*3 * N_SM_2 * 0.5 + 0.5;
	f1 = (int)(v1>=0 ? v1 : v1-1);
	f2 = (int)(v2>=0 ? v2 : v2-1);
	d1 = v1 - f1;		// 5
	d2 = v2 - f2;		// 5
	dsum = d1 + d2;
	fsum = f1 - f2;
	k = fsum>=0 ? fsum - (int)(fsum*0.3333333333333333) * 3 : fsum - (int)(fsum*0.3333333333333333-0.99) * 3;	// 11/29, -3/-6 may wrong
//	k = (f1-f2) - (int)((f1-f2)*0.3333333333333333) * 3;	// 21
//	k = k >= 0 ? k : k+3;
	ff1 = f1;
	ff2 = f2;
	if (k == 0)	// 9
	{
		if (dsum >= 1.5)	// 8
		{
			ff1++;
			ff2++;
		}
		else if (dsum >= 0.5)
		{
			ff1 = ff1 + 0.5;
			ff2 = ff2 + 0.5;
		}
	}
	else if (k == 1)
	{
		if (dsum < 1 && d2 < 0.5)
		{
			ff1 = ff1 + 0.5;
		}
		else if (dsum >= 1 && d1 >= 0.5)
		{
			ff1++;
			ff2 = ff2 + 0.5;
		}
		else
		{
			ff2++;
		}
	}
	else
	{
		if (dsum < 1 && d1 < 0.5)
		{
			ff2 = ff2 + 0.5;
		}
		else if (dsum >= 1 && d2 >= 0.5)
		{
			ff1 = ff1 + 0.5;
			ff2++;
		}
		else
		{
			ff1++;
		}
	}
	f1 = ff1 * 2 - 1;
	f2 = ff2 * 2 - 1;
	fsum = f1 + f2;
	k = (int)(N_SM_2 + 0.5 + fsum*0.3333333333333333);	// 19
	state[0] = k;			// 1
	state[1] = (int)(N_SM_2 + 0.5 - f1*0.3333333333333333);		// 12
	state[2] = k*2 - fsum - state[1];		// 5

//
#elif (method == 8)	// xy, 83-154/71-145
//	v1 = sin_a8[cnt]* 3 * N_SM_2 - state[0]*2 + state[1] + state[2];	// 11, -20 ~ +20
//	v2 = (sin_b8[cnt] - sin_c8[cnt]) * N_SM_2 - state[1] + state[2];				// 8, -10 ~ +10
//	v3 = v2*3-v1;
//	v4 = v2*3+v1;
////	v3 = (-sin_a8[cnt]*2 + sin_b8[cnt]*4 - sin_c8[cnt]*2) * N_SM_2 ...;
////	v4 = (sin_a8[cnt]*2 + sin_b8[cnt]*2 - sin_c8[cnt]*4) * N_SM_2 ...;
//	if (fabsf(v1) >= 1 || fabsf(v3) >= 2 || fabsf(v4) >= 2)	//	27
//	{
//		b1 = __f32_bits_as_u32(v1) & 0x80000000;
//		b2 = __f32_bits_as_u32(v3) & 0x80000000;
//		b3 = __f32_bits_as_u32(v4) & 0x80000000;
//		if (!b3)
//			if (b2)		// 11
//				state[0]++;		// 1
//			else if (b1)
//				state[1]++;		// 1
//			else
//				state[2]--;
//		else
//			if (!b2)		// 11
//				state[0]--;
//			else if (!b1)
//				state[1]--;
//			else
//				state[2]++;
////	if (fabsf(v1) >= 1 || fabsf(v3) >= 2 || fabsf(v4) >= 2)
////	{
////		if (v4 >= 0 && v3 < 0)
////			state[0]++;		// 1
////		else if (v3 >= 0 && v1 >= 0)
////			state[2]--;		// 2
////		else if (v1 < 0 && v4 >= 0)
////			state[1]++;
////		else if (v4 < 0 && v3 >= 0)
////			state[0]--;
////		else if (v3 < 0 && v1 < 0)
////			state[2]++;
////		else
////			state[1]--;		// 6
//		d1 = N_SM_2 - (state[0]+state[1]+state[2])*0.3333333333333333;
//		k = (int)(d1 >= 0 ? d1+0.5 : d1-0.5);	// 12
//		state[0] += k;			// 1
//		state[1] += k;
//		state[2] += k;
//	}

	v1 = sin_a8[cnt]* 3 * N_SM_2 - f1;	// 11, -20 ~ +20
	v2 = (sin_b8[cnt] - sin_c8[cnt]) * N_SM_2 - f2;				// 8, -10 ~ +10
	v3 = v2*3-v1;
	v4 = v2*3+v1;
	if (fabsf(v1) >= 1 || fabsf(v3) >= 2 || fabsf(v4) >= 2)	//	27
	{
		b1 = __f32_bits_as_u32(v1) & 0x80000000;
		b2 = __f32_bits_as_u32(v3) & 0x80000000;
		b3 = __f32_bits_as_u32(v4) & 0x80000000;
		if (!b3)
			if (b2)		// 11
			{
				f1 += 2;		// 1
//				state[0]++;
			}
			else if (b1)
			{
				f1--;
				f2++;
//				state[1]++;
			}
			else
			{
				f1++;
				f2++;
//				state[2]--;
			}
		else
			if (!b2)		// 11
			{
				f1 -= 2;
//				state[0]--;
			}
			else if (!b1)
			{
				f1++;
				f2--;
//				state[1]--;
			}
			else
			{
				f1--;
				f2--;
//				state[2]++;
			}
//		d1 = N_SM_2 + 0.5 - (state[0]+state[1]+state[2])*0.3333333333333333;	// 8
//		k = (int)(d1 >= 0.5 ? d1 : d1-1);	// 13
//		state[0] += k;			// 1
//		state[1] += k;
//		state[2] += k;
		d1 = N_SM_2 + 0.5 + f1*0.3333333333333333;
		state[0] = (int)(d1 >= 0.5 ? d1 : d1-1);;			// 1
		state[1] = state[0] - (f1-f2)/2;		// 9
		state[2] = state[1] - f2;		// 2
	}

#elif (method == 9)	// HD, 81-144
//	v1 = (sin_a8[cnt] - sin_b8[cnt]) * N_SM_2 - state[0] + state[1];		// 19
//	v2 = (sin_a8[cnt] - sin_c8[cnt]) * N_SM_2 - state[0] + state[2];		// 17
//	v3 = v2*2-v1;
//	v4 = v2-v1*2;	// -
//	v5 = v1+v2;		// -
////	v3 = (sin_a8[cnt] + sin_b8[cnt] - sin_c8[cnt]*2) * N_SM_2 - state[0] - state[1] + state[2]*2;	// 30
////	v4 = (-sin_a8[cnt] + sin_b8[cnt]*2 - sin_c8[cnt]) * N_SM_2 + state[0] - state[1]*2 + state[2];
////	v5 = v3 - v4;
//	if (fabsf(v3) >= 1 || fabsf(v4) >= 1 || fabsf(v5) >= 1)	//	27
//	{
//		b1 = __f32_bits_as_u32(v3) & 0x80000000;
//		b2 = __f32_bits_as_u32(v4) & 0x80000000;
//		b3 = __f32_bits_as_u32(v5) & 0x80000000;
//		if (!b1)
//			if (b2)		// 11
//				state[0]++;		// 1
//			else if (b3)
//				state[1]++;		// 1
//			else
//				state[2]--;
//		else
//			if (!b2)		// 11
//				state[0]--;
//			else if (!b3)
//				state[1]--;
//			else
//				state[2]++;		// 1
////	if (fabsf(v5) >= 1 || fabsf(v3) >= 1 || fabsf(v4) >= 1)	// 51
////	{
////		if (v3 >= 0 && v4 < 0)
////			state[0]++;		// 1
////		else if (v4 >= 0 && v5 >= 0)
////			state[2]--;		// 2
////		else if (v5 < 0 && v3 >= 0)
////			state[1]++;
////		else if (v3 < 0 && v4 >= 0)
////			state[0]--;
////		else if (v4 < 0 && v5 < 0)
////			state[2]++;
////		else
////			state[1]--;		// 6
////
////		if (v2*2 >= v1 && v2 < v1*2)
////			state[0]++;		// 1
////		else if (v2 >= v1*2 && v1+v2 >= 0)
////			state[2]--;		// 2
////		else if (v1+v2 < 0 && v2*2 >= v1)
////			state[1]++;
////		else if (v2*2 < v1 && v2 >= v1*2)
////			state[0]--;
////		else if (v2 < v1*2 && v1+v2 < 0)
////			state[2]++;
////		else
////			state[1]--;		// 6
//		d1 = N_SM_2 - (state[0]+state[1]+state[2])*0.3333333333333333;	// 16
//		k = (int)(d1 >= 0 ? d1+0.5 : d1-0.5);	// 16
//		state[0] += k;			// 1
//		state[1] += k;
//		state[2] += k;
//	}

	v1 = (sin_a8[cnt] - sin_b8[cnt]) * N_SM_2 - f1;		// 19
	v2 = (sin_a8[cnt] - sin_c8[cnt]) * N_SM_2 - f2;		// 17
	v3 = v2*2-v1;
	v4 = v2-v1*2;	// -
	v5 = v1+v2;		// -
	if (fabsf(v3) >= 1 || fabsf(v4) >= 1 || fabsf(v5) >= 1)	//	27
	{
		b1 = __f32_bits_as_u32(v3) & 0x80000000;
		b2 = __f32_bits_as_u32(v4) & 0x80000000;
		b3 = __f32_bits_as_u32(v5) & 0x80000000;
		if (!b1)
		{
			if (b2)		// 11
			{
				f1++;
				f2++;
				state[0]++;
			}
			else if (b3)
			{
				f1--;
				state[1]++;
			}
			else
			{
				f2++;
				state[2]--;
			}
		}
		else
		{
			if (!b2)		// 11
			{
				f1--;
				f2--;
				state[0]--;
			}
			else if (!b3)
			{
				f1++;
				state[1]--;
			}
			else
			{
				f2--;
				state[2]++;
			}
		}
		d1 = N_SM_2 + 0.5 - (state[0]+state[1]+state[2])*0.3333333333333333;	// 8
		k = (int)(d1 >= 0.5 ? d1 : d1-1);	// 13
		state[0] += k;			// 1
		state[1] += k;
		state[2] += k;
//		v1 = N_SM_2 + 0.5 + (f1+f2)*0.3333333333333333;
//		state[0] = (int)(v1>=0.5? v1 : v1-1);			// 1
//		state[1] = state[0] - f1;		// 2
//		state[2] = state[0] - f2;		// 3
	}
#elif (method == 10)	// BJ120, 79-147
//	fsum = state[0] + state[1] + state[2];
//	v1 = sin_a8[cnt]*3 * N_SM_2 + state[1] + state[2] - state[0]*2;	// 34, -20 ~ +20
//	v2 = -sin_c8[cnt]*3 * N_SM_2 - state[0] - state[1] + state[2]*2;	// 30, -10 ~ +10
//	v3 = v2 - v1;
//	if (fabsf(v1) >= 1 || fabsf(v2) >= 1 || fabsf(v3) >= 1)	//	27
//		{
//			b1 = __f32_bits_as_u32(v1) & 0x80000000;
//			b2 = __f32_bits_as_u32(v2) & 0x80000000;
//			b3 = __f32_bits_as_u32(v3) & 0x80000000;
//			if (!b2)
//				if (b3)		// 11
//					state[0]++;		// 1
//				else if (b1)
//					state[1]++;		// 1
//				else
//					state[2]--;
//			else
//				if (!b3)		// 11
//					state[0]--;
//				else if (!b1)
//					state[1]--;
//				else
//					state[2]++;		// 1
////	if (fabsf(v1) >= 1 || fabsf(v2) >= 1 || fabsf(v2-v1) >= 1)
////	{
////		if (v2 >= 0 && v2 < v1)
////			state[0]++;		// 1
////		else if (v2 >= v1 && v1 >= 0)
////			state[2]--;		// 2
////		else if (v1 < 0 && v2 >= 0)
////			state[1]++;
////		else if (v2 < 0 && v2 >= v1)
////			state[0]--;
////		else if (v2 < v1 && v1 < 0)
////			state[2]++;
////		else
////			state[1]--;		// 6
//		d1 = N_SM_2 - (state[0] + state[1] + state[2])*0.3333333333333333;
//		k = (int)(d1 >= 0 ? d1+0.5 : d1-0.5);	// 12
//		state[0] += k;			// 1
//		state[1] += k;
//		state[2] += k;
//	}

	v1 = sin_a8[cnt]*3 * N_SM_2 - f1;	// 34, -20 ~ +20
	v2 = -sin_c8[cnt]*3 * N_SM_2 - f2;	// 30, -10 ~ +10
	v3 = v2 - v1;
	if (fabsf(v1) >= 1 || fabsf(v2) >= 1 || fabsf(v3) >= 1)	//	27
	{
		b1 = __f32_bits_as_u32(v1) & 0x80000000;
		b2 = __f32_bits_as_u32(v2) & 0x80000000;
		b3 = __f32_bits_as_u32(v3) & 0x80000000;
		if (!b2)
			if (b3)		// 11
			{
				f1+=2;
				f2++;
				state[0]++;
			}
			else if (b1)
			{
				f1--;
				f2++;
				state[1]++;
			}
			else
			{
				f1++;
				f2+=2;
				state[2]--;
			}
		else
			if (!b3)		// 11
			{
				f1-=2;
				f2--;
				state[0]--;
			}
			else if (!b1)
			{
				f1++;
				f2--;
				state[1]--;
			}
			else
			{
				f1--;
				f2-=2;
				state[2]++;
			}
		d1 = N_SM_2 + 0.5 - (state[0]+state[1]+state[2])*0.3333333333333333;	// 8
		k = (int)(d1 >= 0.5 ? d1 : d1-1);	// 13
		state[0] += k;			// 1
		state[1] += k;
		state[2] += k;
//		v1 = N_SM_2 + 0.5 + f1*0.3333333333333333;
//		state[0] = (int)(v1>=0.5 ? v1 : v1-1);			// 1
//		state[1] = state[0] - (f1*2-f2)*0.3333333333333333;
//		state[2] = state[0]*2 - f1 - state[1];		// 4
	}
#elif (method == 11)	// BJ60, 82-145
//	v1 = -sin_b8[cnt]*3 * N_SM_2 - state[0] + state[1]*2 - state[2];	// 11, -20 ~ +20
//	v2 = -sin_c8[cnt]*3 * N_SM_2 - state[0] - state[1] + state[2]*2;	// 11, -10 ~ +10
//	v3 = v1 + v2;
//	if (fabsf(v1) >= 1 || fabsf(v2) >= 1 || fabsf(v3) >= 1)	//	27
//	{
//		b1 = __f32_bits_as_u32(v1) & 0x80000000;
//		b2 = __f32_bits_as_u32(v2) & 0x80000000;
//		b3 = __f32_bits_as_u32(v3) & 0x80000000;
//		if (!b2)
//			if (!b1)		// 11
//				state[0]++;		// 1
//			else if (b3)
//				state[1]++;		// 1
//			else
//				state[2]--;
//		else
//			if (b1)		// 11
//				state[0]--;
//			else if (!b3)
//				state[1]--;
//			else
//				state[2]++;		// 1
////		d1 = N_SM_2+0.5 - fsum*0.3333333333333333;	// 8
////		k = (int)(d1 >= 0.5 ? d1 : d1-1);	// 13
////	if (fabsf(v1) >= 1 || fabsf(v2) >= 1 || fabsf(v1+v2) >= 1)
////	{
////		if (v2 >= 0 && v1 >= 0)
////			state[0]++;		// 1
////		else if (v1 < 0 && v1+v2 >= 0)
////			state[2]--;		// 2
////		else if (v1+v2 < 0 && v2 >= 0)
////			state[1]++;
////		else if (v2 < 0 && v1 < 0)
////			state[0]--;
////		else if (v1 >= 0 && v1+v2 < 0)
////			state[2]++;
////		else
////			state[1]--;		// 6
//		d1 = N_SM_2 - (state[0]+state[1]+state[2])*0.3333333333333333;
//		k = (int)(d1 >= 0 ? d1+0.5 : d1-0.5);	// 12
//		state[0] += k;			// 1
//		state[1] += k;
//		state[2] += k;
//	}

	v1 = -sin_b8[cnt]*3 * N_SM_2 - f1;	// 11, -20 ~ +20
	v2 = -sin_c8[cnt]*3 * N_SM_2 - f2;	// 11, -10 ~ +10
	v3 = v1 + v2;
	if (fabsf(v1) >= 1 || fabsf(v2) >= 1 || fabsf(v3) >= 1)	//	27
	{
		b1 = __f32_bits_as_u32(v1) & 0x80000000;
		b2 = __f32_bits_as_u32(v2) & 0x80000000;
		b3 = __f32_bits_as_u32(v3) & 0x80000000;
		if (!b2)
			if (!b1)		// 11
			{
				f1++;
				f2++;
				state[0]++;
			}
			else if (b3)
			{
				f1-=2;
				f2++;
				state[1]++;
			}
			else
			{
				f1--;
				f2+=2;
				state[2]--;
			}
		else
			if (b1)		// 11
			{
				f1--;
				f2--;
				state[0]--;
			}
			else if (!b3)
			{
				f1+=2;
				f2--;
				state[1]--;
			}
			else
			{
				f1++;
				f2-=2;
				state[2]++;
			}
		d1 = N_SM_2 + 0.5 - (state[0]+state[1]+state[2])*0.3333333333333333;	// 8
		k = (int)(d1 >= 0.5 ? d1 : d1-1);	// 13
		state[0] += k;			// 1
		state[1] += k;
		state[2] += k;
//		fsum = f1 + f2;
//		v1 = N_SM_2 + 0.5 + fsum*0.3333333333333333;
//		state[0] = (int)(v1>=0.5 ? v1 : v1-1);			// 1
//		state[1] = state[0]-(f1*2+f2)*0.3333333333333333;
//		state[2] = state[0]*2 - fsum - state[1];		// 5
	}

#elif (method == 12)	// ABC, 72-135, 118-172
//	fsum = state[0]+state[1]+state[2];		// 5
//	v1 = (sin_a8[cnt] * N_SM_2 - state[0]) * 3 + fsum;	// 15
//	v2 = (sin_b8[cnt] * N_SM_2 - state[1]) * 3 + fsum;	// 14
//	v3 = v1 + v2;		// 6
////	v3 = -v1 - v2;		// 6
////	if (v1)	// 7/11
////		v3 = -v1 - v2;
////	if (v2 >= 0)	// 7/10
////		v3 = -v1 - v2;
////	if (fsum >= 0)	// 5
////		v3 = -v1 - v2;
////	k = sgn(v1);
////	k = round(v1);	// 113
////	k = floor(v1);	// 69
////	k = lroundf(v1);	// 122
////	v3 = fminf(v1, v2);	// 70
////	x < y ? x : y;
////	v3 = modf(v1, &v2);	// 46
////	GET_FLOAT_WORD(b1, v1);	// 2
////	i = i & 0x80000000;	// 1
////	k = __FLOAT_SIGN_BIT_ZERO(v1);
////	k = b1
////	v2 = fabsf(v1);		// 3
////	v3 = (sin_c8[cnt] * N_SM_2 - state[2]) * 3 + fsum;
////*
//	if (fabsf(v1) >= 1 || fabsf(v2) >= 1 || fabsf(v3) >= 1)	//	27
//	{
//		b1 = __f32_bits_as_u32(v1) & 0x80000000;	// 3
//		b2 = __f32_bits_as_u32(v2) & 0x80000000;
//		b3 = __f32_bits_as_u32(v3) & 0x80000000;
//		if (!b3)
//		{
//			if (b2)		// 11
//			{
//				state[0]++;		// 1
//				fsum++;
//			}
//			else if (b1)
//			{
//				state[1]++;		// 1
//				fsum++;
//			}
//			else
//			{
//				state[2]--;
//				fsum--;
//			}
//		}
//		else
//		{
//			if (!b2)		// 11
//			{
//				state[0]--;
//				fsum--;
//			}
//			else if (!b1)
//			{
//				state[1]--;
//				fsum--;
//			}		// 6 */
//			else
//			{
//				state[2]++;		// 1
//				fsum++;
//			}
//		}
////		if (b3 & b2)		// 11
////		{
////			state[0]++;		// 1
////			fsum++;
////		}
////		else if (!(b2 | b1))	// 8/11
////		{
////			state[2]--;
////			fsum--;
////		}
////		else if (b1 & b3)
////		{
////			state[1]++;		// 1
////			fsum++;
////		}
////		else if (!(b3 | b2))
////		{
////			state[0]--;
////			fsum--;
////		}
////		else if (b2 & b1)
////		{
////			state[2]++;		// 1
////			fsum++;
////		}
////		else
////		{
////			state[1]--;
////			fsum--;
////		}		// 6 */
////		if (v3 < 0 && v2 < 0)		// 11
////			state[0]++;		// 1
////		else if (v2 >= 0 && v1 >= 0)
////			state[2]--;		// 2
////		else if (v1 < 0 && v3 < 0)
////			state[1]++;
////		else if (v3 >= 0 && v2 >= 0)
////			state[0]--;
////		else if (v2 < 0 && v1 < 0)
////			state[2]++;
////		else
////			state[1]--;		// 6 */
////*
////	if (v1 > v2)
////	{
////		maxv = v1;
////		maxi = 0;
////		minv = v2;
////		mini = 1;
////	}
////	else
////	{
////		maxv = v2;
////		maxi = 1;
////		minv = v1;
////		mini = 0;
////	}
////	if (v3 > maxv)
////	{
////		maxv = v3;
////		maxi = 2;
////	}
////	if (v3 < minv)
////	{
////		minv = v3;
////		mini = 2;
////	}
////	if (maxv >= 1 || minv <= -1)
////	{
////		if (maxv + minv >= 0)
////			state[maxi]++;
////		else
////			state[mini]--;	// */
////		fsum = b1 ^ b2 ^ b3 ? fsum-1 : fsum+1;	// 16
////		d1 = N_SM_2 - (state[0]+state[1]+state[2])*0.3333333333333333;	// 16
////		d1 = N_SM_2 - fsum*0.3333333333333333;	// 8
////		k = (int)(d1 >= 0 ? d1+0.5 : d1-0.5);	// 16
////		k = d1 >= 0.5 ? (int)d1 : (int)d1-1;	// 15
//		d1 = N_SM_2+0.5 - fsum*0.3333333333333333;	// 8
//		k = (int)(d1 >= 0.5 ? d1 : d1-1);	// 13
//		state[0] += k;			// 1
//		state[1] += k;
//		state[2] += k;
//	}

//	v1 = sin_a8[cnt] * N_SM_2 * 3 - f1;	// 15 slower
//	v2 = sin_b8[cnt] * N_SM_2 * 3 - f2;	// 14
//	v3 = v1 + v2;		// 6
//	if (fabsf(v1) >= 1 || fabsf(v2) >= 1 || fabsf(v3) >= 1)	//	27
//	{
//		b1 = __f32_bits_as_u32(v1) & 0x80000000;	// 3
//		b2 = __f32_bits_as_u32(v2) & 0x80000000;
//		b3 = __f32_bits_as_u32(v3) & 0x80000000;
//		if (!b3)
//		{
//			if (b2)		// 11
//			{
//				f1 += 2;		// 1
//				f2--;
//			}
//			else if (b1)
//			{
//				f2 += 2;		// 1
//				f1--;
//			}
//			else
//			{
//				f1++;
//				f2++;
//			}
//		}
//		else
//		{
//			if (!b2)		// 11
//			{
//				f1 -= 2;
//				f2++;
//			}
//			else if (!b1)
//			{
//				f2 -= 2;
//				f1++;
//			}
//			else
//			{
//				f1--;
//				f2--;
//			}
//		}
//		v1 = N_SM_2 + 0.5 + f1*0.3333333333333333;
//		state[0] = (int)(v1>=0.5? v1 : v1-1);			// 1
//		state[1] = state[0] + (f2-f1)*0.3333333333333333;
//		state[2] = state[0]*2 - f1 - state[1];		// 5
//	}

	v1 = sin_a8[cnt] * N_SM_2 * 3/8*8.66 - f1;	// 15 slower
	v2 = sin_b8[cnt] * N_SM_2 * 3/8*8.66 - f2;	// 14
	v3 = v1 + v2;		// 6
	if (fabsf(v1) >= 1 || fabsf(v2) >= 1 || fabsf(v3) >= 1)	//	27
	{
		b1 = __f32_bits_as_u32(v1) & 0x80000000;	// 3
		b2 = __f32_bits_as_u32(v2) & 0x80000000;
		b3 = __f32_bits_as_u32(v3) & 0x80000000;
		if (!b3)
		{
			if (b2)		// 11
			{
				f1 += 2;		// 1
				f2--;
				state[0]++;
			}
			else if (b1)
			{
				f2 += 2;		// 1
				f1--;
				state[1]++;
			}
			else
			{
				f1++;
				f2++;
				state[2]--;
			}
		}
		else
		{
			if (!b2)		// 11
			{
				f1 -= 2;
				f2++;
				state[0]--;
			}
			else if (!b1)
			{
				f2 -= 2;
				f1++;
				state[1]--;
			}
			else
			{
				f1--;
				f2--;
				state[2]++;
			}
		}
		d1 = N_SM_2 + 0.5 - (state[0]+state[1]+state[2])*0.3333333333333333;	// 8
		k = (int)(d1 >= 0.5 ? d1 : d1-1);	// 13
		state[0] += k;			// 1
		state[1] += k;
		state[2] += k;
	}

#elif (method == 13)	// Six, bad, 130-194
	fsum = state[0]+state[1]+state[2];
	v1 = (sin_a8[cnt] * N_SM_2 - state[0]) * 3 + fsum;	// 11, -20 ~ +20
	v3 = (sin_b8[cnt] * N_SM_2 - state[1]) * 3 + fsum;
	v2 = v1 + v3;
	v5 = -v2;
	v4 = -v1;
	v6 = -v3;
/*
 	if (abs(v1) >= 1 || abs(v2) >= 1 || abs(v3) >= 1)
	{
		if (v3 < 0 && v2 < 0)
			state[0]++;		// 1
		else if (v2 >= 0 && v1 >= 0)
			state[2]--;		// 2
		else if (v1 < 0 && v3 < 0)
			state[1]++;
		else if (v3 >= 0 && v2 >= 0)
			state[0]--;
		else if (v2 < 0 && v1 < 0)
			state[2]++;
		else
			state[1]--;		// 6 */
//*
	if (v1 > v2)
	{
		maxv = v1;
		maxi = 0;
	}
	else
	{
		maxv = v2;
		maxi = 5;
	}
	if (v3 > maxv)
	{
		maxv = v3;
		maxi = 2;
	}
	if (v4 > maxv)
	{
		maxv = v4;
		maxi = 1;
	}
	if (v5 > maxv)
	{
		maxv = v5;
		maxi = 4;
	}
	if (v6 > maxv)
	{
		maxv = v6;
		maxi = 3;
	}
	if (maxv >= 1)
	{
		if (maxi & 1)
		{
			state[maxi>>1]--;	// 12
		}
		else
			state[maxi>>1]++;	// 8 */
		d1 = N_SM_2 - (state[0]+state[1]+state[2])*0.3333333333333333;
		k = (int)(d1 >= 0 ? d1+0.5 : d1-0.5);	// 12
		state[0] += k;			// 1
		state[1] += k;
		state[2] += k;
	}

#elif (method == 14)	// origin, 181-182
	#define one_sqrt3	0.5773672
	v1 = (sin_a8[cnt]*2 - sin_b8[cnt] - sin_c8[cnt]) * 0.3333333333333333;	// 22
	v2 = (sin_b8[cnt] - sin_c8[cnt]) * one_sqrt3;				// 14
	v1 = v1 * N_SM_2 * 3;		// 6
	v2 = v2 * N_SM_2 * 1.732;	// 7
	f1 = (int)(v1>=0 ? v1+1 : v1-1);	// 17
	f2 = (int)(v2>=0 ? v2+1 : v2-1);	// 18
	f1 = 1;
	f2 = 1;
	if (v2 >= origin_table[(f1*2+f2)*6+0] * v1 + origin_table[(f1*2+f2)*6+1])	// 27
	{
		vs[0] = origin_table[(f1*2+f2)*6+2];	// 14
		vs[1] = origin_table[(f1*2+f2)*6+3];
	}
	else
	{
		vs[0] = origin_table[(f1*2+f2)*6+4];
		vs[1] = origin_table[(f1*2+f2)*6+5];
	}
	v1 = vs[0]*0.3333333333333333;
	state[0] = (int)(v1>=0 ? v1+0.5 : v1-0.5);			// 12
	state[1] = state[0] + (vs[0]-vs[1])/2;		// 8
	state[2] = state[0] - (vs[0]+vs[1])/2;		// 9

#elif (method == 15)	// SVC-rotate-xy, m=0.8, 11-Level, 97-104
	v2 = -sin_c8[cnt] * N_SM_2/8*8.66;				// 10, -10 ~ +10
	v1 = (sin_a8[cnt] - sin_b8[cnt]) * N_SM_2/8*8.66;	// 13, -20 ~ +20
	f1 = (int)(v1>=0 ? v1 : v1-1);	// 13
	f2 = (int)(v2>=0 ? v2 : v2-1);	// 13
	if ((f1 + f2) & 1)	// 6/9, odd
		if ((v2 - f2) * 3 < v1 - f1 + 1.0)	// 17
			f1++;
		else
			f2++;
	else	// even
		if ((v2 - f2) * 3 >= f1 + 2.0 - v1)	// 17
		{
			f1++;
			f2++;
		}
	state[0] = N_SM_2 + (f1+f2)/2;			// 8
	state[1] = state[0] - f1;		// 3
	state[2] = N_SM_2 - f2;		// 3

#endif
//	k = f1 > 0;		// 9
//	k = f1 >= 0;	// 8
//	k = f1 < 0;		// 10
//	k = f1 <= 0;	// 10

	samplecnt[cnt] = 5999 - CpuTimer1Regs.TIM.all;
	if (cnt==0)
	{
		tt = 0;
		minv = 1000;
		maxv = -1000;
		for (i=0;i<1000;i++)
		{
			tt += samplecnt[i];
			if (samplecnt[i] > maxv)	maxv = samplecnt[i];
			if (samplecnt[i] < minv)	minv = samplecnt[i];
		}
		tt /= 1000;
		tt *= 0.00333333333333333333333333;
		lt = maxv * 0.00333333333333333333333333;
		st = minv * 0.00333333333333333333333333;
	}
	minv = 10000;
	maxv = -10000;
	for (ph=0; ph<3; ph++)
	{
	   if (state[ph] > maxv)	maxv = state[ph];
	   if (state[ph] < minv)	minv = state[ph];
	}
	if (maxv > N_SM_2*2)
	{
	   state[0] -= (maxv - N_SM_2*2);
	   state[1] -= (maxv - N_SM_2*2);
	   state[2] -= (maxv - N_SM_2*2);
	}
	if (minv < 0)
	{
	   state[0] -= minv;
	   state[1] -= minv;
	   state[2] -= minv;
	}
	sampleb[cnt] = state[0];
	samplec[cnt] = state[2];
	samplecom[cnt] = (state[0] + state[1] + state[2])/3.0;
	cnt = cnt==1000 ? 0 : cnt+1;

	EALLOW;
// Acknowledge this interrupt to receive more interrupts from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
	EDIS;
}
#endif

void main(void)
{
// Step 1. Initialize System Control: PLL, WatchDog, enable Peripheral Clocks
   InitSysCtrl();
// Step 2. Initialize GPIO: illustrates how to set the GPIO to it's default state.
   InitXintf16Gpio();
// Step 3. Clear all interrupts and initialize PIE vector table: Disable CPU interrupts
   DINT;
   IER = 0x0000;
   IFR = 0x0000; 
// Initialize the PIE control registers to their default state. The default state is all PIE interrupts disabled and flags are cleared.
   InitPieCtrl();
   InitPieVectTable();
//   PieVectTable.EPWM1_INT = &epwm1_timer_isr;
#if !tmode
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.TINT0 = &cpu_timer0_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers
// Enable CPU int1 which is connected to CPU-Timer 0
   IER |= M_INT1;
// Enable TINT0 in the PIE: Group 1 interrupt 7
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
   spi_init();		  // 初始化SPI
   InitCpuTimers();
   ConfigCpuTimer(&CpuTimer0, 300, 20);           //50*1k=50KHz采样率
   ConfigCpuTimer(&CpuTimer1, 300, 20);           //50*1k=50KHz采样率
#endif

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

#if !tmode
   SetLOAD;	//把刷新锁存控制信号拉高
   CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0，开始定时器
   CpuTimer1Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0，开始定时器
#endif

   for(;;)
   {
#if tmode
	#if (method == 0)	// xy, 119/85/84
		v1 = (vabc[0]*2 - vabc[1] - vabc[2]) * N_SM_2;	// 11, -20 ~ +20
		v2 = (vabc[1] - vabc[2]) * N_SM_2;				// 8, -10 ~ +10
		f1 = (int)v1;			// 6
		f2 = (int)v2;			// 6
		if ((f1 + f2) & 1)	// 9, odd
			if ((v2 - f2) * 3 < v1 - f1 + 1.0)
				f1++;
			else
				f2++;
		else	// even
			if ((v2 - f2) * 3 >= f1 + 2.0 - v1)	// 17
			{
				f1++;
				f2++;
			}
		k = (int)(N_SM_2 + 0.5 + f1*0.3333333333333333);	// 12
		state[0] = k;			// 1
		state[1] = k - (f1-f2)/2;		// 9
		state[2] = state[1] - f2;		// 2

	#elif (method == 1)	// HD, 113
	   v1 = (vabc[0] - vabc[1]) * N_SM_2;		// 6
	   v2 = (vabc[0] - vabc[2]) * N_SM_2;		// 6
	   f1 = (int)v1;		// 6
	   f2 = (int)v2;		// 6
	   d1 = v1 - f1;		// 5
	   d2 = v2 - f2;		// 5
	   dsum = d1 + d2;
	   vo[0] = f1;
	   vo[1] = f2;
	   if (d1 >= d2)		// 9	inverted
	   {
		   if (d2 < d1/2 && d2 < d1*2-1)	// low low
			   vo[0]++;
		   else if (dsum >= 1 && d2 >= d1/2)	// right high
		   {
			   vo[0]++;
			   vo[1]++;
		   }
	   }
	   else		// regular
	   {
		   if (d2 >= (d1+1)*0.5 && d2 >= d1*2)	// high high
			   vo[1]++;
		   else if (dsum >= 1 && d2 < d1*2)	// right low
		   {
			   vo[0]++;
			   vo[1]++;
		   }
	   }
	   k = (int)(N_SM_2 + 0.5 + (vo[0]+vo[1])*0.3333333333333333);	// 19
	   state[0] = k;			// 1
	   state[1] = k - vo[0];		// 2
	   state[2] = k - vo[1];		// 3

// 加法无需重新赋值，故为1；
//	   for (ph=0; ph<100; ph++)	   f1 += 5;		// 1404	1
//	   for (ph=0; ph<100; ph++)	   f1 -= 5;		// 1504	2
//	   for (ph=0; ph<100; ph++)	   f1 *= 5;		// 1604	3
//	   for (ph=0; ph<100; ph++)	   f1 /= 5;		// 6090	47
//	   for (ph=0; ph<100; ph++)	   v1 /= 5.0;		// 15853	145
//	   for (ph=0; ph<100; ph++)	   f2 = 9;		// 2
//	   for (ph=0; ph<100; ph++)	   f2 = f1;		// 2
//	   for (ph=0; ph<100; ph++)	   f2 = state[0];	// 2
//	   for (ph=0; ph<100; ph++)	   f2 = state[0] - state[1];	// 3
//	   for (ph=0; ph<100; ph++)	   f2 = (state[0] - state[1]) * 5;	// 5
//	   for (ph=0; ph<100; ph++)	   f1 += f2;	// 1504	2
//	   for (ph=0; ph<100; ph++)	   f1 -= f2;	// 1504	2
//	   for (ph=0; ph<100; ph++)	   f1 *= f2;	// 1604	3
//	   for (ph=0; ph<100; ph++)	   f1 /= f2;	// 6104	47
//	   for (ph=0; ph<100; ph++)	   f1 = (int)v1;	// 1904	6
//	   for (ph=0; ph<100; ph++)	   d[1] = d2;		// 1504	2
//	   for (ph=0; ph<100; ph++)	   d[2] = -ddif;	// 1703	4
//	   for (ph=0; ph<100; ph++)	   d[0] = 1 - d[1] - d[2];	// 1904	6
	#elif (method == 2)	// BJ120, 117
		v1 = (vabc[0]*2 - vabc[1] - vabc[2]) * N_SM_2;	// 11, -20 ~ +20
		v2 = (vabc[0] + vabc[1] - vabc[2]*2) * N_SM_2;	// 11, -10 ~ +10
		f1 = (int)v1;			// 6
		f2 = (int)v2;			// 6
//		d1 = (f1+f2)/3.0;	// 242
//		d1 = (f1+f2)*0.333333333333;	// 14
//		k = (f1+f2)%3;	// 50
//		k = (f1+f2) - (f1+f2)/3 * 3;	// 55
		k = (f1+f2) - (int)((f1+f2)*0.3333333333333333) * 3;	// 21
		if (k == 1)	// 9
		{
			f1++;
			f2++;	// assure f1+f2=3k
		}
		else if (k == 2)	// 8
			if (v1 - f1 >= v2 - f2)	// 17
				f1++;
			else
				f2++;
		k = (int)(N_SM_2 + 0.5 + f1*0.3333333333333333);	// 12
		state[0] = k;			// 1
		state[1] = k - (f1*2-f2)*0.3333333333333333;		// 20
		state[2] = k*2 - f1 - state[1];		// 4

	#elif (method == 3)	// BJ60, 141
		v1 = (vabc[0] - vabc[1]*2 + vabc[2]) * N_SM_2;	// 11, -20 ~ +20
		v2 = (vabc[0] + vabc[1] - vabc[2]*2) * N_SM_2;	// 11, -10 ~ +10
		f1 = (int)v1;			// 6
		f2 = (int)v2;			// 6
		fsum = f1 + f2;
		k = (f2-f1) - (int)((f2-f1)*0.3333333333333333) * 3;	// 21
		if (k == 1)	// 9
			f1++;
		else if (k == 2)	// 8
			f2++;
		else
			if (v1 + v2 >= fsum + 1)	// 23
			{
				f1++;
				f2++;
			}
		k = (int)(N_SM_2 + 0.5 + fsum*0.3333333333333333);	// 19
		state[0] = k;			// 1
		state[1] = k - (f1*2+f2)*0.3333333333333333;		// 20
		state[2] = k*2 - fsum - state[1];		// 5

	#elif (method == 4)	// ABC, 140
		v1 = vabc[0] * 3 * N_SM_2;	// 11, -20 ~ +20
		v2 = vabc[1] * 3 * N_SM_2;
		v3 = vabc[2] * 3 * N_SM_2;
		f1 = (int)v1;			// 6
		f2 = (int)v2-1;			// 6
		f3 = (int)v3;
		temp[0] = f3+1 + f1*2;
		temp[1] = f1+1 + f2*2;
		temp[2] = temp[0] - (int)(temp[0]*0.3333333333333333) * 3;	// 21
		temp[3] = temp[1] - (int)(temp[1]*0.3333333333333333) * 3;	// 21
		if (temp[2] == 0)	// 9
			f2 = -f1 - f3 - 1;
		else if (temp[3] == 0)	// 8
			f1++;
		else
		{
			f1 = -f3 - f2 - 1;
			f2++;
		}
		k = (int)(N_SM_2 + 0.5 + f1*0.3333333333333333);	// 19
		state[0] = k;			// 1
		state[1] = k + (f2-f1)*0.3333333333333333;		// 20
		state[2] = k*2 - f1 - state[1];		// 5

	#elif (method == 5)	// Six, bad
		v1 = vabc[0] * 3 * N_SM_2;	// 11, -20 ~ +20
		v2 = vabc[1] * 3 * N_SM_2;
		v3 = vabc[2] * 3 * N_SM_2;
		f1 = (int)v1;			// 6
		f2 = (int)v2-1;			// 6
		f3 = (int)v3;
		temp[0] = f3+1 + f1*2;
		temp[1] = f1+1 + f2*2;
		temp[2] = temp[0] - (int)(temp[0]*0.3333333333333333) * 3;	// 21
		temp[3] = temp[1] - (int)(temp[1]*0.3333333333333333) * 3;	// 21
		if (temp[2] == 0)	// 9
			f2 = -f1 - f3 - 1;
		else if (temp[3] == 0)	// 8
			f1++;
		else
		{
			f1 = -f3 - f2 - 1;
			f2++;
		}
		k = (int)(N_SM_2 + 0.5 + f1*0.3333333333333333);	// 19
		state[0] = k;			// 1
		state[1] = k + (f2-f1)*0.3333333333333333;		// 20
		state[2] = k*2 - f1 - state[1];		// 5

	#elif (method == 6)	// Shifted HD, 86
		v1 = (vabc[0] - vabc[1]) * N_SM_2 + 0.3333333333333333;		// 6
		v2 = (vabc[0] - vabc[2]) * N_SM_2 + 0.6666666666666666;		// 6
		f1 = (int)v1;		// 6
		f2 = (int)v2;		// 6
		d1 = v1 - f1;		// 5
		d2 = v2 - f2;		// 5
		dsum = d1 + d2;
		if (d1 >= d2)	// 9
			if (dsum < 1 && d2*2 < d1)	// 8
				f2--;
			else if (dsum >= 1 && d2+1 < d1*2)
				f1++;
		k = (int)(N_SM_2 + 0.5 + (f1+f2)*0.3333333333333333);	// 19
		state[0] = k;			// 1
		state[1] = k - f1;		// 20
		state[2] = k - f2;		// 5

	#elif (method == 7)	// Shifted BJ60,
		v1 = (vabc[0] - vabc[1]*2 + vabc[2]) * N_SM_2 * 0.5 + 0.5;	// 11, -20 ~ +20
		v2 = (vabc[0] + vabc[1] - vabc[2]*2) * N_SM_2 * 0.5 + 0.5;
		f1 = (int)v1;		// 6
		f2 = (int)v2-1;		// 6
		d1 = v1 - f1;		// 5
		d2 = v2 - f2;		// 5
		dsum = d1 + d2;
		k = (f1-f2) - (int)((f1-f2)*0.3333333333333333) * 3;	// 21
		ff1 = f1;
		ff2 = f2;
		if (k == 0)	// 9
		{
			if (dsum >= 1.5)	// 8
			{
				ff1++;
				ff2++;
			}
			else if (dsum >= 0.5)
			{
				ff1 = ff1 + 0.5;
				ff2 = ff2 + 0.5;
			}
		}
		else if (k == 1)
		{
			if (dsum < 1 && d2 < 0.5)
			{
				ff1 = ff1 + 0.5;
			}
			else if (dsum >= 1 && d1 >= 0.5)
			{
				ff1++;
				ff2 = ff2 + 0.5;
			}
			else
			{
				ff2++;
			}
		}
		else
		{
			if (dsum < 1 && d2 < 0.5)
			{
				ff2 = ff2 + 0.5;
			}
			else if (dsum >= 1 && d2 >= 0.5)
			{
				ff1 = ff1 + 0.5;
				ff2++;
			}
			else
			{
				ff1++;
			}
		}
		f1 = ff1 * 2 - 1;
		f2 = ff2 * 2 - 1;
		fsum = f1 + f2;
		k = (int)(N_SM_2 + 0.5 + fsum*0.3333333333333333);	// 19
		state[0] = k;			// 1
		state[1] = k - (f1*2+f2)*0.3333333333333333;		// 20
		state[2] = k*2 - fsum - state[1];		// 5

	#elif (method == 8)	// xy,
		v1 = (vabc[0]*2 - vabc[1] - vabc[2]) * N_SM_2;	// 11, -20 ~ +20
		v2 = (vabc[1] - vabc[2]) * N_SM_2;				// 8, -10 ~ +10
		f1 = (int)v1;			// 6
		f2 = (int)v2;			// 6
		if ((f1 + f2) & 1)	// 9, odd
			if ((v2 - f2) * 3 < v1 - f1 + 1.0)
				f1++;
			else
				f2++;
		else	// even
			if ((v2 - f2) * 3 >= f1 + 2.0 - v1)	// 17
			{
				f1++;
				f2++;
			}
		k = (int)(N_SM_2 + 0.5 + f1*0.3333333333333333);	// 12
		state[0] = k;			// 1
		state[1] = k - (f1-f2)/2;		// 9
		state[2] = state[1] - f2;		// 2

	#elif (method == 9)	// HD,
	   v1 = (vabc[0] - vabc[1]) * N_SM_2;		// 6
	   v2 = (vabc[0] - vabc[2]) * N_SM_2;		// 6
	   f1 = (int)v1;		// 6
	   f2 = (int)v2;		// 6
	   d1 = v1 - f1;		// 5
	   d2 = v2 - f2;		// 5
	   dsum = d1 + d2;
	   vo[0] = f1;
	   vo[1] = f2;
	   if (d1 >= d2)		// 9	inverted
	   {
		   if (d2 < d1/2 && d2 < d1*2-1)	// low low
			   vo[0]++;
		   else if (dsum >= 1 && d2 >= d1/2)	// right high
		   {
			   vo[0]++;
			   vo[1]++;
		   }
	   }
	   else		// regular
	   {
		   if (d2 >= (d1+1)*0.5 && d2 >= d1*2)	// high high
			   vo[1]++;
		   else if (dsum >= 1 && d2 < d1*2)	// right low
		   {
			   vo[0]++;
			   vo[1]++;
		   }
	   }
	   k = (int)(N_SM_2 + 0.5 + (vo[0]+vo[1])*0.3333333333333333);	// 19
	   state[0] = k;			// 1
	   state[1] = k - vo[0];		// 2
	   state[2] = k - vo[1];		// 3

	// 加法无需重新赋值，故为1；
	//	   for (ph=0; ph<100; ph++)	   f1 += 5;		// 1404	1
	//	   for (ph=0; ph<100; ph++)	   f1 -= 5;		// 1504	2
	//	   for (ph=0; ph<100; ph++)	   f1 *= 5;		// 1604	3
	//	   for (ph=0; ph<100; ph++)	   f1 /= 5;		// 6090	47
	//	   for (ph=0; ph<100; ph++)	   v1 /= 5.0;		// 15853	145
	//	   for (ph=0; ph<100; ph++)	   f2 = 9;		// 2
	//	   for (ph=0; ph<100; ph++)	   f2 = f1;		// 2
	//	   for (ph=0; ph<100; ph++)	   f2 = state[0];	// 2
	//	   for (ph=0; ph<100; ph++)	   f2 = state[0] - state[1];	// 3
	//	   for (ph=0; ph<100; ph++)	   f2 = (state[0] - state[1]) * 5;	// 5
	//	   for (ph=0; ph<100; ph++)	   f1 += f2;	// 1504	2
	//	   for (ph=0; ph<100; ph++)	   f1 -= f2;	// 1504	2
	//	   for (ph=0; ph<100; ph++)	   f1 *= f2;	// 1604	3
	//	   for (ph=0; ph<100; ph++)	   f1 /= f2;	// 6104	47
	//	   for (ph=0; ph<100; ph++)	   f1 = (int)v1;	// 1904	6
	//	   for (ph=0; ph<100; ph++)	   d[1] = d2;		// 1504	2
	//	   for (ph=0; ph<100; ph++)	   d[2] = -ddif;	// 1703	4
	//	   for (ph=0; ph<100; ph++)	   d[0] = 1 - d[1] - d[2];	// 1904	6
	#elif (method == 10)	// BJ120,
		v1 = (vabc[0]*2 - vabc[1] - vabc[2]) * N_SM_2;	// 11, -20 ~ +20
		v2 = (vabc[0] + vabc[1] - vabc[2]*2) * N_SM_2;	// 11, -10 ~ +10
		f1 = (int)v1;			// 6
		f2 = (int)v2;			// 6
	//		d1 = (f1+f2)/3.0;	// 242
	//		d1 = (f1+f2)*0.333333333333;	// 14
	//		k = (f1+f2)%3;	// 50
	//		k = (f1+f2) - (f1+f2)/3 * 3;	// 55
		k = (f1+f2) - (int)((f1+f2)*0.3333333333333333) * 3;	// 21
		if (k == 1)	// 9
		{
			f1++;
			f2++;	// assure f1+f2=3k
		}
		else if (k == 2)	// 8
			if (v1 - f1 >= v2 - f2)	// 17
				f1++;
			else
				f2++;
		k = (int)(N_SM_2 + 0.5 + f1*0.3333333333333333);	// 12
		state[0] = k;			// 1
		state[1] = k - (f1*2-f2)*0.3333333333333333;		// 20
		state[2] = k*2 - f1 - state[1];		// 4

	#elif (method == 11)	// BJ60,
		v1 = (vabc[0] - vabc[1]*2 + vabc[2]) * N_SM_2;	// 11, -20 ~ +20
		v2 = (vabc[0] + vabc[1] - vabc[2]*2) * N_SM_2;	// 11, -10 ~ +10
		f1 = (int)v1;			// 6
		f2 = (int)v2;			// 6
		fsum = f1 + f2;
		k = (f2-f1) - (int)((f2-f1)*0.3333333333333333) * 3;	// 21
		if (k == 1)	// 9
			f1++;
		else if (k == 2)	// 8
			f2++;
		else
			if (v1 + v2 >= fsum + 1)	// 23
			{
				f1++;
				f2++;
			}
		k = (int)(N_SM_2 + 0.5 + fsum*0.3333333333333333);	// 19
		state[0] = k;			// 1
		state[1] = k - (f1*2+f2)*0.3333333333333333;		// 20
		state[2] = k*2 - fsum - state[1];		// 5

	#elif (method == 12)	// ABC,
		v1 = vabc[0] * 3 * N_SM_2;	// 11, -20 ~ +20
		v2 = vabc[1] * 3 * N_SM_2;
		v3 = vabc[2] * 3 * N_SM_2;
		f1 = (int)v1;			// 6
		f2 = (int)v2-1;			// 6
		f3 = (int)v3;
		temp[0] = f3+1 + f1*2;
		temp[1] = f1+1 + f2*2;
		temp[2] = temp[0] - (int)(temp[0]*0.3333333333333333) * 3;	// 21
		temp[3] = temp[1] - (int)(temp[1]*0.3333333333333333) * 3;	// 21
		if (temp[2] == 0)	// 9
			f2 = -f1 - f3 - 1;
		else if (temp[3] == 0)	// 8
			f1++;
		else
		{
			f1 = -f3 - f2 - 1;
			f2++;
		}
		k = (int)(N_SM_2 + 0.5 + f1*0.3333333333333333);	// 19
		state[0] = k;			// 1
		state[1] = k + (f2-f1)*0.3333333333333333;		// 20
		state[2] = k*2 - f1 - state[1];		// 5

	#elif (method == 13)	// Six,
		v1 = vabc[0] * 3 * N_SM_2;	// 11, -20 ~ +20
		v2 = vabc[1] * 3 * N_SM_2;
		v3 = vabc[2] * 3 * N_SM_2;
		f1 = (int)v1;			// 6
		f2 = (int)v2-1;			// 6
		f3 = (int)v3;
		temp[0] = f3+1 + f1*2;
		temp[1] = f1+1 + f2*2;
		temp[2] = temp[0] - (int)(temp[0]*0.3333333333333333) * 3;	// 21
		temp[3] = temp[1] - (int)(temp[1]*0.3333333333333333) * 3;	// 21
		if (temp[2] == 0)	// 9
			f2 = -f1 - f3 - 1;
		else if (temp[3] == 0)	// 8
			f1++;
		else
		{
			f1 = -f3 - f2 - 1;
			f2++;
		}
		k = (int)(N_SM_2 + 0.5 + f1*0.3333333333333333);	// 19
		state[0] = k;			// 1
		state[1] = k + (f2-f1)*0.3333333333333333;		// 20
		state[2] = k*2 - f1 - state[1];		// 5
	#endif
#endif
   }
}

#if !tmode
void WriteDAC(unsigned char add,unsigned char rng,unsigned char vol)
{
	unsigned short int data;
    data=0x0000;
    ///大家要知道这里所定义的各个变量的含义,add是4个通道的地址（00，01，10，11）
    ///                                     RNG是输出范围的倍数，可以是0或1。
    ///                                     VOL是0~256数据

     data = ((add<<14) | (rng<<13) | (vol<<5));
     //注意这里的有效数据是11位，SPI初始化中也进行了定义

    while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG ==1);			//判断SPI的发送缓冲区是否是空的,等于0可写数据

       SpiaRegs.SPITXBUF = data;	//把发送的数据写如SPI发送缓冲区

    while( SpiaRegs.SPISTS.bit.BUFFULL_FLAG==1);		//当发送缓冲区出现满标志位时,开始琐存数据

	delay(5);//	同通过一负跳变琐存要发送的数据,看TLV5620数据手册即可得知, min=50ns
    ClrLOAD;
	delay(24);	// min = 250ns
    SetLOAD;
}

void delay(unsigned int t)
{
 	while(t>0)
    	t--;
}

//初始化SPI函数
void spi_init()
{
	SpiaRegs.SPICCR.all =0x0a;///进入初始状态，数据在上升沿输出，自测禁止，11位数据模式
	SpiaRegs.SPICTL.all =0x0006; // 使能主机模式，正常相位，使能主机发送，禁止接收, 溢出中断，禁止SPI中断；
	SpiaRegs.SPIBRR =0x00C7;	//SPI波特率=37.5M/50	=0.75MHZ；150M/200=0.75M
    SpiaRegs.SPICCR.all =0x8a; //退出初始状态；
    SpiaRegs.SPIPRI.bit.FREE = 1;  // 自由运行
}
#endif
