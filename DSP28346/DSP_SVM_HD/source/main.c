#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Sintable.h"

#define tmode	0	// tmode is for runtime test
#define method	0	// 0=HD, 1=gh, 2=abbcca, 3=H3D3, 4=YD, 5=NLC, 6=SVC, 7=line-NLC, 8=zeroSVC, -1=zeroSVM, 9=PSPWM, -2=simple DY
#define rsf		0	// 0=normal, 1=conventional, 2=mymethod
#define	sin60	0.8660254
#define	sqrt3	1.7320508

#if !tmode
	#define PWM1_INT_ENABLE  0
	#define PWM2_INT_ENABLE  0
	#define PWM3_INT_ENABLE	 0
	#define k2	 0
	#define k4	 0
#if method == 0
	#define PWM_TIMER_TBPRD	 25000	// f = 1 / (2 * 62500(0xF424) * tbclk(150M)) = 1.2kHz, 1 / (2 * 46875 * tbclk(75M)) = 800Hz, 25000 = 1500Hz
#elif method == 1
	#define PWM_TIMER_TBPRD	 25000	// f = 1 / (2 * 46875 * tbclk(75M)) = 800Hz, 25000 = 1500Hz
#elif method == 2
	#define PWM_TIMER_TBPRD	 25000	// f = 1 / (2 * 46875 * tbclk(75M)) = 800Hz
#elif method == 3
	#define PWM_TIMER_TBPRD	 25000	// 41666.6=900Hz
#elif method == 4
	int table[6][3] = {{0,1,2}, {1,0,2}, {1,2,0}, {2,1,0}, {2,0,1}, {0,2,1}};
	#define PWM_TIMER_TBPRD	 25000	// 7500 = 5kHz
#elif method == -1
	#define PWM_TIMER_TBPRD	 50000/2	// 1.5kHz
#elif method == -2
	#define PWM_TIMER_TBPRD	 25000	// 1.5kHz
#endif
	void InitEPwmTimer(void);
	#define SetLOAD GpioDataRegs.GPBDAT.bit.GPIO55=1;	//将LOAD置高
	#define ClrLOAD GpioDataRegs.GPBDAT.bit.GPIO55=0;	//将LOAD置低
	void WriteDAC(unsigned char add,unsigned char rng,unsigned char vol);
	void delay(unsigned int t);
	void spi_xmit(Uint16 a);
	void spi_fifo_init(void);
	void spi_init(void);
	const int rotate[] = {0, 1, 0, -1};
#endif

#define 	N_SM_2		2

//int	abcpn[6], abcpn_pre[6], pre[3]={0,0,0}, r_cnt[3]={0,0,0}, sdc[]={0,0,0}, pos, sm_num[6][N_SM_2*2], intvals[N_SM_2*2], sub[6][N_SM_2*2];
//float vdif2[9], vdif4[9], idif[6], iabcpn[6], vdif[3], vc[6][N_SM_2*2];
float	vrx, vry, vh, vd, vt, vx, vy, dh, dd, dt, ddif, minf, tt, lt, st, tnow, d[3], D[5]={0,0,0,0,0}, freq, tempf[]={0,0,0,0,0}, vabc[] = {0.6108, -1.034, 0.4227, 0.6108, -1.034}, sin[]={0, sin60, sin60, 0, -sin60, -sin60, 0}, cos[]={1, 0.5, -0.5, -1, -0.5, 0.5, 1};
int		i, j, ch, cd, ct, cmp, ph, fh, fd, ft, ct, k, cnt=0, reg, state[3]={0,0,0}, intvals[N_SM_2*2], temp[]={0,0,0,0,0}, minv=1000, maxv=-1000, samplea[1000], sampleb[1000], samplec[1000], samplecnt[1000], compare[3];
short	mode=0;
float	samplecom[1000];


#if !tmode
interrupt void cpu_timer0_isr(void)
{
	CpuTimer1Regs.TIM.all = 5999;
#if (method == 0)	// HD, 5-Level, 1500Hz, m=0.7, 196-206, 0.6706
	if (cnt == 0)
		EPwm1Regs.TBCTR = PWM_TIMER_TBPRD; 				// reset TB counter
	vh = (sin_a[cnt] - sin_b[cnt]) * N_SM_2/7*10;		// 15
	vd = (sin_a[cnt] - sin_c[cnt]) * N_SM_2/7*10;		// 13
	fh = (int)(vh>=0 ? vh : vh-1);		// 16
	fd = (int)(vd>=0 ? vd : vd-1);		// 16
	dh = vh - fh;		// 5
	dd = vd - fd;		// 5
	if (dh >= dd)		// 9, inverted, ACB
	   D[0] = (1 + dh)*0.5;
	else					// ABC
	   D[0] = (1 + dd)*0.5;
    D[1] = D[0] - dh;
    D[2] = D[0] - dd;

	tempf[0] = N_SM_2 + (fh+fd)*0.3333333333333333;		//15, may < 0
	state[0] = (int)(tempf[0]>=0 ? tempf[0] : tempf[0]-1);			// 15
	state[1] = state[0] - fh;		// 2
	state[2] = state[0] - fd;		// 3
	cmp = EPwm1Regs.TBCTR;
	if (PWM_TIMER_TBPRD * D[0] >= cmp)
		state[0]++;
	if (PWM_TIMER_TBPRD * D[1] >= cmp)
		state[1]++;
	if (PWM_TIMER_TBPRD * D[2] >= cmp)
		state[2]++;
//	tempf[0] = N_SM_2 - (state[0]+state[1]+state[2])*0.3333333333333333;		// may < 0
//	k = (int)(tempf[0]>=0 ? tempf[0]+0.5 : tempf[0]-0.5);	// 20
//	state[0] += k;
//	state[1] += k;
//	state[2] += k;

	samplecnt[cnt] = 5999 - CpuTimer1Regs.TIM.all;
	if (cnt==999)
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
	minv = 1000;
	maxv = -1000;
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
	samplea[cnt] = state[0];
	sampleb[cnt] = state[1];
	samplec[cnt] = state[2];
	samplecom[cnt] = (state[0]+state[1]+state[2])/3.0;
	cnt = cnt==999 ? 0 : cnt+1;

#elif (method == 1)	// gh, 5-Level, 800Hz, m=0.7, 218-231, 0.7467
	if (cnt == 0)
		EPwm1Regs.TBCTR = PWM_TIMER_TBPRD; // clear TB counter
	vh = (sin_a[cnt] - sin_b[cnt]) * N_SM_2;		// 15
	vd = (sin_b[cnt] - sin_c[cnt]) * N_SM_2;		// 13
//	fh = (int)(vh>=0 ? vh+1 : vh);		// 16
//	fd = (int)(vd>=0 ? vd : vd-1);		// 16
//	dh = vh - fh;
//	dd = vd - fd;
//	if (dh + dd < 0)	// regular, ABC
//	{
//	   fh--;
//	   d[1] = dh + 1;
//	   d[2] = dd;
//	   d[0] = 1 - d[1] - d[2];
//	   D[0] = 1 - d[0]*0.5;	// 5
//	   D[1] = D[0] - d[1];	// 6
//	   D[2] = D[1] - d[2];	// 6
//	}
//	else	// inverted, CBA
//	{
//	   fd++;
//	   d[1] = 1 - dd;
//	   d[2] = -dh;
//	   d[0] = 1 - d[1] - d[2];
//	   D[2] = 1 - d[0]*0.5;	// 5
//	   D[1] = D[2] - d[1];	// 6
//	   D[0] = D[1] - d[2];	// 6
//	}
	fh = (int)(vh>=0 ? vh : vh-1);		// 16
	fd = (int)(vd>=0 ? vd : vd-1);		// 16
	if (vh + vd >= fh + fd + 1)	// inverted, CBA
	{
	   fh++;
	   fd++;
		dh = vh - fh;
		dd = vd - fd;
	   D[1] = (1+dh-dd)*0.5;	// 6
	}
	else	// regular, ABC
	{
		dh = vh - fh;
		dd = vd - fd;
	   D[1] = (1-dh+dd)*0.5;	// 6
	}
	D[0] = (1+dh+dd)*0.5;
	D[2] = (1-dh-dd)*0.5;
	tempf[0] = N_SM_2 + (fh*2+fd)*0.3333333333333333;
	k = (int)(tempf[0]>=0 ? tempf[0] : tempf[0]-1);
	state[0] = k;
	state[1] = k - fh;
	state[2] = state[1] - fd;

	cmp = EPwm1Regs.TBCTR;
	if (PWM_TIMER_TBPRD * D[0] >= cmp)
		state[0]++;
	if (PWM_TIMER_TBPRD * D[1] >= cmp)
		state[1]++;
	if (PWM_TIMER_TBPRD * D[2] >= cmp)
		state[2]++;

	samplecnt[cnt] = 5999 - CpuTimer1Regs.TIM.all;
	if (cnt==999)
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
	minv = 1000;
	maxv = -1000;
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
//	WriteDAC(0, 0, state[0]*0.25*256 - 1);
	samplea[cnt] = state[0];
	sampleb[cnt] = state[1];
	samplec[cnt] = state[2];
	cnt = cnt==1000 ? 0 : cnt+1;

#elif (method == 2)	// abbcca, 5-Level, 800Hz, m=0.7, 232-241, 0.7889
	if (cnt == 0)
		EPwm1Regs.TBCTR = PWM_TIMER_TBPRD; // clear TB counter
	vh = (sin_a[cnt] - sin_b[cnt]) * N_SM_2;		// 15
	vd = (sin_b[cnt] - sin_c[cnt]) * N_SM_2;		// 13
	vt = -vh - vd;		// 13
//	fh = (int)(vh>=0 ? vh : vh-1);		// 16
//	fd = (int)(vd>=0 ? vd : vd-1);		// 16
//	ft = (int)(vt>=0 ? vt+1 : vt);		// 16
//	if (fh + fd + ft == 0)		// regular, ABC
//	{
//		d[1] = vh - fh;	// cff
//		d[2] = vd - fd;
//		d[0] = 1 - d[1] - d[2];	// 6, ffc
//		D[0] = 1 - d[0]*0.5;	// 5
//		D[1] = D[0] - d[1];	// 6
//		D[2] = D[1] - d[2];	// 6
//	}
//	else		// inverted, ACB
//	{
//	   d[1] = ft - vt;		// ccf
//	   d[2] = fd + 1 - vd;
//	   d[0] = 1 - d[1] - d[2];	// fcc
//	   D[0] = 1 - d[0]*0.5;
//	   D[2] = D[0] - d[1];
//	   D[1] = D[2] - d[2];
//	}
//	tempf[0] = N_SM_2 + (fh-ft)*0.3333333333333333;
//	k = (int)(tempf[0]>=0 ? tempf[0] : tempf[0]-1);
//	state[0] = k;
//	state[1] = k - fh;
//	state[2] = k + ft;
	fh = (int)(vh>=0 ? vh : vh-1);		// 16
	fd = (int)(vd>=0 ? vd : vd-1);		// 16
	ct = (int)(vt>=0 ? vt+1 : vt);		// 16
	cd = fd + 1;
	dh = vh - fh;
	dt = vt - ct;
	if (fh + fd + ct)		// inverted, ACB
	{
		dd = vd - cd;
		D[0] = (1+dh)*0.5;
		D[1] = (1-dh)*0.5;
		D[2] = (1-dd+dt)*0.5;
	}
	else		// regular, ABC
	{
		dd = vd - fd;
		D[0] = (1-dt)*0.5;
		D[1] = (1-dh+dd)*0.5;
		D[2] = (1+dt)*0.5;
	}
	tempf[0] = N_SM_2 + (fh-ct)*0.3333333333333333;
	state[0] = (int)(tempf[0]>=0 ? tempf[0] : tempf[0]-1);
	state[1] = state[0] - fh;
	state[2] = state[0] + ct;
	cmp = EPwm1Regs.TBCTR;
	if (PWM_TIMER_TBPRD * D[0] >= cmp)
		state[0]++;
	if (PWM_TIMER_TBPRD * D[1] >= cmp)
		state[1]++;
	if (PWM_TIMER_TBPRD * D[2] >= cmp)
		state[2]++;
	samplecnt[cnt] = 5999 - CpuTimer1Regs.TIM.all;

	if (cnt==999)
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
	minv = 1000;
	maxv = -1000;
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
//	WriteDAC(0, 0, state[0]*0.25*256 - 1);
	samplea[cnt] = state[0];
	sampleb[cnt] = state[1];
	samplec[cnt] = state[2];
	cnt = cnt==1000 ? 0 : cnt+1;


#elif (method == 3)	// SVM-H3D3, 5-Level, 1500Hz， m=0.7, 79-284-290, 0.9557
	if (cnt == 0)
		EPwm1Regs.TBCTR = PWM_TIMER_TBPRD; // clear TB counter
	tempf[0] = sin_a[cnt]/7*8.66;			// 8 7 7 2 2
	tempf[1] = sin_b[cnt]/7*8.66;
	tempf[2] = -tempf[0] - tempf[1];
	tempf[3] = tempf[0];
	tempf[4] = tempf[1];
	vh = (tempf[mode] - tempf[mode+1]) * N_SM_2;		// 16
//	if (vh < 0)		// 9/15
//	{
//	   mode = mode==2 ? 0 : mode+1;
//	   vh = (tempf[mode] - tempf[mode+1]) * N_SM_2;
//	}
	if (vh >= 0)
	{
	vd = (tempf[mode] - tempf[mode+2]) * N_SM_2;		// 15
	fh = (int)vh;	// 6, all >= 0
	fd = (int)vd;
	dh = vh - fh;		// 5
	dd = vd - fd;		// 5
//	ddif = dh - dd;		// 4
//	if (ddif > 0)		// 6/9, inverted
//	{
//	   d[1] = dd;			// 2 2 6 6 12 14
//	   d[2] = ddif;
//	   d[0] = 1 - d[1] - d[2];
//	   D[mode] = 1 - d[0]*0.5;		// shift right
//	   D[mode+2] = D[mode] - d[1];
//	   D[mode+1] = D[mode+2] - d[2];
//	}
//	else
//	{
//	   d[1] = dh;		// 2
//	   d[2] = -ddif;	// 2
//	   d[0] = 1 - d[1] - d[2];
//	   D[mode] = 1 - d[0]*0.5;		// shift left
//	   D[mode+1] = D[mode] - d[1];	// 16
//	   D[mode+2] = D[mode+1] - d[2];	// 18
//	}
	if (dh >= dd)		// 9, inverted, ACB
	   D[mode] = (1 + dh)*0.5;
	else					// ABC
	   D[mode] = (1 + dd)*0.5;
    D[mode+1] = D[mode] - dh;
    D[mode+2] = D[mode] - dd;

	k = (int)(N_SM_2 + (fh+fd)*0.3333333333333333);	// 20
	temp[mode] = k;			// 7 11 11 3 3 2
	temp[mode+1] = k - fh;
	temp[mode+2] = k - fd;
	state[0] = temp[0] + temp[3];
	state[1] = temp[1] + temp[4];
	state[2] = temp[2];

	cmp = EPwm1Regs.TBCTR;
	if (PWM_TIMER_TBPRD * (D[0]+D[3]) >= cmp)
		state[0]++;
	if (PWM_TIMER_TBPRD * (D[1]+D[4]) >= cmp)
		state[1]++;
	if (PWM_TIMER_TBPRD * D[2] >= cmp)
		state[2]++;
	}
	else
		mode = mode==2 ? 0 : mode+1;

	samplecnt[cnt] = 5999 - CpuTimer1Regs.TIM.all;
	if (cnt==999)
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
	for (i=0;i<5;i++)
	{
		D[i] = 0;
		temp[i] = 0;
	}
	minv = 1000;
	maxv = -1000;
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
//	WriteDAC(0, 0, state[0]*0.25*256 - 1);
	samplea[cnt] = state[0];
	sampleb[cnt] = state[1];
	samplec[cnt] = state[2];
	cnt = cnt==1000 ? 0 : cnt+1;

#elif (method == 4)	// DY, 5-Level, 800Hz, m=0.7, 683-885, 2.8262
	if (cnt == 0)
		EPwm1Regs.TBCTR = 0; // clear TB counter
	vh = sin_a[cnt] - sin_b[cnt]*0.5 - sin_c[cnt]*0.5;		// 13
	vd = (sin_b[cnt] - sin_c[cnt]) * 0.866;		// 7
	vx = vh * N_SM_2;
	vy = vd * N_SM_2 * 0.577;
	if (vy >= vx)		// 12
	   minf = vx;
	else
	   minf = vy;
	if (-vy < minf)	minf = -vy;		// 11
	state[0] = (int)(vx-minf);		// 9
	state[1] = (int)(vy-minf);
	state[2] = (int)(-vy-minf);	// 10
	k = (int)(N_SM_2 - (state[0]+state[1]+state[2])*0.3333333333333333);	// 20
	state[0] += k;		// 1
	state[1] += k;
	state[2] += k;		// derive the nearest vector

	vrx = vh - (state[0] - state[1]*0.5 - state[2]*0.5)/N_SM_2;	// 12
	vry = vd - (state[1] + state[2])*0.866/N_SM_2;				// derive remainder vector
	if (0 <= vry && vry < sqrt3 * vrx)	reg = 1;	// 66
	else if (vry >= sqrt3 * vrx && vry > -sqrt3 * vrx)	reg = 2;
	else if (0 < vry && vry <= -sqrt3 * vrx)	reg = 3;
	else if (sqrt3 * vrx < vry && vry <= 0)	reg = 4;
	else if (vry <= sqrt3 * vrx && vry < -sqrt3 * vrx)	reg = 5;
	else reg = 6;
	d[1] = 1.1547 * (vrx * sin[reg] - vry * cos[reg]);			// 17
	d[2] = -1.1547 * (vrx * sin[reg-1] - vry * cos[reg-1]);	// 21
	d[0] = 1 - d[1] - d[2];		// 6
	if (reg == 1)
	{
		D[0] = 1 - d[0]*0.5;
		D[1] = D[0] - d[1];
		D[2] = D[1] - d[2];
	}
	else if (reg == 2)
	{
		D[2] = d[0]*0.5;
		D[0] = D[2] + d[1];
		D[1] = D[0] + d[2];
	}
	else if (reg == 3)
	{
		D[1] = 1 - d[0]*0.5;
		D[2] = D[1] - d[1];
		D[0] = D[2] - d[2];
	}
	else if (reg == 4)
	{
		D[0] = d[0]*0.5;
		D[1] = D[0] + d[1];
		D[2] = D[1] + d[2];
	}
	else if (reg == 5)
	{
		D[2] = 1 - d[0]*0.5;
		D[0] = D[2] - d[1];
		D[1] = D[0] - d[2];
	}
	else
	{
		D[1] = d[0]*0.5;
		D[2] = D[1] + d[1];
		D[0] = D[2] + d[2];
	}

	cmp = EPwm1Regs.TBCTR;
	if (PWM_TIMER_TBPRD * D[0] >= cmp)
		state[0]++;
	if (PWM_TIMER_TBPRD * D[1] >= cmp)
		state[1]++;
	if (PWM_TIMER_TBPRD * D[2] >= cmp)
		state[2]++;

	samplecnt[cnt] = 5999 - CpuTimer1Regs.TIM.all;
	if (cnt==999)
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
	minv = 1000;
	maxv = -1000;
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
//	WriteDAC(0, 0, state[0]*0.25*256 - 1);
	samplea[cnt] = state[0];
	sampleb[cnt] = state[1];
	samplec[cnt] = state[2];
	cnt = cnt==999 ? 0 : cnt+1;

#elif (method == -2)	// DY, 5-Level, 800Hz, m=0.7, 316-395, 1.2136
	if (cnt == 0)
		EPwm1Regs.TBCTR = PWM_TIMER_TBPRD; // clear TB counter
	vh = sin_a[cnt]*1.5 * N_SM_2;		// 13
	vd = (sin_b[cnt] - sin_c[cnt]) * N_SM_2 * 0.5;		// 7
	if (vd >= vh)		// 12
	   minf = vh;
	else
	   minf = vd;
	if (-vd < minf)	minf = -vd;		// 11
	state[0] = (int)(vh-minf);		// 9
	state[1] = (int)(vd-minf);
	state[2] = (int)(-vd-minf);	// 10
	k = (int)(N_SM_2 - (state[0]+state[1]+state[2])*0.3333333333333333);
	state[0] += k;		// 1
	state[1] += k;
	state[2] += k;		// derive the nearest vector

	vx = vh - state[0] + (state[1] + state[2])*0.5;	// 12
	vy = vd - (state[1] - state[2])*0.5;				// derive remainder vector
	if (0 <= vy && vy < vx)	reg = 1;	// 66
	else if (vy >= vx && vy > -vx)	reg = 2;
	else if (0 < vy && vy <= -vx)	reg = 3;
	else if (vx < vy && vy <= 0)	reg = 4;
	else if (vy <= vx && vy < -vx)	reg = 5;
	else reg = 6;
	d[1] = 1.1547 * vx * sin[reg] - vy * cos[reg] * 2;			// 17
	d[2] = -1.1547 * vx * sin[reg-1] - vy * cos[reg-1] * 2;	// 21
	d[0] = 1 - d[1] - d[2];		// 6
	if (reg == 1)
	{
		D[0] = 1 - d[0]*0.5;
		D[1] = D[0] - d[1];
		D[2] = D[1] - d[2];
	}
	else if (reg == 2)
	{
		D[1] = 1 - d[0]*0.5;
		D[0] = D[1] - d[1];
		D[2] = D[0] - d[2];
	}
	else if (reg == 3)
	{
		D[1] = 1 - d[0]*0.5;
		D[2] = D[1] - d[1];
		D[0] = D[2] - d[2];
	}
	else if (reg == 4)
	{
		D[2] = 1 - d[0]*0.5;
		D[1] = D[2] - d[1];
		D[0] = D[1] - d[2];
	}
	else if (reg == 5)
	{
		D[2] = 1 - d[0]*0.5;
		D[0] = D[2] - d[1];
		D[1] = D[0] - d[2];
	}
	else
	{
		D[0] = 1 - d[0]*0.5;
		D[2] = D[0] - d[1];
		D[1] = D[2] - d[2];
	}

	cmp = EPwm1Regs.TBCTR;
	if (PWM_TIMER_TBPRD * D[0] >= cmp)
		state[0]++;
	if (PWM_TIMER_TBPRD * D[1] >= cmp)
		state[1]++;
	if (PWM_TIMER_TBPRD * D[2] >= cmp)
		state[2]++;

	samplecnt[cnt] = 5999 - CpuTimer1Regs.TIM.all;
	if (cnt==999)
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
	minv = 1000;
	maxv = -1000;
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
//	WriteDAC(0, 0, state[0]*0.25*256 - 1);
	samplea[cnt] = state[0];
	sampleb[cnt] = state[1];
	samplec[cnt] = state[2];
	cnt = cnt==999 ? 0 : cnt+1;

#elif (method == 5)	// NLC, 11-Level, m=0.8/0.866, 58
	state[0] = (int)(sin_a8[cnt] * N_SM_2 + N_SM_2 + 0.5);	// 17
	state[1] = (int)(sin_b8[cnt] * N_SM_2 + N_SM_2 + 0.5);	// 15
	state[2] = (int)(sin_c8[cnt] * N_SM_2 + N_SM_2 + 0.5);	// 15
	samplecnt[cnt] = 5999 - CpuTimer1Regs.TIM.all;

	samplea[cnt] = state[0];
	sampleb[cnt] = state[1];
	samplec[cnt] = state[2];
	cnt = cnt==1000 ? 0 : cnt+1;

#elif (method == 6)	// SVC-xy, 11-Level, 119-122
	vh = (sin_a8[cnt]*2 - sin_b8[cnt] - sin_c8[cnt]) * 5;	// 11, -20 ~ +20
	vd = (sin_b8[cnt] - sin_c8[cnt]) * 5;				// 8, -10 ~ +10
	fh = (int)(vh>=0 ? vh : vh-1);	// 15
	fd = (int)(vd>=0 ? vd : vd-1);	// 14
	if ((fh + fd) & 1)	// 9, odd
		if ((vd - fd) * 3 < vh - fh + 1.0)
			fh++;
		else
			fd++;
	else	// even
		if ((vd - fd) * 3 >= fh + 2.0 - vh)	// 17
		{
			fh++;
			fd++;
		}
	k = (int)(5 + 0.5 + fh*0.3333333333333333);	// 12
	state[0] = k;			// 1
	state[1] = k - (fh-fd)/2;		// 9
	state[2] = state[1] - fd;		// 2
	samplecnt[cnt] = 5999 - CpuTimer1Regs.TIM.all;

	minv = 1000;
	maxv = -1000;
	for (ph=0; ph<3; ph++)
	{
	   if (state[ph] > maxv)	maxv = state[ph];
	   if (state[ph] < minv)	minv = state[ph];
	}
	if (maxv > 10)
	{
	   state[0] -= (maxv - 10);
	   state[1] -= (maxv - 10);
	   state[2] -= (maxv - 10);
	}
	if (minv < 0)
	{
	   state[0] -= minv;
	   state[1] -= minv;
	   state[2] -= minv;
	}

//	WriteDAC(0, 0, state[0]*0.1*256 - 1);
	samplea[cnt] = state[0];
	sampleb[cnt] = state[1];
	samplec[cnt] = state[2];
	cnt = cnt==1000 ? 0 : cnt+1;

#elif (method == 7)	// line_NLC, 11-Level, m=0.8/0.866, 161-163
//	tempf[0] = (sin_a8[cnt] - sin_b8[cnt]) * N_SM_2;	// vab	15
//	tempf[1] = (sin_b8[cnt] - sin_c8[cnt]) * N_SM_2;	// vbc	14
//	tempf[2] = -tempf[0] - tempf[1];					// vca	6
//	temp[0] = (int)(tempf[0] >= 0 ? tempf[0]+0.5 : tempf[0]-0.5);	// 18
//	temp[1] = (int)(tempf[1] >= 0 ? tempf[1]+0.5 : tempf[1]-0.5);	// 19
//	temp[2] = (int)(tempf[2] >= 0 ? tempf[2]+0.5 : tempf[2]-0.5);	// 19
	temp[0] = (int)((sin_a8[cnt] - sin_b8[cnt] + 2) * N_SM_2 + 0.5);	// 23
	temp[1] = (int)((sin_b8[cnt] - sin_c8[cnt] + 2) * N_SM_2 + 0.5);	// 21
	temp[2] = (int)((sin_c8[cnt] - sin_a8[cnt] + 2) * N_SM_2 + 0.5);	// 21
	state[0] = (int)((temp[0]-temp[2])*0.3333333333333333 + N_SM_2+0.5);	// 21
	state[1] = (int)((temp[1]-temp[0])*0.3333333333333333 + N_SM_2+0.5);	// 21
	state[2] = (int)((temp[2]-temp[1])*0.3333333333333333 + N_SM_2+0.5);	// 21
	samplecnt[cnt] = 5999 - CpuTimer1Regs.TIM.all;

	samplea[cnt] = state[0];
	sampleb[cnt] = state[1];
	samplec[cnt] = state[2];
	cnt = cnt==1000 ? 0 : cnt+1;

#elif (method == 8)	// SVC-rotate-xy, m=0.8, 11-Level, 102-108
	vh = (sin_a8[cnt] - sin_b8[cnt]) * N_SM_2;	// 15, -20 ~ +20
	vd = - sin_c8[cnt] * N_SM_2;				// 8, -10 ~ +10
	fh = vh>=0 ? (int)vh : (int)vh-1;			// 17
	fd = vd>=0 ? (int)vd : (int)vd-1;			// 17
	if ((fh + fd) & 1)	// 9, odd
		if ((vd - fd) * 3 < vh - fh + 1.0)
			fh++;
		else
			fd++;
	else	// even
		if ((vd - fd) * 3 >= fh + 2.0 - vh)	// 17
		{
			fh++;
			fd++;
		}
	state[0] = N_SM_2 + (fh+fd)/2;			// 8
	state[1] = state[0] - fh;		// 3
	state[2] = N_SM_2 - fd;		// 3
	samplecnt[cnt] = 5999 - CpuTimer1Regs.TIM.all;

	samplea[cnt] = state[0];
	sampleb[cnt] = state[1];
	samplec[cnt] = state[2];
	cnt = cnt==1000 ? 0 : cnt+1;

#elif (method == -1)	// zeroSVM, 5-Level, 1500Hz, m=0.7, 164-173, 0.5617
	if (cnt == 0)
		EPwm1Regs.TBCTR = PWM_TIMER_TBPRD; // clear TB counter
	vh = -sin_b[cnt] * N_SM_2/7*8.66;		// 15
	vd = sin_a[cnt] * N_SM_2/7*8.66;		// 13
	fh = (int)(vh>=0 ? vh : vh-1);	// 15
	fd = (int)(vd>=0 ? vd : vd-1);	// 14
	dh = vh - fh;		// 5
	dd = vd - fd;		// 5
	if (dh >= dd)		// 9, inverted, ACB
	   D[0] = (1 + dh)*0.5;
	else					// ABC
	   D[0] = (1 + dd)*0.5;
    D[1] = D[0] - dh;
    D[2] = D[0] - dd;

	state[0] = N_SM_2 + fd;			// 1
	state[1] = N_SM_2 - fh;		// 2
	state[2] = N_SM_2 + fh - fd;		// 3
	cmp = EPwm1Regs.TBCTR;
	if (PWM_TIMER_TBPRD * D[0] >= cmp)
	{
	    state[0]++;	// 2
	    state[1]--;
	}
	if (PWM_TIMER_TBPRD * D[1] >= cmp)
	{
		state[1]++;
		state[2]--;
	}
	if (PWM_TIMER_TBPRD * D[2] >= cmp)
	{
		state[2]++;
		state[0]--;
	}

	samplecnt[cnt] = 5999 - CpuTimer1Regs.TIM.all;
	if (cnt==999)
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

	minv = 1000;
	maxv = -1000;
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
//	WriteDAC(0, 0, state[0]*0.25*256 - 1);
	samplea[cnt] = state[0];
	sampleb[cnt] = state[1];
	samplec[cnt] = state[2];
	samplecom[cnt] = (state[0] + state[1] + state[2])/3.0;
	cnt = cnt==999 ? 0 : cnt+1;

#elif (method == 9)		// PSPWM
	if (cnt == 0)
		CpuTimer2Regs.TIM.all = 0;
//	samplea[cnt] = CpuTimer2Regs.TIM.all / 300;	// ns
	tnow = 4000 - CpuTimer2Regs.TIM.all * 0.003333333333333333333333;
	compare[0] = 0;
	compare[1] = 0;
	compare[2] = 0;
	d[0] = sin_a8[cnt]/8*8.66;
	d[1] = sin_b8[cnt]/8*8.66;
	d[2] = sin_c8[cnt]/8*8.66;
	for (ph=0; ph<3; ph++)
	{
		for (i=0; i<N_SM_2*2; i++)
		{
			tt = tnow + intvals[i];
			tt = tt>=4000 ? tt-4000 : tt;
			if (tt < 2000)
			{
				if (tt*0.001-1 < d[ph])	compare[ph]++;
			}
			else
				if  (-tt*0.001+3 < d[ph])	compare[ph]++;
		}
	}
	samplecnt[cnt] = 5999 - CpuTimer1Regs.TIM.all;
	samplea[cnt] = compare[0];
	sampleb[cnt] = compare[1];
	samplec[cnt] = compare[2];
	cnt = cnt==999 ? 0 : cnt+1;

#endif

/*
	for (i=0; i<6; i++)
		for (j=0; j< N_SM_2*2; j++)
			sm_num[i][j] = j;
	// Sort, low to high
	for (ph=0; ph<6; ph++)
	{
		i = N_SM_2*2-1;
		while (i > 0)
		{
			pos = 0;
			for (j=0; j<i; j++)	// put highest one to last
				if (vc[ph][j] > vc[ph][j+1])
				{
					pos = j;
					tempf[0] = vc[ph][j];
					vc[ph][j] = vc[ph][j+1];
					vc[ph][j+1] = tempf[0];
					temp[0] = sm_num[ph][j];
					sm_num[ph][j] = sm_num[ph][j+1];
					sm_num[ph][j+1] = temp[0];
				}
			i = pos;
		}
	}

	for (ph=0; ph<3; ph++)
	{
		// 10 kHz, 2w & 4w PR control for circulating current suppression
		idif[ph] = (iabcpn[2*ph] + iabcpn[2*ph+1]) / 2;
		vdif2[ph] = 0.999342156239842e-4 * k2 * (idif[ph] - idif[ph+3]) + 1.996053456856543 * vdif2[ph+3] - vdif2[ph+6];
		vdif2[ph+3] = vdif2[ph];
		vdif2[ph+6] = vdif2[ph+3];
		vdif4[ph] = 0.997370182772503e-4 * k4 * (idif[ph] - idif[ph+3]) + 1.984229402628956 * vdif4[ph+3] - vdif4[ph+6];
		vdif4[ph+3] = vdif4[ph];
		vdif4[ph+6] = vdif4[ph+3];
		vdif[ph] = vdif2[ph] + vdif4[ph];
		idif[ph+3] = idif[ph];

		if (pre[ph] != state[ph])	// 2N+1 Level
		{
			if ((rotate[r_cnt[ph]] + state[ph]) & 1)
				r_cnt[ph] = r_cnt[ph]==3 ? 0 : r_cnt[ph]+1;
			sdc[ph] = N_SM_2 + rotate[r_cnt[ph]];
			abcpn[2*ph] = (sdc[ph] + N_SM_2 - state[ph]) / 2;
			abcpn[2*ph+1] = (sdc[ph] - N_SM_2 + state[ph]) / 2;
			pre[ph] = state[ph];
			r_cnt[ph] = r_cnt[ph]==3 ? 0 : r_cnt[ph]+1;
		}
	}
//	samplea[cnt] = abcpn[0];
//	sampleb[cnt] = abcpn[1];
//	samplec[cnt] = abcpn[0] + abcpn[1];
//	sampleac[cnt] = abcpn[1] - abcpn[0];

	for (ph=0; ph<6; ph++)
	{
#if (rsf == 0)
		if (iabcpn[ph] > 0)
		{
			for (i=0; i<abcpn[ph]; i++)
				sub[ph][sm_num[ph][i]] = 1;
			for (i=abcpn[ph]; i<N_SM_2*2; i++)
				sub[ph][sm_num[ph][i]] = 0;
		}
		else
		{
			for (i=N_SM_2*2-abcpn[ph]; i<N_SM_2*2; i++)
				sub[ph][sm_num[ph][i]] = 1;
			for (i=0; i<N_SM_2*2-abcpn[ph]; i++)
				sub[ph][sm_num[ph][i]] = 0;
		}

#elif (rsf == 1)	// conventional reduced switching frequency
		j = abcpn[ph] - abcpn_pre[ph];	// delta state
		if (j > 0)
			if (iabcpn[ph] > 0)
			{
				i = 0;
				while (j > 0)
				{
					if (sub[ph][sm_num[ph][i]] == 0)
					{
						sub[ph][sm_num[ph][i]] = 1;
						j--;
					}
					i++;
				}
			}
			else
			{
				i = N_SM_2*2-1;
				while (j > 0)
				{
					if (sub[ph][sm_num[ph][i]] == 0)
					{
						sub[ph][sm_num[ph][i]] = 1;
						j--;
					}
					i--;
				}
			}
		else if (j < 0)
			if (iabcpn[ph] > 0)
			{
				i = N_SM_2*2-1;
				while (j < 0)
				{
					if (sub[ph][sm_num[ph][i]] == 1)
					{
						sub[ph][sm_num[ph][i]] = 0;
						j++;
					}
					i--;
				}
			}
			else
			{
				i = 0;
				while (j < 0)
				{
					if (sub[ph][sm_num[ph][i]] == 1)
					{
						sub[ph][sm_num[ph][i]] = 0;
						j++;
					}
					i++;
				}
			}
		abcpn_pre[ph] = abcpn[ph];

#elif (rsf == 2)
		j = abcpn[ph] - abcpn_pre[ph];	// delta state
		if (j != 0)
			if (iabcpn[ph] > 0)
			{
				for (i=0; i<abcpn[ph]; i++)
					sub[ph][sm_num[ph][i]] = 1;
				for (i=abcpn[ph]; i<N_SM_2*2; i++)
					sub[ph][sm_num[ph][i]] = 0;
			}
			else
			{
				for (i=N_SM_2*2-abcpn[ph]; i<N_SM_2*2; i++)
					sub[ph][sm_num[ph][i]] = 1;
				for (i=0; i<N_SM_2*2-abcpn[ph]; i++)
					sub[ph][sm_num[ph][i]] = 0;
			}
		abcpn_pre[ph] = abcpn[ph];

#endif
	}
// */

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
   ConfigCpuTimer(&CpuTimer2, 300, 4000);
   InitEPwmTimer();
#endif

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

#if !tmode
   SetLOAD;	//把刷新锁存控制信号拉高
   CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0，开始定时器
   CpuTimer1Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0，开始定时器
	   CpuTimer2Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0，开始定时器
#endif

#if (method == 9)
	freq = 1500.0 / N_SM_2 / 2;
	for (i=0; i<N_SM_2*2; i++)	intvals[i] = 1000000*i/(freq*N_SM_2*2);	// ns
#endif
   for(;;)
   {
#if tmode
	#if (method == 0)	// HD, 85
	   vh = (vabc[0] - vabc[1]) * N_SM_2;		// 6
	   vd = (vabc[0] - vabc[2]) * N_SM_2;		// 6
	   fh = (int)vh;		// 6
	   fd = (int)vd;		// 6
	   dh = vh - fh;		// 5
	   dd = vd - fd;		// 5
	   ddif = dh - dd;		// 5
	   if (ddif > 0)		// 9
	   {
		   d[1] = dd;
		   d[2] = ddif;
	   }
	   else
	   {
		   d[1] = dh;		// 2
		   d[2] = -ddif;	// 2
	   }
	   d[0] = 1 - d[1] - d[2];		// 6
	   k = (int)(N_SM_2 - (fh+fd)*0.3333333333333333);	// 19
	   state[0] = k;			// 1
	   state[1] = k - fh;		// 2
	   state[2] = k - fd;		// 3

// 加法无需重新赋值，故为1；
//	   for (ph=0; ph<100; ph++)	   fh += 5;		// 1404	1
//	   for (ph=0; ph<100; ph++)	   fh -= 5;		// 1504	2
//	   for (ph=0; ph<100; ph++)	   fh *= 5;		// 1604	3
//	   for (ph=0; ph<100; ph++)	   fh /= 5;		// 6090	47
//	   for (ph=0; ph<100; ph++)	   vh /= 5.0;		// 15853	145
//	   for (ph=0; ph<100; ph++)	   fd = 9;		// 2
//	   for (ph=0; ph<100; ph++)	   fd = fh;		// 2
//	   for (ph=0; ph<100; ph++)	   fd = state[0];	// 2
//	   for (ph=0; ph<100; ph++)	   fd = state[0] - state[1];	// 3
//	   for (ph=0; ph<100; ph++)	   fd = (state[0] - state[1]) * 5;	// 5
//	   for (ph=0; ph<100; ph++)	   fh += fd;	// 1504	2
//	   for (ph=0; ph<100; ph++)	   fh -= fd;	// 1504	2
//	   for (ph=0; ph<100; ph++)	   fh *= fd;	// 1604	3
//	   for (ph=0; ph<100; ph++)	   fh /= fd;	// 6104	47
//	   for (ph=0; ph<100; ph++)	   fh = (int)vh;	// 1904	6
//	   for (ph=0; ph<100; ph++)	   d[1] = dd;		// 1504	2
//	   for (ph=0; ph<100; ph++)	   d[2] = -ddif;	// 1703	4
//	   for (ph=0; ph<100; ph++)	   d[0] = 1 - d[1] - d[2];	// 1904	6

	#elif (method == 1)	// gh, 175/164/94
	   vh = (vabc[0] - vabc[1]) * N_SM_2;
	   vd = (vabc[1] - vabc[2]) * N_SM_2;
	   fh = (int)vh + 1;
	   fd = (int)vd;
	   dh = vh - fh;
	   dd = vd - fd;
	   if (dh + dd < 0)
	   {
//		   d[1] = vh - fh + 1;
//		   d[2] = vd - fd;
//		   state[1] = 1 - fh;
//		   state[2] = state[1] - fd;
		   fh -= 1;
		   d[1] = dh + 1;
		   d[2] = dd;
	   }
	   else
	   {
//		   d[1] = fd - vd + 1;
//		   d[2] = fh - vh;
//		   state[1] = -fh;
//		   state[2] = state[1] - fd - 1;
		   fd += 1;
		   d[1] = 1 - dd;
		   d[2] = -dh;
	   }
	   d[0] = 1 - d[1] - d[2];
	   k = (int)(N_SM_2 + (-2*fh-fd)*0.3333333333333333);
	   state[0] = k;
	   state[1] = k - fh;
	   state[2] = state[1] - fd;

	#elif (method == 2)	// abbcca, 102
	   vh = (vabc[0] - vabc[1]) * N_SM_2;
	   vd = (vabc[1] - vabc[2]) * N_SM_2;
	   vt = (vabc[2] - vabc[0]) * N_SM_2;
	   fh = (int)vh;
	   fd = (int)vd;
	   ft = (int)vt;
	   ct = ft + 1;
	   dd = vd - fd;
	   if (fh + fd + ft == -1)
	   {
		   d[1] = vh - fh;
		   d[2] = dd;
	   }
	   else
	   {
		   d[1] = ft - vt;
		   d[2] = 1 - dd;
	   }
	   d[0] = 1 - d[1] - d[2];
	   k = (int)(N_SM_2 + (ct-fh)*0.3333333333333333);
	   state[0] = k;
	   state[1] = k - fh;
	   state[2] = k + ct;

	#elif (method == 3)	// H3D3, 340/168/157
	   vh = (vabc[mode] - vabc[mode+1]) * N_SM_2;	// 18
	   vd = (vabc[mode] - vabc[mode+2]) * N_SM_2;	// 18
	   while (vh < 0)		// 10
	   {
		   mode = mode==2 ? 0 : mode+1;
		   vh = (vabc[mode] - vabc[mode+1]) * N_SM_2;
		   vd = (vabc[mode] - vabc[mode+2]) * N_SM_2;
	   }
	   fh = (int)vh;
	   fd = (int)vd;
	   dh = vh - fh;
	   dd = vd - fd;
	   ddif = dh - dd;	//24
	   if (ddif > 0)
	   {
		   d[1] = dd;
		   d[2] = ddif;
	   }
	   else
	   {
		   d[1] = dh;
		   d[2] = -ddif;
	   }
	   d[0] = 1 - d[1] - d[2];
	   k = (int)(N_SM_2 - (fh+fd)*0.3333);
	   temp[0] = k;			// 1 2 3 2 2 9 9 9
	   temp[1] = k - fh;
	   temp[2] = k - fd;
	   temp[3] = temp[0];
	   temp[4] = temp[1];
	   state[0] = temp[mode];
	   state[1] = temp[mode+1];
	   state[2] = temp[mode+2];
//	   state[mode] = k;
//	   state[mode==2?0:mode+1] = k - fh;
//	   state[mode==0?2:mode-1] = k - fd;
//	   for (ph=0; ph<100; ph++)	   mode = (mode+1) % 3;	// 6204	49
//	   for (ph=0; ph<100; ph++)		vabc[mode] = 1;		// 8
	#elif (method == 4)	// YD, 330/244
	   vh = (vabc[0] - vabc[1]*0.5 - vabc[2]*0.5)  * N_SM_2;		// 13
	   vd = (vabc[1] - vabc[2]) * 2.5;		// 7

	   if (vh < vd)		// 12
		   minf = vh;
	   else
		   minf = vd;
	   if (-vd < minf)	minf = -vd;		// 11

	   state[0] = (int)(vh - minf);		// 9
	   state[1] = (int)(vd - minf);
	   state[2] = (int)(-vd - minf);	// 10
	   k = (int)(N_SM_2 - (state[0]+state[1]+state[2])*0.3333333333333333);	// 20
	   state[0] += k;		// 1
	   state[1] += k;
	   state[2] += k;

	   vx = vh - state[0] + state[1]*0.5 + state[2]*0.5;	// 12
	   vy = vd - state[1]*0.5 + state[2]*0.5;				// 10
	   vt = sqrt3 * vx;		// 6
	   if (0 <= vy && vy < vt)	reg = 0;	// 66
	   else if (vy >= vt && vy > -vt)	reg = 1;
	   else if (0 < vy && vy <= -vt)	reg = 2;
	   else if (vt < vy && vy <= 0)	reg = 3;
	   else if (vy <= vt && vy < -vt)	reg = 4;
	   else reg = 5;

	   d[1] = 1.1547 * (vx * sin[reg] - vy * cos[reg]);			// 17
	   d[2] = -1.1547 * (vx * sin[reg-1] - vy * cos[reg-1]);	// 21
	   d[0] = 1 - d[1] - d[2];		// 6

	#elif (method == 5)	// NLC, 11Levels, 61/33
		state[0] = (int)(vabc[0] * 5 + 5 + 0.5);
		state[1] = (int)(vabc[1] * 5 + 5 + 0.5);
		state[2] = (int)(vabc[2] * 5 + 5 + 0.5);

	#elif (method == 6)	// SVC, 11Levels, 119/85/84
		vx = (vabc[0]*2 - vabc[1] - vabc[2]) * 5;	// 11, -20 ~ +20
		vy = (vabc[1] - vabc[2]) * 5;				// 8, -10 ~ +10
		fh = (int)vx;			// 6
		fd = (int)vy;			// 6
		if ((fh + fd) & 1)	// 9, odd
			if ((vd - fd) * 3 < vh - fh + 1.0)
				fh++;
			else
				fd++;
		else	// even
			if ((vd - fd) * 3 >= fh + 2.0 - vh)	// 17
			{
				fh++;
				fd++;
			}
		k = (int)(5 + 0.5 + fh*0.3333333333333333);	// 12
		state[0] = k;			// 1
		state[1] = k - (fh-fd)/2;		// 9
		state[2] = state[1] - fd;		// 2
	#endif
#endif
   }
}

#if !tmode
void InitEPwmTimer()
{
#if (method < 5)
	EPwm1Regs.TBPRD = PWM_TIMER_TBPRD; // Period = 2* TBCLK counts
	EPwm1Regs.CMPA.half.CMPA = PWM_TIMER_TBPRD * 1; // Compare A = 400 TBCLK counts
	EPwm1Regs.TBPHS.all = 0; // Set Phase register to zero
	EPwm1Regs.TBCTR = 0; // clear TB counter
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetric
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Phase loading disabled
	EPwm1Regs.TBCTL.bit.PRDLD = 0;	// 0=the shadow is enabled and any write or read will automatically go to the shadow register
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 1; // TBCLK = SYSCLKOUT/2
	EPwm1Regs.TBCTL.bit.CLKDIV = 1;
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = 1;				//1=immediately mode, because it's always changing
//	EPwm1Regs.CMPCTL.bit.LOADAMODE = 2; // load on CTR = Zero & prd
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm1Regs.AQSFRC.bit.ACTSFA = 3;		//  toggle
	EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	EPwm1Regs.ETSEL.bit.INTEN = PWM1_INT_ENABLE;  // Enable INT
	EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event

	EPwm2Regs.TBPRD = PWM_TIMER_TBPRD; // Period = 2* TBCLK counts
	EPwm2Regs.CMPA.half.CMPA = PWM_TIMER_TBPRD * 1; // Compare A = 400 TBCLK counts
	EPwm2Regs.TBPHS.all = 0; // Set Phase register to zero
	EPwm2Regs.TBCTR = 0; // clear TB counter
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetric
	EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Phase loading disabled
	EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 1; // TBCLK = SYSCLKOUT/2
	EPwm2Regs.TBCTL.bit.CLKDIV = 1;
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = 1;
//	EPwm2Regs.CMPCTL.bit.LOADAMODE = 2; // load on CTR = Zero
	EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm2Regs.AQSFRC.bit.ACTSFA = 3;		//  toggle
	EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	EPwm2Regs.ETSEL.bit.INTEN = PWM1_INT_ENABLE;  // Enable INT
	EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event

	EPwm3Regs.TBPRD = PWM_TIMER_TBPRD; // Period = 2* TBCLK counts
	EPwm3Regs.CMPA.half.CMPA = PWM_TIMER_TBPRD * 1; // Compare A = 400 TBCLK counts
	EPwm3Regs.TBPHS.all = 0; // Set Phase register to zero
	EPwm3Regs.TBCTR = 0; // clear TB counter
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetric
	EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Phase loading disabled
	EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 1; // TBCLK = SYSCLKOUT/2
	EPwm3Regs.TBCTL.bit.CLKDIV = 1;
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = 1;
//	EPwm3Regs.CMPCTL.bit.LOADAMODE = 2; // load on CTR = Zero
	EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm3Regs.AQSFRC.bit.ACTSFA = 3;		//  toggle
	EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	EPwm3Regs.ETSEL.bit.INTEN = PWM1_INT_ENABLE;  // Enable INT
	EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event
#endif
}

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

	SpiaRegs.SPICTL.all =0x0006; // 使能主机模式，正常相位，使能主机发送，禁止接收
	                            //溢出中断，禁止SPI中断；

	SpiaRegs.SPIBRR =0x00C7;	//SPI波特率=37.5M/50	=0.75MHZ；150M/200=0.75M
    SpiaRegs.SPICCR.all =0x8a; //退出初始状态；
    SpiaRegs.SPIPRI.bit.FREE = 1;  // 自由运行
}
#endif

//===========================================================================
// No more.
//===========================================================================
