#ifndef RECEIVER_H_
#define RECEIVER_H_

#include "Ephemeris.h"
#include <algorithm>
#include "OBS_SAT.h"
#include "Math_utility.h"
	



class BRslip {
public:
	BRslip() { Bslip = 0; Rslip = 0; };
	BRslip(int b, int r) { Bslip = b; Rslip = r; };
	int getBslip() { return Bslip; };
	int getRslip() { return Rslip; };
	void setBslip(int a) { Bslip = a; };
	void setRslip(int a) { Rslip = a; };
private:
	int Bslip;
	int Rslip;
};

class receiver {
public:
	receiver(NAV_RINEX &nav_rinex,int dynamics_on,int arfilter,string outsolname);
	~receiver() {delete RTK_x; delete RTK_P; delete outc_G; delete outc_B; delete slip_G;
	delete slip_B; delete half_G; delete half_B; delete lock_G; delete lock_B; out.close();
	}
	/*�����۲����͸�������״̬�йصĺ���*/
	void updateobs(const OBS_RINEX & obsrinex);
	void distributesats(const vector<int> &GNSS_ID_B, const vector<int> &GNSS_ID_R, map<int, OBS_SAT> &mp, char mode, const OBS_RINEX & obsrinex);//�ѹ۲������䵽�����ṹ��
	void choosecommonsat();//ɾ���ϸ����������� ֻ��base ����ֻ��rover��
	void updateSatinfo();
	/*����ʾ�йصĺ���*/
	void displayobs();
	void displayddres();
	void displaysatpos(int mode);
	void soltofile(string mode,int LS_stat);
	void closefile() { out.close(); };
	/*�붨λ�йصĺ���*/
	int DOPcheck();//1 normal 0 gdop too big
	void calculate_DOP(const vector<double > &azel);
	int validsatnum();
	/*����С���˵��㶨λ�йصĺ���*/
	int LS_pos(char mode);// s����� N ������ͨ��λ�����Ը���sol�����F ���� FDE����ʱ����LS_pos�и���sol����Ҫ������fde�������ж�һ���Ƿ����
	//����ֵ��1 ���� 0 ���������㵼���޷�FDE ���߶�λ��-1 �������������Ƿ�ɢ��
	int rescode(const vector<double > &GPS_ION_CORR, const vector<double > &BDS_ION_CORR,int &countG,int &countB);
	int LS_pos_body(char mode,int countG,int countB);
	int LS_raim();// 1 normal 0 residual too big	
	void LS_fde();
	/*����С���˶����ն����йصĺ���*/
	int LS_vel();
	int resdop();
	int LS_vel_body();
	/*��RTK��λ�йصĺ���*/
	int RTK_pos(string RTKmode);// mode��P L D �ֱ��Ӧ DGPS RTK ��������յ�RTK������mode��Ŀ���ǲ��ظ����㡣�ȴ�α�����λ����
	void RTK_tosol();
	void lock_solve();
	/*1���������й�*/
	int RCV_zdres(char mode, double *);
	/*2 �����˫���й�*/
	int RCV_ddres(string RTKmode, double *);
	void findrefsat();
	void calddres(string RTKmode, double *);
	/*3��״̬�����й�*/
	void initxP( double xvalue, double pvalue, int index);
	void rcv_detslp_ll();
	void reset_outageSAT();
	void initialize_bias_state();
	void udstate();
	void udpos();
	void udbias();
	/*4 ��۲�����й�*/
	void MeasureUpdate(string RTKmode);
	void FormHxP(string RTKmode,Eigen::MatrixXd &H, Eigen::VectorXd &x,Eigen::MatrixXd &P, int ddpairnum_GPSP, int ddpairnum_GPSL, int ddpairnum_BDSP, int ddpairnum_BDSL, vector<int> &ix);
	void FormvR(string RTKmode, Eigen::VectorXd &y,Eigen::MatrixXd &R, int ddpairnum_GPSP, int ddpairnum_GPSL, int ddpairnum_BDSP, int ddpairnum_BDSL);
	void filter_unzip(Eigen::MatrixXd &H, Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::VectorXd &v, Eigen::MatrixXd &R,const vector<int > &ix);
	int solve_validpos(string RTKmode,double *);
	int valpos(double t);
	/*5 ��̶�ģ�����й�*/
	int manage_amb_LAMBDA(double *rtk_fixX,vector<int > &ix);//��Ҫ�����ǽ���lambda������֮�������һЩ���ԣ�����˵һ���ǳ���һ��ʱ��֮���ٽ���ģ���ȹ̶������߽��в���ģ���ȹ̶�
	int exclude_different_sat(int &exc_satsID, int &exc_sats_lock);//ÿ����Ԫ�ų�һ����ͬ�����ǣ�ʵ���ϲ��Թ��ڼ򵥣�ֻ���������������������ܱ�֤ÿ���ų��Ķ��ǲ�ͬ�����ǣ�
	void recover_different_sat(int,int);//�ų��ͻָ��ķ������Ƕ�lock���в��������ָ�Ҳ�������¼���ģ���ȣ�
	int exclude_new_sat();//�ų���ǰ��Ԫ�²������ģ���ȵ����ǡ�
	int resamb_LAMBDA(double * a, vector<int > &ix);
	void restamb(double *RTK_fixX, const vector<int> & ix,const Eigen::MatrixXd &b);
	void holdamb(double *RTK_xfix, const vector<int > &ddres_stateindex,int nb,int a,int b);
	int ddmat_FormxP(Eigen::MatrixXd & D,Eigen::VectorXd &x,Eigen::MatrixXd & P,vector<int> &ix);
	vector<int> find_joinfixsat(int &nb1,int &nb2);
	void outddint(int info, const vector<int > & ix, const Eigen::MatrixXd & D, const Eigen::MatrixXd &b);//���˫��ģ����
	/*void rcv_set_allsateph();*/
	/*6 ����solution�йصĽӿ�*/
	gtime_t getsoltime() { return sol.gettime(); };
	void setsoltime(gtime_t t) { sol.settime(t); }
	void settt(gtime_t LASTsoltime);
	//solution get_sol() { return sol; }
	/*7 ��ͳ�Ʒ���s��Ԫ��LLI��־�й�*/
	void saveslips(const OBS_RINEX & obs, int Btintflag, int Rtintflag);
	void restoreslips();
	
private:
	/*�붨λģʽ�йصı���*/
	//int Pos_stat;//0 �޷���λ 4 ���㶨λ 1�̶�RTK 2 ����RTK  
	/*����С���˵��㶨λ�йص��м���*/
	Eigen::VectorXd Ls_x;
	Eigen::VectorXd v;
	Eigen::MatrixXd H;
	double Ls_res;
	/*��RTK��λ�йص��м���*/
	Eigen::VectorXd RTK_fixXpva;
	Eigen::MatrixXd RTK_fixPpva;
	double tt;//��ǰʱ�̺���һʱ�̵�ʱ������һʱ�̻�δ��ʼ��λ��ttֵӦΪ0,RTKLIB�ǲ����ܲ��ܶ�λ������ֵ
	// ������Ȼ������������
	//Eigen::MatrixXd RTK_Fpva;
	//Eigen::MatrixXd RTK_Q;
	int dynamics_on;
	double com_bias;
	int nx;
	double *RTK_x;
	double *RTK_P;//��һ�����ɱ�������������ϸ�������״̬��Э���//ʵ�����е�󡣡� Ϊ�˺�RTKLIBһ�º������ǰ��д�
	/*��̶�ģ�����йصı���*/
	int nfix;
	short * outc_G;//GPSĳ�Ƕ�ʧʱ��
	short * outc_B;//BDSĳ�Ƕ�ʧʱ��
	short * lock_G;//GPSģ����������Ԫ
	short * lock_B;//BDS ģ����������Ԫ
	char * slip_G;//GPS ĳ�� �������
	char * half_G;
	char * slip_B;//BDS ĳ�� �������,��Ҫԭ������RTKLIB ��LLI���������ʱ��Ҫ����һʱ�̵�������������жϣ����Ա������һ��ȫ�����鱣������һʱ�̵�
	char * half_B;
	int excsatindex;
	int holdamb_flag;
	int arfilter;
	// ˫����������RTKLIB�����������������������˫��
	int SATrefGPSID, SATrefBDSID;
	map<int, double > SatSat_PdresG;//key ֵ������һ����
	map<int, double > SatSat_LdresG;
	map<int, double > SatSat_DdresG;
	map<int, double > SatSat_PdresB;//key ֵ������һ����
	map<int, double > SatSat_LdresB;
	map<int, double > SatSat_DdresB;
	vector<double> DOP;// GDOP PDOp HDOP VDOP
	/*�붨λ�йص������۲���*/
	All_Sats_EPH allsateph;
	map<int, OBS_SAT> GPSobs;
	map<int, OBS_SAT> BDSobs;
	vector<double > basepos;//�������Ƕ�ʹ����ǰ�궨�õ�λ��
	/*�뱣��slip�йص���*/
	map<int, BRslip> GPSslip;//Ŀǰֻ�����˵�Ƶ
	map<int, BRslip> BDSslip;
	solution sol;
	ofstream out;
	
};
int screent_obs(ifstream &fileB, ifstream &fileR, string buffB, string buffR,receiver & rcv, OBS_RINEX &obs);
#endif
