
#include "Rinex.h"
#include "Ephemeris.h"
#include "Receiver.h"
using namespace std;
int numiterator = 0;
int debug_stop =1;

//int main(int argc, char ** argv) {
//	int n = 18;
//	int m = 2;
//	Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n,n);
//	Eigen::VectorXd a = Eigen::VectorXd::Zero(n);
//	Eigen::MatrixXd b  = Eigen::MatrixXd::Zero(n, m);
//	filetovector(a,n,"a");
//	filetomatrix(Q,n,n,"Q");
//	//matrixtofile(Q,"Qtest",'F');
//	int info = 0;
//	vector<double> s(2,0);
//	info = lambda(n, m, a, Q, b, s);
//	int aaaa = 1;
//
//}

int main(int argc,char ** argv){
	string navfile = "D:\\SHMILY\\340_RTK\\RTK_CPP\\data\\base.nav";
	string Bobsfile = "D:\\SHMILY\\340_RTK\\RTK_CPP\\data\\base.obs";
	//string Robsfile = "D:\\SHMILY\\340_RTK\\RTK_CPP\\data\\base1.obs";
	string Robsfile = "D:\\SHMILY\\340_RTK\\RTK_CPP\\data\\rover.obs";
	NAV_RINEX nav(navfile);
	nav.readfile();
	//All_Sats_EPH allsateph(nav); 
	//allsateph.satephsolve(nav);
	OBS_RINEX obs(Bobsfile, Robsfile);
	receiver rcv(nav,1,0,"1");//dynamics_on  arfilter outsolfile
	gtime_t headtime = obs.readheader();
	gtime_t LASTsoltime;
	int LS_stat = 0;
	if (headtime.time == 0)//�Ҳ���headtime�������½��С�
		return 1;
	string buffB,buffR;
	/*ifstream obsfileB(Bobsfile);
	ifstream obsfileR(Robsfile);
	obs.findfirstepoch(obsfileB, obsfileR, buffB, buffR, headtime);
	do {
		obs.readepoch(obsfileB, obsfileR, buffB, buffR, headtime);
		obs.getdoppler();
		
	} while (getline(obsfileB, buffB) && getline(obsfileR, buffR));
	obs.printfdoppler();*/
	/*ԭʼ����*/
	ifstream obsfileB(Bobsfile);
	ifstream obsfileR(Robsfile);
	//double nba;
	//cout << sizeof(&nba) << endl;
	obs.findfirstepoch(obsfileB, obsfileR,buffB,buffR,headtime);//�˴��Ѿ��ҵ��˺�rover��һ����Ԫ��Ӧ��base��ʱ��
	do {	
		/*obs.readepoch(obsfileB, obsfileR,buffB,buffR);
		rcv.updateobs(obs);*/
		if (!screent_obs(obsfileB, obsfileR, buffB, buffR, rcv, obs))
			continue;
		numiterator++;
		cout << "numiterator:"<< numiterator << endl;
		if (numiterator >= debug_stop) {
			int aaaa = 1;
		}
		rcv.updateSatinfo();
		/*if (obs.GPStime_R.time == 1614221810) {
			int a = 1;
		}*/
		LASTsoltime = rcv.getsoltime();
		LS_stat = rcv.LS_pos('N');//��С���˶�λ
		rcv.settt(LASTsoltime);
		if (LS_stat) {
			rcv.LS_vel();//��С���˶��٣�ֻ������С���˶�λ�ɹ�ʱ�Ž��С�
		}
		
		rcv.RTK_pos("PL");//������С���˶�λ�ɲ��ɹ�����Ҫ����RTK�ĸ���
		
		/*if (numiterator == debug_stop) {
			int aaaa = 1;
		}*/
		/*if (numiterator >= 147)
			break;*/
		rcv.soltofile("xyz",LS_stat);
		
			
			
			
		
		
		
	} while (getline(obsfileB, buffB) && getline(obsfileR, buffR));//readepoch�е�getepochdata�����ƽ���obsfileB
	//obsfileB.close();
	//obsfileR.close();
	rcv.closefile();
	return 0;
}