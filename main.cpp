
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
	if (headtime.time == 0)//找不到headtime，不往下进行。
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
	/*原始程序*/
	ifstream obsfileB(Bobsfile);
	ifstream obsfileR(Robsfile);
	//double nba;
	//cout << sizeof(&nba) << endl;
	obs.findfirstepoch(obsfileB, obsfileR,buffB,buffR,headtime);//此处已经找到了和rover第一个历元对应的base的时间
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
		LS_stat = rcv.LS_pos('N');//最小二乘定位
		rcv.settt(LASTsoltime);
		if (LS_stat) {
			rcv.LS_vel();//最小二乘定速，只有在最小二乘定位成功时才进行。
		}
		
		rcv.RTK_pos("PL");//不管最小二乘定位成不成功，都要进行RTK的更新
		
		/*if (numiterator == debug_stop) {
			int aaaa = 1;
		}*/
		/*if (numiterator >= 147)
			break;*/
		rcv.soltofile("xyz",LS_stat);
		
			
			
			
		
		
		
	} while (getline(obsfileB, buffB) && getline(obsfileR, buffR));//readepoch中的getepochdata向下推进了obsfileB
	//obsfileB.close();
	//obsfileR.close();
	rcv.closefile();
	return 0;
}