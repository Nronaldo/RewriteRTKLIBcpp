#ifndef EARTH_UTILITY_H_
#define EARTH_UTILITY_H_

#include "Rinex.h"
#include "Math_utility.h"
extern double D2R;
extern double R2D;
extern double Cspeed;
extern double RE_WGS84;
extern double FE_WGS84;
extern double OMGE;
extern double elmin_mask;
extern double ERR_BRDCI;
extern double ERR_CBIAS;
extern double ERR_SAAS;
extern double REL_HUMI;
extern double Error_std[5];
extern const double lam_carr[3];
extern const double maxgdop;
extern const double chisqr[100];
extern const double Ls_res_threshold;
extern const double VAR_POS;
extern const double VAR_VEL;
extern const double VAR_ACC;
extern const double thresar[8];
extern const double prn[6];
extern const double optstd[3];
extern const double opterr[5];
extern const double opteratio[3];
extern const double optthresar[8];
extern const int NSATGPS;
extern const int NSATCMP;
extern const short maxout;
extern const short minlock;
extern const short minfix;
extern const double sclkstab;
extern const int optmindropsats;
extern const int optminfixsats;
extern const int optminholdsats;
extern const double varholdamb;
extern int numiterator;
extern int debug_stop;
extern const double optmaxino;

void ecef2pos(const std::vector<double > &xecef, std::vector<double > &xllh);
void ecef2enu(const std::vector<double > &xllh, const std::vector<double > &los, std::vector<double > &enu);
void xyz2enu(const std::vector<double > &xllh, Eigen::Matrix3d &E);
void covecef(const std::vector<double > &xllh, const Eigen::Matrix3d &Q, Eigen::Matrix3d &Qv);
void covenu(const std::vector<double > &xllh, const Eigen::Matrix3d &P,Eigen::Matrix3d &Q);
double varerr(double);
double interpc(const double coef[], double lat);
double tropmapf(gtime_t time, const std::vector<double > &posllh, vector<double > azel);
double nmf(gtime_t time, const std::vector<double > &posllh, vector<double > azel);
double mapf(double el, double a, double b, double c);

#endif


#pragma once
