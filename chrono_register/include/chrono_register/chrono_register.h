#ifndef CHRONO_REGISTER_H
#define CHRONO_REGISTER_H

#include <chrono>
#include <ratio>
#include <string>
#include <cstdio>
#include <vector>

class Stopwatch{

	std::chrono::high_resolution_clock::time_point t1, t2;
	std::chrono::duration<double,std::milli> interval;

public:

	Stopwatch();

	void start();
	void stop();
	void reset();

	double get_interval();

};

class ChronoRegister{

	std::string filename;
	std::string path;
	std::string complete_filename;
	FILE *f;

	void write_single(double a, unsigned int precision = 4, unsigned int width = 10);
	void end_line();

public:

	ChronoRegister();

	void set_path(std::string path = ".");

	void write(double a, unsigned int precision = 4, unsigned int width = 10);
	void write(std::string s);
	void write(std::vector<double> v, unsigned int precision = 4, unsigned int width = 10);
	void write(std::string s, double a, unsigned int precision = 4, unsigned int width = 10);

};

#endif