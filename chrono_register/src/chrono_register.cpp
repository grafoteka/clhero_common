#include <chrono_register/chrono_register.h>
#include <ctime>

#define BUFF_SIZE 40

Stopwatch::Stopwatch(){
	this->start();
	this->stop();
	this->reset();
	return;
}

void Stopwatch::start(){
	t1 = std::chrono::high_resolution_clock::now();
	return;
}

void Stopwatch::stop(){
	t2 = std::chrono::high_resolution_clock::now();
	interval = std::chrono::duration_cast<std::chrono::duration<double,std::milli>>(t2 - t1);
	return;
}

void Stopwatch::reset(){
	interval = std::chrono::high_resolution_clock::duration::zero();
	return;
}

double Stopwatch::get_interval(){
	return interval.count();
}

ChronoRegister::ChronoRegister(){
	time_t timer;
	struct tm * time_info;
	char buffer[BUFF_SIZE];

	time(&timer);
	time_info = localtime(&timer);

	strftime(buffer, BUFF_SIZE, "%Y_%m_%d_%H%M%S", time_info);

	this->filename = "Results_";
	this->filename +=  buffer;
	this->filename += ".txt";

	this->set_path();

	return;
}

void ChronoRegister::set_path(std::string s){
	this->path = s;
	this->complete_filename = this->path + "/" + this->filename;
	return;
}

void ChronoRegister::write_single(double a, unsigned int precision, unsigned int width){

	std::string format_str;

	if((f = fopen(this->complete_filename.c_str(),"a")) == NULL){
		return;
	}else{
		format_str += "%" + std::to_string(width) + "." + std::to_string(precision) + "lf ";
		fprintf(f, format_str.c_str(), a);
		fclose(f);
	}

	return;
}

void ChronoRegister::end_line(){

	if((f = fopen(this->complete_filename.c_str(),"a")) == NULL){
		return;
	}else{
		fprintf(f, "\n");
		fclose(f);
	}

	return;
}

void ChronoRegister::write(double a, unsigned int precision, unsigned int width){

	std::string format_str;

	if((f = fopen(this->complete_filename.c_str(),"a")) == NULL){
		return;
	}else{
		format_str += "%" + std::to_string(width) + "." + std::to_string(precision) + "lf \n";
		fprintf(f, format_str.c_str(), a);
		fclose(f);
	}

	return;
}

void ChronoRegister::write(std::string s){

	if((f = fopen(this->complete_filename.c_str(),"a")) == NULL){
		return;
	}else{
		s += "\n";
		fprintf(f, "%s", s.c_str());
		fclose(f);
	}

	return;
}

void ChronoRegister::write(std::vector<double> v, unsigned int precision, unsigned int width){

	std::string format_str;

	for(int i=0; i<v.size(); i++){
		this->write_single(v[i], precision, width);
	}

	this->end_line();

	return;
}

void ChronoRegister::write(std::string s, double a, unsigned int precision, unsigned int width){

	if((f = fopen(this->complete_filename.c_str(),"a")) == NULL){
		return;
	}else{
		
		s += " ";
		fprintf(f, "%s", s.c_str());
		fclose(f);

		this->write_single(a, precision, width);
		this->end_line();
	}

	return;

}