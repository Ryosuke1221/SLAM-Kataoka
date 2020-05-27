#pragma once
#include<iostream>
#include<windows.h>
#include<string>
#include<sstream>
#include<vector>
#include<typeinfo>

//20200519
#include <locale.h>
#include <tchar.h>
//
#include <filesystem>
#include <fstream>

using namespace std;
//namespace sys_ns = std::tr2::sys;	//occur error in JIROS
namespace sys_ns = std::experimental::filesystem;

class CTimeString {

	static void setTime(int& i_year, int& i_month, int& i_day, int& i_hour, int& i_minute, int& i_second, int& i_milliseconds);

	static void getTimeValueFromString(string string_, int &i_minute, int &i_second, int &i_millisecond);
	void getTimeValueFromString(string string_, int &i_hour, int &i_minute, int &i_second, int &i_millisecond);

	string getTElapsefrom2S(string s_former, string s_latter);

public:

	//can use "std::to_string" for it ...but detail is unknown
	template < typename T > static std::string to_string(const T& n)
	{
		std::ostringstream stm;
		stm << n;
		return stm.str();
	}

	static string getTimeString();

	static string getTimeElapsefrom2Strings(string s_former, string s_latter);	//output  second

	static std::vector<int> find_all(const std::string str, const std::string subStr);

	template<typename T>
	static void getCSVFromVecVec(vector<vector<T>> saved_data_vec_vec, string filename_)
	{
		std::ofstream ofs_save;
		ofs_save.open(filename_, std::ios::out);
		for (int i = 0; i < saved_data_vec_vec.size(); i++)
		{
			for (int j = 0; j < saved_data_vec_vec[i].size(); j++)
			{
				ofs_save << saved_data_vec_vec[i][j];
				if (j < saved_data_vec_vec[i].size() - 1) ofs_save << ",";
			}
			ofs_save << endl;
		}
		ofs_save.close();

	}

	static bool getFileNames(std::string folderPath, std::vector<std::string> &file_names, bool b_cout = true, bool b_getDir = false, bool b_check = true);
	static bool getFileNames_extension(std::string folderPath, std::vector<std::string> &file_names,string s_extension);

	static int getTimeElapsefrom2Strings_millisec(string s_former, string s_latter);	//output  second

	static vector<vector<string>> getVecVecFromCSV_string(string filename_, string key_token = ",");
	static vector<vector<double>> getVecVecFromCSV(string filename_);
	

private:
	static bool getDirectoryExistance(string foder_Path);
	static bool getDirectoryExistance_detail(string foder_Path, bool b_first);


};
