#include"TimeString.h"

string CTimeString::getTimeString() 
{
	int i_year, i_month, i_day, i_hour, i_minute, i_second, i_milliseconds;
	setTime(i_year, i_month, i_day, i_hour, i_minute, i_second, i_milliseconds);

	string s_year, s_month, s_day, s_hour, s_minute, s_second, s_milliseconds,s_time;
	s_year = to_string(i_year);
	s_month = to_string(i_month);						//size 2
	if (s_month.size() < 2) s_month = "0" + s_month;
	s_day = to_string(i_day);							//size 2
	if (s_day.size() < 2) s_day = "0" + s_day;
	s_hour = to_string(i_hour);							//size 2
	if (s_hour.size() < 2) s_hour = "0" + s_hour;
	s_minute = to_string(i_minute);						//size 2
	if (s_minute.size() < 2) s_minute = "0" + s_minute;
	s_second = to_string(i_second);						//size 2
	if (s_second.size() < 2) s_second = "0" + s_second;
	s_milliseconds = to_string(i_milliseconds);			//size 3
	if (s_milliseconds.size() < 3) s_milliseconds = "0" + s_milliseconds;
	if (s_milliseconds.size() < 3) s_milliseconds = "0" + s_milliseconds;
	//s_milliseconds.erase(s_milliseconds.begin() + 1, s_milliseconds.begin() + 3);

	s_time = s_year + s_month + s_day + "_" + s_hour + s_minute + "_" + s_second + "_" + s_milliseconds;
	return s_time;
}

string CTimeString::getTimeElapsefrom2Strings(string s_former, string s_latter) {

	//20190504_2130_33_119

	//former
	int minute_f, second_f, millisecond_f;
	getTimeValueFromString(s_former, minute_f, second_f, millisecond_f);

	//latter
	int minute_l, second_l, millisecond_l;
	getTimeValueFromString(s_latter, minute_l, second_l, millisecond_l);

	int sum_millisecond_f, sum_millisecond_l;
	sum_millisecond_f = millisecond_f + 1000 * (second_f + 60 * minute_f);
	sum_millisecond_l = millisecond_l + 1000 * (second_l + 60 * minute_l);

	int error_sum_millisecond = 0;
	int minute_e, second_e, millisecond_e;
	if (sum_millisecond_f > sum_millisecond_l) {

		error_sum_millisecond = sum_millisecond_f - sum_millisecond_l;
	}
	else if (sum_millisecond_f < sum_millisecond_l) {

		error_sum_millisecond = sum_millisecond_l - sum_millisecond_f;
	}
	else {

		return "00_00_000";
	}

	millisecond_e = error_sum_millisecond % 1000;
	second_e = (int)((error_sum_millisecond - millisecond_e) / 1000) % 60;
	minute_e = (int)((error_sum_millisecond - millisecond_e) / 1000) / 60;

	string s_minute, s_second, s_milliseconds, s_output;

	s_minute = to_string(minute_e);						//size 2
	if (s_minute.size() < 2) s_minute = "0" + s_minute;

	s_second = to_string(second_e);						//size 2
	if (s_second.size() < 2) s_second = "0" + s_second;

	s_milliseconds = to_string(millisecond_e);			//size 3
	if (s_milliseconds.size() < 3) s_milliseconds = "0" + s_milliseconds;
	if (s_milliseconds.size() < 3) s_milliseconds = "0" + s_milliseconds;

	s_output = s_minute + "_" + s_second + "_" + s_milliseconds;

	return s_output;
}

string CTimeString::getTElapsefrom2S(string s_former, string s_latter){

	//former
	int hour_f, minute_f, second_f, millisecond_f;
	getTimeValueFromString(s_former, hour_f, minute_f, second_f, millisecond_f);

	//latter
	int hour_l, minute_l, second_l, millisecond_l;
	getTimeValueFromString(s_latter, hour_l, minute_l, second_l, millisecond_l);

	int sum_millisecond_f, sum_millisecond_l;
	sum_millisecond_f = millisecond_f + 1000 * (second_f + 60 * (minute_f + 60 * hour_f));
	sum_millisecond_l = millisecond_l + 1000 * (second_l + 60 * (minute_l + 60 * hour_l));

	int error_sum_millisecond = 0;
	int hour_e, minute_e, second_e, millisecond_e;
	if (sum_millisecond_f > sum_millisecond_l) {

		error_sum_millisecond = sum_millisecond_f - sum_millisecond_l;
	}
	else if (sum_millisecond_f < sum_millisecond_l) {

		error_sum_millisecond = sum_millisecond_l - sum_millisecond_f;
	}
	else {

		return "00_00_000";
	}

	int second_remain, minute_remain, hour_remain;

	millisecond_e = error_sum_millisecond % 1000;
	second_remain = (error_sum_millisecond - millisecond_e) / 1000;

	second_e = second_remain % 60;
	minute_remain = (second_remain - second_e) / 60;

	minute_e = minute_remain % 60;
	hour_remain = (minute_remain - minute_e) / 60;

	hour_e = hour_remain;

	string s_hour, s_minute, s_second, s_milliseconds, s_output;

	s_hour = to_string(hour_e);							//size 2
	if (s_hour.size() < 2) s_hour = "0" + s_hour;

	s_minute = to_string(minute_e);						//size 2
	if (s_minute.size() < 2) s_minute = "0" + s_minute;

	s_second = to_string(second_e);						//size 2
	if (s_second.size() < 2) s_second = "0" + s_second;

	s_milliseconds = to_string(millisecond_e);			//size 3
	if (s_milliseconds.size() < 3) s_milliseconds = "0" + s_milliseconds;
	if (s_milliseconds.size() < 3) s_milliseconds = "0" + s_milliseconds;

	s_output = s_hour + "_" + s_minute + "_" + s_second + "_" + s_milliseconds;

	return s_output;
}


void CTimeString::getTimeValueFromString(string string_, int &i_hour, int &i_minute, int &i_second, int &i_millisecond) {

	vector<int> find_vec = find_all(string_, "_");

	string s_hour, s_minute, s_second, s_millisecond;
	s_hour = string_.substr(find_vec[0] + 1, 2);
	s_minute = string_.substr(find_vec[0] + 3, 2);
	s_second = string_.substr(find_vec[1] + 1, 2);
	s_millisecond = string_.substr(find_vec[2] + 1, 3);

	string s_day;
	s_day = string_.substr(find_vec[0] - 2, 2);

	i_hour = stoi(s_hour) + 24 * stoi(s_day);
	i_minute = stoi(s_minute);
	i_second = stoi(s_second);
	i_millisecond = stoi(s_millisecond);
}


void CTimeString::getTimeValueFromString(string string_,int &i_minute,int &i_second,int &i_millisecond) {

	vector<int> find_vec = find_all(string_, "_");
	string s_minute, s_second, s_millisecond;
	s_minute = string_.substr(find_vec[0] + 3, 2);
	s_second = string_.substr(find_vec[1] + 1, 2);
	s_millisecond = string_.substr(find_vec[2] + 1, 3);

	i_minute = stoi(s_minute);
	i_second = stoi(s_second);
	i_millisecond = stoi(s_millisecond);
}

void CTimeString::setTime(int& i_year, int& i_month, int& i_day, int& i_hour, int& i_minute, int& i_second, int& i_milliseconds)
{
	SYSTEMTIME st;
	GetSystemTime(&st);
	i_year = st.wYear;
	i_month = st.wMonth;
	i_day = st.wDay;
	i_hour = st.wHour+9;
	i_minute = st.wMinute;
	i_second = st.wSecond;
	i_milliseconds = st.wMilliseconds;
	if (i_hour >= 24) {
		i_hour -= 24;
		i_day++;
	}
}

//https://www.sejuku.net/blog/49318
std::vector<int> CTimeString::find_all(const std::string str, const std::string subStr) {
	std::vector<int> result;

	int subStrSize = subStr.size();
	int pos = str.find(subStr);

	while (pos != std::string::npos) {
		result.push_back(pos);
		pos = str.find(subStr, pos + subStrSize);
	}

	return result;
}

 bool CTimeString::getFileNames(std::string folderPath, std::vector<std::string> &file_names, bool b_cout, bool b_getDir, bool b_check)
{
	 //if (b_check)
		// if (!getDirectoryExistance(folderPath))
		//	 return false;
	 if (b_check)
		 if (!getDirectoryExistance_detail(folderPath,true))
			 return false;

	//only work in Multibyte Character Set (not in Unicode)
	//https://qiita.com/tes2840/items/8d295b1caaf10eaf33ad
	//HANDLE hFind;
	//WIN32_FIND_DATA win32fd;
	//std::string search_name = folderPath + "\\*";
	//hFind = FindFirstFile(search_name.c_str(), &win32fd);
	//if (hFind == INVALID_HANDLE_VALUE) {
	//	throw std::runtime_error("file not found");
	//	return false;
	//}
	//do
	//{
	//	if (win32fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
	//		cout << win32fd.cFileName << "(directory)" << endl;
	//	}
	//	else {
	//		file_names.push_back(win32fd.cFileName);
	//		//cout << file_names.back() << endl;
	//	}
	//} while (FindNextFile(hFind, &win32fd));
	//FindClose(hFind);

	//work in Multibyte Character Set and Unicode
	//https://qiita.com/takamon9/items/a9f1fccea740f2b79991
	//https://qiita.com/episteme/items/0e3c2ee8a8c03780f01e
	sys_ns::path fPath(folderPath);
	sys_ns::directory_iterator itr(fPath), end;
	std::error_code err;
	file_names.clear();
	for (itr; itr != end; itr.increment(err)) {
		if (err != std::errc::operation_not_permitted) {
			auto entry = *itr; // auto = std::experimental::filesystem::v1::path
			string path_name = entry.path().generic_string();
			//cout << "substr:dir:  = " << folderPath << endl;
			//cout << "substr:file: = " << path_name << endl;
			string path_relative = path_name.substr(folderPath.size() + 1, 
				path_name.size() - folderPath.size() - 1);//startposition,size
			//cout << "substr:path_relative = " << path_relative << endl;
			if(b_cout)	cout << "getFileNames: ";
			if (sys_ns::is_directory(entry.path()))
			{
				if (b_getDir) file_names.push_back(path_relative);
				if (b_cout)	cout << path_relative << "(directory)" << endl;
			}
			else
			{
				file_names.push_back(path_relative); // add to the vector.
				if (b_cout) cout << path_relative << endl;
			}
		}
		else break;
	}
	return true;
}

bool CTimeString::getFileNames_extension(std::string folderPath, std::vector<std::string> &file_names, string s_extension)
{
	//https://www.sejuku.net/blog/49318
	vector<string> filenames_;
	bool b_success = getFileNames(folderPath, filenames_, false, false, true);
	for (int i = 0; i < filenames_.size(); i++)
	{
		int i_find = filenames_[i].find(s_extension);
		if (i_find == std::string::npos) continue;
		file_names.push_back(filenames_[i]);
		cout << file_names.back() << endl;
	}
	return b_success;
}

int CTimeString::getTimeElapsefrom2Strings_millisec(string s_former, string s_latter)
{
	string t_diff = getTimeElapsefrom2Strings(s_former, s_latter);
	//XX_XX_XXX, min_second_millisec

	int i_value;

	vector<int> find_vec = find_all(t_diff, "_");
	string s_minute, s_second, s_millisecond;
	s_minute = t_diff.substr(0, 2);
	s_second = t_diff.substr(find_vec[0] + 1, 2);
	s_millisecond = t_diff.substr(find_vec[1] + 1, 3);

	i_value =
		stoi(s_millisecond)
		+ stoi(s_second) * 1000
		+ stoi(s_minute) * 1000 * 60;

	return i_value;
}

//if upper directory path is wrong, return false
bool CTimeString::getDirectoryExistance(string foder_Path)
{
	bool b_exist = false;
	vector<int> find_vec = find_all(foder_Path, "/");
	if (find_vec.size() == 0)
		throw std::runtime_error("ERROR(CTimeString::getDirectoryExistance): folderPath not contains / \n");

	string foder_Path_upper = foder_Path.substr(0, find_vec.back());
	//cout << "checking: " << foder_Path << endl;

	vector<string> s_vec;
	CTimeString::getFileNames(foder_Path_upper, s_vec, false, true, false);

	string foder_Path_relative = foder_Path.substr(find_vec.back()+1,
		foder_Path.size() - foder_Path_upper.size() - 1);//startposition,size
	//"xxx/yyyy" -> "8-3-1=3"
	//cout << "foder_Path_relative: " << foder_Path_relative << endl;

	//https://www.sejuku.net/blog/62561
	for (int i = 0; i < s_vec.size(); i++)
		if (s_vec[i] == foder_Path_relative) b_exist = true;

	//if (!b_exist) cout << "NOT FOUND: " << foder_Path << endl;
	return b_exist;
}

bool CTimeString::getDirectoryExistance_detail(string foder_Path,bool b_first)
{
	bool b_exist = false;

	vector<int> find_vec = find_all(foder_Path, "/");
	if (find_vec.size() == 0)
		throw std::runtime_error("ERROR(CTimeString::getDirectoryExistance): folderPath not contains / \n");

	if (b_first)
	{
		if (CTimeString::getDirectoryExistance(foder_Path)) b_exist = true;
	}
	else
	{
		if (CTimeString::getDirectoryExistance(foder_Path)) 
		{
			b_exist = true;
			cout << "FOUND:     " << foder_Path << endl;
		}
	}

	if (!b_exist)
	{
		string foder_Path_upper = foder_Path.substr(0, find_vec.back());
		if (!b_exist) cout << "NOT FOUND: " << foder_Path << endl;
		CTimeString::getDirectoryExistance_detail(foder_Path_upper, false);
	}

	return b_exist;
}
vector<vector<string>> CTimeString::getVecVecFromCSV_string(string filename_, string key_token)
{
	//double,float, int
	//https://qiita.com/hal1437/items/b6deb22a88c76eeaf90c
	//https://docs.oracle.com/cd/E19957-01/805-7887/6j7dsdhfl/index.html
	//https://pknight.hatenablog.com/entry/20090826/1251303641
	ifstream ifs_(filename_);
	string str_;
	int f_cnt = 0;
	vector<vector<string>> all_observation_vec_vec;
	if (ifs_.fail()) cout << "Error: file could not be read." << endl;
	else
	{
		while (getline(ifs_, str_)) {//readed to string from file

			vector<string> one_observation_vec;
			vector<int> find_vec = CTimeString::find_all(str_, key_token);
			if (find_vec.size() == 0)
				one_observation_vec.push_back(str_);
			else
			{
				one_observation_vec.push_back(str_.substr(0, find_vec[0]));
				int s_pos = 0;
				while (s_pos < find_vec.size() - 1)
				{
					one_observation_vec.push_back(
						str_.substr(find_vec[s_pos] + 1, find_vec[s_pos + 1] - (find_vec[s_pos] + 1)));
					s_pos++;
				}
				one_observation_vec.push_back(str_.substr(find_vec[s_pos] + 1, str_.size() - (find_vec[s_pos] + 1)));

			}
			all_observation_vec_vec.push_back(one_observation_vec);
		}
		ifs_.close();
	}
	return all_observation_vec_vec;
}

vector<vector<double>> CTimeString::getVecVecFromCSV(string filename_)
{
	vector<vector<double>> data_vec_vec;

	vector<vector<string>> data_vec_vec_string;
	data_vec_vec_string = CTimeString::getVecVecFromCSV_string(filename_);

	for (int j = 0; j < data_vec_vec_string.size(); j++)
	{
		vector<double> data_vec;
		for (int i = 0; i < data_vec_vec_string[j].size(); i++)
		{
			double value_ = stod(data_vec_vec_string[j][i]);
			data_vec.push_back(value_);
		}
		data_vec_vec.push_back(data_vec);
	}

	return data_vec_vec;
}


