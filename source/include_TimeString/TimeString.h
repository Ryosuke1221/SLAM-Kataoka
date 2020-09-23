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
	static void getTimeValueFromString(string string_, int &i_hour, int &i_minute, int &i_second, int &i_millisecond);

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
	static bool getFileNames_extension(std::string folderPath, std::vector<std::string> &file_names, string s_extension);
	static bool getFileNames_folder(std::string folderPath, std::vector<std::string> &file_names);

	static int getTimeElapsefrom2Strings_millisec(string s_former, string s_latter);	//output  second

	static vector<vector<string>> getVecVecFromCSV_string(string filename_, string key_token = ",");
	static vector<vector<double>> getVecVecFromCSV(string filename_);
	
	static void copyfile(string filename_from, string filename_to);//full path
	static void deletefile(string filename_delete);//full path
	static void makenewfolder(string dir, string newfoldername);//relative path
	static void movefile(string path_before, string path_after);

	static vector<string> inputSomeString();
	static vector<string> inputSomeString_fromCSV(string s_filename);
	static void showParameter(vector<float> parameter_vec, vector<string> name_vec,int i_frame_show = -1);
	static void changeParameter(vector<float> &parameter_vec, vector<string> name_vec);
	static void changeParameter(vector<float> &parameter_vec, vector<string> name_vec, string filename_, 
		int row_small, int row_big, int col_);
	static void changeParameter_2dimension(vector<vector<float>> &parameter_vec_vec, vector<string> name_vec, vector<float> parameter_vec_init);
	static void changeParameter_2dimension(vector<vector<float>> &parameter_vec_vec, vector<string> name_vec, vector<float> parameter_vec_init,
		string filename_, int row_small, int col_small, int row_big, int col_big);
	static vector<vector<float>> inputParameters_2dimension(string filename_, int row_small, int col_small);

	template<typename T>
	static vector<vector<T>> calcVectorPairPattern(vector<vector<T>> vectorPair_vecvec, bool b_cout = false)
	{
		vector<vector<T>> pattern_vecvec;

		//make pattern_vec_vec
		int num_pattern;
		num_pattern = 1;
		for (int j = 0; j < vectorPair_vecvec.size(); j++)
			num_pattern *= vectorPair_vecvec[j].size();
		pattern_vecvec.resize(num_pattern);
		for (int j = 0; j < num_pattern; j++)
			pattern_vecvec[j].resize(vectorPair_vecvec.size());

		//insert to pattern_vec_vec
		for (int j = 0; j < num_pattern; j++)
		{
			int i_devide = num_pattern;
			int i_residual = j;
			for (int i = 0; i < vectorPair_vecvec.size(); i++)
			{
				int idx;
				i_devide /= vectorPair_vecvec[i].size();
				idx = i_residual / i_devide;
				i_residual %= i_devide;
				pattern_vecvec[j][i] = vectorPair_vecvec[i][idx];
			}
		}

		//removeSameParameter(vectorPair_vecvec);
		removeSameValue_vecvec_VectorPairPattern(vectorPair_vecvec);

		if (b_cout)
		{
			cout << "show pattern" << endl;
			for (int j = 0; j < pattern_vecvec.size(); j++)
			{
				string s_j = to_string(j);
				if (s_j.size() < 4) s_j = " " + s_j;
				if (s_j.size() < 4) s_j = " " + s_j;
				if (s_j.size() < 4) s_j = " " + s_j;
				cout << s_j << ":";
				for (int i = 0; i < pattern_vecvec[j].size(); i++)
				{
					string s_value;
					s_value = to_string(pattern_vecvec[j][i]);
					if (s_value.size() < 4) s_value = " " + s_value;
					if (s_value.size() < 4) s_value = " " + s_value;
					if (s_value.size() < 4) s_value = " " + s_value;
					cout << "  " << s_value;

				}
				cout << endl;
			}
			cout << endl;
		}

		return pattern_vecvec;
	}

	template<typename T>
	static void removeSameValue_vecvec_VectorPairPattern(vector<vector<T>> &vectorPair_vecvec)
	{
		for (int j = 0; j < vectorPair_vecvec.size(); j++)
			removeSameValue_vec_VectorPairPattern(vectorPair_vecvec[j]);
	}

	template<typename T>
	static void removeSameValue_vec_VectorPairPattern(vector<T> &vectorPair_vec)
	{
		vector<T> vectorPair_vec_new;
		//sort
		sort(vectorPair_vec.begin(), vectorPair_vec.end());
		//remove same value
		T value_last;
		for (int i = 0; i < vectorPair_vec.size(); i++)
		{
			if (i == 0)
				vectorPair_vec_new.push_back(vectorPair_vec[i]);
			else
				if (value_last != vectorPair_vec[i])
					vectorPair_vec_new.push_back(vectorPair_vec[i]);
			value_last = vectorPair_vec[i];
		}
		vectorPair_vec = vectorPair_vec_new;
	}

	template<typename T>
	static void sortVector(vector<T> &value_vec, bool b_ascending = true)
	{
		for (int i = 0; i < value_vec.size(); i++)
		{
			for (int j = value_vec.size() - 1; j > i; j--)
			{
				bool b_swap = false;
				if (b_ascending && value_vec[j] < value_vec[j - 1])
					b_swap = true;
				else if (!b_ascending && value_vec[j] > value_vec[j - 1])
					b_swap = true;
				if (b_swap) swap(value_vec[j], value_vec[j - 1]);
			}
		}
	}

	template<typename T>
	static void sortVector2d(vector<vector<T>> &value_vecvec, int index_arg, bool b_ascending = true)
	{
		vector<pair<T, int>> frame_pair_vec;
		for (int j = 0; j < value_vecvec.size(); j++)
			frame_pair_vec.push_back(make_pair(value_vecvec[j][index_arg], j));
		for (int i = 0; i < frame_pair_vec.size(); i++)
		{
			for (int j = frame_pair_vec.size() - 1; j > i; j--)
			{
				bool b_swap = false;
				if (b_ascending && frame_pair_vec[j].first < frame_pair_vec[j - 1].first)
					b_swap = true;
				else if(!b_ascending && frame_pair_vec[j].first > frame_pair_vec[j - 1].first)
					b_swap = true;
				if(b_swap) swap(frame_pair_vec[j], frame_pair_vec[j - 1]);
			}
		}
		vector<vector<T>> value_input_vecvec_new;
		for (int j = 0; j < frame_pair_vec.size(); j++)
			value_input_vecvec_new.push_back(value_vecvec[frame_pair_vec[j].second]);
		value_vecvec.clear();
		value_vecvec = value_input_vecvec_new;
	}

	template<typename T>
	static void sortVector2d_2dimension(vector<vector<T>> &value_vecvec, int index_first, int index_second, bool b_ascending = true)
	{
		vector<vector<T>> value_vecvec_new;
		sortVector2d(value_vecvec, index_first, b_ascending);
		vector<vector<int>> index_sameFirstValue_vecvec;
		{
			vector<int> index_sameFirstValue_vec;
			T value_pickup;
			for (int j = 0; j < value_vecvec.size(); j++)
			{
				if (j == 0) value_pickup = value_vecvec[j][index_first];
				else if (value_pickup != value_vecvec[j][index_first])
				{
					index_sameFirstValue_vecvec.push_back(index_sameFirstValue_vec);
					index_sameFirstValue_vec.clear();
					value_pickup = value_vecvec[j][index_first];
				}
				index_sameFirstValue_vec.push_back(j);
				if (j == value_vecvec.size() - 1)
					index_sameFirstValue_vecvec.push_back(index_sameFirstValue_vec);
			}
		}
		for (int j = 0; j < index_sameFirstValue_vecvec.size(); j++)
		{
			vector<vector<T>> value_vecvec_1firstValue;
			for (int i = 0; i < index_sameFirstValue_vecvec[j].size(); i++)
				value_vecvec_1firstValue.push_back(value_vecvec[index_sameFirstValue_vecvec[j][i]]);
			sortVector2d(value_vecvec_1firstValue, index_second, b_ascending);
			value_vecvec_new.insert(value_vecvec_new.end(), value_vecvec_1firstValue.begin(), value_vecvec_1firstValue.end());
		}
		value_vecvec.clear();
		value_vecvec = value_vecvec_new;
	}

	template<typename T>
	static T getMedian(vector<T> value_vec)
	{
		sortVector(value_vec);

		int size = value_vec.size();

		if (size % 2 == 1)
			return value_vec[(size - 1) / 2];
		else
			return (value_vec[(size / 2) - 1] + value_vec[size / 2]) / 2.;
	}

	template<typename T>
	static vector<T> getMedian_Quartile(vector<T> value_vec)
	{
		//https://atarimae.biz/archives/19162
		sortVector(value_vec);
		int size = value_vec.size();
		T median_ = getMedian(value_vec);
		T first_quartile;
		T third_quartile;
		if (size % 2 == 1)
		{
			vector<T> temp_vec_first;
			temp_vec_first.insert(temp_vec_first.begin(), value_vec.begin(), value_vec.begin() + (size - 1) / 2);
			first_quartile = getMedian(temp_vec_first);
			vector<T> temp_vec_third;
			temp_vec_third.insert(temp_vec_third.begin(), value_vec.begin() + (size - 1) / 2 + 1, value_vec.end());
			third_quartile = getMedian(temp_vec_third);
		}
		else
		{
			vector<T> temp_vec_first;
			temp_vec_first.insert(temp_vec_first.begin(), value_vec.begin(), value_vec.begin() + (size / 2));
			first_quartile = getMedian(temp_vec_first);
			vector<T> temp_vec_third;
			temp_vec_third.insert(temp_vec_third.begin(), value_vec.begin() + size / 2, value_vec.end());
			third_quartile = getMedian(temp_vec_third);
		}
		vector<T> output_vec;
		output_vec.push_back(first_quartile);
		output_vec.push_back(median_);
		output_vec.push_back(third_quartile);
		return output_vec;
	}
	
	static vector<vector<int>> getIntCluster_SomeToSome(vector<vector<int>> value_vecvec, bool b_recursive = false);

	template<typename T>
	static vector<vector<T>> getMatrixCSVFromVecVec(vector<vector<T>> saved_data_vec_vec)
	{
		vector<vector<string>> save_vec_vec;

		if (saved_data_vec_vec.size() != saved_data_vec_vec[0].size())
		{
			cout << "ERROR: rows and cols is different" << endl;
			return save_vec_vec;
		}

		for (int j = 0; j < saved_data_vec_vec.size() + 1; j++)
		{
			vector<string> save_vec;
			save_vec.resize(saved_data_vec_vec.size() + 1);
			save_vec_vec.push_back(save_vec);
		}

		//fill except value cell
		for (int i = 0; i < saved_data_vec_vec.size(); i++)
			save_vec_vec[0][i + 1] = to_string(i);
		for (int j = 0; j < saved_data_vec_vec.size(); j++)
			save_vec_vec[j + 1][0] = to_string(j);
		save_vec_vec[0][0] = "-";
		//fill value cell
		for (int j = 0; j < saved_data_vec_vec.size(); j++)
		{
			for (int i = 0; i < saved_data_vec_vec.size(); i++)
			{
				save_vec_vec[j + 1][i + 1] = to_string(saved_data_vec_vec[j][i]);
			}
		}
		return save_vec_vec;
	}

	template<typename T>
	static vector<vector<T>> getTranspositionOfVecVec(vector<vector<T>> T_vecvec)
	{
		vector<vector<T>> T_vecvec_output;
		int rows = 0;
		int cols = 0;
		rows = T_vecvec.size();

		for (int j = 0; j < T_vecvec.size(); j++)
			if (cols < T_vecvec[j].size()) cols = T_vecvec[j].size();

		//fill blank ingredient
		for (int j = 0; j < T_vecvec.size(); j++)
		{
			int error_cols = cols - T_vecvec[j].size();
			for (int i = 0; i < error_cols; i++)
			{
				T temp;
				T_vecvec[j].push_back(temp);
			}
		}

		swap(rows, cols);

		for (int j = 0; j < rows; j++)
		{
			vector<T> T_vec_output;
			for (int i = 0; i < cols; i++)
				T_vec_output.push_back(T_vecvec[i][j]);
			T_vecvec_output.push_back(T_vec_output);
		}
		return T_vecvec_output;
	}

	static vector<vector<string>> getMatrixData_fromFormatOfFPFH(vector<vector<string>> s_input_vecvec,
		string s_start, int i_pos_start_fromS, string s_finish, int i_pos_finish_fromS);

	template<typename T>
	static vector<int> getHistogram(const vector<T> &value_vec, int num_bin, bool b_cout = false)
	{
		T value_max = -std::numeric_limits<T>::max();
		T value_min = std::numeric_limits<T>::max();

		for (int j = 0; j < value_vec.size(); j++)
		{
			if (value_max < value_vec[j]) value_max = value_vec[j];
			if (value_min > value_vec[j]) value_min = value_vec[j];
		}

		cout << "value_max:" << value_max << endl;
		cout << "value_min:" << value_min << endl;
		vector<int> hist_vec = getHistogram(value_vec, value_max, value_min, num_bin, b_cout);
		return hist_vec;
	}

	template<typename T>
	static vector<int> getHistogram(const vector<T> &value_vec_arg, T value_max, T value_min,
		int num_bin, bool b_cout = false)
	{
		vector<int> hist_vec;
		hist_vec.resize(num_bin);
		fill(hist_vec.begin(), hist_vec.end(), 0);

		vector<T> value_vec;
		value_vec.insert(value_vec.end(), value_vec_arg.begin(), value_vec_arg.end());

		//value_min <= value <= value_max
		for (int j = 0; j < value_vec.size(); j++)
		{
			if (value_vec[j] < value_min) value_vec[j] = value_min;
			else if (value_max < value_vec[j]) value_vec[j] = value_max;
		}

		vector<float> mean_hist_vec;
		mean_hist_vec.resize(num_bin);
		fill(mean_hist_vec.begin(), mean_hist_vec.end(), 0.);
		T range_ = (value_max - value_min) / (T)num_bin;
		for (int j = 0; j < value_vec.size(); j++)
		{
			int i_bin = (int)((value_vec[j] - value_min) / range_);
			if (value_vec[j] == value_max) i_bin--;
			if (i_bin < 0 || num_bin - 1 < i_bin)
			{
				cout << "invalid: i_bin = " << i_bin << endl;
				cout << "j:" << j << " value_vec[j]:" << value_vec[j] << endl;
			}
			hist_vec[i_bin]++;
			mean_hist_vec[i_bin] += value_vec[j];
		}
		for (int j = 0; j < mean_hist_vec.size(); j++)
			if (hist_vec[j] != 0) mean_hist_vec[j] / (float)hist_vec[j];

		//calc sigma_deviation
		vector<float> sigma_deviation_vec;
		sigma_deviation_vec.resize(num_bin);
		fill(sigma_deviation_vec.begin(), sigma_deviation_vec.end(), 0.);
		for (int j = 0; j < sigma_deviation_vec.size(); j++)
			if (hist_vec[j] == 0) sigma_deviation_vec[j] = -1.;
		for (int j = 0; j < value_vec.size(); j++)
		{
			int i_bin = (int)((value_vec[j] - value_min) / range_);
			if (value_vec[j] == value_max) i_bin--;
			sigma_deviation_vec[i_bin] += pow(value_vec[j] - mean_hist_vec[i_bin], 2.) / (float)hist_vec[i_bin];
		}
		for (int j = 0; j < sigma_deviation_vec.size(); j++)
			if (sigma_deviation_vec[j] != -1) sigma_deviation_vec[j] = sqrt(sigma_deviation_vec[j]);

		if (b_cout)
		{
			int max_num = 0;
			for (int j = 0; j < hist_vec.size(); j++)
				if (max_num < hist_vec[j]) max_num = hist_vec[j];
			int length_show_max = 110;

			for (int j = 0; j < hist_vec.size(); j++)
			{
				string s_index = to_string(j);
				if (s_index.size() < 2) s_index = " " + s_index;

				T value_ = value_min + range_ * j;
				string s_value = to_string(value_);
				for (int i = 0; i < 9; i++)
					if (s_value.size() < 9) s_value = " " + s_value;

				string s_num = to_string(hist_vec[j]);
				for (int i = 0; i < 6; i++)
					if (s_num.size() < 6) s_num = " " + s_num;

				string s_sigma = to_string(sigma_deviation_vec[j]);
				for (int i = 0; i < 9; i++)
					if (s_sigma.size() < 9) s_sigma = " " + s_sigma;

				int length_show = 0;
				if (max_num != 0) length_show = (int)((float)length_show_max * (float)hist_vec[j] / (float)max_num);

				string s_output = "[" + s_index + "] value(min):" + s_value;
				s_output += " sigma:" + s_sigma;
				s_output += " num:" + s_num + "  ";
				for (int i = 0; i < length_show; i++)
					if (s_output.size() < length_show) s_output += "-";

				//cout << "[" << s_index << "] value:" << s_value << " num:" << s_num << endl;
				cout << s_output << endl;

			}
		}

		return hist_vec;
	}

	template<typename T>
	static vector<vector<int>> getHistogram_IndexOfBin(const vector<T> &value_vec, int num_bin, vector<int> index_bin_vec, bool b_cout = false)
	{
		T value_max = -std::numeric_limits<T>::max();
		T value_min = std::numeric_limits<T>::max();
		for (int j = 0; j < value_vec.size(); j++)
		{
			if (value_max < value_vec[j]) value_max = value_vec[j];
			if (value_min > value_vec[j]) value_min = value_vec[j];
		}
		vector<vector<int>> index_vecvec;
		index_vecvec = getHistogram_IndexOfBin(value_vec, value_max, value_min, num_bin, index_bin_vec, b_cout);
		return index_vecvec;
	}

	template<typename T>
	static vector<vector<int>> getHistogram_IndexOfBin(const vector<T> &value_vec, T value_max, T value_min, int num_bin, vector<int> index_bin_vec, bool b_cout = false)
	{
		for (int j = index_bin_vec.size() - 1; j >= 0; j--)
			if (num_bin <= index_bin_vec[j]) index_bin_vec.erase(index_bin_vec.begin() + j);
		vector<vector<int>> index_vecvec;
		if (index_bin_vec.size() == 0) return index_vecvec;
		T range_ = (value_max - value_min) / (T)num_bin;
		vector<pair<T, T>> boundary_vec;
		for (int j = 0; j < index_bin_vec.size(); j++)
		{
			T value_bin_min = (T)index_bin_vec[j] * range_ + value_min;
			T value_bin_max = value_bin_min + range_;
			boundary_vec.push_back(make_pair(value_bin_min, value_bin_max));
		}
		for (int j = 0; j < boundary_vec.size(); j++)
		{
			vector<int> index_vec;
			for (int i = 0; i < value_vec.size(); i++)
			{
				if ((boundary_vec[j].first <= value_vec[i] && value_vec[i] < boundary_vec[j].second)
					|| (boundary_vec[j].second == value_max && value_max == value_vec[i]))
					index_vec.push_back(i);
			}
			index_vecvec.push_back(index_vec);
		}
		return index_vecvec;
	}

	template<typename T>
	static vector<int> getOuolierRemovedIndex(const vector<T> &value_vec_arg, float th_rate_BigAndSmall, 
		float &output_edge_low, float &output_edge_high)
	{
		vector<vector<float>> value_vecvec;
		for (int j = 0; j < value_vec_arg.size(); j++)
		{
			vector<float> value_vec;
			value_vec.push_back((float)value_vec_arg[j]);
			value_vec.push_back((float)j);
			value_vecvec.push_back(value_vec);
		}

		sortVector2d(value_vecvec, 0);

		vector<int> output_index_vec;

		for (int j = 0; j < value_vecvec.size(); j++)
		{
			if ((float)j / (float)value_vecvec.size() <= th_rate_BigAndSmall) continue;
			if ((float)j / (float)value_vecvec.size() >= 1. - th_rate_BigAndSmall) continue;
			output_index_vec.push_back((int)(value_vecvec[j][1]));
		}

		output_edge_low = value_vec_arg[output_index_vec[0]];
		output_edge_high = value_vec_arg[output_index_vec.back()];

		sortVector(output_index_vec);

		return output_index_vec;
	}

	template<typename T>
	static string to_string_remove0(T value_)
	{
		string s_value = to_string(value_);

		vector<int> find_vec_dot = find_all(s_value, ".");
		// 10 -> 10
		if (find_vec_dot.size() == 0) return s_value;
		//10.0 -> 10
		if (value_ - (int)value_ == 0.)
		{
			for (int j = s_value.size() - 1; j >= find_vec_dot[0]; j--)
				s_value.erase(s_value.begin() + j);
			return s_value;
		}
		//10.0100 -> 10.01
		for (int j = s_value.size() - 1; j > find_vec_dot[0]; j--)
		{
			string s_char = s_value.substr(j, 1);
			if (s_char == "0") s_value.erase(s_value.begin() + j);
			else return s_value;
		}
		return s_value;
	}

	static string getFilename_onlyExtension(string s_filename);
	static string getFilename_removingFolder(string s_input);

private:
	static bool getDirectoryExistance(string foder_Path);
	static bool getDirectoryExistance_detail(string foder_Path, bool b_first);


};
