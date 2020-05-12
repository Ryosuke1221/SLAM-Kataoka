#pragma once

#include <sstream>
#include <iostream>
#include <fstream>

#include <pcl/correspondence.h>
#include <pcl/registration/default_convergence_criteria.h>
#include <Eigen/Core>

#include"KataokaCorrespondence.h"

using namespace std;

class __declspec(dllexport) CKataokaConvergence_ {

	enum ConvergenceState
	{
		CONVERGENCE_CRITERIA_NOT_CONVERGED,
		CONVERGENCE_CRITERIA_ITERATIONS,
		CONVERGENCE_CRITERIA_TRANSFORM,
		CONVERGENCE_CRITERIA_ABS_MSE,
		CONVERGENCE_CRITERIA_REL_MSE,
		CONVERGENCE_CRITERIA_NO_CORRESPONDENCES
	};

	const int &iterations_;
	const Eigen::Matrix4f &transformation_;
	//const pcl::Correspondences &correspondences_;
	const Correspondences_Kataoka &correspondences_original;
	const Correspondences_Spring1 &correspondences_spring1;
	const Correspondences_Spring2 &correspondences_spring2;

	double correspondences_prev_mse_;
	double correspondences_cur_mse_;
	int max_iterations_;
	bool failure_after_max_iter_;
	double rotation_threshold_;
	double translation_threshold_;
	double mse_threshold_relative_;
	double mse_threshold_absolute_;
	int iterations_similar_transforms_;
	int max_iterations_similar_transforms_;
	ConvergenceState convergence_state_;
	int M_i_method;

public:
	//CKataokaConvergence_(const int &iterations, const Eigen::Matrix4f &transform, const pcl::Correspondences &correspondences)
	//	: iterations_(iterations)
	//	, transformation_(transform)
	//	, correspondences_(correspondences)
	//	, correspondences_prev_mse_(std::numeric_limits<double>::max())
	//	, correspondences_cur_mse_(std::numeric_limits<double>::max())
	//	, max_iterations_(100)                 // 100 iterations
	//	, failure_after_max_iter_(false)
	//	, rotation_threshold_(0.99999)         // 0.256 degrees
	//	, translation_threshold_(3e-4 * 3e-4)  // 0.0003 meters
	//	, mse_threshold_relative_(0.00001)     // 0.001% of the previous MSE (relative error)
	//	, mse_threshold_absolute_(1e-12)       // MSE (absolute error)
	//	, iterations_similar_transforms_(0)
	//	, max_iterations_similar_transforms_(0)
	//	, convergence_state_(CONVERGENCE_CRITERIA_NOT_CONVERGED)
	//{
	//}

	CKataokaConvergence_(const int &iterations, const Eigen::Matrix4f &transform, const Correspondences_Kataoka &correspondences_arg)
		: iterations_(iterations)
		, transformation_(transform)
		//, correspondences_(pcl::Correspondences())
		, correspondences_original(correspondences_arg)
		, correspondences_spring1(Correspondences_Spring1())
		, correspondences_spring2(Correspondences_Spring2())
		, correspondences_prev_mse_(std::numeric_limits<double>::max())
		, correspondences_cur_mse_(std::numeric_limits<double>::max())
		, max_iterations_(100)                 // 100 iterations
		, failure_after_max_iter_(false)
		, rotation_threshold_(0.99999)         // 0.256 degrees
		, translation_threshold_(3e-4 * 3e-4)  // 0.0003 meters
		, mse_threshold_relative_(0.00001)     // 0.001% of the previous MSE (relative error)
		, mse_threshold_absolute_(1e-12)       // MSE (absolute error)
		, iterations_similar_transforms_(0)
		, max_iterations_similar_transforms_(0)
		, convergence_state_(CONVERGENCE_CRITERIA_NOT_CONVERGED)
	{
	}

	CKataokaConvergence_(const int &iterations, const Eigen::Matrix4f &transform, const Correspondences_Spring1 &correspondences_arg)
		: iterations_(iterations)
		, transformation_(transform)
		//, correspondences_(pcl::Correspondences())
		//, correspondences_original(correspondences_arg)
		//, correspondences_spring1(Correspondences_Spring_1())
		, correspondences_original(Correspondences_Kataoka())
		, correspondences_spring1(correspondences_arg)
		, correspondences_spring2(Correspondences_Spring2())
		, correspondences_prev_mse_(std::numeric_limits<double>::max())
		, correspondences_cur_mse_(std::numeric_limits<double>::max())
		, max_iterations_(100)                 // 100 iterations
		, failure_after_max_iter_(false)
		, rotation_threshold_(0.99999)         // 0.256 degrees
		, translation_threshold_(3e-4 * 3e-4)  // 0.0003 meters
		, mse_threshold_relative_(0.00001)     // 0.001% of the previous MSE (relative error)
		, mse_threshold_absolute_(1e-12)       // MSE (absolute error)
		, iterations_similar_transforms_(0)
		, max_iterations_similar_transforms_(0)
		, convergence_state_(CONVERGENCE_CRITERIA_NOT_CONVERGED)
	{
	}

	CKataokaConvergence_(const int &iterations, const Eigen::Matrix4f &transform, const Correspondences_Spring2 &correspondences_arg)
		: iterations_(iterations)
		, transformation_(transform)
		//, correspondences_(pcl::Correspondences())
		//, correspondences_original(correspondences_arg)
		//, correspondences_spring1(Correspondences_Spring_1())
		, correspondences_original(Correspondences_Kataoka())
		, correspondences_spring1(Correspondences_Spring1())
		, correspondences_spring2(correspondences_arg)
		, correspondences_prev_mse_(std::numeric_limits<double>::max())
		, correspondences_cur_mse_(std::numeric_limits<double>::max())
		, max_iterations_(100)                 // 100 iterations
		, failure_after_max_iter_(false)
		, rotation_threshold_(0.99999)         // 0.256 degrees
		, translation_threshold_(3e-4 * 3e-4)  // 0.0003 meters
		, mse_threshold_relative_(0.00001)     // 0.001% of the previous MSE (relative error)
		, mse_threshold_absolute_(1e-12)       // MSE (absolute error)
		, iterations_similar_transforms_(0)
		, max_iterations_similar_transforms_(0)
		, convergence_state_(CONVERGENCE_CRITERIA_NOT_CONVERGED)
	{
	}

	~CKataokaConvergence_() {

	}

	bool hasConverged();

	inline void
		setMaximumIterationsSimilarTransforms(const int nr_iterations) { max_iterations_similar_transforms_ = nr_iterations; }
	inline int
		getMaximumIterationsSimilarTransforms() const { return (max_iterations_similar_transforms_); }
	inline void
		setMaximumIterations(const int nr_iterations) { max_iterations_ = nr_iterations; }
	inline int
		getMaximumIterations() const { return (max_iterations_); }
	inline void
		setFailureAfterMaximumIterations(const bool failure_after_max_iter) { failure_after_max_iter_ = failure_after_max_iter; }
	inline bool
		getFailureAfterMaximumIterations() const { return (failure_after_max_iter_); }
	inline void
		setRotationThreshold(const double threshold) { rotation_threshold_ = threshold; }
	inline double
		getRotationThreshold() const { return (rotation_threshold_); }
	inline void
		setTranslationThreshold(const double threshold) { translation_threshold_ = threshold; }
	inline double
		getTranslationThreshold() const { return (translation_threshold_); }
	inline void
		setRelativeMSE(const double mse_relative) { mse_threshold_relative_ = mse_relative; }
	inline double
		getRelativeMSE() const { return (mse_threshold_relative_); }
	inline void
		setAbsoluteMSE(const double mse_absolute) { mse_threshold_absolute_ = mse_absolute; }
	inline double
		getAbsoluteMSE() const { return (mse_threshold_absolute_); }
	inline double
		getFittnessScore()const { return (correspondences_cur_mse_); }	//kataoka
	inline void
		setMethodInt(const int i_method_arg) { M_i_method = i_method_arg; }


	ConvergenceState
		getConvergenceState()
	{
		return (convergence_state_);
	}

	inline void
		setConvergenceState(ConvergenceState c)
	{
		convergence_state_ = c;
	}

	inline double
		calculateMSE(const pcl::Correspondences &correspondences_arg) const
	{
		double mse = 0;
		for (size_t i = 0; i < correspondences_arg.size(); ++i)
			mse += correspondences_arg[i].distance;
		mse /= double(correspondences_arg.size());
		return (mse);
	}

	inline double
		calculateMSE(const Correspondences_Kataoka &correspondences_arg) const
	{
		double mse = 0;
		for (size_t i = 0; i < correspondences_arg.size(); ++i)
			mse += correspondences_arg[i].distance;
		mse /= double(correspondences_arg.size());
		return (mse);
	}

	inline double
		calculateMSE(const Correspondences_Spring1 &correspondences_arg) const
	{
		double mse = 0;
		for (size_t i = 0; i < correspondences_arg.size(); ++i)
			mse += correspondences_arg[i].distance * correspondences_arg[i].weight_spring;
		mse /= double(correspondences_arg.size());
		return (mse);
	}

	inline double
		calculateMSE(const Correspondences_Spring2 &correspondences_arg) const
	{
		double mse = 0;
		for (size_t index_p = 0; index_p < correspondences_arg.size(); ++index_p)
		{
			double mse_index_p = 0.;
			for (size_t index_near = 0; index_near < correspondences_arg[index_p].index_match_vec.size(); ++index_near)
			{
				//mse += correspondences_arg[i].distance * correspondences_arg[i].weight_spring;
				mse_index_p += correspondences_arg[index_p].distance_vec[index_near]
					* correspondences_arg[index_p].weight_spring_vec[index_near];

			}
			mse_index_p /= double(correspondences_arg[index_p].index_match_vec.size());

			mse += mse_index_p;
		}
		mse /= double(correspondences_arg.size());
		return (mse);
	}


};
