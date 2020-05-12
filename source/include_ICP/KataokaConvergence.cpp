#include"KataokaConvergence.h"

bool CKataokaConvergence_::hasConverged() {

	convergence_state_ = CONVERGENCE_CRITERIA_NOT_CONVERGED;

	PCL_DEBUG("[pcl::DefaultConvergenceCriteria::hasConverged] Iteration %d out of %d.\n", iterations_, max_iterations_);
	// 1. Number of iterations has reached the maximum user imposed number of iterations

	if (iterations_ >= max_iterations_)
	{
		if (failure_after_max_iter_)
			return (false);
		else
		{
			convergence_state_ = CONVERGENCE_CRITERIA_ITERATIONS;
			return (true);
		}
		return (failure_after_max_iter_ ? false : true);
	}

	// 2. The epsilon (difference) between the previous transformation and the current estimated transformation
	double cos_angle = 0.5 * (transformation_.coeff(0, 0) + transformation_.coeff(1, 1) + transformation_.coeff(2, 2) - 1);
	double translation_sqr = transformation_.coeff(0, 3) * transformation_.coeff(0, 3) +
		transformation_.coeff(1, 3) * transformation_.coeff(1, 3) +
		transformation_.coeff(2, 3) * transformation_.coeff(2, 3);
	PCL_DEBUG("[pcl::DefaultConvergenceCriteria::hasConverged] Current transformation gave %f rotation (cosine) and %f translation.\n", cos_angle, translation_sqr);

	if (cos_angle >= rotation_threshold_ && translation_sqr <= translation_threshold_)
	{
		if (iterations_similar_transforms_ < max_iterations_similar_transforms_)
		{
			// Increment the number of transforms that the thresholds are allowed to be similar
			++iterations_similar_transforms_;
			return (false);
		}
		else
		{
			iterations_similar_transforms_ = 0;
			convergence_state_ = CONVERGENCE_CRITERIA_TRANSFORM;
			return (true);
		}
	}

	//cout << "correspondences_original = " << correspondences_original.size() << endl;

	if(M_i_method == 2)	correspondences_cur_mse_ = calculateMSE(correspondences_spring1);
	else if (M_i_method == 3) correspondences_cur_mse_ = calculateMSE(correspondences_spring2);
	else correspondences_cur_mse_ = calculateMSE(correspondences_original);
	PCL_DEBUG("[pcl::DefaultConvergenceCriteria::hasConverged] Previous / Current MSE for correspondences distances is: %f / %f.\n", correspondences_prev_mse_, correspondences_cur_mse_);

	// 3. The relative sum of Euclidean squared errors is smaller than a user defined threshold
	// Absolute
	if (fabs(correspondences_cur_mse_ - correspondences_prev_mse_) < mse_threshold_absolute_)
	{
		if (iterations_similar_transforms_ < max_iterations_similar_transforms_)
		{
			// Increment the number of transforms that the thresholds are allowed to be similar
			++iterations_similar_transforms_;
			return (false);
		}
		else
		{
			iterations_similar_transforms_ = 0;
			convergence_state_ = CONVERGENCE_CRITERIA_ABS_MSE;
			return (true);
		}
	}

	// Relative
	if (fabs(correspondences_cur_mse_ - correspondences_prev_mse_) / correspondences_prev_mse_ < mse_threshold_relative_)
	{
		if (iterations_similar_transforms_ < max_iterations_similar_transforms_)
		{
			// Increment the number of transforms that the thresholds are allowed to be similar
			++iterations_similar_transforms_;
			return (false);
		}
		else
		{
			iterations_similar_transforms_ = 0;
			convergence_state_ = CONVERGENCE_CRITERIA_REL_MSE;
			return (true);
		}
	}

	correspondences_prev_mse_ = correspondences_cur_mse_;

	return (false);
}

