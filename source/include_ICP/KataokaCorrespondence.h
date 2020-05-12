#pragma once

#include <sstream>
#include <iostream>
#include <fstream>
#include<vector>

#include <Eigen/Core>

#include <boost/config.hpp>   // for broken compiler workarounds

using namespace std;

namespace Eigen {

	/// Extending Eigen namespace by adding frequently used matrix type
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;

}    // namespace Eigen

struct Correspondence_Kataoka
{
	/** \brief Index of the query (source) point. */
	int index_query;
	/** \brief Index of the matching (target) point. Set to -1 if no correspondence found. */
	int index_match;
	/** \brief Distance between the corresponding points, or the weight denoting the confidence in correspondence estimation */
	union
	{
		float distance;
		float weight;
	};

	/** \brief Standard constructor.
	  * Sets \ref index_query to 0, \ref index_match to -1, and \ref distance to FLT_MAX.
	  */
	inline Correspondence_Kataoka() : index_query(0), index_match(-1),
		distance(std::numeric_limits<float>::max())
	{}

	/** \brief Empty destructor. */
	virtual ~Correspondence_Kataoka() {}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef std::vector< Correspondence_Kataoka, Eigen::aligned_allocator<Correspondence_Kataoka> > Correspondences_Kataoka;
typedef boost::shared_ptr<Correspondences_Kataoka> CorrespondencesPtr_Kataoka;

struct Correspondence_Spring1
{
	/** \brief Index of the query (source) point. */
	int index_query;
	/** \brief Index of the matching (target) point. Set to -1 if no correspondence found. */
	int index_match;
	/** \brief Distance between the corresponding points, or the weight denoting the confidence in correspondence estimation */
	union
	{
		float distance;
		float weight;
	};
	double weight_spring;

	/** \brief Standard constructor.
	  * Sets \ref index_query to 0, \ref index_match to -1, and \ref distance to FLT_MAX.
	  */
	inline Correspondence_Spring1() : index_query(0), index_match(-1),
		distance(std::numeric_limits<float>::max()), weight_spring(0.)
	{}

	/** \brief Empty destructor. */
	virtual ~Correspondence_Spring1() {}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

typedef std::vector< Correspondence_Spring1, Eigen::aligned_allocator<Correspondence_Spring1> > Correspondences_Spring1;
typedef boost::shared_ptr<Correspondences_Spring1> CorrespondencesPtr_Spring1;

struct Correspondence_Spring2
{
	/** \brief Index of the query (source) point. */
	int index_query;
	/** \brief Index of the matching (target) point. Set to -1 if no correspondence found. */
	vector<int> index_match_vec;
	vector<float> distance_vec;
	/** \brief Distance between the corresponding points, or the weight denoting the confidence in correspondence estimation */
	//union
	//{
	//	float distance;
	//	float weight;
	//};

	vector<float> weight_spring_vec;

	/** \brief Standard constructor.
	  * Sets \ref index_query to 0, \ref index_match to -1, and \ref distance to FLT_MAX.
	  */
	inline Correspondence_Spring2()
		: index_query(0), index_match_vec(vector<int>(0)), distance_vec(vector<float>(0)),
		 weight_spring_vec(vector<float>(0))
	{}

	/** \brief Empty destructor. */
	virtual ~Correspondence_Spring2() {}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

typedef std::vector< Correspondence_Spring2, Eigen::aligned_allocator<Correspondence_Spring2> > Correspondences_Spring2;
typedef boost::shared_ptr<Correspondences_Spring2> CorrespondencesPtr_Spring2;

