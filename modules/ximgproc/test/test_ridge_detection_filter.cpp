/*
 *  By downloading, copying, installing or using the software you agree to this license.
 *  If you do not agree to this license, do not download, install,
 *  copy or use the software.
 *
 *
 *  License Agreement
 *  For Open Source Computer Vision Library
 *  (3 - clause BSD License)
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met :
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and / or other materials provided with the distribution.
 *
 *  * Neither the names of the copyright holders nor the names of the contributors
 *  may be used to endorse or promote products derived from this software
 *  without specific prior written permission.
 *
 *  This software is provided by the copyright holders and contributors "as is" and
 *  any express or implied warranties, including, but not limited to, the implied
 *  warranties of merchantability and fitness for a particular purpose are disclaimed.
 *  In no event shall copyright holders or contributors be liable for any direct,
 *  indirect, incidental, special, exemplary, or consequential damages
 *  (including, but not limited to, procurement of substitute goods or services;
 *  loss of use, data, or profits; or business interruption) however caused
 *  and on any theory of liability, whether in contract, strict liability,
 *  or tort(including negligence or otherwise) arising in any way out of
 *  the use of this software, even if advised of the possibility of such damage.
 */

#include "test_precomp.hpp"

namespace cvtest{
	using namespace std;
	using namespace std::tr1;
	using namespace testing;
	using namespace cv;
	using namespace cv::ximgproc;

	TEST(ximgproc_ridgedetectionfilter, compare){
		String openCVExtraDir =  cvtest::TS::ptr()->get_data_path();
		String srcImgPath = "cv/ximgproc/sources/01.jpg";
		String refPath = "cv/ximgproc/results/ridge_filter_test_ref/01.jpg";

		Mat src = imread(openCVExtraDir + srcImgPath);
		Mat ref;
		imread(openCVExtraDir + refPath, 0).convertTo(ref, CV_8UC1);

		Ptr<RidgeDetectionFilter> rdf = RidgeDetectionFilter::create();

		Mat out;
		rdf->getRidgeFilteredImage(src, out);
		Mat out_cmp;
		out.convertTo(out_cmp, CV_8UC1);

		Mat sb;
		subtract(out_cmp, ref, sb);
		int zeros = countNonZero(sb);
		EXPECT_EQ(zeros, 0);


	}

}