// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.
#include "perf_precomp.hpp"


namespace cvtest{
using std::tr1::tuple;
using std::tr1::get;
using namespace perf;
using namespace testing;
using namespace cv;
using namespace cv::ximgproc;

    CV_ENUM(DDEPTH, CV_32FC1);
    CV_ENUM(KSIZE, 3);
    typedef tuple<DDEPTH, KSIZE, Size> RDFParams;

    typedef TestBaseWithParam<RDFParams> RidgeDetectionFilterPerfTest;

    PERF_TEST_P(RidgeDetectionFilterPerfTest, perf, Combine(DDEPTH::all(), KSIZE::all(), SZ_TYPICAL)){
        RDFParams params = GetParam();
        int _ddepth = get<0>(params);
        int _ksize = get<1>(params);
        Size sz = get<2>(params);

        Mat src(sz, _ddepth);
        Mat out(sz, src.type());

        declare.in(src).out(out).tbb_threads(cv::getNumberOfCPUs());
        cv::setNumThreads(cv::getNumberOfCPUs());
        TEST_CYCLE_N(1){
            Ptr<RidgeDetectionFilter> rdf = RidgeDetectionFilter::create(_ddepth,1, 1, _ksize);
            rdf->getRidgeFilteredImage(src, out);
        }

        SANITY_CHECK_NOTHING();

    }

}