#define BOOST_TEST_MODULE SensingTests
#include <boost/test/unit_test.hpp>

#include "ChannelPowerEstimator.h"
#include "general_utils.hpp"

using namespace std;

BOOST_AUTO_TEST_SUITE(Sensing)

int count_bins(vector<int>& bin_mask, int val)
{
	int count = 0;
	for(int i = 0; i < bin_mask.size(); ++i)
		if(bin_mask[i]==val)
			count++;
	return count;
}

BOOST_AUTO_TEST_CASE(test1)
{
	int nBins = 512;
	int Nch = 4;

	auto bin_mask1 = sensing_utils::generate_bin_mask(Nch, nBins);
	auto bin_mask2 = sensing_utils::generate_bin_mask(Nch, nBins, 1.0);
	BOOST_REQUIRE(bin_mask1.size()==nBins);
	BOOST_REQUIRE(bin_mask2.size()==nBins);
//	cout << "Bin mask 1 with no guard\n";
//	cout << print_range(bin_mask1) << endl;
//	cout << "Bin mask 2 with no guard\n";
//	cout << print_range(bin_mask2) << endl;
	for(int i = 0; i < nBins; ++i)
	{
		BOOST_REQUIRE(bin_mask1[i]==bin_mask2[i]);
		BOOST_REQUIRE(bin_mask1[i]>=0);
	}

	auto bin_mask3 = sensing_utils::generate_bin_mask(Nch, nBins, 0.75);
	BOOST_REQUIRE(bin_mask3.size()==nBins);
	cout << "Bin mask 3 with guard of 25\%\n";
	cout << print_range(bin_mask3) << endl;	
	cout << "Number of non-assigned bins: " << count_bins(bin_mask3,-1) << endl;
	vector<int> ref_counts3;
	for(int i = 0; i < Nch; ++i)
		ref_counts3.push_back(count_bins(bin_mask3,i));
	cout << "Reference counts: " << print_range(ref_counts3) << endl;
	
	auto mask_pair = sensing_utils::generate_bin_mask_and_reference(Nch, nBins, 0.75, 0.1);
	auto bin_mask4 = mask_pair.first;
	auto ref_map = mask_pair.second;
	BOOST_REQUIRE(bin_mask4.size()==nBins);
	cout << "Bin mask 4 with guard of 25\% and reference 10\%\n";
	cout << print_range(bin_mask4) << endl;
	cout << "Number of non-assigned bins: " << count_bins(bin_mask4,-1) << endl;
	vector<int> ref_counts;
	vector<int> non_ref_counts;	
	for(int i = 0; i < ref_map.size(); ++i)
	{
		if(ref_map[i].second==true)
			non_ref_counts.push_back(count_bins(bin_mask4,i));
		else
			ref_counts.push_back(count_bins(bin_mask4,i));
	}
	cout << "Reference channel counts: " << endl;
	cout << print_range(ref_counts) << endl;
	cout << "Non-Reference channel counts: " << endl;
	cout << print_range(non_ref_counts) << endl;
}

BOOST_AUTO_TEST_SUITE_END()
