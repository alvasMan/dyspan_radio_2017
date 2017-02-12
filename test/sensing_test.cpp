#define BOOST_TEST_MODULE SensingTests
#include <boost/test/unit_test.hpp>

#include "ChannelPowerEstimator.h"
#include "general_utils.hpp"
#include "sensing_components.h"

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

	auto mask1 = sensing_utils::generate_bin_mask_no_guard(Nch, nBins);
	auto mask2 = sensing_utils::generate_bin_mask(Nch, nBins, 1.0);
	BOOST_REQUIRE(mask1.size()==nBins);
	BOOST_REQUIRE(mask2.size()==nBins);
        BOOST_REQUIRE(mask1.Nch==Nch);
        BOOST_REQUIRE(mask2.Nch==Nch);
//	cout << "Bin mask 1 with no guard\n";
//	cout << print_range(bin_mask1) << endl;
//	cout << "Bin mask 2 with no guard\n";
//	cout << print_range(bin_mask2) << endl;
        
        // Check if the two masks are equal
        BOOST_REQUIRE(std::equal(mask1.begin(), mask1.end(), mask2.begin()));
        // There should be guard bins bc of DC offset
        //BOOST_REQUIRE(std::find_if(mask1.begin(), mask1.end(),[](int i){return i < 0;})==mask1.end());
        // Check if the count of bins for each channel is the expected
        BOOST_REQUIRE(count_bins(mask1.bin_mask,-1)==mask1.ignored_section_props.count);
        for(int i = 0; i < Nch; ++i)
        {
            BOOST_REQUIRE(count_bins(mask1.bin_mask,i)==mask1.section_props[i].count);
            BOOST_REQUIRE(count_bins(mask1.bin_mask,i)==mask2.section_props[i].count);
        }
        
	cout << "Bin mask 3 with guard of 25\%\n";
	auto mask3 = sensing_utils::generate_bin_mask(Nch, nBins, 0.75);
	BOOST_REQUIRE(mask3.size()==nBins);
	cout << print_range(mask3.bin_mask) << endl;	
	cout << "Number of non-assigned bins: " << count_bins(mask3.bin_mask,-1) << endl;
	vector<int> ref_counts3;
	for(int i = 0; i < Nch; ++i)
        {
            auto c = count_bins(mask3.bin_mask,i);
            BOOST_REQUIRE(c==mask3.section_props[i].count);
            BOOST_REQUIRE(mask3.section_props[i].type==BinMask::valid);
            ref_counts3.push_back(c);
	}
        cout << "Reference counts: " << print_range(ref_counts3) << endl;
}


BOOST_AUTO_TEST_CASE(test2)
{
	int nBins = 512;
	int Nch = 4;
        
        // TEST with noise reference sections of the spectrogram
	cout << "Bin mask 4 with guard of 25\% and reference 10\%\n";
	auto mask4 = sensing_utils::generate_bin_mask_and_reference(Nch, nBins, 0.75, 0.1);
	BOOST_REQUIRE(mask4.size()==nBins);
        BOOST_REQUIRE(mask4.Nch==Nch);
        BOOST_REQUIRE(mask4.n_sections()==3*Nch);
        BOOST_REQUIRE(mask4.ignored_section_props.count==count_bins(mask4.bin_mask,-1));
        for(int i = 0; i < mask4.n_sections(); ++i)
        {
            BOOST_REQUIRE(mask4.section_props[i].count==count_bins(mask4.bin_mask,i));
            BOOST_REQUIRE((i%3==1 && mask4.section_props[i].type==BinMask::valid) || (i%3!=1 && mask4.section_props[i].type==BinMask::reference));
        }
//	cout << print_range(bin_mask4) << endl;
//	cout << "Number of non-assigned bins: " << count_bins(bin_mask4,-1) << endl;
//        
        vector<float> sp_powers(Nch*3,1);
        vector<float> test_powers = {1.5,2.5,3.5,4.5};
        for(int i = 0; i < Nch; ++i)
            sp_powers[i*3+1] += i + 0.5;
        cout << "Original channel powers: " << print_range(sp_powers) << endl;
        vector<float> ch_powers = sensing_utils::relative_channel_powers(mask4, sp_powers);
        cout << "Channel powers: " << print_range(ch_powers) << endl;
        BOOST_REQUIRE(ch_powers.size()==test_powers.size());
        BOOST_REQUIRE(std::equal(ch_powers.begin(), ch_powers.end(), test_powers.begin()));
}

BOOST_AUTO_TEST_CASE(test3)
{   
    int Nch = 4;
    int Nfft =512;
    auto maskprops = sensing_utils::generate_bin_mask_and_reference(Nch, Nfft, 0.8, 0.15);
    
    BOOST_REQUIRE(maskprops.Nch==Nch);
    
    SpectrogramResizer s_resizer(maskprops,64);
    vector<float> in_vec = {1,0.5,1.1, 1.4,3,1.2, 1.1,10,1.3, 1.2,100,1.1};
    vector<float> out_vec(64);
    
    s_resizer.resize_line(out_vec, in_vec);
    cout << "Input vector to the resizer: " << print_range(in_vec) << endl;
    cout << "Output vector of the resizer: " << print_range(out_vec) << endl;
}

BOOST_AUTO_TEST_SUITE_END()
