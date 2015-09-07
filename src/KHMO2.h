/* 
 * File:   KHMO.h
 * Author: ctvr
 *
 * Created on August 31, 2015, 11:46 AM
 */

#ifndef KHMO2_H
#define	KHMO2_H

#include <vector>
#include <cmath>
#include <iostream>

class KHMO2
{
private:

    double min_dist_err;
    double alpha;
    double dist_thres;          ///< Maximum distance to not create a new cluster

public:

    struct KHMOCluster
    {
        double mk;
        //double avg;

        KHMOCluster(double x_i) : mk(x_i)//, avg(x_i)
        {
        }

        bool operator<(const KHMOCluster &cl) const
        {
            return mk < cl.mk;
        }
    };

    KHMO2(int _K, int _p, double _alpha = 0.01, double _dist_thres = 0) : p(_p), K(_K), dist(K), dist_thres(_dist_thres)
    {
        clusters.reserve(_K);
        pK_precomputed = (double) p*K;
        min_dist_err = std::pow(1e-16, 1 / (double) p); // avoid divisions by zero when obtaining sum_p
        set_alpha(_alpha);
    }

    size_t push(double x_i)
    {
        size_t k_min = 0;
        for (unsigned int k = 0; k < clusters.size(); ++k)
        {
            dist[k] = std::abs(clusters[k].mk - x_i);
            if (dist[k] < dist[k_min])
                k_min = k;
        }
        if (clusters.size() < K && (clusters.size() == 0 || 
                ((x_i > clusters[k_min].mk) ? x_i > (dist_thres + clusters[k_min].mk) : (dist_thres + x_i) < clusters[k_min].mk)))            // Initializes clusters by order of arrival. THey have to be sufficiently distant from each other
        {
            clusters.push_back(x_i);
            k_min = clusters.size() - 1;
        }
        else
        {
            double sum_p = 0;
            for (unsigned int k = 0; k < clusters.size(); ++k)
                sum_p += std::pow((double) std::max(dist[k], min_dist_err), -p); // Avoids division by zero and creates an upperbound to sum_p. notice the "-p"
            
            for (unsigned int k = 0; k < clusters.size(); ++k)
            {
                double b = std::pow((double) std::max(dist[k], min_dist_err), p); // adds a lowerbound to b. res has always to be higher than 1.0 
                if ((b * sum_p) < 0.9)
                    std::cout << "ERROR: Got an overflow somewhere " << sum_p << ", " << b << "\n";
                double res = std::max((b * sum_p) * (b * sum_p), 1.0);
                clusters[k].mk += alpha * (pK_precomputed / res) * (x_i - clusters[k].mk);
            }
            //clusters[k_min].avg += alpha * (x_i - clusters[k_min].avg);
        }
        
        return k_min;       // index sample was associated to
    }

    void set_alpha(double _alpha)
    {
        // We have to avoid instability by guaranteeing that alpha*pK_precomputed/res is lower than 1
        // We know that res>=1, therefore...
        alpha = std::min(_alpha, 1 / pK_precomputed);
        if (alpha != _alpha)
            std::cout << "alpha was readjusted to avoid instability. Actual alpha = " << alpha << "\n";
    }

    inline double get_alpha()
    {
        return alpha;
    }
    
    void reset()
    {
        clusters.clear();
    }

    int p;
    int K;
    double pK_precomputed;
    std::vector<KHMOCluster> clusters;
    std::vector<double> dist;
};

#endif	/* KHMO2_H */
