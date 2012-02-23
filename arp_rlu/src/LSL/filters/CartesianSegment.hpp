/*
 * CartesianSegment.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_LSL_CARTESIANSEGMENT_HPP_
#define _ARP_RLU_LSL_CARTESIANSEGMENT_HPP_

#include <math/core>

#include "LSL/filters/ParamsInterface.hpp"
#include "LSL/LaserScan.hpp"
#include <vector>

namespace arp_rlu
{

namespace lsl
{

/*!
 *  \addtogroup lsl
 *  @{
 */

/** \class CartesianSegment
 *
 * \brief CartesianSegment est un filtre qui permet de segmenter un scan en plusieurs éléments consistants d'un point de vue statistique.
 *
 * Le filtre utilise l'algo k-means afin de clusterizer le scan pour y détecter les objets cohérents statistiquement.
 *
 */
class CartesianSegment
{
    public:
        /** \ingroup lsl
         * \class Params
         *
         * \brief CartesianSegment::Params rassemble les paramètres du filtre CartesianSegment.
         *
         */
        class Params : ParamsInterface
        {
            public:
            /** Constructeur par défault.
             *  Il initialise des paramètres classiques non-stupides :\n
             *  kmeansMaxIterations = 10
             *  kmeansDispThres = 0.01
             *  minNbPoints = 5
             *  maxStddev = 0.1
             */
            Params();

            /**
             * Permet de formatter les paramètres en un message lisible.
             */
            std::string getInfo();

            /**
             * Permet de vérifier que les paramètres sont consistants.\n
             * A savoir : \n
             * * kmeansMaxIterations > 0\n
             * * kmeansDispThres > 0.0\n
             * * minNbPoints > 0\n
             * * maxStddev > 0.
             */
            bool checkConsistency() const;

            /**
             * Nombre d'itération maximum pour l'algo k-means.
             */
            unsigned int kmeansMaxIterations;

            /**
             * Seuil de déplacement en dessous duquel on arrête les itérations du k-means.
             */
            double kmeansDispThres;

            /**
             * Nombre de points minimal accepter dans un objet.\n
             * Si un objet détecté a un nombre de points inférieur à ce seuil, il est ignoré.
             */
            unsigned int minNbPoints;

            /**
             * Ecart type maximal d'un objet.\n
             * Au delà de cet écart type, on considère qu'on a affaire à plusieurs objet.
             */
            double maxStddev;
        };

    public:
        /** Applique le filtre sur un scan
         * \param ls scan d'origine
         * \param p paramètres du filtre
         * \return vecteur de LaserScan
         */
        static std::vector<LaserScan> apply(const LaserScan &, const Params & p = Params());

        /** Applique l'algorithme k-moyenne sur le LaserScan avec deux graines.
         * \param ls scan d'origine
         * \param p paramètres du filtre
         * \return paire de LaserScan (correspondant aux deux graines)
         * \remarks Cette méthode est publique seulement à des fins d'unittesting.
         * Il n'est pas recommandé de l'utiliser depuis l'extérieur. Utiliser plutôt la méthode apply
         * qui est bien plus adaptée aux besoins hauts niveaux.
         */
        static std::pair<LaserScan, LaserScan> kMeans(const LaserScan & s, const Params & p = Params());


    protected:

};

/*! @} End of Doxygen Groups*/

} // namespace lsl

} // namespace arp_rlu

#endif /* _ARP_RLU_LSL_CARTESIANSEGMENT_HPP_ */
