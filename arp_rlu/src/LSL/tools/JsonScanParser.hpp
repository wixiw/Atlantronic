/*
 * JsonScanParser.hpp
 *
 *  Created on: 2 mars 2012
 *      Author: Boris
 */

#ifndef _ARP_CORE_TOOLS_JSONSCANPARSER_HPP_
#define _ARP_CORE_TOOLS_JSONSCANPARSER_HPP_

#include <tools/vjson/JsonDocument.hpp>

#include "LSL/LaserScan.hpp"

namespace arp_rlu { namespace lsl {

/*!
 *  \addtogroup lsl
 *  @{
 */

/** \class JsonScanParser
 *
 * \brief Cette classe permet d'initialiser un LaserScan a partir de données contenues dans un fichier json.
 */
class JsonScanParser : vjson::JsonDocument
{
    public:
    /**
     * Constructeur par défault.
     * Aucun fichier n'est parsé.
     */
    JsonScanParser();

    /**
     * La méthode \ref parse est appelée dans le corps du constructeur.
     * \param le nom du fichier à parser sous la forme d'un std::string.
     */
    JsonScanParser(std::string filename);

    /**
     * Réalise le parsing du fichier json.
     * \param le nom du fichier sous la forme d'un char*
     * \return un booléen qui indique si le parsing s'est bien passé.
     * \remark Il suffit que le fichier satisfasse la synthaxe json pour que le parsing se passe bien. Le fichier peut ne pas contenir les données d'un LaserScan.
     */
    bool parse(const char *filename);

    /**
     * Permet d'obtenir le LaserScan.
     * \param[out] le scan si tout s'est bien passé
     * \return booléen qui indique si tout c'est bien passé, à savoir que le fichier json contenait bien un scan et que les données ont pu être obtenues.
     */
    bool getScan(LaserScan & ls);


};

/**
 * Permet d'exporter un LaserScan en json
 * \param[in] le scan à exporter
 * \param[in] le nom (chemin) du fichier json à créer
 * \return un booléen de succès
 */
bool export_json(const LaserScan & ls, const std::string filename);

/*! @} End of Doxygen Groups*/

}}


#endif // _ARP_CORE_TOOLS_JSONSCANPARSER_HPP_
