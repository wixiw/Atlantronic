/*
 * JsonDocument.hpp
 *
 *  Created on: 29 february 2012
 *      Author: Boris
 */

#ifndef _ARP_CORE_TOOLS_JSONDOCUMENT_HPP_
#define _ARP_CORE_TOOLS_JSONDOCUMENT_HPP_

#include <vector>
#include <string>
#include <stdio.h>

#include "tools/vjson/json.h"
#include "tools/vjson/block_allocator.h"

namespace vjson
{

/**
 * \class JsonDocument
 *
 * \brief Cette classe permet d'accéder au contenu d'un fichier json.
 */
class JsonDocument
{
    private:
        block_allocator mAllocator;
        json_value *mRoot;

    public:
        /**
         * Constructeur par défault.
         */
        JsonDocument(): mAllocator(1 << 10), mRoot(0)
        {
        }

        /**
         * La méthode de parsing.\n
         * \param le nom du fichier à parser
         * \return un booléen qui indique si tout s'est bien passé.
         */
        bool parse(const char *filename);

        /**
         * Permet d'accéder au noeud racine du json.
         */
        json_value *root()
        {
            return mRoot;
        }

        /**
         * Pour un noeud donné, permet d'accéder aux noms de ses enfants.
         * \param le noeud parent
         * \return le vecteur des noms des enfants.
         * \warning Les enfants peuvent ne pas tous avoir de nom. Alors, le vecteur contient un (des) string vides.
         */
        std::vector< std::string > getChildNames( json_value* value );

        /**
         * Pour un noeud donné, permet d'accéder aux types de ses enfants.
         * \param le noeud parent
         * \return le vecteur des types des enfants.
         */
        std::vector< json_type > getChildTypes( json_value* value );

        /**
         * Permet d'accéder à un enfant par son nom
         * \param le noeud parent
         * \param le nom de l'enfant
         * \return le premier enfant qui porte le nom spécifié.
         * \warning Si aucun enfant ne porte le nom spécifié, la méthode renvoie NULL.
         */
        json_value* getChild( json_value* value, std::string name );

        /**
         * Permet d'accéder à l'ième enfant d'un parent.
         * \param le noeud parent
         * \param l'indice i de l'enfant
         * \return le ième enfant.
         * \warning Si l'index est supérieur ou égal au nombre d'enfant, la méthode renvoie NULL.
         */
        json_value* getChild( json_value* value, unsigned int index );

        /**
         * Pour un noeud donné, permet d'obtenir sa value entière.
         * \warning Si le noeud n'est pas de type Int, la méthode renvoie 0
         */
        int getIntegerData(json_value * value);

        /**
         * Pour un noeud donné, permet d'obtenir sa value flotante.
         * \warning Si le noeud n'est pas de type Float, la méthode renvoie 0.0
         */
        float getFloatData(json_value * value);

        /**
         * Pour un noeud donné, permet d'obtenir sa value en chaine de caractères.
         * \warning Si le noeud n'est pas de type String, la méthode renvoie un string vide.
         */
        std::string getStringData(json_value * value);
};


}

#endif //_ARP_CORE_TOOLS_JSONDOCUMENT_HPP_
