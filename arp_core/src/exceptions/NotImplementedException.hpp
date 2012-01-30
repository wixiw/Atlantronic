/*
 * ARPExceptions.hpp
 *
 *  Created on: 29 January 2012
 *      Author: Boris
 */

#ifndef _ARP_CORE_NOTIMPLEMENTEDEXCEPTION_HPP_
#define _ARP_CORE_NOTIMPLEMENTEDEXCEPTION_HPP_

struct NotImplementedException : public std::exception
{
   NotImplementedException() {}
   const char* what() const throw() { return "Not Implemented yet"; }
};

#endif /* _ARP_CORE_NOTIMPLEMENTEDEXCEPTION_HPP_ */
