#include <map>
#include "ipc/Datagram.hpp"
#include "ipc_disco/RawMessage.hpp"

class I
{
public:
		virtual void truc(){};
		virtual void machin() = 0;
};

class A : public I
{
public:
		virtual void truc(){};
		void machin(){};
};

int main()
{
	std::map<int,int> yordle;
	yordle[3] = 4;

	arp_stm32::Datagram dtg;
	arp_stm32::RawMessage msg;

	yordle[26] = static_cast<int>(dtg.getHeader().type);
	yordle[27] = msg.getType();

//
//	I itf;
//	itf.truc();
	A a;
	a.truc();
	a.machin();

	std::map<int,int>::iterator element = yordle.find(3);
	if( element != yordle.end() )
		element->second++;
}
