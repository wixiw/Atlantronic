#include "kernel/math/trigo.h"
#include <stdlib.h>
#include <math.h>

//! @file trigo.c
//! @brief Fonctions trigo en virgule fixe
//! @author Atlantronic
//!
//! Utilisant un tableau de sinus à 1024 entrées sur le quart de cercle et une interpolation linéaire entre les valeurs.

//! Tableau de sinus en virgule fixe (en 2^-30 unités) sur un quart de cercle
//! Il y a 1024 entrées et 2 entrées supplémentaires pour avoir un algo plus générique
//! Remarque la dernière entrée "ne sert à rien" (en valeur), elle permet de ne pas faire de cas particulier
//! lors de l'interpolation linéaire qui fait un accès à la case "n+1" (et va être multipliée par 0).
static const int32_t sin_tbl[1026] =
{
          0,    1647099,    3294193,    4941281,    6588356,    8235416,    9882456,   11529474,
   13176464,   14823423,   16470347,   18117233,   19764076,   21410872,   23057618,   24704310,
   26350943,   27997515,   29644021,   31290457,   32936819,   34583104,   36229307,   37875426,
   39521455,   41167391,   42813230,   44458968,   46104602,   47750128,   49395541,   51040837,
   52686014,   54331067,   55975992,   57620785,   59265442,   60909960,   62554335,   64198563,
   65842639,   67486561,   69130324,   70773924,   72417357,   74060620,   75703709,   77346620,
   78989349,   80631892,   82274245,   83916404,   85558366,   87200127,   88841683,   90483029,
   92124163,   93765079,   95405776,   97046247,   98686491,  100326502,  101966277,  103605812,
  105245103,  106884147,  108522939,  110161476,  111799753,  113437768,  115075515,  116712992,
  118350194,  119987118,  121623759,  123260114,  124896179,  126531950,  128167423,  129802595,
  131437462,  133072019,  134706263,  136340190,  137973796,  139607077,  141240030,  142872651,
  144504935,  146136880,  147768480,  149399733,  151030634,  152661180,  154291367,  155921191,
  157550647,  159179733,  160808445,  162436778,  164064728,  165692293,  167319468,  168946249,
  170572633,  172198615,  173824192,  175449360,  177074115,  178698453,  180322371,  181945865,
  183568930,  185191564,  186813762,  188435520,  190056834,  191677702,  193298119,  194918080,
  196537583,  198156624,  199775198,  201393302,  203010932,  204628085,  206244756,  207860942,
  209476638,  211091842,  212706549,  214320755,  215934457,  217547651,  219160334,  220772500,
  222384147,  223995270,  225605867,  227215933,  228825464,  230434456,  232042906,  233650811,
  235258165,  236864966,  238471210,  240076892,  241682010,  243286558,  244890535,  246493935,
  248096755,  249698991,  251300640,  252901697,  254502159,  256102022,  257701283,  259299937,
  260897982,  262495412,  264092224,  265688415,  267283981,  268878918,  270473223,  272066891,
  273659918,  275252302,  276844038,  278435122,  280025552,  281615322,  283204430,  284792871,
  286380643,  287967740,  289554160,  291139898,  292724951,  294309316,  295892988,  297475964,
  299058239,  300639811,  302220676,  303800829,  305380268,  306958988,  308536985,  310114257,
  311690799,  313266607,  314841679,  316416009,  317989595,  319562433,  321134518,  322705848,
  324276419,  325846226,  327415267,  328983538,  330551034,  332117752,  333683689,  335248841,
  336813204,  338376774,  339939549,  341501523,  343062693,  344623057,  346182609,  347741347,
  349299266,  350856364,  352412636,  353968079,  355522689,  357076462,  358629395,  360181484,
  361732726,  363283116,  364832652,  366381329,  367929144,  369476093,  371022173,  372567379,
  374111709,  375655159,  377197725,  378739403,  380280190,  381820082,  383359076,  384897167,
  386434353,  387970630,  389505993,  391040440,  392573967,  394106570,  395638246,  397168991,
  398698801,  400227673,  401755603,  403282588,  404808624,  406333708,  407857835,  409381002,
  410903207,  412424444,  413944711,  415464004,  416982319,  418499653,  420016002,  421531363,
  423045732,  424559105,  426071480,  427582852,  429093217,  430602573,  432110916,  433618242,
  435124548,  436629829,  438134084,  439637307,  441139496,  442640647,  444140756,  445639820,
  447137835,  448634799,  450130706,  451625555,  453119340,  454612060,  456103710,  457594286,
  459083786,  460572205,  462059541,  463545789,  465030947,  466515010,  467997976,  469479840,
  470960600,  472440251,  473918791,  475396216,  476872522,  478347705,  479821764,  481294693,
  482766489,  484237150,  485706671,  487175049,  488642281,  490108363,  491573292,  493037064,
  494499676,  495961124,  497421405,  498880516,  500338453,  501795212,  503250791,  504705185,
  506158392,  507610408,  509061229,  510510853,  511959275,  513406493,  514852502,  516297300,
  517740883,  519183248,  520624391,  522064309,  523502998,  524940456,  526376678,  527811662,
  529245404,  530677900,  532109148,  533539144,  534967884,  536395365,  537821584,  539246538,
  540670223,  542092635,  543513772,  544933630,  546352205,  547769495,  549185496,  550600205,
  552013618,  553425732,  554836544,  556246051,  557654248,  559061133,  560466703,  561870954,
  563273883,  564675486,  566075761,  567474703,  568872310,  570268579,  571663506,  573057087,
  574449320,  575840202,  577229728,  578617896,  580004702,  581390144,  582774218,  584156920,
  585538248,  586918198,  588296766,  589673951,  591049748,  592424154,  593797166,  595168781,
  596538995,  597907806,  599275210,  600641203,  602005783,  603368947,  604730691,  606091012,
  607449906,  608807372,  610163404,  611518001,  612871159,  614222875,  615573145,  616921967,
  618269338,  619615253,  620959711,  622302707,  623644239,  624984303,  626322897,  627660017,
  628995660,  630329823,  631662503,  632993696,  634323400,  635651611,  636978327,  638303543,
  639627258,  640949467,  642270169,  643589359,  644907034,  646223192,  647537830,  648850943,
  650162530,  651472587,  652781111,  654088099,  655393548,  656697454,  657999816,  659300629,
  660599890,  661897597,  663193747,  664488336,  665781362,  667072820,  668362709,  669651026,
  670937767,  672222928,  673506508,  674788504,  676068911,  677347728,  678624950,  679900576,
  681174602,  682447025,  683717842,  684987051,  686254647,  687520629,  688784993,  690047736,
  691308855,  692568348,  693826211,  695082441,  696337036,  697589992,  698841307,  700090977,
  701339000,  702585372,  703830092,  705073155,  706314559,  707554301,  708792378,  710028787,
  711263525,  712496590,  713727978,  714957687,  716185713,  717412054,  718636707,  719859669,
  721080937,  722300508,  723518380,  724734549,  725949013,  727161768,  728372813,  729582143,
  730789757,  731995651,  733199822,  734402269,  735602987,  736801974,  737999228,  739194745,
  740388522,  741580558,  742770848,  743959390,  745146182,  746331221,  747514503,  748696026,
  749875788,  751053785,  752230015,  753404474,  754577161,  755748072,  756917205,  758084557,
  759250125,  760413906,  761575898,  762736098,  763894504,  765051111,  766205919,  767358923,
  768510122,  769659512,  770807092,  771952857,  773096806,  774238936,  775379244,  776517728,
  777654384,  778789210,  779922204,  781053363,  782182683,  783310163,  784435800,  785559591,
  786681534,  787801625,  788919863,  790036244,  791150767,  792263427,  793374223,  794483153,
  795590213,  796695401,  797798714,  798900150,  799999706,  801097379,  802193167,  803287068,
  804379079,  805469196,  806557419,  807643743,  808728167,  809810688,  810891304,  811970011,
  813046808,  814121692,  815194659,  816265709,  817334838,  818402043,  819467323,  820530675,
  821592095,  822651583,  823709135,  824764748,  825818421,  826870150,  827919934,  828967769,
  830013654,  831057586,  832099562,  833139580,  834177638,  835213733,  836247863,  837280024,
  838310216,  839338435,  840364679,  841388945,  842411232,  843431536,  844449856,  845466188,
  846480531,  847492882,  848503239,  849511600,  850517961,  851522321,  852524677,  853525028,
  854523370,  855519701,  856514019,  857506321,  858496606,  859484870,  860471112,  861455330,
  862437520,  863417681,  864395810,  865371905,  866345964,  867317984,  868287963,  869255900,
  870221790,  871185633,  872147426,  873107167,  874064853,  875020483,  875974054,  876925563,
  877875009,  878822389,  879767701,  880710943,  881652112,  882591207,  883528225,  884463164,
  885396022,  886326796,  887255485,  888182086,  889106597,  890029016,  890949341,  891867569,
  892783698,  893697727,  894609652,  895519473,  896427186,  897332790,  898236282,  899137661,
  900036924,  900934069,  901829095,  902721998,  903612776,  904501429,  905387953,  906272347,
  907154608,  908034735,  908912725,  909788576,  910662286,  911533853,  912403276,  913270551,
  914135678,  914998653,  915859476,  916718143,  917574653,  918429004,  919281194,  920131221,
  920979082,  921824777,  922668302,  923509656,  924348837,  925185843,  926020672,  926853322,
  927683790,  928512076,  929338177,  930162092,  930983817,  931803352,  932620694,  933435842,
  934248793,  935059546,  935868098,  936674448,  937478595,  938280535,  939080267,  939877790,
  940673101,  941466198,  942257081,  943045745,  943832191,  944616416,  945398418,  946178196,
  946955747,  947731070,  948504163,  949275023,  950043650,  950810042,  951574196,  952336111,
  953095785,  953853216,  954608403,  955361344,  956112036,  956860479,  957606670,  958350608,
  959092290,  959831716,  960568883,  961303790,  962036435,  962766816,  963494932,  964220780,
  964944360,  965665669,  966384706,  967101468,  967815955,  968528165,  969238095,  969945745,
  970651112,  971354196,  972054994,  972753504,  973449725,  974143656,  974835295,  975524639,
  976211688,  976896441,  977578894,  978259047,  978936898,  979612445,  980285688,  980956623,
  981625251,  982291568,  982955574,  983617267,  984276646,  984933708,  985588453,  986240879,
  986890984,  987538766,  988184225,  988827359,  989468165,  990106644,  990742793,  991376610,
  992008094,  992637245,  993264059,  993888536,  994510675,  995130473,  995747930,  996363043,
  996975812,  997586236,  998194311,  998800038,  999403415, 1000004439, 1000603111, 1001199428,
 1001793390, 1002384994, 1002974239, 1003561124, 1004145648, 1004727809, 1005307605, 1005885036,
 1006460100, 1007032796, 1007603122, 1008171077, 1008736660, 1009299870, 1009860704, 1010419162,
 1010975242, 1011528943, 1012080264, 1012629204, 1013175761, 1013719934, 1014261721, 1014801122,
 1015338134, 1015872758, 1016404991, 1016934832, 1017462281, 1017987335, 1018509994, 1019030256,
 1019548121, 1020063586, 1020576651, 1021087314, 1021595575, 1022101432, 1022604883, 1023105929,
 1023604567, 1024100796, 1024594615, 1025086024, 1025575020, 1026061603, 1026545772, 1027027525,
 1027506862, 1027983780, 1028458280, 1028930359, 1029400018, 1029867254, 1030332067, 1030794455,
 1031254418, 1031711954, 1032167062, 1032619742, 1033069992, 1033517810, 1033963197, 1034406151,
 1034846671, 1035284755, 1035720404, 1036153615, 1036584389, 1037012723, 1037438617, 1037862069,
 1038283080, 1038701647, 1039117770, 1039531448, 1039942680, 1040351465, 1040757802, 1041161689,
 1041563127, 1041962114, 1042358649, 1042752731, 1043144360, 1043533534, 1043920252, 1044304514,
 1044686319, 1045065665, 1045442553, 1045816980, 1046188946, 1046558451, 1046925492, 1047290071,
 1047652185, 1048011834, 1048369016, 1048723732, 1049075980, 1049425759, 1049773069, 1050117909,
 1050460278, 1050800175, 1051137599, 1051472550, 1051805027, 1052135029, 1052462555, 1052787604,
 1053110176, 1053430270, 1053747885, 1054063021, 1054375676, 1054685850, 1054993543, 1055298753,
 1055601479, 1055901722, 1056199480, 1056494753, 1056787540, 1057077840, 1057365653, 1057650977,
 1057933813, 1058214159, 1058492016, 1058767381, 1059040255, 1059310638, 1059578527, 1059843923,
 1060106826, 1060367233, 1060625146, 1060880563, 1061133483, 1061383907, 1061631833, 1061877261,
 1062120190, 1062360620, 1062598550, 1062833980, 1063066909, 1063297336, 1063525261, 1063750684,
 1063973603, 1064194019, 1064411931, 1064627338, 1064840240, 1065050636, 1065258526, 1065463909,
 1065666786, 1065867154, 1066065015, 1066260367, 1066453210, 1066643544, 1066831367, 1067016680,
 1067199483, 1067379774, 1067557554, 1067732821, 1067905576, 1068075818, 1068243547, 1068408763,
 1068571464, 1068731650, 1068889322, 1069044479, 1069197120, 1069347245, 1069494854, 1069639946,
 1069782521, 1069922579, 1070060120, 1070195142, 1070327646, 1070457632, 1070585099, 1070710046,
 1070832474, 1070952382, 1071069770, 1071184638, 1071296985, 1071406812, 1071514117, 1071618901,
 1071721163, 1071820903, 1071918122, 1072012818, 1072104991, 1072194642, 1072281769, 1072366374,
 1072448455, 1072528012, 1072605046, 1072679556, 1072751542, 1072821003, 1072887940, 1072952352,
 1073014240, 1073073603, 1073130440, 1073184753, 1073236540, 1073285802, 1073332538, 1073376748,
 1073418433, 1073457592, 1073494225, 1073528332, 1073559913, 1073588967, 1073615496, 1073639498,
 1073660973, 1073679922, 1073696345, 1073710241, 1073721611, 1073730454, 1073736771, 1073740561,
 1073741824,          0,
};

int32_t fx_sin(int32_t alpha)
{
	int sign = 1;
	if(alpha < 0)
	{
		alpha = -alpha;
		sign = -1;
	}

	switch( (alpha >> 24) & 0x03 )
	{
		default:
		case 0:
			// premier quadrant - rien a faire
			break;
		case 1:
			// second quadrant
			alpha = 0x1000000 - (alpha & 0x0ffffff);
			break;
		case 2:
			// 3 quadrant
			alpha &= 0x0ffffff;
			sign *= -1;
			break;
		case 3:
			// 4 quadrant
			alpha = 0x1000000 - (alpha & 0x0ffffff);
			sign *= -1;
			break;
	}

	int id = (alpha >> 14) & 0x7ff;
	return sign * (sin_tbl[id] + (((int64_t)(alpha & 0x3fff) * (int64_t)(sin_tbl[id + 1] - sin_tbl[id])) >> 14));
}

int32_t fx_cos(int32_t alpha)
{
	return fx_sin(0x1000000 - alpha);
}

int32_t fx_atan2(int32_t y, int32_t x)
{
	int32_t t0, t1, t2, t3;
	int n = 0;

	t2 = abs(x);
	t1 = abs(y);
	if(t2 > t1)
	{
		t0 = t2;
	}
	else
	{
		t0 = t1;
		t1 = t2;
		n = 1;
	}

	// t2 < 1 car t1 > t0
	t2 = ((int64_t)t1 << 30) / t0;
	t3 = ((int64_t)t2 * (int64_t)t2) >> 30;
	t0 = -2303695;
	t0 = (((int64_t)t0 * (int64_t)t3) >> 30) + 9822374;
	t0 = (((int64_t)t0 * (int64_t)t3) >> 30) - 20718705;
	t0 = (((int64_t)t0 * (int64_t)t3) >> 30) + 33432481;
	t0 = (((int64_t)t0 * (int64_t)t3) >> 30) - 56905886;
	t0 = (((int64_t)t0 * (int64_t)t3) >> 30) + 170890572;
	t2 = ((int64_t)t0 * (int64_t)t2) >> 30;

	if( n)
	{
		t2 = 0x10000000 - t2; // 0.25 tour
	}
	if( x < 0)
	{
		t2 = 0x20000000 - t2; // 0.5 tour
	}
	if( y < 0)
	{
		t2 = -t2;
	}

	return t2 >> 4;
}

uint32_t fx_sqrt(uint32_t x)
{
	uint32_t result = 0;
	uint32_t bit;
	uint8_t n;

	// Many numbers will be less than 15, so
	// this gives a good balance between time spent
	// in if vs. time spent in the while loop
	// when searching for the starting value.
	if (x & 0xFFF00000)
	{
		bit = (uint32_t)1 << 30;
	}
	else
	{
		bit = (uint32_t)1 << 18;
	}

	while (bit > x) bit >>= 2;

	// The main part is executed twice, in order to avoid
	// using 64 bit values in computations.
	for (n = 0; n < 2; n++)
	{
		// First we get the top 24 bits of the answer.
		while (bit)
		{
			if (x >= result + bit)
			{
				x -= result + bit;
				result = (result >> 1) + bit;
			}
			else
			{
				result = (result >> 1);
			}
			bit >>= 2;
		}

		if (n == 0)
		{
			// Then process it again to get the lowest 8 bits.
			if (x > 65535)
			{
				// The remainder 'x' is too large to be shifted left
				// by 16, so we have to add 1 to result manually and
				// adjust 'x' accordingly.
				// x = a - (result + 0.5)^2
				//   = x + result^2 - (result + 0.5)^2
				//   = x - result - 0.5
				x -= result;
				x = (x << 16) - 0x8000;
				result = (result << 16) + 0x8000;
			}
			else
			{
				x <<= 16;
				result <<= 16;
			}

			bit = 1 << 14;
		}
	}

	// Finally, if next bit would have been 1, round the result upwards.
	if (x > result)
	{
		result++;
	}

	return result;
}

int32_t fx_acos(int32_t x)
{
	int32_t negate = 0;
	int32_t res = -3200675;

	if(x < 0)
	{
		negate = 1;
		x = -x;
	}

	if( x > 65536)
	{
		res = 0;
		goto end;
	}

	res = (((int64_t)res * (int64_t)x) >> 16) + 12690560;
	res = (((int64_t)res * (int64_t)x) >> 16) - 36248510;
	res = (((int64_t)res * (int64_t)x) >> 16) + 268423916;
	res = ((int64_t)res * (int64_t)fx_sqrt(65536-x)) >> 16;
	res = res - 2 * negate * res;

end:
	return negate * 0x2000000 + (res >> 4); // 0.5 tour
}

