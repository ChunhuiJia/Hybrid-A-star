extern "C" {
#include "dubins.h"
}

#include "gtest/gtest.h"

struct Inputs
{
    DubinsPathType word;
    double a;
    double b;
    double d;
};

struct Outputs
{
    int errcode;
    double params[3];
    double length;
};

struct Param
{
    Param(DubinsPathType w, double a, double b, double d, int code, double p1, double p2, double p3, double len)
    {
        inputs.a = a;
        inputs.b = b;
        inputs.d = d;
        inputs.word = w;
        outputs.errcode = code;
        outputs.params[0] = p1;
        outputs.params[1] = p2;
        outputs.params[2] = p3;
        outputs.length = len;

    }

    Inputs inputs;
    Outputs outputs;
};

Param params[] = {
Param(LSL, -1.57079632679490, -1.57079632679490, 0.200000000000000, 0, 1.57079632679490, 0.200000000000000, 4.71238898038469, 6.48318530717959),
	Param(LSR, -1.57079632679490, -1.57079632679490, 0.200000000000000, 1, 0, 0, 0, 0),
	Param(RSL, -1.57079632679490, -1.57079632679490, 0.200000000000000, 0, 5.85348564102816, 0.916515138991168, 5.85348564102816, 12.6234864210475),
	Param(RSR, -1.57079632679490, -1.57079632679490, 0.200000000000000, 0, 4.71238898038469, 0.200000000000000, 1.57079632679490, 6.48318530717959),
	Param(RLR, -1.57079632679490, -1.57079632679490, 0.200000000000000, 0, 1.52077546998913, 6.18314359356805, 4.66236812357892, 12.3662871871361),
	Param(LRL, -1.57079632679490, -1.57079632679490, 0.200000000000000, 0, 4.66236812357892, 6.18314359356805, 1.52077546998913, 12.3662871871361),
	Param(LSL, -1.57079632679490, -1.57079632679490, 0.600000000000000, 0, 1.57079632679490, 0.600000000000000, 4.71238898038469, 6.88318530717959),
	Param(LSR, -1.57079632679490, -1.57079632679490, 0.600000000000000, 1, 0, 0, 0, 0),
	Param(RSL, -1.57079632679490, -1.57079632679490, 0.600000000000000, 0, 5.59002539960283, 1.66132477258361, 5.59002539960283, 12.8413755717893),
	Param(RSR, -1.57079632679490, -1.57079632679490, 0.600000000000000, 0, 4.71238898038469, 0.600000000000000, 1.57079632679490, 6.88318530717959),
	Param(RLR, -1.57079632679490, -1.57079632679490, 0.600000000000000, 0, 1.42022805401821, 5.98204876162621, 4.56182070760800, 11.9640975232524),
	Param(LRL, -1.57079632679490, -1.57079632679490, 0.600000000000000, 0, 4.56182070760800, 5.98204876162621, 1.42022805401821, 11.9640975232524),
	Param(LSL, -1.57079632679490, -1.57079632679490, 1.20000000000000, 0, 1.57079632679490, 1.20000000000000, 4.71238898038469, 7.48318530717959),
	Param(LSR, -1.57079632679490, -1.57079632679490, 1.20000000000000, 1, 0, 0, 0, 0),
	Param(RSL, -1.57079632679490, -1.57079632679490, 1.20000000000000, 0, 5.38752051332172, 2.49799919935936, 5.38752051332172, 13.2730402260028),
	Param(RSR, -1.57079632679490, -1.57079632679490, 1.20000000000000, 0, 4.71238898038469, 1.20000000000000, 1.57079632679490, 7.48318530717959),
	Param(RLR, -1.57079632679490, -1.57079632679490, 1.20000000000000, 0, 1.26610367277950, 5.67379999914879, 4.40769632636929, 11.3475999982976),
	Param(LRL, -1.57079632679490, -1.57079632679490, 1.20000000000000, 0, 4.40769632636929, 5.67379999914879, 1.26610367277950, 11.3475999982976),
	Param(LSL, -1.57079632679490, -1.57079632679490, 10.0000000000000, 0, 1.57079632679490, 10.0000000000000, 4.71238898038469, 16.2831853071796),
	Param(LSR, -1.57079632679490, -1.57079632679490, 10.0000000000000, 0, 1.82347658193698, 7.74596669241483, 1.82347658193698, 11.3929198562888),
	Param(RSL, -1.57079632679490, -1.57079632679490, 10.0000000000000, 0, 4.87983705960438, 11.8321595661992, 4.87983705960438, 21.5918336854080),
	Param(RSR, -1.57079632679490, -1.57079632679490, 10.0000000000000, 0, 4.71238898038469, 10.0000000000000, 1.57079632679490, 16.2831853071796),
	Param(RLR, -1.57079632679490, -1.57079632679490, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LRL, -1.57079632679490, -1.57079632679490, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LSL, -1.57079632679490, 0, 0.200000000000000, 0, 3.81633359581335, 1.28062484748657, 4.03764803816114, 9.13460648146105),
	Param(LSR, -1.57079632679490, 0, 0.200000000000000, 1, 0, 0, 0, 0),
	Param(RSL, -1.57079632679490, 0, 0.200000000000000, 1, 0, 0, 0, 0),
	Param(RSR, -1.57079632679490, 0, 0.200000000000000, 0, 5.40712725658139, 1.56204993518133, 5.58844703098288, 12.5576242227456),
	Param(RLR, -1.57079632679490, 0, 0.200000000000000, 0, 1.86434638999434, 5.48080888118507, 2.04566616439583, 9.39082143557524),
	Param(LRL, -1.57079632679490, 0, 0.200000000000000, 0, 0.348846568576992, 5.63139655988647, 0.570161010924784, 6.55040413938824),
	Param(LSL, -1.57079632679490, 0, 0.600000000000000, 0, 3.52209903070216, 1.07703296142690, 4.33188260327232, 8.93101459540138),
	Param(LSR, -1.57079632679490, 0, 0.600000000000000, 1, 0, 0, 0, 0),
	Param(RSL, -1.57079632679490, 0, 0.600000000000000, 1, 0, 0, 0, 0),
	Param(RSR, -1.57079632679490, 0, 0.600000000000000, 0, 5.27098829572825, 1.88679622641132, 5.72458599183602, 12.8823705139756),
	Param(RLR, -1.57079632679490, 0, 0.600000000000000, 0, 1.63817896180931, 5.30075194652129, 2.09177665791708, 9.03070756624769),
	Param(LRL, -1.57079632679490, 0, 0.600000000000000, 0, 0.107883633503051, 5.73793981996096, 0.917667206073218, 6.76349065953723),
	Param(LSL, -1.57079632679490, 0, 1.20000000000000, 0, 2.94419709373991, 1.01980390271856, 4.90978454023457, 8.87378553669304),
	Param(LSR, -1.57079632679490, 0, 1.20000000000000, 1, 0, 0, 0, 0),
	Param(RSL, -1.57079632679490, 0, 1.20000000000000, 0, 5.26059043786122, 1.35646599662505, 0.548201457476526, 7.16525789196279),
	Param(RSR, -1.57079632679490, 0, 1.20000000000000, 0, 5.13901647351157, 2.41660919471891, 5.85655781405271, 13.4121834822832),
	Param(RLR, -1.57079632679490, 0, 1.20000000000000, 0, 1.34872217225806, 4.98578201185216, 2.06626351279921, 8.40076769690943),
	Param(LRL, -1.57079632679490, 0, 1.20000000000000, 0, 5.82799274420666, 5.76759130093350, 1.51039488352173, 13.1059789286619),
	Param(LSL, -1.57079632679490, 0, 10.0000000000000, 0, 1.68145354796879, 9.05538513813742, 6.17252808600569, 16.9093667721119),
	Param(LSR, -1.57079632679490, 0, 10.0000000000000, 0, 1.68283839261674, 8.83176086632785, 0.112042065821841, 10.6266413247664),
	Param(RSL, -1.57079632679490, 0, 10.0000000000000, 0, 4.80380493701101, 10.8627804912002, 0.0914159566263198, 15.7580013848375),
	Param(RSR, -1.57079632679490, 0, 10.0000000000000, 0, 4.80304886758544, 11.0453610171873, 6.19252541997884, 22.0409353047515),
	Param(RLR, -1.57079632679490, 0, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LRL, -1.57079632679490, 0, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LSL, -1.57079632679490, 1.57079632679490, 0.200000000000000, 0, 4.71238898038469, 1.80000000000000, 4.71238898038469, 11.2247779607694),
	Param(LSR, -1.57079632679490, 1.57079632679490, 0.200000000000000, 1, 0, 0, 0, 0),
	Param(RSL, -1.57079632679490, 1.57079632679490, 0.200000000000000, 1, 0, 0, 0, 0),
	Param(RSR, -1.57079632679490, 1.57079632679490, 0.200000000000000, 0, 4.71238898038469, 2.20000000000000, 4.71238898038469, 11.6247779607694),
	Param(RLR, -1.57079632679490, 1.57079632679490, 0.200000000000000, 0, 0.988432088926153, 5.11845683144210, 0.988432088926153, 7.09532100929441),
	Param(LRL, -1.57079632679490, 1.57079632679490, 0.200000000000000, 0, 1.10403098774760, 5.34965462908499, 1.10403098774760, 7.55771660458019),
	Param(LSL, -1.57079632679490, 1.57079632679490, 0.600000000000000, 0, 4.71238898038469, 1.40000000000000, 4.71238898038469, 10.8247779607694),
	Param(LSR, -1.57079632679490, 1.57079632679490, 0.600000000000000, 1, 0, 0, 0, 0),
	Param(RSL, -1.57079632679490, 1.57079632679490, 0.600000000000000, 1, 0, 0, 0, 0),
	Param(RSR, -1.57079632679490, 1.57079632679490, 0.600000000000000, 0, 4.71238898038469, 2.60000000000000, 4.71238898038469, 12.0247779607694),
	Param(RLR, -1.57079632679490, 1.57079632679490, 0.600000000000000, 0, 0.863211890069541, 4.86801643372887, 0.863211890069541, 6.59444021386796),
	Param(LRL, -1.57079632679490, 1.57079632679490, 0.600000000000000, 0, 1.21322522314939, 5.56804309988857, 1.21322522314939, 7.99449354618734),
	Param(LSL, -1.57079632679490, 1.57079632679490, 1.20000000000000, 0, 4.71238898038469, 0.800000000000000, 4.71238898038469, 10.2247779607694),
	Param(LSR, -1.57079632679490, 1.57079632679490, 1.20000000000000, 1, 0, 0, 0, 0),
	Param(RSL, -1.57079632679490, 1.57079632679490, 1.20000000000000, 1, 0, 0, 0, 0),
	Param(RSR, -1.57079632679490, 1.57079632679490, 1.20000000000000, 0, 4.71238898038469, 3.20000000000000, 4.71238898038469, 12.6247779607694),
	Param(RLR, -1.57079632679490, 1.57079632679490, 1.20000000000000, 0, 0.643501108793284, 4.42859487117636, 0.643501108793284, 5.71559708876293),
	Param(LRL, -1.57079632679490, 1.57079632679490, 1.20000000000000, 0, 1.36943840600457, 5.88046946559892, 1.36943840600457, 8.61934627760806),
	Param(LSL, -1.57079632679490, 1.57079632679490, 10.0000000000000, 0, 1.57079632679490, 8.00000000000000, 1.57079632679490, 11.1415926535898),
	Param(LSR, -1.57079632679490, 1.57079632679490, 10.0000000000000, 0, 1.77215424758523, 9.79795897113271, 4.91374690117502, 16.4838601198930),
	Param(RSL, -1.57079632679490, 1.57079632679490, 10.0000000000000, 0, 4.91374690117502, 9.79795897113271, 1.77215424758523, 16.4838601198930),
	Param(RSR, -1.57079632679490, 1.57079632679490, 10.0000000000000, 0, 4.71238898038469, 12.0000000000000, 4.71238898038469, 21.4247779607694),
	Param(RLR, -1.57079632679490, 1.57079632679490, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LRL, -1.57079632679490, 1.57079632679490, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LSL, 0, -1.57079632679490, 0.200000000000000, 0, 5.58844703098288, 1.56204993518133, 5.40712725658139, 12.5576242227456),
	Param(LSR, 0, -1.57079632679490, 0.200000000000000, 1, 0, 0, 0, 0),
	Param(RSL, 0, -1.57079632679490, 0.200000000000000, 1, 0, 0, 0, 0),
	Param(RSR, 0, -1.57079632679490, 0.200000000000000, 0, 4.03764803816114, 1.28062484748657, 3.81633359581335, 9.13460648146105),
	Param(RLR, 0, -1.57079632679490, 0.200000000000000, 0, 0.570161010924784, 5.63139655988647, 0.348846568576992, 6.55040413938824),
	Param(LRL, 0, -1.57079632679490, 0.200000000000000, 0, 2.04566616439583, 5.48080888118507, 1.86434638999434, 9.39082143557524),
	Param(LSL, 0, -1.57079632679490, 0.600000000000000, 0, 5.72458599183602, 1.88679622641132, 5.27098829572825, 12.8823705139756),
	Param(LSR, 0, -1.57079632679490, 0.600000000000000, 1, 0, 0, 0, 0),
	Param(RSL, 0, -1.57079632679490, 0.600000000000000, 1, 0, 0, 0, 0),
	Param(RSR, 0, -1.57079632679490, 0.600000000000000, 0, 4.33188260327232, 1.07703296142690, 3.52209903070216, 8.93101459540138),
	Param(RLR, 0, -1.57079632679490, 0.600000000000000, 0, 0.917667206073218, 5.73793981996096, 0.107883633503051, 6.76349065953723),
	Param(LRL, 0, -1.57079632679490, 0.600000000000000, 0, 2.09177665791708, 5.30075194652129, 1.63817896180931, 9.03070756624769),
	Param(LSL, 0, -1.57079632679490, 1.20000000000000, 0, 5.85655781405271, 2.41660919471891, 5.13901647351157, 13.4121834822832),
	Param(LSR, 0, -1.57079632679490, 1.20000000000000, 1, 0, 0, 0, 0),
	Param(RSL, 0, -1.57079632679490, 1.20000000000000, 0, 0.548201457476526, 1.35646599662505, 5.26059043786122, 7.16525789196279),
	Param(RSR, 0, -1.57079632679490, 1.20000000000000, 0, 4.90978454023457, 1.01980390271856, 2.94419709373991, 8.87378553669304),
	Param(RLR, 0, -1.57079632679490, 1.20000000000000, 0, 1.51039488352173, 5.76759130093350, 5.82799274420666, 13.1059789286619),
	Param(LRL, 0, -1.57079632679490, 1.20000000000000, 0, 2.06626351279921, 4.98578201185216, 1.34872217225806, 8.40076769690943),
	Param(LSL, 0, -1.57079632679490, 10.0000000000000, 0, 6.19252541997884, 11.0453610171873, 4.80304886758544, 22.0409353047515),
	Param(LSR, 0, -1.57079632679490, 10.0000000000000, 0, 0.112042065821841, 8.83176086632785, 1.68283839261674, 10.6266413247664),
	Param(RSL, 0, -1.57079632679490, 10.0000000000000, 0, 0.0914159566263198, 10.8627804912002, 4.80380493701101, 15.7580013848375),
	Param(RSR, 0, -1.57079632679490, 10.0000000000000, 0, 6.17252808600569, 9.05538513813742, 1.68145354796879, 16.9093667721119),
	Param(RLR, 0, -1.57079632679490, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LRL, 0, -1.57079632679490, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LSL, 0, 0, 0.200000000000000, 0, 0, 0.200000000000000, 0, 0.200000000000000),
	Param(LSR, 0, 0, 0.200000000000000, 0, 0, 0.200000000000000, 0, 0.200000000000000),
	Param(RSL, 0, 0, 0.200000000000000, 0, 0, 0.200000000000000, 0, 0.200000000000000),
	Param(RSR, 0, 0, 0.200000000000000, 0, 0, 0.200000000000000, 0, 0.200000000000000),
	Param(RLR, 0, 0, 0.200000000000000, 0, 3.09157179678402, 6.18314359356805, 3.09157179678402, 12.3662871871361),
	Param(LRL, 0, 0, 0.200000000000000, 0, 3.09157179678402, 6.18314359356805, 3.09157179678402, 12.3662871871361),
	Param(LSL, 0, 0, 0.600000000000000, 0, 0, 0.600000000000000, 0, 0.600000000000000),
	Param(LSR, 0, 0, 0.600000000000000, 0, 0, 0.600000000000000, 0, 0.600000000000000),
	Param(RSL, 0, 0, 0.600000000000000, 0, 0, 0.600000000000000, 0, 0.600000000000000),
	Param(RSR, 0, 0, 0.600000000000000, 0, 0, 0.600000000000000, 0, 0.600000000000000),
	Param(RLR, 0, 0, 0.600000000000000, 0, 2.99102438081311, 5.98204876162621, 2.99102438081311, 11.9640975232524),
	Param(LRL, 0, 0, 0.600000000000000, 0, 2.99102438081311, 5.98204876162621, 2.99102438081311, 11.9640975232524),
	Param(LSL, 0, 0, 1.20000000000000, 0, 0, 1.20000000000000, 0, 1.20000000000000),
	Param(LSR, 0, 0, 1.20000000000000, 0, 0, 1.20000000000000, 0, 1.20000000000000),
	Param(RSL, 0, 0, 1.20000000000000, 0, 0, 1.20000000000000, 0, 1.20000000000000),
	Param(RSR, 0, 0, 1.20000000000000, 0, 0, 1.20000000000000, 0, 1.20000000000000),
	Param(RLR, 0, 0, 1.20000000000000, 0, 2.83689999957440, 5.67379999914879, 2.83689999957440, 11.3475999982976),
	Param(LRL, 0, 0, 1.20000000000000, 0, 2.83689999957440, 5.67379999914879, 2.83689999957440, 11.3475999982976),
	Param(LSL, 0, 0, 10.0000000000000, 0, 0, 10.0000000000000, 0, 10.0000000000000),
	Param(LSR, 0, 0, 10.0000000000000, 0, 0, 10.0000000000000, 0, 10.0000000000000),
	Param(RSL, 0, 0, 10.0000000000000, 0, 0, 10.0000000000000, 0, 10.0000000000000),
	Param(RSR, 0, 0, 10.0000000000000, 0, 0, 10.0000000000000, 0, 10.0000000000000),
	Param(RLR, 0, 0, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LRL, 0, 0, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LSL, 0, 1.57079632679490, 0.200000000000000, 0, 4.03764803816114, 1.28062484748657, 3.81633359581335, 9.13460648146105),
	Param(LSR, 0, 1.57079632679490, 0.200000000000000, 1, 0, 0, 0, 0),
	Param(RSL, 0, 1.57079632679490, 0.200000000000000, 1, 0, 0, 0, 0),
	Param(RSR, 0, 1.57079632679490, 0.200000000000000, 0, 5.58844703098288, 1.56204993518133, 5.40712725658139, 12.5576242227456),
	Param(RLR, 0, 1.57079632679490, 0.200000000000000, 0, 2.04566616439583, 5.48080888118507, 1.86434638999434, 9.39082143557524),
	Param(LRL, 0, 1.57079632679490, 0.200000000000000, 0, 0.570161010924784, 5.63139655988647, 0.348846568576992, 6.55040413938824),
	Param(LSL, 0, 1.57079632679490, 0.600000000000000, 0, 4.33188260327232, 1.07703296142690, 3.52209903070216, 8.93101459540138),
	Param(LSR, 0, 1.57079632679490, 0.600000000000000, 1, 0, 0, 0, 0),
	Param(RSL, 0, 1.57079632679490, 0.600000000000000, 1, 0, 0, 0, 0),
	Param(RSR, 0, 1.57079632679490, 0.600000000000000, 0, 5.72458599183602, 1.88679622641132, 5.27098829572825, 12.8823705139756),
	Param(RLR, 0, 1.57079632679490, 0.600000000000000, 0, 2.09177665791708, 5.30075194652129, 1.63817896180931, 9.03070756624769),
	Param(LRL, 0, 1.57079632679490, 0.600000000000000, 0, 0.917667206073218, 5.73793981996096, 0.107883633503051, 6.76349065953723),
	Param(LSL, 0, 1.57079632679490, 1.20000000000000, 0, 4.90978454023457, 1.01980390271856, 2.94419709373991, 8.87378553669304),
	Param(LSR, 0, 1.57079632679490, 1.20000000000000, 0, 0.548201457476526, 1.35646599662505, 5.26059043786122, 7.16525789196279),
	Param(RSL, 0, 1.57079632679490, 1.20000000000000, 1, 0, 0, 0, 0),
	Param(RSR, 0, 1.57079632679490, 1.20000000000000, 0, 5.85655781405271, 2.41660919471891, 5.13901647351157, 13.4121834822832),
	Param(RLR, 0, 1.57079632679490, 1.20000000000000, 0, 2.06626351279921, 4.98578201185216, 1.34872217225806, 8.40076769690943),
	Param(LRL, 0, 1.57079632679490, 1.20000000000000, 0, 1.51039488352173, 5.76759130093350, 5.82799274420666, 13.1059789286619),
	Param(LSL, 0, 1.57079632679490, 10.0000000000000, 0, 6.17252808600569, 9.05538513813742, 1.68145354796879, 16.9093667721119),
	Param(LSR, 0, 1.57079632679490, 10.0000000000000, 0, 0.0914159566263198, 10.8627804912002, 4.80380493701101, 15.7580013848375),
	Param(RSL, 0, 1.57079632679490, 10.0000000000000, 0, 0.112042065821841, 8.83176086632785, 1.68283839261674, 10.6266413247664),
	Param(RSR, 0, 1.57079632679490, 10.0000000000000, 0, 6.19252541997884, 11.0453610171873, 4.80304886758544, 22.0409353047515),
	Param(RLR, 0, 1.57079632679490, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LRL, 0, 1.57079632679490, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LSL, 1.57079632679490, -1.57079632679490, 0.200000000000000, 0, 4.71238898038469, 2.20000000000000, 4.71238898038469, 11.6247779607694),
	Param(LSR, 1.57079632679490, -1.57079632679490, 0.200000000000000, 1, 0, 0, 0, 0),
	Param(RSL, 1.57079632679490, -1.57079632679490, 0.200000000000000, 1, 0, 0, 0, 0),
	Param(RSR, 1.57079632679490, -1.57079632679490, 0.200000000000000, 0, 4.71238898038469, 1.80000000000000, 4.71238898038469, 11.2247779607694),
	Param(RLR, 1.57079632679490, -1.57079632679490, 0.200000000000000, 0, 1.10403098774760, 5.34965462908499, 1.10403098774760, 7.55771660458019),
	Param(LRL, 1.57079632679490, -1.57079632679490, 0.200000000000000, 0, 0.988432088926153, 5.11845683144210, 0.988432088926153, 7.09532100929441),
	Param(LSL, 1.57079632679490, -1.57079632679490, 0.600000000000000, 0, 4.71238898038469, 2.60000000000000, 4.71238898038469, 12.0247779607694),
	Param(LSR, 1.57079632679490, -1.57079632679490, 0.600000000000000, 1, 0, 0, 0, 0),
	Param(RSL, 1.57079632679490, -1.57079632679490, 0.600000000000000, 1, 0, 0, 0, 0),
	Param(RSR, 1.57079632679490, -1.57079632679490, 0.600000000000000, 0, 4.71238898038469, 1.40000000000000, 4.71238898038469, 10.8247779607694),
	Param(RLR, 1.57079632679490, -1.57079632679490, 0.600000000000000, 0, 1.21322522314939, 5.56804309988857, 1.21322522314939, 7.99449354618734),
	Param(LRL, 1.57079632679490, -1.57079632679490, 0.600000000000000, 0, 0.863211890069541, 4.86801643372887, 0.863211890069541, 6.59444021386796),
	Param(LSL, 1.57079632679490, -1.57079632679490, 1.20000000000000, 0, 4.71238898038469, 3.20000000000000, 4.71238898038469, 12.6247779607694),
	Param(LSR, 1.57079632679490, -1.57079632679490, 1.20000000000000, 1, 0, 0, 0, 0),
	Param(RSL, 1.57079632679490, -1.57079632679490, 1.20000000000000, 1, 0, 0, 0, 0),
	Param(RSR, 1.57079632679490, -1.57079632679490, 1.20000000000000, 0, 4.71238898038469, 0.800000000000000, 4.71238898038469, 10.2247779607694),
	Param(RLR, 1.57079632679490, -1.57079632679490, 1.20000000000000, 0, 1.36943840600457, 5.88046946559892, 1.36943840600457, 8.61934627760806),
	Param(LRL, 1.57079632679490, -1.57079632679490, 1.20000000000000, 0, 0.643501108793284, 4.42859487117636, 0.643501108793284, 5.71559708876293),
	Param(LSL, 1.57079632679490, -1.57079632679490, 10.0000000000000, 0, 4.71238898038469, 12.0000000000000, 4.71238898038469, 21.4247779607694),
	Param(LSR, 1.57079632679490, -1.57079632679490, 10.0000000000000, 0, 4.91374690117502, 9.79795897113271, 1.77215424758523, 16.4838601198930),
	Param(RSL, 1.57079632679490, -1.57079632679490, 10.0000000000000, 0, 1.77215424758523, 9.79795897113271, 4.91374690117502, 16.4838601198930),
	Param(RSR, 1.57079632679490, -1.57079632679490, 10.0000000000000, 0, 1.57079632679490, 8.00000000000000, 1.57079632679490, 11.1415926535898),
	Param(RLR, 1.57079632679490, -1.57079632679490, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LRL, 1.57079632679490, -1.57079632679490, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LSL, 1.57079632679490, 0, 0.200000000000000, 0, 5.40712725658139, 1.56204993518133, 5.58844703098288, 12.5576242227456),
	Param(LSR, 1.57079632679490, 0, 0.200000000000000, 1, 0, 0, 0, 0),
	Param(RSL, 1.57079632679490, 0, 0.200000000000000, 1, 0, 0, 0, 0),
	Param(RSR, 1.57079632679490, 0, 0.200000000000000, 0, 3.81633359581335, 1.28062484748657, 4.03764803816114, 9.13460648146105),
	Param(RLR, 1.57079632679490, 0, 0.200000000000000, 0, 0.348846568576992, 5.63139655988647, 0.570161010924784, 6.55040413938824),
	Param(LRL, 1.57079632679490, 0, 0.200000000000000, 0, 1.86434638999434, 5.48080888118507, 2.04566616439583, 9.39082143557524),
	Param(LSL, 1.57079632679490, 0, 0.600000000000000, 0, 5.27098829572825, 1.88679622641132, 5.72458599183602, 12.8823705139756),
	Param(LSR, 1.57079632679490, 0, 0.600000000000000, 1, 0, 0, 0, 0),
	Param(RSL, 1.57079632679490, 0, 0.600000000000000, 1, 0, 0, 0, 0),
	Param(RSR, 1.57079632679490, 0, 0.600000000000000, 0, 3.52209903070216, 1.07703296142690, 4.33188260327232, 8.93101459540138),
	Param(RLR, 1.57079632679490, 0, 0.600000000000000, 0, 0.107883633503051, 5.73793981996096, 0.917667206073218, 6.76349065953723),
	Param(LRL, 1.57079632679490, 0, 0.600000000000000, 0, 1.63817896180931, 5.30075194652129, 2.09177665791708, 9.03070756624769),
	Param(LSL, 1.57079632679490, 0, 1.20000000000000, 0, 5.13901647351157, 2.41660919471891, 5.85655781405271, 13.4121834822832),
	Param(LSR, 1.57079632679490, 0, 1.20000000000000, 0, 5.26059043786122, 1.35646599662505, 0.548201457476526, 7.16525789196279),
	Param(RSL, 1.57079632679490, 0, 1.20000000000000, 1, 0, 0, 0, 0),
	Param(RSR, 1.57079632679490, 0, 1.20000000000000, 0, 2.94419709373991, 1.01980390271856, 4.90978454023457, 8.87378553669304),
	Param(RLR, 1.57079632679490, 0, 1.20000000000000, 0, 5.82799274420666, 5.76759130093350, 1.51039488352173, 13.1059789286619),
	Param(LRL, 1.57079632679490, 0, 1.20000000000000, 0, 1.34872217225806, 4.98578201185216, 2.06626351279921, 8.40076769690943),
	Param(LSL, 1.57079632679490, 0, 10.0000000000000, 0, 4.80304886758544, 11.0453610171873, 6.19252541997884, 22.0409353047515),
	Param(LSR, 1.57079632679490, 0, 10.0000000000000, 0, 4.80380493701101, 10.8627804912002, 0.0914159566263198, 15.7580013848375),
	Param(RSL, 1.57079632679490, 0, 10.0000000000000, 0, 1.68283839261674, 8.83176086632785, 0.112042065821841, 10.6266413247664),
	Param(RSR, 1.57079632679490, 0, 10.0000000000000, 0, 1.68145354796879, 9.05538513813742, 6.17252808600569, 16.9093667721119),
	Param(RLR, 1.57079632679490, 0, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LRL, 1.57079632679490, 0, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LSL, 1.57079632679490, 1.57079632679490, 0.200000000000000, 0, 4.71238898038469, 0.200000000000000, 1.57079632679490, 6.48318530717959),
	Param(LSR, 1.57079632679490, 1.57079632679490, 0.200000000000000, 0, 5.85348564102816, 0.916515138991168, 5.85348564102816, 12.6234864210475),
	Param(RSL, 1.57079632679490, 1.57079632679490, 0.200000000000000, 1, 0, 0, 0, 0),
	Param(RSR, 1.57079632679490, 1.57079632679490, 0.200000000000000, 0, 1.57079632679490, 0.200000000000000, 4.71238898038469, 6.48318530717959),
	Param(RLR, 1.57079632679490, 1.57079632679490, 0.200000000000000, 0, 4.66236812357892, 6.18314359356805, 1.52077546998913, 12.3662871871361),
	Param(LRL, 1.57079632679490, 1.57079632679490, 0.200000000000000, 0, 1.52077546998913, 6.18314359356805, 4.66236812357892, 12.3662871871361),
	Param(LSL, 1.57079632679490, 1.57079632679490, 0.600000000000000, 0, 4.71238898038469, 0.600000000000000, 1.57079632679490, 6.88318530717959),
	Param(LSR, 1.57079632679490, 1.57079632679490, 0.600000000000000, 0, 5.59002539960283, 1.66132477258361, 5.59002539960283, 12.8413755717893),
	Param(RSL, 1.57079632679490, 1.57079632679490, 0.600000000000000, 1, 0, 0, 0, 0),
	Param(RSR, 1.57079632679490, 1.57079632679490, 0.600000000000000, 0, 1.57079632679490, 0.600000000000000, 4.71238898038469, 6.88318530717959),
	Param(RLR, 1.57079632679490, 1.57079632679490, 0.600000000000000, 0, 4.56182070760800, 5.98204876162621, 1.42022805401821, 11.9640975232524),
	Param(LRL, 1.57079632679490, 1.57079632679490, 0.600000000000000, 0, 1.42022805401821, 5.98204876162621, 4.56182070760800, 11.9640975232524),
	Param(LSL, 1.57079632679490, 1.57079632679490, 1.20000000000000, 0, 4.71238898038469, 1.20000000000000, 1.57079632679490, 7.48318530717959),
	Param(LSR, 1.57079632679490, 1.57079632679490, 1.20000000000000, 0, 5.38752051332172, 2.49799919935936, 5.38752051332172, 13.2730402260028),
	Param(RSL, 1.57079632679490, 1.57079632679490, 1.20000000000000, 1, 0, 0, 0, 0),
	Param(RSR, 1.57079632679490, 1.57079632679490, 1.20000000000000, 0, 1.57079632679490, 1.20000000000000, 4.71238898038469, 7.48318530717959),
	Param(RLR, 1.57079632679490, 1.57079632679490, 1.20000000000000, 0, 4.40769632636929, 5.67379999914879, 1.26610367277950, 11.3475999982976),
	Param(LRL, 1.57079632679490, 1.57079632679490, 1.20000000000000, 0, 1.26610367277950, 5.67379999914879, 4.40769632636929, 11.3475999982976),
	Param(LSL, 1.57079632679490, 1.57079632679490, 10.0000000000000, 0, 4.71238898038469, 10.0000000000000, 1.57079632679490, 16.2831853071796),
	Param(LSR, 1.57079632679490, 1.57079632679490, 10.0000000000000, 0, 4.87983705960438, 11.8321595661992, 4.87983705960438, 21.5918336854080),
	Param(RSL, 1.57079632679490, 1.57079632679490, 10.0000000000000, 0, 1.82347658193698, 7.74596669241483, 1.82347658193698, 11.3929198562888),
	Param(RSR, 1.57079632679490, 1.57079632679490, 10.0000000000000, 0, 1.57079632679490, 10.0000000000000, 4.71238898038469, 16.2831853071796),
	Param(RLR, 1.57079632679490, 1.57079632679490, 10.0000000000000, 1, 0, 0, 0, 0),
	Param(LRL, 1.57079632679490, 1.57079632679490, 10.0000000000000, 1, 0, 0, 0, 0),
};


class MonteCarloTests : public ::testing::TestWithParam<Param>
{
public:
    void SetUp()
    {
        turning_radius = 1.0;
        q0[0] = 0.0;
        q0[1] = 0.0;
        q0[2] = GetParam().inputs.a;
        q1[0] = GetParam().inputs.d;
        q1[1] = 0.0;
        q1[2] = GetParam().inputs.b;
    }

protected:
    double turning_radius;
    double q0[3];
    double q1[3];
};

TEST_P(MonteCarloTests, Simple)
{
    DubinsPath path;
    int code = dubins_path(&path, q0, q1, turning_radius, GetParam().inputs.word);
    if(code != 0) {
        code = 1;
    }
    ASSERT_EQ(code, GetParam().outputs.errcode);

    if(code == 0) 
    {
       for(int i = 0; i < 3; i++) {
            ASSERT_NEAR(path.param[i], GetParam().outputs.params[i], 1e-8);
        }
        double len = dubins_path_length(&path);
        ASSERT_NEAR(len, GetParam().outputs.length, 1e-8);

    }
}

INSTANTIATE_TEST_CASE_P(Simple,
                        MonteCarloTests,
                        ::testing::ValuesIn(params));

