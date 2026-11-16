#include <stdio.h>
#include <string.h>
#include <ctype.h>

typedef struct {
    const char* prefix;
    const char* continent; // Using standard 2-letter ADIF continent codes
} PrefixMap;

// Representative lookup table (ordered generally, searched by exact prefix match)
// Standard ADIF codes: NA, SA, EU, AF, AS, OC, AN
static const PrefixMap prefix_table[] = {
    // --- 3-Character Specific Exceptions ---
    {"3DA", "AF"}, {"3D2", "OC"}, {"3V", "AF"},  {"3W", "AS"},  {"3X", "AF"},
    {"4S", "AS"},  {"4U1", "EU"}, {"4X", "AS"},  {"4Z", "AS"},  {"5A", "AF"},
    {"5B", "AS"},  {"5H", "AF"},  {"5N", "AF"},  {"5R", "AF"},  {"5T", "AF"},
    {"5U", "AF"},  {"5V", "AF"},  {"5W", "OC"},  {"5X", "AF"},  {"5Z", "AF"},
    {"6W", "AF"},  {"6Y", "NA"},  {"7O", "AS"},  {"7P", "AF"},  {"7Q", "AF"},
    {"7X", "AF"},  {"8P", "NA"},  {"8Q", "AS"},  {"8R", "SA"},  {"9A", "EU"},
    {"9G", "AF"},  {"9H", "EU"},  {"9J", "AF"},  {"9K", "AS"},  {"9L", "AF"},
    {"9M2", "AS"}, {"9M6", "OC"}, {"9N", "AS"},  {"9Q", "AF"},  {"9U", "AF"},
    {"9V", "AS"},  {"9X", "AF"},  {"9Y", "SA"},

    // --- 2-Character Allocations ---
    {"AA", "NA"}, {"AB", "NA"}, {"AC", "NA"}, {"AD", "NA"}, {"AE", "NA"},
    {"AF", "NA"}, {"AG", "NA"}, {"AH", "OC"}, {"AI", "NA"}, {"AJ", "NA"},
    {"AK", "NA"}, {"AL", "NA"}, // AL = Alaska (NA), AH = US Pacific (OC)
    {"AP", "AS"}, {"AS", "AS"}, {"AT", "AS"}, {"AU", "AS"}, {"AV", "AS"}, {"AW", "AS"},
    {"AX", "OC"}, {"AY", "SA"}, {"AZ", "SA"},
    {"BA", "AS"}, {"BD", "AS"}, {"BG", "AS"}, {"BH", "AS"}, {"BI", "AS"}, {"BL", "AS"}, {"BM", "AS"}, {"BO", "AS"}, {"BP", "AS"}, {"BQ", "AS"}, {"BU", "AS"}, {"BV", "AS"}, {"BW", "AS"}, {"BX", "AS"}, {"BY", "AS"}, {"BZ", "AS"},
    {"C2", "OC"}, {"C3", "EU"}, {"C5", "AF"}, {"C6", "NA"}, {"C9", "AF"},
    {"CA", "SA"}, {"CB", "SA"}, {"CC", "SA"}, {"CD", "SA"}, {"CE", "SA"},
    {"CF", "NA"}, {"CG", "NA"}, {"CH", "NA"}, {"CI", "NA"}, {"CJ", "NA"}, {"CK", "NA"},
    {"CL", "NA"}, {"CM", "NA"}, {"CN", "AF"}, {"CO", "NA"}, {"CP", "SA"},
    {"CQ", "EU"}, {"CR", "EU"}, {"CS", "EU"}, {"CT", "EU"}, {"CU", "EU"},
    {"CV", "SA"}, {"CW", "SA"}, {"CX", "SA"}, {"CY", "NA"}, {"CZ", "NA"},
    {"D2", "AF"}, {"D3", "AF"}, {"D4", "AF"}, {"D6", "AF"},
    {"DA", "EU"}, {"DB", "EU"}, {"DC", "EU"}, {"DD", "EU"}, {"DE", "EU"}, {"DF", "EU"}, {"DG", "EU"}, {"DH", "EU"}, {"DI", "EU"}, {"DJ", "EU"}, {"DK", "EU"}, {"DL", "EU"}, {"DM", "EU"}, {"DN", "EU"}, {"DO", "EU"}, {"DP", "EU"}, {"DQ", "EU"}, {"DR", "EU"},
    {"DS", "AS"}, {"DT", "AS"}, {"DU", "OC"}, {"DV", "OC"}, {"DW", "OC"}, {"DX", "OC"}, {"DY", "OC"}, {"DZ", "OC"},
    {"E2", "AS"}, {"E3", "AF"}, {"E4", "AS"}, {"E5", "OC"}, {"E6", "OC"}, {"E7", "EU"},
    {"EA", "EU"}, {"EB", "EU"}, {"EC", "EU"}, {"ED", "EU"}, {"EE", "EU"}, {"EF", "EU"}, {"EG", "EU"}, {"EH", "EU"},
    {"EI", "EU"}, {"EJ", "EU"}, {"EK", "AS"}, {"EL", "AF"}, {"EM", "EU"}, {"EN", "EU"}, {"EO", "EU"},
    {"EP", "AS"}, {"EQ", "AS"}, {"ER", "EU"}, {"ES", "EU"}, {"ET", "AF"}, {"EU", "EU"}, {"EV", "EU"}, {"EW", "EU"},
    {"EX", "AS"}, {"EY", "AS"}, {"EZ", "AS"},
    {"FG", "NA"}, {"FH", "AF"}, {"FJ", "NA"}, {"FK", "OC"}, {"FM", "AF"}, {"FO", "OC"}, {"FP", "NA"}, {"FR", "AF"}, {"FS", "NA"}, {"FT", "AF"}, {"FW", "OC"}, {"FY", "SA"},
    {"GD", "EU"}, {"GI", "EU"}, {"GJ", "EU"}, {"GM", "EU"}, {"GU", "EU"}, {"GW", "EU"},
    {"HA", "EU"}, {"HB", "EU"}, {"HC", "SA"}, {"HD", "SA"}, {"HE", "EU"}, {"HF", "EU"}, {"HG", "EU"}, {"HH", "NA"}, {"HI", "NA"}, {"HJ", "SA"}, {"HK", "SA"}, {"HL", "AS"}, {"HM", "AS"}, {"HN", "AS"}, {"HO", "NA"}, {"HP", "NA"}, {"HQ", "NA"}, {"HR", "NA"}, {"HS", "AS"}, {"HT", "NA"}, {"HU", "NA"}, {"HV", "EU"}, {"HW", "EU"}, {"HX", "EU"}, {"HY", "EU"}, {"HZ", "AS"},
    {"IS", "EU"}, {"IT", "EU"},
    {"J2", "AF"}, {"J3", "NA"}, {"J5", "AF"}, {"J6", "NA"}, {"J7", "NA"}, {"J8", "NA"},
    {"JA", "AS"}, {"JE", "AS"}, {"JF", "AS"}, {"JG", "AS"}, {"JH", "AS"}, {"JI", "AS"}, {"JJ", "AS"}, {"JK", "AS"}, {"JL", "AS"}, {"JM", "AS"}, {"JN", "AS"}, {"JO", "AS"}, {"JP", "AS"}, {"JQ", "AS"}, {"JR", "AS"}, {"JS", "AS"},
    {"JT", "AS"}, {"JU", "AS"}, {"JV", "AS"}, {"JW", "EU"}, {"JX", "EU"}, {"JY", "AS"}, {"JZ", "OC"},
    {"KC4", "AN"}, {"KH", "OC"}, {"KL", "NA"}, {"KP", "NA"},
    {"LA", "EU"}, {"LB", "EU"}, {"LC", "EU"}, {"LD", "EU"}, {"LE", "EU"}, {"LF", "EU"}, {"LG", "EU"}, {"LH", "EU"}, {"LI", "EU"}, {"LJ", "EU"}, {"LK", "EU"}, {"LL", "EU"}, {"LM", "EU"}, {"LN", "EU"},
    {"LO", "SA"}, {"LP", "SA"}, {"LQ", "SA"}, {"LR", "SA"}, {"LS", "SA"}, {"LT", "SA"}, {"LU", "SA"}, {"LV", "SA"}, {"LW", "SA"},
    {"LX", "EU"}, {"LY", "EU"}, {"LZ", "EU"},
    {"OA", "SA"}, {"OB", "SA"}, {"OC", "SA"}, {"OD", "AS"}, {"OE", "EU"}, {"OF", "EU"}, {"OG", "EU"}, {"OH", "EU"}, {"OI", "EU"}, {"OJ", "EU"}, {"OK", "EU"}, {"OL", "EU"}, {"OM", "EU"}, {"ON", "EU"}, {"OO", "EU"}, {"OP", "EU"}, {"OQ", "EU"}, {"OR", "EU"}, {"OS", "EU"}, {"OT", "EU"}, {"OU", "EU"}, {"OV", "EU"}, {"OW", "EU"}, {"OX", "NA"}, {"OY", "EU"}, {"OZ", "EU"},
    {"PA", "EU"}, {"PB", "EU"}, {"PC", "EU"}, {"PD", "EU"}, {"PE", "EU"}, {"PF", "EU"}, {"PG", "EU"}, {"PH", "EU"}, {"PI", "EU"},
    {"PJ", "SA"}, {"PK", "OC"}, {"PL", "OC"}, {"PM", "OC"}, {"PN", "OC"}, {"PO", "OC"},
    {"PP", "SA"}, {"PQ", "SA"}, {"PR", "SA"}, {"PS", "SA"}, {"PT", "SA"}, {"PU", "SA"}, {"PV", "SA"}, {"PW", "SA"}, {"PX", "SA"}, {"PY", "SA"}, {"PZ", "SA"},
    {"RA", "EU"}, {"RC", "EU"}, {"RD", "EU"}, {"RE", "EU"}, {"RF", "EU"}, {"RG", "EU"}, {"RI", "EU"}, {"RJ", "EU"}, {"RK", "EU"}, {"RL", "EU"}, {"RM", "EU"}, {"RN", "EU"}, {"RO", "EU"}, {"RP", "EU"}, {"RQ", "EU"}, {"RR", "EU"}, {"RS", "EU"}, {"RT", "EU"}, {"RU", "EU"}, {"RV", "EU"}, {"RW", "EU"}, {"RX", "EU"}, {"RY", "EU"}, {"RZ", "EU"},
    {"RA8", "AS"}, {"RA9", "AS"}, {"RA0", "AS"}, // Asiatic Russia exceptions
    {"S2", "AS"}, {"S5", "EU"}, {"S7", "AF"}, {"S9", "AF"},
    {"SA", "EU"}, {"SB", "EU"}, {"SC", "EU"}, {"SD", "EU"}, {"SE", "EU"}, {"SF", "EU"}, {"SG", "EU"}, {"SH", "EU"}, {"SI", "EU"}, {"SJ", "EU"}, {"SK", "EU"}, {"SL", "EU"}, {"SM", "EU"},
    {"SN", "EU"}, {"SO", "EU"}, {"SP", "EU"}, {"SQ", "EU"}, {"SR", "EU"},
    {"ST", "AF"}, {"SU", "AF"}, {"SV", "EU"}, {"SW", "EU"}, {"SX", "EU"}, {"SY", "EU"}, {"SZ", "EU"},
    {"T2", "OC"}, {"T3", "OC"}, {"T5", "AF"}, {"T7", "EU"}, {"T8", "OC"},
    {"TA", "AS"}, {"TB", "AS"}, {"TC", "AS"}, {"TD", "NA"}, {"TE", "NA"}, {"TF", "EU"}, {"TG", "NA"}, {"TH", "EU"}, {"TI", "NA"}, {"TJ", "AF"}, {"TK", "EU"}, {"TL", "AF"}, {"TN", "AF"}, {"TP", "EU"}, {"TQ", "EU"}, {"TR", "AF"}, {"TS", "AF"}, {"TT", "AF"}, {"TU", "AF"}, {"TV", "EU"}, {"TW", "EU"}, {"TX", "OC"}, {"TY", "AF"}, {"TZ", "AF"},
    {"UA", "EU"}, {"UB", "EU"}, {"UC", "EU"}, {"UD", "EU"}, {"UE", "EU"}, {"UF", "EU"}, {"UG", "EU"}, {"UH", "EU"}, {"UI", "EU"},
    {"UA8", "AS"}, {"UA9", "AS"}, {"UA0", "AS"}, // Asiatic Russia exceptions
    {"UJ", "AS"}, {"UK", "AS"}, {"UL", "AS"}, {"UM", "AS"}, {"UN", "AS"}, {"UO", "AS"}, {"UP", "AS"}, {"UQ", "AS"},
    {"UR", "EU"}, {"US", "EU"}, {"UT", "EU"}, {"UU", "EU"}, {"UV", "EU"}, {"UW", "EU"}, {"UX", "EU"}, {"UY", "EU"}, {"UZ", "EU"},
    {"V2", "NA"}, {"V3", "NA"}, {"V4", "NA"}, {"V5", "AF"}, {"V6", "OC"}, {"V7", "OC"}, {"V8", "OC"},
    {"VA", "NA"}, {"VB", "NA"}, {"VC", "NA"}, {"VD", "NA"}, {"VE", "NA"}, {"VF", "NA"}, {"VG", "NA"},
    {"VH", "OC"}, {"VI", "OC"}, {"VJ", "OC"}, {"VK", "OC"}, {"VL", "OC"}, {"VM", "OC"}, {"VN", "OC"},
    {"VO", "NA"}, {"VP2", "NA"}, {"VP5", "NA"}, {"VP8", "SA"}, {"VP9", "NA"},
    {"VQ9", "AF"}, {"VR", "AS"}, {"VS", "AS"}, {"VT", "AS"}, {"VU", "AS"}, {"VV", "AS"}, {"VW", "AS"},
    {"VX", "NA"}, {"VY", "NA"}, {"VZ", "OC"},
    {"XA", "NA"}, {"XB", "NA"}, {"XC", "NA"}, {"XD", "NA"}, {"XE", "NA"}, {"XF", "NA"}, {"XG", "NA"}, {"XH", "NA"}, {"XI", "NA"},
    {"XJ", "NA"}, {"XK", "NA"}, {"XL", "NA"}, {"XM", "NA"}, {"XN", "NA"}, {"XO", "NA"},
    {"XP", "NA"}, {"XQ", "SA"}, {"XR", "SA"}, {"XS", "AS"}, {"XT", "AF"}, {"XU", "AS"}, {"XV", "AS"}, {"XW", "AS"}, {"XX9", "AS"}, {"XY", "AS"}, {"XZ", "AS"},
    {"YA", "AS"}, {"YB", "OC"}, {"YC", "OC"}, {"YD", "OC"}, {"YE", "OC"}, {"YF", "OC"}, {"YG", "OC"}, {"YH", "OC"},
    {"YI", "AS"}, {"YJ", "OC"}, {"YK", "AS"}, {"YL", "EU"}, {"YM", "AS"}, {"YN", "NA"}, {"YO", "EU"}, {"YP", "EU"}, {"YQ", "EU"}, {"YR", "EU"}, {"YS", "NA"}, {"YT", "EU"}, {"YU", "EU"}, {"YV", "SA"}, {"YW", "SA"}, {"YX", "SA"}, {"YY", "SA"},
    {"Z2", "AF"}, {"Z3", "EU"}, {"Z6", "EU"}, {"Z8", "AF"},
    {"ZA", "EU"}, {"ZB", "EU"}, {"ZC4", "AS"}, {"ZD7", "AF"}, {"ZD8", "AF"}, {"ZD9", "AF"},
    {"ZF", "NA"}, {"ZK3", "OC"}, {"ZL", "OC"}, {"ZM", "OC"}, {"ZP", "SA"}, {"ZR", "AF"}, {"ZS", "AF"}, {"ZT", "AF"}, {"ZU", "AF"},

    // --- 1-Character Single-Letter Allocations ---
    {"B", "AS"}, {"F", "EU"}, {"G", "EU"}, {"I", "EU"},
    {"K", "NA"}, {"M", "EU"}, {"N", "NA"}, {"R", "EU"}, {"W", "NA"}
};

const char* get_continent_from_callsign(const char* callsign) {

    if (callsign == NULL || callsign[0] == '\0') {
        return "Invalid";
    }

    char clean[16] = {0};
    int j = 0;

    // Normalize to uppercase and stop at any portable slash (e.g., W1AW/P -> W1AW)
    // Note: If a prefix precedes a slash (e.g., VE3/W1AW), real parsers take the part before '/'
    for (int i = 0; callsign[i] != '\0' && j < 15; i++) {
        if (callsign[i] == '/') {
            // If slash appears early (e.g., "KH6/W1AW"), reset to evaluate the prefix before slash
            if (i <= 4) {
                break; 
            }
            break;
        }
        if (isalnum((unsigned char)callsign[i])) {
            clean[j++] = toupper((unsigned char)callsign[i]);
        }
    }
    clean[j] = '\0';

    int len = strlen(clean);
    if (len < 2) return "Unknown";

    int table_size = sizeof(prefix_table) / sizeof(prefix_table[0]);

    // Longest-prefix match: check 3 chars, then 2 chars, then 1 char
    for (int check_len = (len >= 3 ? 3 : len); check_len >= 1; check_len--) {
        for (int i = 0; i < table_size; i++) {
            if ((int)strlen(prefix_table[i].prefix) == check_len &&
		strncmp(clean, prefix_table[i].prefix, check_len) == 0) {
	      return prefix_table[i].continent;
	    }
	}
    }
    return "Unknown";
}
