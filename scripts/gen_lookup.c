// gen_lookup.c
// Usage: ./gen_lookup Hip_mag3.csv > hipparcos_lookup.csv

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_STARS 512
#define CV_PI   3.1415926535897932384626433832795
#define DEG2RAD (CV_PI / 180.0)
#define RAD2DEG (180.0 / CV_PI)

typedef struct {
    char   id[32];
    double ra_deg;
    double dec_deg;
    double x, y, z;   // unit vector
} Star;

static Star stars[MAX_STARS];
static int  star_count = 0;

static void to_unit_vec(Star *s) {
    double ra  = s->ra_deg  * DEG2RAD;
    double dec = s->dec_deg * DEG2RAD;
    s->x = cos(dec) * cos(ra);
    s->y = cos(dec) * sin(ra);
    s->z = sin(dec);
}

static double angular_sep_deg(const Star *a, const Star *b) {
    double dot = a->x*b->x + a->y*b->y + a->z*b->z;
    // clamp to avoid NaN from floating-point overshoot
    if (dot >  1.0) dot =  1.0;
    if (dot < -1.0) dot = -1.0;
    return acos(dot) * RAD2DEG;
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <Hip_mag3.csv>\n", argv[0]);
        return 1;
    }

    FILE *fp = fopen(argv[1], "r");
    if (!fp) { perror("fopen"); return 1; }

    char line[256];
    // skip header
    if (!fgets(line, sizeof(line), fp)) { fclose(fp); return 1; }

    while (fgets(line, sizeof(line), fp)) {
        if (star_count >= MAX_STARS) {
            fprintf(stderr, "Warning: MAX_STARS (%d) exceeded, increase limit\n", MAX_STARS);
            break;
        }
        Star *s = &stars[star_count];
        double mag;
        if (sscanf(line, "%31[^,],%lf,%lf,%lf",
                   s->id, &s->ra_deg, &s->dec_deg, &mag) != 4) {
            continue; // skip malformed lines
        }
        to_unit_vec(s);
        star_count++;
    }
    fclose(fp);

    fprintf(stderr, "Loaded %d stars\n", star_count);

    // Write CSV header
    printf("star1,star2,ang_sep\n");

    // All unique pairs
    for (int i = 0; i < star_count - 1; i++) {
        for (int j = i + 1; j < star_count; j++) {
            double sep = angular_sep_deg(&stars[i], &stars[j]);
            printf("%s,%s,%.6f\n", stars[i].id, stars[j].id, sep);
        }
    }

    return 0;
}