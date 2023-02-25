#include <stdio.h>

// const double atom_line = 228.0 * 88.0 / 315.0;  // 63.695us

const double atom_line = 63.695;

void main() {

    double fref = 12.0;

    double best_absppm = 1e6;
    double best_ppm = 0;
    double best_clock = 0;
    int    best_clkdiv = 0;
   
    for (int refdiv = 1; refdiv <= 63; refdiv++) {

        double ref = fref / (double) refdiv;

        // Min frequency for the PLL reference input is 5MHz.
        // With a 12MHz crystal, only refdiv 1 and 2 meet this.
        if (ref < 5.0) {
            continue;
        }

        for (int fbdiv = 16; fbdiv <= 320; fbdiv++) {
            
            double fvco = ref * (double) fbdiv;

            // VCO output freqency must be in the range 750-1600 MHz
            if (fvco < 750.0 || fvco > 1600.0) {
                continue;
            }

            // Try all unique combinations of divider, keeping pd1 >= pd2
            // There will be some duplation of total divider value here.
            // e.g. 6 * 1 == 3 * 2
            //      4 * 1 == 2 * 2
            // So exclude these cases
            for (int pd2 = 1; pd2 <= 7; pd2++) {
                for (int pd1 = pd2; pd1 <= 7; pd1++) {
                    if (pd2 == 2 && (pd1 == 2 || pd1 == 3)) {
                        continue;
                    }

                    double clock = fvco / (double) pd1 / (double) pd2;


                    if (clock >= 190.0 && clock <= 310.0) {

                        // Now find the best PIO CLKDIV setting for the closest pixel clock
                        double linelen = 800.0;

                        // The first *2 is because there are two VGA lines for each Atom line 
                        // The second *2 is because the scanvideo needs a factor of 2
                        double perfect_div = 256 * (clock * atom_line) / (linelen * 4.0);

                        int clkdiv_lower = (int) perfect_div;

                        for (int i = 0; i < 2; i++) {
                            int clkdiv = clkdiv_lower + i;

                            double vga_line = 4.0 * linelen * (double) clkdiv / 256.0 / clock;

                            double ppm = 1000000.0 * (vga_line - atom_line) / (atom_line);
                            double absppm = ppm;
                            if (absppm < 0) {
                                absppm = -absppm;
                            }

                            if (absppm < best_absppm) {
                                best_absppm = absppm;
                                best_ppm = ppm;
                                best_clock = clock;
                                best_clkdiv = clkdiv;
                            }

                            if (absppm < 100.0) {
                                printf("%3.6f (refdiv = %d; fbdiv = %3d, vco = %4d pd1 = %d, pd2 = %d, clkdiv = 0x%x (%4d), ppm = %6.2f\n", clock, refdiv, fbdiv, (int) fvco, pd1, pd2, clkdiv, clkdiv, ppm);
                            }
                        }
                    }
                }
            }
        }
    }

    printf("best ppm = %6.2f at clock %3.6f, clkdiv %d\n", best_ppm, best_clock, best_clkdiv);
}           
